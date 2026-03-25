## Estimate Drag and Momentum

To build the state space model for the Kalman Filter, the robot step response was recorded. It was driven toward a wall using a constant motor pwm while recording the front TOF sensor. 

The PWM used for this test was chosen to be 120. A foam barrier was placed at the wall to avoid damage.

On the Artemis, STEP_RESPONSE_RUN command was added that starts the run, records data, and stops the robot after its done. During the run, the robot stores TOF data and later sends it to laptop.

```cpp
void start_step_response_run(int pwm, int duration_ms)
{    
    step_test_pwm = pwm;
    step_test_duration_ms = duration_ms;

    recording = false;
    record_done = false;
    imu_len = 0;
    tof1_len = 0;
    tof2_len = 0;

    recording = true;
    record_start_us = micros();

    driveForward(step_test_pwm);
    step_test_end_ms = millis() + step_test_duration_ms;
    step_test_active = true;
}
```

A safety stop was included so the robot would stop if it got too close to the wall.

```cpp
if (step_test_active && last_dist_mm > 0 && last_dist_mm < 200) {
    coastStop();
    step_test_active = false;
    recording = false;
    record_done = true;
}
```

<br>

On Python side, code was used to send step command, request data, and parse the returned samples. The recorded times were converted to seconds, and the speed was calculated from the distance data using the gradient:

```cpp
t = np.array(tof1_t)
d = np.array(tof1_mm)

t = t - t[0]
v = -np.gradient(d, t)
```

The plots are shown in Figure 1 below.

<p align="center">
  <img src="../img/lab7/step_dist.png" width="30%">
  <img src="../img/lab7/step_speed.png" width="30%">
  <img src="../img/lab7/step_pwm.png" width="30%">
</p>
<p align="center">
  <b>Figure 1:</b> Step Response Distance, Speed, PWM vs. Time.
</p>

To estimate the model parameters, the measured velocity was fit to an exponential model. Only the data during the active step input (t < 2 seconds) was used.

```cpp
mask = t_fit < 2.0
t_fit = t_fit[mask]
v_fit = v_fit[mask]

def model(t, v_ss, tau):
    return v_ss * (1 - np.exp(-t / tau))

popt, _ = curve_fit(model, t_fit, v_fit, p0=[np.max(v_fit), 0.5])

v_ss_fit, tau_fit = popt
```

The steady state velocity and time constant came from curve fitting. The 90 percent rise time was calculated as:

```cpp
t_90 = tau_fit * np.log(10)
```

The steady state velocity and time constant were then used to estimate d and m of the system. d and m were estimated using:

```cpp
d = u / v_ss_fit
m = d * tau_fit
```

The fitted curve was plotted with the measured velocity data in Figure 2.

<p align="center">
  <img src="../img/lab7/exp_fit.png" width="80%">
</p>
<p align="center">
  <b>Figure 2:</b> Exponential Fit of Velocity Data.
</p>

---

## Initialize KF (Python)

After estimating d and m from the step response, a 2 state model was built. The state vector was defined as position and velocity relative to the wall.

<p align="center">
  <img src="../img/lab7/state_space.png" width="80%">
</p>
<p align="center">
</p>

Using the fitted values, the A and B matrices were calculated in Python:

```cpp
A = np.array([
    [0.0,  1.0],
    [0.0, -(d / m)]
], dtype=float)

B = np.array([
    [0.0],
    [1.0 / m]
], dtype=float)

C = np.array([[-1.0, 0.0]], dtype=float)
```

The model was then discretized. A sampling time of Delta_T = 1/150 = 0.0067s was used, which is the PID control loop frequency measured in Lab 5. This allows the filter to run faster than the sensor and estimate the system state between measurements.

```cpp
Delta_T = 0.067

Ad = np.eye(2) + Delta_T * A
Bd = Delta_T * B
```

The initial state was set using the first TOF measurement. Position was initialized as the negative distance from the wall, and the initial velocity was set to zero.

The state covariance matrix was initialized next. A relatively large uncertainty in both position and velocity was used at the beginning, since the exact initial state is not known accurately.

```cpp
Sigma = np.array([
    [150.0**2, 0.0],
    [0.0, 400.0**2]
], dtype=float)
```

Next, process noise and measurement noise covariance matrices were chosen. Assuming the noise terms were uncorrelated, both matrices were diagonal. A measurement standard deviation of about 20 mm was chosen for TOF readings.

```cpp
sigma_3 = 20.0
Sigma_z = np.array([[sigma_3**2]], dtype=float)
```

The process noise covariance represents uncertainty in the system model. Since the robot does not exactly follow an ideal first order model, uncertainty was included in both position and velocity. Initial values of 30 mm and 30 mm/s were used.

```cpp
sigma_1 = 30.0
sigma_2 = 30.0

Sigma_u = np.array([
    [sigma_1**2, 0.0],
    [0.0, sigma_2**2]
], dtype=float)
```

During tuning, the measurement noise (sigma 3) had the most noticeable effect. Increasing sigma 3 resulted in smoother estimates by reducing the influence of noisy measurements, while decreasing sigma 3 caused the filter to closely follow the raw sensor data.

In contrast, changing the process noise terms (sigma 1 and sigma 2) had a smaller effect on the position estimate. This is likely because the system model was already a reasonable approximation and measurements were incorporated frequently.

These values determine how much the filter trusts the model vs. the measurements. Larger process noise increases reliance on measurements, while larger measurement noise increases reliance on the model.

---

## KF Implementation on Jupyter

Finally, the Kalman Filter was implemented, including the prediction step, Kalman gain computation, and measurement update.

The Kalman Filter was implemented such that prediction is performed at every control loop iteration, while measurement updates are only applied when a new TOF sample is available. This allows the model to estimate system behavior between sensor readings.

The code for Kalman Filter was separated into prediction and update functions:

```cpp
def kf_predict(mu, sigma, u):
    mu_p = Ad.dot(mu) + Bd.dot(u)
    sigma_p = Ad.dot(sigma.dot(Ad.transpose())) + Sigma_u

    return mu_p, sigma_p

def kf_update(mu_p, sigma_p, y):
    sigma_m = C.dot(sigma_p.dot(C.transpose())) + Sigma_z
    kkf_gain = sigma_p.dot(C.transpose().dot(np.linalg.inv(sigma_m)))
    y_m = np.array(y) - C.dot(mu_p)
    mu = mu_p + kkf_gain.dot(y_m)
    sigma = (np.eye(2) - kkf_gain.dot(C)).dot(sigma_p)

    return mu, sigma
```

The filter was run using a higher rate time vector that matches the control loop. At each step, it first predicts the next state using the model. When a new TOF measurement becomes available, the filter updates the estimate using that measurement.

```cpp
for tk in t_kf:
    u_now = u_step if tk < 2 else 0.0

    # prediction
    mu_p, Sigma_p = kf_predict(mu, Sigma, u_now)

    # update only when measurements are available
    while meas_idx < n_meas and t_meas[meas_idx] <= tk:
        y_now = z_meas[meas_idx]
        mu_p, Sigma_p, _ = kf_update(mu_p, Sigma_p, y_now)
        meas_idx += 1

    mu = mu_p
    Sigma = Sigma_p
```

The estimated states were saved and used to compute the distance and velocity. The results were compared with the raw TOF distance data. The Kalman Filter produced a smoother estimate, while still following the overall trend of the measurements.

Small jumps can be seen in the estimated trajectory whenever a new TOF measurement is used. This is expected, since the filter corrects its prediction when new data arrives.

<p align="center">
  <img src="../img/lab7/kf_dist_new.png" width="80%">
</p>
<p align="center">
  <b>Figure 3:</b> KF Distance vs. Raw TOF Distance.
</p>

---

## KF Implementation on the Robot

Kalman Filter was added to the distance PID controller on the Artemis. The linear extrapolation from the previous lab was removed and replaced with the Kalman Filter estimate. The filter runs every loop, and only updates with a measurement when a new TOF reading comes in.

The same model and noise values from Python were copied into the Artemis code. The state was stored as negative distance from the wall and velocity toward the wall. The covariance matrix, process noise, and measurement noise were also initialized using the same values.

At the start of a run, the filter was initialized using the new TOF reading, with the initial velocity set to zero.

```cpp
void kf_init(float dist_mm)
{
    kf_x = -dist_mm;
    kf_v = 0.0f;

    P00 = 22500.0f;
    P01 = 0.0f;
    P10 = 0.0f;
    P11 = 160000.0f;

    last_u_pwm_kf = 0.0f;
}
```

In each loop, the TOF reading was updated first, then the Kalman Filter prediction ran using the previous motor command. If a new TOF measurement was available, the correction step was applied. The estimated distance was then converted back to a positive value and used to calculate the PID error.

```cpp
int raw_dist = last_dist_mm;

kf_step(last_u_pwm_kf, new_tof_sample, (float)last_dist_mm);

int dist = (int)roundf(-kf_x);
int err = dist - setpoint_mm;
```

Inside the filter, the state was first predicted using the model. If a new TOF sample was available, the estimate was corrected using that measurement.

```cpp
void kf_step(float u_pwm, bool has_meas, float y_dist_mm)
    x_pred = A * x + B * u
    v_pred = A * v + B * u
    P_pred = A * P * A^T + Q

    if has_measurement:
        S = P_pred(0,0) + R
        K0 = -P_pred(0,0) / S
        K1 = -P_pred(1,0) / S
        innovation = y + x_pred

        x = x_pred + K0 * innovation
        v = v_pred + K1 * innovation
        P = (I - K*C) * P_pred

    else:
        x = x_pred
        v = v_pred
        P = P_pred
```

When the raw TOF data and the Kalman Filter estimate were plotted on the same graph, the estimated distance followed the same overall trend but was much smoother, making it better for control. This showed that the Kalman Filter was successfully added to the Artemis controller and replaced the linear extrapolation method from the previous lab.


<p align="center">
  <img src="../img/lab7/kf_dist_run.png" width="80%">
</p>
<p align="center">
  <b>Figure 4:</b> KF Distance vs. Raw TOF Distance.
</p>

<p align="center">
  <img src="../img/lab7/error_run.png" width="30%">
  <img src="../img/lab7/pwm_run.png" width="30%">
  <img src="../img/lab7/pid_run.png" width="30%">
</p>
<p align="center">
  <b>Figure 5:</b> KF Error, PWM, and PID.
</p>

Video 1 below shows the result of KF PID controller. Same PID gains (Lab 5) of Kp = 0.1, Ki = 0.001, and Kd = 0.001 were used.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/yNlylsxH1b8"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 1:</b> PID Controller with KF.
</p>

<br>

---

## Discussion

This lab provided experience implementing a Kalman Filter on the robot and integrating it with a closed-loop distance controller. Overall, it improved understanding of state estimation, tuning process and measurement noise, and how model-based prediction can be used alongside sensor data. Using the Kalman Filter showed how combining a system model with noisy TOF measurements can produce a smoother and more reliable estimate, which is especially useful when the sensor updates slowly compared to the control loop.

---

## Acknowledgment

I referenced [Trevor Dales](https://trevordales.github.io/MAE4190/lab7/)’s pages from last year.

Parts of this report and website formatting were assisted by AI tools (ChatGPT) for grammar checking and webpage structuring. All code was written, tested, and validated by the author.
