## Estimate Drag and Momentum

To build the state space model for the Kalman Filter, the robot step response was measured. The robot was driven toward a wall using a constant motor input while logging the front TOF sensor and the motor command. 

The PWM used for this test was chosen to be 120. A foam barrier was placed at the wall to avoid damaging the robot.

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

On Python side, code was used to send step command, request data, and parse the returned samples. The recorded times were converted to seconds, and the speed was computed from the distance data using the gradient:

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

To estimate the model parameters, I fit the measured velocity to a first order exponential model. Only the data during the active step input (t < 2 seconds) was used for fitting.

```cpp
mask = t_fit < 2.0
t_fit = t_fit[mask]
v_fit = v_fit[mask]

def model(t, v_ss, tau):
    return v_ss * (1 - np.exp(-t / tau))

popt, _ = curve_fit(model, t_fit, v_fit, p0=[np.max(v_fit), 0.5])

v_ss_fit, tau_fit = popt
```

From the fitted time constant, the 90 percent rise time was computed as:

t_90 = tau * ln(10)

The steady state velocity and time constant were then used to estimate d and m of the system. At steady state, velocity equals input divided by drag, so drag was estimated as:

d = u / v_ss

m was estimated from the time constant as:

m = d * tau

These values were calculated in Python:

```cpp
t_90 = tau_fit * np.log(10)
d = 120.0 / v_ss_fit
m = d * tau_fit
```

The fitted curve was plotted together with the measured velocity data in Figure 2.

<p align="center">
  <img src="../img/lab7/exp_fit.png" width="30%">
</p>
<p align="center">
  <b>Figure 2:</b> Exponential Fit of Velocity Data.
</p>

---

## Initialize KF (Python) TODO

After estimating d and m from the step response, a 2 state model was built. The state vector was defined as position and velocity relative to the wall. 

Following the lecture sign convention, position was defined so that the ToF measurement is the negative of the state position. In other words, if the robot moves closer to the wall, the measured distance decreases, while the state variable becomes less negative.

The continuous-time state model was written using position and velocity as the two states. The first state equation relates position to velocity, and the second state equation models velocity using the drag and inertia terms found from the step response. Using the fitted values from Section 1, I computed the continuous-time A and B matrices in Python:

```cpp
u_step = 120.0
v_ss = 1122.10
tau  = 0.4167

d_drag = float(u_step / v_ss)
m_mass = float(d_drag * tau)

A = np.array([
    [0.0,  1.0],
    [0.0, -(d_drag / m_mass)]
], dtype=float)

B = np.array([
    [0.0],
    [1.0 / m_mass]
], dtype=float)

C = np.array([[-1.0, 0.0]], dtype=float)
```

The C matrix shows that only position is directly measured, and that the measurement is negative distance from the wall. Velocity is not measured directly.

The model was then discretized using the lab approximation:

Ad = I + Delta_T * A

Bd = Delta_T * B

I used a sampling time of Delta_T = 0.033 s, which matches the approximate ToF update period used on the robot.

```cpp
Delta_T = 0.033

Ad = np.eye(2) + Delta_T * A
Bd = Delta_T * B
```

After this, I prepared the measurement data for the Kalman Filter. The recorded ToF timestamps and distances were converted into numpy arrays, and the time axis was shifted so that it starts at zero. Since the Kalman Filter runs on a fixed sampling interval, I created a uniformly spaced time vector using Delta_T and interpolated the ToF measurements onto that time grid.

```cpp
t_meas = np.array(tof1_t, dtype=float)
z_meas = np.array(tof1_mm, dtype=float)

t_meas = t_meas - t_meas[0]

t_kf = np.arange(0, t_meas[-1], Delta_T)
z_kf = np.interp(t_kf, t_meas, z_meas)
```

The initial state was chosen from the first ToF measurement. Since the lecture convention uses position as negative distance from the wall, the initial state was set to negative of the first measured distance, and the initial velocity was set to zero.

```cpp
mu = np.array([
    [-z_kf[0]],
    [0.0]
], dtype=float)
```

I also initialized the state covariance matrix. I used a relatively large uncertainty in both position and velocity at the beginning, since the exact initial state is not known perfectly.

```cpp
Sigma = np.array([
    [150.0**2, 0.0],
    [0.0, 400.0**2]
], dtype=float)
```

To make the Kalman Filter work properly, process noise and measurement noise covariance matrices also had to be chosen. I assumed the noise terms were uncorrelated, so both matrices were chosen to be diagonal. The measurement noise covariance represents how much the ToF sensor measurement is trusted. Since the ToF readings were fairly noisy but still usable, I started with a measurement standard deviation of about 20 mm.

```cpp
sigma_3 = 20.0
Sigma_z = np.array([[sigma_3**2]], dtype=float)
```

The process noise covariance represents uncertainty in the model itself. Since the robot does not exactly follow an ideal first-order model, I allowed uncertainty in both the position and velocity states. I started with 30 mm and 30 mm/s style ballpark values for the two state uncertainties and used these as tuning parameters later.

```cpp
sigma_1 = 30.0
sigma_2 = 30.0

Sigma_u = np.array([
    [sigma_1**2, 0.0],
    [0.0, sigma_2**2]
], dtype=float)
```

These values were chosen as reasonable starting points. Their relative size determines how much the filter trusts the model versus the sensor. If the process noise is too small, the filter trusts the model too much and may not correct well from real measurements. If the measurement noise is too small, the filter follows the noisy ToF signal too closely. These values were later adjusted during tuning.

Finally, I implemented the Kalman Filter using the same form shown in lecture. The function performs the prediction step first, then computes the Kalman gain, then updates the state and covariance using the measurement.

```cpp
def kf(mu, sigma, u, y):
    mu_p = Ad.dot(mu) + Bd.dot([[u]])
    sigma_p = Ad.dot(sigma).dot(Ad.T) + Sigma_u

    sigma_m = C.dot(sigma_p).dot(C.T) + Sigma_z
    kkf_gain = sigma_p.dot(C.T).dot(np.linalg.inv(sigma_m))

    y_m = np.array([[y]]) - C.dot(mu_p)
    mu = mu_p + kkf_gain.dot(y_m)
    sigma = (np.eye(2) - kkf_gain.dot(C)).dot(sigma_p)

    return mu, sigma
```

This completed the initialization of the Kalman Filter in Python. The next step was to run this filter over the recorded step response data and compare the estimated distance and velocity against the raw sensor data.

---

## KF Implementation on Jupyter

There are limitations of the gyroscope sensor. By default, the ICM-20948 gyroscope is configured with a range of ±250 degrees per second (dps), as shown in the Arduino code.

According to the ICM-20948 datasheet, the gyroscope supports four ranges: ±250, ±500, ±1000, and ±2000 dps. The robot can rotate faster than this, so default setting may not be enough. The range can be adjusted by changing the GYRO_FS_SEL register.

<p align="center">
  <img src="../img/lab6/gyro_sensor.png" width="80%">
</p>
<p align="center">
  <b>Figure 1:</b> ICM-20948 Datasheet, Gyroscope Angular Velocity.
</p>

---

## KF Implementation on the Robot

The goal of this lab was to control the robot's orientation. The robot rotates in place by driving the wheels at equal speeds in opposite directions.

Yaw was used as the feedback signal for the controller. The orientation error was computed as the difference between the target yaw setpoint and the measured yaw.

```cpp
float err = wrap_angle_deg(setpoint_deg - yaw);
```

The wrap_angle_deg() function ensures the controller always takes the shortest rotational path by keeping the error between −180° and 180°.

<br>

#### PID Control

Next, a derivative term was added to help reduce overshoot. Instead of calculating the derivative of the error, the gyroscope angular velocity was used directly since angular velocity is the derivative of orientation.

```cpp
float d = -Kd_yaw * gz_dps;
```

This also helps avoid derivative kick, which can occur when the setpoint changes suddenly. Because the derivative term depends on angular velocity rather than the error derivative, sudden setpoint changes do not create large spikes in the control signal.

Because the derivative term comes directly from the gyroscope measurement rather than the noisy angle samples, an additional low pass filter was not needed.

Kd of 0.3 was chosen for best response.

<p align="center">
  <img src="../img/lab6/PID_angle.png" width="30%">
  <img src="../img/lab6/PID_error.png" width="30%">
  <img src="../img/lab6/PID_pwm.png" width="30%">
</p>
<p align="center">
  <b>Figure 4:</b> Plots of PID Control Data.
</p>

Video 3 below shows the result of PID controller.

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
  <b>Video 3:</b> PID Controller.
</p>

<br>

---

## Discussion

This lab provided experience implementing closed loop orientation control using IMU and DMP. Overall, it improved understanding of PID control, tuning controller gains, and using IMU data to tune stable orientation control. The use of the DMP for yaw estimation demonstrated how sensor fusion can reduce drift and provide more reliable orientation measurements.

---

## Acknowledgment

I referenced [Aidan McNay](https://aidan-mcnay.github.io/fast-robots-docs/lab6/)’s pages from last year.

Parts of this report and website formatting were assisted by AI tools (ChatGPT) for grammar checking and webpage structuring. All code was written, tested, and validated by the author.
