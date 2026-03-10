## Prelab

A bluetooth debugging system was set up before starting controller tuning. The goal was to let the Artemis run PID for a fixed amount of time while storing data locally, and then send the recorded data for analyzing. 

```cpp
initalize lists

def parse_pid(line: str):
    parts = line.split(",")
    parse parts

def data_handler(_uuid, response: bytearray):
    parse_pid(incoming data)
    store in local lists

start BLE notification
set PID gains
start PID run
get PID data
wait for PID data
stop BLE notification
```

On Artemis side, four commands are added:

```cpp
START_PID_RUN, STOP_PID_RUN, GET_PID_DATA, SET_PID_GAINS
```

When the laptop sent START_PID_RUN, the robot cleared the old PID log, reset the controller state, and began the closed loop run. 

```cpp
case START_PID_RUN:
        {
        pid_len = 0;
        pid_running = true;
        pid_start_ms = millis();
        prev_pid_us = 0;
        i_accum = 0.0f;
        prev_err = 0.0f;
        d_filt = 0.0f;
        break;
        }
```

To make tuning easier, PID gains were also sent over BLE with SET_PID_GAINS. This helped quickly test different Kp, Ki, and Kd values without needing to upload code everytime.

```cpp
case SET_PID_GAINS:
        float kp, ki, kd;
        success = robot_cmd.get_next_value(kp); if (!success) return;
        success = robot_cmd.get_next_value(ki); if (!success) return;
        success = robot_cmd.get_next_value(kd); if (!success) return;
        Kp = kp; Ki = ki; Kd = kd;
        break;
```

For safety, there is a hard stop. The robot stopped if the run time exceeded the set limit, if the measured distance became too small, or if BLE connection was lost.

```cpp
if (now_ms - pid_start_ms >= PID_RUN_MS) {
        coastStop();
        pid_running = false;
        return;
}

if (last_dist_mm > 0 && last_dist_mm < 120) {
        coastStop();
        pid_running = false;
        return;
}
```

After the run finished, GET_PID_DATA was called to get data over BLE. The Artemis first sent a header containing the number of samples, then streamed each saved data line back over BLE.

```cpp
case GET_PID_DATA:
        send headers
        for (int i = 0; i < pid_len; i++) {
                send data: time, TOF ready, distance, error, P, I, D, PWM
        }
        break;
```

<br>

---

## Lab Tasks

### Position Control

The goal of the task was to make the robot drive toward a wall as quickly as possible and stop at a target distance of 304 mm using ToF feedback. 

The controller was implemented directly on the Artemis using the front TOF sensor as the feedback. At each step, the robot measured the distance to wall, computed the error, and then generated a motor command from the PID terms.

$$
u_k = K_p e_k + K_i \sum e_k \Delta t + K_d \frac{e_k - e_{k-1}}{\Delta t}
$$

In code, the error was computed as:

```cpp
int dist = last_dist_mm;
int err = dist - setpoint_mm;
```

#### P Control

The proportional term was used first. The proportional controller directly scales the distance error to generate a motor command.

```cpp
float p = Kp * (float)err;
```

After tuning, Kp of 0.1 was chosen. Proportional control was able to drive the robot toward the wall and stop near the target distance, but some oscillation occurred.

<p align="center">
  <img src="../img/lab5/P_dist.png" width="30%">
  <img src="../img/lab5/P_error.png" width="30%">
  <img src="../img/lab5/P_pwm.png" width="30%">
</p>
<p align="center">
  <b>Figure 1:</b> Plots of P Control Data.
</p>

Video 1 below shows the result of P only controller.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/oFxlku3yY9s"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video TODO:</b> P Control to Wall.
</p>

<br>

#### PI Control

To improve the steady state accuracy, an integral term was added. The integral accumulates error over time and pushes the robot closer to the exact setpoint.

```cpp
i_accum += (float)err * dt;
float i = Ki * i_accum;
```

However, when the robot starts far from the wall, the error can remain large for a long time. This causes the integral term to accumulate excessively, which may lead to integrator wind-up. To prevent this, the accumulated integral value was clamped within a fixed range.

```cpp
if (i_accum > I_CLAMP) i_accum = I_CLAMP;
if (i_accum < -I_CLAMP) i_accum = -I_CLAMP
```

With Ki = 0.001, the PI controller reduced the steady state error and allowed the robot to settle very close to the target without oscillation.

<p align="center">
  <img src="../img/lab5/PI_dist.png" width="30%">
  <img src="../img/lab5/PI_error.png" width="30%">
  <img src="../img/lab5/PI_pwm.png" width="30%">
</p>
<p align="center">
  <b>Figure 1:</b> Plots of PI Control Data.
</p>

Video 1 below shows the result of PI controller.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/oFxlku3yY9s"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video TODO:</b> PI Control to Wall.
</p>

<br>

#### PID Control

A derivative term was also added to help reduce overshoot and slow the robot as it approached the wall. The derivative term reacts to the rate of change of the error, which helps dampen motion near the setpoint.

```cpp
float d_raw = ((float)err - prev_err) / dt;
float d = Kd * d_raw;
```

Because the ToF sensor measurements are discrete and noisy, using the raw derivative caused unstable behavior. To reduce this noise, a low pass filter was applied.

```cpp
d_filt = alpha_d * d_raw + (1.0f - alpha_d) * d_filt;
float d = Kd * d_filt;
```

This smoothing reduced sudden spikes in the derivative signal and improved stability.

<p align="center">
  <img src="../img/lab5/PID_dist.png" width="30%">
  <img src="../img/lab5/PID_error.png" width="30%">
  <img src="../img/lab5/PID_pwm.png" width="30%">
</p>
<p align="center">
  <b>Figure 1:</b> Plots of PID Control Data.
</p>

Video 3 below shows the result of PID controller.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/oFxlku3yY9s"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video TODO:</b> PI Control to Wall.
</p>

<br>

#### TOF Sensor Setup

The ToF sensor settings also affect performance. Faster sensing allows the controller to react more quickly to changes.

For this lab, the sensor was configured in short distance mode with a 33 ms timing budget, which provided sufficiently fast updates. 

A faster sampling rate can help reduce the delay between measurement and controller response, which improves stability.


```cpp
distanceSensor1.setDistanceModeShort();
distanceSensor1.setTimingBudgetInMs(33);
distanceSensor1.startRanging();
```

<br>

#### Motor Deadband

From Lab 4, motors have a minimum PWM limit which they would not move. If the controller output became too small near the setpoint, the robot might stop moving even though the error was not zero.

To address this, a deadband helper function was applied to the PWM command before sending it to the motors.

```cpp
int apply_deadband(int pwm)
{
  int a = abs(pwm);
  if (a == 0) return 0;
  if (a < PWM_DEADBAND) a = PWM_DEADBAND;
  if (a > 255) a = 255;
  return (pwm < 0) ? -a : a;
}
```

<br>

#### Perturbation Test

The robot was also tested with external perturbations. After reaching the target distance, the robot was manually pushed closer and farther away.

In both cases, the controller responded by driving the robot back toward the 304 mm setpoint.

<p align="center">
  <img src="../img/lab5/pert_dist.png" width="30%">
  <img src="../img/lab5/pert_error.png" width="30%">
  <img src="../img/lab5/pert_pwm.png" width="30%">
</p>
<p align="center">
  <b>Figure 1:</b> Plots of PID Control Perturbation Data.
</p>

Video 3 below shows the result of PID controller under perturbation.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/oFxlku3yY9s"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video TODO:</b> PID Control, Perturbation.
</p>

<br>

---

### Extrapolation

##### TOF Sensor Frequency

The update frequency of the TOF sensor was measured and compared to the PID control loop rate. This was done by counting how many times the main loop executed in one second and how many times the ToF sensor reported a new reading in that same time.

<p align="center">
  <img src="../img/lab5/pert_dist.png" width="30%">
  <img src="../img/lab5/pert_error.png" width="30%">
  <img src="../img/lab5/pert_pwm.png" width="30%">
</p>
<p align="center">
  <b>Figure TODO:</b> TODO.
</p>

This shows that the control loop is running much faster than the TOF sensor. Because of this, the controller cannot depend on receiving a new distance reading every loop iteration.

To handle this, the PID controller was allowed to run every loop, even when no new TOF data was available. If a new measurement was available, the stored distance value was updated. If no new measurement was available, the controller continued to run using the most recent saved value.

<br>

#### Linear Extrapolation

Using the last saved distance value works, but it causes the controller to act on a stair-step signal because the ToF only updates every ~0.1 s. To improve this, a simple linear extrapolation method was added.

The robot stores the two most recent ToF readings and their timestamps. When a new ToF measurement arrives, the slope between the two points is calculated as:

$$
\text{slope} = \frac{d_{current} - d_{previous}}{t_{current} - t_{previous}}
$$

This slope is then used to estimate the distance at the current time:

$$
\text{slope} = \frac{d_{current} - d_{previous}}{t_{current} - t_{previous}}
$$

This gives an estimated distance that updates every PID loop instead of only when a new ToF sample arrives.

When a new ToF reading is available, the previous distance and timestamp are shifted into history, and the new reading becomes the latest sample.

```cpp
prev_dist_mm = last_dist_mm;
prev_tof_us = last_tof_us;

last_dist_mm = d1;
last_tof_us = now_us;
```

The extrapolated distance is then computed from the last two ToF samples:

```cpp
int get_extrapolated_dist_mm(uint32_t now_us)
{
    if (!tof_hist_valid) return last_dist_mm;

    float dt_sample = (last_tof_us - prev_tof_us) / 1e6f;
    if (dt_sample <= 0.0f) return last_dist_mm;

    float slope = ((float)last_dist_mm - (float)prev_dist_mm) / dt_sample;
    float dt_now = (now_us - last_tof_us) / 1e6f;
    float d_est = (float)last_dist_mm + slope * dt_now;

    if (d_est < 0.0f) d_est = 0.0f;
    if (d_est > 4000.0f) d_est = 4000.0f;

    return (int)roundf(d_est);
}
```

Inside the PID loop, the raw ToF distance and extrapolated distance were both available, and the controller used the extrapolated value:

```cpp
int raw_dist = last_dist_mm;
int dist = get_extrapolated_dist_mm(now_us);
int err = dist - setpoint_mm;
```

With this approach, the PID controller still runs at 164 Hz, but instead of using a stale ToF value for multiple loop iterations, it uses a continuously updated estimate of the wall distance between sensor updates. Since the ToF sensor only updates at 10–11 Hz, this extrapolation helps smooth the distance input to the controller and provides a more responsive estimate of the robot’s position.

To evaluate the method, both the raw ToF distance and the extrapolated distance were logged and plotted on the same graph. The raw signal shows discrete jumps because of the slow sensor update rate, while the extrapolated signal fills in the gaps between measurements. This makes the estimated distance smoother and more suitable for high-speed wall approach.


<p align="center">
  <img src="../img/lab4/freq_scope.jpg" width="80%">
</p>
<p align="center">
  <b>Figure TODO:</b> Input PWM Frequency.
</p>

<br>

---

## Discussion

This lab provided experience implementing closed loop control and sensor based navigation on the robot. Overall, this lab improved understanding of PID control, tuning controller gains, and integrating sensor feedback to achieve stable position control. The addition of distance extrapolation also demonstrated how estimation techniques can improve controller performance.

---

## Acknowledgment

I referenced [Aidan McNay](https://aidan-mcnay.github.io/fast-robots-docs/lab5/)’s pages from last year.

Parts of this report and website formatting were assisted by AI tools (ChatGPT) for grammar checking and webpage structuring. All code was written, tested, and validated by the author.
