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

#### Position Control

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
  <img src="../img/lab5/P_dist.png" width="33%">
  <img src="../img/lab5/P_error.png" width="33%">
  <img src="../img/lab5/P_pwm.png" width="33%">
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

#### I Control

RC car was then used to test if wheel turns. Video 2 below shows RC car wheel turning forward and reverse, confirming functionality. Battery was used instead of external DC power supply.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/sf4BN3Vpjv8"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 2:</b> Motor Driver with Wheels on One Side.
</p>

<br>

#### Two Motor Drivers

After the first motor driver confirmed functionality, same process was applied to the second motor driver. Code below was used for both wheels turing forward and reverse.

```cpp
#define IN1 2
#define IN2 3
#define IN3 0
#define IN4 1

void setup() {
  initialize pins
  Serial.begin(115200);
}

void loop() {
  move forward
  stop
  move reverse
  stop
}
```

Video 3 below shows both wheels turning forward and reverse, powering using battery.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/cwK3nXUN26Y"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 3:</b> Motor Driver with Wheels on Both Sides.
</p>

<br>

#### RC Car Assembly

RC car was assembled with all components: Artemis, IMU, TOF sensors, motor drivers. The two TOF sensors are placed as discussed in lab 3.

<p align="center">
  <img src="../img/lab4/assembly.jpg" width="80%">
</p>
<p align="center">
  <b>Figure 3:</b> Assembled RC Car.
</p>

<br>

#### Lower Limit PWM

Next, the minimum PWM to make RC car travel was experimentally determined to be 25 = 10% duty cycle. Video 4 below shows RC car moving while on the setting.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/Rtli0zgsHWg"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 4:</b> Lower Limit PWM Forward.
</p>

<br>

For on axis turn, the lower limit PWM is 95 = 37% duty cycle. Video 5 below shows RC car turning while on the setting.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/IvMr34Qdg7I"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 5:</b> Lower Limit PWM Turn.
</p>

<br>

#### Calibration Factor

The two motors moved at differnet speeds, making the car unable to travel in straight line. To combat this, a calibration factor was introduced to reduce pwm on the more powerful side. Calibration factor was experimentally determined to be 0.8. The function clipPWM() is used to make sure pwm value is within 0 to 255 after calibration.

```cpp
void driveForward(int pwm) {
  int pwmL = pwm;
  int pwmR = clipPWM((int)(pwm * calibration));

  analogWrite(L_IN1, pwmL);
  analogWrite(L_IN2, 255);

  analogWrite(R_IN1, 255);
  analogWrite(R_IN2, pwmR);
}
```

Video 6 successfully demonstrates RC car traveling in a straight line.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/BOTg3QIemNw"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 6:</b> RC Car in Straight Line.
</p>

<br>

#### Open Loop Control

Code below was sent from Python to Artemis for open loop control.

```cpp
ble.send_command(CMD.FORWARD, "1000")
ble.send_command(CMD.LEFT, "500")
ble.send_command(CMD.BACKWARD, "1000")
ble.send_command(CMD.RIGHT, "500")
ble.send_command(CMD.FORWARD, "1000")
```

Video 7 below shows the RC car traveling forward and turning.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/JOQKsnqk7UQ"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 7:</b> Open Loop Control.
</p>

<br>

#### analogWrite Frequency

The PWM frequency generated by analogWrite() was measured using an oscilloscope by probing the Artemis input to the motor driver. The measured frequency was around 180 Hz, corresponding to a period of about 5.6 ms.

<p align="center">
  <img src="../img/lab4/freq_scope.jpg" width="80%">
</p>
<p align="center">
  <b>Figure 4:</b> Input PWM Frequency.
</p>

This PWM frequency is fast enough for motors. The motor cannot react to each individual PWM pulse because it changes much more slowly than the switching signal. It effectively sees the average voltage over time. Because of this, the motor speed is mainly determined by the duty cycle of the PWM signal.

The advantages of a higher PWM frequency are:
- Reduced audible noise, since the switching frequency would be higher than the human hearing range.
- Reduced current ripple that leads to smoother operation.
- Less nosiy sensor readings since lower frequency vibration and noise can couple into IMU and TOF measurements.

The disadvantages of a higher PWM frequency are:
- Higher switching losses in the motor driver.
- Higher EMI noise.

<br>

#### Lower Limit PWM While Moving

Lastly, the lowest PWM value at which the robot can run while in motion was determined to be 10 (4% duty cycle). This was tested using code below. Robot was first traveling at a higher speed, then slowed down.

```cpp
driveForward(220);
delay(1000);

driveForward(245);
delay(3000);

coastStop();
delay(1000);
```

Video 8 below shows the forward lower limit PWM in motion.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/OyFa-V3Fv2Q"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 8:</b> Lower Limit PWM in Motion Forward.
</p>

However, the lowest PWM value at which the robot can turn while in motion was the same (95 = 37% duty cycle) as before. This is likely because turning requires overcoming similar friction regardless of whether the robot is starting from rest or already turning.

---

## Discussion

This lab provided experience working with motor drivers and final robot assembly. Overall, this lab improved understanding of PWM motor control and tuning required for open loop robot.

---

## Acknowledgment

I referenced [Aidan McNay](https://aidan-mcnay.github.io/fast-robots-docs/lab4/)’s pages from last year.

Parts of this report and website formatting were assisted by AI tools (ChatGPT) for grammar checking and webpage structuring. All code was written, tested, and validated by the author.
