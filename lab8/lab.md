## Objective

The goal of this lab was to combine everything from the previous labs and make the robot do a fast stunt. Task A: Flip was chosen. The robot starts a few meters away from the wall, drives forward quickly, flips near the wall, and then drives back.

---

## Flip State Machine

The flip was implemented using a simple state machine with four main states:

```cpp
enum FlipState {
    FLIP_IDLE = 0,
    FLIP_DRIVE_TO_WALL = 1,
    FLIP_EXECUTE = 2,
    FLIP_DRIVE_AWAY = 3
};
```

The robot stays in FLIP_IDLE until the start command is sent from Python. Then it drives forward at a constant PWM. A primed condition is used to prevent early stopping due to TOF noise. Once the robot travels far enough, it becomes "primed", and then waits until it is close enough to the wall to start the flip.

```cpp
case FLIP_DRIVE_TO_WALL:
{
    driveForwardCal(flip_forward_pwm, calibration_forward);
    flip_last_u_pwm = (float)flip_forward_pwm;

    log_flip_data(now_ms, FLIP_DRIVE_TO_WALL, raw_dist, est_dist, flip_forward_pwm, pitch_cf, roll_cf);

    if (est_dist >= flip_primed_distance_mm) {
        flip_primed = true;
    }

    if (flip_primed && est_dist <= flip_target_mm) {
        flip_state = FLIP_EXECUTE;
        flip_state_start_ms = now_ms;
    }

    break;
}
```

When the robot reaches the target distance, it switches to FLIP_EXECUTE. In this state, the robot reverses at high speed for a fixed amount of time. This part is open loop because maximum speed is needed rather than controlled motion. After flip_time_ms passed, the robot switched to FLIP_DRIVE_AWAY.

```cpp
case FLIP_EXECUTE:
{
    driveReverseCal(flip_reverse_pwm, calibration_reverse);
    flip_last_u_pwm = (float)(-flip_reverse_pwm);

    log_flip_data(now_ms, FLIP_EXECUTE, raw_dist, est_dist, -flip_reverse_pwm, pitch_cf, roll_cf);

    if ((now_ms - flip_state_start_ms) >= (uint32_t)flip_time_ms) {
        flip_state = FLIP_DRIVE_AWAY;
        flip_state_start_ms = now_ms;
    }
    break;
}
```

Finally, in FLIP_DRIVE_AWAY, the robot drives forward and stops when it had moved far enough from the wall.

```cpp
case FLIP_DRIVE_AWAY:
{
    driveReverseCal(flip_reverse_pwm, calibration_reverse);
    flip_last_u_pwm = (float)(-flip_reverse_pwm);

    log_flip_data(now_ms, FLIP_DRIVE_AWAY, raw_dist, est_dist, -flip_reverse_pwm, pitch_cf, roll_cf);

    if ((now_ms - flip_state_start_ms) >= 500) {
        coastStop();
        flip_active = false;
        flip_state = FLIP_IDLE;
        flip_last_u_pwm = 0.0f;
        recording = false;
        record_done = true;
    }
    break;
}
```

---

## Python Control

On the Python side, a BLE handler was implemented to receive and parse the stunt data. These data were appended into arrays for plotting, same as previous labs.

```cpp
initialize arrays
def parse_flip():
    parse incoming data
def flip handler:
    append data to arrays

clear arrays
start BLE notification
send START_FLIP_STUNT
send GET_FLIP_DATA
wait for data
stop BLE notification
plot data
```

After the run, the logged data was sent using GET_FLIP_DATA, which sends the number of samples and then all logged values.

---

## Kalman Filter

The same Kalman Filter from Lab 7 was reused. Inside flip_step(), the Kalman Filter was updated every loop using. The raw TOF distance was stored in raw_dist, while the estimated distance used by the stunt was calculated from the Kalman Filter state:

```cpp
kf_step(flip_last_u_pwm, new_tof_sample, (float)last_dist_mm);

int raw_dist = last_dist_mm;
int est_dist = (int)roundf(-kf_x);
```

Since the TOF sensor updates slowly, using only raw distance can lead to noise and delay. The Kalman Filter provides a smoother and more responsive estimate. This is the same idea described in the previous lab.

---

## Results

The three trials are shown below. 

<p align="center">
  <img src="../img/lab8/1_dist.png" width="30%">
  <img src="../img/lab8/1_angle.png" width="30%">
  <img src="../img/lab8/1_fsm.png" width="30%">
</p>
<p align="center">
  <b>Figure 1:</b> Flip Distance, Angle, and FSM State.
</p>

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/_zxlH9y34Yo"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 1:</b> Flip Trial 1.
</p>

<br>

<p align="center">
  <img src="../img/lab8/2_dist.png" width="30%">
  <img src="../img/lab8/2_angle.png" width="30%">
  <img src="../img/lab8/2_fsm.png" width="30%">
</p>
<p align="center">
  <b>Figure 2:</b> Flip Distance, Angle, and FSM State.
</p>

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/K8BvRX5d73w"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 2:</b> Flip Trial 2.
</p>

<br>

<p align="center">
  <img src="../img/lab8/3_dist.png" width="30%">
  <img src="../img/lab8/3_angle.png" width="30%">
  <img src="../img/lab8/3_fsm.png" width="30%">
</p>
<p align="center">
  <b>Figure 3:</b> Flip Distance, Angle, and FSM State.
</p>

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/3r3O6Sn3EDA"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 3:</b> Flip Trial 3.
</p>

---

## Bloopers

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/Q9FwigFCoAA"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 4:</b> Flip Blooper 1.
</p>

<br>

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/NgSK6AxTGj8"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 5:</b> Flip Blooper 2.
</p>

---

## Discussion

This lab focused on performing a fast flip using open loop control and a state machine. The robot was driven with fixed high PWM to build enough speed, so timing became the main challenge. Using the Kalman Filter estimate made this more consistent than raw TOF data. The state machine also helped organize the behavior and made tuning easier. Overall, the main difficulty was balancing speed and timing to get a reliable flip.

---

## Acknowledgment

I referenced [Trevor Dales](https://trevordales.github.io/MAE4190/lab8/)’s pages from last year.

Parts of this report and website formatting were assisted by AI tools (ChatGPT) for grammar checking and webpage structuring. All code was written, tested, and validated by the author.
