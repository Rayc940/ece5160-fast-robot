## Prelab

---

## IMU Setup

The "SparkFun 9DOF IMU Breakout_ICM 20948_Arduino Library" was installed, and IMU was connected to Artemis board. Example1_Basics was ran from File → Examples → SparkFun 9DOF IMU Breakout - ICM 20948 - Arduino Library → Arduino.

#### AD0_VAL

AD0_VAL is the last bit of I2C address of the IMU, where the IMU supports two possible I2C addresses. The default setting is AD0_VAL = 1. If the ADR jumper is closed, the address bit flips and AD0_VAL should be changed to 0. This allows multiple identical devices to share the same I2C bus without address conflicts.

#### Observation from Accelerometer and Gyroscope

Accelerometer: When the board is held still, z axis reads about 1 g because it is measuring gravity, while the other two axes are close to zero. If the board is flipped over, the sign of z axis changes and becomes -1 g. When the board is accelerated, the acceleration values increase on the axis in the direction of motion.

Gyroscope: The gyroscope measures angular velocity. When the board is not rotating, the gyroscope values stay near zero. When the board is rotated, the corresponding gyroscope axis values changes, and faster rotations produce larger values.

#### Visual Indication

To make it obvious that the board is running, a simple visual indicator was added in setup(). It blinks the LED three times slowly.

```cpp
for (int k = 0; k < 3; k++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(400);
    digitalWrite(LED_BUILTIN, LOW);
    delay(400);
```

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/dTsu4LU6gek"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 1:</b> Setup Indicator: Blink.
</p>
<br>

---

## Accelerometer

Next, I ran Example4_Serial from File → Examples → Apollo3.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/66WNAS3cpZ8"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 2:</b> Serial Communication Example.
</p>
<br>

---

## Gyroscope

I then ran Example2_analogRead from File → Examples → Apollo3.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/_jkc4uSTeV0"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 3:</b> Temperature Sensor Example.
</p>
<br>

---

## Sample Data

Next, I ran Example1_MicrophoneOutput from File → Examples → PDM.
A YouTube Video of C major scale audio was played, and serial monitor showed changing detected frequency content.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/HzKOsx0vtjQ"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 4:</b> Microphone Example.
</p>
<br>

---

## Record a stunt

Video 5 shows serial monitor printing the three different notes, which was played from a tuner app.

<div style="text-align:center; margin:30px 0;">
  <iframe
    width="560"
    height="315"
    src="https://www.youtube.com/embed/eilouPZ5N54"
    frameborder="0"
    allowfullscreen>
  </iframe>
</div>
<p style="text-align:center;">
  <b>Video 5:</b> Simple Electronic Tuner Example.
</p>
<br>

---

#### Example Pic template

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task7.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 8:</b> Jupyter Lab Showing Time and Temp Samples.
</p>

---

## Discussion

This lab provides experience with programming the Artemis board and communicating with computer wirelessly using BLE. I practiced sending commands from the computer to the Artemis and receiving data back, which helped us understand how BLE characteristics and notifications work. I also learned the difference between sending data in real time vs. storing data in an array and sending it all at once.

There was no significant challenge encountered during this lab. Overall, this lab helped build a strong understanding of BLE communication and data handling, which will be important for future labs.

---

## Acknowledgment

I referenced [Jeffrey Cai](https://jcai2565.github.io/2025/02/02/lab1.html) and [Aidan McNay](https://aidan-mcnay.github.io/fast-robots-docs/lab1/#)’s pages from last year.

Parts of this report and website formatting were assisted by AI tools (ChatGPT) for grammar checking and webpage structuring. All code was written, tested, and validated by the author.
