## Prelab

#### I2C Sensor Address

VL53L1X Time of Flight Sensors are used. The default I2C address is 0x52.

<br>

#### ToF Sensors Approach

Both VL53L1X sensors share the same I2C address. As a result, they can't be connected to the same I2C bus at the same time.

There are two approaches to solve this:

1. Programmatically Change Address:
One sensor is powered first. Its address is changed to a new value programmatically. The second sensor is then powered and kept at the default address.

Advantage:
- Both TOF sensors are active
- Fast and real time sampling

Disadvantage:
- More complex initialization for each power cycle

<br>

2. Continuously Enable/Disable Sensors
One sensor is powered first, the other in shutdown, and vice versa. This avoids changing address.

Advantage:
- Simpler, no address reassignment
- Fast and real time sampling

Disadvantage:
- Only one sensor active
- Reduced sampling rate

<br>
  
The first option is chosen because for a fast moving robot that relies on quick and continuous distance updates, it is important to have both sensors operate at the same time and provide real time obstacle detection.

<br>

#### Placement and Miss

The VL53L1X has a narrow field of view of 27°. One sensor will be facing directly forward, the other facing outward to the side. This approach prioritizes detecting obstacles directly ahead while also providing side distance information for wall following. 

However, this approach may lead to a blind spot on the left of the RC car. Figure 1 below shows possible blind spots. In addition, low obstacles, dark absorptive materials, or transparent surfaces may not reflect enough infrared light, resulting in missed readings.

<p align="center">
  <img src="../img/lab2/imu_setup.jpeg" width="80%">
</p>
<p align="center">
  <b>Figure 1:</b> TODO.
</p>

<br>

#### Wiring Diagram

The wiring diagram is shown is figure 2 below.

<p align="center">
  <img src="../img/lab2/imu_setup.jpeg" width="80%">
</p>
<p align="center">
  <b>Figure 2:</b> TODO.
</p>

<br>

---

## Lab Tasks

#### TOF Sensor to QWIIC

Picture of your ToF sensor connected to your QWIIC breakout board
Screenshot of Artemis scanning for I2C device (and discussion on I2C address)
Discussion and pictures of sensor data with chosen mode
2 ToF sensors and the IMU: Discussion and screenshot/video of sensors working in parallel
Tof sensor speed: Discussion on speed and limiting factor; include code snippet of how you do this
Time v Distance: Include graph of data sent over bluetooth (2 sensors)
Time v Angle: Include graph of data sent over bluetooth
(5000) Discussion on infrared transmission based sensors
(5000) Sensitivity of sensors to colors and textures


<p align="center">
  <img src="../img/lab2/pitch_-90.png" width="30%">
  <img src="../img/lab2/pitch_90.png" width="30%">
  <img src="../img/lab2/roll pitch 0.png" width="30%">
</p>
<p align="center">
  <b>Figure 1:</b> Ouputs showing pitch at {-90, 0, 90} degrees.
</p>

<br>

---

## Discussion

This lab provids hands on experience working with the IMU. This helped me understand how filtering and sensor techniques can improve angle estimation. There was no significant challenge encountered during this lab. Overall, this lab built a strong foundation in IMU data processing.

---

## Acknowledgment

I referenced [Aidan McNay](https://aidan-mcnay.github.io/fast-robots-docs/lab2/)’s pages from last year.

Parts of this report and website formatting were assisted by AI tools (ChatGPT) for grammar checking and webpage structuring. All code was written, tested, and validated by the author.
