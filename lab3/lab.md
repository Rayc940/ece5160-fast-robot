## Prelab

#### I2C Sensor Address

VL53L1X Time of Flight Sensors are used. The default I2C address is 0x52 from the datasheet.

<br>

#### ToF Sensors Approach

Both VL53L1X sensors share the same I2C address. As a result, they can't be connected to the same I2C bus at the same time.

There are two approaches to solve this:

###### 1) Programmatically Change Address:

One sensor is powered on first. Its address is changed to a new value programmatically. The second sensor is then powered and kept at the default address.

Advantage:
- Both TOF sensors are active
- Fast and real time sampling

Disadvantage:
- More complex initialization for each power cycle

<br>

###### 2) Continuously Enable/Disable Sensors

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

The VL53L1X has a narrow field of view of 27°. One sensor will be facing directly forward, the other facing outward to the side. This approach prioritizes detecting obstacles directly ahead while also providing side distance information. 

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

#### Solder Connections

One of the 650mAh batteries was cut and soldered to JST connector. TOF sensor was soldered to QWIIC cable:

- Red: 3.3V
- Black: GND
- Yellow: SCL
- Blue: SDA

Picture of your ToF sensor connected to your QWIIC breakout board

#### I2C Scanning

After the first TOF was connected, Example05_wire_I2C was ran to verify I2C address.

Screenshot of Artemis scanning for I2C device (and discussion on I2C address)

The address shown is 0x29, which doesn't match with the datasheet's 0x52. However, this makes sense because 0x29 in binary is 0101001, and 0x52 in binary is 01010010. The I2C 8 bit format is the 7 bit address << 1 + R/W bit. The Arduino is using the 7 bit format.

<br>

#### TOF Mode

The VL53L1X supports short, medium, and long distance modes. These modes trade off range and measurement stability.

- Short mode (~1.3 m) provides more stable and reliable readings at close distances.
- Long mode (~4 m) allows greater range but can be noisier and more sensitive to surface conditions.
- Medium mode (~3 m) is a balance between the two.

Since the robot mainly needs to detect nearby obstacles, short mode was chosen. Most obstacles are within 1 meter, and short mode provides more consistent and stable readings in this range.

#### TOF Sensor Tests

- Range
- Accuracy
- Repeatability
- Ranging Time

Discussion and pictures of sensor data with chosen mode

#### Range Test

TOF sensor was taped perpendicular to white wall. A set of distances: {10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150} cm are tested.

#### Two TOF Sensors

To hook up both TOF sensors, GPIO 4 and 6 on the Artemis are used to connect to XSHUT pins on TOFs. init_tof() are called to first initialize TOF1, change its address to 0x30, then initialize TOF2.

```cpp
void init_tof() {
  Wire.begin();
  Wire.setClock(400000);
  set XSHUT_1 and XSHUT_2 as OUTPUT
  set XSHUT_1 and XSHUT_2 low
  delay(10);

  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  
  distanceSensor1.setI2CAddress(0x30);

  digitalWrite(XSHUT_2, HIGH);
  delay(10);

  set both TOF short distance mode
  set both TOF timing budget 33ms
  start ranging on both TOF
}
```

To test for functionality, code below was ran in loop().

```cpp
if (distanceSensor1.checkForDataReady() &&
  distanceSensor2.checkForDataReady()) {

  int d1 = distanceSensor1.getDistance();
  int d2 = distanceSensor2.getDistance();

  distanceSensor1.clearInterrupt();
  distanceSensor2.clearInterrupt();

  print result

  delay(100);
  }
```

Two TOF Sensor Video

<br>

#### TOF Sensor Speed

To ensure that it doesn't block while waiting for TOF data, the code below was tested.

```cpp
Serial.println((uint32_t)micros());

if (distanceSensor1.checkForDataReady()) {
  int d1 = distanceSensor1.getDistance();
  distanceSensor1.clearInterrupt();

  print d1 result
}

if (distanceSensor2.checkForDataReady()) {
  int d2 = distanceSensor2.getDistance();
  distanceSensor2.clearInterrupt();

  print d2 result
}
```

The loop continuously prints results while TOF is running in parallel, which proves it is working.

non blocking tof pic

Next, to investigate limiting factor and loop time, code below was used:

```cpp
if (distanceSensor1.checkForDataReady()) {
  tof1_ready_count++;
  int d1 = distanceSensor1.getDistance();
  distanceSensor1.clearInterrupt();
}
if (distanceSensor2.checkForDataReady()) {
  tof2_ready_count++;
  int d2 = distanceSensor2.getDistance();
  distanceSensor2.clearInterrupt();
}

loop_count++;

uint32_t now_ms = millis();
  if (now_ms - last_rate_ms >= 1000) {
        print results to serial monitor

  make all counts = 0
  last_rate_ms = now_ms;
```


<p align="center">
  <img src="../img/lab3/loop time.png" width="30%">
  <img src="../img/lab3/loop time tof.png" width="30%">
  <img src="../img/lab3/loop time imu tof.png" width="30%">
</p>
<p align="center">
  <b>Figure TODO:</b> Loop time with nothing, TOF, IMU
</p>



#### TOF and IMU Record

The switch statement START_IMU_RECORD, SEND_IMU_RECORD from lab 2 were modified to START_RECORD and SEND_RECORD. It now sends I: IMU data, T1: TOF1 data, T2: TOF2 data. 

Python code was modified as well to parse the additional data.

Graph of data sent over BLE for TOF and IMU are shown.

#### Infrared Trasmission Sensor

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
