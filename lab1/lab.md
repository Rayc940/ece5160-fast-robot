# Lab 1: The Artemis Board and Bluetooth

---

## Objective

The goal of this lab was to become familiar with the SparkFun RedBoard Artemis Nano and to establish communication between the Artemis board and the computer. This lab was divided into two parts:

- **Lab 1A:** Arduino IDE setup and programming the Artemis  
- **Lab 1B:** Bluetooth Low Energy (BLE) communication using Python and Jupyter

---

## Prelab 1A

The Arduino IDE was installed and updated to the latest version. The SparkFun Apollo3 board package was added through the Boards Manager using the provided JSON configuration. After connecting the Artemis board via USB, the corresponding board and serial port were selected.

---

## Lab 1A Tasks

<br>

#### Blink

We first ran the Blink example from File → Examples → 01.Basics.
This ensures that the board was properly connected and that we could successfully upload code.

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
  <b>Video 1:</b> Blink Example.
</p>
<br>

#### Serial Communication Test

Next, we ran Example4_Serial from File → Examples → Apollo3.
This example was used to test serial communication between the Artemis board and the computer.
The serial monitor was opened to see the output and send input to the board.

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

#### Temperature Sensor Test

We then ran Example2_analogRead from File → Examples → Apollo3.
This example was used to test the onboard temperature sensor.
By blowing warm air on the chip, we were able to observe increases in the temperature values.

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

#### Microphone Test

Next, we ran Example1_MicrophoneOutput from File → Examples → PDM.
This example was used to test the onboard microphone.
A C major scale audio from YouTube was played near the board, and serial monitor showed changing detected frequency content. This confirms that the microphone and PDM interface were functioning correctly.

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

#### Additional Task: Simple Electronic Tuner

For the additional task, we combined the microphone input with serial output to create a simple electronic tuner.
The code can detect three frequency ranges and print the corresponding musical note to the serial monitor. The three chosen frequencies were C4 (262 Hz), A4 (440 Hz), and E5 (659 Hz).

The implementation was built upon the code from Example1_MicrophoneOutput. An additional helper method getFreq() was implemented to help identify the dominant frequency. This function compares the detected peak frequency to the predefined frequencies and determines the closest match. If there is a match, the serial monitor will print the detected musical note.

```cpp
const char* getFreq(uint32_t freq)
{
    if (freq >= 250 && freq <= 275) return "C4";   // 262 Hz
    else if (freq >= 420 && freq <= 460) return "A4"; // 440 Hz
    else if (freq >= 625 && freq <= 690) return "E5"; // 659 Hz
    else return ""; // no matching note
}
```

The video shows serial monitor printing the three different notes, which was played from a tuner app that can select frequencies to play.

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

## Prelab 1B

#### Setup

Python 3.13 was installed and configured. In addition, to ensure package isolation, a virtual environment named FastRobots_ble was created using venv. This virtual environment was activated before installing any required packages. These commands were used for activating or deactivating the virtual environment.

'''cpp
source FastRobots_ble/bin/activate
deactivate
'''

After activating the virtual environment, the following Python packages were installed inside the virtual environment:
- **numpy**
- **pyyaml**
- **colorama**
- **nest_asyncio**
- **bleak**
- **jupyterlab**

After installation, the lab codebase was downloaded and unzipped inside project directory. JupyterLab was launched from the project directory, which can be used for BLE communication.

ArduinoBLE was then installed from library manager, and ble_arduino.ino sketch was burned and loaded. The MAC address of the Artemis was printed:

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/MAC Address.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 1:</b> Artemis MAC Address Printed on Serial Monitor.
</p>

---

#### Codebase Overview

##### Overall Architecture

The provided codebase is divided into two main components:

- Arduino code running on the Artemis board
- Python code running on the computer

These two components communicate using BLE. BLE provides a low power communication tool that allows the computer to send commands to the Artemis board and receive sensor data or responses in return.

##### BLE Communication

Communication between the computer and the Artemis board is implemented using GATT characteristics, each identified by a different UUID. Different characteristics are used for different data types, such as integers, floats, and strings. Commands are sent from the computer to the Artemis as strings, and the Artemis processes the command and sends back a response.

On the computer side, commands are sent using the function:

```cpp
ble.send_command(cmd_type, data)
```

The command is sent as a formatted string over BLE. On the Artemis side, the command string is received and parsed to determine what action to take. A switch statement is then used to execute the correct command, such as responding to a PING or sending data.

##### Arduino Side Code (Artemis)

The Arduino sketch (ble_arduino.ino) is responsible for:
- Setting up the BLE service and characteristics
- Receiving command strings from the computer
- Parsing commands using the RobotCommand class
- Sending responses back to computer. EString is used for building outgoing response strings on the Artemis side. It provides functions like c_str() to construct messages before sending.

##### Python Side Code (Computer)

On the computer side, BLE communication is handled by the ArtemisBLEController class. This class provides simple functions to:
- Connect to the Artemis board
- Send commands
- Read values from BLE characteristics
- Receive asynchronous data using notifications

---

## Lab 1B Tasks

#### Configuration
BLE configuration values such as the Artemis MAC address and UUIDs are stored in connections.yaml. These values must match the UUIDs defined in the Arduino sketch. After changing the configuration file, it must be reloaded before reconnecting.

Before running the tasks, several configurations were required:
- Updated the Artemis MAC address in `connections.yaml`
- Generated and synchronized a new BLE service UUID
- Verified command types matched between Arduino and Python
- Set the serial baud rate to 115200 bps

These steps ensured that the computer connected to the correct Artemis board and avoided cross-connections with nearby devices.

---

### Task 1: ECHO Command

A string was sent from the computer to the Artemis using the `ECHO` command. The Artemis augmented the string and transmitted it back to the computer.

**Example:**
- Sent: `HiHello`
- Received: `Robot says -> HiHello :)`

This confirmed successful two-way BLE communication.

📷 *(Insert Jupyter output screenshot here)*

---

### Task 2: Sending Three Floats

The `SEND_THREE_FLOATS` command was used to transmit three floating-point values from the computer to the Artemis. These values were parsed correctly on the Arduino side and printed to the serial monitor.

This demonstrated that structured numerical data could be reliably transmitted over BLE.

📷 *(Insert serial monitor screenshot here)*

---

### Task 3: Time Stamping and Notification Handling

A new command `GET_TIME_MILLIS` was implemented to return the current time in milliseconds. A Python notification handler was set up to asynchronously receive these messages and extract the timestamp.

A loop was used to repeatedly send timestamps to the computer for several seconds. The received data was logged and used to estimate the effective data transfer rate.

📈 *(Insert plot of timestamps vs. packet count here)*

---

### Task 4: Buffered Data Transfer

Instead of sending timestamps immediately, timestamps were stored in a fixed-size array on the Artemis. Once the array was full, a `SEND_TIME_DATA` command transmitted the buffered data to the computer.

A second array was added to store temperature readings recorded at the same timestamps. These paired values were transmitted together and reconstructed on the computer.

This approach reduced BLE overhead and enabled higher-rate data collection.

---

## Results

The BLE communication was reliable for both real-time and buffered data transmission. Buffered transmission significantly reduced overhead and allowed faster data acquisition without packet loss.

📷 *(Insert plots, screenshots, and videos as needed)*

---

## Discussion

This lab provided hands-on experience with embedded programming, BLE communication, and asynchronous data handling. One challenge was ensuring consistent UUIDs between Arduino and Python, but once resolved, communication was stable.

Buffered data transmission proved more efficient for high-rate sampling, while real-time transmission was useful for debugging and quick feedback. These techniques will be critical for future labs involving sensor data and robot control.

---
