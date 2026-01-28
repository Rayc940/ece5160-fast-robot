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

```cpp
source FastRobots_ble/bin/activate
deactivate
```

After activating the virtual environment, the following Python packages were installed inside the virtual environment:
- numpy
- pyyaml
- colorama
- nest_asyncio
- bleak
- jupyterlab

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

The command is sent as a formatted string over BLE. On the Artemis side, the command string is received and parsed to determine what action to take. A switch statement is then used to execute the correct command, such as responding to a PING or sending data. The helper function handle_commmand() was used to help with switching commands.

##### Arduino Side Code (Artemis)

The Arduino sketch (ble_arduino.ino) is responsible for:
- Setting up the BLE service and characteristics
- Receiving command strings from the computer
- Parsing commands using the RobotCommand class
- Sending responses back to computer. EString is used for building response strings on the Artemis side. It provides functions like c_str() to construct messages before sending.

##### Python Side Code (Computer)

On the computer side, BLE communication is handled by the ArtemisBLEController class. This class is responsible for:
- Connecting to the Artemis board
- Sending commands
- Reading values from BLE characteristics
- Receiving asynchronous data using notifications

---

## Lab 1B Tasks

#### Configuration

Before doing the lab tasks, some configuration steps were required. First, the MAC address printed by the Artemis board in the serial monitor was copied into the computer side configuration file `connections.yaml`.

Next, a new BLE service UUID was generated and updated on both the Arduino and Python sides using the code:

```cpp
from uuid import uuid4
uuid4()
```

This prevents connections to other students’ Artemis boards that may be advertising the default service UUID. The newly generated UUID replaced the original BLE service UUID in the Arduino sketch and the Python configuration file `connections.yaml`.

The command type definitions in the Arduino enum CommandTypes and cmd_types.py were matched exactly, and ble_arduino.ino was reuploaded to Artemis. Finally, the provided Jupyter notebook demo.ipynb was opened and all cells were tested to confirm that BLE communication was functioning before proceeding to the lab tasks.

---

#### Task 1: ECHO Command

A string was sent from the computer to the Artemis using the `ECHO` command. The Artemis added onto the string and responded it back to the computer.

**Example:**
- Sent: `HiHello`
- Received: `Robot says -> HiHello :)`

The code shown below uses RobotCommand: get_next_value() to extract the received string into a character array. An EString object is then used to construct the response by appending multiple string components before sending the final array through the BLE string characteristic. The response is printed to serial monitor for verification.

```cpp
case ECHO:
            char char_arr[MAX_MSG_SIZE];
            // Extract the next value from the command string as a character array
            success = robot_cmd.get_next_value(char_arr);
            if (!success)
                return;
            
            tx_estring_value.clear();
            tx_estring_value.append("Robot says -> ");
            tx_estring_value.append(char_arr);
            tx_estring_value.append(" :)");
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Sent back: ");
            Serial.println(tx_estring_value.c_str());
            
            break;
```

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task1.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 2:</b> Jupyter Lab Showing Response from Artemis for ECHO.
</p>

---

#### Task 2: Send Three Floats

The `SEND_THREE_FLOATS` command was used to send three float values from the computer to the Artemis. These values were printed to the serial monitor.

The code shown below extracts three float values from the received command string using RobotCommand:get_next_value(). Each value is stored in a separate variable.

```cpp
case SEND_THREE_FLOATS:
            float float_a, float_b, float_c;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_a);
            if (!success)
                return;

            // Extract the next value from the command string as an integer
            success = robot_cmd.get_next_value(float_b);
            if (!success)
                return;

            success = robot_cmd.get_next_value(float_c);
            if (!success)
                return;

            Serial.print("Three Floats: ");
            Serial.print(float_a);
            Serial.print(", ");
            Serial.print(float_b);
            Serial.print(", ");
            Serial.print(float_c);
            
            break;
```

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task2.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 3:</b> Serial Monitor Showing Response from Artemis for SEND_THREE_FLOATS.
</p>

---

#### Task 3: Get Time Millis

A new command `GET_TIME_MILLIS` was implemented to return the current time in milliseconds using the string format `T:123456`.

The code shown below builds the response string using EString by appending a prefix and the current system time obtained from millis(). The formatted string is transmitted to the computer through the BLE string characteristic and printed to the serial monitor.

```cpp
case GET_TIME_MILLIS:
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append(String(millis()).c_str());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());

            Serial.print("Current Time: ");
            Serial.print(tx_estring_value.c_str());
            
            break;
```

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task3 Python.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 4:</b> Jupyter Lab Showing Response from Artemis for GET_TIME_MILLIS
</p>

---

#### Task 4: Notification Handler

The code shown below defines a notification handler that decodes incoming data and extracts timestamp values prefixed with "T:". These timestamps are appended to a list as integers each time a notification is received. Notifications are enabled on the string characteristic, and repeated GET_TIME_MILLIS commands are sent to collect multiple timestamp samples before stopping notifications.

```cpp
# Task 4
times = []
def notification_handler(uuid, data: bytearray):
    s = data.decode()
    if s[:2] == "T:":
        times.append(int(s[2:]))

ble.start_notify(ble.uuid['RX_STRING'], notification_handler)

# Testing
for _ in range(10):
    ble.send_command(CMD.GET_TIME_MILLIS, "")
    time.sleep(0.02)
ble.stop_notify(ble.uuid["RX_STRING"])
print("Times:", times)
```

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task4.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 5:</b> Jupyter Lab Showing Notification Handler and Printed Times
</p>

---

#### Task 5: Notification Handler

The notification handler records incoming timestamp while also tracking the total number of bytes received over BLE. A while loop repeatedly sends GET_TIME_MILLIS commands for a fixed duration, allowing multiple samples to be collected through notifications. After completion, the total bytes and elapsed time are used to estimate the effective BLE data transfer rate.

```cpp
# Task 5
times = []
total_bytes = 0
def notification_handler_5(uuid, data: bytearray):
    global total_bytes
    s = data.decode()
    total_bytes += len(data)
    if s[:2] == "T:":
        times.append(int(s[2:]))

ble.start_notify(ble.uuid['RX_STRING'], notification_handler_5)

# Run for < 3s
start = time.time()
while time.time() - start < 3.0:
    ble.send_command(CMD.GET_TIME_MILLIS, "")
    time.sleep(0.02)

ble.stop_notify(ble.uuid["RX_STRING"])
print("Times:", times)

# Calculate Data Transfer Rate
duration = (times[-1] - times[0])/1000.0
rate = total_bytes / duration

print("Samples: ", len(times))
print("Duration: ", duration)
print("Data Transfer Rate: ", rate)
```

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task5.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 6:</b> Jupyter Lab Showing Looped Time Samples and Data Transfer Rate
</p>

The total elapsed time was calculated from the last time data subtract the first time data.
From the shown calculation in Figure 6, the effective data transfer rate is 113 bytes/sec.

---

#### Task 6: ???

The notification handler records incoming timestamp while also tracking the total number of bytes received over BLE. A timed loop repeatedly sends GET_TIME_MILLIS commands for a fixed duration, allowing multiple samples to be collected through notifications. After completion, the total bytes and elapsed time are used to estimate the effective BLE data transfer rate.

```cpp
# Task 6
times = []
total_bytes = 0
N = 0

def notification_handler_6(uuid, data: bytearray):
    global total_bytes, N
    s = data.decode()
    total_bytes += len(data)
    
    if s[:2] == "N:":
        N = int(s[2:])
        
    if s[:2] == "T:":
        times.append(int(s[2:]))

ble.start_notify(ble.uuid['RX_STRING'], notification_handler_6)
ble.send_command(CMD.RECORD_TIME_DATA, "")
ble.send_command(CMD.SEND_TIME_DATA, "")

# Artemis recording ~3s
time.sleep(3.2) 

while len(times) < N:
    time.sleep(0.01)

ble.stop_notify(ble.uuid["RX_STRING"])

print("Times:", times)
print("Samples: ", len(times))
```

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task5.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 6:</b> Jupyter Lab Showing Looped Time Samples and Data Transfer Rate
</p>

---

## Results

The BLE communication was reliable for both real-time and buffered data transmission. Buffered transmission significantly reduced overhead and allowed faster data acquisition without packet loss.

📷 *(Insert plots, screenshots, and videos as needed)*

---

## Discussion

This lab provided hands-on experience with embedded programming, BLE communication, and asynchronous data handling. One challenge was ensuring consistent UUIDs between Arduino and Python, but once resolved, communication was stable.

Buffered data transmission proved more efficient for high-rate sampling, while real-time transmission was useful for debugging and quick feedback. These techniques will be critical for future labs involving sensor data and robot control.

---
