# Lab 1 – The Artemis Board and Bluetooth
**ECE 5160: Fast Robots**
**Ray Chang**

---

## Overview
This lab focused on setting up the SparkFun RedBoard Artemis Nano and establishing reliable communication between the board and a host computer.  
Lab 1A covered basic programming, serial communication, and onboard sensors, while Lab 1B introduced Bluetooth Low Energy (BLE) communication using a provided Arduino–Python codebase.

---

## Prelab

### Computer and Board Setup
I installed the latest version of the Arduino IDE and added the SparkFun Apollo3 board package. After connecting the Artemis board via USB, I selected the correct board and serial port. I verified successful communication by uploading example sketches and observing serial output.

After flashing the BLE Arduino sketch, the Artemis printed its Bluetooth MAC address to the serial monitor, confirming that the board and Bluetooth stack were functioning correctly.

📸 **MAC address printed in Serial Monitor**  
![MAC Address](assets/images/lab1/mac_address.png)

---

### Codebase Overview and BLE Communication
The provided codebase consists of two main components: Arduino firmware running on the Artemis board and a Python interface running on the host computer.

On the Artemis side, BLE communication is implemented using ArduinoBLE. A BLE service is defined along with multiple characteristics for sending and receiving data. Incoming commands are transmitted as strings and parsed using the `RobotCommand` class, which extracts command types and associated values.

On the computer side, Python uses the Bleak library to connect to the Artemis, send commands, and receive data asynchronously through notification handlers. This architecture allows flexible command-based communication and efficient data transfer for debugging and logging sensor data.

---

## Lab Tasks

### Configuration
To ensure reliable communication, several configurations were required:
- Updated the Artemis MAC address in `connections.yaml` to match the address printed by the board
- Generated a unique BLE service UUID and updated it in both `ble_arduino.ino` and `connections.yaml`
- Verified that command types defined in the Arduino enum matched those in `cmd_types.py`
- Reflashed the Artemis with the updated BLE sketch

These steps ensured that my computer connected only to my own Artemis board and that commands were interpreted correctly.

---

### Task 1: ECHO Command
I implemented the ECHO command to send a string from the computer to the Artemis. The Artemis appended a prefix and suffix to the received string and transmitted it back over BLE.

📸 **ECHO command output in Jupyter Notebook**  
![Echo](assets/images/lab1/echo.png)

This verified bidirectional communication and correct string handling.

---

### Task 2: Sending Three Floats
Using the `SEND_THREE_FLOATS` command, I transmitted three floating-point values from Python to the Artemis. The values were extracted on the Arduino side using the `RobotCommand` interface and printed to the serial monitor for verification.

📸 **Received float values on Artemis**  
![Floats](assets/images/lab1/floats.png)

---

### Task 3: Timestamp Command
I added a `GET_TIME_MILLIS` command that returned the current time in milliseconds in the format `T:<value>`. The Artemis sent this string via a BLE characteristic.

On the Python side, a notification handler parsed the received string and extracted the timestamp value.

---

### Task 4: Streaming Timestamps
A loop was implemented on the Artemis to repeatedly send timestamps to the computer. The Python notification handler recorded the arrival times to estimate how quickly messages could be transmitted.

From this experiment, the effective data rate was limited by BLE notification overhead rather than computation speed on the Artemis.

📈 **Timestamp reception over time**  
![Timestamps](assets/images/lab1/timestamps_plot.png)

---

### Task 5: Buffered Data Transfer
Instead of sending each timestamp immediately, I stored timestamps in a global array on the Artemis. Once the array was full, a `SEND_TIME_DATA` command transmitted all stored values sequentially.

This approach reduced communication overhead and improved reliability when transferring larger datasets.

---

### Task 6: Timestamped Temperature Readings
A second array was added to store temperature readings captur
