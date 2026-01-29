# Lab 1: The Artemis Board and Bluetooth

---

## Prelab 1A

The Arduino IDE was installed and updated to the latest version. The Apollo3 board package was added using the provided JSON configuration. After connecting the Artemis board via USB, the corresponding board and serial port were selected.

---

## Lab 1A Tasks

<br>

#### Blink

We first ran the Blink example from File → Examples → 01.Basics.

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
The serial monitor was opened to see the output and send input.

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

#### Additional Task: Simple Electronic Tuner

The sketch can detect three frequencies and print the corresponding musical note to the serial monitor. The three chosen frequencies are C4 (262 Hz), A4 (440 Hz), and E5 (659 Hz).

The implementation was built upon the code from Example1_MicrophoneOutput. An additional helper method getFreq() was implemented to help identify the dominant frequency. This function compares the detected peak frequency to the predefined frequencies and determines the closest match.

```cpp
const char* getFreq(uint32_t freq)
{
    if (freq >= 250 && freq <= 275) return "C4";   // 262 Hz
    else if (freq >= 420 && freq <= 460) return "A4"; // 440 Hz
    else if (freq >= 625 && freq <= 690) return "E5"; // 659 Hz
    else return ""; // no matching note
}
```

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

## Prelab 1B

#### Setup

Python 3.13 was installed and configured. In addition, a virtual environment named FastRobots_ble was created using venv.
These commands were used for activating or deactivating the virtual environment.

```cpp
source FastRobots_ble/bin/activate
deactivate
```

After installation, the lab codebase was downloaded and unzipped inside project directory. JupyterLab was launched from the project directory.

ArduinoBLE was then installed from library manager, and ble_arduino.ino sketch was burned and loaded. The MAC address of the Artemis was printed in Figure 1.

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/MAC Address.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 1:</b> Artemis MAC Address Printed on Serial Monitor.
</p>

---

#### Codebase Overview

##### BLE Communication

Communication between the computer and the Artemis board is implemented using GATT characteristics. Different characteristics are used for different data types. Commands are sent from the computer to the Artemis as strings.

On the computer side, commands are sent using this function:

```cpp
ble.send_command(cmd_type, data)
```

On the Artemis side, the command string is received and parsed to determine what action to take. A switch statement, handle_command(), is then used to execute the correct command.

<br>

##### Arduino Side Code (Artemis)

The Arduino sketch (ble_arduino.ino) is responsible for:
- Setting up the BLE service and characteristics
- Parsing command strings using the RobotCommand class
- Sending responses back to computer. EString is used for building response strings on the Artemis side.

<br>

##### Python Side Code (Computer)

The Jupyter Lab (Python) is responsible for:
- Sending commands
- Reading values from BLE characteristics
- Receiving asynchronous data using notifications

---

## Lab 1B Tasks

#### Configuration

Before doing the lab tasks, some configuration steps were required. First, the MAC address printed by the Artemis board in the serial monitor was copied into the computer side configuration file `connections.yaml`.

Next, a new BLE service UUID was generated using the code:

```cpp
from uuid import uuid4
uuid4()
```

This prevents connections to other students’ Artemis boards that may be advertising the default service UUID. The newly generated UUID replaced the original BLE service UUID in the Arduino sketch and the Python configuration file `connections.yaml`.

The command type definitions in the Arduino enum CommandTypes and cmd_types.py were matched exactly, and ble_arduino.ino was reuploaded to Artemis. Finally, the provided Jupyter notebook demo.ipynb was opened and all cells were tested to confirm that BLE communication was functioning.

---

#### Task 1: ECHO Command

A string was sent from the computer to the Artemis using the `ECHO` command. The Artemis added onto the string and responded.

**Example:**
- Sent: `HiHello`
- Received: `Robot says -> HiHello :)`

The code uses RobotCommand: get_next_value() to extract the received string into a character array. An EString object is then used to construct the response by appending string components.

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

A new command `GET_TIME_MILLIS` was implemented to return the current time in milliseconds using the string format "T:123456".

The code builds the response string using EString by appending a prefix and the current time obtained from millis(). The formatted string is sent to the computer and printed to the serial monitor.

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
  <b>Figure 4:</b> Jupyter Lab Showing Response from Artemis for GET_TIME_MILLIS.
</p>

---

#### Task 4: Notification Handler

The code defines a notification handler that decodes incoming data and extracts timestamp values prefixed with "T:". These timestamps are appended to a list as integers. Repeated GET_TIME_MILLIS commands are sent to collect multiple timestamp before stopping notifications.

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task4.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 5:</b> Jupyter Lab Showing Notification Handler and Printed Times.
</p>

---

#### Task 5: Notification Handler

The notification handler records incoming timestamp while also tracking the total number of bytes received. A while loop repeatedly sends GET_TIME_MILLIS commands. The total bytes and elapsed time are used to estimate the effective BLE data transfer rate.

From the shown calculation in Figure 6, the effective data transfer rate is 113 bytes/sec.

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task5.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 6:</b> Jupyter Lab Showing Looped Time Samples and Data Transfer Rate.
</p>

---

#### Task 6: Send Time Data

Instead of looping over each time and sending times, task 6 was implemented to store times inside a global array. The command SEND_TIME_DATA was used to send over the array.

Two additional methods were implemented in the switch statement, SEND_TIME_DATA and RECORD_TIME_DATA.

For SEND_TIME_DATA, the code iterates through the recorded timestamp array and formats each entry as a string started with "T:", same as in GET_TIME_MILLIS.

```cpp
case SEND_TIME_DATA:
            for (int i = 0; i < time_buf_len; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append(time_buf[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            Serial.print("Sent time data count: ");
            Serial.println(time_buf_len);
        
            break;
```

For RECORD_TIME_DATA, the code initializes the array index and records timestamp and temperature samples for a fixed duration. Each iteration stores the current time from millis() and the corresponding temperature reading into arrays.

```cpp
case RECORD_TIME_DATA:
        {
            time_buf_len = 0;
            unsigned long start = millis();
            while ((millis() - start) < 3000) {
                if (time_buf_len < TIME_BUF_SIZE){
                    time_buf[time_buf_len] = (int) millis();
                    temp_buf[time_buf_len] = getTempDegF();
                    time_buf_len++;
                } else {
                    break;
                }
            }

            Serial.print("Recorded samples: ");
            Serial.println(time_buf_len);

            break;
        }
```

On the computer side, RECORD_TIME_DATA and SEND_TIME_DATA commands were sent to Artemis, and the system sleeps for 3.2 seconds for Artemis to record and send before stopping notification. The collected timestamps and total bytes are used to calculate the effective data transfer rate.

```cpp
# Task 6
times = []
total_bytes = 0

def notification_handler_6(uuid, data: bytearray):
    global total_bytes
    s = data.decode()
    total_bytes += len(data)

    if s[:2] == "T:":
        times.append(int(s[2:]))

ble.start_notify(ble.uuid['RX_STRING'], notification_handler_6)
ble.send_command(CMD.RECORD_TIME_DATA, "")
ble.send_command(CMD.SEND_TIME_DATA, "")

# Artemis recording ~3s
time.sleep(3.2)

ble.stop_notify(ble.uuid["RX_STRING"])

# Calculate Data Transfer Rate
duration = (times[-1] - times[0])/1000.0
rate = total_bytes / duration
```

As shown from Figure 7, the data transfer rate is 17978 bytes/sec, a significant increase from the previous method.

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task6.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 7:</b> Jupyter Lab Showing Time Samples and Data Transfer Rate.
</p>

---

#### Task 7: Get Temp Readings

A second array was added to store the corresponding temperature readings with the time readings. A new command GET_TEMP_READINGS was implemented. The overall structure was similar to SEND_TIME_DATA, but the additional temperature reading was appended to the recording array, separated by a ",".

```cpp
case GET_TEMP_READINGS:            
            for (int i = 0; i < time_buf_len; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append(time_buf[i]);
                tx_estring_value.append(",");
                tx_estring_value.append(temp_buf[i]);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());
            }

            Serial.print("Sent time, temp data count: ");
            Serial.println(time_buf_len);
        
            break;
```

On the computer side, the notification handler decodes incoming data and parses into timestamp and temperature values, each appended to separate arrays.

```cpp
# Task 7
times = []
temps = []

def notification_handler_7(uuid, data: bytearray):
    s = data.decode()

    if s[:2] == "T:":
        result = s[2:]
        t, temp = result.split(",")
        times.append(int(t))
        temps.append(float(temp))

ble.start_notify(ble.uuid["RX_STRING"], notification_handler_7)

ble.send_command(CMD.RECORD_TIME_DATA, "")
ble.send_command(CMD.GET_TEMP_READINGS, "")

# Artemis recording ~3s
time.sleep(3.2)

ble.stop_notify(ble.uuid["RX_STRING"])
```

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task7.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 8:</b> Jupyter Lab Showing Time and Temp Samples.
</p>

---

#### Task 8: Discussion

The two methods differ in when the data is sent over. In the first method, each timestamp is sent immediately from the Artemis to the computer as soon as it is generated. This is easy to implement and is useful for real time debugging. However, this method is slow because every sample must go through BLE communication. As a result, the sampling speed is limited by Bluetooth.

In the second method, the Artemis stores the data locally in arrays first and sends it later. This allows the Artemis to record data much faster. The recording speed is mainly limited by how fast millis() can be read, so it can reach much higher sampling rates. However, it is important to make sure the array size isn't filled and being overwritten.

The Artemis has 384 kB RAM = 393,216 bytes. If only timestamps are stored, there is around 4 bytes/sample, so the maximum number of samples that can be stored is 393,216/4 = 98,304 samples. If there are timestamps and temperature, there is around 8 bytes/sample, so the maximum number of samples that can be stored is 393,216/8 = 49,152 samples. However, in practice not all RAM storage can be used, so the actual samples that can be stored is less than the calculated number.

---

#### Additional Task 9: Effective Data Rate And Overhead

To test for effective data rate and overhead, messages of different sizes were sent to Artemis, and Artemis replied with the specified size. The timestamps of sent commands and received replies were recorded to calculate the transfer rate. Different reply sizes ranging from small packets (5 bytes) to larger packets (120 bytes) were tested.

On the Artemis side, a new command REPLY_N was added to extract an integer value representing the reply size. An array of the specified size is filled with repeating alphabetical characters.

```cpp
case REPLY_N:            
            char arr[MAX_MSG_SIZE];
            int n;

            success = robot_cmd.get_next_value(n);
            if (!success)
                return;

            for (int i = 0; i < n; i++) {
                arr[i] = 'a' + (i%26);
            }
            arr[n] = '\0';
            tx_characteristic_string.writeValue(arr);

            Serial.print("n: ");
            Serial.println(n);

            break;
```

On the computer side, the test were implemented by requesting the Artemis to reply with a specified size. For each size, 20 messages were sent while BLE notifications record. The total number of bytes received and the duration are used to calculate the effective data rate.

```cpp
# Task 9
rx_times = []
rx_sizes = []
n_sizes = [5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120]
rates = []

def notification_handler_8(uuid, data: bytearray):
    rx_times.append(time.time())
    rx_sizes.append(len(data))

for n in n_sizes:
    rx_times.clear()
    rx_sizes.clear()

    ble.start_notify(ble.uuid['RX_STRING'], notification_handler_8)

    start = time.time()
    for _ in range(20):
        ble.send_command(CMD.REPLY_N, n)
        time.sleep(0.02)
    end = time.time()

    ble.stop_notify(ble.uuid['RX_STRING'])

    total_bytes = sum(rx_sizes)
    duration = end - start
    rate = total_bytes / duration
    
    rates.append(rate)
```

The plot in Figure 9 shows that the effective data rate increases almost linearly as the reply size n increases. For small packets like 5 bytes, the effective data rate is very low, indicating that a large portion of the time is dominated by overhead.

As the reply size increases to 120 bytes, the effective data rate improves. This shows that larger replies reduce the relative impact of overhead. While the total duration increases slightly, the efficiency improves.

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task9.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 9:</b> Effective Data Rate vs. N Size.
</p>

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task9 Output.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 10:</b> Number of Bytes, Total Bytesm Duration, and Data Rate Data.
</p>

---

#### Additional Task 10: Reliability

To evaluate the reliability of communication, a new test was implemented to send information at high rates. This test tracks if all messages sent by the Artemis are received, or if there is some missing data.

On the Artemis side, a new command RELIABILITY_TEST is used to send the amount of data points the computer told it to send. The format it sends is "T: i", where i increments from 0 to m−1.

```cpp
case RELIABILITY_TEST:
            int m;

            success = robot_cmd.get_next_value(m);
            if (!success)
                return;

            for (int i = 0; i < m; i++) {
                tx_estring_value.clear();
                tx_estring_value.append("T:");
                tx_estring_value.append(i);
                tx_characteristic_string.writeValue(tx_estring_value.c_str());

            }
            break;
```

On the computer side, the numbers received are stored in a set. The computer listens for incoming data for a fixed duration. The received sequence are then compared against the expected range. Any missing numbers indicate that they were not successfully received.

```cpp
# Task 10
received = set()
M = 1000

def notification_handler_10(uuid, data: bytearray):
    global received
    s = data.decode()
    if s[:2] == "T:":
        received.add(int(s[2:]))

ble.start_notify(ble.uuid["RX_STRING"], notification_handler_10)
ble.send_command(CMD.RELIABILITY_TEST, M)

time.sleep(12)
ble.stop_notify(ble.uuid["RX_STRING"])

missing = []

for i in range (M):
    if i not in received:
        missing.append(i)
loss = 100.0 * len(missing) / M
```

From Figure 11, the computer does read all the data from Artemis, without missing any number.

<div style="text-align:center; margin:20px 0;">
  <img src="../img/lab1/Task10.png" width="600">
</div>
<p style="text-align:center;">
  <b>Figure 11:</b> Jupyter Lab Output Showing No Missing Number.
</p>

---

## Discussion

This lab provides experience with programming the Artemis board and communicating with computer wirelessly using BLE. We practiced sending commands from the computer to the Artemis and receiving data back, which helped us understand how BLE characteristics and notifications work. We also learned the difference between sending data in real time vs. storing data in an array and sending it all at once.

There was no significant challenge encountered during this lab. Overall, this lab helped build a strong understanding of BLE communication and data handling, which will be important for future labs.

---

## Acknowledgment

I referenced [Jeffrey Cai](https://jcai2565.github.io/2025/02/02/lab1.html) and [Aidan McNay](https://aidan-mcnay.github.io/fast-robots-docs/lab1/#)’s pages from last year for reference.

Parts of this report and website formatting were assisted by AI tools (ChatGPT) for grammar checking and webpage structuring. All code was written, tested, and validated by the author.
