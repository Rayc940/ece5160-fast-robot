# Lab 1: The Artemis Board and Bluetooth

---

## Objective

The goal of this lab was to become familiar with the SparkFun RedBoard Artemis Nano and to establish reliable communication between the Artemis board and a computer. This lab was divided into two parts:

- **Lab 1A:** Arduino IDE setup and onboard peripherals  
- **Lab 1B:** Bluetooth Low Energy (BLE) communication using Python and Jupyter

---

## Prelab

### Computer and Software Setup

The Arduino IDE was installed and updated to the latest version. The SparkFun Apollo3 board package was added through the Boards Manager using the provided JSON configuration. After connecting the Artemis board via USB, the correct board and serial port were selected.

Once the BLE Arduino sketch was uploaded, the board successfully printed its MAC address to the serial monitor, confirming correct setup.

📷 *(Insert screenshot of MAC address here)*

---

### Codebase Overview and Bluetooth Communication

Bluetooth Low Energy (BLE) enables low-power wireless communication between the Artemis board and a computer. The provided lab codebase establishes this communication using:

- **ArduinoBLE** on the Artemis side  
- **Bleak (Python)** on the computer side  

Commands are sent from the computer to the Artemis as formatted strings, and the Artemis responds by writing data to BLE characteristics. These characteristics act as communication channels identified by UUIDs.

The codebase separates responsibilities clearly:
- Python handles connection, commands, and data logging
- Arduino handles command parsing and sensor interaction

---

## Lab Tasks

### Configuration

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

## Code Snippets

Relevant code snippets used in this lab are shown below. Full source code is available in the GitHub repository.

```cpp
// Example Arduino command handler snippet
case ECHO:
  tx_estring.clear();
  tx_estring.append("Robot says -> ");
  tx_estring.append(rx_string);
  tx_estring.append(" :)");
  tx_characteristic.writeValue(tx_estring.c_str());
  break;
