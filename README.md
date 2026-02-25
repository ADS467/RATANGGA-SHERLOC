
---

## 1. Hardware Design (CAD)
The `/cad` directory contains the design files for the physical components of the system.

* **Locator (User Segment):** Design for the wearable device worn by the hiker.
* **CubeSat (Space Segment):** Structural design for the 1U satellite relay - featuring a layered internal component stack.

---

## 2. Simulations
The project includes both statistical and functional simulations

### Python Simulation
Located in `/simulations/python`, this tool provides algorithmic validation.

* **Fall Detection Machine Testing:** A test case simulation that tests the MPU6050 state-machine logic against 10,000 iterations of synthetic hiking noise to determine the accuracy and false-positive rates of detection.

### Wokwi Firmware simulation
Located in `/simulations/wokwi`, these folders simulate the firmware and hardware interactions for each segment . 

* **WearableHikingTracker (User Segment):** ESP32-C3-Mini and MPU6050
* **CubesatHikingTracker (Space Segment):** Wemos S2 Mini, LoRa communication, and GNSS simulation
* **GroundStationHikingTracker (Ground Segment):** Data handling and MQTT relay

---

## Instructions for use

### Running the Python simulation
1.  Ensure **Python 3.x** environment is installed.
2.  Install the required libraries, if you haven't yet already:  
    `Numpy, Matplotlib`
3.  Navigate to the simulation directory and execute the script
4.  The script will output the statistical success rate and generate a visualization of the detection results

### Running the Wokwi simulation
The firmware simulations are provided as exported project files. To run them in a web browser:

1.  Go to **Wokwi.com**
2.  Select the **"Start from Scratch"** option and choose the ESP32 platform
3.  Use the **"Upload File"** feature in the Wokwi editor to upload the files contained within the relevant segment folder (e.g., `diagram.json`, `sketch.ino`, and `libraries.txt`)
4.  Click the **"Play"** button to initiate the hardware simulation

> **Note:** The Space and Ground segments require an active internet connection to communicate with the HiveMQ broker via the simulated WiFi bridge.
