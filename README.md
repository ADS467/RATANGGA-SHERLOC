# *RATANGGA* `SHER-LOC`


> ### Hardware Design
> The `/cad` directory contains the following files:
>  - Locator (user segment) CAD design for the wearable device
>  -  Cubesat (space segment) CAD design for the satellite

> ### Simulations
> The `/Simulations` directory contains the following files:
> > **1. Python simulation**
> >  `/Simulation/FallDetectionMachineSimulator.py`
> >  Fall detection machine testing script, which tests the simulation of the IMU process against synthetic hiking data
> > **How to run**
> > - Make sure the Python environment is installed with these libraries: `Numpy, Matplotlib`
> > - Execute the script via terminal / opening the file. Can be configured by changing `G_TRIGGER`, `SAMPLE_RATE_MS`, `ITERATIONS`
> 
> > **2. Wokwi simulations**
> >  `/Simulation/Wokwi Projects`
> >  Exported files from Wokwi simulating the firmware and hardware interactions in a controlled environment. The simulations are divided into 3 segments:
> > - `/WearableHikingTracker`
> > - `/CubesatHikingTracker` 
> > - `/GroundStationHiker` 
> > 
> > **How to run**
> > - Make sure all the folders are installed (`/WearableHikingTracker`,`/CubesatHikingTracker`,`/GroundStationHiker` )
> > - Select the "Start from scratch" button and choose the ESP32 platform. *Note: Create new projects for each folder*
> > - Select the "Upload File" feature to upload the files in each folder (`diagram.json`, `sketch.ino`, `libraries.txt`).
> > - Run the simulation. To simulate LoRa handling, run all the projects at once 

> ### Project Files
> The `/Project Files` directory contains the following files:
> - Cubesat `Cubesat.ino`: Firmware for the Cubesat
> - Wearable Device `WearableDevice.ino`: Firmware for the Wearable Device
> - Ground Station `GroundStation.ino`: Firmware for the Ground Station
> 
> **How to use**
> - Download the `.ino` files and open them as a sketch in [Arduino IDE](https://www.arduino.cc/en/software/)
> - Verify the code 



