# Drone Hardware Upgrade Project - ELEC491_TL101
This is the official repository of TL101 of ELEC 491 Capstone Course of UBC.
This project is in collaboration with ICON Lab @ Columbia

# Autonomous Drone Navigation System – VINS-Fusion + FUEL
<img width="800" height="600" alt="drone" src="https://github.com/user-attachments/assets/eeb8d346-e71f-4aec-9329-7cb6d21e3310" />

## Overview
This repository contains the code and configurations for a **custom-built autonomous drone navigation system** using **VINS-Fusion** for visual–inertial SLAM and **FUEL** for frontier-based path planning.  
Developed as part of a capstone project at the University of British Columbia, the system was designed to replicate an **unmanned HVAC inspection drone** on a smaller, custom-built base frame.  
The platform operates in GPS-denied environments by leveraging ROS-based remote computation over a LAN.

> **Note:** An experimental branch using **ORB-SLAM3** is available on the `orbslam3` branch.  
> This version replaces the D435i depth feed with a **monocular RGB camera**.  
> The RGB images are processed using **Depth-Anything-V2** to generate depth maps,  
> which are then combined with the original RGB frames and fed into ORB-SLAM3 in RGB-D mode.

## Hardware Framework
The drone is a **custom frame** equipped with:
- **Battery**
- **PX4-compatible flight controller**
- **Electronic Speed Controllers (ESCs)**
- **Brushless motors**
- **Onboard IMU**
- **Raspberry Pi 4 (8GB)**
- **Intel RealSense D435i** (RGB-D camera with IMU)
- **Additional RGB-D camera** (optional)

## System Data Flow
1. **Onboard Processing (Raspberry Pi 4)**  
   - Runs **ROS Master**.  
   - Publishes IMU and D435i RGB-D data over LAN via **remote ROS**.

2. **Offboard Processing (Client Computer)**  
   - Subscribes to data from Raspberry Pi.  
   - Runs **VINS-Fusion** to perform visual–inertial SLAM and estimate pose.  
   - Combines pose with depth data from D435i and sends it into **FUEL**.  
   - **FUEL** executes frontier-based path planning and outputs navigation commands.

3. **Command Feedback Loop**  
   - FUEL’s output is sent back to the Raspberry Pi via ROS.  
   - **MAVROS** converts navigation commands into PX4 flight controller commands.  
   - PX4 adjusts motor outputs through ESCs to achieve autonomous movement.

## Key Features
- **SLAM Core:** VINS-Fusion for tightly coupled visual–inertial odometry.
- **Autonomous Exploration:** FUEL frontier-based path planning for exploring unknown spaces.
- **Remote ROS Networking:** Offloads SLAM and planning to a more powerful ground computer.
- **PX4 Integration:** MAVROS-based control of flight operations.
- **Custom Drone Frame:** Tailored to replicate HVAC inspection drone operation on a smaller scale.

## System Architecture
<img width="1600" height="595" alt="system architecture" src="https://github.com/user-attachments/assets/93a73c96-f512-4266-9e5a-fdfcdfeb3a4c" />


## Hardware Requirements
- Custom drone frame (PX4-compatible)
- Raspberry Pi 4 (8GB RAM)
- Intel RealSense D435i
- PX4 flight controller
- Electronic Speed Controllers (ESCs)
- Brushless motors
- LiPo battery
- Optional: Additional RGB-D camera
- Wi-Fi router (LAN connection between Pi and client computer)

## Software Requirements
- **Ubuntu 20.04** (on both Pi and client computer)
- **ROS Noetic**
- **PX4 Autopilot** & **MAVROS**
- **VINS-Fusion**
- **FUEL** (Frontier-based Exploration Using LeGO-LOAM or compatible mapping)
- (Optional) **Depth-Anything-V2** for enhanced depth perception

## Installation

1. **Clone this repository**
```bash
git clone https://github.com/raeditio/ELEC491_TL101
cd ELEC491_TL101
```

2. **Install Dependencies and Build**

```bash
cd icon_drone
. ./build_icondrone.sh
```
> **Note:** You will have to install librealsense on the Raspberry Pi by unpacking the source code. Follow the official guide: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

## Usage
Run the server.sh on the Raspberry Pi and the Client.sh on the Remote Computer.
```bash
cd shfiles
```
```bash
source server.sh
```
```bash
source client.sh
```
Refer to the User Manual for detailed use:
https://github.com/raeditio/ELEC491_TL101/blob/main/TL101%20-%20Drone%20User%20manual.pdf

**For ORB-SLAM3**
git checkout feature/MonoNav

## Contributors
- **Ryan Jung** ([raeditio](https://github.com/raeditio))- Lead Project Organizer, Software Lead
- **Aunark Singh** ([ArunarkSingh](https://github.com/ArunarkSingh))- Lead Administrator & Treasurer, Network Engineer
- **Peter Kim** ([vento277](https://github.com/vento277))- Lead Data & Record Tracking, Power Systems Design
- **Nabiha Khan** ([nk11235](https://github.com/nk11235))- Lead Communication, Simulation Design
- **Spencer Tipold**- Lead Lab & Validation, Hardware Design
- **Rylan-Bowen Colthurst**- Lead Client Support, Hardware Design
 
## Acknowledgments
This project builds upon the base code and framework provided by **ICON Lab @ Columbia** as part of the UBC ELEC 491 Capstone Course.  
The initial source was derived from the **[Fast-Drone-250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)** project and adapted for our custom hardware and navigation pipeline.  
We have since extended the system with monocamera-based RGB-D SLAM using **[ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)** and **[Depth-Anything-V2](https://github.com/DepthAnything/Depth-Anything-V2)**.

## License
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.
