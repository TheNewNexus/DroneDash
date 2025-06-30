
# DroneDash: Autonomous Ground Vehicle

**DroneDash** is an autonomous unmanned ground vehicle (UGV) developed for the Raytheon Engineering Competition. It uses the MAVLink protocol to interface a Pixhawk 6X flight controller with a Jetson Orin Nano onboard computer. The vehicle is capable of GPS-based navigation, real-time object tracking, and mission-based autonomy in outdoor environments.

---

## Table of Contents

- [Overview](#overview)
- [Hardware Components](#hardware-components)
- [Software Architecture](#software-architecture)
- [Core Features](#core-features)
- [Repository Structure](#repository-structure)
- [Usage](#usage)
- [Dependencies](#dependencies)
- [License](#license)

---

## Overview

DroneDash is built on a rugged RC car platform and is designed to perform fully autonomous missions. It integrates GPS, computer vision, servo control, and obstacle avoidance to enable reliable navigation in complex outdoor settings.

The project is built using:
- ArduRover firmware (PX4)
- MAVLink protocol for communication
- Python-based control scripts
- Intel RealSense depth camera for vision-based obstacle detection
- Optional ROS 2 integration for modular expansion

---

## Hardware Components

| Component               | Description                                 |
|------------------------|---------------------------------------------|
| RC Car Chassis         | Off-road capable vehicle base               |
| Pixhawk 6X             | Autopilot running PX4 (ArduRover mode)      |
| Jetson Orin Nano       | Onboard compute unit for autonomy tasks     |
| Intel RealSense D435   | Depth camera for obstacle detection         |
| u-blox NEO-F9P         | RTK-capable GPS module                      |
| SiK Radio Telemetry    | Long-range telemetry module                 |
| FlySky Transmitter     | Manual override and remote control          |
| 4S LiPo Battery        | Power supply for vehicle and systems        |

---

## Software Architecture

- **Pixhawk 6X with PX4 ArduRover**: Provides low-level vehicle control and waypoint execution.
- **Jetson Orin Nano**: Executes high-level autonomy logic, including vision-based tracking, mission control, and MAVLink command publishing.
- **Intel RealSense D435**: Supplies real-time 3D depth data for navigation and obstacle avoidance.
- **MAVLink**: Used for all communication between the onboard computer and Pixhawk.
- **Optional ROS 2 Layer**: Can be used for extending functionality, modular processing, and development scalability.

---

## Core Features

- Autonomous waypoint navigation via GPS
- Object tracking and ArUco marker following
- Real-time obstacle detection using depth camera
- Velocity and servo control through MAVLink
- Remote telemetry monitoring and logging
- Manual override capability with RC transmitter

---

## Repository Structure

```

DroneDash/
│
├── autonomy/
│   ├── aruco\_follow\.py
│   ├── follow\_me.py
│   ├── ground\_followme.py
│   ├── non\_gps\_movement.py
│   ├── velocity\_based\_movement.py
│   ├── radio\_controlled\_fluxgrip.py
│   ├── swivel\_search.py
│   └── d4xx\_to\_mavlink.py
│
├── gps\_navigation/
│   ├── gps\_arauco.py
│   ├── gps\_mission.py
│   ├── gps\_print.py
│   ├── gps\_scan\_aruco.py
│   ├── send\_gps.py
│   ├── get\_gps.py
│   └── map\_picker.py
│
├── mission\_control/
│   ├── drone\_to\_rover\_mission.py
│   ├── mission.py
│   ├── navigate.py
│   └── send\_velocity\_movement.py
│
├── servo\_control/
│   ├── servo.py
│   ├── servo1.py
│   ├── servo2.py
│   └── servo3.py
│
├── throttle\_control/
│   ├── throttle.py
│   ├── throttle2.py
│   ├── throttle3.py
│   └── velocity\_test.py
│
├── tests/
│   ├── camera\_test.py
│   ├── mavtest.py
│   ├── mavtester.py
│   ├── manual\_mode\_test1.py
│   └── telem.py
│
├── config/
│   ├── mav.param
│   ├── \*.parm
│   ├── d4xx-default.json
│
├── logs/
│   ├── mav.tlog
│   ├── mav.tlog.raw
│   └── \*.tlog (session logs)
│
└── README.md

````

---

## Usage

### 1. Connect Hardware
Ensure the Pixhawk is powered and connected to the Jetson Orin Nano via USB or serial. Start the vehicle and ensure telemetry and GPS lock are functioning.

### 2. Launch Mission Example
```bash
python3 gps_mission.py
````

### 3. Start ArUco Marker Tracking

```bash
python3 aruco_follow.py
```

### 4. Send Velocity Commands Directly

```bash
python3 send_velocity_movement.py --vx 0.5 --vy 0 --vz 0
```

---

## Dependencies

Install system dependencies:

```bash
sudo apt update
sudo apt install python3-pip
```

Install required Python packages:

```bash
pip3 install pymavlink opencv-python numpy pyrealsense2
```

Optional:

* `pyserial`
* `matplotlib`
* `pyyaml`

---

## License

This project is released under the MIT License.
© 2025 DroneDash Team – Raytheon Engineering Competition

---

## Acknowledgements

* ArduPilot and PX4 development communities
* MAVLink protocol and documentation contributors
* NVIDIA for Jetson support and developer tools
* Intel RealSense SDK and open-source examples

```

