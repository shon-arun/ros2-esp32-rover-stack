# ros2-esp32-rover-stack

A distributed ROS 2 Visual-Inertial SLAM architecture bridging custom ESP32 hardware control, Raspberry Pi edge networking, and GPU-accelerated spatial mapping.

## Overview
This repository contains the full software stack for an autonomous, differential-drive rover. The system is designed to offload heavy Visual SLAM (vSLAM) and navigation algorithms to a dedicated base station GPU, while maintaining real-time, low-latency motor control and hardware safety lockouts on an embedded microcontroller.

## 🏗️ System Architecture

The architecture is split into three distinct compute tiers communicating over a shared network bridge.

### 1. The Real-Time Embedded Layer (ESP32-S3)
Handles strict-timing hardware interrupts, sensor polling, and closed-loop motor control.
* **IMU:** MPU6500 (Calibrated, hardware 21Hz DLPF, publishing raw data at 26Hz).
* **Odometry:** 4x Hardware-interrupt speed encoders for wheel tick tracking.
* **Safety Net:** VL53L0X Time-of-Flight sensor (angled downward) for immediate cliff/drop-off detection and emergency motor halting.
* **Bumper:** HC-SR04 ultrasonic sensor for close-proximity frontal collision prevention.
* **Firmware:** Written in C++ (PlatformIO) utilizing `micro_ros_platformio` to act as a native ROS 2 node.

### 2. The Edge Node (Raspberry Pi)
Acts as the rover's central nervous system, video streamer, and network bridge.
* **OS:** Ubuntu Server 24.04 LTS
* **ROS Version:** ROS 2 Jazzy
* **Sensors:** Raspberry Pi Camera module streaming video for vSLAM feature tracking.
* **Nodes:** Runs the `micro_ros_agent` over UDP, the Madgwick IMU filter, and the custom Tick-to-Odometry translation nodes.

### 3. The Base Station (Lenovo LOQ Laptop)
Handles all heavy compute, 3D mapping, and path planning over Wi-Fi.
* **OS:** Ubuntu 26.04 LTS
* **ROS Version:** ROS 2 Lyrical
* **Core Tasks:** * Sensor Fusion: Fusing IMU and wheel odometry via `robot_localization` (Extended Kalman Filter).
  * Mapping: Running Monocular-Inertial vSLAM (RTAB-Map / ORB-SLAM3) to establish scale and track visual features.
  * Navigation: Utilizing Nav2 Costmaps to project cliff-sensor data as lethal obstacles into the digital map.

## 🚀 Current Status & Roadmap
- [x] ESP32 Motor driver and hardware interrupts initialized.
- [x] MPU6500 deep calibration, bias removal, and covariance inflation.
- [x] Micro-ROS Wi-Fi transport and DDS network bridge established.
- [x] RViz 3D orientation tracking operational.
- [ ] Implement C++/Python Tick-to-Odometry node on the Raspberry Pi.
- [ ] Calibrate camera extrinsics and initiate visual feature tracking.
- [ ] Tune the Extended Kalman Filter (EKF) for autonomous waypoint navigation.
