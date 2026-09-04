# Awesome ROS packages


## General 


## 📞 Messages, topics

| Title | Description | Link |
| ------------- | --------- | ---------- |
|**topic_tools** | Tools for directing, throttling, selecting, and otherwise manipulating ROS 2 topics at a meta-level. | [github](https://github.com/ros-tooling/topic_tools)|
|**flex_sync** | ROS2 package with headers-only library that implements a message synchronization filter | [github](https://github.com/ros-misc-utilities/flex_sync)|
|**proto2ros** | Tool providing an interoperability bridge between Protobuf messages and ROS messages. proto2ros generates ROS 2 message definitions and bi-directional conversion code directly from .proto files. | [Discourse](https://discourse.openrobotics.org/t/easier-protobuf-and-ros-2-integration/51712), [github](https://github.com/bdaiinstitute/proto2ros)|

## Nodes, services

**ros2systemd**

A ROS2 command extension for managing ROS2 launches and nodes as systemd services.

[github](https://github.com/jerry73204/ros2systemd)

## 🔄 Transformations 
**angles**

This package provides a set of simple math utilities to work with angles.

[github](https://github.com/ros/angles)


## ✍️ Logging

[Logging tools](<Logging.md>)

## 🔍 Introspection, diagnostics, debugging

[List of Introspection, diagnostics, debugging related tools](<Introspect, diagnostics and debug.md>)


## 🕹️ Control, monitoring

[Control and monitoring tools](<Control, monitoring.md>)

## Networking, multi-robot communication

**ROS 2 Cross-Platform Network Fixer**

Automates the complex network configuration that makes ROS 2 DDS discovery work across WSL2, Docker, corporate/school networks, and multi-machine setups — without requiring you to read hours of DDS documentation.

[github](https://github.com/Krymorn/ros2_network_fixer)

**RoboShield**

Embedded, lightweight Network Intrusion Detection System (NIDS) designed specifically for ROS 2 (RTPS/DDS) networks. It runs out-of-band on dedicated network appliances (such as the NanoPi R2S) to passively monitor, detect, and alert on cyber-physical threats without requiring modifications to the robot's onboard firmware or ROS 2 software stack.

[github](https://github.com/Amin-Ahmed-G/robotshield)

## 🛡️ Security, safety 

**ROS 2 Kinematic Guard**

Lightweight safety middleware for ROS 2 mobile robots.

[github](https://github.com/ZC502/ros2_kinematic_guard)

## 👁️ Perception 

**Surround Vision for ROS 2**

A surround-view visualisation stack for ROS 2 Jazzy. The package fuses four fisheye cameras, renders a calibrated top-down view, and overlays a textured 3D vehicle model.

[github](https://github.com/JeyP4/SurroundVisionROS2)

**Focus Peaking ROS2**

ROS 2 package assists in manually focusing lenses on machine vision cameras and evaluating image sharpness. It highlights sharp edges directly in the image display, allowing users to visually determine focus quality and identify which parts of the scene are in focus.

[demo video](https://www.youtube.com/watch?v=Wy3PcOUVC_Y)
[github](https://github.com/MRo47/ros2_focus_peaking)

**mola_lidar_odometry**

[github](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://github.com/MOLAorg/mola_lidar_odometry&ved=2ahUKEwi7otnikt-MAxU6RfEDHbPELVEQjjh6BAgWEAE&usg=AOvVaw1Q7O5aQivlBnnsXEh6md6o)

**GenZ-ICP**

A Robust LiDAR Odometry ROS 2 package

[github](https://github.com/cocel-postech/genz-icp)

**omnivision**

ROS2 package for fusing 360° equirectangular images with 3D LiDAR data.

[github](https://github.com/BlaineKTMO/omnivision)

**multisensor_calibration**

Universal calibration toolbox for assisted, target-based multi-sensor calibration. It provides a variety of methods and applications to calibrate complex multi-sensor systems (Camera-LiDAR, LiDAR-LiDAR, etc).

[github](https://github.com/FraunhoferIOSB/multisensor_calibration)

**ros2_camera_lidar_fusion**

[Discussion on discourse](https://discourse.ros.org/t/ros-2-camera-lidar-fusion-package-released/41550)

[github](https://github.com/CDonosoK/ros2_camera_lidar_fusion)


## 📷 Computer Vision

[Computer Vision tools](<Computer Vision.md>)

## ☁️ Point clouds

**POLKA - Multi-LiDAR fusion node for ROS 2**

Multi-LiDAR fusion node for ROS 2 that merges any mix of PointCloud2 and LaserScan sources into a unified output, with optional CUDA GPU acceleration.

[github](https://github.com/Pana1v/polka)

**PCL Point Cloud Processing Pipeline**

A configurable C++ library for point cloud processing built on top of the Point Cloud Library (PCL).

[github](https://github.com/Muhammad540/pcl-processing-pipeline)

**GSeg3D**

High-Precision Grid-Based Ground Segmentation for Safety-Critical Autonomous Driving and Robotics Applications.

[github](https://github.com/dfki-ric/ground_segmentation)


**Perception Vault**

Comprehensive and extensible perception framework designed to handle the full spectrum of sensor data required in modern robotics and autonomous systems. It provides such nodes like Camera RTSP Streamer, LiDAR PointCloud Filters, Lidar Euclidean Cluster etc.

[github](https://github.com/peakyquest/perception_vault)

**pointcloud_concatenate_ros2**

Package provides a node which can be used for concatenating several pointclouds into one.

[github](https://github.com/atinfinity/pointcloud_concatenate_ros2)


**pcdet_ros2**
ROS 2 Wrapper for OpenPCDet (LIDAR-based 3D Object detection)

[github](https://github.com/pradhanshrijal/pcdet_ros2)

**pointcloud_to_grid**

converts sensor_msgs/PointCloud2 LIDAR data to nav_msgs/OccupancyGrid 2D map data based on intensity and / or height.

[github](https://github.com/jkk-research/pointcloud_to_grid)


**ros2_pcl_segmentation**

ROS2 Package for point cloud segmentation using PCL library. This repository contains multiple segmentation algorithms for point clouds.

[github](https://github.com/CDonosoK/ros2_pcl_segmentation)


## 🗺️ Localization, navigation 

[Localization, navigation](<Localization, navigation.md>)


## 👜 Work with bag files

[Localization, navigation](<Work with bag files.md>)


## 👱🤖 Human-Robot Interaction 

**Agent ROS Bridge**

Universal ROS1/ROS2 bridge for AI agents to control robots and embodied intelligence systems. It speaks WebSocket, MQTT, and gRPC on the agent side, and ROS1/ROS2 on the robot side — with JWT auth, agent memory, safety confirmation, and fleet orchestration built in.

[github](https://github.com/webthree549-bot/agent-ros-bridge)

**ROSGPT**

ChatGPT Interface for ROS2 for Human-Robot Interaction. Useful package if you are interested in integrating LLM in your robotics projects.

[github](https://github.com/aniskoubaa/rosgpt)

**ROS2 Clean Architecture Environment**

Extensive set of Claude Code skills and rules for robotics engineers working with ROS 2. It has a set of rules and skills on architecture, node communication, TF transforms, launch configuration, messaging patterns, and best practices to build robotics systems.

[github](https://github.com/harunkurtdev/ros2-claude-code-template/)

**RosClaw**

RosClaw connects OpenClaw to ROS2 (the Robot Operating System) through an intelligent plugin layer

[github](https://github.com/PlaiPin/rosclaw)

## 🎲 Robot simulation,modelling

**sw2robot**

SolidWorks → robot (URDF) converter. Turn a SolidWorks assembly into a URDF, then clean it up in the browser.

[github](https://github.com/jsk-ros-pkg/solidworks_urdf_exporter2)

**Conveyor Simulation ROS 2 Package**

ROS 2 package provides a simulation of a conveyor belt in Gazebo Harmonic with ROS 2 interface for control

[github](https://github.com/mzahana/conveyor_sim_ros2)

**URDF Architect**

State-of-the-art, web-based visual environment engineered for the seamless creation, manipulation, and export of Unified Robot Description Format (URDF) models.

[github](https://github.com/OpenLegged/URDF-Architect)


## 🖼️ Visualization, GUI

[List of Visualization and GUI related tools](<Visualization and GUI.md>)

## 🧮  Mathematics, numeric operations
**refx**

modern header-only C++ library designed for mobile robotics and navigation. Its core philosophy is to leverage the C++ type system to provide compile-time safety for all geometric and geodetic calculations

[github](https://github.com/mosaico-labs/refx)

## ⚙️ Useful tools

[Useful tools](<Useful tools.md>)

## >_  Terminal based tools
| Title | Description | Link |
| ------------- | --------- | ---------- |
|**TermViz2**| A terminal-based ROS 2 visualizer. Renders maps, laser scans, point clouds, poses, paths, images, and markers directly in the terminal using half-block characters | [github](https://github.com/matijazigic/termviz2) |


## 👷 Development tools

| Title | Description | Link |
| ------------- | --------- | ---------- | 
|**ROS Dev Toolkit**|VS Code extension for ROS development that keeps package creation, node/launch execution, and runtime graph inspection in one place.|[github](https://github.com/BogdanTNT/ROS_vscode_extension)|
| **ROS 2 Project Builder** | CLI-based tool that automates the creation of ROS 2 packages and node structures. |[github](https://github.com/Madwesh-india/ros2-project-builder)|
|**launch_graph** | ROS 2 tool that visualizes the structure of your launch files. It recursively finds included launch files and node actions, then generates a visual graph using Graphviz.|[github](https://github.com/ijnek/launch_graph)|
|**rsl**|RSL is a collection of C++17 utilities for ROS projects.|[github](https://github.com/PickNikRobotics/RSL)|
|**ROS 2 Environment Manager for VS Code**|Visual Studio Code extension that helps you manage isolated ROS2 environments with ease — start, stop, switch, and interact with them directly from your editor.|[github](https://github.com/SakshayMahna/ros2env)|
|**ament_ruff**|Python linting and formatting based on ruff with ament integration.|[github](https://github.com/swri-robotics/ament_ruff)|


## ✅ Testing 

**ros2_test_compose**

Generate ROS 2 launch_test integration test suites from YAML — no boilerplate Python to write or maintain

[github](https://github.com/will-44/ros2_test_compose)



**rtest**

ROS 2 Unit Tests Framework.

[github](https://github.com/Beam-and-Spyrosoft/rtest)

## Language models

**ROS2 RAG**

ROS2 lifecycle node implementing a RAG (Retrieval-Augmented Generation) system

[github](https://github.com/aitor-ibarguren/ros2_rag)


**ROS MCP Server 🧠⇄🤖**

Connect AI models like Claude & GPT with robots using MCP and ROS.

[github](https://github.com/robotmcp/ros-mcp-server)

**Nav2 MCP Server**

An MCP (Model Context Protocol) server that provides tools and resources to control and monitor robots using Nav2.

[github](https://github.com/ajtudela/nav2_mcp_server)

## 🔀 Async / threading

**ICEY**

New client API for modern asynchronous programming in the Robot Operating System (ROS) 2. It uses C++20 coroutines with async/await syntax for service calls and TF lookups.

[github](https://github.com/iv461/icey)

## 🏭 Industrial robots

**reductstore_agent**

ROS 2 node that records selected topics into ReductStore, a high-performance storage and streaming solution. ReductStore is an ELT-based system for robotics and industrial IoT data acquisition.

[github](https://github.com/reductstore/reductstore_agent)

**ROS 2: Sim-to-Real Robot Control**

repository provides ready-to-use ROS2 (Humble) packages to execute simple programs and sequences and control different Industrial and Collaborative Robots using ROS 2.

[github](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl)

## 🔌 Serial communication, microcontrollers, hardware drivers

**Lenna Stereo Camera**

Full software stack for running a dual IMX219-83 stereo camera module on an NVIDIA Jetson Orin Nano. It covers everything from raw dual-CSI capture to stereo calibration, disparity/depth estimation, IMU (ICM-20948) integration, and ROS 2 (Humble) nodes for publishing synchronized stereo images, camera info, IMU data, and computed depth.

[github]([https://github.com/frozenreboot/rplidar_ros2_driver](https://github.com/Lenna-Robotics-Research-Lab/Lenna-Stereo-Camera))

**rplidar_ros2_driver**

This is a heavily refactored, fault-tolerant ROS 2 driver for Slamtec RPLIDAR. Designed with a Lifecycle State Machine and Thread-Safe Architecture

[github](https://github.com/frozenreboot/rplidar_ros2_driver)
[Discourse](https://discourse.openrobotics.org/t/announcement-rplidar-ros2-driver-a-modern-c-c-17-driver-with-lifecycle-support/51601)

**Qualcomm QRB-ROS**

Collection of ROS packages that accelerate common robotics tasks on their hardware platforms providing the features such as hardware acceleration, zero copy and AI inference. Most of them are supported on Ubuntu and can be simply installed by “apt install xxx“.

[Discourse](https://discourse.openrobotics.org/t/ros2-packages-based-on-qualcomm-devices/51245)
[github](https://github.com/qualcomm-qrb-ros)

**oxide_gnss**

A Rust-based ROS 2 GNSS driver for u-blox receivers (ZED-F9P focus) with an integrated NTRIP client and optional integrity monitoring.

[github](https://github.com/greenforge-labs/oxide_gnss)

**Servo Control Interface for ROS 2**

A comprehensive, low-latency interface for controlling a physical servo motor via an Arduino Serial connection within the ROS 2 ecosystem.

[github](https://github.com/dilip-2006/servo)

**Crosstalk**

small single-header C++ 17 library to facilitate communication between microcontrollers and host computers over serial connections.

[github](https://github.com/StefanFabian/crosstalk)

**ros2_imu**

ESP32 based IMU node for ROS2 using Micro ROS.

[github](https://github.com/syedmohiuddinzia/ros2_imu)

## Packages for different languages
**jros2**

A ROS 2 library for Java. Uses Fast-DDS middleware. Fully compatible with other supported ROS 2 middlewares.

[github](https://github.com/vovaekb/awesome_ros_packages_and_tools)

**rclgo**

ROS2 client library Golang wrapper.

[github](https://github.com/merlindrones/rclgo)

**swift-ros2**

Native Swift client library for ROS 2. Publishes and subscribes over Zenoh (via zenoh-pico) or DDS (via CycloneDDS) without a bridge, without pulling in the full ROS 2 stack.

[github](https://github.com/youtalk/swift-ros2)


**rclnodejs**

Node.js client library for ROS 2 that provides comprehensive JavaScript and TypeScript APIs for developing ROS 2 solutions.

[github](https://github.com/RobotWebTools/rclnodejs)

**tf2_rs**

tf2_rs provides Rust bindings for a focused subset of ROS 2 TF2.

[github](https://github.com/olingo99/tf2_rs)

**Copper Runtime & SDK**

Operating system for robots - build, run, and replay your entire robot deterministically.

[github](https://github.com/copper-project/copper-rs)

## 1️⃣ ROS1
**MiniROS cpp distribution**

standalone version of C++17 ROS1 client, which does not need boost, catkin or any complicated external library

[github](https://github.com/dkargin/miniroscpp)
