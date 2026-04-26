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

| Title | Description | Links |
| ------------- | ------------- | ------------- |
|**rcl_logging_syslog** |  | [github](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://github.com/fujitatomoya/rcl_logging_syslog&ved=2ahUKEwjY16uqq9-MAxXFQ_EDHSVRCsUQjjh6BAgKEAE&usg=AOvVaw3Vi9GYtN_GbBUxQr6GsDxr) , [Discourse thread](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://discourse.ros.org/t/ros-2-log-system-meets-rsyslog-and-fluentbit/39280&ved=2ahUKEwjY16uqq9-MAxXFQ_EDHSVRCsUQFnoECCwQAQ&usg=AOvVaw1dRbfujPUXGrdwbKFdgGEK)|
|**diagnostic_remote_logging** | Package allows to log diagnostics data from ROS 2 systems directly to an InfluxDB. This enables long-term trend analysis, visualisation in Grafana, and makes it easier to monitor larger deployments across multiple systems. | [Discourse](https://discourse.ros.org/t/diagnostic-remote-logging-announcement/43696), [github](https://github.com/ros/diagnostics/tree/ros2-humble/diagnostic_remote_logging)|


## 🔍 Introspection, diagnostics, debugging
| Title | Description | Links |
| ------------- | ------------- | ------------- |
|**log_view**| ncurses text UI for viewing rosout logs. Receive, filter, and navigate through all published rosout log messages from within a terminal to support debugging a robot live. | [github](https://github.com/hatchbed/log_view) |
|**breadcrumb**|ROS2 graph static analysis tool that works with cake and clingwrap. | [github](https://github.com/greenforge-labs/breadcrumb) |
|**ros2_medkit**|Automotive-grade diagnostics for ROS 2 robots.| [github](https://github.com/selfpatch/ros2_medkit) |
|**Tf_tree_terminal** | Simple ROS 2 Python tool that prints the current TF tree to your terminal using a recursive "folder-like" structure. | [github](https://github.com/Tanneguydv/tf_tree_terminal) , [Discourse](https://discourse.openrobotics.org/t/tf-tree-terminal/51494) |
|**ros2_tracing** | Provides tracing instrumentation for the core ROS 2 packages | [github](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://github.com/ros2/ros2_tracing&ved=2ahUKEwjwl6uCk9-MAxXDQvEDHYv2HmQQjjh6BAgKEAE&usg=AOvVaw2YXUWIwf_nh0O9rkhi870h)|
|**PlotJuggler**|tool to visualize time series that is fast, powerful and intuitive.|[github](https://github.com/facontidavide/PlotJuggler)|
|**terminal_pcl_visualizer** | A high-performance, real-time 3D Point Cloud visualizer for your terminal. Built with ROS 2 and FTXUI, it allows you to visualize point clouds and control robots directly from your CLI with near-zero latency. |[github](https://github.com/nathanshankar/terminal_pcl_visualizer)|

## 🕹️ Control, monitoring

| Title | Description | Links |
| ------------- | ------------- | ------------- |
|**OpenTelemetry** | A production-grade integration library for instrumenting ROS2 (Robot Operating System 2) applications with OpenTelemetry distributed tracing and observability capabilities. | [github](https://github.com/szobov/ros-opentelemetry)|
|**R'DASH** | R’DASH (Robot Information Telemetry Transport Dashboard) is a cross-platform dashboard for real-time robot data visualization. | [github](https://github.com/vamsi-karnam/rdash)|
|**ros2plot** | terminal-based, real-time plotting tool for ROS2 topics | [github](https://github.com/jeminism/ros2plot)|
|**ROSboard** | ROS node that runs a web server on your robot. Run the node, point your web browser at http://your-robot-ip:8888/ and you get nice visualizations. | [github](https://github.com/dheera/rosboard)|
|**system_webview** | A real-time system monitoring dashboard for ROS 2. It provides a web-based UI that displays live CPU, memory, swap, and load average statistics, a scrollable /rosout log viewer, and interactive node and topic browsers | [github](https://github.com/namo-robotics/ros2_system_webview) |
|**TeleopXR**| Transform your VR/AR headset into a powerful, precise robot controller. |[github](https://github.com/qrafty-ai/teleop_xr)|
|**ros2_teleoperation** | ROS2 package implementing a teleoperation interface using QT. | [github](https://github.com/CDonosoK/ros2_teleoperation)|
|**Telegraf Resource Monitor** | A ROS 2 package that integrates Telegraf with ROS 2 to monitor system resources and publish them as ROS messages. | [github](https://github.com/Bart-van-Ingen/telegraf_resource_monitor)|
|**LGDXRobot Cloud** | Robot management system for Automated Guided Vehicles (AGVs), designed with a focus on flexibility and security. It can monitor the status of robots in real-time and manage automated tasks for transportation and logistics. | [github](https://github.com/yukaitung/lgdxrobot-cloud)|

## Networking, multi-robot communication

**ROS 2 Cross-Platform Network Fixer**

Automates the complex network configuration that makes ROS 2 DDS discovery work across WSL2, Docker, corporate/school networks, and multi-machine setups — without requiring you to read hours of DDS documentation.

[github](https://github.com/Krymorn/ros2_network_fixer)

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

**opencv_ros2**

OpenCV Applications with ROS2

[github](https://github.com/jeffreyttc/opencv_ros2)

**SAM3 ROS Wrapper**

ROS wrapper for SAM3 (Segment Anything Model 3), compatible with both ROS1 and ROS2.

[github](https://github.com/SanghyunPark01/sam3_ros_wrapper)


**dinov3_ros**

ROS 2 nodes for performing multiple vision tasks—such as object detection, semantic segmentation, and depth estimation—using Meta’s DINOv3 as the backbone.

[github](https://github.com/Raessan/dinov3_ros)

**ROS 2 YOLOs-CPP**

Production-grade, lifecycle-managed ROS 2 nodes for Object Detection, Segmentation, Pose, OBB, and Classification using YOLOs-CPP.

[github](https://github.com/Geekgineer/ros2_yolos_cpp)

**ROS 2 OpenCV Face Tracker: Zero-Latency Edge Perception**

A hyper-optimized, real-time computer vision node for ROS 2 that achieves zero-latency face tracking by bypassing standard middleware serialization bottlenecks.

[github](https://github.com/abinaabey2006)

**robovision_ros2**

Examples for an introduction to Robot Vision.

[github](https://github.com/ARTenshi/robovision_ros2/tree/main?tab=readme-ov-file)

**ros2-object-detection-pipeline**

ROS 2 Phone Camera Object Detection with YOLO (and Visualization)
[github](https://github.com/ShahazadAbdulla/ros2-object-detection-pipeline)

**yolo_ros**

ROS 2 wrap for YOLO models from Ultralytics to perform object detection and tracking, instance segmentation, human pose estimation and Oriented Bounding Box (OBB).


[github](https://github.com/mgonzs13/yolo_ros)

**ros2-tensorflow**

ROS2 wrapper for Tensorflow aloowing to load pretrained neural networks and perform inference through ROS2 interfaces.

[github](https://github.com/alsora/ros2-tensorflow)

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

**ROS2 Planning System**

Project whose objective is to provide Robotics developers with a reliable, simple, and efficient PDDL-based planning system.

[github](https://github.com/PlanSys2/ros2_planning_system)
[Wiki](https://plansys2.github.io/)

**apriltag_detector**

This ROS 2 node uses the AprilTag library to detect AprilTags in images and publish their pose, id and additional metadata.

[github](https://github.com/christianrauch/apriltag_ros)

**wayp_plan_tools**

Waypoint and planner tools for ROS 2 with minimal dependencies.

[github](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://github.com/jkk-research/wayp_plan_tools&ved=2ahUKEwil0rL0qt-MAxUCA9sEHY3PMqYQjjh6BAgiEAE&usg=AOvVaw3n9iQt_tYo-6B5lt9VyMsl)

**Open Navigation's Nav2 Complete Coverage**

Coverage Path Planning for ROS2 Robots. Extends the Fields2Cover (F2C) library, integrating its powerful coverage algorithms into ROS 2 & Nav2 as a modular and scalable Complete Coverage Task Server.

[github](https://github.com/open-navigation/opennav_coverage)

**FlexCloud**

modular tool for direct georeferencing and drift correction of point cloud maps. It enables the georeferencing of an existing point cloud map created only from inertial sensor data (e.g. LiDAR) by the use of the corresponding GNSS data.

[github](https://github.com/TUMFTM/FlexCloud)

**mogi_trajectory_server**

Trajectory visualization for ROS2 with pretty much the same functionality as hector_trajectory_server for ROS1

[github](https://github.com/MOGI-ROS/mogi_trajectory_server)

**collision_restraint**

ROS2 module to try to prohibit rectangular robots from driving into obstacles (PointCloud2 points) by restraining 2D twist command velocities.

[Discourse thread](https://discourse.ros.org/t/collision-restraint-a-ros2-package-to-stop-your-robot-driving-into-things/43328)

[github](https://github.com/AlexReimann/collision_restraint)


## 👜 Work with bag files

| Title | Description | Links |
| ------------- | ------------- | ------------- |
|**bag2mesh** | Powerful, standalone Python tool designed to convert ROS Bag files (specifically from RGB-D cameras like Intel RealSense) into high-quality 3D Meshes (.ply). | [github](https://github.com/alex9978/bag2mesh)|
|**ROS2 Bag Manager** | Simple, web-based tool for managing ROS2 bag files. Built with FastAPI and HTMX for a clean, responsive interface. | [github](https://github.com/ilisparrow/ros2bag_manager), [Discourse](https://discourse.openrobotics.org/t/announcement-ros2-bag-manager/51772)|
|**rospeek**|blazing-fast, cross-platform ROS 2 bag analyzer written in Rust.|[github](https://github.com/ktro2828/rosbag2peek)|
|**ros2_unbag** | A ROS 2 tool for exporting bags to human readable files. Supports pluggable export routines to handle any message type. | [github](https://github.com/ika-rwth-aachen/ros2_unbag/tree/main)|
|**ros2bag_tools** | Tool extensions for ros2bag cli | [github](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://github.com/AIT-Assistive-Autonomous-Systems/ros2bag_tools&ved=2ahUKEwiEt8X9qt-MAxX9QvEDHfCuMMIQjjh6BAgrEAE&usg=AOvVaw0xUwam50-vry_bime_V8us)|
|**rosbag-resurrector** | A pandas-like data analysis tool for ROS 2 (MCAP) bag files | [github](https://github.com/vikramnagashoka/rosbag-resurrector)|


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

**Conveyor Simulation ROS 2 Package**

ROS 2 package provides a simulation of a conveyor belt in Gazebo Harmonic with ROS 2 interface for control

[github](https://github.com/mzahana/conveyor_sim_ros2)

**URDF Architect**

State-of-the-art, web-based visual environment engineered for the seamless creation, manipulation, and export of Unified Robot Description Format (URDF) models.

[github](https://github.com/OpenLegged/URDF-Architect)


## 🖼️ Visualization, GUI

| Title | Description | Link |
| ------------- | --------- | ---------- | 
|**RQml** | Modern, QML-based robotics visualization and control toolbox for ROS 2! | [github](https://github.com/StefanFabian/rqml)|
|**Phantom Bridge** | fast WebRTC ROS2 Bridge written in C++ for real-time ROS2 data visualization and video streaming, teleoperation, human-robot interaction, and both local and remote robot monitoring. | [server github](https://github.com/PhantomCybernetics/phntm_bridge_server), [client github](https://github.com/PhantomCybernetics/phntm_bridge_client)|
|**bt_visualizer_pkg** | Simple and intuitive graphical tool to visualize behaviour tree XML files | [github](https://github.com/shivcc/bt_visualizer_pkg)|
|**urdf-viz** | Visualize URDF(Unified Robot Description Format) file. Viewer works on Windows/MacOS/Linux | [github](https://github.com/openrr/urdf-viz)|
|**rqt_frame_editor_plugin** | rqt_frame_editor_plugin allows for easily manipulating frames very quickly by dragging them, changing parents, relative positions etc. | [github](https://github.com/ipa320/rqt_frame_editor_plugin)|
|**ROSplat** | The Online ROS2-Based Gaussian Splatting-Enabled Visualizer | [github](https://github.com/shadygm/ROSplat), [Discourse](https://discourse.openrobotics.org/t/goodbye-rqt-hello-rqml-new-release/51697)|

## 🧮  Mathematics, numeric operations
**refx**

modern header-only C++ library designed for mobile robotics and navigation. Its core philosophy is to leverage the C++ type system to provide compile-time safety for all geometric and geodetic calculations

[github](https://github.com/mosaico-labs/refx)

## ⚙️ Useful tools

| Title | Description | Link |
| ------------- | --------- | ---------- | 
|**ros2_console_tools**| terminal-first toolbox for ROS 2 runtime inspection, visualization, and operator workflows. |[github](https://github.com/nilseuropa/ros2_console_tools)|
|**rostree**| Explore ROS 2 package dependencies from the command line (CLI, TUI, library). | [github](https://github.com/guilyx/rostree) |
|**synchros2** | synchros2 enables a different, at times simpler approach to ROS 2 programming, particularly for those that come with a ROS 1 background. | [github](https://github.com/bdaiinstitute/synchros2)|
|**robotdatapy** | Package designed to make accessing and manipulating robot/geometric data easy in Python. | [github](https://github.com/mbpeterson70/robotdatapy)|
|**fri - Fuzzy ROS2 Introspection** | script - wrapper around fzf to make it easier to use ROS2 | [github](https://github.com/xuyuan/fri)|
|**dynamic_reconfigure** | ROS 2 plugin that integrates with RViz2 to provide a GUI-based interface for dynamically reconfiguring ROS 2 node parameters. This tool allows users to list available nodes, inspect configurable parameters, and modify their values in real-time without restarting the nodes. | [github](https://github.com/shahjesal15/dynamic_reconfigure)|
|**sdf_to_urdf** | A simple ROS 2 tool to convert SDF files to URDF using urdfdom and sdformat_urdf. | [github](https://github.com/andreasBihlmaier/sdf_to_urdf)|
|**ros2mock** | Tool for mocking ROS 2 services and actions. | [github](https://github.com/taDachs/ros2mock)|
|**ROS 2 Launch GUI** | Package provides a Graphical User Interface for ROS 2 Launch System, which allows you to visually monitor every process started by your ROS 2 launch file. | [github](https://github.com/rolker/ros2launch_gui)|

**Mobile Sensor Bridge for ROS2**

Package provides a ROS2 node that streams sensor data directly from an Android smartphone, including JPEG camera streaming (up to 30 FPS), spatial pose tracking, and bidirectional audio communication


[github](https://github.com/VedantC2307/ros2-android-sensor-bridge)


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

**rplidar_ros2_driver**

This is a heavily refactored, fault-tolerant ROS 2 driver for Slamtec RPLIDAR. Designed with a Lifecycle State Machine and Thread-Safe Architecture

[github](https://github.com/frozenreboot/rplidar_ros2_driver)
[Discourse](https://discourse.openrobotics.org/t/announcement-rplidar-ros2-driver-a-modern-c-c-17-driver-with-lifecycle-support/51601)

**Qualcomm QRB-ROS**

Collection of ROS packages that accelerate common robotics tasks on their hardware platforms providing the features such as hardware acceleration, zero copy and AI inference. Most of them are supported on Ubuntu and can be simply installed by “apt install xxx“.

[Discourse](https://discourse.openrobotics.org/t/ros2-packages-based-on-qualcomm-devices/51245)
[github](https://github.com/qualcomm-qrb-ros)

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


**Copper Runtime & SDK**

Operating system for robots - build, run, and replay your entire robot deterministically.

[github](https://github.com/copper-project/copper-rs)

## 1️⃣ ROS1
**MiniROS cpp distribution**

standalone version of C++17 ROS1 client, which does not need boost, catkin or any complicated external library

[github](https://github.com/dkargin/miniroscpp)
