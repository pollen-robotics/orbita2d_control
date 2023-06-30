# Orbita2d actuator control libraries

## Overview

This repository contains all libraries required to control an Orbita2d actuator. It allows:
* to configure and test the actuator via a command-line or grapihcal tool
* to fully control the actuator using Python (only serial)
* access the full API (Rust package or C-API library)
* integrate the actuator in your ROS robot (URDF and ROS2 humble control hardware interface)

## Usage

### Command line tool
### Configuration tool
### Via Python

## Contents

This repository contains the following sub-packages:

### Rust packages for control

* [orbita2d_kinematics](orbita2d_kinematics/README.md): forward/inverse kinematics model (in Rust)
* [orbita2d_control](orbita2d_control/README.md): low-level communication (serial or ethercat) and control (in Rust)

### C-API library


## Related repositories

* Mechanical design: [orbita2d_mechanical]()
* Electronics design: [orbita2d_electronics]()
* Firmware: [orbita2d_firmware]()