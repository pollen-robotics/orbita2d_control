# Orbita2d actuator control libraries

## Overview

This repository contains all libraries required to control an Orbita2d actuator. It allows:
* to fully control the actuator using Python
* access the full API (Rust package or C-API library)
* integrate the actuator in your ROS robot (URDF and ROS2 humble control hardware interface)

## Model definition

The actuator is defined by the following parameters:
* two motors ($A$ and $B$) with their respective reduction ratio ($r_{A}$ and $r_{B}$). **When not specified, they are always used in this order as argument or results.** Yet, you should never nead to control them directly.
* the output axes ($ring$ and $center$). **When not specified, they are always given in that order either as argument or results.**

Please refer to the following schema for more details:

![orbita2d_schema](./static/orbita2d-schema.png)

## Usage

### Via Rust

See [orbita2d_control](orbita2d_controller) for more details.

### Via Python

See [orbita2d python bindings](orbita2d_c_api/python/) for more details.

### ROS2 integration

#### Setup

* To integrate into a ROS2 workspace, simply clone this repository into the src/ directory of your workspace.
* Make sure to use "patched version" of cargo-ament-built (see [this PR](https://github.com/ros2-rust/cargo-ament-build/pull/3) for more details)
* Update colcon-cargo with: `python3 -m pip install --upgrade --force-reinstall git+https://github.com/pollen-robotics/colcon-cargo.git`

#### Build the hwi

 * `colcon build --symlink-install --packages-up-to orbita2d_system_hwi`

 #### Build the description

  * `colcon build --symlink-install --packages-up-to orbita2d_description`

* See the [orbita2d_description](orbita2d_description/README.md) package for more details.


## Contents

This repository contains the following sub-packages:

### Rust packages for control

* [orbita2d_kinematics](orbita2d_kinematics/README.md): forward/inverse kinematics model (in Rust)
* [orbita2d_control](orbita2d_control/README.md): low-level communication (serial or ethercat) and control (in Rust)

### C-API library and Python bindings

* [orbita2d c_api](orbita2d_c_api/README.md): plain C-API library to control the actuator
* [orbita2d python bindings](orbita2d_c_api/python/README.md): Python bindings for the C-API library


### Testing

* Unit test and doc-test: `cargo test`
* Hardware tests:
  - torque on/off: `cargo run --bin test_torque -- --config $(PWD)/orbita2d_controller/config/left_shoulder_flipsky.yaml`
  - velocity limit: `cargo run --bin test_velocity -- --config $(PWD)/orbita2d_controller/config/left_shoulder_flipsky.yaml`
  - torque limit: `cargo run --bin test_torque_limit -- --config $(PWD)/orbita2d_controller/config/left_shoulder_flipsky.yaml`

## Changelog

See [CHANGELOG.md](CHANGELOG.md)

## Related repositories

### Flipsky design

* [Mechanical design](https://cad.onshape.com/documents/d108475d689e47a5561e996c/w/1d6ae1891c12354f3ac85124/e/f54fa638e232e8705b683608)
* [Electronics design](https://github.com/pollen-robotics/orbita2d_elec)
* [Firmware](https://github.com/pollen-robotics/firmware_Orbita2Dofs/tree/Flipsky_FSESC6.7-pro)
