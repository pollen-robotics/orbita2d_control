# Orbita2d rust control package

This package lets you easily control an Orbita2d actuator from Rust. It provides a high-level API to control the actuator, and a low-level API to communicate with the actuator.

It supports both serial and EtherCAT communication.

## Usage

For the flipsky serial communication:
```rust
let mut orbita2d = Orbita2dController::with_flipsky_serial(
    ("/dev/ttyUSB0", "/dev/ttyUSB1"),
    (30, 31),
    [0.0, 0.0],
    [1.0, 1.0],
    None,
)?;

orbita2d.enable_torque(true)?;

let t0 = std::time::Instant::now();

loop {
    let t = t0.elapsed().as_secs_f64();
    let roll =  30_f64.to_radians() * (2.0 * PI * 0.25 * t).sin();
    let target = [roll, 0.0];

    orbita2d.set_target_orientation(target)?;
    let pos = orbita2d.get_current_orientation()?;

    println!("target: {:?}, pos: {:?}", target, pos);

    std::thread::sleep(std::time::Duration::from_millis(10));
}
```

 ## Overview

 ### Control
 - [x] Torque ON/OFF
 - [x] Read the current 2D orientation (position/velocity/torque)
 - [x] Set the desired 2D orientation in radians
 - [x] Extra controller parameters (velocity limit, torque limit, PID gains)

 ### Communication
 - [x] Flipsky Serial communication
 - [ ] EtherCAT communication


## Full API

The full API can be seen using ```cargo doc```.