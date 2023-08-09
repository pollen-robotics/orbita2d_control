# Change Log

## Version 1.0 - Initial release
 
* Rust controller
    * all basic registers control (position, velocity, torque, PID, limits, etc.)
    * abstract communication layer (+ flipsky implementation) 
    * unit test in CI via fake motors
    * configuration file

* C-API library (via cbindgen)
* Python bindings (via maturin and cffi)
* ROS2 humble integration (URDF, ros2_control, hardware system interface)
