# URDF/Xacro definition for Orbita2D

## Test example
- With a fake Orbita2d (which is different than the fake mode where the dummy ros2_controllers are used): ``ros2 launch orbita2d_description test.launch.py fake:=false config_file:=PATH_TO_YOUR_WORKSPACE/src/orbita2d_control/orbita2d_description/config/fake.yaml`` (change config file for your real system)
- Set torque on: ``ros2 topic pub /forward_torque_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0 ,1.0]"``
- Set target position: ``ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0 ,1.0]"``
- Set pids: ``ros2 topic pub /forward_pid_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0 ,1.0,1.0,1.0,1.0,1.0]"`` (P_a, I_a, D_a, P_b, I_b, D_b)
- Set torque limit: ``ros2 topic pub /forward_torque_limit_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0 ,1.0]"`` (raw motor torque limit)
- Set speed limit: ``ros2 topic pub /forward_speed_limit_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0 ,1.0]"`` (raw motor speed limit)
