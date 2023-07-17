use orbita2d_kinematics::Orbita2dKinematicsModel;

#[no_mangle]
pub extern "C" fn orbita2d_forward_position(
    ratio_a: f64,
    ratio_b: f64,
    angle_a: f64,
    angle_b: f64,
    ring: &mut f64,
    center: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_forward_kinematics([angle_a, angle_b]);

    *ring = res[0];
    *center = res[1];

    0
}

#[no_mangle]
pub extern "C" fn orbita2d_forward_velocity(
    ratio_a: f64,
    ratio_b: f64,
    velocity_a: f64,
    velocity_b: f64,
    ring_velocity: &mut f64,
    center_velocity: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_output_velocity([velocity_a, velocity_b]);

    *ring_velocity = res[0];
    *center_velocity = res[1];

    0
}

#[no_mangle]
pub extern "C" fn orbita2d_forward_torque(
    ratio_a: f64,
    ratio_b: f64,
    torque_a: f64,
    torque_b: f64,
    ring_torque: &mut f64,
    center_torque: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_output_torque([torque_a, torque_b]);

    *ring_torque = res[0];
    *center_torque = res[1];

    0
}

#[no_mangle]
pub extern "C" fn orbita2d_inverse_position(
    ratio_a: f64,
    ratio_b: f64,
    ring: f64,
    center: f64,
    angle_a: &mut f64,
    angle_b: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_inverse_kinematics([ring, center]);

    *angle_a = res[0];
    *angle_b = res[1];

    0
}

#[no_mangle]
pub extern "C" fn orbita2d_inverse_velocity(
    ratio_a: f64,
    ratio_b: f64,
    ring_velocity: f64,
    center_velocity: f64,
    velocity_a: &mut f64,
    velocity_b: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_input_velocity([ring_velocity, center_velocity]);

    *velocity_a = res[0];
    *velocity_b = res[1];

    0
}

#[no_mangle]
pub extern "C" fn orbita2d_inverse_torque(
    ratio_a: f64,
    ratio_b: f64,
    ring_torque: f64,
    center_torque: f64,
    torque_a: &mut f64,
    torque_b: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_input_torque([ring_torque, center_torque]);

    *torque_a = res[0];
    *torque_b = res[1];

    0
}
