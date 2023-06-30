use orbita2d_kinematics::Orbita2dKinematicsModel;

#[no_mangle]
pub extern "C" fn forward_position(
    ratio_a: f64,
    ratio_b: f64,
    angle_a: f64,
    angle_b: f64,
    roll: &mut f64,
    pitch: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_forward_kinematics([angle_a, angle_b]);

    *roll = res[0];
    *pitch = res[1];

    0
}

#[no_mangle]
pub extern "C" fn forward_velocity(
    ratio_a: f64,
    ratio_b: f64,
    velocity_a: f64,
    velocity_b: f64,
    roll_velocity: &mut f64,
    pitch_velocity: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_output_velocity([velocity_a, velocity_b]);

    *roll_velocity = res[0];
    *pitch_velocity = res[1];

    0
}

#[no_mangle]
pub extern "C" fn forward_torque(
    ratio_a: f64,
    ratio_b: f64,
    torque_a: f64,
    torque_b: f64,
    roll_torque: &mut f64,
    pitch_torque: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_output_torque([torque_a, torque_b]);

    *roll_torque = res[0];
    *pitch_torque = res[1];

    0
}

#[no_mangle]
pub extern "C" fn inverse_position(
    ratio_a: f64,
    ratio_b: f64,
    roll: f64,
    pitch: f64,
    angle_a: &mut f64,
    angle_b: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_inverse_kinematics([roll, pitch]);

    *angle_a = res[0];
    *angle_b = res[1];

    0
}

#[no_mangle]
pub extern "C" fn inverse_velocity(
    ratio_a: f64,
    ratio_b: f64,
    roll_velocity: f64,
    pitch_velocity: f64,
    velocity_a: &mut f64,
    velocity_b: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_input_velocity([roll_velocity, pitch_velocity]);

    *velocity_a = res[0];
    *velocity_b = res[1];

    0
}

#[no_mangle]
pub extern "C" fn inverse_torque(
    ratio_a: f64,
    ratio_b: f64,
    roll_torque: f64,
    pitch_torque: f64,
    torque_a: &mut f64,
    torque_b: &mut f64,
) -> i32 {
    let model = Orbita2dKinematicsModel::new(ratio_a, ratio_b);
    let res = model.compute_input_torque([roll_torque, pitch_torque]);

    *torque_a = res[0];
    *torque_b = res[1];

    0
}