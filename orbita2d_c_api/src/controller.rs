use crate::sync_map::SyncMap;
use motor_toolbox_rs::Limit;
use motor_toolbox_rs::PID;

use once_cell::sync::Lazy;
use orbita2d_controller::Orbita2dController;

use std::{ffi::CStr, sync::Mutex};
static UID: Lazy<Mutex<u32>> = Lazy::new(|| Mutex::new(0));
static CONTROLLER: Lazy<SyncMap<u32, Orbita2dController>> = Lazy::new(SyncMap::new);
// use log::debug;
fn print_error(e: Box<dyn std::error::Error>) {
    // eprintln!("[ORBITA_2D] {:?}", e);
    log::debug!("[ORBITA_2D] Error: {:?}", e);
}

#[no_mangle]
pub extern "C" fn orbita2d_controller_with_flipsky_serial(
    serial_port_a: *const libc::c_char,
    serial_port_b: *const libc::c_char,
    id_a: u8,
    id_b: u8,
    offset_a: f64,
    offset_b: f64,
    ratio_a: f64,
    ratio_b: f64,
    ring_inverted: bool,
    center_inverted: bool,
    lower_limit_a: f64,
    upper_limit_a: f64,
    lower_limit_b: f64,
    upper_limit_b: f64,
    use_cache: bool,
    uid: &mut u32,
) -> u32 {
    let serial_port_a = unsafe { CStr::from_ptr(serial_port_a) }.to_str().unwrap();
    let serial_port_b = unsafe { CStr::from_ptr(serial_port_b) }.to_str().unwrap();

    let orientation_limits = Some([
        Limit::new(lower_limit_a, upper_limit_a),
        Limit::new(lower_limit_b, upper_limit_b),
    ]);

    match Orbita2dController::with_flipsky_serial(
        (serial_port_a, serial_port_b),
        (id_a, id_b),
        [offset_a, offset_b],
        [ratio_a, ratio_b],
        [ring_inverted, center_inverted],
        orientation_limits,
        use_cache,
    ) {
        Ok(c) => {
            *uid = get_available_uid();
            CONTROLLER.insert(*uid, c);
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_controller_from_config(
    configfile: *const libc::c_char,
    uid: &mut u32,
) -> u32 {
    let configfile = unsafe { CStr::from_ptr(configfile) }.to_str().unwrap();

    let _ = env_logger::try_init();

    match Orbita2dController::with_config(configfile) {
        Ok(c) => {
            *uid = get_available_uid();
            CONTROLLER.insert(*uid, c);
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_is_torque_on(uid: u32, is_on: &mut bool) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().is_torque_on() {
        Ok(v) => {
            *is_on = v;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_enable_torque(uid: u32, reset_target: bool) -> u32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .enable_torque(reset_target)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_disable_torque(uid: u32) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().disable_torque() {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_current_orientation(uid: u32, pos: &mut [f64; 2]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_current_orientation() {
        Ok(v) => {
            *pos = v;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_current_velocity(uid: u32, vel: &mut [f64; 2]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_current_velocity() {
        Ok(v) => {
            *vel = v;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_current_torque(uid: u32, torque: &mut [f64; 2]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_current_torque() {
        Ok(v) => {
            *torque = v;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_target_orientation(uid: u32, pos: &mut [f64; 2]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_target_orientation() {
        Ok(v) => {
            *pos = v;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_target_orientation(uid: u32, pos: &[f64; 2]) -> u32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_target_orientation(*pos)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_target_velocity(uid: u32, vel: &mut [f64; 2]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_target_velocity() {
        Ok(v) => {
            *vel = v;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_target_velocity(uid: u32, vel: &[f64; 2]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().set_target_velocity(*vel) {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_target_torque(uid: u32, torque: &mut [f64; 2]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_target_torque() {
        Ok(t) => {
            *torque = t;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_target_torque(uid: u32, torque: &[f64; 2]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().set_target_torque(*torque) {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_raw_motors_velocity_limit(
    uid: u32,
    raw_motors_velocity_limit: &mut [f64; 2],
) -> u32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_velocity_limit()
    {
        Ok(v) => {
            *raw_motors_velocity_limit = v;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_raw_motors_velocity_limit(
    uid: u32,
    raw_motors_velocity_limit: &[f64; 2],
) -> u32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_velocity_limit(*raw_motors_velocity_limit)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_raw_motors_torque_limit(
    uid: u32,
    raw_motors_torque_limit: &mut [f64; 2],
) -> u32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_torque_limit()
    {
        Ok(v) => {
            *raw_motors_torque_limit = v;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_raw_motors_torque_limit(
    uid: u32,
    raw_motors_torque_limit: &[f64; 2],
) -> u32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_torque_limit(*raw_motors_torque_limit)
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_raw_motors_pid_gains(uid: u32, pids: &mut [f64; 6]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_raw_motors_pid_gains() {
        Ok([pid_a, pid_b]) => {
            pids[0] = pid_a.p;
            pids[1] = pid_a.i;
            pids[2] = pid_a.d;
            pids[3] = pid_b.p;
            pids[4] = pid_b.i;
            pids[5] = pid_b.d;

            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_raw_motors_pid_gains(uid: u32, pids: &[f64; 6]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().set_raw_motors_pid_gains([
        PID {
            p: pids[0],
            i: pids[1],
            d: pids[2],
        },
        PID {
            p: pids[3],
            i: pids[4],
            d: pids[5],
        },
    ]) {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_raw_motors_current(
    uid: u32,
    raw_motors_current: &mut [f64; 2],
) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_raw_motors_current() {
        Ok(c) => {
            *raw_motors_current = c;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_raw_motors_velocity(
    uid: u32,
    raw_motors_velocity: &mut [f64; 2],
) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_raw_motors_velocity() {
        Ok(v) => {
            *raw_motors_velocity = v;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_axis_sensors(uid: u32, axis_sensors: &mut [f64; 2]) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_axis_sensors() {
        Ok(s) => {
            *axis_sensors = s;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_board_state(uid: u32, state: &mut u8) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_board_state() {
        Ok(s) => {
            *state = s;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_board_state(uid: u32, state: &u8) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().set_board_state(*state) {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_error_codes(uid: u32, errors: &mut [i32; 2]) -> i32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_error_codes() {
        Ok(c) => {
            *errors = c;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_motor_temperatures(uid: u32, temp: &mut [f64; 2]) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_temperature()
    {
        Ok(t) => {
            *temp = t;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_board_temperatures(uid: u32, temp: &mut [f64; 2]) -> i32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .get_raw_boards_temperature()
    {
        Ok(t) => {
            *temp = t;
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_control_mode(uid: u32, mode: &mut u8) -> u32 {
    match CONTROLLER.get_mut(&uid).unwrap().get_control_mode() {
        Ok(m) => {
            *mode = m[0];
            0
        }
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_control_mode(uid: u32, mode: &u8) -> u32 {
    match CONTROLLER
        .get_mut(&uid)
        .unwrap()
        .set_control_mode([*mode, *mode])
    {
        Ok(_) => 0,
        Err(e) => {
            print_error(e);
            1
        }
    }
}

#[no_mangle]
pub extern "C" fn orbita3d_emergency_stop(uid: u32) -> i32 {
    CONTROLLER.get_mut(&uid).unwrap().emergency_stop();
    0
}

fn get_available_uid() -> u32 {
    let mut uid = UID.lock().unwrap();
    *uid += 1;
    *uid
}
