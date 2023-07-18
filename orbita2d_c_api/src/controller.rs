use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use orbita2d_controller::{AngleLimit, Orbita2dController, PID};

lazy_static! {
    static ref UID: Mutex<u32> = Mutex::new(0);
    static ref CONTROLLER: Mutex<HashMap<u32, Orbita2dController>> = Mutex::new(HashMap::new());
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
        AngleLimit {
            min: lower_limit_a,
            max: upper_limit_a,
        },
        AngleLimit {
            min: lower_limit_b,
            max: upper_limit_b,
        },
    ]);

    match Orbita2dController::with_flipsky_serial(
        (serial_port_a, serial_port_b),
        (id_a, id_b),
        [offset_a, offset_b],
        [ratio_a, ratio_b],
        orientation_limits,
        use_cache,
    ) {
        Ok(c) => {
            *uid = get_available_uid();
            CONTROLLER.lock().unwrap().insert(*uid, c);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_controller_from_config(
    configfile: *const libc::c_char,
    uid: &mut u32,
) -> u32 {
    let configfile = unsafe { CStr::from_ptr(configfile) }.to_str().unwrap();

    match Orbita2dController::with_config(configfile) {
        Ok(c) => {
            *uid = get_available_uid();
            CONTROLLER.lock().unwrap().insert(*uid, c);
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_is_torque_on(uid: u32, is_on: &mut bool) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .is_torque_on()
    {
        Ok(v) => {
            *is_on = v;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_enable_torque(uid: u32, reset_target: bool) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .enable_torque(reset_target)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_disable_torque(uid: u32) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .disable_torque()
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_current_orientation(uid: u32, pos: &mut [f64; 2]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_current_orientation()
    {
        Ok(v) => {
            *pos = v;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_current_velocity(uid: u32, vel: &mut [f64; 2]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_current_velocity()
    {
        Ok(v) => {
            *vel = v;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_current_torque(uid: u32, torque: &mut [f64; 2]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_current_torque()
    {
        Ok(v) => {
            *torque = v;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_target_orientation(uid: u32, pos: &mut [f64; 2]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_target_orientation()
    {
        Ok(v) => {
            *pos = v;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_target_orientation(uid: u32, pos: &[f64; 2]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_target_orientation(*pos)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_raw_motors_velocity_limit(
    uid: u32,
    raw_motors_velocity_limit: &mut [f64; 2],
) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_velocity_limit()
    {
        Ok(v) => {
            *raw_motors_velocity_limit = v;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_raw_motors_velocity_limit(
    uid: u32,
    raw_motors_velocity_limit: &[f64; 2],
) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_velocity_limit(*raw_motors_velocity_limit)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_raw_motors_torque_limit(
    uid: u32,
    raw_motors_torque_limit: &mut [f64; 2],
) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_torque_limit()
    {
        Ok(v) => {
            *raw_motors_torque_limit = v;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_raw_motors_torque_limit(
    uid: u32,
    raw_motors_torque_limit: &[f64; 2],
) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_torque_limit(*raw_motors_torque_limit)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_raw_motors_pid_gains(uid: u32, pids: &mut [f64; 6]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_raw_motors_pid_gains()
    {
        Ok([pid_a, pid_b]) => {
            pids[0] = pid_a.p;
            pids[1] = pid_a.i;
            pids[2] = pid_a.d;
            pids[3] = pid_b.p;
            pids[4] = pid_b.i;
            pids[5] = pid_b.d;

            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_raw_motors_pid_gains(uid: u32, pids: &[f64; 6]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_raw_motors_pid_gains([
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
        Err(_) => 1,
    }
}

fn get_available_uid() -> u32 {
    let mut uid = UID.lock().unwrap();
    *uid += 1;
    *uid
}
