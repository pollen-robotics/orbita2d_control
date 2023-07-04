use std::{collections::HashMap, ffi::CStr, sync::Mutex};

use orbita2d_controller::{AngleLimit, Orbita2dController, PID};

lazy_static! {
    static ref UID: Mutex<u32> = Mutex::new(0);
    static ref CONTROLLER: Mutex<HashMap<u32, Orbita2dController>> = Mutex::new(HashMap::new());
}

#[no_mangle]
pub extern "C" fn orbita2d_controller_with_flipsky_serial(
    serial_port_1: *const libc::c_char,
    serial_port_2: *const libc::c_char,
    id_1: u8,
    id_2: u8,
    offset_1: f64,
    offset_2: f64,
    ratio_1: f64,
    ratio_2: f64,
    lower_limit_1: f64,
    upper_limit_1: f64,
    lower_limit_2: f64,
    upper_limit_2: f64,
    uid: &mut u32,
) -> u32 {
    let serial_port_1 = unsafe { CStr::from_ptr(serial_port_1) }.to_str().unwrap();
    let serial_port_2 = unsafe { CStr::from_ptr(serial_port_2) }.to_str().unwrap();

    let orientation_limits = Some([
        AngleLimit {
            min: lower_limit_1,
            max: upper_limit_1,
        },
        AngleLimit {
            min: lower_limit_2,
            max: upper_limit_2,
        },
    ]);

    match Orbita2dController::with_flipsky_serial(
        (serial_port_1, serial_port_2),
        (id_1, id_2),
        [offset_1, offset_2],
        [ratio_1, ratio_2],
        orientation_limits,
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
pub extern "C" fn orbita2d_get_velocity_limit(uid: u32, vel_limit: &mut [f64; 2]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_velocity_limit()
    {
        Ok(v) => {
            *vel_limit = v;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_velocity_limit(uid: u32, vel_limit: &[f64; 2]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_velocity_limit(*vel_limit)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_torque_limit(uid: u32, torque_limit: &mut [f64; 2]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_torque_limit()
    {
        Ok(v) => {
            *torque_limit = v;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_torque_limit(uid: u32, torque_limit: &[f64; 2]) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_torque_limit(*torque_limit)
    {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_get_pid_gains(
    uid: u32,
    kp: &mut f64,
    ki: &mut f64,
    kd: &mut f64,
) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .get_pid_gains()
    {
        Ok(pid) => {
            *kp = pid.p;
            *ki = pid.i;
            *kd = pid.d;
            0
        }
        Err(_) => 1,
    }
}

#[no_mangle]
pub extern "C" fn orbita2d_set_pid_gains(uid: u32, kp: f64, ki: f64, kd: f64) -> u32 {
    match CONTROLLER
        .lock()
        .unwrap()
        .get_mut(&uid)
        .unwrap()
        .set_pid_gains(PID {
            p: kp,
            i: ki,
            d: kd,
        }) {
        Ok(_) => 0,
        Err(_) => 1,
    }
}

fn get_available_uid() -> u32 {
    let mut uid = UID.lock().unwrap();
    *uid += 1;
    *uid
}
