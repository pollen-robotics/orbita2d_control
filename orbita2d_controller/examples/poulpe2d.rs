use orbita2d_controller::Orbita2dController;

use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    #[arg(short, long, default_value = "config/dxl_poulpe2d.yaml")]
    configfile: String,
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let args = Args::parse();

    log::info!("Config file: {}", args.configfile);

    let mut controller = Orbita2dController::with_config(&args.configfile)?;

    let res = controller.is_torque_on();
    match res {
        Ok(t) => log::info!("Torque is {}", t),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(10));

    let res = controller.disable_torque();
    match res {
        Ok(_) => log::info!("Torque is off"),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(1000));

    let cur = controller.get_current_orientation()?;
    log::info!("Current orientation: {:?}", cur);
    thread::sleep(Duration::from_millis(10));

    let curtarget = controller.get_target_orientation()?;
    log::info!("Current target: {:?}", curtarget);

    thread::sleep(Duration::from_millis(10));
    let vellimit = controller.get_raw_motors_velocity_limit()?;
    log::info!("Vel limit: {:?}", vellimit);

    thread::sleep(Duration::from_millis(10));
    let torquelimit = controller.get_raw_motors_torque_limit()?;
    log::info!("Torque limit: {:?}", torquelimit);

    thread::sleep(Duration::from_millis(10));
    let pid = controller.get_raw_motors_pid_gains()?;
    log::info!("Pid: {:?}", pid);

    let _ = controller.enable_torque(true);
    thread::sleep(Duration::from_millis(100));
    let _ = controller.set_target_orientation([0.0, 0.0]);

    // let init_pos=controller.get_current_orientation()?;
    // log::info!("zero orientation: {:?}", init_pos);
    // thread::sleep(Duration::from_millis(1000));
    // let r=controller.disable_torque();
    // 	match r {
    // 		Ok(_) => log::info!("Torque is off"),
    // 		Err(e) => log::error!("Error: {}", e),
    // 	}
    // thread::sleep(Duration::from_millis(10000));

    let now = SystemTime::now();
    let mut t = now.elapsed().unwrap().as_secs_f32();

    loop {
        if t > 2.0 {
            break;
        }

        t = now.elapsed().unwrap().as_secs_f32();

        // let s=amplitude*(2.0*PI*freq*t as f64).sin();

        // let target_yaw_mat=conversion::intrinsic_roll_pitch_yaw_to_matrix(0.0, 0.0, s);
        // let target=conversion::rotation_matrix_to_quaternion(target_yaw_mat);

        // let fb=controller.set_target_orientation_fb(target);
        // match fb {
        // Ok(fb) => log::info!("Feedback: {:?}", fb),
        // Err(e) => log::error!("Error: {}", e),
        // }

        thread::sleep(Duration::from_millis(1));
    }

    let res = controller.disable_torque();
    match res {
        Ok(_) => log::info!("Torque is off"),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(1000));

    Ok(())
}
