use clap::Parser;
use orbita2d_controller::Orbita2dController;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};
extern crate log;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Flipsky config file
    #[arg(short, long)]
    config: String,
}

const ERROR_TOLERANCE: f64 = 1e-3;

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    env_logger::init();

    println!("config file: {}", args.config);
    let mut orbita2d = Orbita2dController::with_config(&args.config)?;

    // let mut orbita2d = Orbita2dController::with_fake_motors();

    //ensure that the torque is disabled
    orbita2d.disable_torque()?;
    thread::sleep(Duration::from_millis(1000));

    //Enable the torque with the target reset (curr_targett = curr_orientation). Orbita2D should not move!
    orbita2d.enable_torque(true)?;
    thread::sleep(Duration::from_millis(1000));
    //Set a low velocity limit
    let vel_limit = [10.0, 10.0];
    let init_vel_limit = orbita2d.get_raw_motors_velocity_limit()?;
    orbita2d.set_raw_motors_velocity_limit(vel_limit)?;
    let curr_vel_limit = orbita2d.get_raw_motors_velocity_limit()?;
    println!(
        "setting velocity_limit to {:?} init velocity limit: {:?}",
        vel_limit, curr_vel_limit
    );
    assert!((vel_limit[0] - curr_vel_limit[0]).abs() < ERROR_TOLERANCE);
    assert!((vel_limit[1] - curr_vel_limit[1]).abs() < ERROR_TOLERANCE);

    println!("Moving");
    let curr_orientation = orbita2d.get_current_orientation()?;
    let target = [curr_orientation[0] + 1.0, curr_orientation[1] + 0.5];
    println!(
        "curr_orientation: {:?} target: {:?}",
        curr_orientation, target
    );

    orbita2d.set_target_orientation(target)?;
    let now = SystemTime::now();
    let mut t = now.elapsed().unwrap().as_secs_f32();
    loop {
        if t >= 10.0 {
            break;
        }

        let curr_vel = orbita2d.get_current_velocity()?;
        let curr_orientation = orbita2d.get_current_orientation()?;
        println!(
            "curr_vel: {:?} curr_orientation: {:?}",
            curr_vel, curr_orientation
        );
        //very rough check because the velocity is very noisy
        assert!((curr_vel[0].abs() < 0.4) && (curr_vel[1].abs() < 0.4));

        thread::sleep(Duration::from_millis(10));
        t = now.elapsed().unwrap().as_secs_f32();
    }
    let curr_orientation = orbita2d.get_current_orientation()?;
    println!(
        "curr_orientation: {:?} target: {:?}",
        curr_orientation, target
    );
    //ensure that the torque is disabled
    orbita2d.disable_torque()?;
    orbita2d.set_raw_motors_velocity_limit(init_vel_limit)?; //back to starting value
    let curr_vel_limit = orbita2d.get_raw_motors_velocity_limit()?;
    println!(
        "setting velocity_limit to {:?} read velocity limit: {:?}",
        init_vel_limit, curr_vel_limit
    );
    // Torque off before exit
    thread::sleep(Duration::from_millis(1000));
    orbita2d.disable_torque()?;
    thread::sleep(Duration::from_millis(1000));

    Ok(())
}
