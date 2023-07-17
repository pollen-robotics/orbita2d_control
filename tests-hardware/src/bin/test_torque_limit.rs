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
    //Test for the torque limit at 0.0
    let torque_limit = [0.0, 0.0];
    let init_torque_limit = orbita2d.get_raw_motors_torque_limit()?;
    orbita2d.set_raw_motors_torque_limit(torque_limit)?;
    let curr_torque_limit = orbita2d.get_raw_motors_torque_limit()?;
    println!(
        "setting torque_limit to {:?} init velocity limit: {:?}",
        torque_limit, curr_torque_limit
    );
    assert!((torque_limit[0] - curr_torque_limit[0]).abs() < ERROR_TOLERANCE);
    assert!((torque_limit[1] - curr_torque_limit[1]).abs() < ERROR_TOLERANCE);

    println!("It should not move");
    let init_orientation = orbita2d.get_current_orientation()?;
    let target = [init_orientation[0] + 1.0, init_orientation[1] + 1.0];
    println!(
        "init_orientation: {:?} target: {:?}",
        init_orientation, target
    );

    orbita2d.set_target_orientation(target)?;
    let now = SystemTime::now();
    let mut t = now.elapsed().unwrap().as_secs_f32();
    loop {
        if t >= 5.0 {
            break;
        }

        let curr_torque = orbita2d.get_current_torque()?;
        let curr_orientation = orbita2d.get_current_orientation()?;
        println!(
            "curr_torque: {:?} curr_orientation: {:?}",
            curr_torque, curr_orientation
        );
        //It should not move
        assert!(
            ((curr_orientation[0] - init_orientation[0]).abs() < ERROR_TOLERANCE)
                && ((curr_orientation[1] - init_orientation[1]).abs() < ERROR_TOLERANCE)
        );

        thread::sleep(Duration::from_millis(10));
        t = now.elapsed().unwrap().as_secs_f32();
    }
    let curr_orientation = orbita2d.get_current_orientation()?;
    println!(
        "curr_orientation: {:?} target: {:?}",
        curr_orientation, target
    );

    ///////////////////////////

    orbita2d.disable_torque()?;
    thread::sleep(Duration::from_millis(1000));

    //Enable the torque with the target reset (curr_targett = curr_orientation). Orbita2D should not move!
    orbita2d.enable_torque(true)?;
    thread::sleep(Duration::from_millis(1000));
    //Set a low torque limit, high enough to move
    let torque_limit = [0.5, 0.5];
    orbita2d.set_raw_motors_torque_limit(torque_limit)?;
    let curr_torque_limit = orbita2d.get_raw_motors_torque_limit()?;
    println!(
        "setting torque_limit to {:?} init velocity limit: {:?}",
        torque_limit, curr_torque_limit
    );
    assert!((torque_limit[0] - curr_torque_limit[0]).abs() < ERROR_TOLERANCE);
    assert!((torque_limit[1] - curr_torque_limit[1]).abs() < ERROR_TOLERANCE);

    println!("It should not move");
    let init_orientation = orbita2d.get_current_orientation()?;
    let target = [init_orientation[0] + 1.0, init_orientation[1] + 1.0];
    println!(
        "init_orientation: {:?} target: {:?}",
        init_orientation, target
    );

    orbita2d.set_target_orientation(target)?;
    thread::sleep(Duration::from_millis(500));
    let now = SystemTime::now();
    let mut t = now.elapsed().unwrap().as_secs_f32();
    loop {
        if t >= 5.0 {
            break;
        }

        let curr_torque = orbita2d.get_current_torque()?;
        let curr_orientation = orbita2d.get_current_orientation()?;
        println!(
            "curr_torque: {:?} curr_orientation: {:?}",
            curr_torque, curr_orientation
        );
        //It should move
        assert!(
            ((curr_orientation[0] - init_orientation[0]).abs() > ERROR_TOLERANCE)
                && ((curr_orientation[1] - init_orientation[1]).abs() > ERROR_TOLERANCE)
        );

        thread::sleep(Duration::from_millis(10));
        t = now.elapsed().unwrap().as_secs_f32();
    }
    let curr_orientation = orbita2d.get_current_orientation()?;
    println!(
        "curr_orientation: {:?} target: {:?}",
        curr_orientation, target
    );

    ///////////////////////////

    //ensure that the torque is disabled
    orbita2d.disable_torque()?;
    orbita2d.set_raw_motors_torque_limit(init_torque_limit)?; //back to starting value

    let curr_torque_limit = orbita2d.get_raw_motors_torque_limit()?;
    println!(
        "setting torque_limit to {:?} read torque limit: {:?}",
        init_torque_limit, curr_torque_limit
    );
    // Torque off before exit
    thread::sleep(Duration::from_millis(1000));
    orbita2d.disable_torque()?;
    thread::sleep(Duration::from_millis(1000));

    Ok(())
}
