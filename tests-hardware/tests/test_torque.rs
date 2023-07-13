use clap::Parser;
use std::{error::Error, thread, time::Duration};

use orbita2d_controller::Orbita2dController;

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
    println!("config file: {}", args.config);
    let mut orbita2d = Orbita2dController::with_config(&args.config)?;

    //ensure that the torque is disabled
    let res = orbita2d.disable_torque();
    println!("starting torque: {:?}", res);

    thread::sleep(Duration::from_millis(1000));

    let init_orientation = orbita2d.get_current_orientation()?;
    let target = [init_orientation[0] - 0.1, init_orientation[1] - 0.2];
    orbita2d.set_target_orientation(target)?;
    thread::sleep(Duration::from_millis(2000));
    let curr_orientation = orbita2d.get_current_orientation()?;
    let curr_target = orbita2d.get_target_orientation()?;

    //print the result, it should be the same, the system should not move
    println!("------------------------");
    println!(
        "TEST disable torque: (init_orientation == curr_orientation) && (target_orientation == curr_target)"
    );

    println!(
        "init_orientation: {:?} \ncurr_orientation: {:?} \ntarget_orientation {:?} \ncurr_target {:?} \ntorque {:?}",
        init_orientation, curr_orientation, target, curr_target, orbita2d.is_torque_on()
    );
    assert!((curr_orientation[0] - init_orientation[0]).abs() < ERROR_TOLERANCE);
    assert!((curr_orientation[1] - init_orientation[1]).abs() < ERROR_TOLERANCE);
    assert!((curr_target[0] - target[0]).abs() < ERROR_TOLERANCE);
    assert!((curr_target[1] - target[1]).abs() < ERROR_TOLERANCE);

    thread::sleep(Duration::from_millis(1000));

    //Enable the torque with the target reset (curr_targett = curr_orientation). Orbita2D should not move!
    orbita2d.enable_torque(true)?;
    thread::sleep(Duration::from_millis(1000));
    let curr_orientation = orbita2d.get_current_orientation()?;
    let curr_target = orbita2d.get_target_orientation()?;
    println!("------------------------");
    println!(
        "TEST enable torque (reset_target=true): (init_orientation == curr_orientation) && (target_orientation != curr_target)"
    );

    println!(
        "init_orientation: {:?} \ncurr_orientation: {:?} \ntarget_orientation {:?} \ncurr_target {:?} \ntorque {:?}",
        init_orientation, curr_orientation, target, curr_target, orbita2d.is_torque_on()
    );

    assert!((curr_orientation[0] - init_orientation[0]).abs() < ERROR_TOLERANCE);
    assert!((curr_orientation[1] - init_orientation[1]).abs() < ERROR_TOLERANCE);
    assert!((curr_target[0] - target[0] - 0.1).abs() < ERROR_TOLERANCE);
    assert!((curr_target[1] - target[1] - 0.2).abs() < ERROR_TOLERANCE);

    //Enable the torque without the target reset. Orbita2D should move!
    let target = [init_orientation[0] - 0.1, init_orientation[1] - 0.2];
    orbita2d.set_target_orientation(target)?;
    orbita2d.enable_torque(false)?;
    thread::sleep(Duration::from_millis(1000));
    let curr_orientation = orbita2d.get_current_orientation()?;
    let curr_target = orbita2d.get_target_orientation()?;
    println!("------------------------");
    println!(
        "TEST enable torque (reset_target=false): (init_orientation != curr_orientation) && (target_orientation == curr_target)"
    );

    println!(
        "init_orientation: {:?} \ncurr_orientation: {:?} \ntarget_orientation {:?} \ncurr_target {:?} \ntorque {:?}",
        init_orientation, curr_orientation, target, curr_target, orbita2d.is_torque_on()
    );

    assert!((curr_orientation[0] - init_orientation[0] + 0.1).abs() < ERROR_TOLERANCE);
    assert!((curr_orientation[1] - init_orientation[1] + 0.2).abs() < ERROR_TOLERANCE);
    assert!((curr_target[0] - target[0]).abs() < ERROR_TOLERANCE);
    assert!((curr_target[1] - target[1]).abs() < ERROR_TOLERANCE);

    // Torque off before exit
    thread::sleep(Duration::from_millis(1000));
    orbita2d.disable_torque()?;
    thread::sleep(Duration::from_millis(1000));

    Ok(())
}
