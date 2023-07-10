use clap::Parser;
use std::{error::Error, thread, time::Duration};

use orbita2d_controller::Orbita2dController;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    #[arg(long, default_value = "/dev/right_shoulder_A")]
    serialport_a: String,

    #[arg(long, default_value = "/dev/right_shoulder_B")]
    serialport_b: String,

    /// baud
    #[arg(short, long, default_value_t = 1_000_000)]
    baudrate: u32,

    /// id
    #[arg(long, default_value_t = 81)]
    id_a: u8,

    /// id
    #[arg(long, default_value_t = 82)]
    id_b: u8,
}

const ERROR_TOLERANCE: f64 = 1e-3;

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let serialportname_a: String = args.serialport_a;
    let serialportname_b: String = args.serialport_b;
    let baudrate: u32 = args.baudrate;
    let id_a: u8 = args.id_a;
    let id_b: u8 = args.id_b;

    //print all the argument values
    println!("serialport_a: {}", serialportname_a);
    println!("serialport_b: {}", serialportname_b);

    println!("baudrate: {}", baudrate);
    println!("id_a: {}", id_a);
    println!("id_b: {}", id_b);

    let mut orbita2d = Orbita2dController::with_flipsky_serial(
        (serialportname_a.as_str(), serialportname_b.as_str()),
        (id_a, id_b),
        [0.0, 0.0],
        [1.9 * 25.01, 1.9 * 25.0], //shoulder
        None,
    )?;

    // let mut orbita2d = Orbita2dController::with_fake_motors();

    //ensure thatthe torque is disabled
    orbita2d.disable_torque()?;
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
        "init_orientation: {:?} curr_orientation: {:?} target_orientation {:?} curr_target {:?}",
        init_orientation, curr_orientation, target, curr_target
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
        "init_orientation: {:?} curr_orientation: {:?} target_orientation {:?} curr_target {:?}",
        init_orientation, curr_orientation, target, curr_target
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
        "init_orientation: {:?} curr_orientation: {:?} target_orientation {:?} curr_target {:?}",
        init_orientation, curr_orientation, target, curr_target
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
