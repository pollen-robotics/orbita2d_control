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

    //ensure that the torque is disabled
    orbita2d.disable_torque()?;
    thread::sleep(Duration::from_millis(1000));
    //Enable the torque with the target reset (curr_targett = curr_orientation). Orbita2D should not move!
    orbita2d.enable_torque(true)?;

    //Set a low velocity limit
    orbita2d.set_velocity_limit(0.1)?;

    // Torque off before exit
    thread::sleep(Duration::from_millis(1000));
    orbita2d.disable_torque()?;
    thread::sleep(Duration::from_millis(1000));

    Ok(())
}
