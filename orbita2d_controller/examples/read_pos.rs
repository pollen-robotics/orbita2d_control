use clap::Parser;
use std::{error::Error, thread, time::Duration};

use orbita2d_controller::Orbita2dController;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// tty
    #[arg(long, default_value = "/dev/ttyACM0")]
    serialport_a: String,

    #[arg(long, default_value = "/dev/ttyACM1")]
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
        [1.0, 1.0],
        None,
    )?;

    orbita2d.disable_torque()?;

    loop {
        let pos = orbita2d.get_current_orientation()?;

        println!("pos: {:?}", pos);
        thread::sleep(Duration::from_millis(100));
    }
}
