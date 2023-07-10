use clap::Parser;
use std::{
    error::Error,
    f64::consts::PI,
    thread,
    time::{Duration, SystemTime},
};

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

    // (1.8 * 25.01, 1.8 * 25.0), //elbow
    orbita2d.enable_torque(true)?;

    let freq = 0.25;
    let amp = 30.0_f64.to_radians();
    let amp2 = 15.0_f64.to_radians();

    let t0 = SystemTime::now();
    let mut t = t0.elapsed().unwrap().as_secs_f64();
    while t < 10.0 {
        let pos = orbita2d.get_current_orientation()?;

        t = t0.elapsed().unwrap().as_secs_f64();
        let target = amp * (2.0 * PI * freq * t).sin();
        let target2 = amp2 * (2.0 * PI * freq * t).cos() - 90.0_f64.to_radians();
        // let target3 = 30.0_f64.to_radians() * (2.0 * PI * freq * t).sin();
        // let posik = orbita2d.set_target_orientation((target as f32, target2 as f32))?;
        // println!("pos: {:?} ik {:?}", pos, posik);
        // orbita2d.set_motors_goal_position((target as f32, target2 as f32))?;
        orbita2d.set_target_orientation([target, target2])?;

        println!("pos: {:?}", pos);
        thread::sleep(Duration::from_millis(10));
    }
    println!("STOP");
    orbita2d.enable_torque(false)?;
    Ok(())
}
