use clap::Parser;
use orbita2d_controller::Orbita2dController;
use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};

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
    thread::sleep(Duration::from_millis(1000));
    //Test for the torque limit at 0.0
    let torque_limit = [0.0, 0.0];
    let init_torque_limit = orbita2d.get_torque_limit()?;
    orbita2d.set_torque_limit(torque_limit)?;
    let curr_torque_limit = orbita2d.get_torque_limit()?;
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
    orbita2d.set_torque_limit(torque_limit)?;
    let curr_torque_limit = orbita2d.get_torque_limit()?;
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
    orbita2d.set_torque_limit(init_torque_limit)?; //back to starting value

    let curr_torque_limit = orbita2d.get_torque_limit()?;
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
