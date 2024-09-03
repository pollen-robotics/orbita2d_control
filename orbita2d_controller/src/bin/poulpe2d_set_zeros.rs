use orbita2d_controller::Orbita2dController;

use std::{error::Error, thread, time::Duration};

use log::warn;

use clap::Parser;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long, default_value = "config/dxl_poulpe2d.yaml")]
    configfile: Option<String>,
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let args = Args::parse();

    let configfile = args.configfile.unwrap();
    log::info!("Config file: {}", configfile);

    let mut controller = Orbita2dController::with_config(&configfile)?;

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

    warn!("Set Orbita2D in the zero position!");
    thread::sleep(Duration::from_secs(10));

    let mut axis1: [f64; 10] = [0.0; 10];
    let mut axis2: [f64; 10] = [0.0; 10];

    for i in 0..10 {
        let res = controller.get_axis_sensors()?;
        axis1[i] = res[0];
        axis2[i] = res[1];
        thread::sleep(Duration::from_millis(100));
    }

    let avg1 = axis1.iter().sum::<f64>() / 10.0;
    let avg2 = axis2.iter().sum::<f64>() / 10.0;

    println!("Zeros to write to the yaml config");
    println!();
    println!("motors_offset:");
    println!(" - {}", avg1);
    println!(" - {}", avg2);

    Ok(())
}
