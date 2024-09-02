use clap::Parser;
use lerp::LerpIter;
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
    #[arg(short, long, default_value = "config/dxl_poulpe2d.yaml")]
    configfile: Option<String>,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    let configfile = args.configfile.unwrap();
    log::info!("Config file: {}", configfile);

    let mut orbita2d = Orbita2dController::with_config(&configfile)?;

    let res = orbita2d.is_torque_on();
    match res {
        Ok(t) => log::info!("Torque is {}", t),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(10));

    orbita2d.enable_torque(true)?;

    thread::sleep(Duration::from_millis(10));

    let current_pos = orbita2d.get_current_orientation()?;
    log::info!("Current position: {:?}", current_pos);
    // GOTO zero slowly
    const NB_STEPS: usize = 300;
    let roll_lerp: Vec<_> = current_pos[0].lerp_iter(0.0, NB_STEPS).collect();
    let pitch_lerp: Vec<_> = current_pos[1].lerp_iter(0.0, NB_STEPS).collect();

    roll_lerp
        .iter()
        .zip(pitch_lerp.iter())
        .for_each(|(p, r): (&f64, &f64)| {
            let fb = orbita2d.set_target_orientation_fb([*r, *p]);
            thread::sleep(Duration::from_millis(20));
            match fb {
                Ok(fb) => {
                    log::info!("Feedback: {:?}", fb);
                }
                Err(e) => {
                    log::error!("Error (rp: {:?}): {}", [*r, *p], e);
                    panic!("END");
                }
            }
        });

    thread::sleep(Duration::from_millis(1000));

    let freq = 0.25;
    let amp = 30.0_f64.to_radians();
    let amp2 = 15.0_f64.to_radians();

    let t0 = SystemTime::now();
    let mut t = t0.elapsed().unwrap().as_secs_f64();
    while t < 5.0 {
        // let pos = orbita2d.get_current_orientation()?;

        t = t0.elapsed().unwrap().as_secs_f64();
        let target = amp * (2.0 * PI * freq * t).sin();
        let target2 = amp2 * (2.0 * PI * freq * t).cos() - 90.0_f64.to_radians();

        let fb = orbita2d.set_target_orientation_fb([target, target2])?;

        println!("pos: {:?}", fb);
        thread::sleep(Duration::from_millis(10));
    }

    let current_pos = orbita2d.get_current_orientation()?;
    log::info!("Current position: {:?}", current_pos);
    // GOTO zero slowly

    let roll_lerp: Vec<_> = current_pos[0].lerp_iter(0.0, NB_STEPS).collect();
    let pitch_lerp: Vec<_> = current_pos[1].lerp_iter(0.0, NB_STEPS).collect();

    roll_lerp
        .iter()
        .zip(pitch_lerp.iter())
        .for_each(|(p, r): (&f64, &f64)| {
            let fb = orbita2d.set_target_orientation_fb([*r, *p]);
            thread::sleep(Duration::from_millis(20));
            match fb {
                Ok(fb) => {
                    log::info!("Feedback: {:?}", fb);
                }
                Err(e) => {
                    log::error!("Error (rp: {:?}): {}", [*r, *p], e);
                    panic!("END");
                }
            }
        });

    thread::sleep(Duration::from_millis(1000));

    println!("STOP");
    orbita2d.enable_torque(false)?;
    Ok(())
}
