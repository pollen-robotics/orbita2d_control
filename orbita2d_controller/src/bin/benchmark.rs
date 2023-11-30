use std::time::Duration;

use clap::Parser;

#[macro_use]
extern crate timeit;

/// Benchmark the communication with an Orbita2d
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Name of the person to greet
    #[arg(short, long)]
    configfile: String,

    /// Number of iterations
    #[arg(short, long, default_value = "1000")]
    iterations: usize,
}

fn controller_hwi_read_iteration(
    orbita2d: &mut orbita2d_controller::Orbita2dController,
) -> Result<(), Box<dyn std::error::Error>> {
    let _ = orbita2d.get_current_orientation()?;

    let _ = orbita2d.is_torque_on()?;

    let _ = orbita2d.get_raw_motors_torque_limit()?;
    let _ = orbita2d.get_raw_motors_velocity_limit()?;
    let _ = orbita2d.get_raw_motors_pid_gains()?;

    Ok(())
}

fn controller_hwi_write_iteration(
    orbita2d: &mut orbita2d_controller::Orbita2dController,
) -> Result<(), Box<dyn std::error::Error>> {
    orbita2d.set_target_orientation([0.0, 0.0])?;
    orbita2d.enable_torque(false)?;

    Ok(())
}

fn controller_hwi_iteration(
    orbita2d: &mut orbita2d_controller::Orbita2dController,
) -> Result<(), Box<dyn std::error::Error>> {
    controller_hwi_read_iteration(orbita2d)?;
    controller_hwi_write_iteration(orbita2d)?;

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    println!("Benchmarking Orbita2d:");
    println!("---------------------:");
    println!("Results:");

    println!("\tConnecting to Orbita2d: {:?}", args.configfile);
    let mut orbita2d = orbita2d_controller::Orbita2dController::with_config(&args.configfile)?;

    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        let _ = orbita2d.get_current_orientation()?;
    }));
    println!(
        "\tReading current position: {} ms",
        dur.as_millis() as f64 / args.iterations as f64
    );

    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        controller_hwi_read_iteration(&mut orbita2d)?;
    }));
    println!(
        "\tRead hwi iteration loop: {} ms",
        dur.as_millis() as f64 / args.iterations as f64
    );

    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        controller_hwi_write_iteration(&mut orbita2d)?;
    }));
    println!(
        "\tWrite hwi iteration loop: {} ms",
        dur.as_millis() as f64 / args.iterations as f64
    );

    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        controller_hwi_iteration(&mut orbita2d)?;
    }));
    println!(
        "\tRead/Write hwi iteration loop: {} ms",
        dur.as_millis() as f64 / args.iterations as f64
    );

    let kin = orbita2d_kinematics::Orbita2dKinematicsModel::new(-47.519, -47.519);

    let thetas = [0.20, -1.0];
    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        kin.compute_forward_kinematics(thetas);
    }));
    println!(
        "\tForward kinematics: {} µs",
        dur.as_micros() as f64 / args.iterations as f64
    );

    let rot = kin.compute_forward_kinematics(thetas);
    let dur = Duration::from_secs_f64(timeit_loops!(args.iterations, {
        let _ = kin.compute_inverse_kinematics(rot);
    }));
    println!(
        "\tInverse kinematics: {} µs",
        dur.as_micros() as f64 / args.iterations as f64
    );

    Ok(())
}
