use std::{
    thread,
    time::{Duration, Instant}, fs::File,
};

use clap::Parser;
use ndarray::Array2;
use ndarray_npy::{read_npy, NpzWriter};
use orbita2d_controller::Orbita2dController;

#[derive(Parser, Debug)]
struct Args {
    /// Flipsky config file
    #[arg(short, long)]
    config: String,

    /// Filename of the input trajectory file (npy)
    #[arg(short, long)]
    input: String,

    /// Filename of the output trajectory file (npz)
    #[arg(short, long)]
    output: Option<String>,

    /// Number of times to repeat the trajectory
    #[arg(short, long, default_value = "1")]
    repeat: u32,

    /// Timestep between trajectory points
    /// (in milliseconds)
    /// (default: 1)
    /// (min: 1)
    #[arg(long, default_value = "1")]
    dt: u32,

    /// Velocity limit (same for both motors)
    /// (in radians per second)
    #[arg(long)]
    raw_velocity_limit: Option<f64>,

    /// Torque limit (same for both motors)
    /// (in Newton meters)
    #[arg(long)]
    raw_torque_limit: Option<f64>,

    /// Fade in time
    /// (in seconds)
    /// (default: 1)
    #[arg(long, default_value = "1")]
    fade_in: Option<f64>,
}

fn goto(
    orbita: &mut Orbita2dController,
    target: [f64; 2],
    duration: Duration,
) -> Result<(), Box<dyn std::error::Error>> {
    // Get current position
    let current = orbita.get_current_orientation()?;

    let start_time = Instant::now();

    while start_time.elapsed() < duration {
        let elapsed = start_time.elapsed().as_secs_f64();
        let t = elapsed / duration.as_secs_f64();

        let pos = [
            current[0] + (target[0] - current[0]) * t,
            current[1] + (target[1] - current[1]) * t,
        ];

        orbita.set_target_orientation(pos)?;

        // Sleep for the desired interval
        thread::sleep(Duration::from_millis(1));
    }

    Ok(())
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let mut orbita = Orbita2dController::with_config(&args.config)?;

    // Setup orbita2d
    // Set velocity limit (if any)
    if let Some(velocity_limit) = args.raw_velocity_limit {
        orbita.set_raw_motors_velocity_limit([velocity_limit, velocity_limit])?;
    }
    // Set torque limit (if any)
    if let Some(torque_limit) = args.raw_torque_limit {
        orbita.set_raw_motors_torque_limit([torque_limit, torque_limit])?;
    }
    // Enable torque
    orbita.enable_torque(true)?;

    // Load and prepare trajectory
    let input_trajectory: Array2<f64> = read_npy(args.input)?;

    // Goto first point depending on fade in time
    if let Some(fade_in) = args.fade_in {
        let target: [f64; 2] = [input_trajectory[[0, 0]], input_trajectory[[0, 1]]];
        goto(&mut orbita, target, Duration::from_secs_f64(fade_in))?;
    }

    // Play and record trajectory
    let mut output_position_trajectory = Vec::new();
    let mut output_velocity_trajectory = Vec::new();
    let mut output_torque_trajectory = Vec::new();
    // output_trajectory.

    for _ in 0..args.repeat {
        for row in input_trajectory.rows() {
            let target: [f64; 2] = [row[0], row[1]];

            orbita.set_target_orientation(target)?;

            let position = orbita.get_current_orientation()?;
            output_position_trajectory.push([position[0], position[1]]);
            let velocity = orbita.get_current_velocity()?;
            output_velocity_trajectory.push([velocity[0], velocity[1]]);
            let torque = orbita.get_current_torque()?;
            output_torque_trajectory.push([torque[0], torque[1]]);

            thread::sleep(Duration::from_millis(args.dt as u64));
        }
    }

    // Disable torque
    orbita.disable_torque()?;

    if let Some(output) = args.output {
        let output_position_trajectory = Array2::from_shape_vec(
            (output_position_trajectory.len(), 2),
            output_position_trajectory.into_iter().flatten().collect(),
        )?;
        let output_velocity_trajectory = Array2::from_shape_vec(
            (output_velocity_trajectory.len(), 2),
            output_velocity_trajectory.into_iter().flatten().collect(),
        )?;
        let output_torque_trajectory = Array2::from_shape_vec(
            (output_torque_trajectory.len(), 2),
            output_torque_trajectory.into_iter().flatten().collect(),
        )?;

        let mut npz = NpzWriter::new(File::create(output)?);
        npz.add_array("position", &output_position_trajectory)?;
        npz.add_array("velocity", &output_velocity_trajectory)?;
        npz.add_array("torque", &output_torque_trajectory)?;
        npz.finish()?;
    }

    Ok(())
}
