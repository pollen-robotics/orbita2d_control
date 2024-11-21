#![feature(file_create_new)]
use orbita2d_controller::Orbita2dController;

use serde::Deserialize;
use serde::Serialize;

use std::time::SystemTime;
use std::{error::Error, thread, time::Duration};
// use log::info;
// use log::Level;

use clap::Parser;

use rerun;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    #[arg(short, long, default_value = "config/ethercat_poulpe.yaml")]
    configfile: String,

    #[arg(short, long, default_value = "input.csv")]
    input_csv: String,

    #[arg(short, long, default_value = "output.csv")]
    output_csv: String,

    #[arg(short, long)]
    viewer: bool,
}

#[derive(Debug, Deserialize)]
// #[serde(rename_all = "PascalCase")]
struct Input {
    timestamp: f64,
    torque_on: bool,
    target_axis0: f64,
    target_axis1: f64,
    velocity_limit_A: f64,
    velocity_limit_B: f64,
    torque_limit_A: f64,
    torque_limit_B: f64,
}

#[derive(Debug, Serialize)]
struct Output {
    timestamp: f64,
    torque_on: bool,
    present_axis0: f64,
    present_axis1: f64,
    present_velocity_axis0: f64,
    present_velocity_axis1: f64,
    present_torque_axis0: f64,
    present_torque_axis1: f64,
    axis_senror_axis0: f64,
    axis_senror_axis1: f64,
    board_state: u8,
}

fn main() -> Result<(), Box<dyn Error>> {
    env_logger::init();
    let args = Args::parse();

    let rec = if args.viewer {
        let _rec = rerun::RecordingStreamBuilder::new("Test Orbita2d").spawn()?;
        Some(_rec)
    } else {
        None
    };

    log::info!("Config file: {}", args.configfile);
    log::info!("Input csv file: {}", args.input_csv);

    let infile = match std::fs::File::open(&args.input_csv) {
        Ok(f) => f,
        Err(e) => {
            log::error!("Error opening input csv file: {}", e);
            return Err(e.into());
        }
    };

    let outfile = match std::fs::File::create_new(&args.output_csv) {
        Ok(f) => f,
        Err(e) => {
            log::error!("Error opening output csv file: {}", e);
            return Err(e.into());
        }
    };

    let mut input_csv = csv::Reader::from_reader(infile);
    let mut output_csv = csv::Writer::from_writer(outfile);

    let mut controller = Orbita2dController::with_config(&args.configfile)?;
    let t = controller.is_torque_on();
    match t {
        Ok(t) => log::info!("Torque is {}", t),
        Err(e) => log::error!("Error: {}", e),
    }
    let t = controller.disable_torque(); //Start with torque_off
    match t {
        Ok(_) => log::info!("Torque is off"),
        Err(e) => log::error!("Error: {}", e),
    }

    thread::sleep(Duration::from_millis(1000));

    let now = SystemTime::now();

    for in_csv in input_csv.deserialize() {
        let t = now.elapsed().unwrap().as_secs_f64();
        let input_csv_data: Input = in_csv?;
        log::debug!("INPUT: {:?}", input_csv_data);

        //Read feedback from Orbita
        let curr_ori = controller.get_current_orientation()?;
        let torque = controller.is_torque_on()?;
        let curr_vel = controller.get_current_velocity()?;
        let curr_torque = controller.get_current_torque()?;
        let curr_axis = controller.get_axis_sensors()?;
        let curr_state = controller.get_board_state()?;
        output_csv.serialize(Output {
            timestamp: t,
            torque_on: torque,
            present_axis0: curr_ori[0],
            present_axis1: curr_ori[1],
            present_velocity_axis0: curr_vel[0],
            present_velocity_axis1: curr_vel[1],
            present_torque_axis0: curr_torque[0],
            present_torque_axis1: curr_torque[1],
            axis_senror_axis0: curr_axis[0],
            axis_senror_axis1: curr_axis[1],
            board_state: curr_state,
        })?;
        output_csv.flush()?;

        let tosleep = (input_csv_data.timestamp - t) * 1000.0;
        thread::sleep(Duration::from_millis(tosleep as u64));

        //Write commands to Orbita
        if input_csv_data.torque_on {
            controller.enable_torque(true)?;
        } else {
            controller.disable_torque()?;
        }
        controller
            .set_target_orientation([input_csv_data.target_axis0, input_csv_data.target_axis1])?;

        controller.set_raw_motors_velocity_limit([
            input_csv_data.velocity_limit_A,
            input_csv_data.velocity_limit_B,
        ])?;

        controller.set_raw_motors_torque_limit([
            input_csv_data.torque_limit_A,
            input_csv_data.torque_limit_B,
        ])?;

        // Rerun
        if let Some(rec) = &rec {
            rec.set_time_seconds("timestamp", t);
            rec.log(
                "target/torque_on",
                &rerun::Scalar::new(if input_csv_data.torque_on { 1.0 } else { 0.0 }),
            )?;
            rec.log("target/board_state", &rerun::Scalar::new(curr_state as f64))?;

            rec.log(
                "position/target/axis0",
                &rerun::Scalar::new(input_csv_data.target_axis0),
            )?;
            rec.log(
                "position/target/axis1",
                &rerun::Scalar::new(input_csv_data.target_axis1),
            )?;

            rec.log("position/present/axis0", &rerun::Scalar::new(curr_ori[0]))?;
            rec.log("position/present/axis1", &rerun::Scalar::new(curr_ori[1]))?;

            rec.log("velocity/present/axis0", &rerun::Scalar::new(curr_vel[0]))?;
            rec.log("velocity/present/axis1", &rerun::Scalar::new(curr_vel[1]))?;

            rec.log("torque/present/axis0", &rerun::Scalar::new(curr_torque[0]))?;
            rec.log("torque/present/axis1", &rerun::Scalar::new(curr_torque[1]))?;

            rec.log(
                "position/axis_sensor/axis0",
                &rerun::Scalar::new(curr_axis[0]),
            )?;
            rec.log(
                "position/axis_sensor/axis1",
                &rerun::Scalar::new(curr_axis[1]),
            )?;

            rec.log(
                "limits/velocity/A",
                &rerun::Scalar::new(input_csv_data.velocity_limit_A),
            )?;
            rec.log(
                "limits/velocity/B",
                &rerun::Scalar::new(input_csv_data.velocity_limit_B),
            )?;

            rec.log(
                "limits/torque/A",
                &rerun::Scalar::new(input_csv_data.torque_limit_A),
            )?;
            rec.log(
                "limits/torque/B",
                &rerun::Scalar::new(input_csv_data.torque_limit_B),
            )?;
        }
    }

    let torque = controller.disable_torque();
    match torque {
        Ok(_) => log::info!("Torque is off"),
        Err(e) => log::error!("Error: {}", e),
    }
    thread::sleep(Duration::from_millis(1000));

    Ok(())
}
