use std::{error::Error, fs, thread};
use crossbeam_channel::Receiver;
use log::{info, LevelFilter};
use simple_logger::SimpleLogger;

mod config;
mod radar_config;
mod device;
mod dsp;
mod bgt60tr13c_const;

use device::bgt60tr13c::Device;
use bgt60tr13c_const::bgt60tr13c_const::*;
use dsp::processor::process_loop;
use radar_config::RadarParams;

fn main() -> Result<(), Box<dyn Error>> {
    let config_dir = "/home/justin/radar_rust/radar_config/fox_test";
    let register_file = format!("{}/BGT60TR13C_export_registers_20241208-224133.txt", config_dir);
    let config_file = format!("{}/settings.json", config_dir);

    // Initialize logger
    SimpleLogger::new().with_level(LevelFilter::Debug).init().unwrap();
    info!("Starting radar application...");

    // --- Parse configuration ---
    let data = fs::read_to_string(config_file)?;
    let config: config::Config = serde_json::from_str(&data)?;
    let params = RadarParams::from_config(&config);

    // --- SPI + Device setup ---
    let spi_bus = rppal::spi::Bus::Spi0;
    let spi_dev = rppal::spi::SlaveSelect::Ss0;
    let spi_speed = 10_000_000; // Hz
    let rst_pin = 12;
    let irq_pin = 25;
    let version = 0;

    let (bgt60tr13c, rx): (Device, Receiver<Vec<u8>>) =
        Device::new(spi_bus, spi_dev, spi_speed, rst_pin, irq_pin, version)?;

    bgt60tr13c.set_fifo_parameters(
        params.num_samples_per_frame as u32,
        params.num_samples_irq as u32,
        params.num_samples_per_burst as u32,
    );

    if bgt60tr13c.check_chip_id() == RET_VAL_OK {
        bgt60tr13c.set_register_config_file(register_file);
        println!("Frame size is {:.2}", params.num_samples_per_frame);

        // --- Start DSP thread ---
        let dsp_thread = thread::spawn(move || {
            process_loop(
                rx,
                params.num_samples_per_chirp,
                params.sample_rate_hz,
                params.chirp_bw_hz,
                params.lambda,
                params.prt,
            );
        });

        // --- Start radar stream ---
        bgt60tr13c.start()?;

        // --- Wait for DSP thread ---
        dsp_thread.join().unwrap();
    }
    Ok(())
}