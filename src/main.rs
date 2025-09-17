use anyhow::Result;
use clap::Parser;
use crossbeam_channel::Receiver;
use crate::device::bgt60tr13c::Device;
use crate::dsp::processor::Processor;
use log::{error, info};

mod config;
mod radar_config;
mod bgt60tr13c_const;
mod device;
mod dsp;

use bgt60tr13c_const::bgt60tr13c_const::*;

#[derive(Parser)]
#[command(name = "foxfire")]
#[command(about = "mmWave Radar Gun for Baseball Lab")]
struct Args {
    #[arg(short, long, default_value = "/home/justin/radar_rust/radar_config/fox_test")]
    config_dir: String,
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::init(); // Reads RUST_LOG env var set at cmd prompt with RUST_LOG=info cargo run
    let args = Args::parse();
    let register_file = format!("{}/BGT60TR13C_export_registers_20241208-224133.txt", args.config_dir);
    let config_file = format!("{}/settings.json", args.config_dir);
    let params = radar_config::RadarParams::from_config(config_file);

    // --- SPI + Device setup ---
    let spi_bus = rppal::spi::Bus::Spi0;
    let spi_dev = rppal::spi::SlaveSelect::Ss0;
    let spi_speed = 10_000_000; // Hz
    let rst_pin = 12;
    let irq_pin = 25;
    let version = 0;

    let (bgt60tr13c, rx): (Device, Receiver<Vec<u8>>) =
        Device::new(spi_bus, spi_dev, spi_speed, rst_pin, irq_pin, version).unwrap();


    // Dynamic IRQ calc: e.g., 25% of frame for partial reads
    let num_samples_irq = params.num_samples_per_frame / 4;
    bgt60tr13c.set_fifo_parameters(params.num_samples_per_frame as u32, num_samples_irq as u32, params.num_chirps as u32);

    if bgt60tr13c.check_chip_id() == RET_VAL_OK {
        bgt60tr13c.set_register_config_file(register_file);
        info!("Frame size is {:.2}", params.num_samples_per_frame);
        let _ = bgt60tr13c.start();
    }

    let bgt60tr13c_clone = bgt60tr13c.clone();

    // Graceful shutdown
    ctrlc::set_handler(move || {
        bgt60tr13c.stop();
        std::process::exit(0);
    })?;

    let processor = Processor::new(params);

    // Process frames in blocking thread to avoid tokio blocking issues
    std::thread::spawn(move || {
        while let Ok(frame) = rx.recv() {
            if let Ok(detections) = processor.compute_range_doppler(&frame) {
                for (range, vel, spin) in detections {
                    let vel_mph = vel * 2.23694;
                    if range < 100.0 && vel_mph > 0.0 && vel_mph < 110.0 {
                        info!("Detection: Range {:.2}m, Velocity {:.1}m/s ({:.0}mph), Spin {:.0} RPM", range, vel, vel_mph, spin);
                    }
                }
            } else if let Err(e) = processor.compute_range_doppler(&frame) {
                error!("Processing error: {}", e);
            }
        }
    });

    info!("Foxfire streaming... Press Ctrl+C to stop.");
    tokio::signal::ctrl_c().await?;
    bgt60tr13c_clone.stop();
    Ok(())
}