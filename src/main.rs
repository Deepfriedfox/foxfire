use std::fs;
use std::process;
use std::time::UNIX_EPOCH;
use anyhow::Result;
use chrono::Utc;
use clap::Parser;
use crossbeam_channel::Receiver;
use crate::device::bgt60tr13c::Device;
use crate::dsp::processor::Processor;
use log::{error, info};
use crate::util::raw_data_io::{record_raw_data, playback_raw_data,append_raw_frame,RawFrame};

mod config;
mod radar_config;
mod bgt60tr13c_const;
mod device;
mod dsp;
mod util;

use bgt60tr13c_const::bgt60tr13c_const::*;

#[derive(Parser)]
#[command(name = "foxfire")]
#[command(about = "mmWave Radar Gun for Baseball Lab")]
struct Args {
    #[arg(short, long, default_value = "/home/justin/radar_rust/radar_config/fox_test")]
    config_dir: String,
    #[arg(long)]
    record: bool,
    #[arg(long, default_value_t = 10000u64)]
    max_frames: u64,
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
    let bgt60tr13c_clone2 = bgt60tr13c.clone();
    // Graceful shutdown
    ctrlc::set_handler(move || {
        bgt60tr13c.stop();
        std::process::exit(0);
    })?;

    let record_mode = args.record;
    let max_frames = args.max_frames;
    std::thread::spawn(move || {
        if record_mode {
            let mut counter = 0u64;
        let timestamp = Utc::now().format("%Y-%m-%d_%H-%M-%S").to_string();
        let filename = format!("recordings/radar_session_{}.bin", timestamp);
        let base_dir = "recordings";
        if let Err(e) = fs::create_dir_all(base_dir) {
            error!("Failed to create recordings dir: {}", e);
            return;
        }
        
        // Touch the file first (empty append creates it)
        if let Err(e) = std::fs::OpenOptions::new().write(true).create(true).open(&filename) {
            error!("Failed to create session file {}: {}", filename, e);
            return;
        }
        
        info!("Recording mode: Appending up to {} frames to {}", max_frames, filename);
        
        while let Ok(frame) = rx.recv() {
            let raw_frame = RawFrame {
                frame_id: counter,
                timestamp_ns: std::time::SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos(),
                raw_data: frame.clone(),  // Clone Vec<u8> (cheap if small)
            };
            
            if let Err(e) = append_raw_frame(&raw_frame, &filename) {
                error!("Append error for frame {}: {}", counter, e);
            } else {
                info!("Appended frame {}", counter);
            }
            
            counter += 1;
            if counter >= max_frames {
                info!("Completed {} frames in {}. Stopping radar.", max_frames, filename);
                bgt60tr13c_clone2.stop();
                process::exit(0);
            }
        }
        info!("Recording stopped (channel closed). Total frames appended: {}", counter);
        } else {
            let processor = Processor::new(params);
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
        }
    });

    info!("Foxfire streaming... Press Ctrl+C to stop.");
    tokio::signal::ctrl_c().await?;
    bgt60tr13c_clone.stop();
    Ok(())
}