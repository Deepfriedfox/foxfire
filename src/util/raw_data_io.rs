use bincode::{serde::*, config::*};
use serde::{Deserialize, Serialize};
use std::fs::File;
use std::io::{self, Read, Write};
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};
use chrono::Utc;

// Metadata struct (extend with your radar_param, etc.)
#[derive(Serialize, Deserialize, Debug)]
pub struct RawFrame {
    pub frame_id: u64,
    pub timestamp_ns: u128,  // Nanosecond precision
    pub raw_data: Vec<u8>,
    // Optional: pub radar_param: YourRadarParams,  // If serializable
}

pub fn record_raw_data<P: AsRef<Path>>(raw_data: &[u8], filename: &P, frame_id: u64) -> io::Result<()> {
    let frame = RawFrame {
        frame_id,
        timestamp_ns: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos(),
        raw_data: raw_data.to_vec(),
    };
    let config = standard().with_little_endian();
    let encoded: Vec<u8> = encode_to_vec(&frame, config).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    
    let mut file = File::create(filename)?;
    file.write_all(&encoded)?;
    println!("Recorded frame {} ({} bytes + metadata) to {:?}", frame.frame_id, encoded.len(), filename.as_ref());
    Ok(())
}

pub fn playback_raw_data<P: AsRef<Path>>(filename: &P) -> io::Result<RawFrame> {
    let mut file = File::open(filename)?;
    let mut encoded = Vec::new();
    file.read_to_end(&mut encoded)?;
    
    let config = standard().with_little_endian();
    let (frame, _): (RawFrame, usize) = decode_from_slice(&encoded, config).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    println!("Loaded frame {} ({} bytes raw) from {:?}", frame.frame_id, frame.raw_data.len(), filename.as_ref());
    Ok(frame)
}

/// Append a single RawFrame to an existing file (creates if not exists).
pub fn append_raw_frame<P: AsRef<Path>>(frame: &RawFrame, filename: &P) -> io::Result<()> {
    let config = standard().with_little_endian();
    let encoded: Vec<u8> = encode_to_vec(frame, config).map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;
    
    // Open in append mode (create if missing)
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .append(true)
        .open(filename)?;
    file.write_all(&encoded)?;
    file.flush()?;  // Ensure written
    println!("Appended frame {} ({} bytes) to {:?}", frame.frame_id, encoded.len(), filename.as_ref());
    Ok(())
}

/// Load all RawFrames from a session file (sequential deserialization).
pub fn playback_session<P: AsRef<Path>>(filename: &P) -> io::Result<Vec<RawFrame>> {
    let mut file = File::open(filename)?;
    let mut encoded = Vec::new();
    file.read_to_end(&mut encoded)?;
    
    let mut frames = Vec::new();
    let mut offset = 0;
    let config = standard().with_little_endian();
    
    while offset < encoded.len() {
        match decode_from_slice(&encoded[offset..], config) {
            Ok((frame, len)) => {
                frames.push(frame);
                offset += len;
            }
            Err(e) => {
                if offset == 0 {
                    return Err(io::Error::new(io::ErrorKind::InvalidData, format!("Failed to decode first frame: {}", e)));
                }
                // Tolerate trailing garbage (e.g., partial write); warn and stop
                eprintln!("Warning: Failed to decode at offset {} ({} bytes left): {}", offset, encoded.len() - offset, e);
                break;
            }
        }
    }
    
    println!("Loaded {} frames from {:?}", frames.len(), filename.as_ref());
    Ok(frames)
}