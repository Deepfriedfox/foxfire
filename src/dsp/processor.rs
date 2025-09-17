//! DSP Processor module for range-Doppler processing in Foxfire radar.

use anyhow::{Result, anyhow};
use log::{info, trace};
use rustfft::{FftPlanner, num_complex::Complex};
use std::f32::consts::PI;
use std::time::Instant;

use crate::dsp::fft;
use crate::radar_config::RadarParams;

const BASEBALL_RADIUS_M: f64 = 0.0365;  // Standard baseball radius for spin calc
const MAX_RANGE_M: f64 = 20.0; // 20m(60 ft) should be plenty for indoor lab
const MIN_VEL_MPS: f64 = 2.0; // used to establish a noise floor
const MAX_VEL_MPS: f64 = 110.0;


/// Processor for handling frame data into range-Doppler detections.
pub struct Processor {
    pub params: RadarParams,
}

impl Processor {
    /// Create a new processor with given parameters.
    pub fn new(params: RadarParams) -> Self {
        Self { params }
    }

    /// Compute 2D range-Doppler and get detections with velocities and spin rates.
    pub fn compute_range_doppler(&self, frame: &Vec<u8>) -> Result<Vec<(f64, f64, f64)>> {
        let start = Instant::now();
        let chirps = self.extract_chirps(frame)?;
        let range_matrix = self.compute_range_matrix(chirps)?;
        let detections = self.extract_doppler_peaks(&range_matrix)?;
        let duration = start.elapsed();
        trace!("compute_range_doppler took {:.3}ms", duration.as_secs_f64() * 1000.0);
        Ok(detections)
    }

    /// Extract individual chirps from interleaved frame.
    fn extract_chirps(&self, frame: &Vec<u8>) -> Result<Vec<Vec<u8>>> {
        let start = Instant::now();
        let num_chirps = self.params.num_chirps;
        let num_samples = self.params.num_samples_per_chirp;
        if frame.len() < num_chirps * num_samples {
            return Err(anyhow!("Frame too short for {} chirps x {} samples", num_chirps, num_samples));
        }

        let mut chirps: Vec<Vec<u8>> = vec![vec![]; num_chirps];
        for chirp_idx in 0..num_chirps {
            let start_idx = chirp_idx * num_samples;
            let end_idx = start_idx + num_samples;
            chirps[chirp_idx] = frame[start_idx..end_idx].to_vec();
        }
        let duration = start.elapsed();
        trace!("extract_chirps took {:.3}ms", duration.as_secs_f64() * 1000.0);
        Ok(chirps)
    }

    /// Compute range FFTs for all chirps, with low-bin clipping.
    fn compute_range_matrix(&self, chirps: Vec<Vec<u8>>) -> Result<Vec<Vec<f32>>> {
        let start = Instant::now();
        let mut range_matrix: Vec<Vec<f32>> = vec![vec![]; chirps.len()];
        let clip_bins = 10;  // Low bins to zero for clutter suppression

        for (chirp_idx, chirp) in chirps.into_iter().enumerate() {
            let range_fft = fft::process_fft(&chirp, self.params.num_samples_per_chirp, 0, 0);  // Pass dummies for unused params
            let mut clipped = range_fft.clone();
            for i in 0..=clip_bins.min(clipped.len().saturating_sub(1)) {
                clipped[i] = 0.0;
            }
            range_matrix[chirp_idx] = clipped;
        }
        let duration = start.elapsed();
        trace!("compute_range_matrix took {:.3}ms", duration.as_secs_f64() * 1000.0);
        Ok(range_matrix)
    }

    /// Extract Doppler peaks (velocities and spin) per range bin from slow-time matrix.
    fn extract_doppler_peaks(&self, range_matrix: &Vec<Vec<f32>>) -> Result<Vec<(f64, f64, f64)>> {
        let start = Instant::now();
        let num_chirps = self.params.num_chirps;
        let num_range_bins = range_matrix[0].len();
        let fft_doppler = FftPlanner::new().plan_fft_forward(num_chirps);
        let mut detections = vec![];

        let doppler_res = self.params.lambda as f64 / (2.0 * (num_chirps as f64) * self.params.prt);  // Vel res m/s

        for bin in 0..num_range_bins {
            // Stack slow-time for this range bin
            let mut doppler_input: Vec<Complex<f32>> = range_matrix.iter()
                .map(|row| Complex { re: row[bin], im: 0.0 })
                .collect();

            // Hann window on slow-time
            let window: Vec<f32> = (0..num_chirps).map(|i| {
                0.5 * (1.0 - (2.0 * PI * i as f32 / (num_chirps - 1) as f32).cos())
            }).collect();
            for (c, w) in doppler_input.iter_mut().zip(&window) {
                c.re *= *w;
            }

            // Doppler FFT
            fft_doppler.process(&mut doppler_input);

            // Magnitudes (normalized)
            let magnitudes: Vec<f32> = doppler_input.iter()
                .map(|c| (c.re.powi(2) + c.im.powi(2)).sqrt() / num_chirps as f32)
                .collect();

            // Find peak (TODO: Integrate CFAR for better detection)
            if let Some((peak_bin, &peak_mag)) = magnitudes.iter().enumerate()
                .filter(|&(_, &mag)| mag > 0.1)  // Tune threshold
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap_or(std::cmp::Ordering::Equal)) {
                if peak_mag > 0.1 {
                    let range_m = (bin as f64) * (3e8 * self.params.prt / (2.0 * self.params.num_samples_per_chirp as f64));
                    let vel_mps = self.params.doppler_bin_to_velocity(peak_bin);

                    trace!("Peak mag at bin {}: {:.2}", peak_bin, peak_mag);

                    if range_m > MAX_RANGE_M || vel_mps < MIN_VEL_MPS || vel_mps > MAX_VEL_MPS || vel_mps < 0.0 {  // NEW: Positive vel only
                        trace!("Filtered: range {:.2}m, vel {:.1}m/s, mag {:.2}", range_m, vel_mps, peak_mag);
                        continue;
                    }

                    // Spin rate: Weighted STD around peak for spectral spread (micro-Doppler)
                    let mut sum_mag = 0.0;
                    let mut sum_bin_mag = 0.0;
                    let mut sum_bin_sq_mag = 0.0;
                    let window_half = 5;  // Bins around peak to consider for spread
                    for i in (peak_bin.saturating_sub(window_half))..=(peak_bin + window_half).min(magnitudes.len() - 1) {
                        let mag = magnitudes[i];
                        if mag > 0.5 * peak_mag {  // Only strong sidebands
                            sum_mag += mag;
                            sum_bin_mag += (i as f32) * mag;
                            sum_bin_sq_mag += ((i as f32).powi(2)) * mag;
                        }
                    }
                    let mean_bin = sum_bin_mag / sum_mag;
                    let var_bin = (sum_bin_sq_mag / sum_mag) - mean_bin.powi(2);
                    let std_bin = var_bin.sqrt() as f64;
                    let std_vel = std_bin * doppler_res as f64;  // Map to velocity spread
                    let omega = std_vel / BASEBALL_RADIUS_M;  // rad/s
                    let spin_rpm = (omega * 60.0) / (2.0 * PI as f64);  // Convert to RPM

                    detections.push((range_m, vel_mps, spin_rpm));
                }
            }
        }

        // Sort by range descending (farthest first)
        detections.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        let duration = start.elapsed();
        trace!("extract_doppler_peaks took {:.3}ms", duration.as_secs_f64() * 1000.0);
        Ok(detections)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::radar_config::RadarParams;

    /// Mock params for testing.
    fn mock_params() -> RadarParams {
        RadarParams {
            num_chirps: 4,
            num_samples_per_chirp: 64,
            prt: 0.001,  // 1ms
            lambda: 0.005,  // 60GHz
            // Add other fields as needed
            ..Default::default()  // Assumes impl Default for RadarParams
        }
    }

    #[test]
    fn test_extract_chirps() {
        let processor = Processor::new(mock_params());
        let frame = vec![0u8; 256];  // 4 chirps x 64 samples
        let chirps = processor.extract_chirps(&frame).unwrap();
        assert_eq!(chirps.len(), 4);
        assert_eq!(chirps[0].len(), 64);
    }

    #[test]
    fn test_compute_range_matrix() {
        let processor = Processor::new(mock_params());
        let chirps = vec![vec![1u8; 64]; 4];  // Fake chirps
        let matrix = processor.compute_range_matrix(chirps).unwrap();
        assert_eq!(matrix.len(), 4);
        assert!(!matrix[0].is_empty());
        // Check clipping: First 11 should be 0 (clip_bins=10 +0)
        assert_eq!(matrix[0][0..=10].iter().sum::<f32>(), 0.0);
    }

    #[test]
    fn test_extract_doppler_peaks() {
        let processor = Processor::new(mock_params());
        // Synthetic matrix: Constant for zero vel, spread for spin
        let num_chirps = 4;
        let num_bins = 32;  // Half FFT
        let mut matrix: Vec<Vec<f32>> = vec![vec![0.0; num_bins]; num_chirps];
        for row in matrix.iter_mut() {
            row[10] = 1.0;  // Peak at range bin 10
            row[11] = 0.5;  // Spread for spin
        }
        let detections = processor.extract_doppler_peaks(&matrix).unwrap();
        assert!(!detections.is_empty());
        let (_, vel, spin) = detections[0];
        assert!(vel.abs() < 1.0);  // Near zero
        assert!(spin > 0.0);  // Some spread
    }

    #[test]
    fn test_compute_range_doppler_full() {
        let processor = Processor::new(mock_params());
        let frame = vec![0u8; 256];  // Empty -> no detections
        let detections = processor.compute_range_doppler(&frame).unwrap();
        assert!(detections.is_empty());

        // Error case
        let short_frame = vec![0u8; 100];
        assert!(processor.compute_range_doppler(&short_frame).is_err());
    }
}