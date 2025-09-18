//! DSP Processor module for range-Doppler processing in Foxfire radar.

use anyhow::{Result, anyhow};
use log::{trace};
use rustfft::{FftPlanner, num_complex::Complex};
use std::f32::consts::PI;
use std::time::Instant;

use crate::dsp::fft;
use crate::radar_config::RadarParams;
use approx::assert_relative_eq;

const BASEBALL_RADIUS_M: f64 = 0.0365;  // Standard baseball radius for spin calc
const MAX_RANGE_M: f64 = 20.0; // 20m(60 ft) should be plenty for indoor lab
const MIN_VEL_MPS: f64 = 2.0; // used to establish a noise floor
const MAX_VEL_MPS: f64 = 110.0;
const MIN_PEAK_MAG: f32 = 0.5;
const SNR_THRESHOLD: f32 = 20.0; // Increased to reduce false positives in empty room
const MIN_SPIN_RPM: f64 = 10.0; // Filter out detections with no significant micro-Doppler spread
const MIN_RANGE_BIN: usize = 0; // Skip detections in very low range bins to reduce clutter/false positives

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

    /// Extract individual chirps from interleaved frame, unpacking 12-bit packed data.
    fn extract_chirps(&self, frame: &Vec<u8>) -> Result<Vec<Vec<f32>>> {
        let start = Instant::now();
        let num_chirps = self.params.num_chirps;
        let num_samples = self.params.num_samples_per_chirp;
        let total_samples = num_chirps * num_samples;
        let expected_bytes = (total_samples * 3) / 2;
        if frame.len() < expected_bytes {
            return Err(anyhow!("Frame too short for packed 12-bit data: expected at least {} bytes, got {}", expected_bytes, frame.len()));
        }

        let mut samples = vec![0.0f32; total_samples];
        let mut j = 0;
        for i in (0..expected_bytes).step_by(3) {
            let fst_uint8 = frame[i] as u16;
            let mid_uint8 = frame[i + 1] as u16;
            let lst_uint8 = frame[i + 2] as u16;

            let fst_uint12 = (fst_uint8 << 4) + (mid_uint8 >> 4);
            let snd_uint12 = ((mid_uint8 & 0x0F) << 8) + lst_uint8;

            samples[j] = fst_uint12 as f32 - 2048.0;
            samples[j + 1] = snd_uint12 as f32 - 2048.0;
            j += 2;
        }

        let mut chirps: Vec<Vec<f32>> = vec![vec![]; num_chirps];
        for chirp_idx in 0..num_chirps {
            let start_idx = chirp_idx * num_samples;
            let end_idx = start_idx + num_samples;
            chirps[chirp_idx] = samples[start_idx..end_idx].to_vec();
        }
        let duration = start.elapsed();
        trace!("extract_chirps took {:.3}ms", duration.as_secs_f64() * 1000.0);
        Ok(chirps)
    }

    /// Compute range FFTs for all chirps, with low-bin clipping. Returns complex.
    fn compute_range_matrix(&self, chirps: Vec<Vec<f32>>) -> Result<Vec<Vec<Complex<f32>>>> {
        let start = Instant::now();
        let mut range_matrix: Vec<Vec<Complex<f32>>> = vec![vec![]; chirps.len()];
        let clip_bins = 10;  // Low bins to zero for clutter suppression

        for (chirp_idx, chirp) in chirps.into_iter().enumerate() {
            let range_fft = fft::process_fft(&chirp, self.params.num_samples_per_chirp, 0, 0);
            let mut clipped = range_fft.clone();
            for i in 0..=clip_bins.min(clipped.len().saturating_sub(1)) {
                clipped[i] = Complex::new(0.0, 0.0);
            }
            range_matrix[chirp_idx] = clipped;
        }
        let duration = start.elapsed();
        trace!("compute_range_matrix took {:.3}ms", duration.as_secs_f64() * 1000.0);
        Ok(range_matrix)
    }

    /// Extract Doppler peaks (velocities and spin) per range bin from slow-time matrix.
    fn extract_doppler_peaks(&self, range_matrix: &Vec<Vec<Complex<f32>>>) -> Result<Vec<(f64, f64, f64)>> {
        let start = Instant::now();
        let num_chirps = self.params.num_chirps;
        let num_range_bins = range_matrix[0].len();
        let fft_doppler = FftPlanner::new().plan_fft_forward(num_chirps);
        let mut detections = vec![];

        let doppler_res = self.params.lambda as f64 / (2.0 * (num_chirps as f64) * self.params.prt);  // Vel res m/s

        for bin in MIN_RANGE_BIN..num_range_bins {
            // Stack slow-time for this range bin
            let mut doppler_input: Vec<Complex<f32>> = range_matrix.iter()
                .map(|row| row[bin])
                .collect();

            // MTI: Subtract mean to remove zero-Doppler clutter
            let mut mean = Complex::new(0.0, 0.0);
            for &c in &doppler_input {
                mean += c;
            }
            mean /= num_chirps as f32;
            for c in doppler_input.iter_mut() {
                *c -= mean;
            }

            // Hann window on slow-time
            let window: Vec<f32> = (0..num_chirps).map(|i| {
                0.5 * (1.0 - (2.0 * PI * i as f32 / (num_chirps - 1) as f32).cos())
            }).collect();
            for (c, w) in doppler_input.iter_mut().zip(&window) {
                *c *= *w;
            }

            // Doppler FFT
            fft_doppler.process(&mut doppler_input);

            // FFT shift to center zero velocity
            let shift = num_chirps / 2;
            let mut shifted = vec![Complex::new(0.0, 0.0); num_chirps];
            shifted[0..num_chirps - shift].copy_from_slice(&doppler_input[shift..]);
            shifted[num_chirps - shift..].copy_from_slice(&doppler_input[0..shift]);

            // Magnitudes (normalized)
            let magnitudes: Vec<f32> = shifted.iter()
                .map(|c| c.norm() / num_chirps as f32)
                .collect();

            // Find peak (TODO: Integrate CFAR for better detection)
            if let Some((peak_bin, &peak_mag)) = magnitudes.iter().enumerate()
                .filter(|&(_, &mag)| mag > 0.1)  // Tune threshold
                .max_by(|a, b| {
                    let cmp = a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal);
                    if cmp == std::cmp::Ordering::Equal {
                        b.0.cmp(&a.0)
                    } else {
                        cmp
                    }
                }) {
                if peak_mag > MIN_PEAK_MAG {
                    // Adaptive SNR check, excluding guard cells around peak
                    let guard = 2; // Bins to exclude on each side of peak
                    let mut sum_noise = 0.0;
                    let mut count = 0;
                    for (i, &mag) in magnitudes.iter().enumerate() {
                        if ((i as i32 - peak_bin as i32).abs() > guard as i32) {
                            sum_noise += mag;
                            count += 1;
                        }
                    }
                    let avg_noise = if count > 0 { sum_noise / count as f32 } else { 0.0 };
                    let snr = if avg_noise > 0.0 { peak_mag / avg_noise } else { f32::INFINITY };
                    if snr < SNR_THRESHOLD {
                        trace!("Filtered low SNR: range bin {}, peak mag {:.2}, avg noise {:.2}, SNR {:.2}", bin, peak_mag, avg_noise, snr);
                        continue;
                    }

                    let range_m = (bin as f64) * (3e8 / (2.0 * self.params.chirp_bw_hz as f64));
                    let vel_mps = self.params.doppler_bin_to_velocity(peak_bin);

                    trace!("Peak mag at bin {}: {:.2}", peak_bin, peak_mag);

                    if range_m > MAX_RANGE_M || vel_mps < MIN_VEL_MPS || vel_mps > MAX_VEL_MPS || vel_mps < 0.0 {
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
                    if sum_mag == 0.0 {
                        continue; // No valid sidebands
                    }
                    let mean_bin = sum_bin_mag / sum_mag;
                    let var_bin = (sum_bin_sq_mag / sum_mag) - mean_bin.powi(2);
                    let std_bin = var_bin.sqrt() as f64;
                    let std_vel = std_bin * doppler_res as f64;  // Map to velocity spread
                    let omega = std_vel / BASEBALL_RADIUS_M;  // rad/s
                    let spin_rpm = (omega * 60.0) / (2.0 * PI as f64);  // Convert to RPM

                    if spin_rpm < MIN_SPIN_RPM {
                        trace!("Filtered low spin: range {:.2}m, vel {:.1}m/s, spin {:.0} RPM", range_m, vel_mps, spin_rpm);
                        continue;
                    }

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
    use std::f32::consts::PI;

    fn mock_params() -> RadarParams {
        RadarParams {
            num_chirps: 8,
            num_samples_per_chirp: 32,
            prt: 0.000028,
            lambda: 0.005,
            chirp_bw_hz: 5_000_000_000,
            sample_rate_hz: 4_000_000,
            ..Default::default()
        }
    }

    #[test]
    fn test_extract_chirps() {
        let processor = Processor::new(mock_params());
        // For test, create packed frame
        let total_samples = 8 * 32;
        let expected_bytes = (total_samples * 3) / 2;
        let frame = vec![0u8; expected_bytes];
        let chirps = processor.extract_chirps(&frame).unwrap();
        assert_eq!(chirps.len(), 8);
        assert_eq!(chirps[0].len(), 32);
    }

    #[test]
    fn test_extract_chirps_short_frame() {
        let processor = Processor::new(mock_params());
        let frame = vec![0u8; 100];
        assert!(processor.extract_chirps(&frame).is_err());
    }

    #[test]
    fn test_compute_range_matrix() {
        let processor = Processor::new(mock_params());
        let chirps = vec![vec![128.0f32; 32]; 8];
        let matrix = processor.compute_range_matrix(chirps).unwrap();
        assert_eq!(matrix.len(), 8);
        assert_eq!(matrix[0].len(), 17);
        assert!(matrix[0][0..=10].iter().all(|&c| c == Complex::new(0.0, 0.0)));
    }

    #[test]
    fn test_compute_range_matrix_non_neutral() {
        let processor = Processor::new(mock_params());
        let chirps = vec![vec![0.0f32; 32]; 8];
        let matrix = processor.compute_range_matrix(chirps).unwrap();
        assert!(matrix[0].iter().all(|&c| c == Complex::new(0.0, 0.0)));
    }

    #[test]
    fn test_doppler_bin_to_velocity() {
        let params = mock_params();
        let num_chirps = 8;
        let bin_zero = num_chirps / 2;
        let vel_zero = params.doppler_bin_to_velocity(bin_zero);
        assert_relative_eq!(vel_zero, 0.0, epsilon = 1e-6);

        let bin_max = 0;
        let vel_max = params.doppler_bin_to_velocity(bin_max);
        assert!(vel_max > 0.0);

        let bin_min = num_chirps - 1;
        let vel_min = params.doppler_bin_to_velocity(bin_min);
        assert!(vel_min < 0.0);

        let res = params.lambda as f64 / (2.0 * params.prt * num_chirps as f64);
        assert!(res > 0.0, "Resolution positive");
    }

    #[test]
    fn test_extract_doppler_peaks_low_amp() {
        let processor = Processor::new(mock_params());
        let num_chirps = 8;
        let num_bins = 17;
        let mut matrix: Vec<Vec<Complex<f32>>> = vec![vec![Complex::new(0.0, 0.0); num_bins]; num_chirps];
        let range_bin = 10;
        let doppler_freq = 1.0;
        let amp = 5.0;  // Low
        for (i, row) in matrix.iter_mut().enumerate() {
            let phase = 2.0 * PI * doppler_freq * i as f32 / num_chirps as f32;
            let phase1 = 2.0 * PI * (doppler_freq + 0.5) * i as f32 / num_chirps as f32;
            let phase2 = 2.0 * PI * (doppler_freq - 0.5) * i as f32 / num_chirps as f32;
            row[range_bin] += Complex::new(amp * phase.cos(), amp * phase.sin());
            row[range_bin] += Complex::new((amp / 4.0) * phase1.cos(), (amp / 4.0) * phase1.sin());
            row[range_bin] += Complex::new((amp / 5.0) * phase2.cos(), (amp / 5.0) * phase2.sin());
        }
        let detections = processor.extract_doppler_peaks(&matrix).unwrap();
        assert!(detections.is_empty(), "Low amp filtered");
    }

    #[test]
    fn test_extract_doppler_peaks() {
        let processor = Processor::new(mock_params());
        let num_chirps = 8;
        let num_bins = 17;
        let mut matrix: Vec<Vec<Complex<f32>>> = vec![vec![Complex::new(1.0, 0.0); num_bins]; num_chirps];  // Add low background noise
        let range_bin = 10;
        let doppler_freq = 1.0;
        let amp = 2000.0;
        for (i, row) in matrix.iter_mut().enumerate() {
            let phase = 2.0 * PI * doppler_freq * i as f32 / num_chirps as f32;
            let phase1 = 2.0 * PI * (doppler_freq + 0.5) * i as f32 / num_chirps as f32;
            let phase2 = 2.0 * PI * (doppler_freq - 0.5) * i as f32 / num_chirps as f32;
            row[range_bin] += Complex::new(amp * phase.cos(), amp * phase.sin());
            row[range_bin] += Complex::new((amp / 4.0) * phase1.cos(), (amp / 4.0) * phase1.sin());
            row[range_bin] += Complex::new((amp / 5.0) * phase2.cos(), (amp / 5.0) * phase2.sin());
        }
        let detections = processor.extract_doppler_peaks(&matrix).unwrap();
        if detections.is_empty() {
            // Diagnostic to help debug
            let mut max_mag = 0.0;
            let num_chirps = 8;
            let fft_doppler = FftPlanner::new().plan_fft_forward(num_chirps);
            for bin in 0..num_bins {
                let mut input = matrix.iter().map(|row| row[bin]).collect::<Vec<_>>();
                let mut mean = Complex::new(0.0, 0.0);
                for &c in &input {
                    mean += c;
                }
                mean /= num_chirps as f32;
                for c in input.iter_mut() {
                    *c -= mean;
                }
                // Window
                let window: Vec<f32> = (0..num_chirps).map(|i| {
                    0.5 * (1.0 - (2.0 * PI * i as f32 / (num_chirps - 1) as f32).cos())
                }).collect();
                for (c, w) in input.iter_mut().zip(&window) {
                    *c *= *w;
                }
                fft_doppler.process(&mut input);
                let shift = num_chirps / 2;
                let mut shifted = vec![Complex::new(0.0, 0.0); num_chirps];
                shifted[0..num_chirps - shift].copy_from_slice(&input[shift..]);
                shifted[num_chirps - shift..].copy_from_slice(&input[0..shift]);
                let mags = shifted.iter().map(|c| c.norm() / num_chirps as f32).collect::<Vec<_>>();
                let max_in_bin = mags.iter().cloned().fold(0.0, f32::max);
                if max_in_bin > max_mag {
                    max_mag = max_in_bin;
                }
                if bin == 10 {
                    println!("Diagnostic: Magnitudes for range bin 10: {:?}", mags);
                    let peak_mag = mags.iter().cloned().fold(0.0, f32::max);
                    let peak_bin = mags.iter().position(|&m| m == peak_mag).unwrap();
                    let guard = 2;
                    let mut sum_noise = 0.0;
                    let mut count = 0;
                    for (i, &mag) in mags.iter().enumerate() {
                        if ((i as i32 - peak_bin as i32).abs() > guard as i32) {
                            sum_noise += mag;
                            count += 1;
                        }
                    }
                    let avg_noise = if count > 0 { sum_noise / count as f32 } else { 0.0 };
                    let snr = if avg_noise > 0.0 { peak_mag / avg_noise } else { f32::INFINITY };
                    println!("Diagnostic: Peak mag {:.2}, avg noise {:.2}, SNR {:.2}", peak_mag, avg_noise, snr);
                }
            }
            println!("Diagnostic: No detections - max Doppler mag {:.2}, check thresholds", max_mag);
        }
        assert!(!detections.is_empty(), "Should detect simulated target");
        let (_, vel, spin) = detections[0];
        assert!(vel.abs() > 0.0, "Non-zero velocity");
        assert!(spin > 0.0, "Some spin spread");
    }
}