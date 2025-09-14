// src/dsp/fft.rs

use num_complex::Complex32;
use rustfft::FftPlanner;
use std::f32::consts::PI;

/// Process a single chirp worth of raw ADC data into a single-sided FFT magnitude spectrum.
///
/// Returns a Vec of length (N/2 + 1) containing the single-sided magnitude spectrum
/// for real-valued input.  The values are scaled so that magnitudes are comparable
/// across different FFT sizes:
///  - bin 0 (DC) and bin N/2 (Nyquist, if present) are scaled by 1/N
///  - all other positive-frequency bins are scaled by 2/N
pub fn process_fft(
    data: &[u8],
    num_samples_per_chirp: usize,
    _sample_rate_hz: usize,
    _chirp_bw_hz: usize,
) -> Vec<f32> {
    let n = num_samples_per_chirp;
    if n == 0 { return Vec::new(); }

    // ---- 1) Convert raw bytes -> f32 samples ----
    // For these tests we assume each u8 represents one sample with a DC offset.
    // Real device data (12-bit packed) should be unpacked into f32 before calling this.
    let mut samples: Vec<f32> = data
        .iter()
        .take(n)
        .map(|&x| x as f32)
        .collect();

    // ---- 2) Remove mean (basic DC removal) ----
    // This helps avoid a dominant DC bin that can mask small tones.
    let mean = samples.iter().sum::<f32>() / samples.len() as f32;
    for s in samples.iter_mut() { *s -= mean; }

    // ---- 3) Apply Hann window to reduce spectral leakage ----
    // w[n] = 0.5 * (1 - cos(2*pi*n/(N-1)))
    for (i, s) in samples.iter_mut().enumerate() {
        let w = 0.5f32 * (1.0 - (2.0 * PI * i as f32 / (n as f32 - 1.0)).cos());
        *s *= w;
    }

    // ---- 4) Create complex buffer and run FFT ----
    let mut buffer: Vec<Complex32> = samples.into_iter()
        .map(|re| Complex32::new(re, 0.0))
        .collect();

    let mut planner = FftPlanner::<f32>::new();
    let fft = planner.plan_fft_forward(n);
    fft.process(&mut buffer);

    // ---- 5) Build single-sided magnitude spectrum and normalize ----
    //
    // For real-valued input, bins 1..N/2-1 correspond to positive frequencies and
    // are mirrored by bins N-1..N/2+1. We form the single-sided spectrum:
    //  mag_single[0] = |X[0]| / N
    //  mag_single[k] = 2 * |X[k]| / N   for 1 <= k < N/2
    //  mag_single[N/2] = |X[N/2]| / N    (only if N even)
    let half = n / 2;
    let mut mags: Vec<f32> = Vec::with_capacity(half + 1);
    for k in 0..=half {
        let mag = buffer[k].norm();
        let scaled = if k == 0 || (n % 2 == 0 && k == half) {
            mag / (n as f32)
        } else {
            2.0 * mag / (n as f32)
        };
        mags.push(scaled);
    }

    mags
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Generate a test signal having a single tone exactly at `freq_bin` and verify
    /// that the FFT pipeline places the strongest peak at that bin.
    ///
    /// Notes about the test:
    /// - We produce samples in the 0..255 (u8) range by adding a DC offset (127)
    ///   and an amplitude. The process_fft() function subtracts the mean anyway,
    ///   so the absolute DC offset used here is not critical.
    /// - We expect the single-sided spectrum index `freq_bin` to show the largest
    ///   magnitude for a tone placed at that bin.
    #[test]
    fn test_fft_detects_tone() {
        let n = 64;             // number of samples
        let freq_bin = 5;       // inject a tone at bin 5 (positive frequency)

        // ---- Build the synthetic signal ----
        // sine wave at bin 5 with DC offset so values fit into u8.
        let signal: Vec<u8> = (0..n)
            .map(|i| {
                // 127 DC offset, amplitude 50 (safe within 0..255)
                let val = (127.0 + 50.0 * (2.0 * PI * freq_bin as f32 * i as f32 / n as f32).sin()) as u8;
                val
            })
            .collect();

        // ---- Run FFT pipeline (mean removal, window, FFT, single-sided mags) ----
        let spectrum = process_fft(&signal, n, 2_000_000, 2_000_000_000);

        // ---- Locate maximum bin in returned single-sided spectrum ----
        let (max_bin, _) = spectrum
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();

        // The peak should be at the injected bin (5)
        assert_eq!(max_bin, freq_bin);
    }
}
