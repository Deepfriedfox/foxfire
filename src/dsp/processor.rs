use crossbeam_channel::Receiver;
use log::info;

use super::{fft, cfar};

// Convert doppler bin index -> radial velocity (m/s).
/// `bin` may be an unsigned index 0..N-1; this function handles signed wrap.
/// `n` = num_chirps, `prt_s` = pulse repetition time (s), `lambda_m` = wavelength (m).
fn doppler_bin_to_velocity(bin: usize, n: usize, prt_s: f64, lambda_m: f64) -> f64 {
    let mut k = bin as isize;
    if k > (n as isize) / 2 {
        k -= n as isize; // wrap to negative bins
    }
    let f_d = k as f64 / (n as f64 * prt_s); // Hz
    let v = f_d * lambda_m / 2.0; // m/s
    v
}

pub fn process_loop(
    rx: Receiver<Vec<u8>>,
    num_samples_per_chirp: usize,
    sample_rate_hz: usize,
    chirp_bw_hz: usize,
    lambda: f32,
    prt: f64,
) {
    info!("DSP loop started...");
    for frame in rx.iter() {
        let fft_out = fft::process_fft(&frame, num_samples_per_chirp, sample_rate_hz, chirp_bw_hz);
        let detections = cfar::run_cfar(&fft_out);
        if !detections.is_empty() {
            info!("Detections: {:?}", detections);
        }
    }
}
