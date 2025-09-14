use log::info;
use crate::config;

pub struct RadarParams {
    pub num_chirps: usize,
    pub num_samples_per_chirp: usize,
    pub sample_rate_hz: usize,
    pub chirp_bw_hz: usize,
    pub prt: f64,
    pub num_samples_per_frame: usize,
    pub num_samples_irq: usize,
    pub num_samples_per_burst: usize,
    pub lambda: f32,
}

impl RadarParams {
    pub fn from_config(config: &config::Config) -> Self {
        let c = 3.0e8; // speed of light

        let mut num_chirps = 0;
        let mut num_samples_per_chirp = 0;
        let mut sample_rate_hz = 0;
        let mut chirp_bw_hz = 0;
        let mut prt = 0.0;
        let mut lambda = 0.0;

        if let Some(config::Sequence::Loop { num_repetitions: _, repetition_time_s: frame_period, sequence }) = config.sequence.get(0) {
            info!("Frame Rate: {}", 1.0 / frame_period);

            if let Some(config::Sequence::Loop { num_repetitions: inner_reps, repetition_time_s: inner_rep_time, sequence: inner_seq }) = sequence.get(0) {
                num_chirps = *inner_reps;
                info!("Chirps per frame: {}", num_chirps);

                prt = *inner_rep_time as f64;
                info!("Pulse Repetition Time: {}", prt);

                let wavelength = 3e8_f64 / 60e9_f64;
                let velo_m_per_sec = wavelength / (4.0 * *inner_rep_time as f64);
                info!("Theoretical max velocity (mph): {}", velo_m_per_sec * 2.236936);

                if let Some(config::Sequence::Chirp(chirp)) = inner_seq.get(0) {
                    info!("Samples per chirp: {}", chirp.num_samples);
                    let num_rx_ant = chirp.rx_mask.count_ones() as usize;
                    info!("Antennas enabled: {}", num_rx_ant);
                    info!("ADC Rate: {}", chirp.sample_rate_hz);

                    num_samples_per_chirp = chirp.num_samples * num_rx_ant;
                    sample_rate_hz = chirp.sample_rate_hz;
                    chirp_bw_hz = chirp.end_frequency_hz - chirp.start_frequency_hz;

                    let frame_period = num_chirps as f32 * prt as f32;
                    let doppler_resolution = 1.0 / frame_period;

                    lambda = c as f32 / ((chirp.end_frequency_hz + chirp.start_frequency_hz) as f32 / 2.0);
                    let velo_resolution_mps = doppler_resolution * lambda / 2.0;

                    info!("Range resolution (ft): {}", c / (2.0 * chirp_bw_hz as f32) * 3.28);
                    info!("Doppler resolution (mph): {}", velo_resolution_mps * 2.237);
                }
            }
        }

        Self {
            num_chirps,
            num_samples_per_chirp,
            sample_rate_hz,
            chirp_bw_hz,
            prt,
            num_samples_per_frame: num_chirps * num_samples_per_chirp,
            num_samples_irq: 8192,
            num_samples_per_burst: 1024,
            lambda,
        }
    }
}
