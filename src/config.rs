use serde::Deserialize;
#[allow(dead_code)]
#[derive(Debug, Deserialize)]
pub struct Chirp {
    pub num_samples: usize,
    pub rx_mask: usize,
    pub sample_rate_hz: usize,
    pub end_frequency_hz: usize,
    pub start_frequency_hz: usize,
    pub hp_cutoff_hz: usize,
    pub if_gain_db: usize,
    pub lp_cutoff_hz: usize,
    pub tx_mask: usize,
    pub tx_power_level: usize
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type")]
pub enum Sequence {
    #[serde(rename = "chirp")]
    Chirp(Chirp),

    #[serde(rename = "loop")]
    Loop {
        num_repetitions: usize,
        repetition_time_s: f32,
        sequence: Vec<Sequence>,
    },
}

#[derive(Debug, Deserialize)]
pub struct Config {
    pub sequence: Vec<Sequence>,
}