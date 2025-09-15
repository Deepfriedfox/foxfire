# Foxfire: mmWave Radar Gun for Baseball Lab

A Rust-based driver and DSP pipeline for the Infineon BGT60TR13C mmWave radar, optimized for pitch velocity tracking on Raspberry Pi.

## Setup
1. Clone the repo: `git clone https://github.com/Deepfriedfox/foxfire`
2. Install deps: `cargo build --release`
3. Run: `sudo ./target/release/foxfire --config-dir radar_config/fox_test`

## Features
- SPI/GPIO config from JSON sequences
- Raw ADC streaming with IRQ polling
- FFT + CFAR for range-Doppler detections
- Velocity calcs for baseball speeds (up to ~50 mph unambiguous)

## Configs
See `radar_config/` subdirs (e.g., `config_3rx_5m`: 3 RX antennas, 5m max range). Edit `config.json` for chirps.

## Testing
Run `cargo test` for DSP units. Hardware tests: Monitor logs for detections.

For issues: Check SPI timing; ensure Pi GPIO pins match.