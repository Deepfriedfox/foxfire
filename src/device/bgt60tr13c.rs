use std::error::Error;
use std::fs::File;
use std::io::{BufWriter, Read, Write};
use std::path::Path;
use std::sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}};
use std::thread;
use std::time::Duration;
use crossbeam_channel::{bounded, Sender, Receiver};
use log::{info, debug, error};
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};
use rppal::gpio::{Gpio, InputPin, OutputPin};
use std::ops::Deref;
use crate::bgt60tr13c_const::bgt60tr13c_const::*;

// Struct representing the BGT60TR13C radar sensor device.
pub struct Bgt60tr13c {
    // SPI interface for communication with the sensor.
    spi: Arc<Mutex<Spi>>,
    // Reset pin for hardware reset control.
    rst: Arc<Mutex<OutputPin>>,
    // Interrupt pin to signal FIFO data availability.
    irq: Arc<Mutex<InputPin>>,
    // Optional path to the register configuration file.
    register_config_file_name: Arc<Mutex<Option<String>>>,
    // Number of samples per SPI burst (optional, set via set_fifo_parameters).
    fifo_num_sampler_per_burst: Arc<Mutex<Option<u32>>>,
    // FIFO threshold for IRQ trigger (optional, set via set_fifo_parameters).
    fifo_num_samples_irq: Arc<Mutex<Option<u32>>>,
    // Number of samples per frame (optional, set via set_fifo_parameters).
    fifo_num_samples_per_frame: Arc<Mutex<Option<u32>>>,
    // Computed number of samples per frame (derived from fifo_num_samples_per_frame).
    num_samples_per_frame: Arc<Mutex<u32>>,
    // Computed bytes per frame (based on num_samples_per_frame).
    num_bytes_per_frame: Arc<Mutex<u32>>,
    // Computed samples per burst (derived from fifo_num_sampler_per_burst).
    num_sampler_per_burst: Arc<Mutex<u32>>,
    // Last read GSR0 register value for error checking.
    last_gsr_reg: Arc<Mutex<u8>>,
    // Channel to send complete frames to the host application.
    frame_buffer: Sender<Vec<u8>>,
    // Buffer to accumulate FIFO data until a full frame is collected.
    sub_frame_buffer: Arc<Mutex<Vec<u8>>>,
    // Handle for the data collection thread.
    data_collection_thread: Mutex<Option<thread::JoinHandle<()>>>,
    // Flag to stop the data collection thread.
    data_collection_stop: Arc<AtomicBool>,
    // Version identifier for frame metadata.
    version: u32,
    // Sequence number for frame ordering.
    seq: Arc<Mutex<u32>>,
    // Optional file descriptor for writing frames to disk (currently unused).
    file_fd: Arc<Mutex<Option<BufWriter<File>>>>,
}

// Wrapper struct for thread-safe access to Bgt60tr13c.
#[derive(Clone)]
pub struct Device(Arc<Bgt60tr13c>);

impl Deref for Device {
    type Target = Bgt60tr13c;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl Device {
    /// Creates a new BGT60TR13C device instance and initializes hardware.
    ///
    /// # Arguments
    /// * `spi_bus` - SPI bus to use (e.g., Bus::Spi0).
    /// * `spi_dev` - SPI slave select (e.g., SlaveSelect::Ss0).
    /// * `spi_speed` - SPI clock speed in Hz.
    /// * `rst_pin` - GPIO pin number for reset.
    /// * `irq_pin` - GPIO pin number for interrupt.
    /// * `version` - Version number for frame metadata.
    ///
    /// # Returns
    /// A tuple containing the `Device` instance and a `Receiver` for frame data.
    pub fn new(
        spi_bus: Bus,
        spi_dev: SlaveSelect,
        spi_speed: u32,
        rst_pin: u8,
        irq_pin: u8,
        version: u32,
    ) -> Result<(Self, Receiver<Vec<u8>>), Box<dyn Error>> {
        // Initialize SPI interface with specified bus, slave select, and speed.
        let spi = Arc::new(Mutex::new(Spi::new(spi_bus, spi_dev, spi_speed, Mode::Mode0)?));
        // Initialize GPIO for reset and interrupt pins.
        let gpio = Gpio::new()?;
        let rst = Arc::new(Mutex::new(gpio.get(rst_pin)?.into_output()));
        let irq = Arc::new(Mutex::new(gpio.get(irq_pin)?.into_input()));
        // Create a bounded channel for sending frame data to the host.
        let (tx, rx) = bounded(256);
        // Flag to control data collection thread termination.
        let stop = Arc::new(AtomicBool::new(false));
        // Construct the inner device struct with initialized fields.
        let inner = Bgt60tr13c {
            spi,
            rst,
            irq,
            register_config_file_name: Arc::new(Mutex::new(None)),
            fifo_num_sampler_per_burst: Arc::new(Mutex::new(None)),
            fifo_num_samples_irq: Arc::new(Mutex::new(None)),
            fifo_num_samples_per_frame: Arc::new(Mutex::new(None)),
            num_samples_per_frame: Arc::new(Mutex::new(0)),
            num_bytes_per_frame: Arc::new(Mutex::new(0)),
            num_sampler_per_burst: Arc::new(Mutex::new(0)),
            last_gsr_reg: Arc::new(Mutex::new(0)),
            frame_buffer: tx,
            sub_frame_buffer: Arc::new(Mutex::new(Vec::new())),
            data_collection_thread: Mutex::new(None),
            data_collection_stop: stop,
            version,
            seq: Arc::new(Mutex::new(0)),
            file_fd: Arc::new(Mutex::new(None)),
        };
        let device = Self(Arc::new(inner));
        // Perform hardware reset to ensure clean device state.
        device.hard_reset()?;
        // Wait for device stabilization after reset.
        thread::sleep(Duration::from_millis(50));
        Ok((device, rx))
    }

    /// Starts the data collection thread for continuous radar frame acquisition.
    ///
    /// Stops any existing thread, performs a soft reset, loads configuration,
    /// and begins reading FIFO data triggered by IRQ signals.
    pub fn start(&self) -> Result<(), Box<dyn Error>> {
        debug!("Starting BGT60TR13C data collection");
        // Stop any existing data collection thread.
        if self.data_collection_thread.lock().unwrap().is_some() {
            debug!("Stopping existing thread");
            self.stop();
        }
        // Reset stop flag to allow thread execution.
        self.data_collection_stop.store(false, Ordering::SeqCst);
        let device_clone = self.clone();
        // Spawn a new thread for continuous data collection.
        let thread = thread::spawn(move || {
            debug!("Data collection thread started");
            // Perform software reset to clear internal state.
            device_clone.soft_reset(BGT60TRXX_RESET_SW).unwrap();
            // Load register configuration from file.
            device_clone.__load_register_config_file().unwrap();
            // Configure FIFO parameters (samples per frame, IRQ threshold, etc.).
            device_clone.__set_fifo_parameters().unwrap();
            // Main loop: continue until stop signal is received.
            while !device_clone.data_collection_stop.load(Ordering::SeqCst) {
                // Reset sequence number for new frame batch.
                *device_clone.seq.lock().unwrap() = 0;
                // Trigger frame acquisition by setting FRAME_START bit.
                let status = device_clone.__get_reg(BGT60TRXX_REG_MAIN).unwrap();
                let tmp = status | BGT60TRXX_REG_MAIN_FRAME_START_MSK;
                device_clone.__set_reg(BGT60TRXX_REG_MAIN, tmp).unwrap();
                let mut error_flag = false;
                // Inner loop: process frames until error or stop.
                while !device_clone.data_collection_stop.load(Ordering::SeqCst) {
                    if error_flag {
                        break;
                    }
                    // Wait for IRQ signal indicating FIFO data is ready.
                    let mut irq_guard = device_clone.irq.lock().unwrap();
                    loop {
                        if irq_guard.is_high() {
                            break;
                        }
                        drop(irq_guard);
                        if device_clone.data_collection_stop.load(Ordering::SeqCst) {
                            return;
                        }
                        // Brief sleep to reduce CPU usage while polling IRQ.
                        thread::sleep(Duration::from_micros(1));
                        irq_guard = device_clone.irq.lock().unwrap();
                    }
                    drop(irq_guard);
                    // Process FIFO data until empty or error occurs.
                    loop {
                        // Check FIFO status for errors or emptiness.
                        let (fof_err, _full, _cref, empty, fuf_err, spi_burst_err, clk_num_err, fill_status) =
                            device_clone.__check_fifo_status().unwrap();
                        if fof_err != 0 || fuf_err != 0 || spi_burst_err != 0 || clk_num_err != 0 {
                            error!(
                                "FIFO errors detected: FOF_ERR: {}, FUF_ERR: {}, SPI_BURST_ERR: {}, CLK_NUM_ERR: {}",
                                fof_err, fuf_err, spi_burst_err, clk_num_err
                            );
                            // Reset FIFO to recover from errors.
                            let _ = device_clone.reset_fifo();
                            error_flag = true;
                            break;
                        }
                        if empty == 1 {
                            break; // FIFO is empty, move to next frame.
                        }
                        // Read available samples (limited by burst size or FIFO fill level).
                        let actual_samples = std::cmp::min(
                            *device_clone.num_sampler_per_burst.lock().unwrap(),
                            fill_status * 2,
                        );
                        let fifo_data = device_clone.__get_fifo_data(actual_samples);
                        match fifo_data {
                            Ok(value) => {
                                // Append FIFO data to sub-frame buffer.
                                let mut sub_frame_buffer = device_clone.sub_frame_buffer.lock().unwrap();
                                sub_frame_buffer.extend(value);
                                // Process complete frames from the buffer.
                                while sub_frame_buffer.len() >= *device_clone.num_bytes_per_frame.lock().unwrap() as usize {
                                    let frame = sub_frame_buffer
                                        .drain(0..*device_clone.num_bytes_per_frame.lock().unwrap() as usize)
                                        .collect::<Vec<u8>>();
                                    let mut full_frame = vec![];
                                    if device_clone.version == 0 {
                                        // Prepend metadata: version, sequence number, and frame length.
                                        full_frame.extend(device_clone.version.to_le_bytes());
                                        full_frame.extend(device_clone.seq.lock().unwrap().to_le_bytes());
                                        full_frame.extend((frame.len() as u32).to_le_bytes());
                                        full_frame.extend(frame);
                                        *device_clone.seq.lock().unwrap() += 1;
                                    }
                                    // Send complete frame to the host via channel.
                                    device_clone.frame_buffer.send(full_frame).unwrap();
                                }
                            }
                            Err(e) => {
                                println!("Error reading FIFO data: {:?}", e);
                            }
                        }
                    }
                }
            }
            debug!("Data collection thread stopped");
        });
        // Store the thread handle for later cleanup.
        *self.data_collection_thread.lock().unwrap() = Some(thread);
        Ok(())
    }
}

impl Bgt60tr13c {
    /// Sets FIFO parameters for frame and burst configuration.
    ///
    /// # Arguments
    /// * `num_samples_per_frame` - Total samples per frame.
    /// * `num_samples_irq` - FIFO threshold for triggering IRQ.
    /// * `num_sampler_per_burst` - Samples to read per SPI burst.
    pub fn set_fifo_parameters(&self, num_samples_per_frame: u32, num_samples_irq: u32, num_sampler_per_burst: u32) {
        // Store FIFO configuration parameters.
        *self.fifo_num_samples_per_frame.lock().unwrap() = Some(num_samples_per_frame);
        *self.fifo_num_samples_irq.lock().unwrap() = Some(num_samples_irq);
        *self.fifo_num_sampler_per_burst.lock().unwrap() = Some(num_sampler_per_burst);
    }

    // Configures FIFO parameters and sets the IRQ threshold in the SFCTL register.
    fn __set_fifo_parameters(&self) -> Result<(), Box<dyn Error>> {
        // Log configured parameters.
        info!("num_samples_per_frame: {}", self.fifo_num_samples_per_frame.lock().unwrap().unwrap());
        info!("num_samples_irq: {}", self.fifo_num_samples_irq.lock().unwrap().unwrap());
        info!("num_sampler_per_burst: {}", self.fifo_num_sampler_per_burst.lock().unwrap().unwrap());
        // Update internal state with validated parameters.
        *self.num_samples_per_frame.lock().unwrap() = self.fifo_num_samples_per_frame.lock().unwrap().unwrap();
        // Calculate bytes per frame (assuming 12-bit samples, 2 samples per 3-byte word).
        *self.num_bytes_per_frame.lock().unwrap() = (*self.num_samples_per_frame.lock().unwrap() >> 1) * 3;
        *self.num_sampler_per_burst.lock().unwrap() = self.fifo_num_sampler_per_burst.lock().unwrap().unwrap();
        // Set FIFO IRQ threshold.
        self.__set_fifo_limit(self.fifo_num_samples_irq.lock().unwrap().unwrap())?;
        Ok(())
    }

    // Writes a value to a specified register via SPI.
    fn __set_reg(&self, reg_addr: u32, data: u32) -> Result<u32, Box<dyn Error>> {
        let spi = self.spi.lock().unwrap();
        // Construct SPI command with register address, write operation, and data.
        let temp = (reg_addr << BGT60TRXX_SPI_REGADR_POS) & BGT60TRXX_SPI_REGADR_MSK
            | BGT60TRXX_SPI_WR_OP_MSK
            | (data << BGT60TRXX_SPI_DATA_POS) & BGT60TRXX_SPI_DATA_MSK;
        let tx_data = temp.to_be_bytes();
        let mut rx_data = [0u8; 4];
        // Perform SPI transfer.
        spi.transfer(&mut rx_data, &tx_data)?;
        let result = u32::from_be_bytes(rx_data);
        // Store GSR0 register for error checking.
        *self.last_gsr_reg.lock().unwrap() = rx_data[0];
        Ok(result)
    }

    // Reads a value from a specified register via SPI.
    fn __get_reg(&self, reg_addr: u32) -> Result<u32, Box<dyn Error>> {
        let spi = self.spi.lock().unwrap();
        // Construct SPI read command with register address.
        let temp = (reg_addr << BGT60TRXX_SPI_REGADR_POS) & BGT60TRXX_SPI_REGADR_MSK;
        let tx_data = temp.to_be_bytes();
        let mut rx_data = [0u8; 4];
        // Perform SPI transfer.
        spi.transfer(&mut rx_data, &tx_data)?;
        let result = u32::from_be_bytes(rx_data);
        // Store GSR0 register for error checking.
        *self.last_gsr_reg.lock().unwrap() = rx_data[0];
        Ok(result)
    }

    // Reads FIFO data for the specified number of samples via SPI burst mode.
    fn __get_fifo_data(&self, num_samples: u32) -> Result<Vec<u8>, Box<dyn Error>> {
        let mut fifo_data = vec![];
        // Validate sample count (must be non-zero, even, and within FIFO size).
        if num_samples > 0 && (num_samples >> 1) <= BGT60TRXX_REG_FSTAT_TR13C_FIFO_SIZE && num_samples % 2 == 0 {
            // Check if prefix header is enabled.
            let sfctl = self.__get_reg(BGT60TRXX_REG_SFCTL)?;
            let prefix_enabled = (sfctl & BGT60TRXX_REG_SFCTL_PREFIX_EN_MSK) != 0;
            let prefix_bytes = if prefix_enabled { 3 } else { 0 };
            // Calculate words to read (2 samples per 24-bit word).
            let words = num_samples >> 1;
            let len_val = 0; // Unbounded burst mode to avoid length errors.
            // Construct SPI burst read command for FIFO.
            let burst_cmd = BGT60TRXX_SPI_BURST_MODE_CMD
                | (BGT60TRXX_REG_FIFO_TR13C << BGT60TRXX_SPI_BURST_MODE_SADR_POS) & BGT60TRXX_SPI_BURST_MODE_SADR_MSK
                | BGT60TRXX_SPI_BURST_MODE_RWB_MSK
                | (len_val << BGT60TRXX_SPI_BURST_MODE_LEN_POS) & BGT60TRXX_SPI_BURST_MODE_LEN_MSK;
            let burst_cmd_bytes = burst_cmd.to_be_bytes();
            let mut tx_data = burst_cmd_bytes.to_vec();
            // Allocate buffer for burst data (command + words + prefix).
            tx_data.extend(vec![0u8; words as usize * 3 + prefix_bytes as usize]);
            let mut rx_data = vec![0u8; tx_data.len()];
            // Perform SPI burst transfer.
            self.spi.lock().unwrap().transfer(&mut rx_data, &tx_data)?;
            // Store GSR0 for error checking.
            *self.last_gsr_reg.lock().unwrap() = rx_data[0];
            // Check for SPI errors.
            if self.check_gsr_reg() != RET_VAL_OK {
                error!("GSR Error Detected");
                self.print_gsr_reg();
                self.print_fifo_status();
            } else {
                // Extract raw data, skipping command and prefix bytes.
                let data_start = 4 + prefix_bytes as usize;
                fifo_data = rx_data[data_start..].to_vec();
            }
        } else {
            error!("Invalid num_samples: {}", num_samples);
        }
        Ok(fifo_data)
    }

    /// Verifies the chip ID to confirm BGT60TR13C device.
    ///
    /// # Returns
    /// `RET_VAL_OK` if the chip is BGT60TR13C, `RET_VAL_ERR` otherwise.
    pub fn check_chip_id(&self) -> i32 {
        // Retry up to 3 times to read chip ID.
        for _ in 0..3 {
            match self.__get_reg(BGT60TRXX_REG_CHIP_ID) {
                Ok(chip_id) => {
                    info!("Chip ID: 0x{:08x}", chip_id);
                    let chip_id_digital = (chip_id & BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_MSK) >> BGT60TRXX_REG_CHIP_ID_DIGITAL_ID_POS;
                    let chip_id_rf = (chip_id & BGT60TRXX_REG_CHIP_ID_RF_ID_MSK) >> BGT60TRXX_REG_CHIP_ID_RF_ID_POS;
                    if chip_id_digital == 3 && chip_id_rf == 3 {
                        info!("This chip is BGT60TR13C");
                        return RET_VAL_OK;
                    } else {
                        info!("Chip is NOT BGT60TR13C");
                    }
                }
                Err(e) => {
                    error!("Failed to read CHIP_ID: {:?}", e);
                }
            }
        }
        RET_VAL_ERR
    }

    /// Sets the path to the register configuration file.
    ///
    /// # Arguments
    /// * `file_name` - Path to the configuration file.
    pub fn set_register_config_file(&self, file_name: String) {
        *self.register_config_file_name.lock().unwrap() = Some(file_name);
    }

    // Loads and applies register configuration from a file.
    fn __load_register_config_file(&self) -> Result<(), Box<dyn Error>> {
        if let Some(file_name) = &*self.register_config_file_name.lock().unwrap() {
            info!("Loading radar config file: {}", file_name);
            let path = Path::new(file_name);
            let mut file = File::open(path)?;
            let mut contents = String::new();
            file.read_to_string(&mut contents)?;
            // Parse each line of the config file (format: <cmd> <addr> <value>).
            for line in contents.lines() {
                let line = line.trim();
                let parts: Vec<&str> = line.split_whitespace().collect();
                if parts.len() == 3 {
                    // Parse register address and value (hex format).
                    let addr = if parts[1].starts_with("0x") {
                        &parts[1][2..].trim()
                    } else {
                        parts[1].trim()
                    };
                    let address = u32::from_str_radix(addr, 16)?;
                    let val = if parts[2].starts_with("0x") {
                        &parts[2][2..].trim()
                    } else {
                        parts[2].trim()
                    };
                    let mut value = u32::from_str_radix(val, 16)?;
                    // Adjust SFCTL for high-speed MISO if SPI clock > 20MHz.
                    if address == BGT60TRXX_REG_SFCTL {
                        if self.spi.lock().unwrap().clock_speed()? > 20_000_000 {
                            value |= BGT60TRXX_REG_SFCTL_MISO_HS_READ_MSK;
                        } else {
                            value &= !BGT60TRXX_REG_SFCTL_MISO_HS_READ_MSK;
                        }
                    }
                    self.__set_reg(address, value)?;
                } else {
                    return Err(format!("Line format is incorrect: {}.", line).into());
                }
            }
            // Disable PREFIX_EN, LFSR_EN, and FIFO_LP_MODE for consistent data format.
            let mut sfctl = self.__get_reg(BGT60TRXX_REG_SFCTL)?;
            sfctl &= !BGT60TRXX_REG_SFCTL_PREFIX_EN_MSK;
            sfctl &= !BGT60TRXX_REG_SFCTL_LFSR_EN_MSK;
            sfctl &= !BGT60TRXX_REG_SFCTL_FIFO_LP_MODE_MSK;
            self.__set_reg(BGT60TRXX_REG_SFCTL, sfctl)?;
            info!("SFCTL final: 0x{:08X}", self.__get_reg(BGT60TRXX_REG_SFCTL)?);
            Ok(())
        } else {
            Err("No register config file set.".into())
        }
    }

    /// Stops the data collection thread and performs a soft reset.
    ///
    /// # Returns
    /// `RET_VAL_OK` on success, `RET_VAL_ERR` on reset failure.
    pub fn stop(&self) -> i32 {
        // Perform soft reset to halt device operation.
        if self.soft_reset(BGT60TRXX_RESET_SW).is_err() {
            return RET_VAL_ERR;
        }
        // Signal the data collection thread to stop.
        self.data_collection_stop.store(true, Ordering::SeqCst);
        // Join the thread to ensure clean shutdown.
        if let Some(thread) = self.data_collection_thread.lock().unwrap().take() {
            thread.join().unwrap();
        }
        // Flush and close any open file descriptor (unused in this version).
        if let Some(mut fd) = self.file_fd.lock().unwrap().take() {
            fd.flush().unwrap();
        }
        // Clear partial frame data.
        *self.sub_frame_buffer.lock().unwrap() = vec![];
        RET_VAL_OK
    }

    // Performs a soft reset of the device with the specified reset type.
    fn soft_reset(&self, reset_type: u32) -> Result<(), Box<dyn Error>> {
        // Clear sub-frame buffer to discard partial data.
        *self.sub_frame_buffer.lock().unwrap() = vec![];
        // Set reset bit in MAIN register.
        let status = self.__get_reg(BGT60TRXX_REG_MAIN)?;
        let tmp = status | reset_type;
        self.__set_reg(BGT60TRXX_REG_MAIN, tmp)?;
        // Wait for reset bit to self-clear (timeout after 500ms).
        let mut time_out_cnt = 0;
        let n = 50;
        for _ in 0..n {
            thread::sleep(Duration::from_millis(10));
            let status = self.__get_reg(BGT60TRXX_REG_MAIN)?;
            info!("Main reg status: {}", status);
            if status & reset_type == 0 {
                break;
            }
            time_out_cnt += 1;
        }
        if time_out_cnt == n {
            Err("Soft reset timeout!".into())
        } else {
            // Additional delay for stabilization.
            thread::sleep(Duration::from_millis(10));
            Ok(())
        }
    }

    // Performs a hardware reset using the reset pin.
    fn hard_reset(&self) -> Result<(), Box<dyn Error>> {
        let mut rst = self.rst.lock().unwrap();
        // Toggle reset pin: high -> low -> high with delays.
        rst.set_high();
        thread::sleep(Duration::from_millis(10));
        rst.set_low();
        thread::sleep(Duration::from_millis(10));
        rst.set_high();
        // Extra delay for device stabilization.
        thread::sleep(Duration::from_millis(50));
        Ok(())
    }

    // Reads FIFO status register and extracts status flags.
    fn __check_fifo_status(&self) -> Result<(u32, u32, u32, u32, u32, u32, u32, u32), Box<dyn Error>> {
        let status = self.__get_reg(BGT60TRXX_REG_FSTAT_TR13C)?;
        // Extract individual status bits.
        let fof_err = (status & BGT60TRXX_REG_FSTAT_FOF_ERR_MSK) >> BGT60TRXX_REG_FSTAT_FOF_ERR_POS;
        let full = (status & BGT60TRXX_REG_FSTAT_FULL_MSK) >> BGT60TRXX_REG_FSTAT_FULL_POS;
        let cref = (status & BGT60TRXX_REG_FSTAT_CREF_MSK) >> BGT60TRXX_REG_FSTAT_CREF_POS;
        let empty = (status & BGT60TRXX_REG_FSTAT_EMPTY_MSK) >> BGT60TRXX_REG_FSTAT_EMPTY_POS;
        let fuf_err = (status & BGT60TRXX_REG_FSTAT_FUF_ERR_MSK) >> BGT60TRXX_REG_FSTAT_FUF_ERR_POS;
        let spi_burst_err = (status & BGT60TRXX_REG_FSTAT_SPI_BURST_ERR_MSK) >> BGT60TRXX_REG_FSTAT_SPI_BURST_ERR_POS;
        let clk_num_err = (status & BGT60TRXX_REG_FSTAT_CLK_NUM_ERR_MSK) >> BGT60TRXX_REG_FSTAT_CLK_NUM_ERR_POS;
        let fill_status = (status & BGT60TRXX_REG_FSTAT_FILL_STATUS_MSK) >> BGT60TRXX_REG_FSTAT_FILL_STATUS_POS;
        Ok((fof_err, full, cref, empty, fuf_err, spi_burst_err, clk_num_err, fill_status))
    }

    // Sets the FIFO IRQ threshold in the SFCTL register.
    fn __set_fifo_limit(&self, num_samples: u32) -> Result<i32, Box<dyn Error>> {
        // Validate sample count (non-zero, even, within FIFO size).
        if num_samples > 0 && (num_samples >> 1) <= BGT60TRXX_REG_FSTAT_TR13C_FIFO_SIZE && num_samples % 2 == 0 {
            // Update SFCTL register with new FIFO threshold.
            let mut tmp = self.__get_reg(BGT60TRXX_REG_SFCTL)?;
            tmp &= !BGT60TRXX_REG_SFCTL_FIFO_CREF_MSK;
            tmp |= ((num_samples >> 1) << BGT60TRXX_REG_SFCTL_FIFO_CREF_POS) & BGT60TRXX_REG_SFCTL_FIFO_CREF_MSK;
            self.__set_reg(BGT60TRXX_REG_SFCTL, tmp)?;
            Ok(RET_VAL_OK)
        } else {
            Err("Invalid num_samples.".into())
        }
    }

    /// Prints FIFO status flags for debugging.
    pub fn print_fifo_status(&self) {
        let (fof_err, _full, _cref, _empty, fuf_err, spi_burst_err, clk_num_err, _fill_status) =
            self.__check_fifo_status().unwrap_or((0, 0, 0, 0, 0, 0, 0, 0));
        info!(
            "FOF_ERR: {}, FUF_ERR: {}, SPI_BURST_ERR: {}, CLK_NUM_ERR: {}",
            fof_err, fuf_err, spi_burst_err, clk_num_err
        );
    }

    /// Prints GSR0 register flags for debugging.
    pub fn print_gsr_reg(&self) {
        let last_gsr_reg = *self.last_gsr_reg.lock().unwrap();
        info!(
            "GSR FIFO OVERFLOW/UNDERFLOW ERROR: {}",
            if last_gsr_reg & BGT60TRXX_REG_GSR0_FOU_ERR_MSK != 0 { 1 } else { 0 }
        );
        info!(
            "GSR MISO HS: {}",
            if last_gsr_reg & BGT60TRXX_REG_GSR0_MISO_HS_READ_MSK != 0 { 1 } else { 0 }
        );
        info!(
            "GSR SPI BURST ERR: {}",
            if last_gsr_reg & BGT60TRXX_REG_GSR0_SPI_BURST_ERR_MSK != 0 { 1 } else { 0 }
        );
        info!(
            "GSR CLOCK NUMBER ERR: {}",
            if last_gsr_reg & BGT60TRXX_REG_GSR0_CLK_NUM_ERR_MSK != 0 { 1 } else { 0 }
        );
    }

    /// Checks GSR0 register for errors.
    ///
    /// # Returns
    /// `RET_VAL_OK` if no errors, `RET_VAL_ERR` if errors are detected.
    pub fn check_gsr_reg(&self) -> i32 {
        let last_gsr_reg = *self.last_gsr_reg.lock().unwrap();
        if last_gsr_reg & (BGT60TRXX_REG_GSR0_FOU_ERR_MSK | BGT60TRXX_REG_GSR0_SPI_BURST_ERR_MSK | BGT60TRXX_REG_GSR0_CLK_NUM_ERR_MSK) != 0 {
            RET_VAL_ERR
        } else {
            RET_VAL_OK
        }
    }

    // Resets the FIFO and clears partial frame data.
    fn reset_fifo(&self) -> Result<(), Box<dyn Error>> {
        // Set FIFO reset bit in MAIN register.
        let status = self.__get_reg(BGT60TRXX_REG_MAIN)?;
        let tmp = status | BGT60TRXX_RESET_FIFO;
        self.__set_reg(BGT60TRXX_REG_MAIN, tmp)?;
        // Wait for reset bit to self-clear (timeout after 10ms).
        let mut time_out_cnt = 0;
        let n = 10;
        for _ in 0..n {
            thread::sleep(Duration::from_millis(1));
            let status = self.__get_reg(BGT60TRXX_REG_MAIN)?;
            if status & BGT60TRXX_RESET_FIFO == 0 {
                break;
            }
            time_out_cnt += 1;
        }
        if time_out_cnt == n {
            return Err("FIFO reset timeout!".into());
        }
        // Clear sub-frame buffer to discard partial data.
        *self.sub_frame_buffer.lock().unwrap() = vec![];
        // Log status for debugging.
        self.print_fifo_status();
        self.print_gsr_reg();
        Ok(())
    }
}

impl Drop for Bgt60tr13c {
    // Ensures the device is stopped when dropped.
    fn drop(&mut self) {
        self.stop();
    }
}