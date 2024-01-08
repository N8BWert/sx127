//!
//! Executable Tests for State runnable on the raspberry pi 4.
//! 
//! Wiring (Raspberry Pi -> RFM9x LoRa):
//! 3v3 Power (1) -> VIM
//! Ground (6) -> GND
//! _ -> EN
//! _ -> G0
//! GPIO 11 (23) -> SCK
//! GPIO 9 (21) -> MISO
//! GPIO 10 (19) -> MOSI
//! GPIO 8 (24) -> CS
//! GPIO 21 (40) -> RESET
//! 

use std::time::Duration;
use std::thread;

use rppal::spi::{Spi, Bus, SlaveSelect, Mode};
use rppal::gpio::Gpio;
use rppal::hal::Delay;

use sx127::config::Config;
use sx127::Radio;
use sx127::error::Error;

#[test]
fn test_to_tx() {
    let sample_configuration = Config{
        modulation_type: sx127::config::ModulationType::FSK,
        bitrate: 100_000u32,
        frequency: 915_000,
        rx_timeout: None,
        preamble_size: 1u16,
        sync_word: Some(b"b"),
        crc_on: true,
        address_filtering: sx127::config::AddressFiltering::NodeAddress,
        payload_length: None,
        node_address: Some(0u8),
        broadcast_address: None,
        pa_ramp: None,
    };

    let gpio = Gpio::new().unwrap();
    let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
    let cs = gpio.get(8u8).unwrap().into_output();
    let reset = gpio.get(21u8).unwrap().into_output();
    let mut delay = Delay::new();

    // Initialize the Radio Driver
    let radio = Radio::new(sample_configuration, Some(cs), reset, &mut delay, &mut spi);

    let mut radio = match radio {
        Ok(radio) => radio,
        Err(err) => match err {
            Error::DataExceedsFifoSize => panic!("Data Exceeds FIFO Size"),
            Error::GpioError(_) => panic!("Gpio Error Occurred"),
            Error::SpiError(_) => panic!("SPI Error Occurred"),
            Error::UnexpectedConfiguration(register, expected, found) => panic!("Unexpected Configuration for Register {}, Expected {} but found {}", register, expected, found),
            Error::GpioSpiError(_) => panic!("Unexpected GPIO and SPI Error Occurred"),
        }
    };

    if radio.to_tx(&mut spi).is_err() {
        panic!("Unable to Switch to Tx Mode");
    }

    thread::sleep(Duration::from_micros(60));

    // Read State Register
    let state_register = radio.spi_read(0x01, &mut spi);
    let state_register = match state_register {
        Ok(value) => value,
        Err(_) => panic!("Unable to Read State Register"),
    };

    assert_eq!(state_register & 0b111, 0b011);
}

#[test]
fn test_to_rx() {
    let sample_configuration = Config{
        modulation_type: sx127::config::ModulationType::FSK,
        bitrate: 100_000u32,
        frequency: 915_000,
        rx_timeout: None,
        preamble_size: 1u16,
        sync_word: Some(b"b"),
        crc_on: true,
        address_filtering: sx127::config::AddressFiltering::NodeAddress,
        payload_length: None,
        node_address: Some(0u8),
        broadcast_address: None,
        pa_ramp: None,
    };

    let gpio = Gpio::new().unwrap();
    let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
    let cs = gpio.get(8u8).unwrap().into_output();
    let reset = gpio.get(21u8).unwrap().into_output();
    let mut delay = Delay::new();

    // Initialize the Radio Driver
    let radio = Radio::new(sample_configuration, Some(cs), reset, &mut delay, &mut spi);

    let mut radio = match radio {
        Ok(radio) => radio,
        Err(err) => match err {
            Error::DataExceedsFifoSize => panic!("Data Exceeds FIFO Size"),
            Error::GpioError(_) => panic!("Gpio Error Occurred"),
            Error::SpiError(_) => panic!("SPI Error Occurred"),
            Error::UnexpectedConfiguration(register, expected, found) => panic!("Unexpected Configuration for Register {}, Expected {} but found {}", register, expected, found),
            Error::GpioSpiError(_) => panic!("Unexpected GPIO and SPI Error Occurred"),
        }
    };

    if radio.to_rx(&mut spi).is_err() {
        panic!("Unable to Convert to RX");
    }

    thread::sleep(Duration::from_millis(3));

    let state_register = radio.spi_read(0x01, &mut spi);
    let state_value = match state_register {
        Ok(value) => value,
        Err(_) => panic!("Unable to Read State Register"),
    };

    assert_eq!(state_value & 0b111, 0b101);
}

#[test]
fn rx_to_tx() {
    let sample_configuration = Config{
        modulation_type: sx127::config::ModulationType::FSK,
        bitrate: 100_000u32,
        frequency: 915_000,
        rx_timeout: None,
        preamble_size: 1u16,
        sync_word: Some(b"b"),
        crc_on: true,
        address_filtering: sx127::config::AddressFiltering::NodeAddress,
        payload_length: None,
        node_address: Some(0u8),
        broadcast_address: None,
        pa_ramp: None,
    };

    let gpio = Gpio::new().unwrap();
    let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
    let cs = gpio.get(8u8).unwrap().into_output();
    let reset = gpio.get(21u8).unwrap().into_output();
    let mut delay = Delay::new();

    // Initialize the Radio Driver
    let radio = Radio::new(sample_configuration, Some(cs), reset, &mut delay, &mut spi);

    let mut radio = match radio {
        Ok(radio) => radio,
        Err(err) => match err {
            Error::DataExceedsFifoSize => panic!("Data Exceeds FIFO Size"),
            Error::GpioError(_) => panic!("Gpio Error Occurred"),
            Error::SpiError(_) => panic!("SPI Error Occurred"),
            Error::UnexpectedConfiguration(register, expected, found) => panic!("Unexpected Configuration for Register {}, Expected {} but found {}", register, expected, found),
            Error::GpioSpiError(_) => panic!("Unexpected GPIO and SPI Error Occurred"),
        }
    };

    if radio.to_rx(&mut spi).is_err() {
        panic!("Unable to Switch to Rx");
    }

    thread::sleep(Duration::from_millis(3));

    if radio.to_tx(&mut spi).is_err() {
        panic!("Unable to Switch to Tx");
    }

    thread::sleep(Duration::from_micros(100));

    let state_value = radio.spi_read(0x01, &mut spi);
    let state_value = match state_value {
        Ok(value) => value,
        Err(_) => panic!("Unable to Read State Register"),
    };

    assert_eq!(state_value & 0b111, 0b011);
}

#[test]
fn tx_to_rx() {
    let sample_configuration = Config{
        modulation_type: sx127::config::ModulationType::FSK,
        bitrate: 100_000u32,
        frequency: 915_000,
        rx_timeout: None,
        preamble_size: 1u16,
        sync_word: Some(b"b"),
        crc_on: true,
        address_filtering: sx127::config::AddressFiltering::NodeAddress,
        payload_length: None,
        node_address: Some(0u8),
        broadcast_address: None,
        pa_ramp: None,
    };

    let gpio = Gpio::new().unwrap();
    let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
    let cs = gpio.get(8u8).unwrap().into_output();
    let reset = gpio.get(21u8).unwrap().into_output();
    let mut delay = Delay::new();

    // Initialize the Radio Driver
    let radio = Radio::new(sample_configuration, Some(cs), reset, &mut delay, &mut spi);

    let mut radio = match radio {
        Ok(radio) => radio,
        Err(err) => match err {
            Error::DataExceedsFifoSize => panic!("Data Exceeds FIFO Size"),
            Error::GpioError(_) => panic!("Gpio Error Occurred"),
            Error::SpiError(_) => panic!("SPI Error Occurred"),
            Error::UnexpectedConfiguration(register, expected, found) => panic!("Unexpected Configuration for Register {}, Expected {} but found {}", register, expected, found),
            Error::GpioSpiError(_) => panic!("Unexpected GPIO and SPI Error Occurred"),
        }
    };

    if radio.to_tx(&mut spi).is_err() {
        panic!("Unable to Switch to Tx");
    }

    thread::sleep(Duration::from_micros(100));

    if radio.to_rx(&mut spi).is_err() {
        panic!("Unable to Switch to Rx");
    }

    thread::sleep(Duration::from_millis(3));

    let state_value = radio.spi_read(0x01, &mut spi);
    let state_value = match state_value {
        Ok(value) => value,
        Err(_) => panic!("Unable to Read State Regsiter"),
    };

    assert_eq!(state_value & 0b111, 0b101);
}