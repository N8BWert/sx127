//!
//! Executable Tests for Interrupts on the raspberry pi 4
//! 
//! Wiring (Raspberry Pi -> RFM9x LoRa):
//! 
//! Rx:
//! 3v3 Power (1) -> VIN
//! Ground (6) -> GND
//! _ -> EN
//! GPIO 24 (18) -> G0
//! GPIO 22 (15) -> G4
//! GPIO 11 (23) -> SCK
//! GPIO 9 (21) -> MISO
//! GPIO 10 (19) -> MOSI
//! GPIO 8 (24) -> CS
//! GPIO 25 (22) -> RESET
//! 
//! Tx:
//! 3v3 Power(1) -> VIN
//! Ground (6) -> GND
//! _ -> EN
//! GPIO 13 (33) -> G0
//! GPIO 21 (40) -> SCK
//! GPIO 19 (35) -> MISO
//! GPIO 20 (38) -> MOSI
//! GPIO 16 (36) -> CS
//! GPIO 26 (37) -> RESET
//! 

use std::thread;
use std::time::Duration;

use rppal::spi::{Spi, Bus, SlaveSelect, Mode};
use rppal::gpio::Gpio;
use rppal::hal::Delay;

use sx127::config::*;
use sx127::Radio;
use sx127::error::Error;

#[test]
fn test_rx_timeout_interrupt() {
    // TODO: Figure out what units this is
    let config = Config {
        modulation_type: ModulationType::FSK,
        bitrate: 100_000,
        frequency: 915_000,
        rx_timeout: Some(1600000),
        preamble_size: 1,
        sync_word: Some(b"b"),
        crc_on: true,
        address_filtering: AddressFiltering::NodeAddress,
        payload_length: None,
        node_address: Some(0u8),
        broadcast_address: None,
        pa_ramp: Some(PaRamp::PA50),
    };

    let gpio = Gpio::new().unwrap();
    let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
    let cs = gpio.get(8u8).unwrap().into_output();
    let reset = gpio.get(21u8).unwrap().into_output();
    let mut delay = Delay::new();

    // Initialize the Radio Driver
    let radio = Radio::new(config, Some(cs), reset, &mut delay, &mut spi);

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

    let mut timeout_interrupt_pin = gpio.get(22u8).unwrap().into_input();
    timeout_interrupt_pin.set_async_interrupt(rppal::gpio::Trigger::RisingEdge, move |_| {
        println!("Received Interrupt From Rx Timeout");
    }).unwrap();

    if radio.to_rx(&mut spi).is_err() {
        panic!("Unable to Set to Rx");
    }

    // Wait until sleeps
    thread::sleep(Duration::from_secs(5));
}

#[test]
fn test_tx_complete_interrupt() {
    let config = Config {
        modulation_type: ModulationType::FSK,
        bitrate: 100_000,
        frequency: 915_000,
        rx_timeout: None,
        preamble_size: 1,
        sync_word: Some(b"b"),
        crc_on: false,
        address_filtering: AddressFiltering::NodeAddress,
        payload_length: None,
        node_address: Some(0u8),
        broadcast_address: None,
        pa_ramp: Some(PaRamp::PA50)
    };

    let gpio = Gpio::new().unwrap();
    let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
    let cs = gpio.get(8u8).unwrap().into_output();
    let reset = gpio.get(25u8).unwrap().into_output();
    let mut delay = Delay::new();

    // Initialize the Radio Driver
    let radio = Radio::new(config, Some(cs), reset, &mut delay, &mut spi);

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

    let mut send_interrupt = gpio.get(24u8).unwrap().into_input();
    send_interrupt.set_async_interrupt(rppal::gpio::Trigger::RisingEdge, move |_| {
        println!("Data Sent");
    }).unwrap();

    if radio.send_data(b"Hello World", &mut spi).is_err() {
        panic!("Unable to Send Data");
    }
    println!("Sending Data");

    thread::sleep(Duration::from_secs(3));
}

#[test]
fn test_rx_interrupt() {
    let rx_config = Config {
        modulation_type: ModulationType::FSK,
        bitrate: 100_000,
        frequency: 915_000,
        rx_timeout: None,
        preamble_size: 1,
        sync_word: Some(b"b"),
        crc_on: true,
        address_filtering: AddressFiltering::NONE,
        payload_length: None,
        node_address: None,
        broadcast_address: None,
        pa_ramp: Some(PaRamp::PA50),
    };

    let tx_config = Config {
        modulation_type: ModulationType::FSK,
        bitrate: 100_000,
        frequency: 915_000,
        rx_timeout: None,
        preamble_size: 1,
        sync_word: Some(b"b"),
        crc_on: true,
        address_filtering: AddressFiltering::NONE,
        payload_length: None,
        node_address: None,
        broadcast_address: None,
        pa_ramp: Some(PaRamp::PA50),
    };

    let gpio = Gpio::new().unwrap();
    let mut rx_spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
    let mut tx_spi = Spi::new(Bus::Spi1, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
    let rx_cs = gpio.get(8u8).unwrap().into_output();
    let tx_cs = gpio.get(16u8).unwrap().into_output();
    let rx_reset = gpio.get(25u8).unwrap().into_output();
    let tx_reset = gpio.get(26u8).unwrap().into_output();
    let mut rx_delay = Delay::new();
    let mut tx_delay = Delay::new();

    let rx_radio = Radio::new(rx_config, Some(rx_cs), rx_reset, &mut rx_delay, &mut rx_spi);

    let mut rx_radio = match rx_radio {
        Ok(radio) => radio,
        Err(err) => match err {
            Error::DataExceedsFifoSize => panic!("Data Exceeds FIFO Size"),
            Error::GpioError(_) => panic!("Gpio Error Occurred"),
            Error::SpiError(_) => panic!("SPI Error Occurred"),
            Error::UnexpectedConfiguration(register, expected, found) => panic!("Unexpected Configuration for Register {}, Expected {} but found {}", register, expected, found),
            Error::GpioSpiError(_) => panic!("Unexpected GPIO and SPI Error Occurred"),
        }
    };

    let tx_radio = Radio::new(tx_config, Some(tx_cs), tx_reset, &mut tx_delay, &mut tx_spi);

    let mut tx_radio = match tx_radio {
        Ok(radio) => radio,
        Err(err) => match err {
            Error::DataExceedsFifoSize => panic!("Data Exceeds FIFO Size"),
            Error::GpioError(_) => panic!("Gpio Error Occurred"),
            Error::SpiError(_) => panic!("SPI Error Occurred"),
            Error::UnexpectedConfiguration(register, expected, found) => panic!("Unexpected Configuration for Register {}, Expected {} but found {}", register, expected, found),
            Error::GpioSpiError(_) => panic!("Unexpected GPIO and SPI Error Occurred"),
        }
    };

    let mut rx_interrupt = gpio.get(24u8).unwrap().into_input();
    rx_interrupt.set_async_interrupt(rppal::gpio::Trigger::RisingEdge, move |_| {
        println!("Received Data");
    }).unwrap();

    if rx_radio.to_rx(&mut rx_spi).is_err() {
        panic!("Unable to Set to Rx");
    }

    thread::sleep(Duration::from_secs(1));

    let mut tx_interrupt = gpio.get(13).unwrap().into_input();
    tx_interrupt.set_async_interrupt(rppal::gpio::Trigger::RisingEdge, move |_| {
        println!("Sent Data");
    }).unwrap();

    if tx_radio.send_data(b"Hello World", &mut tx_spi).is_err() {
        panic!("Unable to Send Data");
    }

    thread::sleep(Duration::from_secs(5));

    let mut buffer = [0x00; 65];
    tx_radio.transfer_data_fifo(&mut buffer, &mut tx_spi);
    println!("Buffer: {:?}", buffer);

    let mut buffer = [0x00; 65];
    rx_radio.transfer_data_fifo(&mut buffer, &mut rx_spi);
    println!("Buffer: {:?}", buffer);
}