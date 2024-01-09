//!
//! Executable Tests for FIFO Filling and addressing on the raspberry pi 4
//! 
//! The main point in this test is to figure out how the FIFO addressing mode works.
//! 
//! Wiring (Raspberry Pi -> RFM9x LoRa):
//! 3v3 Power (17) -> VIN
//! Ground (20) -> GND
//! _ -> EN
//! _ -> G0
//! GPIO 11 (23) -> SCK
//! GPIO 9 (21) -> MISO
//! GPIO 10 (19) -> MOSI
//! GPIO 8 (24) -> CS
//! GPIO 25 (22) -> RESET
//! 

use rppal::spi::{Spi, Bus, SlaveSelect, Mode};
use rppal::gpio::Gpio;
use rppal::hal::Delay;

use sx127::config::Config;
use sx127::Radio;
use sx127::error::Error;

#[test]
fn test_no_intermittant_addresses() {
    let config = Config {
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
    let reset = gpio.get(25u8).unwrap().into_output();
    let mut delay = Delay::new();

    // Initialize the Radio Driver
    let radio = Radio::new(config, Some(cs), reset, &mut delay, &mut spi);

    let mut radio = match radio {
        Ok(radio) => radio,
        Err(err) => match err {
            Error::DataExceedsFifoSize => panic!("Data Exceeds FIFO Size"),
            Error::GpioError(_) => panic!("Gpio Error Occured"),
            Error::SpiError(_) => panic!("SPI Error Occurred"),
            Error::UnexpectedConfiguration(register, expected, found) => panic!("Unexpected Configuration for Register {}, Expected {}, but found {}", register, expected, found),
            Error::GpioSpiError(_) => panic!("Unexpected GPIO and SPI Error Occurred"),
        },
    };

    // First Test Write
    let mut write_buffer = [0x00; 65];
    write_buffer[0] = 0b1000_0000;
    for i in 0..64 {
        write_buffer[i+1] = (i+1) as u8;
    }

    if radio.transfer_data_fifo(&mut write_buffer, &mut spi).is_err() {
        panic!("Unable to Write to FIFO");
    }

    let mut read_buffer = [0x00; 65];
    
    if radio.transfer_data_fifo(&mut read_buffer, &mut spi).is_ok() {
        println!("Wrote {:?} To Fifo", read_buffer);
    } else {
        panic!("Unable to Read from FIFO");
    }
}