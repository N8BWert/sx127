//!
//! Executable Tests for Configuration runnable on the raspberry pi 4.
//! 
//! Wiring (Raspberry Pi -> RFM9x LoRa):
//! 3v3 Power (17) -> VIM
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
fn test_write_sample_configuration() {
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
        pa_ramp: Some(sx127::config::PaRamp::PA50),
    };

    let gpio = Gpio::new().unwrap();
    let mut spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
    let cs = gpio.get(8u8).unwrap().into_output();
    let reset = gpio.get(25u8).unwrap().into_output();
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

    for register in 0x00..=0x41 {
        let result = radio.spi_read(register, &mut spi);

        if let Ok(result) = result {
            println!("Read {:#010b} from Register {:#x}", result, register);
        } else {
            println!("Unable to Read from Register {:#x}", register);
        }
    }
}