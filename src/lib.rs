#![allow(unused_assignments)]
#![no_std]
#![crate_type = "lib"]
#![crate_name = "sx127"]

// TODO: Configure DIO4 to 0b10 to get rx timeout interrupt


// State Change Data:
//
// TS_OSC = 250us
// TS_FS = 60us
// TS_TR = 120us
//  FSK - 5us + 1.25 * PaRamp + 1/2 * Tbit (bit time)
//  OOK - 5us + 1/2 * Tbit (bit time)
// TS_RE = 2.33 ms (2.6 kHz) - 63 us (250 kHz) (see page 57)
// TS_HOP = 20-50us per hop (change frequency)
//
// Sleep -> Standby : TS_OSC = 250us
// Standby -> FSTx or FSRx : TS_FS = 60us
// FSTSx -> Transmit : TS_TR = 120us
// FSRx -> Receive : TS_RE
// Tx -> Rx : TS_HOP + TS_RE
// Rx -> Tx : TS_HOP + TS_TR

use core::iter::zip;
use core::marker::PhantomData;

use config::Config;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::{DelayMs, DelayUs};
use embedded_hal::spi::{Mode, Phase, Polarity};

pub mod config;

pub mod error;
use error::Error;

mod register;

#[derive(PartialEq, Eq, PartialOrd, Ord)]
enum RadioMode {
    Standby,
    Rx,
    Tx,
}

pub struct Radio<
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>,
    CS: OutputPin<Error = GPIOE>,
    RESET: OutputPin<Error = GPIOE>,
    DELAY: DelayMs<u8> + DelayUs<u8>,
    GPIOE,
    SPIE> {
    cs: Option<CS>,
    reset: RESET,
    radio_mode: RadioMode,
    phantom: PhantomData<(SPI, DELAY)>,
}

impl<GPIOE, SPIE, SPI, CS, RESET, DELAY> Radio<SPI, CS, RESET, DELAY, GPIOE, SPIE>
    where SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>,
          CS: OutputPin<Error = GPIOE>, RESET: OutputPin<Error = GPIOE>, DELAY: DelayMs<u8> + DelayUs<u8> {
    pub fn new(configuration: Config, cs: Option<CS>, reset: RESET, delay: &mut DELAY, spi: &mut SPI,) -> Result<Self, Error<GPIOE, SPIE>> {
        let mut device = Self {
            cs,
            reset,
            radio_mode: RadioMode::Standby,
            phantom: PhantomData,
        };

        // Write Configuration
        device.write_configuration(configuration, spi, delay)?;

        // Finished Configuration
        Ok(device)
    }

    // Write a given configuration to the radio
    pub fn write_configuration(&mut self, configuration: Config, spi: &mut SPI, delay: &mut DELAY) -> Result<(), Error<GPIOE, SPIE>> {
        // Reset the chip
        delay.delay_ms(10);
        self.reset.set_low().map_err(Error::GpioError)?;
        delay.delay_ms(10);
        self.reset.set_high().map_err(Error::GpioError)?;
        delay.delay_ms(10);

        // Get configuration register values.
        let (registers, values) = configuration.register_values();

        // Write the Values to the Chip's Registers
        for (register, value) in zip(registers, values) {
            if register == 0x00 {
                break;
            }

            self.spi_write(register, value, spi)?;
            // TODO: Tweak the delay
            delay.delay_ms(10);
            let found = self.spi_read(register, spi)?;
            if found != value {
                return Err(Error::UnexpectedConfiguration(register, value, found)); 
            }
        }

        Ok(())
    }

    /// Enqueue Data into the FIFO.
    /// 
    /// Switch to Standby and Enqueue Data.
    /// Then Switch to Tx Mode which will asynchronously empty the FIFO.
    /// When complete, an interrupt will be raised on DIO0.
    pub fn send_data(&mut self, data: &[u8], spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        // Check Size of Data
        if data.len() > 64 {
            return Err(Error::DataExceedsFifoSize);
        }

        // Switch to Standby
        if self.radio_mode != RadioMode::Standby {
            self.to_standby(spi)?;
        }

        // Enqueue the data into the FIFO
        let mut address = 0x00;
        for data in data {
            self.write_fifo(address, *data, spi)?;
            address += 1;
        }

        // Zero the Remaining Bytes
        for addr in address..64 {
            self.write_fifo(addr, 0x00, spi)?;
        }

        // Switch to Tx Mode.
        self.to_tx(spi)?;

        Ok(())
    }

    /// Dequeue Data from the FIFO.
    /// 
    /// Switch to Standby and Dequeue Data.
    ///     It will be known data is in the FIFO, when the DIO0 interrupt is raised.
    /// Then Unset the Interrupt
    pub fn read_data(&mut self, buffer: &mut [u8], spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        // Check SIze of Buffer
        if buffer.len() > 64 {
            return Err(Error::DataExceedsFifoSize);
        }

        // Switch to Standby to Prevent Overwrite
        if self.radio_mode != RadioMode::Standby {
            self.to_standby(spi)?;
        }

        for address in 0..buffer.len() {
            buffer[address] = self.read_fifo(address as u8, spi)?;
        }

        Ok(())
    }

    /// Read Data from the FIFO and then send a response.
    /// 
    /// Steps:
    /// 1. Switch to Standby and Dequeue Data into the buffer.
    /// 3. Enqueue the data into the FIFO.
    /// 4. Switch to Tx Mode.
    pub fn read_data_and_respond(&mut self, buffer: &mut [u8], data: &[u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        // Check Size of Buffer and Data
        if buffer.len() > 64 || data.len() > 64 {
            return Err(Error::DataExceedsFifoSize);
        }

        // Switch to Standby
        if self.radio_mode != RadioMode::Standby {
            self.to_standby(spi)?;
        }

        // Dequeue Data into the buffer
        for address in 0..buffer.len() {
            buffer[address] = self.read_fifo(address as u8, spi)?;
        }

        // Enqueue the data into the FIFO.
        let mut address = 0x00;
        for data in data {
            self.write_fifo(address, *data, spi)?;
            address += 1;
        }
        
        // Zero the Remaining Bytes
        for addr in address..64 {
            self.write_fifo(addr, 0x00, spi)?;
        }

        // Switch to Tx Mode.
        self.to_tx(spi)?;

        Ok(0u8)
    }

    // Switch to Tx Mode.
    pub fn to_tx(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        match self.radio_mode {
            RadioMode::Rx | RadioMode::Standby => {
                let current_contents = self.spi_read(0x01, spi)?;
                let new_contents = (current_contents & 0b11111000) | 0b011;
                self.spi_write(0x01, new_contents, spi)?;

                // TODO: Wait Some Amount of Time for State Change to Latch
            },
            RadioMode::Tx => (),
        }

        Ok(())
    }

    // Switch to Rx Mode.
    pub fn to_rx(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        match self.radio_mode {
            RadioMode::Tx | RadioMode::Standby => {
                let current_contents = self.spi_read(0x01, spi)?;
                let new_contents = (current_contents & 0b1111_1000) | 0b101;
                self.spi_write(0x01, new_contents, spi)?;

                // TODO: Wait Some Amount of Time for State Change to Latch
            },
            RadioMode::Rx => (),
        }

        Ok(())
    }

    // Switch to Standby Mode.
    pub fn to_standby(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        match self.radio_mode {
            RadioMode::Rx | RadioMode::Tx => {
                let current_contents = self.spi_read(0x01, spi)?;
                let new_contents = (current_contents & 0b1111_1000) | 0b001;
                self.spi_write(0x01, new_contents, spi)?;

                // TODO: Wait Roughly (60us) for State Change to Latch
            },
            RadioMode::Standby => (),
        }

        Ok(())
    }

    // Read a value from the fifo register on the radio
    fn read_fifo(&mut self, address: u8, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        let mut read_command = [0x00, address, 0x00];

        match self.cs.as_mut() {
            Some(cs) => {
                cs.set_low().map_err(Error::GpioError)?;
                let transfer_result = spi.transfer(&mut read_command);
                let gpio_error = cs.set_high();

                match (transfer_result, gpio_error) {
                    (Err(err), Ok(_)) => Err(Error::SpiError(err)),
                    (Ok(_), Err(err)) => Err(Error::GpioError(err)),
                    (Err(spi_err), Err(gpio_err)) => Err(Error::GpioSpiError((gpio_err, spi_err))),
                    _ => Ok(read_command[2]),
                }
            },
            None => {
                spi.transfer(&mut read_command).map_err(Error::SpiError)?;
                Ok(read_command[2])
            }
        }
    }

    // Write a value to the fifo register on the radio
    fn write_fifo(&mut self, address: u8, data: u8, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        let mut write_command = [0x00, address, data];

        match self.cs.as_mut() {
            Some(cs) => {
                cs.set_low().map_err(Error::GpioError)?;
                let transfer_result = spi.transfer(&mut write_command);
                let gpio_error = cs.set_high();

                match (transfer_result, gpio_error) {
                    (Err(err), Ok(_)) => Err(Error::SpiError(err)),
                    (Ok(_), Err(err)) => Err(Error::GpioError(err)),
                    (Err(spi_err), Err(gpio_err)) => Err(Error::GpioSpiError((gpio_err, spi_err))),
                    _ => Ok(())
                }
            },
            None => {
                spi.transfer(&mut write_command).map_err(Error::SpiError)?;
                Ok(())
            }
        }
    }

    // Read the value of a register on the radio
    pub fn spi_read(&mut self, register: u8, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        let mut read_command = [register, 0x00];

        match self.cs.as_mut() {
            Some(cs) => {
                cs.set_low().map_err(Error::GpioError)?;
                let transfer_result = spi.transfer(&mut read_command);
                let gpio_error = cs.set_high();

                match (transfer_result, gpio_error) {
                    (Err(err), Ok(_)) => Err(Error::SpiError(err)),
                    (Ok(_), Err(err)) => Err(Error::GpioError(err)),
                    (Err(spi_err), Err(gpio_err)) => Err(Error::GpioSpiError((gpio_err, spi_err))),
                    _ => Ok(read_command[1]),
                }
            },
            None => {
                spi.transfer(&mut read_command).map_err(Error::SpiError)?;
                Ok(read_command[1])
            }
        }
    }

    // Write a specific value into a given register on the radio.
    pub fn spi_write(&mut self, register: u8, data: u8, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        let mut write_command = [0b1000_0000 | register, data];

        match self.cs.as_mut() {
            Some(cs) => {
                cs.set_low().map_err(Error::GpioError)?;
                let transfer_result = spi.transfer(&mut write_command);
                let gpio_error = cs.set_high();

                match (transfer_result, gpio_error) {
                    (Err(err), Ok(_)) => Err(Error::SpiError(err)),
                    (Ok(_), Err(err)) => Err(Error::GpioError(err)),
                    (Err(spi_err), Err(gpio_err)) => Err(Error::GpioSpiError((gpio_err, spi_err))),
                    _ => Ok(()),
                }
            },
            None => {
                spi.transfer(&mut write_command).map_err(Error::SpiError)?;
                Ok(())  
            },
        }
    }
}

/// Provides the necessary SPI mode configuration for the radio
pub const MODE: Mode = Mode {
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

// /// Provides high-level access to Semtech SX1276/77/78/79 based boards connected to a Raspberry Pi
// pub struct LoRa<SPI, CS, RESET, DELAY> {
//     spi: SPI,
//     cs: CS,
//     reset: RESET,
//     delay: DELAY,
//     frequency: i64,
//     pub explicit_header: bool,
//     pub mode: RadioMode,
// }

// #[derive(Debug)]
// pub enum Error<SPI, CS, RESET> {
//     Uninformative,
//     VersionMismatch(u8),
//     CS(CS),
//     Reset(RESET),
//     SPI(SPI),
//     Transmitting,
// }

// use Error::*;


// impl<SPI, CS, RESET, DELAY, E> LoRa<SPI, CS, RESET, DELAY>
//     where SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
//           CS: OutputPin, RESET: OutputPin, DELAY: DelayMs<u8> + DelayUs<u8>{
//     /// Builds and returns a new instance of the radio. Only one instance of the radio should exist at a time.
//     /// This also preforms a hardware reset of the module and then puts it in standby.
//     pub fn new(spi: SPI, cs: CS, reset: RESET, frequency: i64, delay: DELAY)
//                -> Result<Self, Error<E, CS::Error, RESET::Error>> {

//         let mut sx127x = LoRa {
//             spi,
//             cs,
//             reset,
//             delay,
//             frequency,
//             explicit_header: true,
//             mode: RadioMode::Sleep,
//         };

//         // fast reset radio for default init values
//         sx127x.reset.set_low().map_err(Reset)?;
//         sx127x.delay.delay_ms(10);
//         sx127x.reset.set_high().map_err(Reset)?;
//         sx127x.delay.delay_ms(10);

//         // check for the correct ID
//         let version = sx127x.read_register(Register::RegVersion.addr())?;
//         if version != 0x12 {
//             return Err(Error::VersionMismatch(version));
//         }

//         // set radio to LoRa Sleep mode
//         sx127x.set_mode(RadioMode::Sleep)?;
//         sx127x.delay.delay_ms(10);

//         // set the radio frequency
//         sx127x.set_frequency(frequency)?;

//         // set the modem config
//         // | 125 khz bandwitdh | 4/5 CR | Explicit Header |
//         // | 128 SF | Single TX | CRC On | Rx timeout msb? |
//         // | Low data rate off | AGC auto on |
//         sx127x.write_register(Register::RegModemConfig1.addr(), 0x72)?;
//         sx127x.write_register(Register::RegModemConfig2.addr(), 0x74)?;
//         sx127x.write_register(Register::RegModemConfig3.addr(), 0x04)?;

//         // set the base pointer of the TX and RX fifo queue
//         sx127x.write_register(Register::RegFifoTxBaseAddr.addr(), 0)?;
//         sx127x.write_register(Register::RegFifoRxBaseAddr.addr(), 0)?;

//         // set LNA to the highest gain
//         sx127x.write_register(Register::RegLna.addr(), 0x20)?;

//         // Set the radio to LoRa standby mode
//         sx127x.set_mode(RadioMode::Stdby)?;

//         // return lora instance
//         Ok(sx127x)
//     }

//     /// Transmits up to 255 bytes of data. To avoid the use of an allocator, this takes a fixed 255 u8
//     /// array and a payload size and returns the number of bytes sent if successful.
//     pub fn transmit_payload_busy(&mut self, buffer: [u8; 255], payload_size: usize)
//                             -> Result<usize,Error<E, CS::Error, RESET::Error>>{
        
//         // check if radio is currently transmitting
//         while self.transmitting()? {}

//         // set radio to standby and wait for it to be ready
//         self.set_mode(RadioMode::Stdby)?;
//         self.delay.delay_ms(10);

//         self.write_register(Register::RegIrqFlags.addr(), 0)?;
//         self.write_register(Register::RegFifoAddrPtr.addr(), 0)?;
//         self.write_register(Register::RegPayloadLength.addr(), 0)?;

//         for byte in buffer.iter().take(payload_size) {
//             self.write_register(Register::RegFifo.addr(), *byte)?;
//         }

//         self.write_register(Register::RegPayloadLength.addr(), payload_size as u8)?;
//         self.set_mode(RadioMode::Tx)?;

//         while self.transmitting()? {};

//         Ok(payload_size)  
//     }

//     pub fn set_dio0_tx_done(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         self.write_register(Register::RegDioMapping1.addr(), 0b01_00_00_00)
//     }

//     pub fn transmit_payload(&mut self, buffer: [u8; 255], payload_size: usize)
//                             -> Result<(), Error<E, CS::Error, RESET::Error>>{
//         if self.transmitting()? {
//             return Err(Transmitting);
//         }else{
//             self.set_mode(RadioMode::Stdby)?;
//             if self.explicit_header {
//                 self.set_explicit_header_mode()?;
//             }else{
//                 self.set_implicit_header_mode()?;
//             }

//             self.write_register(Register::RegIrqFlags.addr(), 0)?;
//             self.write_register(Register::RegFifoAddrPtr.addr(), 0)?;
//             self.write_register(Register::RegPayloadLength.addr(), 0)?;
//             for byte in buffer.iter().take(payload_size){
//                 self.write_register(Register::RegFifo.addr(), *byte)?;
//             }
//             self.write_register(Register::RegPayloadLength.addr(),payload_size as u8)?;
//             self.set_mode(RadioMode::Tx)?;
//             Ok(())
//         }
//     }

//     /// Blocks the current thread, returning the size of a packet if one is received or an error is the
//     /// task timed out. The timeout can be supplied with None to make it poll indefinitely or
//     /// with `Some(timeout_in_mill_seconds)`
//     pub fn poll_irq(&mut self, timeout_ms: Option<i32>) -> Result<usize, Error<E, CS::Error, RESET::Error>>{

//         self.set_mode(RadioMode::RxContinuous)?;
//         let mut count = 0;
        
//         match timeout_ms {
//             Some(value) => {
//                 while (self.read_register(Register::RegIrqFlags.addr())? & 0x40 != 1)
//                     && (count < value) {
//                     count += 1;
//                     self.delay.delay_ms(1);
//                 }

//                 if count >= value {
//                     Err(Uninformative)
//                 } else {
//                     Ok(self.read_register(Register::RegRxNbBytes.addr())? as usize)
//                 }
//             },
//             None => {
//                 while (self.read_register(Register::RegIrqFlags.addr())? & 0x40) != 1 {
//                     self.delay.delay_ms(100);
//                 }

//                 Ok(self.read_register(Register::RegRxNbBytes.addr())? as usize)
//             }
//         }
//     }

//     /// Returns the contents of the fifo as a fixed 255 u8 array. This should only be called is there is a
//     /// new packet ready to be read.
//     pub fn read_packet(&mut self) -> Result<[u8; 255], Error<E, CS::Error, RESET::Error>> {
        
//         // create buffer to store received data
//         let nbytes = self.read_register(Register::RegRxNbBytes.addr())?;
//         let mut buffer = [0 as u8; 255];

//         // set pointer to beginning of packet
//         let fifo_addr = self.read_register(Register::RegFifoRxCurrentAddr.addr())?;
//         self.write_register(Register::RegFifoAddrPtr.addr(), fifo_addr)?;

//         // read the packet and store it in the buffer
//         for i in 0..nbytes {
//             let byte = self.read_register(Register::RegFifo.addr())?;
//             buffer[i as usize] = byte;
//         }

//         // clear the IRQ flag
//         self.clear_irq()?;

//         // reset the fifo queue pointer
//         self.write_register(Register::RegFifoAddrPtr.addr(), 0)?;   

//         // return buffer
//         Ok(buffer)
//     }

//     /// Returns true if the radio is currently transmitting a packet.
//     fn transmitting(&mut self) -> Result<bool, Error<E, CS::Error, RESET::Error>> {
//         if (self.read_register(Register::RegOpMode.addr())? & RadioMode::Tx.addr())
//             == RadioMode::Tx.addr() {
//             Ok(true)
//         }else{
//             if (self.read_register(Register::RegIrqFlags.addr())?
//                 & IRQ::IrqTxDoneMask.addr()) == 1{
//                 self.write_register(Register::RegIrqFlags.addr(),
//                                     IRQ::IrqTxDoneMask.addr())?;
//             }
//             Ok(false)
//         }
//     }

//     /// Clears the radio's IRQ registers.
//     pub fn clear_irq(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         // reads current flags and any flag that's ON is cleared by rewriting it
//         let irq_flags = self.read_register(Register::RegIrqFlags.addr())?;
//         self.write_register(Register::RegIrqFlags.addr(), irq_flags)?;

//         Ok(())
//     }


//     /// Sets the transmit power and pin. Levels can range from 0-14 when the output
//     /// pin = 0(RFO), and form 0-20 when output pin = 1(PaBoost). Power is in dB.
//     /// Default value is `17`.
//     pub fn set_tx_power(&mut self, mut level: i32, output_pin: u8) -> Result<(), Error<E, CS::Error, RESET::Error>>{
//         if PaConfig::PaOutputRfoPin.addr() == output_pin {
//             // RFO
//             if level < 0 {
//                 level = 0;
//             } else if level > 14 {
//                 level = 14;
//             }
//             self.write_register(Register::RegPaConfig.addr(), (0x70 | level) as u8)
//         } else {
//             // PA BOOST
//             if level > 17 {
//                 if level > 20 {
//                     level = 20;
//                 }
//                 // subtract 3 from level, so 18 - 20 maps to 15 - 17
//                 level -= 3;

//                 // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
//                 self.write_register(Register::RegPaDac.addr(), 0x87)?;
//                 self.set_ocp(140)?;
//             } else {
//                 if level < 2 {
//                     level = 2;
//                 }

//                 //Default value PA_HF/LF or +17dBm
//                 self.write_register(Register::RegPaDac.addr(), 0x84)?;
//                 self.set_ocp(100)?;
//             }
//             level -= 2;
//             self.write_register(Register::RegPaConfig.addr(), PaConfig::PaBoost.addr()
//                 | level as u8)
//         }
//     }

//     /// Sets the over current protection on the radio(mA).
//     pub fn set_ocp(&mut self, ma: u8) -> Result<(), Error<E, CS::Error, RESET::Error>>{
//         let mut ocp_trim: u8 = 27;

//         if ma <= 120 {
//             ocp_trim = (ma - 45) / 5;
//         } else if ma <=240 {
//             ocp_trim = (ma + 30) / 10;
//         }
//         self.write_register(Register::RegOcp.addr(), 0x20 | (0x1F & ocp_trim))
//     }

//     /// Sets the state of the radio. Default mode after initiation is `Standby`.
//     pub fn set_mode(&mut self,mode: RadioMode) -> Result<(), Error<E, CS::Error, RESET::Error>> {

//         self.write_register(Register::RegOpMode.addr(), RadioMode::LongRangeMode.addr()
//             | mode.addr())?;

//         self.mode = mode;

//         Ok(())
//     }

//     /// Sets the frequency of the radio. Values are in megahertz.
//     /// I.E. 915 MHz must be used for North America. Check regulation for your area.
//     pub fn set_frequency(&mut self, freq: i64) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         self.frequency = freq;
//         // calculate register values
//         let base = 1;
//         let frf = (freq * (base << 19)) / 32;
//         // write registers
//         self.write_register(Register::RegFrfMsb.addr(), ((frf & 0x00FF_0000) >> 16) as u8)?;
//         self.write_register(Register::RegFrfMid.addr(), ((frf & 0x0000_FF00) >> 8) as u8)?;
//         self.write_register(Register::RegFrfLsb.addr(), (frf & 0x0000_00FF) as u8)
//     }

//     /// Sets the radio to use an explicit header. Default state is `ON`.
//     fn set_explicit_header_mode(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         let reg_modem_config_1 = self.read_register(Register::RegModemConfig1.addr())?;
//         self.write_register(Register::RegModemConfig1.addr(), reg_modem_config_1 & 0xfe)?;
//         self.explicit_header = true;
//         Ok(())
//     }

//     /// Sets the radio to use an implicit header. Default state is `OFF`.
//     fn set_implicit_header_mode(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         let reg_modem_config_1 = self.read_register(Register::RegModemConfig1.addr())?;
//         self.write_register(Register::RegModemConfig1.addr(),reg_modem_config_1  & 0x01)?;
//         self.explicit_header = false;
//         Ok(())
//     }


//     /// Sets the spreading factor of the radio. Supported values are between 6 and 12.
//     /// If a spreading factor of 6 is set, implicit header mode must be used to transmit
//     /// and receive packets. Default value is `7`.
//     pub fn set_spreading_factor(&mut self, mut sf: u8) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         if sf < 6 {
//             sf = 6;
//         } else if sf > 12 {
//             sf = 12;
//         }

//         if sf == 6 {
//             self.write_register(Register::RegDetectionOptimize.addr(), 0xc5)?;
//             self.write_register(Register::RegDetectionThreshold.addr(), 0x0c)?;
//         } else {
//             self.write_register(Register::RegDetectionOptimize.addr(), 0xc3)?;
//             self.write_register(Register::RegDetectionThreshold.addr(), 0x0a)?;
//         }
//         let modem_config_2 = self.read_register(Register::RegModemConfig2.addr())?;
//         self.write_register(Register::RegModemConfig2.addr(), (modem_config_2 & 0x0f)
//             | ((sf << 4) & 0xf0))?;
//         self.set_ldo_flag()?;
//         Ok(())
//     }

//     /// Sets the signal bandwidth of the radio. Supported values are: `7800 Hz`, `10400 Hz`,
//     /// `15600 Hz`, `20800 Hz`, `31250 Hz`,`41700 Hz` ,`62500 Hz`,`125000 Hz` and `250000 Hz`
//     /// Default value is `125000 Hz`
//     pub fn set_signal_bandwidth(&mut self, sbw: i64) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         let bw: i64;
//         match sbw {
//             7_800 => bw = 0,
//             10_400 => bw = 1,
//             15_600 => bw = 2,
//             20_800 => bw = 3,
//             31_250 => bw = 4,
//             41_700 => bw = 5,
//             62_500 => bw = 6,
//             125_000 => bw = 7,
//             250_000 => bw = 8,
//             _ => bw = 9,
//         }
//         let modem_config_1 = self.read_register(Register::RegModemConfig1.addr())?;
//         self.write_register(Register::RegModemConfig1.addr(), (modem_config_1 & 0x0f)
//             | ((bw << 4) as u8))?;
//         self.set_ldo_flag()?;
//         Ok(())
//     }

//     /// Sets the coding rate of the radio with the numerator fixed at 4. Supported values
//     /// are between `5` and `8`, these correspond to coding rates of `4/5` and `4/8`.
//     /// Default value is `5`.
//     pub fn set_coding_rate_4(&mut self, mut denominator: u8) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         if denominator < 5 {
//             denominator = 5;
//         } else if denominator > 8 {
//             denominator = 8;
//         }
//         let cr = denominator - 4;
//         let modem_config_1 = self.read_register(Register::RegModemConfig1.addr())?;
//         self.write_register( Register::RegModemConfig1.addr(), (modem_config_1 & 0xf1)
//             | (cr << 1))
//     }

//     /// Sets the preamble length of the radio. Values are between 6 and 65535.
//     /// Default value is `8`.
//     pub fn set_preamble_length(&mut self, length: i64) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         self.write_register(Register::RegPreambleMsb.addr(), (length >> 8) as u8)?;
//         self.write_register(Register::RegPreambleLsb.addr(), length as u8)
//     }

//     /// Enables are disables the radio's CRC check. Default value is `false`.
//     pub fn set_crc(&mut self, value: bool) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         let modem_config_2 = self.read_register(Register::RegModemConfig2.addr())?;
//         if value {
//             self.write_register(Register::RegModemConfig2.addr(), modem_config_2 | 0x04)
//         }else{
//             self.write_register(Register::RegModemConfig2.addr(), modem_config_2 & 0xfb)
//         }
//     }

//     /// Inverts the radio's IQ signals. Default value is `false`.
//     pub fn set_invert_iq(&mut self, value: bool) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         if value {
//             self.write_register(Register::RegInvertiq.addr(),  0x66)?;
//             self.write_register(Register::RegInvertiq2.addr(), 0x19)
//         }else{
//             self.write_register(Register::RegInvertiq.addr(),  0x27)?;
//             self.write_register(Register::RegInvertiq2.addr(), 0x1d)
//         }
//     }


//     /// Returns the spreading factor of the radio.
//     pub fn get_spreading_factor(&mut self) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
//         Ok(self.read_register(Register::RegModemConfig2.addr())? >> 4)
//     }

//     /// Returns the signal bandwidth of the radio.
//     pub fn get_signal_bandwidth(&mut self) -> Result<i64, Error<E, CS::Error, RESET::Error>> {
//         let bw = self.read_register(Register::RegModemConfig1.addr())? >> 4;
//         let bw = match bw {
//             0 => 7_800,
//             1 => 10_400,
//             2 => 15_600,
//             3 => 20_800,
//             4 => 31_250,
//             5 => 41_700,
//             6 => 62_500,
//             7 => 125_000,
//             8 => 250_000,
//             9 => 500_000,
//             _ => -1,
//         };
//         Ok(bw)
//     }

//     /// Returns the RSSI of the last received packet.
//     pub fn get_packet_rssi(&mut self) -> Result<i32, Error<E, CS::Error, RESET::Error>> {
//         Ok(i32::from(self.read_register(Register::RegPktRssiValue.addr())?) - 157)
//     }

//     /// Returns the signal to noise radio of the the last received packet.
//     pub fn get_packet_snr(&mut self) -> Result<f64, Error<E, CS::Error, RESET::Error>> {
//         Ok(f64::from(self.read_register(Register::RegPktSnrValue.addr())?))
//     }

//     /// Returns the frequency error of the last received packet in Hz.
//     pub fn get_packet_frequency_error(&mut self) -> Result<i64, Error<E, CS::Error, RESET::Error>> {
//         let mut freq_error: i32 = 0;
//         freq_error = i32::from(self.read_register( Register::RegFreqErrorMsb.addr())? & 0x7);
//         freq_error <<= 8i64;
//         freq_error += i32::from(self.read_register(Register::RegFreqErrorMid.addr())?);
//         freq_error <<= 8i64;
//         freq_error += i32::from(self.read_register( Register::RegFreqErrorLsb.addr())?);

//         let f_xtal = 32_000_000; // FXOSC: crystal oscillator (XTAL) frequency (2.5. Chip Specification, p. 14)
//         let f_error = ((f64::from(freq_error) * (1i64 << 24) as f64) / f64::from(f_xtal)) *
//             (self.get_signal_bandwidth()? as f64 / 500_000.0f64); // p. 37
//         Ok(f_error as i64)
//     }

//     pub fn get_radio_version(&mut self) -> Result<u8, Error<E, CS::Error, RESET::Error>> {
//         Ok(self.read_register(Register::RegVersion.addr())?)
//     }

//     fn set_ldo_flag(&mut self) -> Result<(), Error<E, CS::Error, RESET::Error>> {
//         let sw = self.get_signal_bandwidth()?;
//         // Section 4.1.1.5
//         let symbol_duration = 1000 / (sw / ((1 as i64) << self.get_spreading_factor()?)) ;

//         // Section 4.1.1.6
//         let ldo_on = symbol_duration > 16;

//         let mut config_3 = self.read_register(Register::RegModemConfig3.addr())?;
//         config_3.set_bit(3,ldo_on);
//         self.write_register(Register::RegModemConfig3.addr(), config_3)
//     }

//     fn read_register(&mut self, reg: u8) -> Result<u8,Error<E, CS::Error, RESET::Error>> {
//         self.cs.set_low().map_err(CS)?;

//         let mut buffer = [reg & 0x7f, 0];
//         let transfer = self.spi.transfer(&mut buffer).map_err(SPI)?;
//         self.cs.set_high().map_err(CS)?;
        
//         // --- EDIT WAS DONE HERE ---
//         // an extra 1 us delay is needed to avoid a problem with concurrent SPI actions
//         self.delay.delay_us(1);
        
//         Ok(transfer[1])
//     }

//     fn write_register(&mut self, reg: u8, byte: u8) -> Result<(),Error<E, CS::Error, RESET::Error>>{
//         self.cs.set_low().map_err(CS)?;

//         let buffer = [reg | 0x80, byte];
//         let write = self.spi.write(& buffer).map_err(SPI)?;
//         self.cs.set_high().map_err(CS)?;

//         // --- EDIT WAS DONE HERE ---
//         // an extra 1 us delay is needed to avoid a problem with concurrent SPI actions
//         self.delay.delay_us(1);
        
//         Ok(write)
//     }
// }
// /// Modes of the radio and their corresponding register values.
// #[derive(Clone, Copy)]
// pub enum RadioMode{
//     LongRangeMode = 0x80,
//     Sleep = 0x00,
//     Stdby = 0x01,
//     Tx = 0x03,
//     RxContinuous = 0x05,
//     RxSingle = 0x06,
// }

// impl RadioMode {
//     /// Returns the address of the mode.
//     pub fn addr(self) -> u8 {
//         self as u8
//     }
// }
