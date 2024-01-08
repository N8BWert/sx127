//!
//! Configuration for the SX127X Radio Module
//! 

#![allow(dead_code)]

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub enum ModulationType {
    FSK = 0b00,
    OOK = 0b01,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub enum AddressFiltering {
    NONE = 0b00,
    NodeAddress = 0b01,
    NodeAndBroadcast = 0b10,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
/// PAXXXX has a rise/fall time /ramp up in FSK of XXXX us
pub enum PaRamp {
    PA3400 = 0b0000,
    PA2000 = 0b0001,
    PA1000 = 0b0010,
    PA500 = 0b0011,
    PA250 = 0b0100,
    PA125 = 0b0101,
    PA100 = 0b0110,
    PA62 = 0b0111,
    PA50 = 0b1000,
    PA40 = 0b1001,
    PA31 = 0b1010,
    PA25 = 0b1011,
    PA20 = 0b1100,
    PA15 = 0b1101,
    PA12 = 0b1110,
    PA10 = 0b1111,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub struct Config<'a> {
    pub modulation_type: ModulationType,
    // MAX is Roughly 300 kbps
    pub bitrate: u32,
    pub frequency: u16,
    pub rx_timeout: Option<u8>,
    pub preamble_size: u16,
    pub sync_word: Option<&'a [u8]>,
    pub crc_on: bool,
    pub address_filtering: AddressFiltering,
    pub payload_length: Option<u16>,
    pub node_address: Option<u8>,
    pub broadcast_address: Option<u8>,
    pub pa_ramp: Option<PaRamp>,
}

#[cfg(feature = "fsk-ook")]
impl<'a> Config<'a> {
    pub fn register_values(&self) -> ([u8; 21], [u8; 21]) {
        let mut registers = [0u8; 21];
        let mut values = [0u8; 21];
        let mut current_register = 0x01;

        // Set Modulation Type
        registers[current_register] = 0x01;
        values[current_register] = 0b0000_1001 | ((self.modulation_type as u8) << 5);
        current_register += 1;

        // Set Bitrate
        // Bit Rate = FXOSC / (Bitrate(15,0) + BitRateFrac/16) = FXOSC / BitRate(15,0)
        // BitRate(15, 0) = FXOSC / Bit Rate
        let bitrate = (32_000_000 / self.bitrate) as u16;

        registers[current_register] = 0x02;
        values[current_register] = ((bitrate & 0xFF00) >> 8) as u8;
        current_register += 1;

        registers[current_register] = 0x03;
        values[current_register] = (bitrate & 0x00FF) as u8;
        current_register += 1;

        // Set Frequency
        registers[current_register] = 0x06;
        values[current_register] = ((self.frequency & 0xFF00) >> 8) as u8;
        current_register += 1;

        registers[current_register] = 0x07;
        values[current_register] = (self.frequency & 0x00FF) as u8;
        current_register += 1;

        // Set PaRamp
        if let Some(pa_ramp) = self.pa_ramp {
            registers[current_register] = 0x0A;
            values[current_register] = pa_ramp as u8;
            current_register += 1;
        }

        // Set Rx Timeout
        if let Some(timeout) = self.rx_timeout {
            registers[current_register] = 0x20 ;
            values[current_register] = timeout;
            current_register += 1;
        }

        // Set Preamble Size
        registers[current_register] = 0x25;
        values[current_register] = ((self.preamble_size & 0xFF00) >> 8) as u8;
        current_register += 1;

        registers[current_register] = 0x26;
        values[current_register] = (self.preamble_size & 0x00FF) as u8;
        current_register += 1;

        // Set Sync Word
        if let Some(sync_word) = self.sync_word {
            let sync_size = sync_word.len() - 1;

            registers[current_register] = 0x27;
            values[current_register] = 0b0001000 | sync_size as u8;
            current_register += 1;

            for (i, word) in sync_word.iter().enumerate() {
                registers[current_register] = 0x28 + (i as u8);
                values[current_register] = *word;
                current_register += 1;
            }
        } else {
            registers[current_register] = 0x27;
            values[current_register] = 0x00;
            current_register += 1;
        }

        // Set Packet Config Register
        registers[current_register] = 0x30;
        values[current_register] = ((if self.payload_length.is_some() { 1u8 } else { 0u8 }) << 7) | 
                                    ((self.crc_on as u8) << 4) |
                                    ((self.address_filtering as u8) << 1);
        current_register += 1;

        if let Some(payload_length) = self.payload_length {
            registers[current_register] = 0x31;
            values[current_register] = 0b0100_0000 | ((payload_length & (0x07 << 8)) >> 8) as u8;
            current_register += 1;
        }

        // Set Node Address
        if let Some(node_address) = self.node_address {
            registers[current_register] = 0x33;
            values[current_register] = node_address;
            current_register += 1;
        }

        // Set Broadcast Address
        if let Some(broadcast_address) = self.node_address {
            registers[current_register] = 0x34;
            values[current_register] = broadcast_address;
            current_register += 1;
        }

        (registers, values)
    }
}