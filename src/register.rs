#![allow(dead_code)]

#[cfg(feature = "lora")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum DeviceMode {
    //
    SLEEP = 0b000,
    // 
    STDBY = 0b001,
    // Frequency Synthesis TX
    FSTX = 0b010,
    // Transmit
    TX = 0b011,
    // Frequency Synthesis RX
    FSRX = 0b100,
    // Receive Continuous
    RXCONTINUOUS = 0b101,
    // Receive Single
    RXSINGLE = 0b110,
    // Channel Activity Detection
    CAD = 0b111,
}

#[cfg(feature = "lora")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegOpMode {
    // { 0 : FSK/OOK Mode, 1 : LoRa Mode}
    long_range_mode: bool,
    // {0 : Access LoRa Registers (0x0D: 0x3F), 1 : Access FSK Registers (0x0D: 0x3F)}
    access_shared_registers: bool,
    // Access Low Frequency Mode Registers
    low_frequency_mode: bool,
    // Device Mode
    device_mode: DeviceMode,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum ModulationType {
    FSK = 0b00,
    OOK = 0b01,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum DeviceMode {
    SLEEP = 0b000,
    STDBY = 0b001,
    FSTX = 0b010,
    TX = 0b011,
    FSRX = 0b100,
    RX = 0b101,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegOpMode {
    // { 0 : RSK/OOK Mode, 1 : LoRa Mode }
    long_range_mode: bool,
    modulation_type: ModulationType,
    low_frequency_mode: bool,
    device_mode: DeviceMode,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegPaConfig {
    // 6 -> {0 : RFO Pin (max 14 dBm), 1 : PA_BOOST Pin (max 20 dBm)}
    pa_select: bool,
    // 6-3 -> Max Power Pmax = 10.8 + 0.6 * MaxPower
    max_power: u8,
    // 3-0 -> Output Power
    //  if PaSelect == 0 => Pout = Pmax - (15 - OutputPower)
    //  if PaSelect == 1 => Pout = 17 - (15 - OutputPower)
    output_power: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
/// More Info (here)[https://cdn-shop.adafruit.com/product-files/3179/sx1276_77_78_79.pdf]
pub(crate) struct RegPaRamp {
    modulation_shaping: u8,
    pa_ramp: u8,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegOcp {
    // 5 -> Enables Overload Current Protection for PA
    ocp_on: bool,
    // 4-0 -> Trimming of OCP current (default = 100mA)
    //  if OcpTrim <= 15 (120 mA) => Imax = 45 + 5 * OcpTrim [mA]
    //  if 15 < OcpTrim <= 27 (130-240 mA) => Imax = -30 + 10 * OcpTrim [mA]
    //  else iMax = 240mA
    ocp_trim: u8,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum LNAGain {
    // G1 = Maximum Gain
    G1 = 0b001,
    G2 = 0b010,
    G3 = 0b011,
    G4 = 0b100,
    G5 = 0b101,
    // G6 = Minimum Gain
    G6 = 0b110,
}

#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegLna {
    // 7-5 -> LNA Gain Setting
    lna_gain: LNAGain,
    // 4-3 -> Low Frequency (RFI_LF) LNA Current Adjustment
    lna_boost_lf: u8,
    // 1-0 -> High Frequency (RFI_HF) LNA Current Adjustment
    lna_boost_hf: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegRxConfig {
    restart_rx_on_collision: bool,
    restart_rx_without_pll_lock: bool,
    restart_rx_with_pll_lock: bool,
    afc_auto_on: bool,
    agc_auto_on: bool,
    // Selects the event triggering AGC and/or AFC at receiver startups
    rx_trigger: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegRssiConfig {
    rssi_offset: i8,
    // 2^x samples used
    rssi_smoothing: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegRxBw {
    bandwidth_mantissa: u8,
    bandwidth_exponent: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegAfcBw {
    bandwidth_mantissa: u8,
    bandwidth_exponent: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum OokThresholdType {
    Fixed = 0b00,
    Peak = 0b01,
    Average = 0b10,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegOokPeak {
    bit_sync_on: bool,
    ook_thresh_type: OokThresholdType,
    // 0.5 * x dB
    ook_peak_thresh_step: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegOokAvg {
    // Period of decrement of the RSSI threshold in the OOK demodulator
    // 2^x times per chip
    ook_peak_thresh_dec: u8,
    // Static offset added to the threshold in average mode in order to reduce glitching
    // activity (2 * x dB)
    ook_average_offset: u8,
    // Filter coefficient in average mode of the OOK demodulator
    ook_average_thresh_filt: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegAfcFei {
    agc_start: bool,
    afc_clear: bool,
    afc_auto_clear: bool,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegPreambleDetect {
    preamble_detector_on: bool,
    detect_size: u8,
    detect_tolerance: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegOsc {
    rc_cal_start: bool,
    clk_out: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum RxRestart {
    Off = 0b00,
    OnWithoutPll = 0b01,
    OnWithPll = 0b10,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegSyncConfig {
    rx_restart_mode: RxRestart,
    // { 0 : 0xAA, 1 : 0x55}
    preamble_polarity: bool,
    sync_on: bool,
    // Size of the Sync word: (SyncSize + 1) Bytes
    sync_size: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum DCFreeEncoding {
    NONE = 0b00,
    MANCHESTER = 0b01,
    WHITENING = 0b10,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum AddressFiltering {
    NONE = 0b00,
    NodeAddress = 0b01,
    NodeOrBroadcastAddress = 0b10,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegPacketConfig1 {
    variable_length: bool,
    encoding: DCFreeEncoding,
    crc_on: bool,
    crc_auto_clear: bool,
    address_filtering: AddressFiltering,
    // { 0 : CCITT CRC implementation with standard whitening, 1 : IBM CRC implementation with alternative whitening}
    crc_whitening_type: bool,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum DataMode {
    Continuous = 0,
    Packet = 1,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegPacketConfig2 {
    data_processing_mode: DataMode,
    io_home_on: bool,
    io_home_power_frame: bool,
    beacon_on: bool,
    // Most significant 2 bits
    payload_length: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegFifoThresh {
    start_on_low_fifo: bool,
    low_fifo_level: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegSeqConfig1 {
    // On 1 begin the start transition
    sequencer_start: bool,
    // FOrces the sequencer off
    sequencer_stop: bool,
    // Chip mode for idle { 0 : Standby, 1 : Sleep}
    idle_mode: bool,
    // state to transition to 
    // { 0 : to LowPowerSelection,
    //   1 : To Receive State,
    //   2 : To Transmit State,
    //   3 : To Transmit State on a FifoLevel Interrupt }
    from_start: u8,
    // Select state to transfer to after toLowPowerSelection { 0 : Initial, 1 : Idle}
    low_power_selection: bool,
    // Control transition from idle on a T1 interrupt { 0 : Transmit, 1 : Receive }
    from_idle: bool,
    // Control transition from transmit { 0 : LowPowerSelection on PacketSent, 1 : Receive on PacketSent }
    from_transmit: bool,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegSeqConfig2 {
    // Sequencer transition from Receive State
    // { 1 : To PacketReceivedState on PayloadReady, 2 : To lowPowerSelection on PayloadReady,
    //   3 : to PacketReceived on CrcOk, 4 : To SequencerOff on RSSI Interrupt,
    //   5 : To SequencerOff on SyncAddress Interrupt, 6 : To SequencerOff on PreambleDetect}
    from_receive: u8,
    // { 0 : To Receive State via Receive Restart, 1 : to Transmit State,
    //   2 : To LowPowerSelection, 3: To SequencerOff }
    from_rx_timeout: u8,
    // Control State transition from PacketReceived State
    // { 0 : To SequencerOff, 1 : To Transmit State on FifoEmpty,
    //   2 : ToLowPowerSelection, 3: To Receive va FS Mode, 4 : To Receive State}
    from_packet_received: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum TimerResolution {
    Disabled,
    R64us,
    R4ms,
    R262ms,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegTimerResol {
    timer_1_resolution: TimerResolution,
    timer_2_resolution: TimerResolution,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegImageCal {
    auto_image_cal_on: bool,
    image_cal_start: bool,
    image_cal_running: bool,
    temp_change: bool,
    temp_threshold: u8,
    temp_monitor_off: bool,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegLowBat {
    low_bat_on: bool,
    low_bat_trim: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegIrqFlags1 {
    mode_ready: bool,
    rx_ready: bool,
    tx_ready: bool,
    pll_lock: bool,
    rssi: bool,
    timeout: bool,
    preamble_detect: bool,
    sync_address_match: bool,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegIrqFlags2 {
    fifo_full: bool,
    fifo_empty: bool,
    fifo_level: bool,
    fifo_overrun: bool,
    packet_sent: bool,
    payload_ready: bool,
    crc_ok: bool,
    low_battery: bool,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
/// More Data at [tables 29 and 30](https://cdn-shop.adafruit.com/product-files/3179/sx1276_77_78_79.pdf)
pub(crate) struct RegDioMapping1 {
    dio_0_mapping: u8,
    dio_1_mapping: u8,
    dio_2_mapping: u8,
    dio_3_mapping: u8,
}

#[cfg(feature = "fsk-ook")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
/// More Data at [tables 29 and 30](https://cdn-shop.adafruit.com/product-files/3179/sx1276_77_78_79.pdf)
pub(crate) struct RegDioMapping2 {
    dio_4_mapping: u8,
    dio5_mapping: u8,
    map_preamble_detect: bool,
}

#[cfg(feature = "lora")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct InterruptMask {
    // Don't Receive Interrupts on Rx Timeout
    rx_timeout_mask: bool,
    // Don't Receive Interrupts on Rx Done
    rx_done_mask: bool,
    // Don't Receive Interrupts on Crc Error
    crc_error_mask: bool,
    // Don't Receive Interrupts on Valid Header Received
    valid_header_mask: bool,
    // Don't Receive Interrupts on Tx Completed
    tx_done_mask: bool,
    // Don't Receive Interrupts on CAD Complete
    cad_done_mask: bool,
    // Don't Receive Interrupts on FHSS Channel Change
    fhss_change_mask: bool,
    // Don't Receive Interrupts on Cad Detected
    cad_detected_mask: bool,
}

#[cfg(feature = "lora")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct Interrupt {
    rx_timeout: bool,
    rc_done: bool,
    payload_crc_error: bool,
    valid_header: bool,
    tx_done: bool,
    cad_done: bool,
    fhss_change_channel: bool,
    cad_detected: bool,
}

#[cfg(feature = "lora")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum CodingRate {
    // 4/5
    CRC45 = 0b001,
    // 4/6
    CRC46 = 0b010,
    // 4/7
    CRC47 = 0b011,
    // 4/8
    CRC48 = 0b100,
}

#[cfg(feature = "lora")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegModemConfig1 {
    // Bandwidth (in kHz)
    bandwidth: f32,
    coding_rate: CodingRate,
    // { 0 : Explicit Header Mode, 1 : Implicit Header Mode }
    implicit_header_mode: bool,
}

#[cfg(feature = "lora")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
/// All Values are in Chips / Symbol
pub(crate) enum SpreadingFactor {
    S64 = 6,
    S128 = 7,
    S256 = 8,
    S512 = 9,
    S1024 = 10,
    S2048 = 11,
    S4096 = 12,
}

#[cfg(feature = "lora")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegModemConfig2 {
    spreading_factor: SpreadingFactor,
    tx_continuous: bool,
    rx_payload_crc: bool,
    rx_timeout_msb: u8,
}

#[cfg(feature = "lora")]
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) struct RegModemConfig3 {
    // {0 : Disabled, 1 : Enabled (mandated for when the symbol length exceeds 16ms)}
    low_data_optimize: bool,
    // {0: LNA gain set by register LnaGain, 1 : LNA gain set by the internal AGC loop}
    agc_auto_on: bool,
}

#[cfg(feature = "lora")]
/// Register Layout for the LoRa Mode of the sx127x Radio
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
pub(crate) enum Register {
    // FIFO read/write access
    RegFifo(u8) = 0x00,

    // Operating Mode and LoRa / RSK selection
    RegOpMode(Option<RegOpMode>)= 0x01,

    // 0x02-0x05 Are Unused

    // RF Carrier Frequency, Most Significant Bits
    // f = F(XOSC) * Frf / 2^19
    RegFrfMsb(Option<u8>) = 0x06,

    // RF Carrier Frequency, Intermediate Bits
    RegFrfMid(Option<u8>) = 0x07,

    // RF Carrier Frequency, Least Significant Bits
    RegFrfLsb(Option<u8>) = 0x08,

    // PA Selection and Output Power Control
    RegPaConfig(Option<RegPaConfig>) = 0x09,

    // Control of PA ramp time, low phase noise PLL
    // 3-0 -> Rise/Fall time of ramp up/down in FSK
    //  [Link](https://cdn-shop.adafruit.com/product-files/3179/sx1276_77_78_79.pdf)
    RegPaRamp(Option<u8>) = 0x0A,

    // Over Current Protection Control
    RegOcp(Option<RegOcp>) = 0x0B,

    // LNA settings
    RegLna(Option<RegLna>) = 0x0C,

    // FIFO SPI pointer
    // SPI Interface Address Pointer in FIFO Data Buffer (I.E. Where the SPI is Writing the the FIFO Buffer)
    RegFifoAddrPtr(Option<u8>) = 0x0D,

    // Start Tx Data
    // Write Base Address in FIFO Data Buffer for TX Modulator
    RegFifoTxBaseAddr(Option<u8>) = 0x0E,

    // Start Rx Data
    // Read Base Address in FIFO Data Buffer for Rx Demodulator
    RegFifoRxBaseAddr(Option<u8>) = 0x0F,

    // Start Address of Last Packet Received
    FifoRxCurrentAddr(Option<u8>) = 0x10,

    // Optional IRQ Flag Mask
    RegIrqFlagsMask(Option<InterruptMask>) = 0x11,

    // IRQ flags
    RegIrqFlags(Option<Interrupt>) = 0x12,

    // Number of Received Bytes since last Rx transition
    RegRxNbBytes(Option<u8>) = 0x13,

    // Number of Valid Headers Received (MSB)
    RegRxHeaderCntValueMsb(Option<u8>) = 0x14,

    // Number of Valid Headers Received (LSB)
    RegRxHeaderCntValueLsb(Option<u8>) = 0x15,

    // Number of valid packets received (MSB)
    RegRxPacketCntValueMsb(Option<u8>) = 0x16,

    // Number of valid packets received (LSB)
    RegRxPacketCntValueLsb(Option<u8>) = 0x17,

    // Live LoRa Modem Status (Read-Only)
    // 7-5 Coding Rate of Last Header Received
    // 4 -> Modem Clear
    // 3 -> Header Info Valid
    // 2 -> Rx on-going
    // 1 -> Signal Synchronized
    // 0 -> Signal Detected
    RegModemStat(Option<u8>) = 0x18,

    // Estimation of last packet SNR (Read-Only)
    // SNR[dB] = PacketSnr[twos complement] / 4
    RegPktSnrValue(Option<u8>) = 0x19,

    // RSSI of last packet (Read-Only)
    // RSSI[dBm] = -157 + Rssi (using HF output port, SNR >= 0)
    // RSSI[dBm] = -164 + Rssi (using LF output port, SNF >= 0)
    RegPktRssiValue(Option<u8>) = 0x1A,

    // Current RSSI (Read-Only)
    // RSSI[dBm] = -157 + Rssi (using HF output port)
    // RSSI[dBm] = -164 + Rssi (using LF output port)
    RegRSSIValue(Option<u8>) = 0x1B,

    // FHSS Start Channel (Read-Only)
    // 7 -> { 1 : PLL did not lock, 0 : PLL did lock}
    // 6 -> { 0: Header Indicates CRC off, 1 : Header Indicates CRC on}
    // 5-0 -> Current Value of Frequency Hopping Channel in Use
    RegHopChannel(Option<u8>) = 0x1C,

    // Modem PHY config 1
    RegModemConfig1(Option<RegModemConfig1>) = 0x1D,

    // Modem PHY config 2
    RegModemConfig2(Option<RegModemConfig2>) = 0x1E,
    
    // Receiver Timeout Value
    RegPreambleDetect(Option<u8>) = 0x1F,

    // Size of Preamble (MSB)
    RegPreambleMsb(Option<u8>) = 0x20,

    // Size of Preamble (LSB)
    RegPreambleLsb(Option<u8>) = 0x21,

    // LoRa Payload Length
    RegPayloadLength(Option<u8>) = 0x22,

    // LoRa Maximum Payload Length
    RegMaxPayloadLength(Option<u8>) = 0x23,

    // FHSS Hop Period
    RegHopPeriod(Option<u8>) = 0x24,

    // Address of Last Byte Written in FIFO
    RegFifoRxByteAddr(Option<u8>) = 0x25,

    // Modem PHY Config 3
    RegModemConfig3(Option<RegModemConfig3>) = 0x26,

    // 0x27 RESERVED

    // Estimated Frequency Error (MSB)
    // Ferror = (FreqError * 2^24 * BW[kHz]) / (Fxtal * 500)
    RegFeiMsb(Option<u8>) = 0x28,

    // Estimated Frequency Error (MID)
    RegFeiMid(Option<u8>) = 0x29,
    
    // Estimated Frequency Error (LSB)
    RegFeiLsb(Option<u8>) = 0x2A,

    // 0x2B RESERVED

    // Wideband RSSI Measurement
    RegRssiWideband(Option<u8>) = 0x2C,

    // 0x2D-0x30 RESERVED

    // LoRa Detection Optimize for SF6
    RegDetectOptimize = 0x31,

    // 0x32 RESERVED

    // Invert LoRa I and Q signals
    RegInvertIQ = 0x33,

    // 0x34-0x36 RESERVED

    // LoRa Detection Threshold for SF6
    RegDetectionThreshold = 0x37,

    // 0x38 RESERVED

    // LoRa Sync Word
    RegSyncWord = 0x39,

    // 0x3A-0x3F RESERVED

    // Mapping of pins DIO0 to DIO3
    RegDioMapping1 = 0x40,

    // Mapping of pins DIO4 to DIO5, ClkOut Frequency
    RegDioMapping2 = 0x41,
    
    // Semtech ID relating the silicon revision
    RegVersion = 0x42,

    // 0x44 UNUSED

    // TCXO or XTAL Input Setting
    RegTcxo = 0x4B,

    // Higher Power Settings of the PA
    RegPaDac = 0x4D,

    // Stored Temperature During the Former IQ Calibration
    RegFormerTemp = 0x5B,

    // 0x5D UNUSED

    // Adjustment of the AGC thresholds
    RegAgcRef = 0x61,

    // Adjustment of the AGC thresholds
    RegAgcThresh1 = 0x62,

    // Adjustment of the AGC thresholds
    RegAgcThresh2 = 0x63,

    // Adjustment of the AGC thresholds
    RegAgcThresh4 = 0x64,

    // Control of the PLL Bandwidth
    RegPll = 0x70,
}

#[cfg(feature = "fsk-ook")]
/// Register Layout for the FSK-OOK Mode of the sx127x Radio
#[derive(Clone, Copy, Eq, PartialEq, PartialOrd, Ord)]
#[repr(u8)]
pub(crate) enum Register {
    // Used to Read and Write the FIFO
    RegFifo(Option<u8>) = 0x00,

    // Operating Mode and LoRa/FSK Selection
    RegOpMode(Option<RegOpMode>) = 0x01,

    // Bit Rate (Most Significant Bit)
    RegBitrateMsb(Option<u8>) = 0x02,

    // Bit Rate (Least Significant Bits)
    RegBitrateLsb(Option<u8>) = 0x03,

    // Register Frequency Deviation Most Significant Bits
    RegFdevMsb(Option<u8>) = 0x04,

    // Register Frequency Deviation Least Significant Bits
    RegFdevLsb(Option<u8>) = 0x05,

    // RF Carrier Frequency (Most Significant Bits)
    RegFrfMsb(Option<u8>) = 0x06,

    // RF Carrier Frequency (Intermediate Bits)
    RegFrfMid(Option<u8>) = 0x07,

    // RF Carrier Frequency (Least Significant Bits)
    RegFrfLsb(Option<u8>) = 0x08,

    // PA Selection and Output Power Control
    RegPaConfig(Option<RegPaConfig>) = 0x09,

    // Control of PA ramp time, low phase noise PLL
    RegPaRamp(Option<RegPaRamp>) = 0x0a,

    // Over Current Protection Control
    RegOcp(Option<RegOcp>) = 0x0b,

    // LNA settings
    RegLna(Option<RegLna>) = 0x0c,

    // AFC, AGC, ctrl
    RegRxConfig(Option<RegRxConfig>) = 0x0D,

    // RSSI
    RegRssiConfig(Option<RegRssiConfig>) = 0x0E,

    // RSSI Collision Detector
    RegRssiCollision(Option<u8>) = 0x0F,

    // RSSI Threshold Control
    RegRssiThresh(Option<u8>) = 0x10,

    // RSSI value in dBm
    RegRssiValue(Option<u8>) = 0x11,

    // Channel Filter BW Control
    RegRxBw(Option<RegRxBw>) = 0x12,

    // AFC Channel Filter BW
    RegAfcBw(Option<RegAfcBw>) = 0x13,

    // OOK Demodulator
    RegOokPeak(Option<RegOokPeak>) = 0x14,

    // Threshold of the OOK demod
    RegOokFix(Option<u8>) = 0x15,

    // Average of the OOK demod
    RegOokAvg(Option<RegOokAvg>) = 0x16,

    // 0x17-0x19 are reserved

    // AFC and FEI Control
    RegAfcFei(Option<RegAfcFei>) = 0x1A,

    // Frequency Correction Value of the AFC (Most Significant Bit)
    RegAfcMsb(Option<u8>) = 0x1B,

    // Frequency Correction Value of the AFC (Least Significant Bit)
    RegAfcLsb(Option<u8>) = 0x1C,

    // Value of the Calculated Frequency Error (Most Significant Bit)
    RegFeiMsb(Option<u8>) = 0x1D,

    // Value of the Calculated Frequency Error (Least Significant Bit)
    RegFeiLsb(Option<u8>) = 0x1E,

    // Settings of the Preamble Detector
    RegPreambleDetect(Option<RegPreambleDetect>) = 0x1F,

    // Timeout Rx Request and RSSI
    RegRxTimeout1(Option<u8>) = 0x20,

    // Timeout RSSI and PayloadReady
    RegRxTimeout2(Option<u8>) = 0x21,

    // Timeout RSSI and SyncAddress
    RegRxTimeout3(Option<u8>) = 0x22,

    // Delay Between Rx Cycles
    RegRxDelay(Option<u8>) = 0x23,

    // Rc Oscillators Settings, CLK-OUT frequency
    RegOsc(Option<RegOsc>) = 0x24,

    // Preamble Length, MSB
    RegPreambleMsb(Option<u8>) = 0x25,
    
    // Preamble Length, LSB
    RegPreambleLsb(Option<u8>) = 0x26,

    // Sync Word Recognition ControlPreambleDetect
    RegSyncConfig(Option<RegSyncConfig>) = 0x27,

    // Sync Word Bytes 1
    RegSyncValue1(Option<u8>) = 0x28,

    // Sync Word Bytes 2
    RegSyncValue2(Option<u8>) = 0x29,

    // Sync Word Bytes 3
    RegSyncValue3(Option<u8>) = 0x2A,

    // Sync Word Bytes 4
    RegSyncValue4(Option<u8>) = 0x2B,

    // Sync Word Bytes 5
    RegSyncValue5(Option<u8>) = 0x2C,

    // Sync Word Bytes 6
    RegSyncValue6(Option<u8>) = 0x2D,

    // Sync Word Bytes 7
    RegSyncValue7(Option<u8>) = 0x2E,

    // Sync Word Bytes 8
    RegSyncValue8(Option<u8>) = 0x2F,

    // Packet Mode Settings
    RegPacketConfig1(Option<RegPacketConfig1>) = 0x30,

    // Packet Mode Settings
    RegPacketConfig2(Option<RegPacketConfig2>) = 0x31,

    // Payload Length Setting
    RegPayloadLength(Option<u8>) = 0x32,

    // Node Address
    RegNodeAdrs(Option<u8>) = 0x33,

    // Broadcast Address
    RegBroadcastAdrs(Option<u8>) = 0x34,

    // FIFO Threshold, Tx start condition
    RegFifoThresh(Option<RegFifoThresh>) = 0x35,

    // Top Level Sequencer Settings
    RegSeqConfig1(Option<RegSeqConfig1>) = 0x36,

    // Top Level Sequencer Settings
    RegSeqConfig2(Option<RegSeqConfig2>) = 0x37,

    // Timer 1 and 2 resolution control
    RegTimerResol(Option<RegTimerResol>) = 0x38,

    // Timer 1 setting
    RegTimer1Coef(Option<u8>) = 0x39,

    // Timer 2 setting
    RegTimer2Coef(Option<u8>) = 0x3A,

    // Image Calibration engine control
    RegImageCal(Option<RegImageCal>) = 0x3B,

    // Temperature Sensor Value
    RegTemp(Option<u8>) = 0x3C,

    // Low Battery Indicator Settings
    RegLowBat(Option<RegLowBat>) = 0x3D,

    // Status Register: PLL Lock state, Timeout, RSSI
    RegIrqFlags1(Option<RegIrqFlags1>) = 0x3E,

    // Status Register: FIFO Handling Flags, Low Battery
    RegIrqFlags2(Option<RegIrqFlags2>) = 0x3F,

    // Mapping of pins DIO0 to DIO3
    RegDioMapping1(Option<RegDioMapping1>) = 0x40,

    // Mapping of pins Dio4 and Dio5, ClkOut Frequency
    RegDioMapping2(Option<RegDioMapping2>) = 0x41,

    // Semtech ID relating the silicon revision
    RegVersion(Option<u8>) = 0x42,

    // Control the fast frequency hop-ping mode
    RegPllHop(Option<bool>) = 0x44,

    // TCXO or XTAL input setting
    // { 0 : Crystal Oscillator with eternal Crystal, 1 : External clipped sine TCXO AC-connected to XTA pin}
    RegTcxo(Option<bool>) = 0x4B,

    // Higher Power SEttings of the PA
    // 0x04 -> Default, 0x07 +20dBm
    RegPaDac(Option<u8>) = 0x4D,

    // Stored Temperature During the Former IQ Calibration
    RegFormerTemp(Option<u8>) = 0x5B,

    // Fractional Part in the Bit Rate Divions Ratio
    RegBitRateFrac(Option<u8>) = 0x5D,

    // Adjustment of the AGC thresholds
    RegAgcRef(Option<u8>) = 0x61,

    // Adjustment of the AGC thresholds
    RegAgcThresh1(Option<u8>) = 0x62,

    // Adjustment of the AGC thresholds
    RegAgcThresh2(Option<u8>) = 0x63,

    // Adjustment of the AGC thresholds
    RegAgcThresh3(Option<u8>) = 0x64,

    // Control the PLL bandwidth
    RegPll(Option<u8>) = 0x70,
}

#[derive(Clone, Copy)]
pub(crate) enum PaConfig {
    PaBoost = 0x80,
    PaOutputRfoPin = 0,
}

#[derive(Clone, Copy)]
pub(crate) enum IRQ {
    IrqTxDoneMask = 0x08,
    IrqPayloadCrcErrorMask = 0x20,
    IrqRxDoneMask = 0x40,
}

impl Register {
    pub(crate) fn addr(&self) -> u8 {
        unsafe { *<*const _>::from(self).cast::<u8>() }
    }
}

impl PaConfig {
    pub(crate) fn addr(self) -> u8 {
        self as u8
    }
}

impl IRQ {
    pub(crate) fn addr(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy)]
pub(crate) enum FskDataModulationShaping {
    None = 1,
    GaussianBt1d0 = 2,
    GaussianBt0d5 = 10,
    GaussianBt0d3 = 11
}

#[derive(Clone, Copy)]
pub(crate) enum FskRampUpRamDown {
    #[allow(non_camel_case_types)]
    _3d4ms = 0b000,
    _2ms = 0b0001,
    _1ms = 0b0010,
    _500us = 0b0011,
    _250us = 0b0100,
    _125us = 0b0101,
    _100us = 0b0110,
    _62us = 0b0111,
    _50us = 0b1000,
    _40us = 0b1001,
    _31us = 0b1010,
    _25us = 0b1011,
    _20us = 0b1100,
    _15us = 0b1101,
    _12us = 0b1110,
    _10us = 0b1111
}