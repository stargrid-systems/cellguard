//! Black-box tests that drive the public API through a mocked SPI device.
//!
//! The device always streams an output CRC, so the response helpers compute a
//! valid CCITT CRC over each assembled frame.

use ads131m08::{Ads131m08, Config};
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

type Spi = SpiMock<u8>;
type Txn = SpiTransaction<u8>;

const WORD_BYTES: usize = 3;
const FULL_FRAME_BYTES: usize = 10 * WORD_BYTES;

const NULL: u16 = 0x0000;
const RESET: u16 = 0x0011;
const LOCK: u16 = 0x0555;
const UNLOCK: u16 = 0x0666;

const ID_ADDR: u16 = 0x00;
const STATUS_ADDR: u16 = 0x01;
const GAIN2_ADDR: u16 = 0x05;

const fn rreg(addr: u16, count: u16) -> u16 {
    0xA000 | (addr << 7) | (count - 1)
}

fn crc16_ccitt(data: &[u8]) -> u16 {
    data.iter().fold(0xFFFF, |seed, &byte| {
        let mut crc = seed ^ (u16::from(byte) << 8);
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
        crc
    })
}

/// A 16-bit value MSB-aligned in a 24-bit word.
const fn word(value: u16) -> [u8; WORD_BYTES] {
    let [hi, lo] = value.to_be_bytes();
    [hi, lo, 0]
}

/// The low 24 bits of a sample as a conversion-data word.
const fn sample(value: i32) -> [u8; WORD_BYTES] {
    let [_, b1, b2, b3] = value.to_be_bytes();
    [b1, b2, b3]
}

/// A full input frame carrying a single command followed by zero words.
fn full_command(command: u16) -> Vec<u8> {
    let mut bytes = word(command).to_vec();
    bytes.resize(FULL_FRAME_BYTES, 0);
    bytes
}

/// A response frame built from 16-bit words with a trailing CRC word.
fn response(words: &[u16]) -> Vec<u8> {
    let mut bytes: Vec<u8> = words.iter().flat_map(|&w| word(w)).collect();
    bytes.extend_from_slice(&word(crc16_ccitt(&bytes)));
    bytes
}

/// A conversion-data response: status word, eight samples, then a CRC word.
fn data_response(status: u16, samples: [i32; 8]) -> Vec<u8> {
    let mut bytes = word(status).to_vec();
    for value in samples {
        bytes.extend_from_slice(&sample(value));
    }
    bytes.extend_from_slice(&word(crc16_ccitt(&bytes)));
    bytes
}

fn write(bytes: Vec<u8>) -> Vec<Txn> {
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(bytes),
        SpiTransaction::transaction_end(),
    ]
}

fn transfer(input: Vec<u8>, output: Vec<u8>) -> Vec<Txn> {
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::transfer_in_place(input, output),
        SpiTransaction::transaction_end(),
    ]
}

/// Transactions for reading a single register: the RREG command in a short
/// frame, then a full data frame whose response word holds the value.
fn read_single(addr: u16, value: u16) -> Vec<Txn> {
    let mut txns = write(word(rreg(addr, 1)).to_vec());
    let mut words = vec![value];
    words.resize(9, 0);
    txns.extend(transfer(vec![0; FULL_FRAME_BYTES], response(&words)));
    txns
}

/// Transactions for writing a single register: the WREG full frame, then the
/// verifying readback.
fn write_single(addr: u16, value: u16) -> Vec<Txn> {
    let wreg = 0x6000 | (addr << 7);
    let mut block = word(wreg).to_vec();
    block.extend_from_slice(&word(value));
    block.resize(FULL_FRAME_BYTES, 0);
    let mut txns = write(block);
    txns.extend(read_single(addr, value));
    txns
}

/// The register image produced by [`Config::default`], `02h` through `30h`.
const DEFAULT_IMAGE: [u16; 47] = [
    0x0110, 0xFF0E, 0x0000, 0x0000, 0x0600, 0x0000, 0x0000, // MODE..THRSHLD_LSB
    0x0000, 0x0000, 0x0000, 0x8000, 0x0000, // channel 0
    0x0000, 0x0000, 0x0000, 0x8000, 0x0000, // channel 1
    0x0000, 0x0000, 0x0000, 0x8000, 0x0000, // channel 2
    0x0000, 0x0000, 0x0000, 0x8000, 0x0000, // channel 3
    0x0000, 0x0000, 0x0000, 0x8000, 0x0000, // channel 4
    0x0000, 0x0000, 0x0000, 0x8000, 0x0000, // channel 5
    0x0000, 0x0000, 0x0000, 0x8000, 0x0000, // channel 6
    0x0000, 0x0000, 0x0000, 0x8000, 0x0000, // channel 7
];

/// Transactions for `configure(Config::default())`: the block write followed by
/// the verifying block read.
fn configure_default() -> Vec<Txn> {
    // WREG and RREG of all 47 writable registers starting at MODE (02h).
    const WREG_BLOCK: u16 = 0x612E;
    const RREG_BLOCK: u16 = 0xA12E;

    let mut block: Vec<u8> = word(WREG_BLOCK).to_vec();
    for &value in &DEFAULT_IMAGE {
        block.extend_from_slice(&word(value));
    }
    let mut txns = write(block);

    txns.extend(write(word(RREG_BLOCK).to_vec()));
    let mut response_words = vec![0xE12E];
    response_words.extend_from_slice(&DEFAULT_IMAGE);
    let read_words = 1 + DEFAULT_IMAGE.len() + 1;
    txns.extend(transfer(
        vec![0; read_words * WORD_BYTES],
        response(&response_words),
    ));
    txns
}

fn run(txns: &[Txn], body: impl FnOnce(Ads131m08<Spi>)) {
    let mut spi = SpiMock::new(txns);
    let device = Ads131m08::new(spi.clone());
    body(device);
    spi.done();
}

#[test]
fn reset_start_sends_full_frame() {
    let txns = write(full_command(RESET));
    run(&txns, |mut device| {
        let Ok(()) = device.reset_device_start() else {
            panic!("reset_device_start failed");
        };
    });
}

#[test]
fn reset_complete_recognizes_acknowledgment() {
    let txns = transfer(word(NULL).to_vec(), vec![0xFF, 0x28, 0x00]);
    run(&txns, |mut device| {
        let Ok(Ok(())) = device.reset_device_complete() else {
            panic!("reset not acknowledged");
        };
    });
}

#[test]
fn read_id_reports_channel_count() {
    let txns = read_single(ID_ADDR, 0x2800);
    run(&txns, |mut device| {
        let Ok(id) = device.read_id() else {
            panic!("read_id failed");
        };
        assert_eq!(id.channel_count(), 8);
    });
}

#[test]
fn configure_writes_block_and_verifies() {
    let txns = configure_default();
    run(&txns, |device| {
        let Ok(_device) = device.configure(Config::default()) else {
            panic!("configure failed");
        };
    });
}

#[test]
fn set_gain_does_read_modify_write() {
    use ads131m08::Gain;

    // Channel 5 lives in GAIN2 at bit offset 4. Gain X8 is code 0b011.
    let mut txns = configure_default();
    txns.extend(read_single(GAIN2_ADDR, 0x0000));
    txns.extend(write_single(GAIN2_ADDR, 0x0030));
    run(&txns, |device| {
        let Ok(mut device) = device.configure(Config::default()) else {
            panic!("configure failed");
        };
        let Ok(Ok(())) = device.set_gain(5, Gain::X8) else {
            panic!("set_gain failed");
        };
    });
}

#[test]
fn read_data_decodes_all_channels() {
    let samples = [1, -1, 0x7F_FFFF, -0x80_0000, 0x1234, -0x1234, 2, -2];
    let mut txns = configure_default();
    txns.extend(transfer(
        vec![0; FULL_FRAME_BYTES],
        data_response(0x0500, samples),
    ));
    run(&txns, |device| {
        let Ok(mut device) = device.configure(Config::default()) else {
            panic!("configure failed");
        };
        let mut channels = [0i32; 8];
        let Ok(status) = device.read_data(&mut channels) else {
            panic!("read_data failed");
        };
        assert_eq!(channels, samples);
        // 0x0500 is the reset STATUS: reset flag set, no data-ready flags.
        assert!(status.reset_occurred());
        assert!(!status.data_ready(0));
    });
}

#[test]
fn read_data_after_pause_reads_twice() {
    let stale = [0; 8];
    let fresh = [11, 22, 33, 44, 55, 66, 77, 88];
    let mut txns = configure_default();
    txns.extend(transfer(
        vec![0; FULL_FRAME_BYTES],
        data_response(0x00FF, stale),
    ));
    txns.extend(transfer(
        vec![0; FULL_FRAME_BYTES],
        data_response(0x00FF, fresh),
    ));
    run(&txns, |device| {
        let Ok(mut device) = device.configure(Config::default()) else {
            panic!("configure failed");
        };
        let mut channels = [0i32; 8];
        let Ok(_status) = device.read_data_after_pause(&mut channels) else {
            panic!("read_data_after_pause failed");
        };
        assert_eq!(channels, fresh);
    });
}

#[test]
fn lock_confirms_via_status_register() {
    let mut txns = configure_default();
    txns.extend(write(word(LOCK).to_vec()));
    txns.extend(read_single(STATUS_ADDR, 0x8000));
    run(&txns, |device| {
        let Ok(mut device) = device.configure(Config::default()) else {
            panic!("configure failed");
        };
        let Ok(Ok(())) = device.lock_registers() else {
            panic!("lock not confirmed");
        };
    });
}

#[test]
fn unlock_confirms_via_status_register() {
    let mut txns = configure_default();
    txns.extend(write(word(UNLOCK).to_vec()));
    txns.extend(read_single(STATUS_ADDR, 0x0000));
    run(&txns, |device| {
        let Ok(mut device) = device.configure(Config::default()) else {
            panic!("configure failed");
        };
        let Ok(Ok(())) = device.unlock_registers() else {
            panic!("unlock not confirmed");
        };
    });
}
