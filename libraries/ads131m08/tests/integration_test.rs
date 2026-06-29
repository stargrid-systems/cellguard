//! Black-box tests that drive the public API through a mocked SPI device.
//!
//! The device always streams an output CRC, so the response helpers compute a
//! valid CCITT CRC over each assembled frame.

use ads131m08::Ads131m08;
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
fn read_data_decodes_all_channels() {
    let samples = [1, -1, 0x7F_FFFF, -0x80_0000, 0x1234, -0x1234, 2, -2];
    let txns = transfer(vec![0; FULL_FRAME_BYTES], data_response(0x0500, samples));
    run(&txns, |device| {
        let mut device = device.configure();
        let mut channels = [0i32; 8];
        let Ok(()) = device.read_data(&mut channels) else {
            panic!("read_data failed");
        };
        assert_eq!(channels, samples);
    });
}

#[test]
fn lock_confirms_via_status_register() {
    let mut txns = write(word(LOCK).to_vec());
    txns.extend(read_single(STATUS_ADDR, 0x8000));
    run(&txns, |device| {
        let mut device = device.configure();
        let Ok(Ok(())) = device.lock_registers() else {
            panic!("lock not confirmed");
        };
    });
}

#[test]
fn unlock_confirms_via_status_register() {
    let mut txns = write(word(UNLOCK).to_vec());
    txns.extend(read_single(STATUS_ADDR, 0x0000));
    run(&txns, |device| {
        let mut device = device.configure();
        let Ok(Ok(())) = device.unlock_registers() else {
            panic!("unlock not confirmed");
        };
    });
}
