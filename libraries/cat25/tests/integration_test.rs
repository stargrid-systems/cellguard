//! Black-box tests that drive the public API through a mocked SPI device.
//!
//! The in-module unit tests cover CAT25128. These lean on CAT25M01 to exercise
//! three address bytes and the 256-byte page.

use cat25::{BlockProtection, CAT25M01, CAT25128, Cat25, Error, Model};
use embedded_hal_mock::eh1::delay::NoopDelay;
use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

/// The driver gives up after this many status polls. Mirrors the private
/// constant in the crate.
const WRITE_POLL_ATTEMPTS: usize = 10;

type Spi = SpiMock<u8>;

fn build(model: Model, expectations: &[SpiTransaction<u8>]) -> Cat25<Spi, NoopDelay> {
    Cat25::new(SpiMock::new(expectations), model, NoopDelay::new())
}

fn finish(cat: Cat25<Spi, NoopDelay>) {
    let (mut spi, _delay) = cat.into_parts();
    spi.done();
}

fn wren() -> Vec<SpiTransaction<u8>> {
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x06]),
        SpiTransaction::transaction_end(),
    ]
}

fn rdsr(value: u8) -> Vec<SpiTransaction<u8>> {
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x05]),
        SpiTransaction::read_vec(vec![value]),
        SpiTransaction::transaction_end(),
    ]
}

fn wrsr(value: u8) -> Vec<SpiTransaction<u8>> {
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x01, value]),
        SpiTransaction::transaction_end(),
    ]
}

fn wrdi() -> Vec<SpiTransaction<u8>> {
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(vec![0x04]),
        SpiTransaction::transaction_end(),
    ]
}

fn read_cmd(header: Vec<u8>, data: Vec<u8>) -> Vec<SpiTransaction<u8>> {
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(header),
        SpiTransaction::read_vec(data),
        SpiTransaction::transaction_end(),
    ]
}

fn write_cmd(header: Vec<u8>, data: Vec<u8>) -> Vec<SpiTransaction<u8>> {
    vec![
        SpiTransaction::transaction_start(),
        SpiTransaction::write_vec(header),
        SpiTransaction::write_vec(data),
        SpiTransaction::transaction_end(),
    ]
}

#[test]
fn model_geometry_matches_datasheets() {
    assert_eq!(CAT25128.size(), 16_384);
    assert_eq!(CAT25128.page_size(), 64);
    assert_eq!(CAT25128.address_bytes(), 2);

    assert_eq!(CAT25M01.size(), 131_072);
    assert_eq!(CAT25M01.page_size(), 256);
    assert_eq!(CAT25M01.address_bytes(), 3);
}

#[test]
fn driver_exposes_its_model() {
    let cat = build(CAT25M01, &[]);
    assert_eq!(cat.model().size(), 131_072);
    finish(cat);
}

#[test]
fn status_decodes_fields() {
    let mut cat = build(CAT25128, &rdsr(0b1000_0010));
    let status = cat.status().unwrap();
    assert!(status.ready());
    assert!(status.write_enabled());
    assert!(status.write_protect_enabled());
    finish(cat);
}

#[test]
fn read_uses_three_address_bytes_on_m01() {
    let expected = read_cmd(vec![0x03, 0x01, 0x23, 0x45], vec![0xAA, 0xBB]);
    let mut cat = build(CAT25M01, &expected);
    let mut buf = [0u8; 2];
    cat.read(0x01_2345, &mut buf).unwrap();
    assert_eq!(buf, [0xAA, 0xBB]);
    finish(cat);
}

#[test]
fn write_single_page_enables_and_waits() {
    let mut expected = Vec::new();
    expected.extend(wren());
    expected.extend(write_cmd(vec![0x02, 0x00, 0x00, 0x10], vec![0xDE, 0xAD]));
    expected.extend(rdsr(0x00));

    let mut cat = build(CAT25M01, &expected);
    cat.write(0x10, &[0xDE, 0xAD]).unwrap();
    finish(cat);
}

#[test]
fn write_splits_on_m01_page_boundary() {
    // Page size 256. Starting at 255 with 3 bytes spans two pages.
    let mut expected = Vec::new();
    expected.extend(wren());
    expected.extend(write_cmd(vec![0x02, 0x00, 0x00, 0xFF], vec![0xA0]));
    expected.extend(rdsr(0x00));
    expected.extend(wren());
    expected.extend(write_cmd(vec![0x02, 0x00, 0x01, 0x00], vec![0xA1, 0xA2]));
    expected.extend(rdsr(0x00));

    let mut cat = build(CAT25M01, &expected);
    cat.write(255, &[0xA0, 0xA1, 0xA2]).unwrap();
    finish(cat);
}

#[test]
fn write_polls_until_ready() {
    let mut expected = Vec::new();
    expected.extend(wren());
    expected.extend(write_cmd(vec![0x02, 0x00, 0x00], vec![0x01]));
    expected.extend(rdsr(0b0000_0001)); // busy
    expected.extend(rdsr(0x00)); // ready, write enable cleared

    let mut cat = build(CAT25128, &expected);
    cat.write(0, &[0x01]).unwrap();
    finish(cat);
}

#[test]
fn write_rejected_reports_protected() {
    let mut expected = Vec::new();
    expected.extend(wren());
    expected.extend(write_cmd(vec![0x02, 0x00, 0x00], vec![0x01]));
    // Ready but write enable still set means the write did not run.
    expected.extend(rdsr(0b0000_0010));

    let mut cat = build(CAT25128, &expected);
    assert_eq!(cat.write(0, &[0x01]), Err(Error::WriteProtected));
    finish(cat);
}

#[test]
fn write_disable_sends_wrdi() {
    let mut cat = build(CAT25128, &wrdi());
    cat.write_disable().unwrap();
    finish(cat);
}

#[test]
fn write_disable_clears_latch_after_rejected_write() {
    let mut expected = Vec::new();
    expected.extend(wren());
    expected.extend(write_cmd(vec![0x02, 0x00, 0x00], vec![0x01]));
    expected.extend(rdsr(0b0000_0010)); // ready, but write enable still set
    expected.extend(wrdi());

    let mut cat = build(CAT25128, &expected);
    assert_eq!(cat.write(0, &[0x01]), Err(Error::WriteProtected));
    cat.write_disable().unwrap();
    finish(cat);
}

#[test]
fn write_times_out_when_never_ready() {
    let mut expected = Vec::new();
    expected.extend(wren());
    expected.extend(write_cmd(vec![0x02, 0x00, 0x00], vec![0x01]));
    for _ in 0..WRITE_POLL_ATTEMPTS {
        expected.extend(rdsr(0b0000_0001)); // always busy
    }

    let mut cat = build(CAT25128, &expected);
    assert_eq!(cat.write(0, &[0x01]), Err(Error::Timeout));
    finish(cat);
}

#[test]
fn read_id_page_selects_then_reads() {
    let mut expected = Vec::new();
    expected.extend(rdsr(0x00)); // read current status for the modify
    expected.extend(wren());
    expected.extend(wrsr(0x40)); // IPL set
    expected.extend(rdsr(0x00));
    expected.extend(read_cmd(
        vec![0x03, 0x00, 0x00, 0x00],
        vec![0x01, 0x02, 0x03, 0x04],
    ));

    let mut cat = build(CAT25M01, &expected);
    let mut buf = [0u8; 4];
    cat.read_id_page(0, &mut buf).unwrap();
    assert_eq!(buf, [0x01, 0x02, 0x03, 0x04]);
    finish(cat);
}

#[test]
fn write_id_page_selects_then_writes() {
    let mut expected = Vec::new();
    expected.extend(rdsr(0x00));
    expected.extend(wren());
    expected.extend(wrsr(0x40)); // IPL set
    expected.extend(rdsr(0x00));
    expected.extend(wren());
    expected.extend(write_cmd(vec![0x02, 0x00, 0x00, 0x05], vec![0x11, 0x22]));
    expected.extend(rdsr(0x00));

    let mut cat = build(CAT25M01, &expected);
    cat.write_id_page(5, &[0x11, 0x22]).unwrap();
    finish(cat);
}

#[test]
fn lock_id_page_sets_lip_bit() {
    let mut expected = Vec::new();
    expected.extend(rdsr(0x00));
    expected.extend(wren());
    expected.extend(wrsr(0x10)); // LIP set
    expected.extend(rdsr(0x00));

    let mut cat = build(CAT25M01, &expected);
    cat.lock_id_page().unwrap();
    finish(cat);
}

#[test]
fn set_block_protection_preserves_other_writable_bits() {
    let mut expected = Vec::new();
    // Current status has WPEN set. Setting block protection must keep it.
    expected.extend(rdsr(0b1000_0000));
    expected.extend(wren());
    expected.extend(wrsr(0b1000_1100)); // WPEN preserved plus BlockProtection::All
    expected.extend(rdsr(0x00));

    let mut cat = build(CAT25M01, &expected);
    cat.set_block_protection(BlockProtection::All).unwrap();
    finish(cat);
}

#[test]
fn set_write_protect_enabled_writes_status() {
    let mut expected = Vec::new();
    expected.extend(rdsr(0x00));
    expected.extend(wren());
    expected.extend(wrsr(0b1000_0000)); // WPEN
    expected.extend(rdsr(0x00));

    let mut cat = build(CAT25M01, &expected);
    cat.set_write_protect_enabled(true).unwrap();
    finish(cat);
}

#[test]
fn read_rejects_out_of_bounds() {
    let mut cat = build(CAT25128, &[]);
    let mut buf = [0u8; 4];
    assert_eq!(cat.read(0x3FFF, &mut buf), Err(Error::OutOfBounds));
    finish(cat);
}

#[test]
fn write_rejects_out_of_bounds() {
    let mut cat = build(CAT25M01, &[]);
    assert_eq!(cat.write(131_071, &[0, 0]), Err(Error::OutOfBounds));
    finish(cat);
}

#[test]
fn id_page_write_rejects_out_of_bounds() {
    let mut cat = build(CAT25M01, &[]);
    // The M01 identification page is 256 bytes.
    assert_eq!(cat.write_id_page(255, &[0, 0]), Err(Error::OutOfBounds));
    finish(cat);
}
