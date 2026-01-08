use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction};
use tca9535::{Address, Configuration, Input, Output, PinIndex, PolarityInversion, Tca9535};

#[test]
fn test_address_new_valid() {
    // Test all valid addresses
    assert_eq!(Address::new(0x20), Some(Address::Lll));
    assert_eq!(Address::new(0x21), Some(Address::Llh));
    assert_eq!(Address::new(0x22), Some(Address::Lhl));
    assert_eq!(Address::new(0x23), Some(Address::Lhh));
    assert_eq!(Address::new(0x24), Some(Address::Hll));
    assert_eq!(Address::new(0x25), Some(Address::Hlh));
    assert_eq!(Address::new(0x26), Some(Address::Hhl));
    assert_eq!(Address::new(0x27), Some(Address::Hhh));
}

#[test]
fn test_address_new_invalid() {
    // Test invalid addresses
    assert_eq!(Address::new(0x00), None);
    assert_eq!(Address::new(0x1F), None);
    assert_eq!(Address::new(0x28), None);
    assert_eq!(Address::new(0xFF), None);
}

#[test]
fn test_address_get() {
    assert_eq!(Address::Lll.get(), 0x20);
    assert_eq!(Address::Llh.get(), 0x21);
    assert_eq!(Address::Lhl.get(), 0x22);
    assert_eq!(Address::Lhh.get(), 0x23);
    assert_eq!(Address::Hll.get(), 0x24);
    assert_eq!(Address::Hlh.get(), 0x25);
    assert_eq!(Address::Hhl.get(), 0x26);
    assert_eq!(Address::Hhh.get(), 0x27);
}

#[test]
fn test_pin_index_bit() {
    assert_eq!(PinIndex::P0.bit(), 0);
    assert_eq!(PinIndex::P1.bit(), 1);
    assert_eq!(PinIndex::P2.bit(), 2);
    assert_eq!(PinIndex::P3.bit(), 3);
    assert_eq!(PinIndex::P4.bit(), 4);
    assert_eq!(PinIndex::P5.bit(), 5);
    assert_eq!(PinIndex::P6.bit(), 6);
    assert_eq!(PinIndex::P7.bit(), 7);
    assert_eq!(PinIndex::P8.bit(), 8);
    assert_eq!(PinIndex::P9.bit(), 9);
    assert_eq!(PinIndex::P10.bit(), 10);
    assert_eq!(PinIndex::P11.bit(), 11);
    assert_eq!(PinIndex::P12.bit(), 12);
    assert_eq!(PinIndex::P13.bit(), 13);
    assert_eq!(PinIndex::P14.bit(), 14);
    assert_eq!(PinIndex::P15.bit(), 15);
}

#[test]
fn test_pin_index_mask() {
    assert_eq!(PinIndex::P0.mask(), 0x0001);
    assert_eq!(PinIndex::P1.mask(), 0x0002);
    assert_eq!(PinIndex::P2.mask(), 0x0004);
    assert_eq!(PinIndex::P3.mask(), 0x0008);
    assert_eq!(PinIndex::P4.mask(), 0x0010);
    assert_eq!(PinIndex::P5.mask(), 0x0020);
    assert_eq!(PinIndex::P6.mask(), 0x0040);
    assert_eq!(PinIndex::P7.mask(), 0x0080);
    assert_eq!(PinIndex::P8.mask(), 0x0100);
    assert_eq!(PinIndex::P9.mask(), 0x0200);
    assert_eq!(PinIndex::P10.mask(), 0x0400);
    assert_eq!(PinIndex::P11.mask(), 0x0800);
    assert_eq!(PinIndex::P12.mask(), 0x1000);
    assert_eq!(PinIndex::P13.mask(), 0x2000);
    assert_eq!(PinIndex::P14.mask(), 0x4000);
    assert_eq!(PinIndex::P15.mask(), 0x8000);
}

#[test]
fn test_input_is_high() {
    let input = Input(0b1010_1010_1010_1010);
    assert!(!input.is_high(PinIndex::P0));
    assert!(input.is_high(PinIndex::P1));
    assert!(!input.is_high(PinIndex::P2));
    assert!(input.is_high(PinIndex::P3));
    assert!(!input.is_high(PinIndex::P4));
    assert!(input.is_high(PinIndex::P5));
    assert!(!input.is_high(PinIndex::P6));
    assert!(input.is_high(PinIndex::P7));
    assert!(!input.is_high(PinIndex::P8));
    assert!(input.is_high(PinIndex::P9));
    assert!(!input.is_high(PinIndex::P10));
    assert!(input.is_high(PinIndex::P11));
    assert!(!input.is_high(PinIndex::P12));
    assert!(input.is_high(PinIndex::P13));
    assert!(!input.is_high(PinIndex::P14));
    assert!(input.is_high(PinIndex::P15));
}

#[test]
fn test_input_is_low() {
    let input = Input(0b1010_1010_1010_1010);
    assert!(input.is_low(PinIndex::P0));
    assert!(!input.is_low(PinIndex::P1));
    assert!(input.is_low(PinIndex::P2));
    assert!(!input.is_low(PinIndex::P3));
    assert!(input.is_low(PinIndex::P4));
    assert!(!input.is_low(PinIndex::P5));
    assert!(input.is_low(PinIndex::P6));
    assert!(!input.is_low(PinIndex::P7));
    assert!(input.is_low(PinIndex::P8));
    assert!(!input.is_low(PinIndex::P9));
    assert!(input.is_low(PinIndex::P10));
    assert!(!input.is_low(PinIndex::P11));
    assert!(input.is_low(PinIndex::P12));
    assert!(!input.is_low(PinIndex::P13));
    assert!(input.is_low(PinIndex::P14));
    assert!(!input.is_low(PinIndex::P15));
}

#[test]
fn test_output_with_high() {
    let output = Output(0x0000);
    let output = output.with_high(PinIndex::P0);
    assert_eq!(output.0, 0x0001);
    let output = output.with_high(PinIndex::P7);
    assert_eq!(output.0, 0x0081);
    let output = output.with_high(PinIndex::P15);
    assert_eq!(output.0, 0x8081);
}

#[test]
fn test_output_with_low() {
    let output = Output(0xFFFF);
    let output = output.with_low(PinIndex::P0);
    assert_eq!(output.0, 0xFFFE);
    let output = output.with_low(PinIndex::P7);
    assert_eq!(output.0, 0xFF7E);
    let output = output.with_low(PinIndex::P15);
    assert_eq!(output.0, 0x7F7E);
}

#[test]
fn test_output_is_high() {
    let output = Output(0b0101_0101_0101_0101);
    assert!(output.is_high(PinIndex::P0));
    assert!(!output.is_high(PinIndex::P1));
    assert!(output.is_high(PinIndex::P2));
    assert!(!output.is_high(PinIndex::P3));
    assert!(output.is_high(PinIndex::P4));
    assert!(!output.is_high(PinIndex::P5));
    assert!(output.is_high(PinIndex::P6));
    assert!(!output.is_high(PinIndex::P7));
    assert!(output.is_high(PinIndex::P8));
    assert!(!output.is_high(PinIndex::P9));
    assert!(output.is_high(PinIndex::P10));
    assert!(!output.is_high(PinIndex::P11));
    assert!(output.is_high(PinIndex::P12));
    assert!(!output.is_high(PinIndex::P13));
    assert!(output.is_high(PinIndex::P14));
    assert!(!output.is_high(PinIndex::P15));
}

#[test]
fn test_polarity_inversion_with_inverted() {
    let polarity = PolarityInversion(0x0000);
    let polarity = polarity.with_inverted(PinIndex::P0);
    assert_eq!(polarity.0, 0x0001);
    let polarity = polarity.with_inverted(PinIndex::P7);
    assert_eq!(polarity.0, 0x0081);
    let polarity = polarity.with_inverted(PinIndex::P15);
    assert_eq!(polarity.0, 0x8081);
}

#[test]
fn test_polarity_inversion_with_normal() {
    let polarity = PolarityInversion(0xFFFF);
    let polarity = polarity.with_normal(PinIndex::P0);
    assert_eq!(polarity.0, 0xFFFE);
    let polarity = polarity.with_normal(PinIndex::P7);
    assert_eq!(polarity.0, 0xFF7E);
    let polarity = polarity.with_normal(PinIndex::P15);
    assert_eq!(polarity.0, 0x7F7E);
}

#[test]
fn test_polarity_inversion_is_inverted() {
    let polarity = PolarityInversion(0b1100_1100_1100_1100);
    assert!(!polarity.is_inverted(PinIndex::P0));
    assert!(!polarity.is_inverted(PinIndex::P1));
    assert!(polarity.is_inverted(PinIndex::P2));
    assert!(polarity.is_inverted(PinIndex::P3));
    assert!(!polarity.is_inverted(PinIndex::P4));
    assert!(!polarity.is_inverted(PinIndex::P5));
    assert!(polarity.is_inverted(PinIndex::P6));
    assert!(polarity.is_inverted(PinIndex::P7));
    assert!(!polarity.is_inverted(PinIndex::P8));
    assert!(!polarity.is_inverted(PinIndex::P9));
    assert!(polarity.is_inverted(PinIndex::P10));
    assert!(polarity.is_inverted(PinIndex::P11));
    assert!(!polarity.is_inverted(PinIndex::P12));
    assert!(!polarity.is_inverted(PinIndex::P13));
    assert!(polarity.is_inverted(PinIndex::P14));
    assert!(polarity.is_inverted(PinIndex::P15));
}

#[test]
fn test_polarity_inversion_is_normal() {
    let polarity = PolarityInversion(0b1010_1010_1010_1010);
    assert!(polarity.is_normal(PinIndex::P0));
    assert!(!polarity.is_normal(PinIndex::P1));
    assert!(polarity.is_normal(PinIndex::P2));
    assert!(!polarity.is_normal(PinIndex::P3));
    assert!(polarity.is_normal(PinIndex::P4));
    assert!(!polarity.is_normal(PinIndex::P5));
    assert!(polarity.is_normal(PinIndex::P6));
    assert!(!polarity.is_normal(PinIndex::P7));
    assert!(polarity.is_normal(PinIndex::P8));
    assert!(!polarity.is_normal(PinIndex::P9));
    assert!(polarity.is_normal(PinIndex::P10));
    assert!(!polarity.is_normal(PinIndex::P11));
    assert!(polarity.is_normal(PinIndex::P12));
    assert!(!polarity.is_normal(PinIndex::P13));
    assert!(polarity.is_normal(PinIndex::P14));
    assert!(!polarity.is_normal(PinIndex::P15));
}

#[test]
fn test_configuration_with_input() {
    let config = Configuration(0x0000);
    let config = config.with_input(PinIndex::P0);
    assert_eq!(config.0, 0x0001);
    let config = config.with_input(PinIndex::P7);
    assert_eq!(config.0, 0x0081);
    let config = config.with_input(PinIndex::P15);
    assert_eq!(config.0, 0x8081);
}

#[test]
fn test_configuration_with_output() {
    let config = Configuration(0xFFFF);
    let config = config.with_output(PinIndex::P0);
    assert_eq!(config.0, 0xFFFE);
    let config = config.with_output(PinIndex::P7);
    assert_eq!(config.0, 0xFF7E);
    let config = config.with_output(PinIndex::P15);
    assert_eq!(config.0, 0x7F7E);
}

#[test]
fn test_configuration_is_input() {
    let config = Configuration(0b0011_0011_0011_0011);
    assert!(config.is_input(PinIndex::P0));
    assert!(config.is_input(PinIndex::P1));
    assert!(!config.is_input(PinIndex::P2));
    assert!(!config.is_input(PinIndex::P3));
    assert!(config.is_input(PinIndex::P4));
    assert!(config.is_input(PinIndex::P5));
    assert!(!config.is_input(PinIndex::P6));
    assert!(!config.is_input(PinIndex::P7));
    assert!(config.is_input(PinIndex::P8));
    assert!(config.is_input(PinIndex::P9));
    assert!(!config.is_input(PinIndex::P10));
    assert!(!config.is_input(PinIndex::P11));
    assert!(config.is_input(PinIndex::P12));
    assert!(config.is_input(PinIndex::P13));
    assert!(!config.is_input(PinIndex::P14));
    assert!(!config.is_input(PinIndex::P15));
}

#[test]
fn test_read_input() {
    let expectations = [Transaction::write_read(0x20, vec![0x00], vec![0x34, 0x12])];
    let mock = I2cMock::new(&expectations);

    let mut device = Tca9535::new(mock, Address::Lll);
    let result = device.read_input().unwrap();
    assert_eq!(result.0, 0x1234);

    let mut i2c = device.into_inner();
    i2c.done();
}

#[test]
fn test_read_output() {
    let expectations = [Transaction::write_read(0x20, vec![0x02], vec![0xCD, 0xAB])];
    let mock = I2cMock::new(&expectations);

    let mut device = Tca9535::new(mock, Address::Lll);
    let result = device.read_output().unwrap();
    assert_eq!(result.0, 0xABCD);

    device.into_inner().done();
}

#[test]
fn test_write_output() {
    let expectations = [Transaction::write(0x20, vec![0x02, 0x78, 0x56])];
    let mock = I2cMock::new(&expectations);

    let mut device = Tca9535::new(mock, Address::Lll);
    device.write_output(Output(0x5678)).unwrap();

    device.into_inner().done();
}

#[test]
fn test_read_polarity_inversion() {
    let expectations = [Transaction::write_read(0x20, vec![0x04], vec![0xEF, 0xBE])];
    let mock = I2cMock::new(&expectations);

    let mut device = Tca9535::new(mock, Address::Lll);
    let result = device.read_polarity_inversion().unwrap();
    assert_eq!(result.0, 0xBEEF);

    device.into_inner().done();
}

#[test]
fn test_write_polarity_inversion() {
    let expectations = [Transaction::write(0x20, vec![0x04, 0xAD, 0xDE])];
    let mock = I2cMock::new(&expectations);

    let mut device = Tca9535::new(mock, Address::Lll);
    device
        .write_polarity_inversion(PolarityInversion(0xDEAD))
        .unwrap();

    device.into_inner().done();
}

#[test]
fn test_read_configuration() {
    let expectations = [Transaction::write_read(0x20, vec![0x06], vec![0xFE, 0xCA])];
    let mock = I2cMock::new(&expectations);

    let mut device = Tca9535::new(mock, Address::Lll);
    let result = device.read_configuration().unwrap();
    assert_eq!(result.0, 0xCAFE);

    device.into_inner().done();
}

#[test]
fn test_write_configuration() {
    let expectations = [Transaction::write(0x20, vec![0x06, 0xEF, 0xBE])];
    let mock = I2cMock::new(&expectations);

    let mut device = Tca9535::new(mock, Address::Lll);
    device.write_configuration(Configuration(0xBEEF)).unwrap();

    device.into_inner().done();
}

#[test]
fn test_device_with_different_addresses() {
    // Test with address 0x21 (Llh)
    let expectations = [Transaction::write_read(0x21, vec![0x00], vec![0x00, 0x00])];
    let mock = I2cMock::new(&expectations);

    let mut device = Tca9535::new(mock, Address::Llh);
    let result = device.read_input().unwrap();
    assert_eq!(result.0, 0x0000);
    device.into_inner().done();

    // Test with address 0x27 (Hhh)
    let expectations = [Transaction::write_read(0x27, vec![0x00], vec![0xFF, 0xFF])];
    let mock = I2cMock::new(&expectations);

    let mut device = Tca9535::new(mock, Address::Hhh);
    let result = device.read_input().unwrap();
    assert_eq!(result.0, 0xFFFF);
    device.into_inner().done();
}

#[test]
fn test_complex_workflow() {
    let expectations = [
        // 1. Read current configuration
        Transaction::write_read(0x20, vec![0x06], vec![0xFF, 0xFF]),
        // 2. Configure some pins as outputs
        Transaction::write(0x20, vec![0x06, 0x00, 0xF0]),
        // 3. Set output values
        Transaction::write(0x20, vec![0x02, 0xAA, 0x00]),
        // 4. Read input values
        Transaction::write_read(0x20, vec![0x00], vec![0x55, 0xAA]),
    ];
    let mock = I2cMock::new(&expectations);

    let mut device = Tca9535::new(mock, Address::Lll);

    // Execute the workflow
    let config = device.read_configuration().unwrap();
    assert_eq!(config.0, 0xFFFF);

    device.write_configuration(Configuration(0xF000)).unwrap();
    device.write_output(Output(0x00AA)).unwrap();

    let input = device.read_input().unwrap();
    assert_eq!(input.0, 0xAA55);

    device.into_inner().done();
}

#[test]
fn test_all_pins_manipulation() {
    // Test that all pin indices work correctly with Output
    let mut output = Output(0x0000);
    output = output.with_high(PinIndex::P0);
    output = output.with_high(PinIndex::P1);
    output = output.with_high(PinIndex::P2);
    output = output.with_high(PinIndex::P3);
    output = output.with_high(PinIndex::P4);
    output = output.with_high(PinIndex::P5);
    output = output.with_high(PinIndex::P6);
    output = output.with_high(PinIndex::P7);
    output = output.with_high(PinIndex::P8);
    output = output.with_high(PinIndex::P9);
    output = output.with_high(PinIndex::P10);
    output = output.with_high(PinIndex::P11);
    output = output.with_high(PinIndex::P12);
    output = output.with_high(PinIndex::P13);
    output = output.with_high(PinIndex::P14);
    output = output.with_high(PinIndex::P15);
    assert_eq!(output.0, 0xFFFF);

    // Now set them all low
    output = output.with_low(PinIndex::P0);
    output = output.with_low(PinIndex::P1);
    output = output.with_low(PinIndex::P2);
    output = output.with_low(PinIndex::P3);
    output = output.with_low(PinIndex::P4);
    output = output.with_low(PinIndex::P5);
    output = output.with_low(PinIndex::P6);
    output = output.with_low(PinIndex::P7);
    output = output.with_low(PinIndex::P8);
    output = output.with_low(PinIndex::P9);
    output = output.with_low(PinIndex::P10);
    output = output.with_low(PinIndex::P11);
    output = output.with_low(PinIndex::P12);
    output = output.with_low(PinIndex::P13);
    output = output.with_low(PinIndex::P14);
    output = output.with_low(PinIndex::P15);
    assert_eq!(output.0, 0x0000);
}
