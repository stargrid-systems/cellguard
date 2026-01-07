//! Integration tests for the P3T1755 temperature sensor driver.

use embedded_hal::i2c::{ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation};
use p3t1755::alert::AlertCondition;
use p3t1755::{Address, Config, ConversionTime, FaultQueue, P3t1755, Temperature, alert};

/// A simple mock I2C bus for testing.
///
/// This mock records all transactions and can be pre-programmed with expected
/// transactions and responses.
struct MockI2c {
    /// Expected transactions with their responses.
    expectations: Vec<Transaction>,
    /// Current index in the expectations vector.
    current: usize,
}

#[derive(Debug, Clone)]
struct Transaction {
    /// Expected I2C address.
    addr: u8,
    /// Expected operations.
    operations: Vec<MockOperation>,
}

#[derive(Debug, Clone)]
enum MockOperation {
    Write(Vec<u8>),
    Read(Vec<u8>),
    ReadNackAddress,
}

#[derive(Debug, Clone, Copy)]
struct MockError(ErrorKind);

impl MockError {
    fn new(kind: ErrorKind) -> Self {
        Self(kind)
    }
}

impl embedded_hal::i2c::Error for MockError {
    fn kind(&self) -> ErrorKind {
        self.0
    }
}

impl ErrorType for MockI2c {
    type Error = MockError;
}

impl I2c for MockI2c {
    fn read(&mut self, addr: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        if self.current >= self.expectations.len() {
            panic!("Unexpected I2C read to address 0x{:02X}", addr);
        }

        let expected = &self.expectations[self.current];
        self.current += 1;

        if addr != expected.addr {
            panic!(
                "I2C address mismatch: expected 0x{:02X}, got 0x{:02X}",
                expected.addr, addr
            );
        }

        if expected.operations.len() != 1 {
            panic!(
                "Operation count mismatch for read: expected 1, got {}",
                expected.operations.len()
            );
        }

        match expected.operations[0].clone() {
            MockOperation::Read(expected_data) => {
                if buf.len() != expected_data.len() {
                    panic!(
                        "Read buffer size mismatch: expected {}, got {}",
                        expected_data.len(),
                        buf.len()
                    );
                }
                buf.copy_from_slice(&expected_data);
                Ok(())
            }
            MockOperation::ReadNackAddress => Err(MockError::new(ErrorKind::NoAcknowledge(
                NoAcknowledgeSource::Address,
            ))),
            other => panic!("Unexpected operation for read: {:?}", other),
        }
    }

    fn write(&mut self, addr: u8, _bytes: &[u8]) -> Result<(), Self::Error> {
        panic!(
            "Unexpected write without expectations to addr 0x{:02X}",
            addr
        );
    }

    fn write_read(&mut self, addr: u8, _bytes: &[u8], _buf: &mut [u8]) -> Result<(), Self::Error> {
        panic!(
            "Unexpected write_read without expectations to addr 0x{:02X}",
            addr
        );
    }

    fn transaction(
        &mut self,
        addr: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        if self.current >= self.expectations.len() {
            panic!("Unexpected I2C transaction to address 0x{:02X}", addr);
        }

        let expected = &self.expectations[self.current];
        self.current += 1;

        if addr != expected.addr {
            panic!(
                "I2C address mismatch: expected 0x{:02X}, got 0x{:02X}",
                expected.addr, addr
            );
        }

        if operations.len() != expected.operations.len() {
            panic!(
                "Operation count mismatch: expected {}, got {}",
                expected.operations.len(),
                operations.len()
            );
        }

        for (i, (op, expected_op)) in operations
            .iter_mut()
            .zip(expected.operations.iter())
            .enumerate()
        {
            match (op, expected_op) {
                (Operation::Write(data), MockOperation::Write(expected_data)) => {
                    if *data != expected_data.as_slice() {
                        panic!(
                            "Write data mismatch at operation {}: expected {:?}, got {:?}",
                            i, expected_data, data
                        );
                    }
                }
                (Operation::Read(buf), MockOperation::Read(expected_data)) => {
                    if buf.len() != expected_data.len() {
                        panic!(
                            "Read buffer size mismatch at operation {}: expected {}, got {}",
                            i,
                            expected_data.len(),
                            buf.len()
                        );
                    }
                    buf.copy_from_slice(expected_data);
                }
                (_, MockOperation::ReadNackAddress) => {
                    return Err(MockError::new(ErrorKind::NoAcknowledge(
                        NoAcknowledgeSource::Address,
                    )));
                }
                _ => panic!("Operation type mismatch at operation {}", i),
            }
        }

        Ok(())
    }
}

impl MockI2c {
    fn new(expectations: Vec<Transaction>) -> Self {
        Self {
            expectations,
            current: 0,
        }
    }

    fn done(&self) {
        if self.current != self.expectations.len() {
            panic!(
                "Not all expected transactions were executed: {}/{}",
                self.current,
                self.expectations.len()
            );
        }
    }
}

#[test]
fn test_read_temperature() {
    // Temperature: 25.0625°C = 0x0191 (401 in 1/16°C units)
    // Register format: 12-bit value left-shifted by 4 bits
    // 0x0191 << 4 = 0x1910
    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x00]),      // Temperature register
            MockOperation::Read(vec![0x19, 0x10]), // 25.0625°C
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    let temp = sensor.read_temperature().unwrap();

    assert_eq!(temp.raw(), 401);
    assert_eq!(temp.degrees_celsius(), 25);
    assert_eq!(temp.centi_degrees_celsius(), 2506);

    sensor.into_inner().done();
}

#[test]
fn test_read_negative_temperature() {
    // Temperature: -10.5°C = -168 in 1/16°C units = 0xFF58
    // Register format: 12-bit value left-shifted by 4 bits
    // -168 << 4 = -2688 = 0xF580 in two's complement
    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x00]),      // Temperature register
            MockOperation::Read(vec![0xF5, 0x80]), // -10.5°C
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    let temp = sensor.read_temperature().unwrap();

    assert_eq!(temp.raw(), -168);
    assert_eq!(temp.degrees_celsius(), -11); // Truncated
    assert_eq!(temp.centi_degrees_celsius(), -1050);

    sensor.into_inner().done();
}

#[test]
fn test_read_config() {
    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x01]), // Config register
            MockOperation::Read(vec![0x28]),  // Default config
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    let config = sensor.read_config().unwrap();

    assert!(!config.shutdown_mode());
    assert!(!config.thermostat_mode());
    assert!(!config.polarity());
    assert!(!config.one_shot());

    sensor.into_inner().done();
}

#[test]
fn test_write_config() {
    let config = Config::RESET
        .with_shutdown_mode(true)
        .with_polarity(true)
        .with_fault_queue(FaultQueue::Four)
        .with_conversion_time(ConversionTime::Ms110);

    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x01]), // Config register
            MockOperation::Write(vec![0x55]), // Config value: 0b01010101
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    sensor.write_config(config).unwrap();

    sensor.into_inner().done();
}

#[test]
fn test_read_t_low() {
    // T_LOW: 75°C = 1200 in 1/16°C units
    // Register format: 1200 << 4 = 19200 = 0x4B00
    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x02]),      // T_LOW register
            MockOperation::Read(vec![0x4B, 0x00]), // 75°C
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    let temp = sensor.read_t_low().unwrap();

    assert_eq!(temp.degrees_celsius(), 75);

    sensor.into_inner().done();
}

#[test]
fn test_write_t_low() {
    let temp = Temperature::from_degrees_celsius(50);

    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x02]),       // T_LOW register
            MockOperation::Write(vec![0x32, 0x00]), // 50°C
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    sensor.write_t_low(temp).unwrap();

    sensor.into_inner().done();
}

#[test]
fn test_read_t_high() {
    // T_HIGH: 80°C = 1280 in 1/16°C units
    // Register format: 1280 << 4 = 20480 = 0x5000
    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x03]),      // T_HIGH register
            MockOperation::Read(vec![0x50, 0x00]), // 80°C
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    let temp = sensor.read_t_high().unwrap();

    assert_eq!(temp.degrees_celsius(), 80);

    sensor.into_inner().done();
}

#[test]
fn test_write_t_high() {
    let temp = Temperature::from_degrees_celsius(85);

    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x03]),       // T_HIGH register
            MockOperation::Write(vec![0x55, 0x00]), // 85°C
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    sensor.write_t_high(temp).unwrap();

    sensor.into_inner().done();
}

#[test]
fn test_register_pointer_latching() {
    // Test that the driver optimizes by not re-writing the register pointer
    // when reading the same register twice in a row
    let mock = MockI2c::new(vec![
        Transaction {
            addr: 0x48,
            operations: vec![
                MockOperation::Write(vec![0x00]), // Temperature register (first time)
                MockOperation::Read(vec![0x19, 0x10]),
            ],
        },
        Transaction {
            addr: 0x48,
            operations: vec![
                // No write operation - pointer is latched
                MockOperation::Read(vec![0x19, 0x20]),
            ],
        },
    ]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);

    // First read - sets the pointer
    let _temp1 = sensor.read_temperature().unwrap();

    // Second read - pointer should be latched, no write operation
    let _temp2 = sensor.read_temperature().unwrap();

    sensor.into_inner().done();
}

#[test]
fn test_different_addresses() {
    let addresses = [
        (Address::Addr1, 0x40),
        (Address::Addr9, 0x48),
        (Address::Addr16, 0x4F),
        (Address::Addr32, 0x5F),
    ];

    for (addr_enum, addr_val) in addresses.iter() {
        let mock = MockI2c::new(vec![Transaction {
            addr: *addr_val,
            operations: vec![
                MockOperation::Write(vec![0x00]),
                MockOperation::Read(vec![0x19, 0x10]),
            ],
        }]);

        let mut sensor = P3t1755::new(mock, *addr_enum);
        let _temp = sensor.read_temperature().unwrap();
        sensor.into_inner().done();
    }
}

#[test]
fn test_config_all_features() {
    // Test all configuration options
    let config = Config::RESET
        .with_shutdown_mode(true)
        .with_thermostat_mode(true)
        .with_polarity(true)
        .with_fault_queue(FaultQueue::Six)
        .with_conversion_time(ConversionTime::Ms220)
        .with_one_shot(true);

    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x01]),
            MockOperation::Write(vec![0xFF]), // All bits set: 0b11111111
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    sensor.write_config(config).unwrap();

    sensor.into_inner().done();
}

#[test]
fn test_full_temperature_range() {
    // Test minimum temperature
    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x00]),
            MockOperation::Read(vec![0x80, 0x00]), // -128°C
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    let temp = sensor.read_temperature().unwrap();
    assert_eq!(temp.degrees_celsius(), -128);
    sensor.into_inner().done();

    // Test maximum temperature
    let mock = MockI2c::new(vec![Transaction {
        addr: 0x48,
        operations: vec![
            MockOperation::Write(vec![0x00]),
            MockOperation::Read(vec![0x7F, 0xF0]), // 127.9375°C
        ],
    }]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);
    let temp = sensor.read_temperature().unwrap();
    assert_eq!(temp.degrees_celsius(), 127);
    assert_eq!(temp.centi_degrees_celsius(), 12793);
    sensor.into_inner().done();
}

#[test]
fn test_sequential_different_registers() {
    // Test that switching between different registers works correctly
    let mock = MockI2c::new(vec![
        Transaction {
            addr: 0x48,
            operations: vec![
                MockOperation::Write(vec![0x00]), // Temperature register
                MockOperation::Read(vec![0x19, 0x10]),
            ],
        },
        Transaction {
            addr: 0x48,
            operations: vec![
                MockOperation::Write(vec![0x01]), // Config register (different register)
                MockOperation::Read(vec![0x28]),
            ],
        },
        Transaction {
            addr: 0x48,
            operations: vec![
                MockOperation::Write(vec![0x02]), // T_LOW register
                MockOperation::Read(vec![0x4B, 0x00]),
            ],
        },
    ]);

    let mut sensor = P3t1755::new(mock, Address::Addr9);

    let _temp = sensor.read_temperature().unwrap();
    let _config = sensor.read_config().unwrap();
    let _t_low = sensor.read_t_low().unwrap();

    sensor.into_inner().done();
}

#[test]
fn test_alert_over_temperature() {
    let addr = Address::Addr9;
    let alert_byte = (addr.get() << 1) | 0x01; // LSB = 1 => over-temperature

    let mut bus = MockI2c::new(vec![Transaction {
        addr: 0x0C,
        operations: vec![MockOperation::Read(vec![alert_byte])],
    }]);

    let alert = alert::process(&mut bus)
        .expect("i2c ok")
        .expect("alert present");

    assert_eq!(alert.address().get(), addr.get());
    assert!(matches!(alert.condition(), AlertCondition::OverTemperature));

    bus.done();
}

#[test]
fn test_alert_under_temperature() {
    let addr = Address::Addr1;
    let alert_byte = addr.get() << 1; // LSB = 0 => under-temperature

    let mut bus = MockI2c::new(vec![Transaction {
        addr: 0x0C,
        operations: vec![MockOperation::Read(vec![alert_byte])],
    }]);

    let alert = alert::process(&mut bus)
        .expect("i2c ok")
        .expect("alert present");

    assert_eq!(alert.address().get(), addr.get());
    assert!(matches!(
        alert.condition(),
        AlertCondition::UnderTemperature
    ));

    bus.done();
}

#[test]
fn test_alert_no_pending() {
    let mut bus = MockI2c::new(vec![Transaction {
        addr: 0x0C,
        operations: vec![MockOperation::ReadNackAddress],
    }]);

    let alert = alert::process(&mut bus).expect("i2c ok");
    assert!(alert.is_none());

    bus.done();
}

#[test]
fn test_alert_invalid_address() {
    // Address bits decode to 0x00 which is outside valid range => should return
    // None
    let mut bus = MockI2c::new(vec![Transaction {
        addr: 0x0C,
        operations: vec![MockOperation::Read(vec![0x00])],
    }]);

    let alert = alert::process(&mut bus).expect("i2c ok");
    assert!(alert.is_none());

    bus.done();
}
