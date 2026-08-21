//! The central message-kind registry.
//!
//! Every message on the bus, request or response, has a [`Kind`]. Keeping them
//! in one enum makes the wire unambiguous and lets a trace tool decode any
//! packet. Discriminants are assigned as needed and must never be reused.
//! Bootloader kinds are compiled in only with the `bootloader` feature, but
//! keep fixed discriminants so a trace of a bootloader exchange reads the same
//! everywhere.

use zerocopy::{Immutable, IntoBytes, KnownLayout, TryFromBytes, Unaligned};

/// A message kind. The `u8` discriminant is what travels on the wire.
///
/// Not every byte is a valid kind, so this derives [`TryFromBytes`] rather than
/// `FromBytes`: an unknown discriminant is rejected instead of accepted.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, TryFromBytes, IntoBytes, Immutable, KnownLayout, Unaligned,
)]
#[non_exhaustive]
#[repr(u8)]
pub enum Kind {
    /// Request: read the `ATtiny` 3-byte device id.
    ReadDeviceId = 1,
    /// Request: read the 10-byte serial number.
    ReadSerialNumber = 2,
    /// Request: read the temperature.
    ReadTemperature = 3,
    /// Response: device id.
    DeviceId = 4,
    /// Response: serial number.
    SerialNumber = 5,
    /// Response: temperature.
    Temperature = 6,
    /// Response: the request succeeded.
    Ack = 7,
    /// Response: the request was rejected.
    Nack = 8,

    /// Bootloader request: report update state.
    #[cfg(feature = "bootloader")]
    BootProbe = 9,
    /// Bootloader request: begin an update (payload is the image header).
    #[cfg(feature = "bootloader")]
    BootBegin = 10,
    /// Bootloader request: deliver a payload chunk.
    #[cfg(feature = "bootloader")]
    BootData = 11,
    /// Bootloader request: verify and stage the image.
    #[cfg(feature = "bootloader")]
    BootCommit = 12,
    /// Bootloader request: abandon the current update.
    #[cfg(feature = "bootloader")]
    BootAbort = 13,
    /// Bootloader response: the update state, in reply to `BootProbe`.
    #[cfg(feature = "bootloader")]
    BootStatus = 14,
    /// Bootloader response: a command succeeded.
    #[cfg(feature = "bootloader")]
    BootAck = 15,
    /// Bootloader response: a command was rejected.
    #[cfg(feature = "bootloader")]
    BootNack = 16,
    /// Bootloader request: replace the shared authentication key. The payload
    /// is the new key followed by an authentication tag over it.
    /// Development use only, and inert once the key store is locked.
    #[cfg(feature = "bootloader")]
    BootReplaceKey = 17,
    /// Programmer request (main MCU to `cellprog`): program a staged image into
    /// a target. The payload selects which staged image.
    #[cfg(feature = "bootloader")]
    ProgProgram = 18,
    /// Programmer result (`cellprog` to main MCU): the outcome of a
    /// [`Kind::ProgProgram`] request.
    #[cfg(feature = "bootloader")]
    ProgResult = 19,
    /// Request: set the cellagent balancer gate state.
    /// Payload: 1 byte bitmask (bit 0 = `GATE_A`, bit 1 = `GATE_B`, bit 2 =
    /// `ALL_OFF`).
    SetBalancer = 20,
    /// Request: report the last panic record. Available on every node.
    PanicProbe = 21,
    /// Response: the panic record, in reply to [`Kind::PanicProbe`]. An empty
    /// payload means no panic is recorded for this session.
    PanicStatus = 22,
    /// Programmer session request (main MCU to `cellprog`): chip-erase the
    /// target and enter programming mode. Payload: 1 target byte. See the
    /// `session` module.
    #[cfg(feature = "bootloader")]
    ProgSessionBegin = 23,
    /// Programmer session request: program up to `PAGE_MAX` bytes at a flash
    /// address. Payload: 2 address bytes then data.
    #[cfg(feature = "bootloader")]
    ProgPageWrite = 24,
    /// Programmer session request: read back flash. Payload: 2 address bytes
    /// and 1 length byte.
    #[cfg(feature = "bootloader")]
    ProgPageRead = 25,
    /// Programmer session request: leave programming mode and reset the
    /// target. Empty payload.
    #[cfg(feature = "bootloader")]
    ProgSessionEnd = 26,
    /// Programmer session response: the outcome of a command. Payload: 1
    /// status byte, plus the addressed command's 2 address bytes when
    /// replying to a page command.
    #[cfg(feature = "bootloader")]
    ProgSessionStatus = 27,
    /// Programmer session response: read-back data. Payload: 1 status byte, 2
    /// address bytes, then the data.
    #[cfg(feature = "bootloader")]
    ProgPageData = 28,
    /// Balancing-test request: read the latest cell-voltage snapshot.
    /// Response is [`Kind::CellVoltages`]. Payload: none. The response is
    /// raw 24-bit ADC codes; see the telemetry module for conversion.
    ReadCellVoltages = 29,
    /// Response: `seq` byte, then 4 little-endian `i32` raw ADC codes
    /// (cells 1-4, ADC A ch0-3). `seq` increments per snapshot.
    CellVoltages = 30,
    /// Balancing-test request: read the latest balance-current snapshot.
    /// Response is [`Kind::BalanceCurrents`].
    ReadBalanceCurrents = 31,
    /// Response: `seq` byte, then 4 little-endian `i32` raw ADC codes
    /// (per-cell INA190 outputs, ADC B ch0-3 at IR-mux position 0).
    BalanceCurrents = 32,
    /// Balancing-test request: read the supply rails. Response is
    /// [`Kind::Rails`].
    ReadRails = 33,
    /// Response: 8 little-endian `u16` MCU-ADC codes, fixed order:
    /// `VBAT_A`, `VBAT_B`, `5V0`, `3V3`, `3V3B`, `1V8AN`, `12V_CON`,
    /// `20V_MOS`.
    Rails = 34,
    /// Balancing-test request: read all temperature sensors. Response is
    /// [`Kind::Temperatures`].
    ReadTemperatures = 35,
    /// Response: little-endian `i16` centi-degrees Celsius per sensor,
    /// fixed order: main `P3T1755` (U908), ADC board `LM61`, cellagent
    /// `LM61`.
    /// A sensor that could not be read reports `i16::MIN`.
    Temperatures = 36,
    /// Balancing-test request: read the full balancing status. Response is
    /// [`Kind::BalancerStatus`].
    ReadBalancerStatus = 37,
    /// Response: the balancing status frame. See the telemetry module for
    /// the payload layout.
    BalancerStatus = 38,
    /// Balancing-test request: set the bleed-leg enable masks on the PWM
    /// expander. Payload: 2 bytes, `en_3r6` then `en_36r5`, bit x = cell
    /// x+1. Response is [`Kind::Ack`].
    SetBleed = 39,
    /// Balancing-test request: set the bleed PWM duty. Payload: 2
    /// little-endian bytes, duty in 1/65536 units. 0 disables modulation
    /// (statically on when legs enabled). Response is [`Kind::Ack`].
    SetBleedPwm = 40,
    /// Balancing-test request: set power-enable flags. Payload: 1 byte:
    /// bit 0 = `ACTIVE_BALANCER_ON`, bit 1 = `EN_ALL`. Response is
    /// [`Kind::Ack`].
    SetPower = 41,
    /// Balancing-test request: assert the hardware gate-off (PB5). Payload:
    /// 1 byte, 1 = assert (gates off), 0 = release. Response is
    /// [`Kind::Ack`].
    GateOff = 42,
    /// Balancing-test request (cellagent): report the last commanded gate
    /// mask. Response is [`Kind::BalancerGateState`]. Routed through the
    /// cellcore like [`Kind::SetBalancer`].
    ReadBalancerGateState = 43,
    /// Response: 1 byte, the last commanded gate mask.
    BalancerGateState = 44,
}

impl Kind {
    /// Returns the wire byte for this kind.
    #[must_use]
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire byte into a kind, or `None` if it is not known.
    ///
    /// Validity is derived from the enum discriminants by [`TryFromBytes`], so
    /// this cannot drift out of sync with the variants.
    #[must_use]
    pub fn from_u8(value: u8) -> Option<Self> {
        Self::try_read_from_bytes(&[value]).ok()
    }
}

#[cfg(test)]
mod tests {
    use super::Kind;

    #[test]
    fn roundtrips_known_kinds() {
        for byte in 0..=u8::MAX {
            if let Some(kind) = Kind::from_u8(byte) {
                assert_eq!(kind.to_u8(), byte);
            }
        }
    }

    #[test]
    fn rejects_zero() {
        assert_eq!(Kind::from_u8(0), None);
    }
}
