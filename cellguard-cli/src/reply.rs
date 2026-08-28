//! Interpretation of reply packets from the device.

use std::error::Error;

use cellboot::state::{PersistentState, STATE_LEN};
use cellcore::update::command::NackReason;
use cellguard_protocol::Kind;

use crate::transport::Reply;

/// Checks that a reply is a `BootAck` carrying `expected_offset`.
///
/// A missing or short offset field counts as offset zero. A `BootNack`
/// becomes an error with the decoded reason.
///
/// # Examples
///
/// ```
/// use cellguard_cli::reply::expect_ack;
/// use cellguard_cli::transport::Reply;
/// use cellguard_protocol::Kind;
///
/// let reply = Reply {
///     kind: Kind::BootAck,
///     payload: 128u32.to_le_bytes().to_vec(),
/// };
/// assert!(expect_ack(&reply, 128).is_ok());
/// ```
///
/// # Errors
///
/// Returns an error if the reply is a `BootNack`, has any other kind, or
/// acks a different offset.
pub fn expect_ack(reply: &Reply, expected_offset: u32) -> Result<(), Box<dyn Error>> {
    match reply.kind {
        Kind::BootAck => {
            let next_offset = reply
                .payload
                .get(..4)
                .and_then(|b| b.try_into().ok())
                .map_or(0, u32::from_le_bytes);
            if next_offset != expected_offset {
                return Err(format!(
                    "unexpected next_offset: expected {expected_offset}, got {next_offset}"
                )
                .into());
            }
            Ok(())
        }
        Kind::BootNack => Err(format!("device rejected: {}", nack_reason(&reply.payload)).into()),
        other => Err(format!("expected BootAck/BootNack, got {other:?}").into()),
    }
}

/// Decodes a Nack payload byte into a human-readable reason.
#[must_use]
pub const fn nack_reason(payload: &[u8]) -> &str {
    let Some(&code) = payload.first() else {
        return "no reason code";
    };
    let Some(reason) = NackReason::from_code(code) else {
        return "unknown reason code";
    };
    match reason {
        NackReason::Malformed => "malformed command",
        NackReason::WrongTarget => "wrong target",
        NackReason::BadState => "bad session state",
        NackReason::OutOfOrder => "chunk out of order",
        NackReason::TooLarge => "image too large",
        NackReason::StorageError => "storage error",
        NackReason::VerifyFailed => "verify failed",
        NackReason::Unauthorized => "unauthorized",
        NackReason::RouteTimeout => "route timeout",
        // The enum is non_exhaustive, so a wildcard arm is required.
        _ => "unknown reason code",
    }
}

/// Parses a `BootStatus` payload into a [`PersistentState`].
///
/// # Examples
///
/// ```
/// use cellboot::state::PersistentState;
/// use cellguard_cli::reply::parse_state;
///
/// let bytes = PersistentState::new(3).serialize();
/// let state = parse_state(&bytes).unwrap();
/// assert_eq!(state.agent_version, 3);
/// ```
///
/// # Errors
///
/// Returns an error if the payload is shorter than [`STATE_LEN`] or fails
/// the record checks.
pub fn parse_state(payload: &[u8]) -> Result<PersistentState, Box<dyn Error>> {
    let bytes: &[u8; STATE_LEN] = payload
        .get(..STATE_LEN)
        .and_then(|s| s.try_into().ok())
        .ok_or("status payload is not STATE_LEN bytes")?;
    PersistentState::parse(bytes).map_err(|e| format!("state parse failed: {e}").into())
}

#[cfg(test)]
mod tests {
    use cellboot::state::PersistentState;
    use cellcore::update::command::NackReason;
    use cellguard_protocol::Kind;

    use super::{expect_ack, parse_state};
    use crate::transport::Reply;

    fn ack(offset: u32) -> Reply {
        Reply {
            kind: Kind::BootAck,
            payload: offset.to_le_bytes().to_vec(),
        }
    }

    #[test]
    fn expect_ack_accepts_matching_offset() {
        assert!(expect_ack(&ack(0), 0).is_ok());
        assert!(expect_ack(&ack(4096), 4096).is_ok());
    }

    #[test]
    fn expect_ack_rejects_wrong_offset() {
        let err = expect_ack(&ack(8), 16).unwrap_err();
        assert!(err.to_string().contains("unexpected next_offset"));
    }

    #[test]
    fn expect_ack_treats_missing_offset_as_zero() {
        let reply = Reply {
            kind: Kind::BootAck,
            payload: Vec::new(),
        };
        assert!(expect_ack(&reply, 0).is_ok());
        assert!(expect_ack(&reply, 1).is_err());
    }

    #[test]
    fn expect_ack_reports_nack_reason() {
        let reply = Reply {
            kind: Kind::BootNack,
            payload: vec![NackReason::VerifyFailed.to_code()],
        };
        let err = expect_ack(&reply, 0).unwrap_err();
        assert!(err.to_string().contains("verify failed"));
    }

    #[test]
    fn expect_ack_rejects_other_kinds() {
        let reply = Reply {
            kind: Kind::Ack,
            payload: Vec::new(),
        };
        assert!(expect_ack(&reply, 0).is_err());
    }

    #[test]
    fn parse_state_round_trips() {
        let state = PersistentState::new(7);
        let parsed = parse_state(&state.serialize()).unwrap();
        assert_eq!(parsed, state);
    }

    #[test]
    fn parse_state_rejects_short_payload() {
        assert!(parse_state(&[0u8; 4]).is_err());
    }

    #[test]
    fn parse_state_rejects_bad_crc() {
        let mut bytes = PersistentState::new(7).serialize();
        bytes[24] ^= 0xFF;
        assert!(parse_state(&bytes).is_err());
    }
}
