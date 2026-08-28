//! Format/parse round-trips for both line directions.

use hiltest_protocol::{
    AckList, Command, CommandError, Event, Outcome, SCAN_FIRST, SCAN_LAST, TestId,
};

#[test]
fn test_ids_round_trip_names_and_codes() {
    for id in TestId::ALL {
        assert_eq!(TestId::from_name(id.name()), Some(id));
        assert_eq!(TestId::from_code(id.code()), Some(id));
    }
    assert_eq!(TestId::from_name("uart-echo-rc"), Some(TestId::UartEchoRc));
    assert_eq!(TestId::from_name("nope"), None);
    assert_eq!(TestId::from_code(0xFF), None);
}

#[test]
fn outcomes_round_trip() {
    for outcome in [Outcome::Pass, Outcome::Fail, Outcome::Skip] {
        assert_eq!(Outcome::from_name(outcome.as_str()), Some(outcome));
    }
    assert_eq!(Outcome::from_name("pass"), None);
}

#[test]
fn commands_parse() {
    assert_eq!(Command::parse("PING 7"), Ok(Command::Ping(7)));
    assert_eq!(Command::parse("LIST"), Ok(Command::List));
    assert_eq!(
        Command::parse("RUN uart-echo-rc"),
        Ok(Command::Run("uart-echo-rc"))
    );
    assert_eq!(Command::parse("REBOOT"), Ok(Command::Reboot));
    // Trailing CR from a CRLF host is tolerated.
    assert_eq!(Command::parse("PING 7\r"), Ok(Command::Ping(7)));
}

#[test]
fn bad_commands_are_rejected() {
    assert_eq!(Command::parse(""), Err(CommandError::Unknown));
    assert_eq!(Command::parse("FROB"), Err(CommandError::Unknown));
    assert_eq!(Command::parse("PING"), Err(CommandError::BadArgument));
    assert_eq!(Command::parse("PING x"), Err(CommandError::BadArgument));
    assert_eq!(Command::parse("PING 1 2"), Err(CommandError::BadArgument));
    assert_eq!(Command::parse("RUN"), Err(CommandError::BadArgument));
    assert_eq!(Command::parse("LIST now"), Err(CommandError::BadArgument));
}

#[test]
fn commands_round_trip_through_display() {
    let commands = [
        Command::Ping(42),
        Command::List,
        Command::Run("clock-extclk"),
        Command::Reboot,
    ];
    for command in commands {
        let line = command.to_string();
        assert_eq!(Command::parse(&line), Ok(command));
    }
}

#[test]
fn events_parse() {
    assert_eq!(
        Event::parse("|HIL v1 boot rstfr=0x10 clk=rc4m"),
        Some(Event::Boot {
            rstfr: 0x10,
            clk: "rc4m"
        })
    );
    assert_eq!(Event::parse("|HIL ready"), Some(Event::Ready));
    assert_eq!(Event::parse("|HIL pong 7"), Some(Event::Pong(7)));
    assert_eq!(
        Event::parse("|HIL run uart-echo-rc"),
        Some(Event::RunAck { id: "uart-echo-rc" })
    );
    assert_eq!(
        Event::parse("|HIL test clock-extclk"),
        Some(Event::Test { id: "clock-extclk" })
    );
    assert_eq!(
        Event::parse("|HIL log uart-echo-rc send"),
        Some(Event::Log {
            body: "uart-echo-rc send"
        })
    );
    assert_eq!(
        Event::parse("|HIL echo one two three"),
        Some(Event::Echo {
            payload: "one two three"
        })
    );
    assert_eq!(
        Event::parse("|HIL result clock-extclk FAIL exts-clear"),
        Some(Event::Result {
            id: "clock-extclk",
            outcome: Outcome::Fail,
            detail: Some("exts-clear"),
        })
    );
    assert_eq!(
        Event::parse("|HIL result uart-echo-rc PASS"),
        Some(Event::Result {
            id: "uart-echo-rc",
            outcome: Outcome::Pass,
            detail: None,
        })
    );
    assert_eq!(
        Event::parse("|HIL err unknown-test"),
        Some(Event::Err {
            reason: "unknown-test"
        })
    );
}

#[test]
fn noise_lines_are_dropped() {
    assert_eq!(Event::parse(""), None);
    assert_eq!(Event::parse("hello world"), None);
    assert_eq!(Event::parse("|HIL"), None);
    assert_eq!(Event::parse("|HIL frobnicate"), None);
    assert_eq!(Event::parse("|HIL v1 boot rstfr=16 clk=rc4m"), None);
    assert_eq!(Event::parse("|HIL result x NOPE"), None);
    assert_eq!(Event::parse("\u{fffd}\u{fffd}|HIL ready"), None);
}

#[test]
fn ack_lists_parse() {
    let mut acks = AckList::new();
    assert!(acks.push(0x20));
    assert!(acks.push(0x21));
    assert!(acks.push(0x42));
    assert_eq!(AckList::parse("acks=20,21,42"), Some(acks));
    // Lowercase hex is tolerated.
    let mut single = AckList::new();
    assert!(single.push(0x4A));
    assert_eq!(AckList::parse("acks=4a"), Some(single));
    assert_eq!(AckList::parse("acks="), Some(AckList::new()));
}

#[test]
fn bad_ack_lists_are_rejected() {
    assert_eq!(AckList::parse("20,21"), None);
    assert_eq!(AckList::parse("acks=zz"), None);
    assert_eq!(AckList::parse("acks=123"), None);
    assert_eq!(AckList::parse("acks=20,"), None);
    assert_eq!(AckList::parse("acks=,20"), None);
    assert_eq!(AckList::parse("acks=+2"), None);
}

#[test]
fn ack_list_capacity_covers_the_scan_range() {
    let mut acks = AckList::new();
    for addr in SCAN_FIRST..=SCAN_LAST {
        assert!(acks.push(addr));
    }
    assert_eq!(acks.as_slice().len(), 112);
    assert!(!acks.push(0x7F));
}

#[test]
fn twi_scan_result_detail_stays_one_token() {
    let event = Event::parse("|HIL result twi-scan FAIL acks=20,21").unwrap();
    let Event::Result {
        detail: Some(detail),
        ..
    } = event
    else {
        panic!("not a result line: {event:?}");
    };
    let mut expected = AckList::new();
    assert!(expected.push(0x20));
    assert!(expected.push(0x21));
    assert_eq!(AckList::parse(detail), Some(expected));
}

#[cfg(feature = "ufmt")]
mod ufmt_round_trip {
    use super::*;

    /// `uWrite` sink over a `String` for the round-trip tests.
    struct Sink(String);

    impl ufmt::uWrite for Sink {
        type Error = core::convert::Infallible;

        fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
            self.0.push_str(s);
            Ok(())
        }
    }

    fn format(event: &Event<'_>) -> String {
        let mut sink = Sink(String::new());
        ufmt::uwrite!(sink, "{}", *event).unwrap();
        sink.0
    }

    #[test]
    fn events_round_trip_through_udisplay() {
        let events = [
            Event::Boot {
                rstfr: 0x2A,
                clk: "rc4m",
            },
            Event::Ready,
            Event::Pong(1234),
            Event::RunAck { id: "uart-echo-rc" },
            Event::Test {
                id: "spi0-cat25-probe-app",
            },
            Event::Log {
                body: "clock-extclk switching",
            },
            Event::Echo {
                payload: "payload-1234",
            },
            Event::Result {
                id: "uart-echo-24m",
                outcome: Outcome::Skip,
                detail: Some("needs-clock-extclk"),
            },
            Event::Result {
                id: "uart-echo-rc",
                outcome: Outcome::Pass,
                detail: None,
            },
            Event::Err { reason: "bad-arg" },
        ];
        for event in events {
            let line = format(&event);
            assert_eq!(Event::parse(&line), Some(event), "line: {line}");
        }
    }

    #[test]
    fn boot_banner_formats_hex_flags() {
        let line = format(&Event::Boot {
            rstfr: 0x08,
            clk: "rc4m",
        });
        assert_eq!(line, "|HIL v1 boot rstfr=0x08 clk=rc4m");
    }
    #[test]
    fn ack_lists_round_trip_through_udisplay() {
        let mut acks = AckList::new();
        for addr in [0x20, 0x21, 0x42, 0x4A] {
            assert!(acks.push(addr));
        }
        let mut sink = Sink(String::new());
        ufmt::uwrite!(sink, "{}", acks).unwrap();
        assert_eq!(sink.0, "acks=20,21,42,4A");
        assert_eq!(AckList::parse(&sink.0), Some(acks));

        let mut sink = Sink(String::new());
        ufmt::uwrite!(sink, "{}", AckList::new()).unwrap();
        assert_eq!(sink.0, "acks=");
        assert_eq!(AckList::parse(&sink.0), Some(AckList::new()));
    }
}
