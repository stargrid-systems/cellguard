//! Cross-reset resume record in `.noinit` SRAM.
//!
//! SRAM keeps its content through watchdog and software resets (not through
//! power-on), and `.noinit` keeps the startup code from zeroing it. The
//! record is armed before a test runs and cleared after its result line went
//! out. A record that survives into the next boot therefore carries a
//! verdict: the test panicked (the panic handler stored the location) or it
//! hung until the watchdog deadman fired.

use core::mem::MaybeUninit;

use hiltest_protocol::TestId;

const MAGIC: u32 = 0x4849_4C31;
/// A test was armed and no panic was recorded: a reset in this phase means
/// the deadman fired or the test reset the chip unexpectedly.
const PHASE_RUNNING: u8 = 1;
const PHASE_PANICKED: u8 = 2;
/// `test` value when no test was armed.
const TEST_NONE: u8 = 0xFF;
/// Bytes kept from the tail of the panic file path.
const FILE_CAP: usize = 24;

#[repr(C)]
#[derive(Clone, Copy)]
struct Record {
    magic: u32,
    test: u8,
    phase: u8,
    line: u32,
    file_len: u8,
    file: [u8; FILE_CAP],
}

const CLEARED: Record = Record {
    magic: 0,
    test: TEST_NONE,
    phase: 0,
    line: 0,
    file_len: 0,
    file: [0; FILE_CAP],
};

#[unsafe(link_section = ".noinit")]
static mut RECORD: MaybeUninit<Record> = MaybeUninit::uninit();

// The accesses are volatile byte copies: volatile keeps the compiler from
// eliding stores it cannot see a later read for (the reader is the next
// boot), and byte granularity keeps them volatile-compatible.

fn read() -> Record {
    let mut out = MaybeUninit::<Record>::uninit();
    let src = (&raw const RECORD).cast::<u8>();
    let dst = out.as_mut_ptr().cast::<u8>();
    for i in 0..size_of::<Record>() {
        // SAFETY: single core, no interrupts, and `i` stays within the
        // record on both sides.
        unsafe { dst.add(i).write(src.add(i).read_volatile()) }
    }
    // SAFETY: every byte was just written, and every bit pattern is a valid
    // `Record`. `take` validates the magic before trusting the content.
    unsafe { out.assume_init() }
}

fn write(record: Record) {
    let src = (&raw const record).cast::<u8>();
    let dst = (&raw mut RECORD).cast::<u8>();
    for i in 0..size_of::<Record>() {
        // SAFETY: single core, the static is only reached through these
        // helpers, and `i` stays within the record on both sides.
        unsafe { dst.add(i).write_volatile(src.add(i).read()) }
    }
}

/// Arms the record before a test runs.
pub fn arm(id: TestId) {
    write(Record {
        magic: MAGIC,
        test: id.code(),
        phase: PHASE_RUNNING,
        ..CLEARED
    });
}

/// Clears the record after a test reported its result in this boot.
pub fn disarm() {
    write(CLEARED);
}

/// Stores a panic location, keeping the armed test id when one is armed.
/// Called from the panic handler right before the software reset.
pub fn record_panic(file: &str, line: u32) {
    let current = read();
    let test = if current.magic == MAGIC {
        current.test
    } else {
        TEST_NONE
    };
    let mut record = Record {
        magic: MAGIC,
        test,
        phase: PHASE_PANICKED,
        line,
        ..CLEARED
    };
    // Keep the tail of the path: the file name is the informative part.
    let bytes = file.as_bytes();
    let tail = bytes
        .get(bytes.len().saturating_sub(FILE_CAP)..)
        .unwrap_or(&[]);
    for (slot, byte) in record.file.iter_mut().zip(tail) {
        *slot = *byte;
    }
    record.file_len = u8::try_from(tail.len()).unwrap_or(0);
    write(record);
}

/// A verdict carried over from the previous boot.
pub struct Deferred {
    /// The test that was armed, when its code is still known.
    pub test: Option<TestId>,
    /// Whether the panic handler ran (as opposed to a plain hang).
    pub panicked: bool,
    /// Panic line number.
    pub line: u32,
    file: [u8; FILE_CAP],
    file_len: u8,
}

impl Deferred {
    /// The stored tail of the panic file path.
    pub fn file(&self) -> &str {
        let len = usize::from(self.file_len).min(FILE_CAP);
        core::str::from_utf8(self.file.get(..len).unwrap_or(&[])).unwrap_or("?")
    }
}

/// Takes and clears the record the previous boot left behind.
pub fn take() -> Option<Deferred> {
    let record = read();
    disarm();
    if record.magic != MAGIC {
        return None;
    }
    Some(Deferred {
        test: TestId::from_code(record.test),
        panicked: record.phase == PHASE_PANICKED,
        line: record.line,
        file: record.file,
        file_len: record.file_len,
    })
}
