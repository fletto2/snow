//! Lightweight global performance instrumentation for the emulated Mac.
//!
//! Gated by the `SNOW_PERF` env var (any non-empty, non-`"0"` value enables it).
//! When enabled it counts VIA/SCC register reads + writes and CPU interrupts,
//! measures how long the CPU spends at elevated IPL (i.e. inside an ISR), and
//! emits a once-per-emulated-second line via the `log` crate (`info!`, so
//! `RUST_LOG=snow_core::perf=info` surfaces it).
//!
//! Added for the macrom instability study: the custom Mac Plus monitor ROM
//! ("macmon") is suspected of destabilising marginal boards by doing far more
//! per-tick VIA/bus work and staying masked (IPL 7) far longer than the stock
//! Apple ROM. This measures both the bus-access RATE and the time-at-elevated-IPL
//! so the two ROMs can be compared quantitatively on the same emulated machine.
//!
//! Calibrated for the 7.8336 MHz Mac Plus (the microsecond conversions and the
//! one-second report boundary assume that clock; on a 16 MHz Mac II the report
//! simply fires every ~2 s and the us figures are scaled accordingly).

use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
use std::sync::OnceLock;

use log::info;

use crate::tickable::Ticks;

/// Mac Plus CPU clock -> cycles in one emulated second (7.8336 MHz).
const ONESEC_CYCLES: Ticks = 7_833_600;
/// CPU cycles per microsecond at 7.8336 MHz.
const CYCLES_PER_US: f64 = 7.8336;
/// `ISR_SINCE` sentinel: the CPU is not currently at elevated IPL.
const NOT_IN_ISR: u64 = u64::MAX;

static VIA_READS: AtomicU64 = AtomicU64::new(0);
static VIA_WRITES: AtomicU64 = AtomicU64::new(0);
static SCC_READS: AtomicU64 = AtomicU64::new(0);
static SCC_WRITES: AtomicU64 = AtomicU64::new(0);

/// Interrupts taken, indexed by level (1..=7; index 0 unused).
static IRQ_COUNT: [AtomicU64; 8] = [
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
];

/// ISR ("elevated-IPL region") duration accounting, in CPU cycles.
static ISR_COUNT: AtomicU64 = AtomicU64::new(0);
static ISR_CYCLES_SUM: AtomicU64 = AtomicU64::new(0);
static ISR_CYCLES_MAX: AtomicU64 = AtomicU64::new(0);
static ISR_CYCLES_MIN: AtomicU64 = AtomicU64::new(u64::MAX);

/// Cycle timestamp when the current elevated-IPL region began (or `NOT_IN_ISR`).
static ISR_SINCE: AtomicU64 = AtomicU64::new(NOT_IN_ISR);
/// VIA level-1 IRQ triggers, counted per IFR source bit (0..6) on the rising
/// edge of (IFR & IER).  Bit 1 = CA1 (the 60 Hz VBL tick); everything else is
/// "excess" -- SR (bit 2, M0110 handshake), T2 (bit 5, kbd handover), etc.
static VIA_SRC: [AtomicU64; 8] = [
    AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0),
    AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0), AtomicU64::new(0),
];
static VIA_SRC_PREV: AtomicU64 = AtomicU64::new(0);
/// Cycle timestamp of the last emitted report.
static LAST_REPORT: AtomicU64 = AtomicU64::new(0);

/// True if `SNOW_PERF` instrumentation is enabled. Cached after the first call,
/// so this is a cheap atomic load on the hot path.
#[inline]
pub fn enabled() -> bool {
    static ENABLED: OnceLock<bool> = OnceLock::new();
    *ENABLED.get_or_init(|| {
        std::env::var("SNOW_PERF")
            .map(|v| v != "0" && !v.is_empty())
            .unwrap_or(false)
    })
}

#[inline]
pub fn via_read() {
    VIA_READS.fetch_add(1, Relaxed);
}
#[inline]
pub fn via_write() {
    VIA_WRITES.fetch_add(1, Relaxed);
}
#[inline]
pub fn scc_read() {
    SCC_READS.fetch_add(1, Relaxed);
}
#[inline]
pub fn scc_write() {
    SCC_WRITES.fetch_add(1, Relaxed);
}

/// Record that an interrupt of `level` (1..=7) was taken.
#[inline]
pub fn irq(level: u8) {
    if let Some(c) = IRQ_COUNT.get(level as usize) {
        c.fetch_add(1, Relaxed);
    }
}

/// Record the VIA level-1 IRQ sources from the current (IFR & IER) byte (bit 7
/// masked off).  Counts each source once on its rising edge, so calling this
/// every CPU step -- most of which see no change -- yields one count per actual
/// trigger.  bit1=CA1 tick, bit2=SR, bit5=T2, etc.
#[inline]
pub fn via_irq_edge(cur: u8) {
    let prev = VIA_SRC_PREV.swap(cur as u64, Relaxed) as u8;
    let rising = cur & !prev;
    if rising == 0 {
        return;
    }
    for bit in 0..7 {
        if rising & (1 << bit) != 0 {
            VIA_SRC[bit].fetch_add(1, Relaxed);
        }
    }
}

/// Sample the CPU's IPL mask (0 = foreground, >0 = inside an ISR) at cycle
/// `now`. Detects the outermost enter/exit of the elevated-IPL region and, on
/// exit, records its duration. Nesting-safe (only the outermost region is
/// timed) and independent of RTE matching, so a non-interrupt RTE cannot skew
/// it. Call once per instruction step.
#[inline]
pub fn sample_ipl(mask: u8, now: Ticks) {
    let since = ISR_SINCE.load(Relaxed);
    if mask > 0 {
        if since == NOT_IN_ISR {
            ISR_SINCE.store(now, Relaxed);
        }
    } else if since != NOT_IN_ISR {
        ISR_SINCE.store(NOT_IN_ISR, Relaxed);
        let dur = now.saturating_sub(since);
        ISR_COUNT.fetch_add(1, Relaxed);
        ISR_CYCLES_SUM.fetch_add(dur, Relaxed);
        ISR_CYCLES_MAX.fetch_max(dur, Relaxed);
        ISR_CYCLES_MIN.fetch_min(dur, Relaxed);
    }
}

/// Emit a report and reset the counters if an emulated second has elapsed.
/// Cheap when it hasn't (a single atomic load + compare). Call once per step.
#[inline]
pub fn maybe_report(now: Ticks) {
    let last = LAST_REPORT.load(Relaxed);
    let interval = now.wrapping_sub(last);
    if interval < ONESEC_CYCLES {
        return;
    }
    // Single-threaded core: this CAS effectively always wins. It only guards
    // against a double report should the emulation core ever go multi-threaded.
    if LAST_REPORT
        .compare_exchange(last, now, Relaxed, Relaxed)
        .is_err()
    {
        return;
    }
    report(interval);
}

fn report(interval: Ticks) {
    let via_r = VIA_READS.swap(0, Relaxed);
    let via_w = VIA_WRITES.swap(0, Relaxed);
    let scc_r = SCC_READS.swap(0, Relaxed);
    let scc_w = SCC_WRITES.swap(0, Relaxed);
    let mut irqs = [0u64; 8];
    for (i, c) in IRQ_COUNT.iter().enumerate() {
        irqs[i] = c.swap(0, Relaxed);
    }
    let isr_n = ISR_COUNT.swap(0, Relaxed);
    let isr_sum = ISR_CYCLES_SUM.swap(0, Relaxed);
    let isr_max = ISR_CYCLES_MAX.swap(0, Relaxed);
    let isr_min = ISR_CYCLES_MIN.swap(u64::MAX, Relaxed);
    let mut via_src = [0u64; 8];
    for (i, c) in VIA_SRC.iter().enumerate() {
        via_src[i] = c.swap(0, Relaxed);
    }
    let isr_min = if isr_min == u64::MAX { 0 } else { isr_min };

    // Normalise counts to a per-second rate (the interval is ~1 s but rarely
    // exactly ONESEC_CYCLES, since it lands on an instruction boundary).
    let per_s = |n: u64| (n as f64 * ONESEC_CYCLES as f64 / interval as f64).round() as u64;
    let us = |cyc: u64| cyc as f64 / CYCLES_PER_US;
    let isr_mean = if isr_n > 0 { isr_sum / isr_n } else { 0 };
    let masked_pct = 100.0 * isr_sum as f64 / interval as f64;

    info!(
        "[perf/1s] VIA {}r+{}w ({}/s)  SCC {}r+{}w ({}/s)  \
         IRQ L1={} L2={} L4={} L7={} ({}/s)  \
         ISR n={} min={}cyc/{:.0}us mean={}cyc/{:.0}us max={}cyc/{:.0}us  masked={:.1}%",
        via_r,
        via_w,
        per_s(via_r + via_w),
        scc_r,
        scc_w,
        per_s(scc_r + scc_w),
        irqs[1],
        irqs[2],
        irqs[4],
        irqs[7],
        per_s(irqs.iter().sum()),
        isr_n,
        isr_min,
        us(isr_min),
        isr_mean,
        us(isr_mean),
        isr_max,
        us(isr_max),
        masked_pct,
    );
    // VIA level-1 IRQ source breakdown: CA1 is the 60 Hz tick; the rest are excess.
    info!(
        "[perf/1s] VIA-IRQ CA1(tick)={} SR(kbd)={} T2={} T1={} CB1={} CB2={} CA2={}  (excess over tick = {})",
        via_src[1], via_src[2], via_src[5], via_src[6], via_src[4], via_src[3], via_src[0],
        via_src.iter().sum::<u64>() - via_src[1],
    );
}
