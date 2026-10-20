use std::ops::{Deref, DerefMut};

use crate::bus::{Address, BusMember};
use crate::debuggable::Debuggable;
use crate::emulator::EmuContext;
use crate::mac::pluskbd::PlusKeyboard;
use crate::mac::rtc::Rtc;
use crate::mac::via_common::{LineState, ViaCommon, ViaLine, ViaReg, ViaWiring};
use crate::tickable::{TickConverter, Tickable, Ticks};
use crate::types::Byte;

use anyhow::Result;
use proc_bitfield::bitfield;
use serde::{Deserialize, Serialize};

use super::MacModel;
use super::adb::AdbTransceiver;

/// Counter at which to trigger the one second interrupt
/// (counted on the E Clock)
const ONESEC_TICKS: Ticks = 783360;

/// Time it takes the keyboard or the ADB transceiver to clock a byte through
/// the shift register. Ideally, these would time the transfer themselves.
const SHIFT_DELAY: Ticks = ONESEC_TICKS * 3 / 1000;
/// How long the M0110 holds an Inquiry ($10) with no key event before answering
/// $7B (~250 ms).  Modelling this stops macmon re-polling every tick (60/s) and
/// paces it at the real-hardware rate its own watchdog dictates.
const INQUIRY_HOLD: Ticks = ONESEC_TICKS / 4;

/// Env-gated keyboard/shift-register trace (SNOW_KBD_TRACE=1).
///
/// Added to settle macrom issues.md #175: a phantom `c` reaching the guest,
/// which decodes to raw $10 -- the Inquiry command byte the ROM itself wrote
/// to this register. The question the trace answers is whether the guest reads
/// SR back while it still holds that command, and what the ACR mode was when
/// it did.
fn kbdtrace(what: &str, a: u8, b: u8) {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    if *ON.get_or_init(|| std::env::var("SNOW_KBD_TRACE").is_ok()) {
        eprintln!("[KBD] {what} {a:#04x} {b:#04x}");
    }
}

const ACR_SHIFT_OUT: u8 = 0b111;
const ACR_SHIFT_IN: u8 = 0b011;

// SNOW_WATCH_IER T1 diagnostics
static T1CH_RESETS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
static T1CL_READS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

fn watch_ier() -> bool {
    use std::sync::OnceLock;
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var("SNOW_WATCH_IER").is_ok())
}

bitfield! {
    /// VIA Register A (for classic models)
    #[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct RegisterA(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        /// Sound volume (non-ASC models)
        pub sound: u8 @ 0..=2,

        /// Sound buffer
        /// (true = main, false = alternate)
        pub sndpg2: bool @ 3,

        /// ROM overlay map is used when set (512K and Plus)
        pub overlay: bool @ 4,

        /// Floppy drive select
        pub drivesel: bool @ 4,

        /// Disk SEL line
        pub sel: bool @ 5,

        /// Video page to be used by video circuitry
        /// (true = main, false = alternate)
        pub page2: bool @ 6,

        /// Another model sense bit
        pub model6: bool @ 6,

        /// SCC Wait/Request (false)
        pub sccwrreq: bool @ 7,
    }
}

bitfield! {
    /// VIA Register B (for classic models)
    #[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct RegisterB(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        /// RTC data line
        pub rtcdata: bool @ 0,

        /// RTC clock line
        pub rtcclk: bool @ 1,

        /// RTC enabled
        pub rtcenb: bool @ 2,

        /// Mouse switch (false = down) (512K and Plus)
        pub sw: bool @ 3,

        /// ADB interrupt (SE+)
        pub adb_int: bool @ 3,

        /// Mouse X2 (512K and Plus)
        pub mouse_x2: bool @ 4,

        /// ADB state input 0 (ST0) (SE+)
        pub adb_st0: bool @ 4,

        /// Mouse Y2 (512K and Plus)
        pub mouse_y2: bool @ 5,

        /// ADB state input 1 (ST1) (SE+)
        pub adb_st1: bool @ 5,

        /// HBlank (512K and Plus)
        pub h4: bool @ 6,

        /// SCSI interrupt (false = enabled) (SE+)
        pub scsi_int: bool @ 6,

        /// VBlank enable for SE/30
        pub se30_vblank_enable: bool @ 6,

        /// Sound enable
        pub sndenb: bool @ 7,

        pub pb0: bool @ 0,
        pub pb1: bool @ 1,
        pub pb2: bool @ 2,
        pub pb3: bool @ 3,
        pub pb4: bool @ 4,
        pub pb5: bool @ 5,
        pub pb6: bool @ 6,
        pub pb7: bool @ 7,
    }
}

bitfield! {
    /// VIA Interrupt flag/enable registers
    #[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
    pub struct RegisterIRQ(pub u8): Debug, FromStorage, IntoStorage, DerefStorage {
        /// One-second interrupt
        /// Cleared on Register A read/write
        pub onesec: bool @ 0,

        /// Vertical blank
        /// Cleared on Register A read/write
        pub vblank: bool @ 1,

        /// Keyboard data ready
        /// Cleared on read/write shift reg
        pub kbdready: bool @ 2,

        /// Keyboard data
        /// Cleared on Register B read/write
        pub kbddata: bool @ 3,

        /// Keyboard clock
        /// Cleared on Register B read/write
        pub kbdclock: bool @ 4,

        /// Timer T2
        /// Cleared on read of T2 counter LSB or write of T2 counter MSB
        pub t2: bool @ 5,

        /// Timer T1
        /// Cleared on read of T1 counter LSB or write of T1 counter MSB
        pub t1: bool @ 6,
    }
}

pub struct Via1Wiring;

impl ViaWiring for Via1Wiring {
    type RegisterA = RegisterA;
    type RegisterB = RegisterB;
    type RegisterIRQ = RegisterIRQ;
}

/// Synertek SY6522 Versatile Interface Adapter
#[derive(Serialize, Deserialize)]
pub struct Via {
    core: ViaCommon<Via1Wiring>,

    /// The currently emulated Macintosh model
    model: MacModel,

    /// Counter for the one-second timer
    onesec: Ticks,

    kbdshift_in: u8,
    kbdshift_in_time: Ticks,
    kbdshift_out_time: Ticks,
    /// >0 while an Inquiry is being held (M0110 not yet answering); counts down
    /// to the ~250 ms timeout, or is cleared early when a key event arrives.
    #[serde(default)]
    kbd_inquiry_hold: Ticks,

    pub keyboard: PlusKeyboard,
    pub(crate) rtc: Rtc,

    pub(crate) adb: AdbTransceiver,

    /// Runs the VIA at the speed of the bus of the real hardware, whatever
    /// the bus frequency is: the VIA is used as a timer
    bus_tick_converter: TickConverter,
}

impl Via {
    pub fn new(model: MacModel, bus_frequency: Ticks) -> Self {
        let mut core = ViaCommon::new();
        core.a_in = model.via1_a_in();

        Self {
            core,
            model,

            onesec: 0,
            kbdshift_in: 0,
            kbdshift_in_time: 0,
            kbdshift_out_time: 0,
            kbd_inquiry_hold: 0,

            keyboard: PlusKeyboard::default(),
            rtc: Rtc::default(),
            adb: AdbTransceiver::default(),

            bus_tick_converter: TickConverter::new(bus_frequency),
        }
    }
}

impl Deref for Via {
    type Target = ViaCommon<Via1Wiring>;

    fn deref(&self) -> &Self::Target {
        &self.core
    }
}

impl DerefMut for Via {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.core
    }
}

impl BusMember<Address> for Via {
    fn read(&mut self, addr: Address) -> Option<Byte> {
        if crate::perf::enabled() {
            crate::perf::via_read();
        }
        let reg = ViaReg::from_addr(addr);

        match reg {
            ViaReg::T1CL => {
                T1CL_READS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            }
            ViaReg::SR => kbdtrace("SR-read  val/acrmode", self.core.sr, self.core.acr.sr_mode()),
            ViaReg::B => {
                // Lazy update ADB Int to current state
                if self.model.has_adb() {
                    self.core.b_in.set_adb_int(!self.adb.get_int());
                }
            }
            ViaReg::A | ViaReg::ANH => {
                // The SCC write request bit is used for checking if there's data ready
                // from the SCC in critical sections where SCC interrupts are disabled.
                // Reporting 1 signals that there's no data ready.
                self.core.a_in.set_sccwrreq(true);
            }
            _ => (),
        }

        Some(self.core.read(reg))
    }

    fn write(&mut self, addr: Address, val: Byte) -> Option<()> {
        if crate::perf::enabled() {
            crate::perf::via_write();
        }
        let reg = ViaReg::from_addr(addr);

        match reg {
            ViaReg::T1CH => {
                T1CH_RESETS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            }
            ViaReg::SR => kbdtrace("SR-write val/acrmode", val, self.core.acr.sr_mode()),
            ViaReg::IER if watch_ier() => {
                // SNOW_WATCH_IER: log every IER write; flag when the T1 tick (0x40)
                // or CA1 (0x02) enable transitions 1->0 (loses the clock tick).
                let old = self.core.ier.0;
                let new = if val & 0x80 != 0 {
                    old | (val & 0x7F)
                } else {
                    old & !(val & 0x7F)
                };
                let lost_t1 = (old & 0x40) != 0 && (new & 0x40) == 0;
                let lost_ca1 = (old & 0x02) != 0 && (new & 0x02) == 0;
                eprintln!(
                    "[IER] write val={:#04x} ({}) IER {:#04x}->{:#04x}{}{}",
                    val,
                    if val & 0x80 != 0 { "SET" } else { "CLR" },
                    old,
                    new,
                    if lost_t1 { "  *** LOST T1 TICK ***" } else { "" },
                    if lost_ca1 { "  *** LOST CA1 ***" } else { "" },
                );
            }
            _ => (),
        }

        if reg == ViaReg::ACR {
            // SNOW_WATCH_IER: flag when ACR bit 6 (T1 free-run reload) is
            // cleared -- that switches T1 to one-shot, so the tick stops
            // after its next fire.
            if watch_ier() && (self.core.acr.0 & 0x40) != 0 && (val & 0x40) == 0 {
                eprintln!(
                    "[ACR] val={:#04x} {:#04x}->{:#04x}  *** LOST T1 FREE-RUN (bit6) ***",
                    val, self.core.acr.0, val
                );
            }
            let oldmode = self.core.acr.sr_mode();
            self.core.write(reg, val);
            let newmode = self.core.acr.sr_mode();

            if newmode != oldmode {
                kbdtrace("ACR-mode old/new", oldmode, newmode);
                // Reset shifter operation
                self.kbdshift_in_time = 0;
                self.kbdshift_out_time = 0;
            }
            if newmode == ACR_SHIFT_IN {
                self.kbdshift_in_time = SHIFT_DELAY;
            } else if newmode == ACR_SHIFT_OUT {
                self.kbdshift_out_time = SHIFT_DELAY;
            }
            return Some(());
        }

        self.core.write(reg, val);

        match reg {
            // Keyboard shift register
            ViaReg::SR => {
                if self.core.acr.sr_mode() == ACR_SHIFT_OUT {
                    // Start shift-out
                    self.kbdshift_out_time = SHIFT_DELAY;
                }
            }
            ViaReg::B => {
                let rtcin = self.rtc.io(
                    self.core.pb().rtcenb(),
                    self.core.pb().rtcclk(),
                    self.core.pb().rtcdata(),
                );
                self.core.b_in.set_rtcdata(rtcin);

                if self.model.has_adb()
                    && let Some(b) = self
                        .adb
                        .io(self.core.pb().adb_st0(), self.core.pb().adb_st1())
                {
                    self.kbdshift_in = b;
                }
            }
            _ => (),
        }

        Some(())
    }
}

impl Tickable<&dyn EmuContext> for Via {
    fn tick(&mut self, ticks: Ticks, ctx: &dyn EmuContext) -> Result<Ticks> {
        self.bus_tick_converter.add_a_ticks(ticks);
        let via_ticks = self.bus_tick_converter.get_b_ticks(ctx.bus_frequency());
        if via_ticks > 0 {
            self.tick_16mhz(via_ticks)?;
            self.bus_tick_converter
                .subtract_b_ticks(via_ticks, ctx.bus_frequency());
        }
        Ok(ticks)
    }
}

impl Via {
    fn tick_16mhz(&mut self, ticks: Ticks) -> Result<Ticks> {
        // SNOW_WATCH_IER: periodic dump of T1 health -- fires, T1CH-resets,
        // enable/free-run/IER state -- to see whether the tick T1 stops
        // overflowing under load and why.
        if watch_ier() {
            use std::sync::atomic::{AtomicU64, Ordering::Relaxed};
            static CALLS: AtomicU64 = AtomicU64::new(0);
            if CALLS.fetch_add(1, Relaxed) % 3_000_000 == 2_999_999 {
                eprintln!(
                    "[T1] fires={} t1ch_resets={} t1cl_reads={} | t1_enable={} freerun={} ier_t1={} t1cnt={:#06x} latch={:#06x}",
                    self.core.t1_fires,
                    T1CH_RESETS.load(Relaxed),
                    T1CL_READS.load(Relaxed),
                    self.core.t1_enabled(),
                    self.core.acr.t1_freerun(),
                    (self.core.ier.0 & 0x40) != 0,
                    self.core.t1cnt.0,
                    self.core.t1latch.0,
                );
            }
        }
        // This is ticked on the E Clock
        self.onesec += ticks;

        // One second interrupt
        if self.onesec >= ONESEC_TICKS {
            self.onesec -= ONESEC_TICKS;
            self.core.set_line(ViaLine::CA2, LineState::Pulse);
            self.rtc.second();
        }

        self.core.tick_timers(ticks)?;

        // ADB/Keyboard response
        if self.kbdshift_out_time > 0 {
            self.kbdshift_out_time = self.kbdshift_out_time.saturating_sub(ticks);
            if self.kbdshift_out_time == 0 {
                let data = self.core.shift_sr(0);
                if !self.model.has_adb() {
                    match self.keyboard.cmd(data)? {
                        Some(b) => {
                            self.kbdshift_in = b;
                            self.kbd_inquiry_hold = 0;
                        }
                        // Inquiry with no key: the keyboard holds -- don't commit
                        // a response byte yet; the receive phase below waits.
                        None => self.kbd_inquiry_hold = INQUIRY_HOLD,
                    }
                } else {
                    self.adb.data_in(data);
                }
                kbdtrace("OUT-done  cmd/sr", data, self.core.sr);
            }
        }
        if self.kbdshift_in_time > 0 {
            self.kbdshift_in_time = self.kbdshift_in_time.saturating_sub(ticks);
            if self.kbdshift_in_time == 0 {
                // Resolve a held Inquiry: the real M0110 clocks nothing back
                // until a key event arrives or ~250 ms elapses.  Until then,
                // re-arm and keep waiting instead of delivering a response.
                if self.kbd_inquiry_hold > 0 && !self.model.has_adb() {
                    if self.keyboard.pending_events() {
                        if let Some(b) = self.keyboard.cmd(0x14)? {
                            self.kbdshift_in = b;
                        }
                        self.kbd_inquiry_hold = 0;
                    } else {
                        self.kbd_inquiry_hold = self.kbd_inquiry_hold.saturating_sub(SHIFT_DELAY);
                        if self.kbd_inquiry_hold == 0 {
                            self.kbdshift_in = 0x7B; // hold expired -> null
                        } else {
                            self.kbdshift_in_time = SHIFT_DELAY; // still holding
                            return Ok(ticks);
                        }
                    }
                }
                kbdtrace("IN-done   byte/acrmode", self.kbdshift_in, self.core.acr.sr_mode());
                self.core.shift_sr(self.kbdshift_in);
                self.kbdshift_in_time = SHIFT_DELAY;
            }
        }

        Ok(ticks)
    }
}

impl Debuggable for Via {
    fn get_debug_properties(&self) -> crate::debuggable::DebuggableProperties {
        use crate::debuggable::*;
        use crate::{dbgprop_byte_bin, dbgprop_group, dbgprop_udec};

        let mut props = self.core.debug_properties_ports();
        props.push(dbgprop_group!(
            "Keyboard shifter",
            vec![
                dbgprop_byte_bin!("Shift register", self.core.sr),
                dbgprop_byte_bin!("Input", self.kbdshift_in),
                dbgprop_udec!("Input timer", self.kbdshift_in_time),
                dbgprop_udec!("Output timer", self.kbdshift_out_time),
            ]
        ));
        props.push(dbgprop_udec!("One second timer", self.onesec));
        props.extend(self.core.debug_properties_control());
        props
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mac::via_common::{IRQ_CA1, IRQ_CA2, IRQ_CB1, IRQ_CB2};

    /// Register A with handshake
    const ADDR_A_HS: Address = 0x01 << 9;
    /// Register A without handshake (vBufA)
    const ADDR_A: Address = 0x0F << 9;
    const ADDR_PCR: Address = 0x0C << 9;
    const ADDR_B: Address = 0x00 << 9;

    #[test]
    fn reg_a_no_handshake_keeps_irq_flags() {
        let mut via = Via::new(MacModel::Plus, 16_000_000);

        via.set_irq(IRQ_CA1);
        via.set_irq(IRQ_CA2);
        via.read(ADDR_A);
        assert!(via.ifr().vblank() && via.ifr().onesec());
        via.write(ADDR_A, 0);
        assert!(via.ifr().vblank() && via.ifr().onesec());
    }

    #[test]
    fn reg_a_handshake_clears_irq_flags() {
        let mut via = Via::new(MacModel::Plus, 16_000_000);

        via.set_irq(IRQ_CA1);
        via.set_irq(IRQ_CA2);
        via.read(ADDR_A_HS);
        assert!(!via.ifr().vblank() && !via.ifr().onesec());

        via.set_irq(IRQ_CA1);
        via.set_irq(IRQ_CA2);
        via.write(ADDR_A_HS, 0);
        assert!(!via.ifr().vblank() && !via.ifr().onesec());
    }

    #[test]
    fn independent_ca2_keeps_irq_flag() {
        let mut via = Via::new(MacModel::Plus, 16_000_000);

        // CA2 independent interrupt input, negative edge
        via.write(ADDR_PCR, 0x02);
        via.set_irq(IRQ_CA1);
        via.set_irq(IRQ_CA2);
        via.read(ADDR_A_HS);
        assert!(!via.ifr().vblank() && via.ifr().onesec());

        // Positive edge
        via.write(ADDR_PCR, 0x06);
        via.write(ADDR_A_HS, 0);
        assert!(via.ifr().onesec());

        // Handshake output is not independent
        via.write(ADDR_PCR, 0x0A);
        via.read(ADDR_A_HS);
        assert!(!via.ifr().onesec());
    }

    #[test]
    fn independent_cb2_keeps_irq_flag() {
        let mut via = Via::new(MacModel::Plus, 16_000_000);

        via.set_irq(IRQ_CB2);
        via.set_irq(IRQ_CB1);
        via.read(ADDR_B);
        assert!(!via.ifr().kbddata() && !via.ifr().kbdclock());

        // CB2 independent interrupt input, negative edge
        via.write(ADDR_PCR, 0x20);
        via.set_irq(IRQ_CB2);
        via.set_irq(IRQ_CB1);
        via.read(ADDR_B);
        assert!(via.ifr().kbddata() && !via.ifr().kbdclock());

        // Positive edge
        via.write(ADDR_PCR, 0x60);
        via.write(ADDR_B, 0);
        assert!(via.ifr().kbddata());

        // Handshake output is not independent
        via.write(ADDR_PCR, 0xA0);
        via.read(ADDR_B);
        assert!(!via.ifr().kbddata());
    }
}
