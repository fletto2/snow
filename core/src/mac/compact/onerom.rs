//! Behavioural model of a One ROM device sitting in the Macintosh ROM socket.
//!
//! Ported from `agfaio/emulator/onerom.c` in the XCP/M-68K tree, which was
//! itself written from the DEVICE FIRMWARE (piersfinlayson/one-rom, commit
//! 12e6c619) rather than from the specification -- so a host tested against
//! this is tested against what the device does, not what the document says.
//! Where the two differ, the firmware wins and the difference is a finding.
//!
//! # Why this exists
//!
//! macrom has two One ROM hosts -- `romsel` (a POSiniX program) and MacGrub (a
//! boot-time selector that IS the ROM image) -- and neither had ever spoken to
//! a device. Every device path was "built and refusal-tested only": the code
//! probes, gets no answer, and reports so. That is the correct behaviour with
//! no device fitted, and it exercises none of the protocol.
//!
//! # How a command reaches the device
//!
//! The device has no data-in path. It WATCHES ITS OWN ADDRESS BUS: a command
//! byte `b` is sent by the host READING `base + (cmd_page * 256 + b) * stride`.
//! So every read in the window is fed to the state machine, which walks
//! knock -> group -> command -> arguments and then executes. Replies come back
//! through the "back-channel", a region inside the slot the device is serving
//! that it writes its response header and data into.
//!
//! That is also why this must model a BUS CYCLE rather than a byte: a word
//! access is one cycle with every strobe asserted and presents ONE command
//! byte, not two. `access()` observes a cycle; `fetch()` returns the second
//! byte of a word without observing another.
//!
//! # What is and is not ported
//!
//! Ported: the state machine, the addressing, the back-channel, and the
//! commands the two hosts actually use -- ENTER/EXIT, the READ group, the
//! MODIFY group, the NV group, and RESET.
//!
//! NOT ported: the fault-injection knobs (`orom_skew`, `orom_scramble`,
//! `orom_deaf`, `orom_stuck`, `orom_loadfail`, ...). Those exist to test a
//! host's ERROR handling, and they should come across when there is a host
//! error path worth exercising. Their absence is why this is a device model
//! and not yet a device-fault model, and it is stated here rather than left to
//! be discovered by someone who assumes a green run covered them.
//!
//! Off unless `SNOW_ONEROM=1`, so ordinary emulation is untouched.

use serde::{Deserialize, Serialize};

const KNOCK: [u8; 6] = [b'!', b'R', b'B', b'C', b'P', b'!'];

const HDR_LAST_G: u32 = 0;
const HDR_LAST_C: u32 = 1;
const HDR_TOK_LO: u32 = 2;
const HDR_TOK_HI: u32 = 3;
const HDR_PROG: u32 = 4;
const HDR_RESP: u32 = 5;
const HDR_SIZE: u32 = 8;

const G_CONTROL: u8 = 0x00;
const G_READ: u8 = 0x01;
const G_MODIFY: u8 = 0x02;
const G_NV: u8 = 0x03;
const G_RESET: u8 = 0xAA;

const NRAM: usize = 4;
const NFLASH: usize = 16;
const NV_SIZE: usize = 4096;

/// Fault-injection settings. These exist to exercise a HOST'S ERROR HANDLING,
/// which is the half of correctness a happy-path model cannot reach: a guard
/// nobody has watched fire is a claim, not a feature.
#[derive(Clone, Copy, Default, Serialize, Deserialize)]
struct Faults {
    /// Devices disagree about which RAM slot is active. A host picks its write
    /// destination AGAINST that field, so on a broadcast bus a disagreement
    /// means one device's answer choosing a slot that is LIVE on another.
    skew: bool,
    /// Stop maintaining the back-channel after N commands: the host issues a
    /// command and watches a token that never moves.
    deaf: u32,
    /// Start already in command-response mode, as a device left that way by a
    /// host reset mid-session really is -- a Mac reset restarts the CPU and
    /// NOT the One ROM. In that state no knock is needed and every read on the
    /// command page decodes as a command.
    entered: bool,
    /// Refuse SLOT_PEEK: the state in which a read-back verify proves nothing
    /// and must say so rather than passing.
    nopeek: bool,
    /// 1 = LOAD_SLOT refuses; 2 = it fails HALF DONE, leaving the target slot
    /// holding part of an image.
    loadfail: u8,
}

/// Reserved protocol values. A device that does not know an answer reports one
/// of these, so they must never be accepted as a slot number.
const RESERVED_AA: u8 = 0xAA;

#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
enum St {
    Knock,
    Group,
    Cmd,
    Args,
}

#[derive(Serialize, Deserialize)]
struct Dev {
    ram: Vec<Vec<u8>>,
    flash: Vec<Vec<u8>>,
    size: u32,
    active_slot: usize,

    st: St,
    kmatch: usize,
    kmax: usize,
    group: u8,
    cmd: u8,
    args: [u8; 16],
    nargs: usize,
    want: usize,

    active: bool,
    cmd_page: u16,
    region_off: u32,
    data_size: u32,
    complete: u8,
    status_ok: u8,
    tok_lo: u8,
    tok_hi: u8,

    cmds: u32,
    idx: usize,
    nram: usize,
    nflash: usize,
    nv: Vec<u8>,
    stage: Option<Vec<u8>>,
    boot_flash: u8,
    boot_ram: u8,
}

impl Dev {
    fn new(idx: usize, size: u32, nram: usize, nflash: usize) -> Self {
        Self {
            ram: (0..NRAM).map(|_| vec![0u8; size as usize]).collect(),
            flash: (0..NFLASH).map(|_| vec![0u8; size as usize]).collect(),
            size,
            active_slot: 0,
            st: St::Knock,
            kmatch: 0,
            kmax: 0,
            group: 0,
            cmd: 0,
            args: [0; 16],
            nargs: 0,
            want: 0,
            active: false,
            cmd_page: 0,
            region_off: 0,
            data_size: 0,
            // A device that has never been entered reports 0xFF for both, so
            // a host reading them before ENTER sees "does not know".
            complete: 0xFF,
            status_ok: 0xFF,
            tok_lo: 0,
            tok_hi: 0,
            cmds: 0,
            idx,
            nram,
            nflash,
            nv: vec![0xFF; NV_SIZE], // blank NV is 0xFF, per the spec
            stage: None,
            boot_flash: 0,
            boot_ram: 0,
        }
    }

    fn hdr_write(&mut self, off: u32, v: u8) {
        let a = self.region_off + off;
        if a < self.size {
            let slot = self.active_slot;
            self.ram[slot][a as usize] = v;
        }
    }

    fn hdr_read(&self, off: u32) -> u8 {
        let a = self.region_off + off;
        if a < self.size {
            self.ram[self.active_slot][a as usize]
        } else {
            0
        }
    }

    fn data_write(&mut self, off: u32, p: &[u8]) {
        if off >= self.data_size {
            return;
        }
        let mut n = p.len() as u32;
        if off + n > self.data_size {
            n = self.data_size - off;
        }
        for i in 0..n {
            self.hdr_write(HDR_SIZE + off + i, p[i as usize]);
        }
    }

    /// Bump the token and record the command, BEFORE running it. The host
    /// polls the token to see that its command was noticed at all, separately
    /// from whether it succeeded.
    fn cmd_begin(&mut self, g: u8, c: u8) {
        let prog = !self.complete;
        self.hdr_write(HDR_PROG, prog);
        self.tok_lo = self.tok_lo.wrapping_add(1);
        if self.tok_lo == 0 {
            self.tok_hi = self.tok_hi.wrapping_add(1);
        }
        let (lo, hi) = (self.tok_lo, self.tok_hi);
        self.hdr_write(HDR_TOK_LO, lo);
        self.hdr_write(HDR_TOK_HI, hi);
        self.hdr_write(HDR_LAST_G, g);
        self.hdr_write(HDR_LAST_C, c);
    }

    fn cmd_end(&mut self, ok: bool) {
        let resp = if ok { self.status_ok } else { !self.status_ok };
        let prog = self.complete;
        self.hdr_write(HDR_RESP, resp);
        self.hdr_write(HDR_PROG, prog);
    }
}

/// Arguments a command consumes before it executes. An unknown command
/// consumes nothing, which is how a desynchronised session stays desynchronised
/// rather than silently swallowing the bytes that follow.
fn arg_count(g: u8, c: u8) -> usize {
    match g {
        G_CONTROL => match c {
            0x01 => 9, // ENTER_CMD_RESP
            0x04 => 1, // SWITCH_AND_EXIT
            0x05 => 2, // LOAD_AND_EXIT
            0x06 => 9, // EXIT_CMD_RESP_RESTORE
            _ => 0,
        },
        G_READ => match c {
            0x01 => 1, // FLASH_SLOT_INFO   slot
            0x07 => 5, // SLOT_PEEK         count, addr[3], slot
            _ => 0,
        },
        G_NV => match c {
            0x01 => 3, // NV_PEEK           count, loc_lsb, loc_msb
            0x02 => 1, // NV_POKE_BEGIN     slot
            0x03 => 3, // NV_POKE           byte, loc_lsb, loc_msb
            0x06 => 4, // NV_POKE_COMMIT_BYTE
            _ => 0,
        },
        G_MODIFY => match c {
            0x00 => 5, // SLOT_POKE
            0x01 => 1, // SWITCH_SLOT
            0x02 => 2, // LOAD_SLOT
            0x03 => 2, // SLOT_POKE_ALL_BYTE
            _ => 0,
        },
        _ => 0,
    }
}

#[derive(Serialize, Deserialize)]
pub struct OneRom {
    devs: Vec<Dev>,
    ndev: usize,
    gated: bool,
    dev_bytes: u32,
    stride: u32,
    base: u32,
    per_dev: u32,
    swap: bool,
    faults: Faults,
}

impl OneRom {
    /// Build the model from the environment, or None when not enabled.
    ///
    /// * `SNOW_ONEROM=1`          - enable
    /// * `SNOW_ONEROM_BASE`       - window base (default 0x400000, the Plus ROM)
    /// * `SNOW_ONEROM_DEVS`       - devices on the bus (default 2, one per lane)
    /// * `SNOW_ONEROM_FLASH`      - flash slots each device reports (default 3)
    /// * `SNOW_ONEROM_RAMSLOTS`   - RAM slots each device reports (default 2)
    /// * `SNOW_ONEROM_NV0`        - preload NV byte 0 (the saved default slot);
    ///                              absent leaves NV blank (0xFF = nothing saved)
    pub fn from_env(rom: &[u8]) -> Option<Self> {
        if std::env::var("SNOW_ONEROM").ok().as_deref() != Some("1") {
            return None;
        }
        let num = |k: &str, d: u32| {
            std::env::var(k)
                .ok()
                .and_then(|v| {
                    let v = v.trim().to_string();
                    if let Some(h) = v.strip_prefix("0x") {
                        u32::from_str_radix(h, 16).ok()
                    } else {
                        v.parse::<u32>().ok()
                    }
                })
                .unwrap_or(d)
        };

        let base = num("SNOW_ONEROM_BASE", 0x0040_0000);
        let ndev = num("SNOW_ONEROM_DEVS", 2).clamp(1, 4) as usize;
        let nflash = num("SNOW_ONEROM_FLASH", 3).clamp(1, NFLASH as u32) as usize;
        let nram = num("SNOW_ONEROM_RAMSLOTS", 2).clamp(1, NRAM as u32) as usize;
        // NV byte 0 is where both hosts keep the default boot slot. Blank NV is
        // 0xFF ("nothing saved"), and with that a boot menu's honest default is
        // the slot it is already running -- which a switch must refuse. So a
        // preloaded value is what makes the SWITCH path reachable at all.
        let nv0 = std::env::var("SNOW_ONEROM_NV0")
            .ok()
            .and_then(|v| v.trim().parse::<u8>().ok());

        let flag = |k: &str| std::env::var(k).ok().as_deref() == Some("1");
        let faults = Faults {
            skew: flag("SNOW_ONEROM_SKEW"),
            deaf: num("SNOW_ONEROM_DEAF", 0),
            entered: flag("SNOW_ONEROM_ENTERED"),
            nopeek: flag("SNOW_ONEROM_NOPEEK"),
            loadfail: num("SNOW_ONEROM_LOADFAIL", 0) as u8,
        };

        let dev_bytes: u32 = 1; // two 8-bit devices on a 16-bit bus
        let stride = ndev as u32 * dev_bytes;
        let per_dev = (rom.len() as u32) / stride.max(1);

        let mut s = Self {
            devs: (0..ndev)
                .map(|i| {
                    let mut d = Dev::new(i, per_dev, nram, nflash);
                    if let Some(v) = nv0 {
                        d.nv[0] = v;
                    }
                    d
                })
                .collect(),
            ndev,
            gated: true,
            dev_bytes,
            stride,
            base,
            per_dev,
            swap: dev_bytes == 2,
            faults,
        };

        // A device left in command-response mode by a host reset. The page and
        // back-channel default to MacGrub's own configuration, since that is
        // the host this state is most likely to be inherited from.
        if faults.entered {
            let page = num("SNOW_ONEROM_ENTERED_PAGE", 0xE0) as u16;
            let bch = num("SNOW_ONEROM_ENTERED_BCH", 0xE100);
            for d in s.devs.iter_mut() {
                d.active = true;
                d.cmd_page = page;
                d.region_off = bch;
                d.data_size = 160 - HDR_SIZE;
                d.complete = 0xBB;
                d.status_ok = 0xCC;
            }
        }
        // Devices disagreeing about the active slot.
        if faults.skew {
            for (i, d) in s.devs.iter_mut().enumerate() {
                if i > 0 && nram > 1 {
                    d.active_slot = 1;
                }
            }
        }

        // Seed every slot with the ROM image the machine was launched with,
        // split across the devices exactly as the hardware splits it. Without
        // this the device serves zeros and the machine cannot boot at all.
        s.load_image(rom);

        log::warn!(
            "One ROM model ENABLED: {} device(s) at ${:06X}, {} bytes each, \
             {} flash slot(s), {} RAM slot(s)",
            ndev,
            base,
            per_dev,
            nflash,
            nram
        );
        Some(s)
    }

    fn load_image(&mut self, image: &[u8]) {
        for (i, byte) in image.iter().enumerate() {
            let cyc = i as u32 / self.stride;
            let lane = (i as u32 % self.stride) as usize / self.dev_bytes as usize;
            let k = (i as u32 % self.stride) % self.dev_bytes;
            let off = cyc * self.dev_bytes + k;
            if lane < self.ndev && off < self.per_dev {
                for s in 0..NRAM {
                    self.devs[lane].ram[s][off as usize] = *byte;
                }
                for s in 0..NFLASH {
                    self.devs[lane].flash[s][off as usize] = *byte;
                }
            }
        }
    }

    pub fn covers(&self, addr: u32) -> bool {
        addr >= self.base && addr < self.base + self.per_dev * self.stride
    }

    /// Device offset and lane for a CPU address.
    fn map(&self, addr: u32) -> (usize, u32) {
        let rel = addr - self.base;
        let cyc = rel / self.stride;
        let within = rel % self.stride;
        let lane = (within / self.dev_bytes) as usize;
        let mut k = within % self.dev_bytes;
        if self.swap {
            k = self.dev_bytes - 1 - k;
        }
        (lane.min(self.ndev - 1), cyc * self.dev_bytes + k)
    }

    /// One CPU access. `width` is 1 for a byte access, 2 for a word.
    ///
    /// A word access is ONE bus cycle with every strobe asserted, so it reaches
    /// every device however the selects are wired -- which is why the observed
    /// value is fed to all of them here, and why `fetch` exists for the second
    /// byte.
    pub fn access(&mut self, addr: u32, width: u32) -> u8 {
        let (lane, off) = self.map(addr);
        let observed = off;

        if width >= 2 || !self.gated {
            for i in 0..self.ndev {
                feed(&mut self.devs[i], observed, &self.faults);
            }
        } else {
            feed(&mut self.devs[lane], observed, &self.faults);
        }
        self.fetch(addr)
    }

    /// The served byte WITHOUT observing a cycle -- the second half of a word
    /// access, whose single cycle `access` has already fed. Feeding twice would
    /// present two command bytes where the hardware presents one.
    pub fn fetch(&self, addr: u32) -> u8 {
        let (lane, off) = self.map(addr);
        let d = &self.devs[lane];
        if off < d.size {
            d.ram[d.active_slot][off as usize]
        } else {
            0xFF
        }
    }

    /// Harness diagnostics.
    pub fn entered(&self, dev: usize) -> bool {
        self.devs.get(dev).map(|d| d.active).unwrap_or(false)
    }
    pub fn commands(&self, dev: usize) -> u32 {
        self.devs.get(dev).map(|d| d.cmds).unwrap_or(0)
    }
}

/// Feed one observed bus cycle to a device's state machine.
fn feed(d: &mut Dev, observed: u32, faults: &Faults) {
    // Once entered, only reads on the command page are commands; everything
    // else is an ordinary fetch of the image being served.
    if d.active && (observed >> 8) != d.cmd_page as u32 {
        return;
    }
    let b = (observed & 0xFF) as u8;

    match d.st {
        St::Knock => {
            if b == KNOCK[d.kmatch] {
                d.kmatch += 1;
                if d.kmatch == KNOCK.len() {
                    d.kmatch = 0;
                    d.st = St::Group;
                    if std::env::var("SNOW_ONEROM_TRACE").ok().as_deref() == Some("1") {
                        log::warn!("[onerom] dev{} KNOCK accepted", d.idx);
                    }
                }
            } else {
                // Record how far the knock ever got before being broken. A host
                // whose OWN INSTRUCTION FETCHES come from this window
                // interleaves them with its command reads, and the device
                // cannot tell the two apart -- so the sequence never completes.
                if d.kmatch > d.kmax {
                    d.kmax = d.kmatch;
                    if std::env::var("SNOW_ONEROM_TRACE").ok().as_deref() == Some("1") {
                        log::warn!(
                            "[onerom] dev{} knock reached {}/{} then broke on {:02X}",
                            d.idx, d.kmatch, KNOCK.len(), b
                        );
                    }
                }
                d.kmatch = usize::from(b == KNOCK[0]);
            }
        }
        St::Group => {
            d.group = b;
            d.st = St::Cmd;
        }
        St::Cmd => {
            d.cmd = b;
            d.nargs = 0;
            d.want = arg_count(d.group, d.cmd);
            if d.want == 0 {
                run_command(d, faults);
                d.st = if d.active { St::Group } else { St::Knock };
            } else {
                d.st = St::Args;
            }
        }
        St::Args => {
            if d.nargs < d.args.len() {
                d.args[d.nargs] = b;
            }
            d.nargs += 1;
            if d.nargs >= d.want {
                run_command(d, faults);
                d.st = if d.active { St::Group } else { St::Knock };
            }
        }
    }
}

/// Commands that exit "without updating the response header", per the spec --
/// so the host must NOT wait for a completion that will never be written.
/// Getting this set wrong is not subtle: ENTER is NOT silent, and treating it
/// as such leaves the host reporting "command received but never completed"
/// against a device that entered perfectly well.
fn cmd_is_silent(g: u8, c: u8) -> bool {
    match g {
        G_RESET => c == 0xAA,
        G_CONTROL => matches!(c, 0x03 | 0x04 | 0x05 | 0x06),
        _ => false,
    }
}

fn run_command(d: &mut Dev, faults: &Faults) {
    let was = d.active;
    // Deaf: execute the command but write nothing back, so the host sees a
    // token that never moves and must time out rather than hang.
    if faults.deaf > 0 && d.cmds >= faults.deaf {
        let _ = dispatch(d, faults);
        d.cmds += 1;
        return;
    }
    if std::env::var("SNOW_ONEROM_TRACE").ok().as_deref() == Some("1") {
        log::warn!(
            "[onerom] dev{} #{} g={:02X} c={:02X} args={:02X?} active={}",
            d.idx, d.cmds, d.group, d.cmd, &d.args[..d.want.min(9)], was
        );
    }
    let silent = cmd_is_silent(d.group, d.cmd);

    // The header can only be written while a region is known, which is why the
    // bookkeeping is keyed on the state BEFORE dispatch.
    if was && !silent {
        let (g, c) = (d.group, d.cmd);
        d.cmd_begin(g, c);
    }

    let ok = dispatch(d, faults);

    if was && !silent {
        d.cmd_end(ok);
    } else if !was && d.active {
        // ENTER: there was no region to write into when the command arrived,
        // and there is one now. Both halves of the header go out here, or the
        // host waits forever for a completion nobody wrote.
        let (g, c) = (d.group, d.cmd);
        d.cmd_begin(g, c);
        d.cmd_end(true);
    }

    d.cmds += 1;
}

fn dispatch(d: &mut Dev, faults: &Faults) -> bool {
    match d.group {
        // RESET resynchronises a device whose session has desynchronised, so
        // it must work in ANY state and must not touch the back-channel.
        G_RESET => {
            d.st = St::Knock;
            d.kmatch = 0;
            d.active = false;
            d.stage = None;
            true
        }
        G_CONTROL => match d.cmd {
            0x00 => true, // NOP
            0x01 => exec_enter(d),
            0x02 | 0x03 => {
                d.active = false;
                true
            }
            0x04 => {
                // SWITCH_AND_EXIT: the whole point of the toolkit.
                if d.args[0] != RESERVED_AA && (d.args[0] as usize) < d.nram {
                    d.active_slot = d.args[0] as usize;
                }
                d.active = false;
                true
            }
            0x05 => {
                // LOAD_AND_EXIT
                let (r, f) = (d.args[0] as usize, d.args[1] as usize);
                if d.args[0] != RESERVED_AA && d.args[1] != RESERVED_AA
                    && r < d.nram && f < NFLASH
                {
                    d.ram[r] = d.flash[f].clone();
                }
                d.active = false;
                true
            }
            0x06 => {
                // EXIT_CMD_RESP_RESTORE: put back the bytes the back-channel
                // displaced, then leave. Count must be 1..8.
                let n = d.args[8];
                if (1..=8).contains(&n) {
                    for k in 0..n as u32 {
                        let v = d.args[k as usize];
                        d.hdr_write(k, v);
                    }
                }
                d.active = false;
                true
            }
            _ => false, // unknown: desync
        },
        G_READ => {
            if !d.active {
                return false;
            }
            match d.cmd {
                0x01 => {
                    // FLASH_SLOT_INFO: one 32-byte record
                    let sl = d.args[0] as usize;
                    if d.args[0] == RESERVED_AA || sl >= d.nflash {
                        return false;
                    }
                    if d.data_size < 32 {
                        return false;
                    }
                    let rec = slot_record(sl, d.size);
                    d.data_write(0, &rec);
                    true
                }
                0x02 => {
                    // FLASH_SLOT_INFO_ALL: preamble then as many records as fit
                    let total = d.nflash as u8;
                    let cap = (d.data_size.saturating_sub(4) / 32) as u8;
                    let whole = total.min(cap);
                    let pre = [total, whole, 32u8, 0u8];
                    d.data_write(0, &pre);
                    for s in 0..whole as usize {
                        let rec = slot_record(s, d.size);
                        d.data_write(4 + (s as u32) * 32, &rec);
                    }
                    true
                }
                0x03 => {
                    // RAM_SLOT_INFO_ALL: total, active, TYPE, 0.
                    //
                    // Offset 2 was left out of this port and read back as
                    // whatever the back-channel still held -- zero, which the
                    // type table reads as a 2 KB part. A host checks the size
                    // it was configured with against this byte, so a 64 KB
                    // device claiming 2 KB is refused before any write, which
                    // is what romsel did. The byte has to name the part the
                    // slot actually serves.
                    let info = [
                        d.nram as u8,
                        d.active_slot as u8,
                        rom_type_for(d.size),
                        0u8,
                    ];
                    d.data_write(0, &info);
                    true
                }
                0x04 => {
                    d.data_write(0, b"ONEROM-MODEL\0");
                    true
                }
                0x05 => {
                    d.data_write(0, b"0.0.0\0");
                    true
                }
                0x06 => {
                    d.data_write(0, &[0u8, 1u8, 2u8]); // protocol 0.1.2
                    true
                }
                0x07 => {
                    // SLOT_PEEK: read back what a poke wrote
                    if faults.nopeek {
                        return false;
                    }
                    let count = d.args[0] as u32;
                    let addr = d.args[1] as u32
                        | (d.args[2] as u32) << 8
                        | (d.args[3] as u32) << 16;
                    let slot = d.args[4] as usize;
                    if slot >= d.nram || count == 0 || count > d.data_size {
                        return false;
                    }
                    let mut buf = Vec::with_capacity(count as usize);
                    for i in 0..count {
                        let a = addr + i;
                        buf.push(if a < d.size { d.ram[slot][a as usize] } else { 0xFF });
                    }
                    d.data_write(0, &buf);
                    true
                }
                0x08 => {
                    let info = [d.boot_flash, d.boot_ram];
                    d.data_write(0, &info);
                    true
                }
                _ => false,
            }
        }
        G_MODIFY => {
            if !d.active {
                return false;
            }
            match d.cmd {
                0x00 => {
                    // SLOT_POKE: byte, addr[3], slot
                    let byte = d.args[0];
                    let addr = d.args[1] as u32
                        | (d.args[2] as u32) << 8
                        | (d.args[3] as u32) << 16;
                    let slot = d.args[4] as usize;
                    // Never the running slot, and never a reserved number:
                    // the device refuses rather than overwrite what it serves.
                    if d.args[4] == RESERVED_AA || slot >= d.nram || slot == d.active_slot {
                        return false;
                    }
                    if addr >= d.size {
                        return false;
                    }
                    d.ram[slot][addr as usize] = byte;
                    true
                }
                0x01 => {
                    let slot = d.args[0] as usize;
                    if d.args[0] == RESERVED_AA || slot >= d.nram {
                        return false;
                    }
                    d.active_slot = slot;
                    true
                }
                0x02 => {
                    // LOAD_SLOT: copy a flash image into a RAM slot
                    let (r, f) = (d.args[0] as usize, d.args[1] as usize);
                    if d.args[0] == RESERVED_AA || r >= d.nram || f >= d.nflash {
                        return false;
                    }
                    if faults.loadfail == 1 {
                        return false;
                    }
                    if faults.loadfail == 2 {
                        // HALF DONE: the target slot now holds part of an
                        // image. This is why "nothing was changed" is a lie
                        // after a failed write, and why a host must say a
                        // spare slot may hold a fragment.
                        let half = d.size as usize / 2;
                        d.ram[r][..half].copy_from_slice(&d.flash[f][..half]);
                        return false;
                    }
                    d.ram[r] = d.flash[f].clone();
                    true
                }
                0x03 => {
                    let byte = d.args[0];
                    let slot = d.args[1] as usize;
                    if d.args[1] == RESERVED_AA || slot >= d.nram || slot == d.active_slot {
                        return false;
                    }
                    for b in d.ram[slot].iter_mut() {
                        *b = byte;
                    }
                    true
                }
                _ => false,
            }
        }
        G_NV => {
            if !d.active {
                return false;
            }
            match d.cmd {
                0x00 => {
                    let sz = NV_SIZE as u32;
                    let info = [(sz & 0xFF) as u8, (sz >> 8) as u8, 1u8];
                    d.data_write(0, &info);
                    true
                }
                0x01 => {
                    let count = d.args[0] as u32;
                    let loc = d.args[1] as u32 | (d.args[2] as u32) << 8;
                    if count == 0 || loc + count > NV_SIZE as u32 {
                        return false;
                    }
                    let buf: Vec<u8> =
                        (0..count).map(|i| d.nv[(loc + i) as usize]).collect();
                    d.data_write(0, &buf);
                    true
                }
                0x02 => {
                    // NV_POKE_BEGIN: the device stages an NV write THROUGH a
                    // RAM slot and overwrites it, so it may not be the one
                    // being served.
                    let slot = d.args[0] as usize;
                    if d.args[0] == RESERVED_AA || slot >= d.nram || slot == d.active_slot {
                        return false;
                    }
                    d.stage = Some(d.nv.clone());
                    true
                }
                0x03 => {
                    let byte = d.args[0];
                    let loc = d.args[1] as usize | (d.args[2] as usize) << 8;
                    match d.stage.as_mut() {
                        Some(st) if loc < NV_SIZE => {
                            st[loc] = byte;
                            true
                        }
                        _ => false,
                    }
                }
                0x04 => match d.stage.take() {
                    Some(st) => {
                        d.nv = st;
                        true
                    }
                    None => false,
                },
                0x05 => {
                    d.stage = None;
                    true
                }
                0x06 => {
                    // NV_POKE_COMMIT_BYTE: begin, poke and commit in one.
                    let byte = d.args[0];
                    let loc = d.args[1] as usize | (d.args[2] as usize) << 8;
                    let slot = d.args[3] as usize;
                    if d.args[3] == RESERVED_AA || slot >= d.nram || slot == d.active_slot {
                        return false;
                    }
                    if loc >= NV_SIZE {
                        return false;
                    }
                    d.nv[loc] = byte;
                    true
                }
                _ => false,
            }
        }
        _ => false,
    }
}

/// The ROM-type byte a device of this size must report.
///
/// It was hardcoded to 0x1C (28C256, 32 KB) while the model served 64 KB per
/// device, and romsel caught it: the type byte is what a host checks its
/// configured size against, so a device claiming a part half its own size is
/// refused before any write ("The device serves a different size than -s
/// says").  That refusal was correct, which is the point -- the guard exists
/// because a size taken on trust writes a partial image into every device and
/// reports success.  0xFF is the protocol's "no opinion", which leaves the
/// host's figure standing rather than contradicting it.
fn rom_type_for(size: u32) -> u8 {
    match size {
        2048 => 0x1A,   // 28C16
        8192 => 0x1B,   // 28C64
        32768 => 0x1C,  // 28C256
        65536 => 0x1D,  // 28C512
        _ => 0xFF,
    }
}

/// A 32-byte flash slot record: ROM type at 0, then a NUL-padded name.
///
/// The name starts at byte 1, not byte 8. This port put it at 8 and MacGrub
/// read it from 8, so the two agreed with each other and disagreed with the
/// device: romsel, which is the host written against real silicon, reads
/// byte 1 and duly showed every slot as "(unnamed)". A model that only ever
/// talks to the host built alongside it proves nothing.
fn slot_record(slot: usize, size: u32) -> [u8; 32] {
    let mut rec = [0u8; 32];
    rec[0] = rom_type_for(size);
    let name: &[u8] = match slot {
        0 => b"macmon",
        1 => b"Macintosh Plus ROM",
        2 => b"MacGrub",
        _ => b"slot",
    };
    let n = name.len().min(31);
    rec[1..1 + n].copy_from_slice(&name[..n]);
    rec
}

fn exec_enter(d: &mut Dev) -> bool {
    let page = d.args[0] as u16 | (d.args[1] as u16) << 8;
    let roff = d.args[2] as u32 | (d.args[3] as u32) << 8 | (d.args[4] as u32) << 16;
    let rsize = d.args[5] as u32 | (d.args[6] as u32) << 8;
    let cplt = d.args[7];
    let sok = d.args[8];

    // A reserved value for either sentinel is discarded outright: the host
    // would be unable to tell completion from non-completion.
    if cplt == RESERVED_AA || sok == RESERVED_AA {
        return false;
    }
    if roff & 3 != 0 {
        return false; // must be 4-byte aligned
    }
    if roff + HDR_SIZE > d.size {
        return false; // no room for a header
    }

    d.region_off = roff;
    d.complete = cplt;
    d.status_ok = sok;

    // The token is snapshotted BEFORE the region is initialised, because the
    // device does not initialise it and the host needs its prior value.
    d.tok_lo = d.hdr_read(HDR_TOK_LO);
    d.tok_hi = d.hdr_read(HDR_TOK_HI);

    if roff + rsize > d.size {
        // Reported rather than discarded: the host gets a failure it can see.
        d.cmd_begin(G_CONTROL, 0x01);
        d.cmd_end(false);
        return false;
    }

    d.cmd_page = page;
    d.data_size = rsize - HDR_SIZE;
    for i in 0..HDR_SIZE {
        d.hdr_write(i, 0);
    }
    d.active = true;
    true
}
