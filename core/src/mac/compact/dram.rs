//! DRAM row decay model for the compact Macs.
//!
//! Real hardware has no dedicated refresh counter: refresh is a side effect of
//! the video fetches. Each video cycle the LAG PAL muxes the video address onto
//! the RAM address lines, so scanning the framebuffer cycles the row address
//! through every value many times per frame, and the single common /RAS strobe
//! refreshes that row in all four SIMMs at once.
//!
//! The catch is that the video path sweeps only RA0..RA8. RA9 comes from the
//! B-half of the F253 at U10G, which during video selects a static control net
//! rather than a sweeping address bit, so RA9 is held at a fixed value for the
//! whole scan.
//!
//! On a 512K/1MB board (256K chips, R8 fitted) the chips have 9 row bits and
//! ignore RA9, so video refresh is complete. On a 2/2.5/4MB board (1M chips,
//! R8 removed) RA9 is a real row bit and it is A19, so **video physically
//! cannot refresh half the rows** - the half whose A19 differs from the value
//! RA9 is parked at. Those rows are refreshed only by CPU accesses.
//!
//! That is invisible under an OS, which reads all over the map, but it bites
//! bare-metal code that idles in ROM, and it bites a RAM test that walks a bank
//! linearly: such a walk flips A19 only once per 512KB, so the opposite half can
//! go a second or more without a single RAS and decay. The resulting read-back
//! mismatches look exactly like bad RAM.
//!
//! This module models that. It is OFF by default and does not affect ordinary
//! emulation; enable it to test whether a bare-metal program refreshes DRAM
//! properly. See `DramDecay::from_env`.

use serde::{Deserialize, Serialize};

/// Row-address bits swept by the video path (RA0..RA8 = A1..A9).
const VIDEO_ROW_BITS: usize = 9;
/// Rows reachable by the video sweep alone.
const VIDEO_ROWS: usize = 1 << VIDEO_ROW_BITS;
/// Total rows on a 1M-chip board, where RA9 (= A19) is also a row bit.
const TOTAL_ROWS: usize = VIDEO_ROWS * 2;

/// Smallest RAM size that implies 1M chips (R8 removed, RA9 is a row bit).
const ONE_MEG_CHIP_THRESHOLD: usize = 2 * 1024 * 1024;

/// Value RA9 is parked at during video cycles. Rows whose A19 differs from this
/// get no refresh from the video scan at all.
const VIDEO_RA9: usize = 0;

#[derive(Serialize, Deserialize)]
pub struct DramDecay {
    /// Frame number each row was last refreshed on.
    last_refresh: Vec<u32>,
    /// Frames elapsed (VBlanks seen).
    frame: u32,
    /// Frames a row may go unrefreshed before it starts losing bits.
    retention_frames: u32,
    /// 1-in-N chance per frame that a given stale row loses a bit.
    decay_odds: u32,
    /// True when the board has 1M chips, so RA9 is a row bit and video refresh
    /// covers only half the array. On 256K chips video refresh is complete.
    ra9_is_row_bit: bool,
    rng: u64,
    /// Bits flipped so far, for reporting.
    pub corrupted: u64,
}

impl DramDecay {
    /// Builds the model from the environment, or returns None if not enabled.
    ///
    /// * `SNOW_DRAM_DECAY=1`         - enable
    /// * `SNOW_DRAM_DECAY_FRAMES=60` - frames without refresh before decay
    /// * `SNOW_DRAM_DECAY_ODDS=4`    - 1-in-N chance per frame per stale row
    pub fn from_env(ram_size: usize) -> Option<Self> {
        if std::env::var("SNOW_DRAM_DECAY").ok().as_deref() != Some("1") {
            return None;
        }
        let env_num = |k: &str, d: u32| {
            std::env::var(k)
                .ok()
                .and_then(|v| v.parse::<u32>().ok())
                .filter(|v| *v > 0)
                .unwrap_or(d)
        };
        let ra9_is_row_bit = ram_size >= ONE_MEG_CHIP_THRESHOLD;
        let s = Self {
            last_refresh: vec![0; TOTAL_ROWS],
            frame: 0,
            retention_frames: env_num("SNOW_DRAM_DECAY_FRAMES", 60),
            decay_odds: env_num("SNOW_DRAM_DECAY_ODDS", 4),
            ra9_is_row_bit,
            // Fixed seed: weak cells leak the same way every run on real
            // hardware, and a reproducible failure is far more useful here.
            rng: 0x2026_0814_DEAD_BEEF,
            corrupted: 0,
        };
        log::warn!(
            "DRAM decay model ENABLED: {} chips ({} KB), retention {} frames (~{} ms), 1-in-{} per stale row per frame",
            if ra9_is_row_bit { "1M" } else { "256K" },
            ram_size / 1024,
            s.retention_frames,
            s.retention_frames * 1000 / 60,
            s.decay_odds
        );
        if !ra9_is_row_bit {
            log::warn!(
                "  256K-chip board: RA9 is not a row bit, so the video scan refreshes every row and nothing should ever decay."
            );
        }
        Some(s)
    }

    /// DRAM row selected by a CPU address: RA0..RA8 = A1..A9, RA9 = A19.
    #[inline]
    fn row_of(addr: usize) -> usize {
        ((addr >> 1) & (VIDEO_ROWS - 1)) | (((addr >> 19) & 1) << VIDEO_ROW_BITS)
    }

    /// Records that a CPU access strobed /RAS for this address' row. /RAS is
    /// common to all four SIMMs, so one access refreshes that row everywhere.
    #[inline]
    pub fn touch(&mut self, addr: usize) {
        let row = Self::row_of(addr);
        self.last_refresh[row] = self.frame;
    }

    #[inline]
    fn next_rand(&mut self) -> u64 {
        // xorshift64*
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        self.rng.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    /// Picks a random byte address belonging to `row`.
    fn addr_in_row(&mut self, row: usize, ram_mask: usize) -> usize {
        let r = self.next_rand() as usize;
        // Reconstruct an address whose row bits match, filling the column bits
        // (A0, A10..A18 and anything above A19) at random.
        let a0 = r & 1;
        let mid = (r >> 1) & 0x1FF; // A10..A18
        let high = (r >> 10) & 0x3; // A20..A21 (up to 4MB)
        let addr = a0
            | ((row & (VIDEO_ROWS - 1)) << 1)
            | (mid << 10)
            | (((row >> VIDEO_ROW_BITS) & 1) << 19)
            | (high << 20);
        addr & ram_mask
    }

    /// Called once per VBlank. Applies the video scan's partial refresh, then
    /// decays any row that has gone too long without a RAS.
    ///
    /// Returns the byte indices corrupted this frame, so the caller can mark
    /// them dirty for the display.
    pub fn vblank(&mut self, ram: &mut [u8], ram_mask: usize) -> Vec<usize> {
        self.frame = self.frame.wrapping_add(1);

        // The video scan sweeps RA0..RA8 many times per frame, but RA9 is held
        // static, so it refreshes only the rows on that side. On a 256K-chip
        // board RA9 is not a row bit at all, so every row is covered.
        for r in 0..VIDEO_ROWS {
            self.last_refresh[r | (VIDEO_RA9 << VIDEO_ROW_BITS)] = self.frame;
            if !self.ra9_is_row_bit {
                self.last_refresh[r | ((VIDEO_RA9 ^ 1) << VIDEO_ROW_BITS)] = self.frame;
            }
        }

        let mut dirty = Vec::new();
        for row in 0..TOTAL_ROWS {
            let age = self.frame.wrapping_sub(self.last_refresh[row]);
            if age < self.retention_frames {
                continue;
            }
            if (self.next_rand() % u64::from(self.decay_odds)) != 0 {
                continue;
            }
            let idx = self.addr_in_row(row, ram_mask);
            if idx >= ram.len() {
                continue;
            }
            // A leaking cell drifts to its discharged state. Model that as a
            // 0 bit reading back as 1, always the same direction, so a decayed
            // machine fails reproducibly the way real weak cells do.
            let bit = 1u8 << (self.next_rand() % 8);
            if ram[idx] & bit == 0 {
                ram[idx] |= bit;
                self.corrupted += 1;
                dirty.push(idx);
            }
        }
        if !dirty.is_empty() {
            log::debug!(
                "DRAM decay: rotted {} cell(s) this frame ({} total), e.g. {:06X}",
                dirty.len(),
                self.corrupted,
                dirty[0]
            );
        }
        dirty
    }
}
