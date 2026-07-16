use serde_big_array::BigArray;
use std::collections::VecDeque;

use anyhow::Result;
use log::*;
use serde::{Deserialize, Serialize};

use crate::keymap::{KeyEvent, Keymap};

/// Apple M0110 keyboard, for the 512K/Plus
#[derive(Serialize, Deserialize)]
pub struct PlusKeyboard {
    event_queue: VecDeque<KeyEvent>,

    #[serde(with = "BigArray")]
    keystate: [bool; 256],
}

impl Default for PlusKeyboard {
    fn default() -> Self {
        Self {
            event_queue: Default::default(),
            keystate: [false; 256],
        }
    }
}

// Scancodes
pub const SC_BACKSPACE: u8 = 0x67;
pub const SC_TAB: u8 = 0x61;
pub const SC_CAPSLOCK: u8 = 0x73;
pub const SC_RETURN: u8 = 0x49;
pub const SC_SHIFT: u8 = 0x71;
pub const SC_OPTION: u8 = 0x75;
pub const SC_APPLE: u8 = 0x6F;
pub const SC_SPACE: u8 = 0x63;

impl PlusKeyboard {
    const KEYMAP: Keymap = Keymap::AkM0110;

    pub fn pending_events(&self) -> bool {
        !self.event_queue.is_empty()
    }

    pub fn event(&mut self, ev: KeyEvent) {
        self.event_queue.push_back(ev);
    }

    pub fn release_all(&mut self) {
        for (sc, _) in self.keystate.iter().enumerate().filter(|(_, s)| **s) {
            self.event_queue
                .push_back(KeyEvent::KeyUp(sc.try_into().unwrap(), Self::KEYMAP));
        }
    }

    /// Process a keyboard command.  Returns the response byte, or `None` for an
    /// Inquiry ($10) with no key event queued -- the real M0110 HOLDS an Inquiry
    /// (does not clock a response back) until a key event or ~250 ms elapses, so
    /// the caller must model that hold.  Instant ($14) always answers at once.
    pub fn cmd(&mut self, cmd: u8) -> Result<Option<u8>> {
        match cmd {
            // Inquiry / Instant
            0x10 | 0x14 => {
                if let Some(ev) = self
                    .event_queue
                    .pop_front()
                    .and_then(|ke| ke.translate_scancode(Self::KEYMAP))
                {
                    let result = match ev {
                        KeyEvent::KeyDown(sc, _) => {
                            self.keystate[sc as usize] = true;
                            sc
                        }
                        KeyEvent::KeyUp(sc, _) => {
                            self.keystate[sc as usize] = false;
                            0x80 | sc
                        }
                    };
                    Ok(Some(result | 0x01))
                } else if cmd == 0x10 {
                    // Inquiry with nothing queued: hold (no response yet).
                    Ok(None)
                } else {
                    // Instant: null immediately.
                    Ok(Some(0x7B))
                }
            }
            // Model
            0x16 => {
                // US layout
                info!("Keyboard reset");
                Ok(Some(3))
            }
            // Test
            0x36 => Ok(Some(0x7D)),
            _ => {
                warn!("Unknown keyboard command ${:02X}", cmd);
                Ok(Some(0))
            }
        }
    }
}
