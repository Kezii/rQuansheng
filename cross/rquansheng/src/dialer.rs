use heapless::String;

use crate::keyboard::{Key, KeyEvent};

#[derive(Clone)]
pub struct Dialer {
    number: String<6>,
}

impl Default for Dialer {
    fn default() -> Self {
        Self {
            number: String::<6>::new(),
        }
    }
}

impl Dialer {
    pub fn eat_keyboard_event(&mut self, event: KeyEvent) {
        if let KeyEvent::KeyPressed(key) = event {
            if let Some(ch) = key.as_ascii_char() {
                self.number.push(ch).ok();
            }
        }
    }

    pub fn get_as_string(&self) -> String<6> {
        self.number.clone()
    }

    pub fn clear(&mut self) {
        self.number.clear();
    }

    pub fn get_frequency(&mut self) -> Option<u32> {
        if self.number.len() != 6 {
            return None;
        }
        let mut frequency = 0;
        for (i, ch) in self.number.as_str().chars().enumerate() {
            frequency = frequency + (ch as u32 - '0' as u32) * 10_u32.pow(5 - i as u32);
        }

        self.clear();
        Some(frequency)
    }
}
