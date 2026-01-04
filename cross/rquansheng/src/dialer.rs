use heapless::String;

use crate::keyboard::{KeyEvent, QuanshengKey};

#[derive(Clone)]
pub struct Dialer<const N: usize> {
    number: String<N>,
}

impl<const N: usize> Default for Dialer<N> {
    fn default() -> Self {
        Self {
            number: String::<N>::new(),
        }
    }
}

impl<const N: usize> Dialer<N> {
    pub fn eat_keyboard_event(&mut self, event: KeyEvent) {
        if let KeyEvent::KeyPressed(key) = event {
            if let Some(ch) = key.as_ascii_char() {
                self.number.push(ch).ok();
            }
        }
    }

    pub fn get_as_string(&self) -> String<N> {
        self.number.clone()
    }

    pub fn is_dialing(&self) -> bool {
        !self.number.is_empty() && self.number.len() < N
    }

    pub fn clear(&mut self) {
        self.number.clear();
    }

    pub fn get_frequency(&mut self) -> Option<u32> {
        if self.number.len() != N {
            return None;
        }
        let mut frequency = 0;
        for (i, ch) in self.number.as_str().chars().enumerate() {
            frequency += (ch as u32 - '0' as u32) * 10_u32.pow(5 - i as u32);
        }

        self.clear();
        Some(frequency)
    }
}
