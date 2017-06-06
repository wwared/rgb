const MEM_TOP: usize = 0x10000;

pub struct Memory {
  mem: [u8; MEM_TOP],
}


pub fn new() -> Box<Memory> {
  return Box::new(Memory {
    mem: [0; MEM_TOP],
  });
}

impl Memory {
  pub fn get(&self, pos: u16) -> u8 {
    return self.mem[pos as usize];
  }

  pub fn set(&mut self, pos: u16, val: u8) {
    self.mem[pos as usize] = val;
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn memtests() {
    let mut mem = new();
    assert!(mem.get(0) == 0);
    mem.set(0, 1);
    assert!(mem.get(0) == 1);
    mem.set(0xFFFF, 0x7E);
    assert!(mem.get(0xFFFF) == 0x7E);
  }
}
