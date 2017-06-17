pub const MEM_TOP: usize = 0x10000;

pub trait MemoryBlock {
  fn read8(&self, pos: u16) -> u8 {
    panic!("Unhandled read at {:?}", pos);
  }

  fn write8(&mut self, pos: u16, val: u8) {
    panic!("Unhandled write at {:?} ({:?})", pos, val);
  }
}

pub struct RAM {
  pub addr: usize,
  pub v: Vec<u8>,
}

impl MemoryBlock for RAM {
  fn read8(&self, pos: u16) -> u8 {
    self.v[pos as usize - self.addr]
  }

  fn write8(&mut self, pos: u16, val: u8) {
    self.v[pos as usize - self.addr] = val;
  }
}

pub struct ROM {
  pub addr: usize,
  pub v: Vec<u8>,
}

impl MemoryBlock for ROM {
  fn read8(&self, pos: u16) -> u8 {
    self.v[pos as usize - self.addr]
  }

  fn write8(&mut self, _pos: u16, _val: u8) {
    /* Writes to ROM are silently ignored */
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  fn vec(size: usize) -> Vec<u8> {
    let mut v = Vec::with_capacity(size);
    v.resize(size, 0);
    v
  }

  #[test]
  #[should_panic]
  fn bad_read() {
    let ram = RAM {
      addr: 0,
      v: vec(0xFF),
    };
    ram.read8(0xFF);
  }

  #[test]
  #[should_panic]
  fn bad_write() {
    let mut ram = RAM {
      addr: 0,
      v: vec(0xFF),
    };
    ram.write8(0xFF, 0xAB);
  }

  #[test]
  fn memtests() {
    let (mut ram, mut rom) = (
      RAM {
        addr: 0x50,
        v: vec(0x100),
      },
      ROM {
        addr: 0x150,
        v: vec(0x100),
      },
    );
    assert_eq!(ram.read8(0x55), 0);
    assert_eq!(rom.read8(0x155), 0);
    ram.write8(0x149, 0xFF);
    assert_eq!(ram.read8(0x149), 0xFF);
    rom.write8(0x150, 0xFF);
    assert_eq!(rom.read8(0x150), 0);
  }
}
