use super::mem;

pub struct CpuState<'a> {
  a: u8,
  flags: u8,
  b: u8,
  c: u8,
  d: u8,
  e: u8,
  h: u8,
  l: u8,
  sp: u16,
  pc: u16,

  mem: &'a mut mem::Memory,
}

pub enum Flag {
  C,
  H,
  N,
  Z,
}

pub enum Operand8 {
  A,
  B,
  C,
  D,
  E,
  H,
  L,
  HL,
  Imm(u8),
}

impl Operand8 {
  pub fn get(&self, state: &CpuState) -> u8 {
    match *self {
      Operand8::A => state.a,
      Operand8::B => state.b,
      Operand8::C => state.c,
      Operand8::D => state.d,
      Operand8::E => state.e,
      Operand8::H => state.h,
      Operand8::L => state.l,
      Operand8::HL => state.mem.get(state.get_hl()),
      Operand8::Imm(i) => i,
    }
  }

  pub fn set(&self, state: &mut CpuState, value: u8) {
    match *self {
      Operand8::A => state.a = value,
      Operand8::B => state.b = value,
      Operand8::C => state.c = value,
      Operand8::D => state.d = value,
      Operand8::E => state.e = value,
      Operand8::H => state.h = value,
      Operand8::L => state.l = value,
      Operand8::HL => {
        let hl = state.get_hl();
        state.mem.set(hl, value)
      },
      Operand8::Imm(i) => state.mem.set(i as u16, value),
    };
  }
}

pub enum Operand16 {
  AF,
  BC,
  DE,
  HL,
  SP,
  PC,
  Imm(u16),
}

pub enum OpCode {
  LD8(Operand8, Operand8),
}

impl Flag {
  pub fn off(&self) -> u8 {
    match *self {
      Flag::C => 4,
      Flag::H => 5,
      Flag::N => 6,
      Flag::Z => 7,
    }
  }
}

const CLOCK_FREQ_PER_SEC: u32 = 4194304;

pub fn new<'a>(memory: &'a mut mem::Memory) -> CpuState<'a> {
  return CpuState {
    a: 0,
    flags: 0,
    b: 0,
    c: 0,
    d: 0,
    e: 0,
    h: 0,
    l: 0,
    sp: 0xFFFE,
    pc: 0x100,

    mem: memory,
  };
}

fn concat_u8(h: u8, l: u8) -> u16 {
  ((h as u16) << 8) | (l as u16)
}

impl<'a> CpuState<'a> {
  pub fn load_8bit(&mut self, left: Operand8, right: Operand8) {
    let val = left.get(self);
    right.set(self, val);
  }

  fn get_hl(&self) -> u16 {
    concat_u8(self.h, self.l)
  }

  fn get_af(&self) -> u16 {
    concat_u8(self.a, self.flags)
  }

  fn get_bc(&self) -> u16 {
    concat_u8(self.b, self.c)
  }

  fn get_de(&self) -> u16 {
    concat_u8(self.d, self.e)
  }

  fn get_flag(&self, flag: Flag) -> bool {
    return (1u8 << flag.off()) & (self.flags) != 0;
  }

  fn set_flag(&mut self, flag: Flag, value: bool) {
    self.flags = if value {
      self.flags | (1u8 << flag.off())
    } else {
      self.flags & !(1u8 << flag.off())
    };
  }
}


#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn flags() {
    let mut mem = mem::new();
    let mut cpu = new(&mut mem);
    assert!(cpu.get_flag(Flag::C) == false);
    assert!(cpu.get_flag(Flag::H) == false);
    assert!(cpu.get_flag(Flag::N) == false);
    assert!(cpu.get_flag(Flag::Z) == false);
    cpu.set_flag(Flag::C, true);
    cpu.set_flag(Flag::H, true);
    cpu.set_flag(Flag::N, true);
    cpu.set_flag(Flag::Z, true);
    assert!(cpu.get_flag(Flag::C) == true);
    assert!(cpu.get_flag(Flag::H) == true);
    assert!(cpu.get_flag(Flag::N) == true);
    assert!(cpu.get_flag(Flag::Z) == true);
    cpu.set_flag(Flag::C, false);
    cpu.set_flag(Flag::H, false);
    cpu.set_flag(Flag::N, false);
    cpu.set_flag(Flag::Z, false);
    assert!(cpu.get_flag(Flag::C) == false);
    assert!(cpu.get_flag(Flag::H) == false);
    assert!(cpu.get_flag(Flag::N) == false);
    assert!(cpu.get_flag(Flag::Z) == false);
  }
}
