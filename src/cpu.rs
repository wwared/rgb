use std::collections::HashMap;

use super::mem;

pub struct CpuState {
  a: u8,
  f: u8,
  b: u8,
  c: u8,
  d: u8,
  e: u8,
  h: u8,
  l: u8,
  sp: u16,
  pc: u16,

  mem: Box<mem::Memory>,
}

pub enum Flag {
  C,
  H,
  N,
  Z,
}

impl Flag {
  // get the offset in bits
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

lazy_static! {
  // note: returns number of cycles the operation took
  // possibly change this in the future with more useful info (what?)
  static ref OPCODES: HashMap<u8, fn(&mut CpuState) -> usize> = {
    let mut m = HashMap::<u8, fn(&mut CpuState) -> usize>::new();
    m.insert(0x00, CpuState::nop);
    m.insert(0x02, CpuState::ld_b_d8);
    m.insert(0x12, CpuState::ld_d_d8);
    m.insert(0x22, CpuState::ld_h_d8);
    m.insert(0x32, CpuState::ld_hl_d8);
    m.insert(0x0E, CpuState::ld_c_d8);
    m.insert(0x1E, CpuState::ld_e_d8);
    m.insert(0x2E, CpuState::ld_l_d8);
    m.insert(0x3E, CpuState::ld_a_d8);
    m.insert(0xC5, CpuState::push_bc);
    m.insert(0xD5, CpuState::push_de);
    m.insert(0xE5, CpuState::push_hl);
    m.insert(0xF5, CpuState::push_af);
    m.insert(0xC1, CpuState::pop_bc);
    m.insert(0xD1, CpuState::pop_de);
    m.insert(0xE1, CpuState::pop_hl);
    m.insert(0xF1, CpuState::pop_af);
    m
  };
}

pub fn new() -> CpuState {
  return CpuState {
    a: 0,
    f: 0,
    b: 0,
    c: 0,
    d: 0,
    e: 0,
    h: 0,
    l: 0,
    sp: 0xFFFE,
    pc: 0x100,

    mem: mem::new(),
  };
}

fn concat_u8(h: u8, l: u8) -> u16 {
  ((h as u16) << 8) | (l as u16)
}

fn break_u16(val: u16) -> (u8, u8) {
  let hi = (val & (0xFF << 8)) >> 8;
  let lo = val & 0xFF;
  (hi as u8, lo as u8)
}

impl CpuState {
  // so much boilerplate
  fn get_hl(&self) -> u16 {
    concat_u8(self.h, self.l)
  }

  fn set_hl(&mut self, val: u16) {
    let (h, l) = break_u16(val);
    self.h = h;
    self.l = l;
  }

  fn get_af(&self) -> u16 {
    concat_u8(self.a, self.f)
  }

  fn set_af(&mut self, val: u16) {
    let (a, f) = break_u16(val);
    self.a = a;
    self.f = f;
  }

  fn get_bc(&self) -> u16 {
    concat_u8(self.b, self.c)
  }

  fn set_bc(&mut self, val: u16) {
    let (b, c) = break_u16(val);
    self.b = b;
    self.c = c;
  }

  fn get_de(&self) -> u16 {
    concat_u8(self.d, self.e)
  }

  fn set_de(&mut self, val: u16) {
    let (d, e) = break_u16(val);
    self.d = d;
    self.e = e;
  }

  // not sure if i'll need these two fns
  fn get_flag(&self, flag: Flag) -> bool {
    return (1u8 << flag.off()) & (self.f) != 0;
  }

  fn set_flag(&mut self, flag: Flag, value: bool) {
    self.f = if value {
      self.f | (1u8 << flag.off())
    } else {
      self.f & !(1u8 << flag.off())
    };
  }

  fn push(&mut self, val: u16) {
    let (hi, lo) = break_u16(val);
    self.sp -= 2;
    self.mem.set(self.sp, hi);
    self.mem.set(self.sp+1, lo);
  }

  fn pop(&mut self) -> u16 {
    let hi = self.mem.get(self.sp);
    let lo = self.mem.get(self.sp+1);
    let val = concat_u8(hi, lo);
    self.sp += 2;
    return val;
  }

  fn step(&mut self) {
    let next_opcode = self.mem.get(self.pc);
    // crashes on unknown opcode, just like a real console!
    let opcode_fn = OPCODES.get(&next_opcode).unwrap();
    let elapsed_cycles = opcode_fn(self);
    println!("opcode: {} took {} cycles", next_opcode, elapsed_cycles);
  }

  //
  // Opcode implementations follow
  //

  fn nop(&mut self) -> usize {
    println!("NOP");
    self.pc += 1;
    return 4;
  }

  fn ld_b_d8(&mut self) -> usize {
    let imm = self.mem.get(self.pc+1);
    println!("LD B, d8({})", imm);
    self.b = imm;
    self.pc += 2;
    return 8;
  }

  fn ld_d_d8(&mut self) -> usize {
    let imm = self.mem.get(self.pc+1);
    println!("LD D, d8({})", imm);
    self.d = imm;
    self.pc += 2;
    return 8;
  }

  fn ld_h_d8(&mut self) -> usize {
    let imm = self.mem.get(self.pc+1);
    println!("LD H, d8({})", imm);
    self.h = imm;
    self.pc += 2;
    return 8;
  }

  fn ld_hl_d8(&mut self) -> usize {
    let imm = self.mem.get(self.pc+1);
    println!("LD (HL), d8({})", imm);
    let hl = self.get_hl();
    self.mem.set(hl, imm);
    self.pc += 2;
    return 12;
  }

  fn ld_c_d8(&mut self) -> usize {
    let imm = self.mem.get(self.pc+1);
    println!("LD C, d8({})", imm);
    self.c = imm;
    self.pc += 2;
    return 8;
  }

  fn ld_e_d8(&mut self) -> usize {
    let imm = self.mem.get(self.pc+1);
    println!("LD E, d8({})", imm);
    self.e = imm;
    self.pc += 2;
    return 8;
  }

  fn ld_l_d8(&mut self) -> usize {
    let imm = self.mem.get(self.pc+1);
    println!("LD L, d8({})", imm);
    self.l = imm;
    self.pc += 2;
    return 8;
  }

  fn ld_a_d8(&mut self) -> usize {
    let imm = self.mem.get(self.pc+1);
    println!("LD A, d8({})", imm);
    self.a = imm;
    self.pc += 2;
    return 8;
  }

  fn push_bc(&mut self) -> usize {
    let val = self.get_bc();
    println!("PUSH BC({})", val);
    self.push(val);
    self.pc += 1;
    return 16;
  }

  fn push_de(&mut self) -> usize {
    let val = self.get_de();
    println!("PUSH DE({})", val);
    self.push(val);
    self.pc += 1;
    return 16;
  }

  fn push_hl(&mut self) -> usize {
    let val = self.get_hl();
    println!("PUSH HL({})", val);
    self.push(val);
    self.pc += 1;
    return 16;
  }

  fn push_af(&mut self) -> usize {
    let val = self.get_af();
    println!("PUSH AF({})", val);
    self.push(val);
    self.pc += 1;
    return 16;
  }

  fn pop_bc(&mut self) -> usize {
    let val = self.pop();
    println!("POP BC({})", val);
    self.set_bc(val);
    self.pc += 1;
    return 12;
  }

  fn pop_de(&mut self) -> usize {
    let val = self.pop();
    println!("POP DE({})", val);
    self.set_de(val);
    self.pc += 1;
    return 12;
  }

  fn pop_hl(&mut self) -> usize {
    let val = self.pop();
    println!("POP HL({})", val);
    self.set_hl(val);
    self.pc += 1;
    return 12;
  }

  fn pop_af(&mut self) -> usize {
    let val = self.pop();
    println!("POP AF({})", val);
    self.set_af(val);
    self.pc += 1;
    return 12;
  }
}


#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn nop() {
    let mut cpu = new();
    cpu.mem.set(cpu.pc, 0);
    cpu.step();
    // would be nice to assert we actually called a nop but w/e
  }

  #[test]
  fn flags() {
    let mut cpu = new();
    assert_eq!(cpu.get_flag(Flag::C), false);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::Z), false);
    cpu.set_flag(Flag::C, true);
    cpu.set_flag(Flag::H, true);
    cpu.set_flag(Flag::N, true);
    cpu.set_flag(Flag::Z, true);
    assert_eq!(cpu.get_flag(Flag::C), true);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::N), true);
    assert_eq!(cpu.get_flag(Flag::Z), true);
    cpu.set_flag(Flag::C, false);
    cpu.set_flag(Flag::H, false);
    cpu.set_flag(Flag::N, false);
    cpu.set_flag(Flag::Z, false);
    assert_eq!(cpu.get_flag(Flag::C), false);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::Z), false);
  }

  #[test]
  fn concat_test() {
    assert_eq!(concat_u8(0, 0), 0);
    assert_eq!(concat_u8(0xFF, 0), 0xFF00);
    assert_eq!(concat_u8(0, 0xFF), 0xFF);
    assert_eq!(concat_u8(0xFF, 0xFF), 0xFFFF);
    assert_eq!(concat_u8(0x12, 0x34), 0x1234);
  }

  #[test]
  fn break_test() {
    assert_eq!(break_u16(0xFFFF), (0xFF, 0xFF));
    assert_eq!(break_u16(0x1234), (0x12, 0x34));
    assert_eq!(break_u16(0), (0, 0));
    assert_eq!(break_u16(0xFF), (0, 0xFF));
    assert_eq!(break_u16(0xFF00), (0xFF, 0));
  }
}
