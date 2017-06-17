use mem;
use mem::MemoryBlock;
use mem::RAM;
use mem::ROM;

pub struct Registers {
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
}

pub struct CPU {
  regs: Registers,

  mem: RAM,
}

#[derive(Copy, Clone, Debug)]
pub enum Reg8 {
  A = 0,
  B,
  C,
  D,
  E,
  H,
  L,
  F,
  MemHL,
}

#[derive(Copy, Clone, Debug)]
pub enum Reg16 {
  AF = 0,
  BC,
  DE,
  HL,
  SP,
  PC,
}

#[derive(Copy, Clone, Debug)]
pub enum Flag {
  Carry = 1 << 4,
  H = 1 << 5, // BCD flags
  N = 1 << 6,
  Zero = 1 << 7,
}

// used for changing values of registers in instructions
#[derive(Copy, Clone, Debug)]
pub enum InstrFlag {
  None = 0,
  Dec,
  Inc,
}

#[derive(Copy, Clone, Debug)]
pub enum Instruction {
  Nop(),
  Stop(),
  Halt(),
  DisableInterrupts(),
  EnableInterrupts(),

  Jr(i8),
  JrFlag(i8, Flag, bool),
  Jp(u16),
  JpHL(),
  JpFlag(u16, Flag, bool),
  Call(u16),
  CallFlag(u16, Flag, bool),
  Ret(),
  RetInterrupt(),
  RetFlag(Flag, bool),

  AddReg8(Reg8),
  AddCarryReg8(Reg8),
  AddImm8(u8),
  AddCarryImm8(u8),
  IncReg8(Reg8),
  IncReg16(Reg16),
  SubReg8(Reg8),
  SubCarryReg8(Reg8),
  SubImm8(u8),
  SubCarryImm8(u8),
  Compare(Reg8),
  CompareImm8(u8),
  DecReg8(Reg8),
  DecReg16(Reg16),
  AndReg8(Reg8),
  AndImm8(u8),
  XorReg8(Reg8),
  XorImm8(u8),
  OrReg8(Reg8),
  OrImm8(u8),

  LoadReg8(Reg8, Reg8),
  LoadImm8(Reg8, u8),
  LoadImm16(Reg16, u16),
  WriteA(Reg16, InstrFlag),
  WriteAImm16(u16),
  ReadA(Reg16, InstrFlag),
  ReadAImm16(u16),
  WriteMemSP(u16),
  HiLoad(u8),
  HiLoadReg(),
  HiWrite(u8),
  HiWriteReg(),

  AddHL(Reg16),
  LoadSPOffset(i8),
  SwapSPHL(),

  Pop(Reg16),
  Push(Reg16),
  AddSP(i8),

  Rlca(),
  Rla(),
  Rrca(),
  Rra(),
  Rlc(Reg8),
  Rrc(Reg8),
  Rl(Reg8),
  Rr(Reg8),
  Sla(Reg8),
  Sra(Reg8),
  Swap(Reg8),
  Srl(Reg8),
  TestBit(u8, Reg8), // note: 0-7 only
  SetBit(u8, Reg8, bool),

  Complement(),
  ComplementCarry(),

  Daa(),
  SetCarryFlag(),

  Restart(u8),
}

use self::Instruction::*;

impl Instruction {
  fn size(i: Instruction) -> u16 {
    match i {
      WriteMemSP(_) | Jp(_) | JpFlag(_, _, _) |
        Call(_) | CallFlag(_, _, _) | ReadAImm16(_) |
        WriteAImm16(_) | LoadImm16(_, _) => 3,
      Jr(_) | JrFlag(_, _, _) | HiLoad(_) | HiWrite(_) |
        AddImm8(_) | SubImm8(_) | AndImm8(_) | OrImm8(_) |
        LoadImm8(_, _) | AddSP(_) | LoadSPOffset(_) | AddCarryImm8(_) |
        SubCarryImm8(_) | XorImm8(_) | CompareImm8(_) |
        Rlc(_) | Rrc(_) | Rl(_) | Rr(_) | Sla(_) | Sra(_) |
        Swap(_) | Srl(_) | TestBit(_, _) | SetBit(_, _, _) => 2,
      _ => 1,
    }
  }
}

// TODO: unused
//const CLOCK_FREQ_PER_SEC: u32 = 4194304;

fn concat_u8(h: u8, l: u8) -> u16 {
  ((h as u16) << 8) | (l as u16)
}

fn break_u16(val: u16) -> (u8, u8) {
  let hi = (val & (0xFF << 8)) >> 8;
  let lo = val & 0xFF;
  (hi as u8, lo as u8)
}

impl CPU {
  pub fn new() -> CPU {
    let mut mem = Vec::with_capacity(mem::MEM_TOP);
    mem.resize(mem::MEM_TOP, 0);
    CPU {
      regs: Registers {
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
      },

      mem: RAM { addr: 0, v: mem },
    }
  }

  fn get_reg8(&self, r: Reg8) -> u8 {
    match r {
      Reg8::A => self.regs.a,
      Reg8::B => self.regs.b,
      Reg8::C => self.regs.c,
      Reg8::D => self.regs.d,
      Reg8::E => self.regs.e,
      Reg8::H => self.regs.h,
      Reg8::L => self.regs.l,
      Reg8::F => self.regs.f,
      Reg8::MemHL => self.read8(self.get_reg16(Reg16::HL)),
    }
  }

  fn set_reg8(&mut self, r: Reg8, val: u8) {
    match r {
      Reg8::A => self.regs.a = val,
      Reg8::B => self.regs.b = val,
      Reg8::C => self.regs.c = val,
      Reg8::D => self.regs.d = val,
      Reg8::E => self.regs.e = val,
      Reg8::H => self.regs.h = val,
      Reg8::L => self.regs.l = val,
      Reg8::F => self.regs.f = val,
      Reg8::MemHL => {
        let pos = self.get_reg16(Reg16::HL);
        self.write8(pos, val);
      }
    }
  }

  fn get_reg16(&self, r: Reg16) -> u16 {
    match r {
      Reg16::AF => concat_u8(self.regs.a, self.regs.f),
      Reg16::BC => concat_u8(self.regs.b, self.regs.c),
      Reg16::DE => concat_u8(self.regs.d, self.regs.e),
      Reg16::HL => concat_u8(self.regs.h, self.regs.l),
      Reg16::SP => self.regs.sp,
      Reg16::PC => self.regs.pc,
    }
  }

  fn set_reg16(&mut self, r: Reg16, val: u16) {
    match r {
      Reg16::AF => {
        let (a, f) = break_u16(val);
        self.regs.a = a; self.regs.f = f;
      },
      Reg16::BC => {
        let (b, c) = break_u16(val);
        self.regs.b = b; self.regs.c = c;
      },
      Reg16::DE => {
        let (d, e) = break_u16(val);
        self.regs.d = d; self.regs.e = e;
      },
      Reg16::HL => {
        let (h, l) = break_u16(val);
        self.regs.h = h; self.regs.l = l;
      },
      Reg16::SP => { self.regs.sp = val },
      Reg16::PC => { self.regs.pc = val },
    }
  }

  // TODO: implement memory mapping
  fn read8(&self, pos: u16) -> u8 {
    self.mem.read8(pos)
  }

  fn write8(&mut self, pos: u16, val: u8) {
    self.mem.write8(pos, val);
  }

  fn read16(&self, pos: u16) -> u16 {
    let h = self.read8(pos);
    let l = self.read8(pos.wrapping_add(1));
    concat_u8(h, l)
  }

  fn write16(&mut self, pos: u16, val: u16) {
    let (h, l) = break_u16(val);
    self.write8(pos, h);
    self.write8(pos.wrapping_add(1), l);
  }

  // not sure if i'll need these two fns
  fn get_flag(&self, flag: Flag) -> bool {
    let f = flag as u8;
    f & (self.regs.f) != 0
  }

  fn set_flag(&mut self, flag: Flag, value: bool) {
    let f = flag as u8;
    self.regs.f = if value { self.regs.f | f } else { self.regs.f & !f };
  }

  fn push(&mut self, val: u16) {
    self.regs.sp -= 2;
    let p = self.regs.sp;
    self.write16(p, val);
  }

  fn pop(&mut self) -> u16 {
    let val = self.read16(self.regs.sp);
    self.regs.sp += 2;
    val
  }

  fn step(&mut self) {
    let next = self.decode_next();
    let cycles = self.run(next);
  }

  fn opcode_u8(&self) -> u8 {
    self.read8(self.regs.pc+1)
  }

  fn opcode_i8(&self) -> i8 {
    self.opcode_u8() as i8
  }

  fn opcode_u16(&self) -> u16 {
    self.read16(self.regs.pc+1)
  }

  fn decode_next(&mut self) -> Instruction {
    let opcode = self.read8(self.regs.pc);

    let reg8: [Reg8; 8] = [Reg8::B, Reg8::C, Reg8::D, Reg8::E, Reg8::H, Reg8::L, Reg8::MemHL, Reg8::A];
    let reg16: [Reg16; 4] = [Reg16::BC, Reg16::DE, Reg16::HL, Reg16::SP];

    // TODO: complete
    match opcode {
      0x00 => Nop(),
      0x10 => Stop(),

      0x20 => JrFlag(self.opcode_i8(), Flag::Zero, false),
      0x28 => JrFlag(self.opcode_i8(), Flag::Zero, true),
      0x30 => JrFlag(self.opcode_i8(), Flag::Carry, false),
      0x38 => JrFlag(self.opcode_i8(), Flag::Carry, true),

      0xC0 => RetFlag(Flag::Zero, false),
      0xC8 => RetFlag(Flag::Zero, true),
      0xD0 => RetFlag(Flag::Carry, false),
      0xD8 => RetFlag(Flag::Carry, true),

      0xC9 => Ret(),
      0xD9 => RetInterrupt(),

      0xC2 => JpFlag(self.opcode_u16(), Flag::Zero, false),
      0xCA => JpFlag(self.opcode_u16(), Flag::Zero, true),
      0xD2 => JpFlag(self.opcode_u16(), Flag::Carry, false),
      0xDA => JpFlag(self.opcode_u16(), Flag::Carry, true),

      0xC4 => CallFlag(self.opcode_u16(), Flag::Zero, false),
      0xCC => CallFlag(self.opcode_u16(), Flag::Zero, true),
      0xD4 => CallFlag(self.opcode_u16(), Flag::Carry, false),
      0xDC => CallFlag(self.opcode_u16(), Flag::Carry, true),

      0xCD => Call(self.opcode_u16()),

      0xC3 => Jp(self.opcode_u16()),
      0xE9 => JpHL(),

      0x18 => Jr(self.opcode_i8()),

      0xE0 => HiWrite(self.opcode_u8()),
      0xF0 => HiLoad(self.opcode_u8()),
      0xE2 => HiWriteReg(),
      0xF2 => HiLoadReg(),

      0x08 => WriteMemSP(self.opcode_u16()),
      0xE8 => AddSP(self.opcode_i8()),
      0xF8 => LoadSPOffset(self.opcode_i8()),
      0xF9 => SwapSPHL(),

      0x09 | 0x19 | 0x29 | 0x39 => AddHL(reg16[((opcode & 0xf0) >> 4) as usize]),

      0x80 ... 0x87 => AddReg8(reg8[(opcode & 0x0f) as usize]),
      0x88 ... 0x8F => AddCarryReg8(reg8[(opcode & 0x0f - 8) as usize]),
      0x90 ... 0x97 => SubReg8(reg8[(opcode & 0x0f) as usize]),
      0x98 ... 0x9F => SubCarryReg8(reg8[(opcode & 0x0f - 8) as usize]),
      0xA0 ... 0xA7 => AndReg8(reg8[(opcode & 0x0f) as usize]),
      0xA8 ... 0xAF => XorReg8(reg8[(opcode & 0x0f - 8) as usize]),
      0xB0 ... 0xB7 => OrReg8(reg8[(opcode & 0x0f) as usize]),
      0xB8 ... 0xBF => Compare(reg8[(opcode & 0x0f - 8) as usize]),

      0xC6 => AddImm8(self.opcode_u8()),
      0xD6 => SubImm8(self.opcode_u8()),
      0xE6 => AndImm8(self.opcode_u8()),
      0xF6 => OrImm8(self.opcode_u8()),
      0xCE => AddCarryImm8(self.opcode_u8()),
      0xDE => SubCarryImm8(self.opcode_u8()),
      0xEE => XorImm8(self.opcode_u8()),
      0xFE => CompareImm8(self.opcode_u8()),

      0x01 | 0x11 | 0x21 | 0x31 => LoadImm16(reg16[((opcode & 0xf0) >> 4) as usize], self.opcode_u16()),

      0x0A | 0x1A => ReadA(reg16[((opcode & 0xf0) >> 4) as usize], InstrFlag::None),
      0x2A => ReadA(Reg16::HL, InstrFlag::Inc), 
      0x3A => ReadA(Reg16::HL, InstrFlag::Dec), 

      0x02 | 0x12 => WriteA(reg16[((opcode & 0xf0) >> 4) as usize], InstrFlag::None),
      0x22 => WriteA(Reg16::HL, InstrFlag::Inc), 
      0x32 => WriteA(Reg16::HL, InstrFlag::Dec), 

      0xEA => WriteAImm16(self.opcode_u16()),
      0xFA => ReadAImm16(self.opcode_u16()),

      0xC1 | 0xD1 | 0xE1 | 0xF1 => Pop(reg16[(((opcode & 0xf0) >> 4) - 12) as usize]),
      0xC5 | 0xD5 | 0xE5 | 0xF5 => Push(reg16[(((opcode & 0xf0) >> 4) - 12) as usize]),

      0x40 ... 0x47 => LoadReg8(Reg8::B, reg8[(opcode & 0x0f) as usize]),
      0x48 ... 0x4F => LoadReg8(Reg8::C, reg8[(opcode & 0x0f - 8) as usize]),
      0x50 ... 0x57 => LoadReg8(Reg8::D, reg8[(opcode & 0x0f) as usize]),
      0x58 ... 0x5F => LoadReg8(Reg8::E, reg8[(opcode & 0x0f - 8) as usize]),
      0x60 ... 0x67 => LoadReg8(Reg8::H, reg8[(opcode & 0x0f) as usize]),
      0x68 ... 0x6F => LoadReg8(Reg8::L, reg8[(opcode & 0x0f - 8) as usize]),
      0x70 ... 0x75 | 0x77 => LoadReg8(Reg8::MemHL, reg8[(opcode & 0x0f) as usize]),
      0x78 ... 0x7F => LoadReg8(Reg8::A, reg8[(opcode & 0x0f - 8) as usize]),

      0x03 | 0x13 | 0x23 | 0x33 => IncReg16(reg16[((opcode & 0xf0) >> 4) as usize]),

      0x0B | 0x1B | 0x2B | 0x3B => DecReg16(reg16[((opcode & 0xf0) >> 4) as usize]),

      0x04 | 0x14 | 0x24 | 0x34 => IncReg8(reg8[(((opcode & 0xf0) >> 4) * 2) as usize]),
      0x0C | 0x1C | 0x2C | 0x3C => IncReg8(reg8[(((opcode & 0xf0) >> 4) * 2 + 1) as usize]),

      0x05 | 0x15 | 0x25 | 0x35 => DecReg8(reg8[(((opcode & 0xf0) >> 4) * 2) as usize]),
      0x0D | 0x1D | 0x2D | 0x3D => DecReg8(reg8[(((opcode & 0xf0) >> 4) * 2 + 1) as usize]),

      0x06 | 0x16 | 0x26 | 0x36 => LoadImm8(reg8[(((opcode & 0xf0) >> 4) * 2) as usize], self.opcode_u8()),
      0x0E | 0x1E | 0x2E | 0x3E => LoadImm8(reg8[(((opcode & 0xf0) >> 4) * 2 + 1) as usize], self.opcode_u8()),

      0x76 => Halt(),

      0xF3 => DisableInterrupts(),
      0xFB => EnableInterrupts(),

      0x07 => Rlca(),
      0x17 => Rla(),
      0x27 => Daa(),
      0x37 => SetCarryFlag(),
      0x0F => Rrca(),
      0x1F => Rra(),
      0x2F => Complement(),
      0x3F => ComplementCarry(),

      0xC7 => Restart(0x00),
      0xD7 => Restart(0x10),
      0xE7 => Restart(0x20),
      0xF7 => Restart(0x30),
      0xCF => Restart(0x08),
      0xDF => Restart(0x18),
      0xEF => Restart(0x28),
      0xFF => Restart(0x38),

      0xCB => {
        let opcode2 = self.opcode_u8();
        match opcode2 {
          0x00 ... 0x07 => Rlc(reg8[(opcode2 & 0x0f) as usize]),
          0x08 ... 0x0F => Rrc(reg8[(opcode2 & 0x0f - 8) as usize]),

          0x10 ... 0x17 => Rl(reg8[(opcode2 & 0x0f) as usize]),
          0x18 ... 0x1F => Rr(reg8[(opcode2 & 0x0f - 8) as usize]),

          0x20 ... 0x27 => Sla(reg8[(opcode2 & 0x0f) as usize]),
          0x28 ... 0x2F => Sra(reg8[(opcode2 & 0x0f - 8) as usize]),

          0x30 ... 0x37 => Swap(reg8[(opcode2 & 0x0f) as usize]),
          0x38 ... 0x3F => Srl(reg8[(opcode2 & 0x0f - 8) as usize]),

          0x40 ... 0x47 => TestBit(0, reg8[(opcode2 & 0x0f) as usize]),
          0x48 ... 0x4F => TestBit(1, reg8[(opcode2 & 0x0f - 8) as usize]),
          0x50 ... 0x57 => TestBit(2, reg8[(opcode2 & 0x0f) as usize]),
          0x58 ... 0x5F => TestBit(3, reg8[(opcode2 & 0x0f - 8) as usize]),
          0x60 ... 0x67 => TestBit(4, reg8[(opcode2 & 0x0f) as usize]),
          0x68 ... 0x6F => TestBit(5, reg8[(opcode2 & 0x0f - 8) as usize]),
          0x70 ... 0x77 => TestBit(6, reg8[(opcode2 & 0x0f) as usize]),
          0x78 ... 0x7F => TestBit(7, reg8[(opcode2 & 0x0f - 8) as usize]),

          0x80 ... 0x87 => SetBit(0, reg8[(opcode2 & 0x0f) as usize], false),
          0x88 ... 0x8F => SetBit(1, reg8[(opcode2 & 0x0f - 8) as usize], false),
          0x90 ... 0x97 => SetBit(2, reg8[(opcode2 & 0x0f) as usize], false),
          0x98 ... 0x9F => SetBit(3, reg8[(opcode2 & 0x0f - 8) as usize], false),
          0xA0 ... 0xA7 => SetBit(4, reg8[(opcode2 & 0x0f) as usize], false),
          0xA8 ... 0xAF => SetBit(5, reg8[(opcode2 & 0x0f - 8) as usize], false),
          0xB0 ... 0xB7 => SetBit(6, reg8[(opcode2 & 0x0f) as usize], false),
          0xB8 ... 0xBF => SetBit(7, reg8[(opcode2 & 0x0f - 8) as usize], false),

          0xC0 ... 0xC7 => SetBit(0, reg8[(opcode2 & 0x0f) as usize], true),
          0xC8 ... 0xCF => SetBit(1, reg8[(opcode2 & 0x0f - 8) as usize], true),
          0xD0 ... 0xD7 => SetBit(2, reg8[(opcode2 & 0x0f) as usize], true),
          0xD8 ... 0xDF => SetBit(3, reg8[(opcode2 & 0x0f - 8) as usize], true),
          0xE0 ... 0xE7 => SetBit(4, reg8[(opcode2 & 0x0f) as usize], true),
          0xE8 ... 0xEF => SetBit(5, reg8[(opcode2 & 0x0f - 8) as usize], true),
          0xF0 ... 0xF7 => SetBit(6, reg8[(opcode2 & 0x0f) as usize], true),
          0xF8 ... 0xFF => SetBit(7, reg8[(opcode2 & 0x0f - 8) as usize], true),

          _ => {
            panic!("Unhandled 0xCB opcode {:?}", opcode2);
          }
        }
      },
      
      _ => {
        panic!("Unknown opcode {:?}", opcode);
      },
    }
  }

  fn run(&mut self, instr: Instruction) -> usize {
    5
  }

  //
  // Opcode implementations follow
  //

}


#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn nop() {
    let mut cpu = CPU::new();
    cpu.mem.write8(cpu.regs.pc, 0);
    cpu.step();
    // would be nice to assert we actually called a nop but w/e
  }

  #[test]
  fn flags() {
    let mut cpu = CPU::new();
    assert_eq!(cpu.get_flag(Flag::Carry), false);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::Zero), false);
    cpu.set_flag(Flag::Carry, true);
    cpu.set_flag(Flag::H, true);
    cpu.set_flag(Flag::N, true);
    cpu.set_flag(Flag::Zero, true);
    assert_eq!(cpu.get_flag(Flag::Carry), true);
    assert_eq!(cpu.get_flag(Flag::H), true);
    assert_eq!(cpu.get_flag(Flag::N), true);
    assert_eq!(cpu.get_flag(Flag::Zero), true);
    cpu.set_flag(Flag::Carry, false);
    cpu.set_flag(Flag::H, false);
    cpu.set_flag(Flag::N, false);
    cpu.set_flag(Flag::Zero, false);
    assert_eq!(cpu.get_flag(Flag::Carry), false);
    assert_eq!(cpu.get_flag(Flag::H), false);
    assert_eq!(cpu.get_flag(Flag::N), false);
    assert_eq!(cpu.get_flag(Flag::Zero), false);
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
