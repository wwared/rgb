use std::num::Wrapping as W;

use mem;
use mem::MemoryBlock;
use mem::RAM;
use mem::ROM;

pub struct Registers {
  a: W<u8>,
  f: W<u8>,
  b: W<u8>,
  c: W<u8>,
  d: W<u8>,
  e: W<u8>,
  h: W<u8>,
  l: W<u8>,
  sp: W<u16>,
  pc: W<u16>,
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

  Jr(W<i8>),
  JrFlag(W<i8>, Flag, bool),
  Jp(W<u16>),
  JpHL(),
  JpFlag(W<u16>, Flag, bool),
  Call(W<u16>),
  CallFlag(W<u16>, Flag, bool),
  Ret(),
  RetInterrupt(),
  RetFlag(Flag, bool),

  AddReg8(Reg8),
  AddCarryReg8(Reg8),
  AddImm8(W<u8>),
  AddCarryImm8(W<u8>),
  IncReg8(Reg8),
  IncReg16(Reg16),
  SubReg8(Reg8),
  SubCarryReg8(Reg8),
  SubImm8(W<u8>),
  SubCarryImm8(W<u8>),
  Compare(Reg8),
  CompareImm8(W<u8>),
  DecReg8(Reg8),
  DecReg16(Reg16),
  AndReg8(Reg8),
  AndImm8(W<u8>),
  XorReg8(Reg8),
  XorImm8(W<u8>),
  OrReg8(Reg8),
  OrImm8(W<u8>),

  LoadReg8(Reg8, Reg8),
  LoadImm8(Reg8, W<u8>),
  LoadImm16(Reg16, W<u16>),
  WriteA(Reg16, InstrFlag),
  WriteAImm16(W<u16>),
  ReadA(Reg16, InstrFlag),
  ReadAImm16(W<u16>),
  WriteMemSP(W<u16>),
  HiLoad(W<u8>),
  HiLoadReg(),
  HiWrite(W<u8>),
  HiWriteReg(),

  AddHL(Reg16),
  LoadSPOffset(W<i8>),
  SwapSPHL(),

  Pop(Reg16),
  Push(Reg16),
  AddSP(W<i8>),

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

  Restart(W<u8>),

  UnknownOpcode(),
}

use self::Instruction::*;

impl Instruction {
  fn size(&self) -> u16 {
    match *self {
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

fn concat_u8(h: W<u8>, l: W<u8>) -> W<u16> {
  (extend_u8(h) << 8) | extend_u8(l)
}

fn break_u16(val: W<u16>) -> (W<u8>, W<u8>) {
  let hi = (val.0 & (0xFF << 8)) >> 8;
  let lo = val.0 & 0xFF;
  (W(hi as u8), W(lo as u8))
}

fn signed_add(a: W<u16>, b: W<i8>) -> W<u16> {
  if b < W(0) {
    a - W(b.0.abs() as u16)
  } else {
    a + W(b.0 as u16)
  }
}

fn extend_u8(n: W<u8>) -> W<u16> {
  W(n.0 as u16)
}

impl CPU {
  pub fn new() -> CPU {
    let mut mem = Vec::with_capacity(mem::MEM_TOP);
    mem.resize(mem::MEM_TOP, 0);
    CPU {
      regs: Registers {
        a: W(0),
        f: W(0),
        b: W(0),
        c: W(0),
        d: W(0),
        e: W(0),
        h: W(0),
        l: W(0),
        sp: W(0xFFFE),
        pc: W(0x100),
      },

      mem: RAM { addr: 0, v: mem },
    }
  }

  fn get_reg8(&self, r: Reg8) -> W<u8> {
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

  fn set_reg8(&mut self, r: Reg8, val: W<u8>) {
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

  fn get_reg16(&self, r: Reg16) -> W<u16> {
    match r {
      Reg16::AF => concat_u8(self.regs.a, self.regs.f),
      Reg16::BC => concat_u8(self.regs.b, self.regs.c),
      Reg16::DE => concat_u8(self.regs.d, self.regs.e),
      Reg16::HL => concat_u8(self.regs.h, self.regs.l),
      Reg16::SP => self.regs.sp,
      Reg16::PC => self.regs.pc,
    }
  }

  fn set_reg16(&mut self, r: Reg16, val: W<u16>) {
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
  fn read8(&self, pos: W<u16>) -> W<u8> {
    W(self.mem.read8(pos.0))
  }

  fn write8(&mut self, pos: W<u16>, val: W<u8>) {
    self.mem.write8(pos.0, val.0);
  }

  fn read16(&self, pos: W<u16>) -> W<u16> {
    let h = self.read8(pos);
    let l = self.read8(pos + W(1));
    concat_u8(h, l)
  }

  fn write16(&mut self, pos: W<u16>, val: W<u16>) {
    let (h, l) = break_u16(val);
    self.write8(pos, h);
    self.write8(pos + W(1), l);
  }

  // not sure if i'll need these two fns
  fn get_flag(&self, flag: Flag) -> bool {
    let f = flag as u8;
    f & (self.regs.f.0) != 0
  }

  fn set_flag(&mut self, flag: Flag, value: bool) {
    let f = flag as u8;
    self.regs.f.0 = if value { self.regs.f.0 | f } else { self.regs.f.0 & !f };
  }

  fn push(&mut self, val: W<u16>) {
    self.regs.sp -= W(2);
    let p = self.regs.sp;
    self.write16(p, val);
  }

  fn pop(&mut self) -> W<u16> {
    let val = self.read16(self.regs.sp);
    self.regs.sp += W(2);
    val
  }

  fn step(&mut self) {
    let next = self.decode_next();
    let cycles = self.run(next);
  }

  fn opcode_u8(&self) -> W<u8> {
    self.read8(self.regs.pc + W(1))
  }

  fn opcode_i8(&self) -> W<i8> {
    W(self.opcode_u8().0 as i8)
  }

  fn opcode_u16(&self) -> W<u16> {
    self.read16(self.regs.pc + W(1))
  }

  pub fn decode_next(&mut self) -> Instruction {
    let opcode = self.read8(self.regs.pc).0;

    let reg8: [Reg8; 8] = [Reg8::B, Reg8::C, Reg8::D, Reg8::E, Reg8::H, Reg8::L, Reg8::MemHL, Reg8::A];
    let reg16: [Reg16; 4] = [Reg16::BC, Reg16::DE, Reg16::HL, Reg16::SP];

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

      0xC1 | 0xD1 | 0xE1 => Pop(reg16[(((opcode & 0xf0) >> 4) - 12) as usize]),
      0xF1 => Pop(Reg16::AF),
      0xC5 | 0xD5 | 0xE5 => Push(reg16[(((opcode & 0xf0) >> 4) - 12) as usize]),
      0xF5 => Push(Reg16::AF),

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

      0xC7 => Restart(W(0x00)),
      0xD7 => Restart(W(0x10)),
      0xE7 => Restart(W(0x20)),
      0xF7 => Restart(W(0x30)),
      0xCF => Restart(W(0x08)),
      0xDF => Restart(W(0x18)),
      0xEF => Restart(W(0x28)),
      0xFF => Restart(W(0x38)),

      0xCB => {
        let opcode2 = self.opcode_u8().0;
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
            // Should never happen (0..FF are all covered above)
            panic!("Unhandled opcode 0xCB 0x{:X}", opcode2);
          }
        }
      },

      _ => {
        println!("Unknown opcode 0x{:X} at 0x{:X}", opcode, self.regs.pc);
        UnknownOpcode()
      },
    }
  }

  pub fn duration(&self, instr: Instruction, jumped: bool) -> usize {
   match instr {
      JrFlag(_, _, _) => { if jumped { 12 } else { 8 } },
      JpFlag(_, _, _) => { if jumped { 16 } else { 12 } },
      RetFlag(_, _) => { if jumped { 20 } else { 8 } },
      CallFlag(_, _, _) => { if jumped { 24 } else { 12 } },
      HiLoad(_) | HiWrite(_) | LoadImm16(_, _) | Pop(_) |
        IncReg8(Reg8::MemHL) | DecReg8(Reg8::MemHL) |
        LoadImm8(Reg8::MemHL, _) | Jr(_) | LoadSPOffset(_) => 12,
      LoadReg8(Reg8::MemHL, _) | LoadReg8(_, Reg8::MemHL) | WriteA(_, _) |
        AddReg8(Reg8::MemHL) | SubReg8(Reg8::MemHL) | AndReg8(Reg8::MemHL) |
        OrReg8(Reg8::MemHL) | AddCarryReg8(Reg8::MemHL) |
        SubCarryReg8(Reg8::MemHL) | XorReg8(Reg8::MemHL) |
        Compare(Reg8::MemHL) | AddCarryImm8(_) | SubCarryImm8(_) |
        XorImm8(_) | CompareImm8(_) | AddImm8(_) | SubImm8(_) |
        AndImm8(_) | OrImm8(_) | HiWriteReg() | HiLoadReg() | IncReg16(_) |
        LoadImm8(_, _) | AddHL(_) | SwapSPHL() | ReadA(_, _) | DecReg16(_) => 8,
      Jp(_) | Push(_) | Restart(_) | AddSP(_) | Ret() | RetInterrupt() |
        WriteAImm16(_) | ReadAImm16(_) => 16,
      WriteMemSP(_) => 20,
      Call(_) => 24,
      // 0xCB prefixed opcodes
      Rlc(Reg8::MemHL) | Rrc(Reg8::MemHL) | Rl(Reg8::MemHL) | Rr(Reg8::MemHL) |
        Sla(Reg8::MemHL) | Sra(Reg8::MemHL) | Swap(Reg8::MemHL) |
        Srl(Reg8::MemHL) | TestBit(_, Reg8::MemHL) | SetBit(_, Reg8::MemHL, _) => 16,
      Rlc(_) | Rrc(_) | Rl(_) | Rr(_) | Sla(_) | Sra(_) |
        Swap(_) | Srl(_) | TestBit(_, _) | SetBit(_, _, _) => 8,
      _ => 4,
    }
  }

  //
  // Opcode implementations follow
  //

  fn run(&mut self, instr: Instruction) -> usize {
    let mut jumped = false;
    match instr {
      Nop() => (),
      Stop() => { /* TODO */ },
      Halt() => { /* TODO */ },

      DisableInterrupts() => { /* TODO */ },
      EnableInterrupts() => { /* TODO */ },

      Jr(i) => {
        self.regs.pc = signed_add(self.regs.pc, i);
        jumped = true;
      },
      JrFlag(i, f, v) => {
        if self.get_flag(f) == v {
          self.regs.pc = signed_add(self.regs.pc, i);
          jumped = true;
        }
      },
      Jp(n) => {
        self.regs.pc = n;
        jumped = true;
      },
      JpHL() => {
        self.regs.pc = self.get_reg16(Reg16::HL);
        jumped = true;
      },
      JpFlag(n, f, v) => {
        if self.get_flag(f) == v {
          self.regs.pc = n;
          jumped = true;
        }
      },
      Call(a) => {
        let val = self.regs.pc;
        self.push(val);
        self.regs.pc = a;
        jumped = true;
      },
      CallFlag(a, f, v) => {
        if self.get_flag(f) == v {
          self.run(Call(a));
          jumped = true;
        }
      },
      Ret() => {
        let val = self.pop();
        self.regs.pc = val;
        jumped = true;
      },
      RetFlag(f, v) => {
        if self.get_flag(f) == v {
          self.run(Ret());
          jumped = true;
        }
      },
      RetInterrupt() => {
        self.run(Ret());
        self.run(EnableInterrupts());
        jumped = true;
      },

      AddReg8(r) => { /* TODO */ },
      AddCarryReg8(r) => { /* TODO */ },
      AddImm8(n) => { /* TODO */ },
      AddCarryImm8(n) => { /* TODO */ },
      IncReg8(r) => { /* TODO */ },
      IncReg16(r) => { /* TODO */ },
      SubReg8(r) => { /* TODO */ },
      SubCarryReg8(r) => { /* TODO */ },
      SubImm8(n) => { /* TODO */ },
      SubCarryImm8(n) => { /* TODO */ },
      Compare(r) => { /* TODO */ },
      CompareImm8(n) => { /* TODO */ },
      DecReg8(r) => { /* TODO */ },
      DecReg16(r) => { /* TODO */ },
      AndReg8(r) => { /* TODO */ },
      AndImm8(n) => { /* TODO */ },
      XorReg8(r) => { /* TODO */ },
      XorImm8(n) => { /* TODO */ },
      OrReg8(r) => { /* TODO */ },
      OrImm8(n) => { /* TODO */ },

      LoadReg8(p, q) => { /* TODO */ },
      LoadImm8(r, n) => { /* TODO */ },
      LoadImm16(r, n) => { /* TODO */ },
      WriteA(r, f) => { /* TODO */ },
      WriteAImm16(n) => { /* TODO */ },
      ReadA(r, f) => { /* TODO */ },
      ReadAImm16(n) => { /* TODO */ },
      WriteMemSP(n) => { /* TODO */ },
      HiLoad(n) => { /* TODO */ },
      HiLoadReg() => { /* TODO */ },
      HiWrite(n) => { /* TODO */ },
      HiWriteReg() => { /* TODO */ },

      AddHL(r) => {
        let val = self.get_reg16(Reg16::HL) + self.get_reg16(r);
        self.set_reg16(Reg16::HL, val);
        // TODO update flags
      },
      LoadSPOffset(i) => {
        let pos = signed_add(self.regs.sp, i);
        let val = self.read16(pos);
        self.set_reg16(Reg16::HL, val);
        // TODO update flags
      },
      SwapSPHL() => {
        self.regs.sp = self.get_reg16(Reg16::HL);
      },

      Pop(r) => {
        let val = self.pop();
        self.set_reg16(r, val);
      },
      Push(r) => {
        let val = self.get_reg16(r);
        self.push(val);
      },
      AddSP(i) => {
        self.regs.sp = signed_add(self.regs.pc, i);
        // TODO update flags
      },

      Rlca() => { /* TODO */ },
      Rla() => { /* TODO */ },
      Rrca() => { /* TODO */ },
      Rra() => { /* TODO */ },
      Rlc(r) => { /* TODO */ },
      Rrc(r) => { /* TODO */ },
      Rl(r) => { /* TODO */ },
      Rr(r) => { /* TODO */ },
      Sla(r) => { /* TODO */ },
      Sra(r) => { /* TODO */ },
      Srl(r) => { /* TODO */ },
      Swap(r) => { /* TODO */ },
      TestBit(n, r) => {
        assert!(n <= 7);
        let off = W(1 << n);
        let mut val = self.get_reg8(r);
        let bit = (val & off) == W(0);
        self.set_flag(Flag::Zero, bit);
        self.set_flag(Flag::H, 1);
        self.set_flag(Flag::N, 0);
      },
      SetBit(n, r, v) => {
        assert!(n <= 7);
        let off = W(1 << n);
        let mut val = self.get_reg8(r);
        if v {
          val |= off;
        } else {
          val &= off;
        }
        self.set_reg8(r, val);
      },

      Complement() => { /* TODO */ },
      ComplementCarry() => { /* TODO */ },

      Daa() => { /* TODO */ },
      SetCarryFlag() => { /* TODO */ },

      Restart(n) => {
        let val = self.regs.pc;
        self.push(val);
        self.set_reg16(Reg16::PC, extend_u8(n));
      },

      UnknownOpcode() => {
        panic!("Executed unknown opcode at 0x{:X}", self.regs.pc);
      },
    }
    if !jumped {
      self.regs.pc += W(1);
    }
    self.duration(instr, jumped)
  }

}


#[cfg(test)]
mod tests {
  use super::*;

  // TODO `test` opcode decoding and size/duration

  #[test]
  fn decode() {
    let mut cpu = CPU::new();
    cpu.regs.pc = 0;
    for opcode in 0..256 {
      cpu.mem.write8(cpu.regs.pc, opcode as u8);
      cpu.regs.pc += 1;
    }
    cpu.regs.pc = 0;
    for opcode in 0..256 {
      let instr = cpu.decode_next();
      let dur = cpu.duration(instr, true);
      let dur2 = cpu.duration(instr, false);
      let size = instr.size();
      cpu.regs.pc += 1;
      if dur == dur2 {
        println!("0x{:X} -> {:?} {:?} bytes {:?} cycles", opcode, instr, size, dur);
      } else {
        println!("0x{:X} -> {:?} {:?} bytes {:?}/{:?} cycles", opcode, instr, size, dur, dur2);
      }
      if opcode & 0x0F == 0x0F {
        println!("-------");
      }
    }
    // 0xCB codes
    cpu.regs.pc = 0;
    for opcode in 0..256 {
      cpu.mem.write8(cpu.regs.pc, 0xCB as u8);
      cpu.regs.pc += 1;
      cpu.mem.write8(cpu.regs.pc, opcode as u8);
      cpu.regs.pc += 1;
    }
    cpu.regs.pc = 0;
    for opcode in 0..256 {
      let instr = cpu.decode_next();
      let dur = cpu.duration(instr, true);
      let dur2 = cpu.duration(instr, false);
      let size = instr.size();
      cpu.regs.pc += 2;
      if dur == dur2 {
        println!("0x{:X} -> {:?} {:?} bytes {:?} cycles", opcode, instr, size, dur);
      } else {
        println!("0x{:X} -> {:?} {:?} bytes {:?}/{:?} cycles", opcode, instr, size, dur, dur2);
      }
      if opcode & 0x0F == 0x0F {
        println!("-------");
      }
    }
  }

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
