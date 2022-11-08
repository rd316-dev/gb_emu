#include "gameboyemulator.h"

#include <iostream>
#include <string>

GbE::GbE(uint8_t boot_rom[256])
{

}

void unimplemented(std::string name)
{
    std::cerr << name << " is unimplemented" << std::endl;
}

uint8_t GbE::read_memory(uint16_t addr)
{

}

void GbE::write_memory(uint16_t addr, uint8_t val)
{

}

bool GbE::half_carry_happened16(uint16_t val1, uint16_t val2)
{
    return (((val1 & 0xFFF) + (val2 & 0xFFF)) & 0x1000) == 0x1000;
}

bool GbE::half_carry_happened8(uint8_t val1, uint8_t val2)
{
    return (((val1 & 0xF) + (val2 & 0xF)) & 0x10) == 0x10;
}

bool GbE::carry_happened16(uint16_t val1, uint16_t val2)
{
    return (0xFFFF - val1) < val2;
}

bool GbE::carry_happened8(uint8_t val1, uint8_t val2)
{
    return (0xFF - val1) < val2;
}

uint8_t GbE::read_register8(Reg8 reg)
{
    if (reg == Reg8::_HL) {
        // should it be the other way around?
        uint16_t addr = registers[5];
        addr = addr | ((uint16_t) registers[4] << 8);

        return read_memory(addr);
    }

    return registers[(uint8_t) reg];
}

void GbE::write_register8(Reg8 reg, uint8_t val)
{
    if (reg == Reg8::_HL) {
        uint16_t addr = registers[5];
        addr = addr | ((uint16_t) registers[4] << 8);

        write_memory(addr, val);

        return;
    }

    registers[(uint8_t) reg] = val;
}

uint16_t GbE::read_register16(Reg16 reg)
{
    uint8_t addr = (uint8_t) reg;

    return (registers[addr] << 8) | registers[addr+1];
}

void GbE::write_register16(Reg16 reg, uint16_t val)
{
    uint8_t addr = (uint8_t) reg;

    registers[addr] = (uint8_t) val >> 8;
    registers[addr+1] = (uint8_t) val;
}

uint16_t GbE::read_register16(Reg16_SP reg)
{
    if (reg == Reg16_SP::SP) {
        return SP;
    }

    uint8_t addr = (uint8_t) reg;
    return (registers[addr] << 8) | registers[addr+1];
}

void GbE::write_register16(Reg16_SP reg, uint16_t val)
{
    if (reg == Reg16_SP::SP) {
        SP = val;
        return;
    }

    uint8_t addr = (uint8_t) reg;

    registers[addr] = (uint8_t) val >> 8;
    registers[addr+1] = (uint8_t) val;
}

uint8_t GbE::fetch()
{
    return read_memory(PC++);
}

int8_t GbE::fetch_signed()
{
    return (int8_t) read_memory(PC++);
}

uint16_t GbE::fetch16()
{
    uint16_t result = read_memory(PC++);
    result = result | (read_memory(PC++) << 8);

    return result;
}

int16_t GbE::fetch16_signed()
{
    int16_t result = read_memory(PC++);
    result = result | (((int16_t) read_memory(PC++)) << 8);

    return result;
}

//===========Control==============
void GbE::nop()
{
    // do nothing
}

void GbE::stop()
{
    stopped = true;
}

void GbE::halt()
{
    halted = true;
}

void GbE::daa()
{
    unimplemented("daa");
}

void GbE::scf()
{
    set_N(false);
    set_H(false);
    set_C(true);
}

void GbE::cpl()
{
    set_N(true);
    set_H(true);
    registers[A] = ~registers[A];
}

void GbE::ccf()
{
    set_N(false);
    set_H(false);
    set_C(!get_C());
}

void GbE::di()
{
    unimplemented("di");
}

void GbE::ei()
{
    unimplemented("ei");
}

void GbE::cb()
{
    fetch();
    unimplemented("cb prefix");
}
//===========Control==============

//===========Bitwise==============
// if !carry then store the carry, else use it
void GbE::rla(bool carry)
{
    rl_reg8(Reg8::A, carry);
}

void GbE::rra(bool carry)
{
    rr_reg8(Reg8::A, carry);
}

// prefixed by $CB
void GbE::rl_reg8(Reg8 reg, bool carry)
{
    uint8_t data = read_register8(reg);
    uint8_t bit0;
    if (carry) {
        bit0 = get_C();
    } else {
        bit0 = data >> 7;
        set_C(bit0);
    }

    data = (data << 1) | bit0;

    set_Z(~data);
    set_N(0);
    set_H(0);

    write_register8(reg, data);
}

void GbE::rr_reg8(Reg8 reg, bool carry)
{
    uint8_t data = read_register8(reg);
    uint8_t bit7;
    if (carry) {
        bit7 = get_C();
    } else {
        bit7 = data << 7;
        set_C(bit7);
    }

    data = (data >> 1) | bit7;

    set_Z(~data);
    set_N(0);
    set_H(0);

    write_register8(reg, data);
}

void GbE::sla_reg8(Reg8 reg)
{
    uint8_t data = read_register8(reg);

    set_Z(~data);
    set_N(0);
    set_H(0);
    set_C(data >> 7);

    write_register8(reg, data << 1);
}
void GbE::sra_reg8(Reg8 reg)
{
    uint8_t data = read_register8(reg);
    uint8_t bit7 = (data >> 7) << 7;

    set_Z(~data);
    set_N(0);
    set_H(0);
    set_C(data & 1);

    write_register8(reg, (data >> 1) || bit7);
}

void GbE::srl_reg8(Reg8 reg)
{
    uint8_t data = read_register8(reg);
    set_C(data & 1);
    data = data >> 1;

    set_Z(~data);
    set_N(0);
    set_H(0);

    write_register8(reg, data >> 1);
}

void GbE::swap_reg8(Reg8 reg) {
    uint8_t data = read_register8(reg);

    uint8_t high = data << 4;
    uint8_t low = data >> 4;

    data = high | low;

    set_Z(~data);
    set_N(0);
    set_H(0);
    set_C(0);

    write_register8(reg, high | low);
}

void GbE::bit_reg8(Reg8 reg, uint8_t bit) {
    uint8_t data = read_register8(reg);

    uint8_t bitData = (data << (7 - bit)) >> 7;

    set_Z(!bitData);
    set_N(0);
    set_H(0);
}

void GbE::res_reg8(Reg8 reg, uint8_t bit) {
    uint8_t data = read_register8(reg);
    data = data & ~(1 << bit);

    write_register8(reg, data);
}

void GbE::set_reg8(Reg8 reg, uint8_t bit)
{
    uint8_t data = read_register8(reg);
    data = data | 1 << bit;

    write_register8(reg, data);
}
//===========Bitwise==============

//========16-bit loads============
void GbE::load_reg16_d16(Reg16_SP reg) {
    write_register16(reg, fetch16());
}

void GbE::load_hl_sp_s8() {
    int8_t s8 = fetch_signed();
    uint16_t result = SP + s8;

    set_H(half_carry_happened16(SP, s8));
    set_C(carry_happened16(SP, s8));

    write_register16(Reg16::HL, result);
}

void GbE::load_SP_HL() {
    SP = read_register16(Reg16::HL);
}

void GbE::load_a16_SP()
{
    uint16_t addr = fetch16();
    write_memory(addr, SP);
}

uint16_t GbE::pop16()
{
    uint16_t result = read_memory(SP);
    SP++;

    result = result | ((uint16_t) read_memory(SP) << 8);
    SP++;

    return result;
}

void GbE::pop(Reg16 reg)
{
    write_register16(reg, pop16());
}

void GbE::push16(uint16_t val)
{
    SP--;
    write_memory(SP, (uint8_t) (val >> 8));

    SP--;
    write_memory(SP, (uint8_t) val);
}

void GbE::push(Reg16 reg)
{
    push16(read_register16(reg));
}
//========16-bit loads============

//=========8-bit loads============
void GbE::load_reg8_d8(Reg8 reg)
{
    uint8_t data = fetch();
    write_register8(reg, data);
}

void GbE::load_aReg16_A(Reg16 reg)
{
    uint16_t addr = read_register16(reg);
    write_memory(addr, read_register8(Reg8::A));
}

void GbE::load_A_aReg16(Reg16 reg)
{
    uint16_t addr = read_register16(reg);
    write_register8(Reg8::A, read_memory(addr));
}

void GbE::load_reg8_reg8(Reg8 reg1, Reg8 reg2)
{
    uint8_t data = read_register8(reg2);
    write_register8(reg1, data);
}

void GbE::load_a8_A()
{
    uint16_t addr = 0xFF00 + fetch();
    write_memory(addr, read_register8(Reg8::A));
}

void GbE::load_A_a8()
{
    uint16_t addr = 0xFF00 + fetch();
    write_register8(Reg8::A, read_memory(addr));
}

void GbE::load_aC_A()
{
    uint8_t addr = 0xFF00 + read_register8(Reg8::C);
    write_memory(addr, read_register8(Reg8::A));
}

void GbE::load_A_aC()
{
    uint8_t addr = 0xFF00 + read_register8(Reg8::C);
    write_register8(Reg8::A, read_memory(addr));
}

void GbE::load_a16_A()
{
    uint16_t addr = fetch16();
    write_memory(addr, read_register8(Reg8::A));
}

void GbE::load_A_a16()
{
    uint16_t addr = fetch16();
    write_register8(Reg8::A, read_memory(addr));
}
//=========8-bit loads============

//======16-bit arithmetic=========
void GbE::inc_reg16(Reg16_SP reg16)
{
    write_register16(reg16, read_register16(reg16) + 1);
}

void GbE::dec_reg16(Reg16_SP reg16)
{
    write_register16(reg16, read_register16(reg16) - 1);
}

void GbE::add_HL_reg16(Reg16_SP reg16)
{
    uint16_t val1 = read_register16(Reg16::HL);
    uint16_t val2 = read_register16(reg16);
    uint16_t result = val1 + val2;

    set_N(0);
    set_H(half_carry_happened16(val1, val2));
    set_C(carry_happened16(val1, val2));

    write_register16(Reg16::HL, result);
}

void GbE::add_SP_s8()
{
    uint16_t val1 = read_register16(Reg16::HL);
    uint16_t val2 = fetch_signed();
    uint16_t result = val1 + val2;

    set_Z(0);
    set_N(0);
    set_H(half_carry_happened16(val1, val2));
    set_C(carry_happened16(val1, val2));

    write_register16(Reg16::HL, result);
}
//======16-bit arithmetic=========

//=======8-bit arithmetic=========
void GbE::add_reg8(Reg8 reg)
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);
    uint8_t result = val1 + val2;

    set_Z(result == 0);
    set_N(0);
    set_H(half_carry_happened8(val1, val2));
    set_C(carry_happened8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::add_d8()
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();
    uint8_t result = val1 + val2;

    set_Z(result == 0);
    set_N(0);
    set_H(half_carry_happened8(val1, val2));
    set_C(carry_happened8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::adc_reg8(Reg8 reg)
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);
    uint8_t result = val1 + val2 + get_C();

    set_Z(result == 0);
    set_N(0);
    set_H(half_carry_happened8(val1, val2));
    set_C(carry_happened8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::adc_d8()
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();
    uint8_t result = val1 + val2 + get_C();

    set_Z(result == 0);
    set_N(0);
    set_H(half_carry_happened8(val1, val2));
    set_C(carry_happened8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::sub_reg8(Reg8 reg)
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);
    uint8_t result = val1 - val2;

    set_Z(result == 0);
    set_N(true);
    set_H(half_carry_happened8(val1, val2));
    set_C(carry_happened8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::sub_d8()
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();
    uint8_t result = val1 - val2;

    set_Z(result == 0);
    set_N(true);
    set_H(half_carry_happened8(val1, val2));
    set_C(carry_happened8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::sbc_reg8(Reg8 reg)
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);
    uint8_t result = val1 - val2 - get_C();

    set_Z(result == 0);
    set_N(true);
    set_H(half_carry_happened8(val1, val2));
    set_C(carry_happened8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::sbc_d8()
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();
    uint8_t result = val1 - val2 - get_C();

    set_Z(result == 0);
    set_N(true);
    set_H(half_carry_happened8(val1, val2));
    set_C(carry_happened8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::and_reg8(Reg8 reg)
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);
    uint8_t result = val1 & val2;

    set_Z(result == 0);
    set_N(0);
    set_H(1);
    set_C(0);

    write_register8(Reg8::A, result);
}

void GbE::and_d8()
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();
    uint8_t result = val1 & val2;

    set_Z(result == 0);
    set_N(0);
    set_H(1);
    set_C(0);

    write_register8(Reg8::A, result);
}

void GbE::xor_reg8(Reg8 reg)
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);
    uint8_t result = val1 ^ val2;

    set_Z(result == 0);
    set_N(0);
    set_H(0);
    set_C(0);

    write_register8(Reg8::A, result);
}

void GbE::xor_d8()
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();
    uint8_t result = val1 ^ val2;

    set_Z(result == 0);
    set_N(0);
    set_H(0);
    set_C(0);

    write_register8(Reg8::A, result);
}

void GbE::or_reg8(Reg8 reg)
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);
    uint8_t result = val1 | val2;

    set_Z(result == 0);
    set_N(0);
    set_H(0);
    set_C(0);

    write_register8(Reg8::A, result);
}

void GbE::or_d8()
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();
    uint8_t result = val1 | val2;

    set_Z(result == 0);
    set_N(0);
    set_H(0);
    set_C(0);

    write_register8(Reg8::A, result);
}

void GbE::cp_reg8(Reg8 reg)
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);
    uint8_t result = val1 - val2;

    set_Z(result == 0);
    set_N(1);
    set_H(half_carry_happened8(val1, val2));
    set_C(carry_happened8(val1, val2));
}

void GbE::cp_d8()
{
    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();
    uint8_t result = val1 - val2;

    set_Z(result == 0);
    set_N(1);
    set_H(half_carry_happened8(val1, val2));
    set_C(carry_happened8(val1, val2));
}
//=======8-bit arithmetic=========

//============Jumps===============
// invoke if flag == val
// relative jumps
void GbE::jr_F_s8(CF flag, bool val)
{
    bool flagVal;
    if (flag == CF::Z)
        flagVal = get_Z();
    else
        flagVal = get_C();

    if (flagVal == val)
        PC += fetch_signed();
}

void GbE::jr_s8()
{
    PC += fetch_signed();
}

// absolute jumps
void GbE::jp_F_a16(CF flag, bool val)
{
    bool flagVal;
    if (flag == CF::Z)
        flagVal = get_Z();
    else
        flagVal = get_C();

    if (flagVal == val)
        PC = fetch16();
}

void GbE::jp_a16()
{
    PC = fetch16();
}

void GbE::jp_HL()
{
    PC = read_register16(Reg16::HL);
}

// calls
void GbE::call_F_a16(CF flag, bool val)
{
    uint16_t addr = fetch16();

    bool flagVal;
    if (flag == CF::Z)
        flagVal = get_Z();
    else
        flagVal = get_C();

    if (flagVal == val) {
        push16(PC);
        PC = addr;
    }
}

void GbE::call_a16()
{
    uint16_t addr = fetch16();
    push16(PC);
    PC = addr;
}

// returns
void GbE::ret_F(CF flag, bool val)
{
    bool flagVal;
    if (flag == CF::Z)
        flagVal = get_Z();
    else
        flagVal = get_C();

    if (flagVal == val) {
        PC = pop16();
    }
}

void GbE::ret()
{
    PC = pop16();
}

void GbE::reti() { unimplemented("reti"); }

// resets
void GbE::rst_val(uint8_t val)
{
    push16(PC);
    PC = read_memory(val);
}
//============Jumps===============
