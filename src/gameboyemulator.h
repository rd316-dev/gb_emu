#ifndef GAMEBOYEMULATOR_H
#define GAMEBOYEMULATOR_H


#include <cstdint>

enum class Reg8 {B, C, D, E, H, L, _HL, A};
enum class Reg16 {BC=0, DE=2, HL=4, AF=6};
enum class Reg16_SP {BC=0, DE=2, HL=4, SP=6};

enum class CF {Z, C}; // zero or carry

class GbE
{
public:
    GbE(uint8_t boot_rom[256]);

    void decode(uint8_t instruction);

    //===========Control==============
    void nop();
    void stop();
    void halt();

    void daa();

    void scf();
    void cpl();
    void ccf();

    void di();
    void ei();

    void cb();
    //===========Control==============

    //===========Bitwise==============
    // if !carry then store the carry, else use it
    void rla(bool carry);
    void rra(bool carry);

    // prefixed by $CB
    void rl_reg8(Reg8 reg, bool carry);
    void rr_reg8(Reg8 reg, bool carry);

    void sla_reg8(Reg8 reg);
    void sra_reg8(Reg8 reg);
    void srl_reg8(Reg8 reg);

    void swap_reg8(Reg8 reg);
    void bit_reg8(Reg8 reg, uint8_t bit);
    void res_reg8(Reg8 reg, uint8_t bit);
    void set_reg8(Reg8 reg, uint8_t bit);
    //===========Bitwise==============

    //========16-bit loads============
    void load_reg16_d16(Reg16_SP reg);
    void load_hl_sp_s8();
    void load_SP_HL();
    void load_a16_SP();

    void pop(Reg16 reg);
    void push(Reg16 reg);
    //========16-bit loads============

    //=========8-bit loads============
    void load_reg8_d8(Reg8 reg);
    void load_aReg16_A(Reg16 reg);
    void load_A_aReg16(Reg16 reg);
    void load_reg8_reg8(Reg8 reg1, Reg8 reg2);

    void load_a8_A();
    void load_A_a8();

    void load_aC_A();
    void load_A_aC();

    void load_a16_A();
    void load_A_a16();
    //=========8-bit loads============

    //======16-bit arithmetic=========
    void inc_reg16(Reg16_SP reg16);
    void dec_reg16(Reg16_SP reg16);
    void add_HL_reg16(Reg16_SP reg16);
    void add_SP_s8();
    //======16-bit arithmetic=========

    //=======8-bit arithmetic=========
    void add_reg8(Reg8 reg);
    void adc_reg8(Reg8 reg);
    void sub_reg8(Reg8 reg);
    void sbc_reg8(Reg8 reg);
    void and_reg8(Reg8 reg);
    void xor_reg8(Reg8 reg);
    void or_reg8(Reg8 reg);
    void cp_reg8(Reg8 reg);

    void add_d8();
    void adc_d8();
    void sub_d8();
    void sbc_d8();
    void and_d8();
    void xor_d8();
    void or_d8();
    void cp_d8();
    //=======8-bit arithmetic=========

    //============Jumps===============
    // invoke if flag == val
    // relative jumps
    void jr_F_s8(CF flag, bool val);
    void jr_s8();

    // absolute jumps
    void jp_F_a16(CF flag, bool val);
    void jp_a16();

    void jp_HL();

    // calls
    void call_F_a16(CF flag, bool val);
    void call_a16();

    // returns
    void ret_F(CF flag, bool val);

    void ret();
    void reti();

    // resets
    void rst_val(uint8_t val);
    //============Jumps===============

protected:
    uint8_t read_memory(uint16_t addr);
    void write_memory(uint16_t addr, uint8_t val);

    uint8_t read_register8(Reg8 reg);
    void write_register8(Reg8 reg, uint8_t val);

    uint16_t read_register16(Reg16 reg);
    void write_register16(Reg16 reg, uint16_t val);

    uint16_t read_register16(Reg16_SP reg);
    void write_register16(Reg16_SP reg, uint16_t val);

    uint8_t fetch();
    uint16_t fetch16();
    int8_t fetch_signed();
    int16_t fetch16_signed();

    uint16_t pop16();
    void push16(uint16_t val);

    void set_Z(bool value);
    void set_N(bool value);
    void set_H(bool value);
    void set_C(bool value);

    uint8_t get_Z();
    uint8_t get_N();
    uint8_t get_H();
    uint8_t get_C();

    bool half_carry_happened16(uint16_t val1, uint16_t val2);
    bool half_carry_happened8(uint8_t val1, uint8_t val2);

    bool carry_happened16(uint16_t val1, uint16_t val2);
    bool carry_happened8(uint8_t val1, uint8_t val2);

    uint8_t registers[8]; // B, C, D, E, H, L, F, A

    uint16_t SP;
    uint16_t PC;

    bool stopped = false;
    bool halted = false;

    uint32_t rom_size;

    uint8_t boot_rom[256];
    uint8_t rom[];
};

#endif // GAMEBOYEMULATOR_H
