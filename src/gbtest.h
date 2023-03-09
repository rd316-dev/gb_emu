#ifndef GBTEST_H
#define GBTEST_H

#include <gameboyemulator.h>
#include <cstdint>
#include <sstream>

typedef struct Reg8_Exp       { GbE::Reg8       reg; uint8_t  val; } Reg8_Exp;
typedef struct Reg16_Exp      { GbE::Reg16      reg; uint16_t val; } Reg16_Exp;
typedef struct Reg16_SP_Exp   { GbE::Reg16_SP   reg; uint16_t val; } Reg16_SP_Exp;
//typedef struct Reg16_Addr_Exp { GbE::Reg16_Addr reg; uint16_t val; } Reg16_Addr_Exp;

typedef struct Val_u8_Exp  { uint16_t addr; uint8_t  val; } Val_u8_Exp;
typedef struct Val_s8_Exp  { uint16_t addr; int8_t   val; } Val_s8_Exp;
//typedef struct Val_u16_Exp { uint16_t addr; uint16_t val; } Val_u16_Exp;
//typedef struct Val_s16_Exp { uint16_t addr; int16_t  val; } Val_s16_Exp;

typedef struct Flag_Exp { GbE::Flag flag; bool val; } Flag_Exp;

class GbTest
{
public:
    GbTest(GbE::CPU *emu);
    ~GbTest();
    void launch_tests();

    void test_loads_reg8_d8();
    void test_loads_reg8_reg8();
    void test_loads_reg8_hl();
    void test_loads_8bit_other();

    void test_loads_16bit();
    void test_stack();

    void test_alu_8bit_A_reg();
    void test_alu_8bit_other();
    void test_alu_16bit();

    void test_jumps();
    void test_control();
    void test_rotates();
    void test_interrupts();
    void test_cb();
protected:
    void check();
    void cleanup();

    void begin_test_suit(const std::string &name);
    void end_test_suit();
    void begin_subtest_suit(const std::string &name);
    void end_subtest_suit();

    void prepare_subtest_suit();
    void start_subtest();

    void execute(const uint8_t &opcode);
    void execute(const uint8_t &opcode, const uint8_t &arg1);
    void execute(const uint8_t &opcode, const uint8_t &arg1, const uint8_t &arg2);

    void expect_flag(const GbE::Flag &flag, const bool &val);

    void expect_val_u8 (const uint16_t &addr, const uint8_t &val);
    void expect_val_s8 (const uint16_t &addr, const int8_t  &val);
    //void expect_val_u16(const uint16_t &addr, const uint16_t &val);
    //void expect_val_s16(const uint16_t &addr, const int16_t  &val);

    void expect_reg(const GbE::Reg8     &reg, const uint8_t  &val);
    void expect_reg(const GbE::Reg16    &reg, const uint16_t &val);
    void expect_reg(const GbE::Reg16_SP &reg, const uint16_t &val);
    //void expect_reg(const GbE::Reg16_Addr &reg, const uint16_t &val);

private:
    std::vector<Reg8_Exp>       exp_reg8;
    std::vector<Reg16_Exp>      exp_reg16;
    std::vector<Reg16_SP_Exp>   exp_reg16_sp;
    //std::vector<Reg16_Addr_Exp> exp_reg16_addr;

    std::vector<Val_u8_Exp>  exp_val_u8;
    std::vector<Val_s8_Exp>  exp_val_s8;
    //std::vector<Val_u16_Exp> exp_val_u16;
    //std::vector<Val_s16_Exp> exp_val_s16;

    std::vector<Flag_Exp> exp_flag;

    std::string test_suit;
    std::string subtest_suit;

    int test_suit_num = 0;
    int subtest_suit_num = 0;
    int subtest_num = 0;

    GbE::CPU* emu = nullptr;
    uint8_t* mem = nullptr;
};

#endif // GBTEST_H

