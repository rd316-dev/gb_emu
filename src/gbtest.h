#ifndef GBTEST_H
#define GBTEST_H

#include <gameboyemulator.h>
#include <cstdint>
#include <sstream>

namespace GbE {
typedef struct Reg8_Exp     { Reg8     reg; uint8_t  val; } Reg8_Exp;
typedef struct Reg16_Exp    { Reg16    reg; uint16_t val; } Reg16_Exp;
typedef struct Reg16_SP_Exp { Reg16_SP reg; uint16_t val; } Reg16_SP_Exp;
//typedef struct Reg16_Addr_Exp { Reg16_Addr reg; uint16_t val; } Reg16_Addr_Exp;

typedef struct Val_u8_Exp  { uint16_t addr; uint8_t  val; } Val_u8_Exp;
typedef struct Val_s8_Exp  { uint16_t addr; int8_t   val; } Val_s8_Exp;
//typedef struct Val_u16_Exp { uint16_t addr; uint16_t val; } Val_u16_Exp;
//typedef struct Val_s16_Exp { uint16_t addr; int16_t  val; } Val_s16_Exp;

typedef struct Flag_Exp { Flag flag; bool val; } Flag_Exp;

typedef struct Context_u8 { std::string name; uint8_t val; } Context_u8;
typedef struct Context_s8 { std::string name; int8_t  val; } Context_s8;

class Test
{
public:
    Test(CPU *emu);
    ~Test();
    bool launch_json_tests(const std::string &test_name, const std::string &json);
    void launch_tests();

    void test_loads_reg8_d8();
    void test_loads_reg8_reg8();
    void test_loads_reg8_hl();
    void test_loads_aReg16_A();
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

    void test_daa();

protected:
    bool check();
    void cleanup();

    void begin_test_suite(const std::string &name);
    void end_test_suite();
    void begin_subtest_suite(const std::string &name);
    void end_subtest_suite();

    void prepare_subtest_suite();
    void start_subtest();

    bool cycle();
    bool execute(const uint8_t opcode);
    bool execute(const uint8_t opcode, const uint8_t arg1);
    bool execute(const uint8_t opcode, const uint8_t arg1, const uint8_t arg2);

    void expect_state(const State &state);

    void expect_pc(const uint16_t pc);
    void expect_sp(const uint16_t sp);

    void expect_flag(const Flag flag, const bool val);

    void expect_val_u8 (const uint16_t addr, const uint8_t val);
    void expect_val_s8 (const uint16_t addr, const int8_t  val);
    //void expect_val_u16(const uint16_t addr, const uint16_t val);
    //void expect_val_s16(const uint16_t addr, const int16_t  val);

    void expect_reg(const Reg8     reg, const uint8_t  val);
    void expect_reg(const Reg16    reg, const uint16_t val);
    void expect_reg(const Reg16_SP reg, const uint16_t val);
    //void expect_reg(const Reg16_Addr reg, const uint16_t val);

    void add_context(const std::string name, const uint8_t val);
    void add_context(const std::string name, const int8_t  val);

private:
    std::vector<Context_u8> context_u8;
    std::vector<Context_s8> context_s8;

    bool pc_expected = false;
    bool sp_expected = false;
    uint16_t exp_pc;
    uint16_t exp_sp;

    std::vector<Reg8_Exp>     exp_reg8;
    std::vector<Reg16_Exp>    exp_reg16;
    std::vector<Reg16_SP_Exp> exp_reg16_sp;
    //std::vector<Reg16_Addr_Exp> exp_reg16_addr;

    std::vector<Val_u8_Exp> exp_val_u8;
    std::vector<Val_s8_Exp> exp_val_s8;
    //std::vector<Val_u16_Exp> exp_val_u16;
    //std::vector<Val_s16_Exp> exp_val_s16;

    std::vector<Flag_Exp> exp_flag;

    std::string test_suite;
    std::string subtest_suite;

    int test_suite_num = 0;
    int subtest_suite_num = 0;

    CPU* emu = nullptr;
    uint8_t* mem = nullptr;
};
}

#endif // GBTEST_H

