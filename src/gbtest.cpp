#include "gbtest.h"
#include <iomanip>
#include <iostream>

GbTest::GbTest(GbE *emu)
{
    this->emu = emu;

    size_t size = 32 * 1024;
    mem = (uint8_t*) malloc(size);

    emu->load_rom(size, mem);
}

GbTest::~GbTest()
{
    free(mem);
}

void GbTest::launch_tests()
{
    emu->set_PC(0x150);

    begin_test_suit("Loads");

    begin_subtest_suit("ld reg8, d8");
    test_loads_reg8_d8();
    end_subtest_suit();

    begin_subtest_suit("ld reg8, reg8");
    test_loads_reg8_reg8();
    end_subtest_suit();

    begin_subtest_suit("ld reg8, (HL)");
    test_loads_reg8_hl();
    end_subtest_suit();

    end_test_suit();
}

void GbTest::test_loads_reg8_d8()
{
    uint8_t val = 0x22;

    for (int reg_num = 0; reg_num < 8; reg_num++) {
        auto reg = (GbAccess::Reg8) reg_num;
        if (reg == GbAccess::Reg8::_HL) {       // test ld (HL), d8 separately
            continue;
        }

        expect_reg(reg, val + reg_num);

        uint8_t opcode = 0x6 | (reg_num << 3);  // ld {reg8}, d8
        execute(opcode, val + reg_num);
    }

    // write address 0xc000 to HL
    execute(0x26, 0xc0);
    execute(0x2e, 0x00);

    // test both as a memory region and a register read
    expect_reg(GbAccess::Reg8::_HL, 0xCD);
    expect_val_u8(0xC000, 0xCD);
    // write 0xCD to (HL) which is 0xC000
    execute(0x36, 0xCD);
}

void GbTest::test_loads_reg8_reg8()
{
    for (int reg_src_num = 0; reg_src_num < 8; reg_src_num++) {
        auto reg_src = (GbAccess::Reg8) reg_src_num;
        uint8_t val = 0x32 + reg_src_num;
        uint8_t ld_reg8_d8_opcode = 0x6 | (reg_src_num << 3);

        if (reg_src == GbAccess::Reg8::_HL) {
            continue;
        }

        expect_reg(reg_src, val);
        execute(ld_reg8_d8_opcode, val);

        for (int reg_dest_num = 0; reg_dest_num < 8; reg_dest_num++) {
            auto reg_dest = (GbAccess::Reg8) reg_dest_num;
            uint8_t ld_reg8_reg8_opcode = (0x1 << 6) | (reg_dest_num << 3) | reg_src_num;

            if (reg_dest == GbAccess::Reg8::_HL) {
                continue;
            }

            expect_reg(reg_src, val);
            expect_reg(reg_dest, val);
            execute(ld_reg8_reg8_opcode);
        }
    }
}

void GbTest::test_loads_reg8_hl()
{
    auto reg_src = GbAccess::Reg8::_HL;

    uint8_t ld_hl_d8_opcode = 0x36;
    uint8_t val = 0x50;

    // write (HL) to all the registers
    for (int reg_dest_num = 0; reg_dest_num < 8; reg_dest_num++) {
        auto reg_dest = (GbAccess::Reg8) reg_dest_num;
        uint8_t ld_reg8_hl_opcode = (0x1 << 6) | (reg_dest_num << 3) | ((int) reg_src);

        if (ld_reg8_hl_opcode == 0x76) {
            continue; // 0x76 is HALT
        }

        // write address 0xc020 to HL
        expect_reg(GbAccess::Reg8::H, 0xc0);
        execute(0x26, 0xc0); // write to H
        expect_reg(GbAccess::Reg8::L, 0x20);
        execute(0x2e, 0x20); // write to L

        // test both as a memory region and a register read
        expect_reg(reg_src, val);
        expect_val_u8(0xC020, val);
        // write val to (HL)
        execute(ld_hl_d8_opcode, val);

        expect_reg(reg_dest, val);
        execute(ld_reg8_hl_opcode);
    }
}

void GbTest::test_loads_8bit_other()
{

}

void GbTest::execute(const uint8_t &opcode)
{
    mem[emu->get_PC()] = opcode;

    emu->execute();
    check();
    cleanup();
}

void GbTest::execute(const uint8_t &opcode, const uint8_t &arg1)
{
    mem[emu->get_PC()] = opcode;
    mem[emu->get_PC() + 1] = arg1;

    emu->execute();
    check();
    cleanup();
}

void GbTest::execute(const uint8_t &opcode, const uint8_t &arg1, const uint8_t &arg2)
{
    mem[emu->get_PC()] = opcode;
    mem[emu->get_PC() + 1] = arg1;
    mem[emu->get_PC() + 2] = arg2;

    emu->execute();
    check();
    cleanup();
}

void GbTest::check()
{
    std::vector<std::string> flags = {"Z", "N", "H", "C"};
    std::vector<std::string> regs8 = {"B", "C", "D", "E", "H", "L", "(HL)", "A"};
    std::vector<std::string> regs16 = {"BC", "DE", "HL", "AF"};
    std::vector<std::string> regs16_sp = {"BC", "DE", "HL", "SP"};
    std::vector<std::string> regs16_addr = {"(BC)", "(DE)", "(HL+)", "(HL-)"};

    int errors = 0;

    for (auto i : exp_flag) {
        if (emu->get_flag(i.flag) != i.val) {
            errors++;
            std::cout << std::hex << "Flag " << flags[(int) i.flag] << " is incorrect: "
                    << "expected " << i.val << " "
                    << "got " << (emu->get_flag(i.flag) > 0)
                    << std::endl;
        }
    }

    for (auto i : exp_reg8) {
        auto val = emu->read_register8(i.reg);
        if (val != i.val) {
            errors++;
            std::cout << std::hex << "8-bit register " << regs8[(int) i.reg] << " is incorrect: "
                    << "expected " << (unsigned int) i.val << " "
                    << "got " << (unsigned int) val
                    << std::endl;
        }
    }

    for (auto i : exp_reg16) {
        auto val = emu->read_register16(i.reg);
        if (val != i.val) {
            errors++;
            std::cout << std::hex << "16-bit register " << regs16[(int) i.reg] << " is incorrect: "
                    << "expected " << (unsigned int) i.val << " "
                    << "got " << (unsigned int) val
                    << std::endl;
        }
    }

    for (auto i : exp_reg16_sp) {
        auto val = emu->read_register16(i.reg);
        if (val != i.val) {
            errors++;
            std::cout << std::hex << "16-bit register " << regs16_sp[(int) i.reg] << " is incorrect: "
                    << "expected " << (unsigned int) i.val << " "
                    << "got " << (unsigned int) val
                    << std::endl;
        }
    }

    for (auto i : exp_val_u8) {
        auto val = emu->read_memory(i.addr);
        if (val != i.val) {
            errors++;
            std::cout << std::hex << "8-bit unsigned value at " << i.addr << " is incorrect: "
                    << "expected " << (unsigned int) i.val << " "
                    << "got " << (unsigned int) val
                    << std::endl;
        }
    }

    for (auto i : exp_val_s8) {
        auto val = emu->read_memory(i.addr);
        if (val != i.val) {
            errors++;
            std::cout << std::hex << "8-bit signed value at " << i.addr << " is incorrect: "
                    << std::dec
                    << "expected " << (int) i.val << " "
                    << "got " << (int) val
                    << std::endl;
        }
    }

    if (errors > 0) {
        std::cout << std::dec << errors << " errors in checks found. Instruction:\n"
                  << std::hex << std::setfill('0') << std::setw(4)
                  << (unsigned int) emu->get_last_PC() << std::dec << "\t"
                  << emu->get_inst() << "\t"
                  << emu->get_arg1() << "\t"
                  << emu->get_arg2() << std::endl;
    }
}

void GbTest::cleanup()
{
    exp_reg8 = {};
    exp_reg16 = {};
    exp_reg16_sp = {};

    exp_val_u8 = {};
    exp_val_s8 = {};
    //exp_val_u16 = {};
    //exp_val_s16 = {};

    exp_flag = {};
}

void GbTest::begin_test_suit(const std::string &name)
{
    subtest_suit = "";
    subtest_suit_num = 0;

    test_suit = name;
    test_suit_num += 1;

    std::cout << "TEST SUIT BEGIN\t[ " << test_suit_num << " | " << test_suit << "]" << std::endl;
}

void GbTest::end_test_suit()
{
    std::cout << "TEST SUIT END" << std::endl;
}

void GbTest::begin_subtest_suit(const std::string &name)
{
    subtest_suit = name;
    subtest_suit_num += 1;

    std::cout << "SUBTEST SUIT BEGIN\t[ " << subtest_suit_num << " | " << subtest_suit << " ]" << std::endl;
}

void GbTest::end_subtest_suit()
{
    std::cout << "SUBTEST SUIT END" << std::endl;
}

void GbTest::expect_flag(const GbAccess::Flag &flag, const bool &val)
{
    exp_flag.push_back({flag, val});
}

void GbTest::expect_val_u8(const uint16_t &addr, const uint8_t &val)
{
    exp_val_u8.push_back({addr, val});
}

void GbTest::expect_val_s8(const uint16_t &addr, const int8_t &val)
{
    exp_val_s8.push_back({addr, val});
}

/*void GbTest::expect_val_u16(const uint16_t &addr, const uint16_t &val)
{
    exp_val_u16.push_back({addr, val});
}

void GbTest::expect_val_s16(const uint16_t &addr, const int16_t &val)
{
    exp_val_s16.push_back({addr, val});
}*/

void GbTest::expect_reg(const GbAccess::Reg8 &reg, const uint8_t &val)
{
    exp_reg8.push_back({reg, val});
}

void GbTest::expect_reg(const GbAccess::Reg16 &reg, const uint16_t &val)
{
    exp_reg16.push_back({reg, val});
}

void GbTest::expect_reg(const GbAccess::Reg16_SP &reg, const uint16_t &val)
{
    exp_reg16_sp.push_back({reg, val});
}

/*void GbTest::expect_reg(const GbAccess::Reg16_Addr &reg, const uint16_t &val)
{
    exp_reg16_addr.push_back({reg, val});
}*/

