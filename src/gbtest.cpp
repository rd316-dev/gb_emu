#include "gbtest.h"
#include <iomanip>
#include <iostream>

#include <nlohmann/json.hpp>

#include <sstream>

#include "utils.h"

GbE::Test::Test(CPU *emu)
{
    this->emu = emu;

    size_t mem_size = 32 * 1024;
    mem = (uint8_t*) malloc(mem_size);

    emu->load_rom(mem_size, mem);
}

GbE::Test::~Test()
{
    free(mem);
}

void GbE::Test::launch_json_tests(const std::string &test_name, const std::string &json)
{
    begin_test_suite("JSON tests: " + test_name);
    auto tests = nlohmann::json::parse(json);

    std::cout << "Loaded " << tests.size() << " JSON tests" << std::endl;
    for (const auto &test : tests) {
        auto name = test["name"].get<std::string>();

        auto initial_data = test["initial"];
        auto initial_cpu = initial_data["cpu"];
        auto initial_ram = initial_data["ram"];

        auto final_data = test["final"];
        auto final_cpu = final_data["cpu"];
        auto final_ram = final_data["ram"];

        auto cycles = test["cycles"];

        uint8_t init_F = utils::from_hex(initial_cpu["f"].get<std::string>());

        auto initial_state = State {
            (uint16_t) utils::from_hex(initial_cpu["pc"].get<std::string>()),
            (uint16_t) utils::from_hex(initial_cpu["sp"].get<std::string>()),

            (uint8_t) utils::from_hex(initial_cpu["a"].get<std::string>()),
            (uint8_t) utils::from_hex(initial_cpu["b"].get<std::string>()),
            (uint8_t) utils::from_hex(initial_cpu["c"].get<std::string>()),
            (uint8_t) utils::from_hex(initial_cpu["d"].get<std::string>()),
            (uint8_t) utils::from_hex(initial_cpu["e"].get<std::string>()),
            (uint8_t) utils::from_hex(initial_cpu["h"].get<std::string>()),
            (uint8_t) utils::from_hex(initial_cpu["l"].get<std::string>()),

            (bool) (init_F & flag_mask(Flag::Z)),
            (bool) (init_F & flag_mask(Flag::N)),
            (bool) (init_F & flag_mask(Flag::C)),
            (bool) (init_F & flag_mask(Flag::H)),
            {}
        };

        for (const auto &m : initial_ram) {
            uint16_t addr = utils::from_hex(m[0].get<std::string>());
            uint8_t val = utils::from_hex(m[1].get<std::string>());

            initial_state.mem_delta.push_back(MemoryValue{addr, val});
        }

        uint8_t final_F = utils::from_hex(final_cpu["f"].get<std::string>());

        auto final_state = State {
            (uint16_t) utils::from_hex(final_cpu["pc"].get<std::string>()),
            (uint16_t) utils::from_hex(final_cpu["sp"].get<std::string>()),

            (uint8_t) utils::from_hex(final_cpu["a"].get<std::string>()),
            (uint8_t) utils::from_hex(final_cpu["b"].get<std::string>()),
            (uint8_t) utils::from_hex(final_cpu["c"].get<std::string>()),
            (uint8_t) utils::from_hex(final_cpu["d"].get<std::string>()),
            (uint8_t) utils::from_hex(final_cpu["e"].get<std::string>()),
            (uint8_t) utils::from_hex(final_cpu["h"].get<std::string>()),
            (uint8_t) utils::from_hex(final_cpu["l"].get<std::string>()),

            (bool) (final_F & flag_mask(Flag::Z)),
            (bool) (final_F & flag_mask(Flag::N)),
            (bool) (final_F & flag_mask(Flag::C)),
            (bool) (final_F & flag_mask(Flag::H)),
            {}
        };

        for (const auto &m : final_ram) {
            uint16_t addr = utils::from_hex(m[0].get<std::string>());
            uint8_t val = utils::from_hex(m[1].get<std::string>());

            final_state.mem_delta.push_back(MemoryValue{addr, val});
        }

        begin_subtest_suite(name);
        emu->init(initial_state);
        expect_state(final_state);

        if (!cycle()) {
            end_subtest_suite();
            break;
        }

        end_subtest_suite();
    }

    end_test_suite();
}

void GbE::Test::launch_tests()
{
    emu->set_PC(0x150);

    begin_test_suite("Loads");

    begin_subtest_suite("ld reg8, d8");
    test_loads_reg8_d8();
    end_subtest_suite();

    begin_subtest_suite("ld reg8, reg8");
    test_loads_reg8_reg8();
    end_subtest_suite();

    begin_subtest_suite("ld reg8, (HL)");
    test_loads_reg8_hl();
    end_subtest_suite();

    begin_subtest_suite("ld (reg16), A");
    test_loads_aReg16_A();
    end_subtest_suite();

    end_test_suite();

    begin_test_suite("Special");

    begin_subtest_suite("daa");
    test_daa();
    end_subtest_suite();

    end_test_suite();
}

void GbE::Test::test_loads_reg8_d8()
{
    uint8_t val = 0x22;

    for (int reg_num = 0; reg_num < 8; reg_num++) {
        auto reg = (Reg8) reg_num;
        if (reg == Reg8::_HL) {       // test ld (HL), d8 separately
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
    expect_reg(Reg8::_HL, 0xCD);
    expect_val_u8(0xC000, 0xCD);
    // write 0xCD to (HL) which is 0xC000
    execute(0x36, 0xCD);
}

void GbE::Test::test_loads_reg8_reg8()
{
    for (int reg_src_num = 0; reg_src_num < 8; reg_src_num++) {
        auto reg_src = (Reg8) reg_src_num;
        uint8_t val = 0x32 + reg_src_num;
        uint8_t ld_reg8_d8_opcode = 0x6 | (reg_src_num << 3);

        if (reg_src == Reg8::_HL) {
            continue;
        }

        expect_reg(reg_src, val);
        execute(ld_reg8_d8_opcode, val);

        for (int reg_dest_num = 0; reg_dest_num < 8; reg_dest_num++) {
            auto reg_dest = (Reg8) reg_dest_num;
            uint8_t ld_reg8_reg8_opcode = (0x1 << 6) | (reg_dest_num << 3) | reg_src_num;

            if (reg_dest == Reg8::_HL) {
                continue;
            }

            expect_reg(reg_src, val);
            expect_reg(reg_dest, val);
            execute(ld_reg8_reg8_opcode);
        }
    }
}

void GbE::Test::test_loads_reg8_hl()
{
    auto reg_src = Reg8::_HL;

    uint8_t ld_hl_d8_opcode = 0x36;
    uint8_t val = 0x50;

    // write (HL) to all the registers
    for (int reg_dest_num = 0; reg_dest_num < 8; reg_dest_num++) {
        auto reg_dest = (Reg8) reg_dest_num;
        uint8_t ld_reg8_hl_opcode = (0x1 << 6) | (reg_dest_num << 3) | ((int) reg_src);

        if (ld_reg8_hl_opcode == 0x76) {
            continue; // 0x76 is HALT
        }

        // write address 0xc020 to HL
        expect_reg(Reg8::H, 0xc0);
        execute(0x26, 0xc0); // write to H
        expect_reg(Reg8::L, 0x20);
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

void GbE::Test::test_loads_aReg16_A()
{
    uint8_t val = 0x55;
    uint16_t addr_bc = 0xc000;
    expect_reg(Reg8::A, val);
    execute(0x3E, val); // ld A, 0x55

    expect_reg(Reg16::BC, addr_bc);
    execute(0x01, (uint8_t) addr_bc, (uint8_t) (addr_bc >> 8)); // ld BC, 0xc000

    expect_reg(Reg8::A, val);
    expect_reg(Reg16::BC, addr_bc);
    expect_val_u8(addr_bc, val);
    execute(0x02); // ld (BC), A

    uint16_t addr_de = 0xc010;

    expect_reg(Reg16::DE, addr_de);
    execute(0x11, (uint8_t) addr_de, (uint8_t) (addr_de >> 8)); // ld DE, 0xc010

    expect_reg(Reg8::A, val);
    expect_reg(Reg16::DE, addr_de);
    expect_val_u8(addr_de, val);
    execute(0x12); // ld (DE), A

    uint16_t addr_hl = 0xc020;

    expect_reg(Reg16::HL, addr_hl);
    execute(0x11, (uint8_t) addr_hl, (uint8_t) (addr_hl >> 8)); // ld HL, 0xc000

    expect_reg(Reg8::A, val);
    expect_reg(Reg16::HL, addr_hl + 1);
    expect_val_u8(addr_hl, val);
    execute(0x22); // ld (HL+), A

    expect_reg(Reg8::A, val);
    expect_reg(Reg16::HL, addr_hl);
    expect_val_u8(addr_hl + 1, val);
    execute(0x32); // ld (HL-), A
}

void GbE::Test::test_loads_8bit_other()
{

}

void GbE::Test::test_daa()
{
    struct CALC {
        uint8_t first;
        uint8_t second;
        uint8_t result;
        bool subtract = false;
    };

    std::vector<CALC> vals = {
        {0x09, 0x01, 0x10, false},
        {0x03, 0x06, 0x09, false},
        {0x08, 0x08, 0x16, false},
        {0x14, 0x08, 0x22, false},
        {0x57, 0x12, 0x45, true}
    };

    for (const auto i : vals) {
        expect_reg(Reg8::A, i.first);
        execute(0x3E, i.first);

        expect_reg(Reg8::B, i.second);
        execute(0x06, i.second);

        if (i.subtract) {
            expect_reg(Reg8::A, i.first - i.second);
            expect_flag(Flag::Z, i.first - i.second == 0);
            expect_flag(Flag::N, true);
            execute(0x90);
        } else {
            expect_reg(Reg8::A, i.first + i.second);
            expect_flag(Flag::Z, i.first + i.second == 0);
            expect_flag(Flag::N, false);
            execute(0x80);
        }

        add_context("A", i.first);
        add_context("B", i.second);
        add_context("Expected", i.result);

        expect_reg(Reg8::A, i.result); // converted to BCD
        expect_flag(Flag::Z, i.result == 0);
        expect_flag(Flag::H, 0);
        expect_flag(Flag::C, i.result > 0x99);
        execute(0x27);
    }
}

bool GbE::Test::cycle()
{
    emu->execute();
    bool result = check();
    cleanup();

    return result;
}


bool GbE::Test::execute(const uint8_t opcode)
{
    mem[emu->get_PC()] = opcode;

    return cycle();
}

bool GbE::Test::execute(const uint8_t opcode, const uint8_t arg1)
{
    mem[emu->get_PC()] = opcode;
    mem[emu->get_PC() + 1] = arg1;

    return cycle();
}

bool GbE::Test::execute(const uint8_t opcode, const uint8_t arg1, const uint8_t arg2)
{
    mem[emu->get_PC()] = opcode;
    mem[emu->get_PC() + 1] = arg1;
    mem[emu->get_PC() + 2] = arg2;

    return cycle();
}

bool GbE::Test::check()
{
    const std::vector<std::string> flags = {"Z", "N", "C", "H"};
    const std::vector<std::string> regs8 = {"B", "C", "D", "E", "H", "L", "(HL)", "A"};
    const std::vector<std::string> regs16 = {"BC", "DE", "HL", "AF"};
    const std::vector<std::string> regs16_sp = {"BC", "DE", "HL", "SP"};
    const std::vector<std::string> regs16_addr = {"(BC)", "(DE)", "(HL+)", "(HL-)"};

    int errors = 0;

    if (pc_expected && emu->get_PC() != exp_pc) {
        errors++;
        std::cout << std::hex << std::setw(4) << "PC is incorrect: " 
                  << "expected " << exp_pc << " "
                  << "got " << emu->get_PC()
                  << std::dec << std::endl;
    }

    if (sp_expected && emu->get_SP() != exp_sp) {
        errors++;
        std::cout << std::hex << std::setw(4) << "SP is incorrect: " 
                  << "expected " << exp_sp << " "
                  << "got " << emu->get_SP()
                  << std::dec << std::endl;
    }

    for (auto i : exp_flag) {
        if (emu->get_flag(i.flag) != i.val) {
            errors++;
            std::cout << "Flag " << flags[(int) i.flag] << " is incorrect: "
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
                      << std::dec << std::endl;
        }
    }

    for (auto i : exp_reg16) {
        uint16_t val = emu->read_register16(i.reg);
        if (val != i.val) {
            errors++;
            std::cout << std::hex << "16-bit register " << regs16[(int) i.reg] << " is incorrect: "
                      << "expected " << (unsigned int) i.val << " "
                      << "got " << (unsigned int) val
                      << std::dec << std::endl;
        }
    }

    for (auto i : exp_reg16_sp) {
        auto val = emu->read_register16(i.reg);
        if (val != i.val) {
            errors++;
            std::cout << std::hex << "16-bit register " << regs16_sp[(int) i.reg] << " is incorrect: "
                      << "expected " << (unsigned int) i.val << " "
                      << "got " << (unsigned int) val
                      << std::dec << std::endl;
        }
    }

    for (auto i : exp_val_u8) {
        auto val = emu->peek_memory(i.addr);
        if (val != i.val) {
            errors++;
            std::cout << std::hex << "8-bit unsigned value at " << i.addr << " is incorrect: "
                      << "expected " << (unsigned int) i.val << " "
                      << "got " << (unsigned int) val
                      << std::dec << std::endl;
        }
    }

    for (const auto &i : exp_val_s8) {
        auto val = emu->peek_memory(i.addr);
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
                  << emu->get_arg2() << "\n"
                  << "Context values:\n";

        for (const auto &i : context_u8) {
            std::cout << i.name << "[u8]: "
                      << std::hex << std::setfill('0') << std::setw(2)
                      << (unsigned int) i.val << std::dec << "\n";
        }

        for (const auto &i : context_s8) {
            std::cout << i.name << "[s8]: "
                      << std::hex << std::setfill('0') << std::setw(2)
                      << (int) i.val << std::dec << "\n";
        }
    }

    return errors <= 0;
}

void GbE::Test::cleanup()
{
    pc_expected = false;
    sp_expected = false;

    exp_reg8 = {};
    exp_reg16 = {};
    exp_reg16_sp = {};

    exp_val_u8 = {};
    exp_val_s8 = {};
    //exp_val_u16 = {};
    //exp_val_s16 = {};

    exp_flag = {};

    context_u8 = {};
    context_s8 = {};
}

void GbE::Test::begin_test_suite(const std::string &name)
{
    subtest_suite = "";
    subtest_suite_num = 0;

    test_suite = name;
    test_suite_num += 1;

    std::cout << "TEST suite BEGIN\t[ " << test_suite_num << " | " << test_suite << "]" << std::endl;
}

void GbE::Test::end_test_suite()
{
    std::cout << "TEST suite END" << std::endl;
}

void GbE::Test::begin_subtest_suite(const std::string &name)
{
    subtest_suite = name;
    subtest_suite_num += 1;

    std::cout << "SUBTEST suite BEGIN\t[ " << subtest_suite_num << " | " << subtest_suite << " ]" << std::endl;
}

void GbE::Test::end_subtest_suite()
{
    std::cout << "SUBTEST suite END" << std::endl;
}

void GbE::Test::expect_state(const State &state)
{
    expect_pc(state.PC);
    expect_sp(state.SP);

    expect_reg(Reg8::A, state.A);
    expect_reg(Reg8::B, state.B);
    expect_reg(Reg8::C, state.C);
    expect_reg(Reg8::D, state.D);
    expect_reg(Reg8::E, state.E);
    expect_reg(Reg8::H, state.H);
    expect_reg(Reg8::L, state.L);

    expect_flag(Flag::Z, state.fZ);
    expect_flag(Flag::N, state.fN);
    expect_flag(Flag::C, state.fC);
    expect_flag(Flag::H, state.fH);

    for (const auto &m : state.mem_delta) {
        expect_val_u8(m.addr, m.val);
    }
}

void GbE::Test::expect_pc(const uint16_t pc)
{
    exp_pc = pc;
    pc_expected = true;
}

void GbE::Test::expect_sp(const uint16_t sp)
{
    exp_sp = sp;
    sp_expected = true;
}

void GbE::Test::expect_flag(const Flag flag, const bool val)
{
    exp_flag.push_back({flag, val});
}

void GbE::Test::expect_val_u8(const uint16_t addr, const uint8_t val)
{
    exp_val_u8.push_back({addr, val});
}

void GbE::Test::expect_val_s8(const uint16_t addr, const int8_t val)
{
    exp_val_s8.push_back({addr, val});
}

/*void GbTest::expect_val_u16(const uint16_t addr, const uint16_t val)
{
    exp_val_u16.push_back({addr, val});
}

void GbTest::expect_val_s16(const uint16_t &addr, const int16_t val)
{
    exp_val_s16.push_back({addr, val});
}*/

void GbE::Test::expect_reg(const Reg8 reg, const uint8_t val)
{
    exp_reg8.push_back({reg, val});
}

void GbE::Test::expect_reg(const Reg16 reg, const uint16_t val)
{
    exp_reg16.push_back({reg, val});
}

void GbE::Test::expect_reg(const Reg16_SP reg, const uint16_t val)
{
    exp_reg16_sp.push_back({reg, val});
}

void GbE::Test::add_context(const std::string name, const uint8_t val)
{
    context_u8.push_back( {name, val} );
}

void GbE::Test::add_context(const std::string name, const int8_t val)
{
    context_s8.push_back( {name, val} );
}

/*void GbTest::expect_reg(const Reg16_Addr reg, const uint16_t val)
{
    exp_reg16_addr.push_back({reg, val});
}*/

