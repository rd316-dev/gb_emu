#ifndef GAMEBOYEMULATOR_H
#define GAMEBOYEMULATOR_H


#include "memorymapper.h"

#include <string>
#include <cstdint>
#include <memory>
#include <functional>

namespace GbE {
enum class Reg8 {B, C, D, E, H, L, _HL, A};
enum class Reg16 {BC=0, DE=1, HL=2, AF=3};
enum class Reg16_SP {BC=0, DE=1, HL=2, SP=3};
enum class Reg16_Addr {BC=0, DE=1, HLI=2, HLD=3};

enum class CF {Z, C}; // zero or carry

enum class Flag {Z, N, C, H};

// 80 bytes header
struct CartidgeHeader {
    uint32_t entry_point;
    uint8_t  logo[48];
    uint8_t  title[16];
    uint16_t licensee_code;
    uint8_t  sgb_flag;
    uint8_t  cartridge_type;
    uint8_t  rom_size;
    uint8_t  ram_size;
    uint8_t  destination_code;
    uint8_t  old_licensee_code;
    uint8_t  rom_version;
    uint8_t  header_checksum;
    uint16_t global_checksum;
};

struct LcdControl {
    bool lcd_ppu_enabled;
    bool window_tile_map_area;
    bool window_enabled;
    bool bg_window_tile_data_area;
    bool bg_tile_map_area;
    bool obj_size;
    bool obj_enable;
    bool priority;
};

class CPU
{
public:
    CPU();
    ~CPU();

    void reset();
    void execute_sequence(const std::vector<uint8_t> &bytes);

    void load_boot_rom(const size_t &size, uint8_t* boot_rom);
    void load_rom     (const size_t &size, uint8_t* rom);
    void execute();

    uint8_t ppu_cycle();
    void timer_cycle();

    uint16_t get_last_PC() const;
    uint8_t  get_last_opcode() const;
    uint8_t  get_last_CB_opcode() const;
    const std::vector<uint8_t> get_spi_buffer_data() const;

    std::string get_inst() const;
    std::string get_arg1() const;
    std::string get_arg2() const;

    bool isStopped() const;
    bool isHalted() const;

    void     set_PC(const uint16_t &addr);
    uint16_t get_PC() const;

    void     set_flag(const Flag &flag, const bool &val);
    uint8_t  get_flag(const Flag &flag) const;

    uint8_t  read_memory(const uint16_t &addr);

    uint8_t  read_register8(const Reg8 &reg);
    uint16_t read_register16(const Reg16 &reg) const;
    uint16_t read_register16(const Reg16_SP &reg) const;

    void write_memory(const uint16_t &addr, const uint8_t &val);
    void write_register8(const Reg8 &reg, const uint8_t &val);
    void write_register16(const Reg16 &reg, const uint16_t &val);
    void write_register16(const Reg16_SP &reg, const uint16_t &val);

protected:
    bool half_carry_happened16(const uint16_t &val1, const uint16_t &val2) const;
    bool half_carry_happened8(const uint8_t &val1, const uint8_t &val2) const;

    bool carry_happened16(const uint16_t &val1, const uint16_t &val2) const;
    bool carry_happened8(const uint8_t &val1, const uint8_t &val2) const;

    void init_instruction_table();
    void init_cb_instruction_table();

    void unknown();

    // debug functions for displaying current instruction
    void debug_push_arg(const std::string &arg);

    void debug_push_inst(const std::string &inst);
    void debug_push_reg(const Reg8 &reg);
    void debug_push_reg(const Reg16 &reg);
    void debug_push_reg(const Reg16_SP &reg);
    void debug_push_reg(const Reg16_Addr &reg);

    void debug_push_addr16(const uint16_t  &addr);
    void debug_push_val_s8(const int8_t    &val);
    void debug_push_val_u8(const uint8_t   &val);
    void debug_push_val_s16(const int16_t  &val);
    void debug_push_val_u16(const uint16_t &val);

    void debug_push_flag(const CF &flag);
    void debug_push_flag_n(const CF &flag);
    void debug_push_custom_arg(const std::string &arg);
    // debug functions for displaying current instruction

    void start_dma(const uint8_t &src);
    void dma_cycle(const uint8_t &machine_cycles);

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
    void rla(const bool &carry);
    void rra(const bool &carry);

    // prefixed by $CB
    void rl_reg8(const Reg8 &reg, const bool &carry);
    void rr_reg8(const Reg8 &reg, const bool &carry);

    void sla_reg8(const Reg8 &reg);
    void sra_reg8(const Reg8 &reg);
    void srl_reg8(const Reg8 &reg);

    void swap_reg8(const Reg8 &reg);
    void bit_reg8(const Reg8 &reg, const uint8_t &bit);
    void res_reg8(const Reg8 &reg, const uint8_t &bit);
    void set_reg8(const Reg8 &reg, const uint8_t &bit);
    //===========Bitwise==============

    //========16-bit loads============
    void load_reg16_d16(const Reg16_SP &reg);
    void load_HL_SP_s8();
    void load_SP_HL();
    void load_a16_SP();

    void pop(const Reg16 &reg);
    void push(const Reg16 &reg);
    //========16-bit loads============

    //=========8-bit loads============
    void load_reg8_d8(const Reg8 &reg);
    void load_aReg16_A(const Reg16_Addr &reg);
    void load_A_aReg16(const Reg16_Addr &reg);
    void load_reg8_reg8(const Reg8 &reg1, const Reg8 &reg2);

    void load_a8_A();
    void load_A_a8();

    void load_aC_A();
    void load_A_aC();

    void load_a16_A();
    void load_A_a16();
    //=========8-bit loads============

    //======16-bit arithmetic=========
    void inc_reg16(const Reg16_SP &reg16);
    void dec_reg16(const Reg16_SP &reg16);
    void add_HL_reg16(const Reg16_SP &reg16);
    void add_SP_s8();
    //======16-bit arithmetic=========

    //=======8-bit arithmetic=========
    void add_reg8(const Reg8 &reg);
    void adc_reg8(const Reg8 &reg);
    void sub_reg8(const Reg8 &reg);
    void sbc_reg8(const Reg8 &reg);
    void and_reg8(const Reg8 &reg);
    void xor_reg8(const Reg8 &reg);
    void or_reg8(const Reg8 &reg);
    void cp_reg8(const Reg8 &reg);

    void inc_reg8(const Reg8 &reg);
    void dec_reg8(const Reg8 &reg);

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
    void jr_F_s8(const CF &flag, const bool &val);
    void jr_s8();

    // absolute jumps
    void jp_F_a16(const CF &flag, const bool &val);
    void jp_a16();

    void jp_HL();

    // calls
    void call_F_a16(const CF &flag, const bool &val);
    void call_a16();

    // returns
    void ret_F(const CF &flag, const bool &val);

    void ret();
    void reti();

    // resets
    void rst_val(const uint8_t &val);
    //============Jumps===============

    uint8_t  fetch();
    uint16_t fetch16();
    int8_t   fetch_signed();
    int16_t  fetch16_signed();

    uint16_t pop16();
    void     push16(const uint16_t &val);

private:
    uint8_t registers[8]; // B, C, D, E, H, L, F, A

    uint16_t SP = 0;
    uint16_t PC = 0;

    uint8_t div_incr_counter = 0;

    uint8_t div     = 0;
    uint8_t tima    = 0;
    uint8_t tma     = 0;
    uint8_t tac     = 0;

    uint8_t scy     = 0;
    uint8_t scx     = 0;

    uint8_t LY      = 0;
    uint8_t LYC     = 0;

    uint8_t wy      = 0;
    uint8_t wx      = 0;

    // non-cgb palettes
    uint8_t bgp     = 0;
    uint8_t obp0    = 0;
    uint8_t obp1    = 0;

    uint32_t bg_fifo  = 0;
    uint32_t oam_fifo = 0;

    LcdControl lcdc = {0,0,0,0,0,0,0,0};

    uint8_t interrupt_enabled = 0x01;

    int fetcher_y = 0;
    int fetcher_x = 0;

    int current_dot  = 0;
    int fetcher_step = 0;

    uint16_t fetcher_addr = 0;

    bool stopped = false;
    bool halted  = false;

    bool dma_started = false;

    bool vram_accessible = true;
    bool oam_accessible  = true;
    bool boot_rom_mapped = true;

    uint16_t dma_src_addr   = 0;
    uint16_t dma_dest_addr  = 0;

    //uint8_t current_bank    = 0;
    //uint8_t secondary_bank  = 0;

    bool enable_interrupts;

    uint64_t machine_cycle  = 0;
    int used_cycles         = 0;

    size_t boot_rom_size    = 0;
    size_t rom_size         = 0;
    size_t external_ram_size= 0;

    uint16_t last_PC = 0;
    uint8_t last_opcode = 0;
    uint8_t last_cb_opcode = 0;

    bool debug_inst_output_enabled = false;

    bool debug_write_spi_to_buffer = false;
    bool spi_started = false;
    uint8_t spi_byte = 0;
    std::vector<uint8_t> spi_buffer;

    std::string debug_current_inst;
    std::string debug_arg1;
    std::string debug_arg2;

    CartidgeHeader header;
    MemoryMapper mapper;

    std::function<void()> itab[256];
    std::function<void()> cbtab[256];

    uint16_t frame_buffer_1[160*144] = {};
    uint16_t frame_buffer_2[160*144] = {};
    uint16_t* current_frame_buffer = frame_buffer_1;

    uint8_t* wram = nullptr;
    uint8_t* vram = nullptr;
    uint8_t* hram = nullptr;
    uint8_t* oam  = nullptr;

    uint8_t* boot_rom = nullptr;
    uint8_t* rom = nullptr;

    uint8_t* external_ram = nullptr;
};
};

#endif // GAMEBOYEMULATOR_H
