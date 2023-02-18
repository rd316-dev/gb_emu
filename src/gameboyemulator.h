#ifndef GAMEBOYEMULATOR_H
#define GAMEBOYEMULATOR_H


#include "memorymapper.h"

#include <cstdint>
#include <memory>
#include <functional>

enum class Reg8 {B, C, D, E, H, L, _HL, A};
enum class Reg16 {BC=0, DE=2, HL=4, AF=6};
enum class Reg16_SP {BC=0, DE=2, HL=4, SP=6};
enum class Reg16_Addr {BC=0, DE=2, HLI=4, HLD=6};

enum class CF {Z, C}; // zero or carry

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

class GbE
{
public:
    GbE();
    ~GbE();

    void load_boot_rom(size_t size, std::shared_ptr<uint8_t> boot_rom);
    void load_rom(size_t size, std::shared_ptr<uint8_t> rom);
    void execute();

    void init_instruction_table();
    void init_cb_instruction_table();

    void unknown();

    void start_dma(uint8_t src);
    void dma_cycle(uint8_t machine_cycles);
    void ppu_cycle(uint8_t machine_cycles);

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
    void load_HL_SP_s8();
    void load_SP_HL();
    void load_a16_SP();

    void pop(Reg16 reg);
    void push(Reg16 reg);
    //========16-bit loads============

    //=========8-bit loads============
    void load_reg8_d8(Reg8 reg);
    void load_aReg16_A(Reg16_Addr reg);
    void load_A_aReg16(Reg16_Addr reg);
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

    void inc_reg8(Reg8 reg);
    void dec_reg8(Reg8 reg);

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
    uint8_t     read_memory(uint16_t addr);
    void        write_memory(uint16_t addr, uint8_t val);

    uint8_t     read_register8(Reg8 reg);
    void        write_register8(Reg8 reg, uint8_t val);

    uint16_t    read_register16(Reg16 reg);
    void        write_register16(Reg16 reg, uint16_t val);

    uint16_t    read_register16(Reg16_SP reg);
    void        write_register16(Reg16_SP reg, uint16_t val);

    uint8_t     fetch();
    uint16_t    fetch16();
    int8_t      fetch_signed();
    int16_t     fetch16_signed();

    uint16_t    pop16();
    void        push16(uint16_t val);

    bool    half_carry_happened16(uint16_t val1, uint16_t val2);
    bool    half_carry_happened8(uint8_t val1, uint8_t val2);

    bool    carry_happened16(uint16_t val1, uint16_t val2);
    bool    carry_happened8(uint8_t val1, uint8_t val2);

    void    set_Z(bool value);
    void    set_N(bool value);
    void    set_H(bool value);
    void    set_C(bool value);

    uint8_t get_Z();
    uint8_t get_N();
    uint8_t get_H();
    uint8_t get_C();

    uint8_t registers[8]; // B, C, D, E, H, L, F, A

    uint16_t SP = 0;
    uint16_t PC = 0;

    uint8_t div     = 0;
    uint8_t tima    = 0;
    uint8_t tma     = 0;
    uint8_t tac     = 0;

    uint8_t scy     = 0;
    uint8_t scx     = 0;

    uint8_t ly      = 0;
    uint8_t lyc     = 0;

    uint8_t wy      = 0;
    uint8_t wx      = 0;

    // non-cgb palettes
    uint8_t bgp     = 0;
    uint8_t obp0    = 0;
    uint8_t obp1    = 0;

    uint32_t bg_fifo    = 0;
    uint32_t oam_fifo   = 0;

    LcdControl lcdc = {};

    uint8_t interrupt_enabled = 0x01;

    int fetcher_y                   = 0;
    int fetcher_x                   = 0;

    int current_dot                 = 0;
    int fetcher_step                = 0;

    bool stopped                    = false;
    bool halted                     = false;

    bool ppu_enabled                = false;
    bool window_tile_map_area       = false;
    bool window_enabled             = false;
    bool bg_window_tile_data_area   = false;
    bool bg_tile_map_area           = false;

    bool vram_accessible            = true;
    bool oam_accessible             = true;
    bool boot_rom_mapped            = true;

    bool dma_started                = false;

    uint16_t dma_src_addr   = 0;
    uint16_t dma_dest_addr  = 0;

    uint8_t current_bank    = 0;
    uint8_t secondary_bank  = 0;

    uint64_t machine_cycle  = 0;
    int used_cycles         = 0;

    size_t boot_rom_size    = 0;
    size_t rom_size         = 0;
    size_t external_ram_size= 0;

    CartidgeHeader header;
    MemoryMapper mapper;

    std::function<void()> itab[256];
    std::function<void()> cbtab[256];

    uint16_t frame_buffer_1[160*144] = {};
    uint16_t frame_buffer_2[160*144] = {};
    uint16_t* current_frame_buffer = frame_buffer_1;

    std::shared_ptr<uint8_t> wram;
    std::shared_ptr<uint8_t> vram;
    std::shared_ptr<uint8_t> hram;
    std::shared_ptr<uint8_t> oam;

    std::shared_ptr<uint8_t> boot_rom;
    std::shared_ptr<uint8_t> rom;
    std::shared_ptr<uint8_t> external_ram;
};

#endif // GAMEBOYEMULATOR_H
