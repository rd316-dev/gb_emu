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

enum class Flag {Z=0, N=1, H=2, C=3};

enum class LcdControl {
    BgWindowEnable = 0,
    ObjEnable = 1,
    ObjSize = 2,
    BgTileMap = 3,
    BgWindowTiles = 4,
    WindowEnable = 5,
    WindowTileMap = 6,
    LcdPpuEnable = 7
};

enum class Interrupt {
    VBlank = 0,
    STAT   = 1,
    Timer  = 2,
    Serial = 3,
    Joypad = 4
};

struct MemoryValue {
    uint16_t addr;
    uint8_t val;
};

struct State {
    uint16_t PC;
    uint16_t SP;

    uint8_t A;
    uint8_t B;
    uint8_t C;
    uint8_t D;
    uint8_t E;
    uint8_t H;
    uint8_t L;

    bool fZ;
    bool fN;
    bool fH;
    bool fC;

    std::vector<MemoryValue> mem_delta;
};

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

uint8_t flag_mask(Flag flag, bool val = true);
uint8_t lcdc_mask(LcdControl control, bool val = true);

class CPU
{
public:
    CPU();
    ~CPU();

    bool is_frame_ready() const;
    uint8_t* acquire_frame();

    void use_testing_memory();
    void init(const State &state);

    void resume();
    void reset();

    void request_interrupt(Interrupt interrupt);

    void load_boot_rom(const size_t &size, uint8_t* boot_rom);
    void load_rom     (const size_t &size, uint8_t* rom);

    int execute();

    void m_cycle();
    void ppu_cycle();
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

    void     set_PC(const uint16_t addr);
    uint16_t get_PC() const;

    uint16_t get_SP() const;

    void    set_flag(const Flag flag, const bool val);
    uint8_t get_flag(const Flag flag) const;

    uint8_t  read_register8(const Reg8 reg);
    uint16_t read_register16(const Reg16 reg) const;
    uint16_t read_register16(const Reg16_SP reg) const;

    uint8_t peek_memory(const uint16_t addr) const;
    void change_memory(const uint16_t addr, const uint8_t val);

protected:
    uint8_t read_memory(const uint16_t addr);
    void write_memory(const uint16_t addr, const uint8_t val);

    void write_register8(const Reg8 reg, const uint8_t val);
    void write_register16(const Reg16 reg, const uint16_t val);
    void write_register16(const Reg16_SP reg, const uint16_t val);

    bool half_carry16(const uint16_t val1, const uint16_t val2) const;
    bool half_carry8(const uint8_t val1, const uint8_t val2) const;
    bool half_carry_sub8(const uint8_t val1, const uint8_t val2) const;

    bool carry16(const uint16_t val1, const uint16_t val2) const;
    bool carry8(const uint8_t val1, const uint8_t val2) const;
    bool carry_sub8(const uint8_t val1, const uint8_t val2) const;

    void init_instruction_table();
    void init_cb_instruction_table();

    void unknown() const;

    // debug functions for displaying current instruction
    void debug_push_arg(const std::string &arg);

    void debug_push_inst(const std::string &inst);
    void debug_push_reg(const Reg8 reg);
    void debug_push_reg(const Reg16 reg);
    void debug_push_reg(const Reg16_SP reg);
    void debug_push_reg(const Reg16_Addr reg);

    void debug_push_addr8(const  uint8_t   addr);
    void debug_push_addr16(const uint16_t  addr);
    void debug_push_val_s8(const int8_t    val);
    void debug_push_val_u8(const uint8_t   val);
    void debug_push_val_s16(const int16_t  val);
    void debug_push_val_u16(const uint16_t val);

    void debug_push_flag(const CF flag);
    void debug_push_flag_n(const CF flag);
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
    void rla(const bool carry);
    void rra(const bool carry);

    // prefixed by $CB
    void rl_reg8(const Reg8 reg, const bool carry);
    void rr_reg8(const Reg8 reg, const bool carry);

    void sla_reg8(const Reg8 reg);
    void sra_reg8(const Reg8 reg);
    void srl_reg8(const Reg8 reg);

    void swap_reg8(const Reg8 reg);
    void bit_reg8(const Reg8 reg, const uint8_t bit);
    void res_reg8(const Reg8 reg, const uint8_t bit);
    void set_reg8(const Reg8 reg, const uint8_t bit);
    //===========Bitwise==============

    //========16-bit loads============
    void load_reg16_d16(const Reg16_SP reg);
    void load_HL_SP_s8();
    void load_SP_HL();
    void load_a16_SP();

    void pop(const Reg16 reg);
    void push(const Reg16 reg);
    //========16-bit loads============

    //=========8-bit loads============
    void load_reg8_d8(const Reg8 reg);
    void load_aReg16_A(const Reg16_Addr reg);
    void load_A_aReg16(const Reg16_Addr reg);
    void load_reg8_reg8(const Reg8 reg1, const Reg8 reg2);

    void load_a8_A();
    void load_A_a8();

    void load_aC_A();
    void load_A_aC();

    void load_a16_A();
    void load_A_a16();
    //=========8-bit loads============

    //======16-bit arithmetic=========
    void inc_reg16(const Reg16_SP reg16);
    void dec_reg16(const Reg16_SP reg16);
    void add_HL_reg16(const Reg16_SP reg16);
    void add_SP_s8();
    //======16-bit arithmetic=========

    //=======8-bit arithmetic=========
    void add_reg8(const Reg8 reg);
    void adc_reg8(const Reg8 reg);
    void sub_reg8(const Reg8 reg);
    void sbc_reg8(const Reg8 reg);
    void and_reg8(const Reg8 reg);
    void xor_reg8(const Reg8 reg);
    void or_reg8(const Reg8 reg);
    void cp_reg8(const Reg8 reg);

    void inc_reg8(const Reg8 reg);
    void dec_reg8(const Reg8 reg);

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
    void jr_F_s8(const CF flag, const bool val);
    void jr_s8();

    // absolute jumps
    void jp_F_a16(const CF flag, const bool val);
    void jp_a16();

    void jp_HL();

    // calls
    void call_F_a16(const CF flag, const bool val);
    void call_a16();

    // returns
    void ret_F(const CF flag, const bool val);

    void ret();
    void reti();

    // resets
    void rst_val(const uint8_t val);
    //============Jumps===============

    uint8_t  fetch();
    uint16_t fetch16();
    int8_t   fetch_signed();
    int16_t  fetch16_signed();

    uint16_t pop16();
    void     push16(const uint16_t val);

private:
    uint8_t registers[8]; // B, C, D, E, H, L, F, A

    uint16_t SP = 0;
    uint16_t PC = 0;

    uint8_t div_counter = 0;

    uint16_t timer_counter = 0;
    uint8_t timer_freq = 0;

    uint8_t div     = 0;
    uint8_t tima    = 0;
    uint8_t tma     = 0;
    uint8_t tac     = 0;

    uint8_t scy     = 0;
    uint8_t scx     = 0;

    uint8_t LY      = 0;
    uint8_t LYC     = 0;
    uint8_t STAT    = 0;

    uint8_t wy      = 0;
    uint8_t wx      = 0;

    // non-cgb palettes
    uint8_t bgp     = 0;
    uint8_t obp0    = 0;
    uint8_t obp1    = 0;

    uint32_t bg_fifo  = 0;
    uint32_t oam_fifo = 0;

    uint8_t lcdc = 0x00;

    int interrupt_start_state = 0;

    bool ime_set = false;
    bool ime_planned = false;

    uint8_t interrupt_master_enable = 0x00;
    uint8_t interrupt_enable = 0x00;
    uint8_t interrupt_flag = 0x00;

    uint16_t interrupt_addr = 0x00;

    bool vblank_interrupt_requested = false;

    int fetcher_y = 0;
    int fetcher_x = 0;

    int current_dot  = 0;
    int fetcher_step = 0;

    uint16_t fetcher_addr = 0;
    uint16_t bg_map_addr = 0;
    uint16_t window_map_addr = 0;

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
    uint8_t spi_control = 0;
    uint8_t spi_byte = 0;
    std::vector<uint8_t> spi_buffer;

    uint8_t spi_clock = 0;

    std::string debug_current_inst;
    std::string debug_arg1;
    std::string debug_arg2;

    CartidgeHeader header;
    MemoryMapper mapper;

    std::function<void()> itab[256];
    std::function<void()> cbtab[256];

    uint8_t frame_buffer_1[160*144*4] = {};
    uint8_t frame_buffer_2[160*144*4] = {};
    uint8_t* active_frame_buffer = frame_buffer_1;
    uint8_t* displaying_frame_buffer = frame_buffer_2;

    bool frame_ready = false;

    uint8_t* wram = nullptr;
    uint8_t* vram = nullptr;
    uint8_t* hram = nullptr;
    uint8_t* oam  = nullptr;

    uint8_t* boot_rom = nullptr;
    uint8_t* rom = nullptr;

    uint8_t* external_ram = nullptr;

    uint8_t* testing_memory = nullptr;
};
};

#endif // GAMEBOYEMULATOR_H
