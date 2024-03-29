#include "gameboyemulator.h"

#include <iostream>
#include <string>
#include <iomanip>

#include "mbc1_mapper.h"
#include "utils.h"

#define DEBUG_PRINT_ENABLED

void unimplemented(const std::string &name)
{
    std::cout << name << " is unimplemented" << std::endl;
}

std::string GbE::CPU::get_inst() const
{
    return debug_current_inst;
}

std::string GbE::CPU::get_arg1() const
{
    return debug_arg1;
}

std::string GbE::CPU::get_arg2() const
{
    return debug_arg2;
}

void GbE::CPU::debug_push_inst(const std::string &inst)
{
    #ifdef DEBUG_PRINT_ENABLED
    debug_current_inst = inst;
    debug_arg1 = "";
    debug_arg2 = "";
    #endif
}

void GbE::CPU::debug_push_arg(const std::string &arg)
{
    #ifdef DEBUG_PRINT_ENABLED
    if (debug_arg1.length() <= 0)
        debug_arg1 = arg;
    else
        debug_arg2 = arg;
    #endif
}

void GbE::CPU::debug_push_reg(const Reg8 reg)
{
    #ifdef DEBUG_PRINT_ENABLED
    const std::string arr[] = {"B", "C", "D", "E", "H", "L", "(HL)", "A"};
    debug_push_arg(arr[(int) reg]);
    #endif
}

void GbE::CPU::debug_push_reg(const Reg16 reg)
{
    #ifdef DEBUG_PRINT_ENABLED
    const std::string arr[] = {"BC", "DE", "HL", "AF"};
    debug_push_arg(arr[(int) reg]);
    #endif
}

void GbE::CPU::debug_push_reg(const Reg16_SP reg)
{
    #ifdef DEBUG_PRINT_ENABLED
    const std::string arr[] = {"BC", "DE", "HL", "SP"};
    debug_push_arg(arr[(int) reg]);
    #endif
}

void GbE::CPU::debug_push_reg(const Reg16_Addr reg)
{
    #ifdef DEBUG_PRINT_ENABLED
    const std::string arr[] = {"(BC)", "(DE)", "(HL+)", "(HL-)"};
    debug_push_arg(arr[(int) reg]);
    #endif
}

void GbE::CPU::debug_push_addr8(const uint8_t addr)
{
    #ifdef DEBUG_PRINT_ENABLED
    std::stringstream stream;
    stream << "(" << std::hex << std::setfill('0')
           << std::setw(2) << (int) addr << std::dec << ")";

    debug_push_arg(stream.str());
    #endif
}

void GbE::CPU::debug_push_addr16(const uint16_t addr)
{
    #ifdef DEBUG_PRINT_ENABLED
    std::stringstream stream;
    stream << "(" << std::hex << std::setfill('0')
           << std::setw(4) << (int) addr << std::dec << ")";

    debug_push_arg(stream.str());
    #endif
}

void GbE::CPU::debug_push_val_s8(const int8_t val)
{
    #ifdef DEBUG_PRINT_ENABLED
    debug_push_arg(std::to_string((int) val));
    #endif
}

void GbE::CPU::debug_push_val_u8(const uint8_t val)
{
    #ifdef DEBUG_PRINT_ENABLED
    std::stringstream stream;
    stream << std::hex << std::setfill('0') << std::setw(2)
           << (unsigned int) val << std::dec;
    debug_push_arg(stream.str());
    #endif
}

void GbE::CPU::debug_push_val_s16(const int16_t val)
{
    #ifdef DEBUG_PRINT_ENABLED
    debug_push_arg(std::to_string((int) val));
    #endif
}

void GbE::CPU::debug_push_val_u16(const uint16_t val)
{
    #ifdef DEBUG_PRINT_ENABLED
    std::stringstream stream;
    stream << std::hex << std::setfill('0') << std::setw(4)
           << (unsigned int) val << std::dec;
    debug_push_arg(stream.str());
    #endif
}

void GbE::CPU::debug_push_flag(const GbE::CF flag)
{
    #ifdef DEBUG_PRINT_ENABLED
    const std::string arr[] = {"Z", "C"};
    debug_push_arg(arr[(int) flag]);
    #endif
}

void GbE::CPU::debug_push_flag_n(const GbE::CF flag)
{
    #ifdef DEBUG_PRINT_ENABLED
    const std::string arr[] = {"NZ", "NC"};
    debug_push_arg(arr[(int) flag]);
    #endif
}

void GbE::CPU::debug_push_custom_arg(const std::string &arg)
{
    #ifdef DEBUG_PRINT_ENABLED
    debug_push_arg(arg);
    #endif
}

uint8_t GbE::flag_mask(Flag flag, bool val)
{
    return val << (7 - (uint8_t) flag);
}

uint8_t GbE::lcdc_mask(LcdControl control, bool val)
{
    return val << ((uint8_t) control);
}

GbE::CPU::CPU()
{
    init_instruction_table();
    init_cb_instruction_table();

    mapper = MemoryMapper();

    vram = (uint8_t*) malloc(0x2000);
    wram = (uint8_t*) malloc(0x2000);
    external_ram = (uint8_t*) malloc(0x2000);
    oam  = (uint8_t*) malloc(0xA0);
    hram = (uint8_t*) malloc(0x7F);
}

GbE::CPU::~CPU()
{
    free(vram);
    free(wram);
    free(external_ram);
    free(oam);
    free(hram);
    free(testing_memory);
}

bool GbE::CPU::is_frame_ready() const
{
    return frame_ready;
}

uint8_t* GbE::CPU::acquire_frame()
{
    frame_ready = false;
    return displaying_frame_buffer;
}

void GbE::CPU::use_testing_memory()
{
    testing_memory = (uint8_t*) malloc(0x10000);
}

void GbE::CPU::init(const State &data)
{
    PC = data.PC;
    SP = data.SP;

    write_register8(Reg8::A, data.A);
    write_register8(Reg8::B, data.B);
    write_register8(Reg8::C, data.C);
    write_register8(Reg8::D, data.D);
    write_register8(Reg8::E, data.E);
    write_register8(Reg8::H, data.H);
    write_register8(Reg8::L, data.L);
    set_flag(Flag::Z, data.fZ);
    set_flag(Flag::N, data.fN);
    set_flag(Flag::C, data.fC);
    set_flag(Flag::H, data.fH);

    for (const auto &m : data.mem_delta) {
        write_memory(m.addr, m.val);
    }
}

void GbE::CPU::resume()
{
    stopped = false;
    halted = false;
}

void GbE::CPU::reset()
{
    SP = 0;
    PC = 0;

    machine_cycle = 0;
    used_cycles = 0;

    last_PC = 0;
    last_opcode = 0;
    last_cb_opcode = 0;

    scy = 0;
    scx = 0;

    lcdc = 0;
    interrupt_master_enable = 0x01;

    fetcher_y = 0;
    fetcher_x = 0;

    current_dot  = 0;
    fetcher_step = 0;

    vram_accessible = true;
    oam_accessible  = true;
    boot_rom_mapped = true;

    stopped = false;
    halted  = false;

    dma_started = false;

    free(testing_memory);
    testing_memory = nullptr;
}

void GbE::CPU::request_interrupt(Interrupt interrupt)
{
    uint8_t mask = 1 << ((uint8_t) interrupt);
    interrupt_flag = interrupt_flag | mask;
}

void GbE::CPU::load_boot_rom(const size_t &size, uint8_t* boot_rom)
{
    if (size < 0xFF) {
        std::cerr << "Error: Boot ROM size is lower than 256 bytes. Current Size: " << size << std::endl;
        return;
    }

    this->boot_rom = boot_rom;
    this->boot_rom_mapped = true;
    this->PC = 0;

    std::cout << "Boot ROM loaded. Size: " << size << std::endl;
}

void GbE::CPU::load_rom(const size_t &size, uint8_t* rom)
{
    // minimal size for a rom with a valid header is 335 bytes
    // 0x14F is the last byte of header
    if (size < 0x14F) {
        std::cerr << "Error: ROM size is lower than 335 bytes. Current Size: " << size << std::endl;
        return;
    }

    this->rom_size = size;
    this->rom = rom;

    // copy header to structure for easier read
    CartidgeHeader header;
    memcpy(&(header.entry_point),       rom + 0x100, 4);
    memcpy(header.logo,                 rom + 0x104, 48);
    memcpy(header.title,                rom + 0x134, 16);
    memcpy(&(header.licensee_code),     rom + 0x144, 2);
    memcpy(&(header.sgb_flag),          rom + 0x146, 1);
    memcpy(&(header.cartridge_type),    rom + 0x147, 1);
    memcpy(&(header.rom_size),          rom + 0x148, 1);
    memcpy(&(header.ram_size),          rom + 0x149, 1);
    memcpy(&(header.destination_code),  rom + 0x14A, 1);
    memcpy(&(header.old_licensee_code), rom + 0x14B, 1);
    memcpy(&(header.rom_version),       rom + 0x14C, 1);
    memcpy(&(header.header_checksum),   rom + 0x14D, 1);
    memcpy(&(header.global_checksum),   rom + 0x14E, 2);

    std::cout << "ROM loaded. Size: " << size << std::endl;
    std::cout << "Title: " << header.title << std::endl;

    switch (header.cartridge_type) {
        case 0x00: {
            break;
        } case 0x01: {
            mapper = MBC1_Mapper(rom_size, header.ram_size);
            break;
        } default: {
            std::cout << "Unsupported mapper: " << (unsigned int) header.cartridge_type << std::endl;
        }
    }
}

int GbE::CPU::execute()
{
    stopped = false;

    used_cycles = 0;

    uint8_t available_interrupts = interrupt_enable & interrupt_flag;
    if (interrupt_master_enable && available_interrupts) [[unlikely]] {
        interrupt_master_enable = 0x0;
        interrupt_start_state = 1;

        if (available_interrupts & 0x01) {
            available_interrupts ^= 0x01;
            interrupt_addr = 0x40;
        } else if (available_interrupts & 0x02) {
            available_interrupts ^= 0x02;
            interrupt_addr = 0x48;
        } else if (available_interrupts & 0x04) {
            available_interrupts ^= 0x04;
            interrupt_addr = 0x50;
        } else if (available_interrupts & 0x08) {
            available_interrupts ^= 0x08;
            interrupt_addr = 0x58;
        } else if (available_interrupts & 0x10) {
            available_interrupts ^= 0x10;
            interrupt_addr = 0x60;
        }
    }

    if (interrupt_start_state <= 0) [[likely]] {
        if (!halted) [[likely]] {
            last_PC = PC;

            uint8_t inst = fetch();
            last_opcode = inst;

            itab[inst]();
        } else {
            m_cycle();
        }
    } else {
        halted = false;

        switch (interrupt_start_state) {
            case 1:
            case 2:
                m_cycle();
                interrupt_start_state++;
                break;
            case 3:
                push16(PC);

                interrupt_start_state++;
                break;
            case 4:
                PC = interrupt_addr;

                m_cycle();
                interrupt_start_state = 0;
                break;
        }
    }

    machine_cycle += used_cycles;

    if (dma_started) [[unlikely]] {
        dma_cycle(used_cycles);
    }

    if (ime_planned) [[unlikely]] {
        interrupt_master_enable = 0x01;
        ime_set = false;
        ime_planned = false;
    }

    if (ime_set) [[unlikely]] {
        ime_planned = true;
    }

    return used_cycles;
}

void GbE::CPU::m_cycle()
{
    used_cycles++;

    if (!halted) {
        timer_cycle();
    }

    if (lcdc & lcdc_mask(LcdControl::LcdPpuEnable)) [[likely]] {
        ppu_cycle();
        ppu_cycle();
        ppu_cycle();
        ppu_cycle();
    }
}

void GbE::CPU::timer_cycle()
{
    // prevent the branching by using binary operations
    div_counter++;

    uint8_t mask = 0x40;
    uint8_t increment = (div_counter & mask) >> 6;

    // increment DIV if the counter is equal to 64
    div += increment;

    // reset the counter if it's equal to 64
    div_counter = div_counter ^ mask;

    if (tac & 0x04) [[unlikely]] {
        timer_counter++;
        if (timer_counter >= timer_freq) {
            tima++;
            timer_counter = 0;

            if (tima == 0) [[unlikely]] {
                tima = tma;
                request_interrupt(Interrupt::Timer);
            }
        }
    }
}

void GbE::CPU::ppu_cycle()
{
    int current_scanline = current_dot / 456;
    int scanline_dot = current_dot % 456;

    if (current_scanline > LY) {
        int i = 0;
        i += 1;
    }

    LY = current_scanline;

    uint8_t stat_mask = 1 << 2;
    uint8_t stat_val_mask = (LY == LYC) << 2;

    STAT = (STAT ^ stat_mask) & stat_val_mask;

    if (stat_val_mask && (STAT & 0x40)) {
        request_interrupt(Interrupt::STAT);
    }

    if (current_scanline <= 143) {
        vblank_interrupt_requested = false;

        if (scanline_dot < 80) { // mode 2
            // oam scan
        } else if (scanline_dot < 240) { // mode 3
            int pixel_x = (scanline_dot) - 80;

            uint16_t bg_map_addr = 0;
            if (lcdc & lcdc_mask(LcdControl::BgTileMap)) {
                bg_map_addr = 0x9C00;
            } else {
                bg_map_addr = 0x9800;
            }

            uint16_t bg_data_addr = 0;
            if (lcdc & lcdc_mask(LcdControl::BgWindowTiles)) {
                bg_data_addr = 0x8000;
            } else {
                bg_data_addr = 0x8800;
            }

            // find the index of the tile
            uint8_t tile_map_x = ((pixel_x + scx) >> 3);
            uint8_t tile_map_y = ((current_scanline + scy) >> 3);

            uint16_t tile_map_addr = bg_map_addr + (tile_map_y << 5) + tile_map_x;
            uint8_t tile_data_index = peek_memory(tile_map_addr);

            // find the data of the tile
            uint16_t tile_data_addr = bg_data_addr + (tile_data_index << 4) + (((current_scanline + scy) & 0xF8) << 1);

            uint8_t tile_low = peek_memory(tile_data_addr);
            uint8_t tile_high = peek_memory(tile_data_addr + 1);

            // get color of the pixel
            uint8_t inner_tile_x = (7 - ((pixel_x + scx) & 0x07));
            uint8_t mask = 1 << inner_tile_x;

            uint8_t palette_index = (((tile_high & mask) << 1) + (tile_low & mask)) >> inner_tile_x;
            uint8_t palette_val = (bgp >> (palette_index << 1)) & 0x02;

            // convert to Grayscale to RGB
            uint8_t grayscale = 0;

            if (palette_val == 0) {
                grayscale = 0xFF;
            } else if (palette_val == 1) {
                grayscale = 0xF0;
            } else if (palette_val == 2) {
                grayscale = 0x0F;
            } else if (palette_val == 3) {
                grayscale = 0x00;
            }

            int pixel_addr = ((LY * 160) + pixel_x) * 4;

            active_frame_buffer[pixel_addr] = grayscale;
            active_frame_buffer[pixel_addr + 1] = grayscale;
            active_frame_buffer[pixel_addr + 2] = grayscale;
            active_frame_buffer[pixel_addr + 3] = 0xff;
        } else if (scanline_dot < 252) {
            // whatever
        } else { // mode 0
            // hblank
        }
    } else if (current_scanline <= 153) { // mode 1
        if (!vblank_interrupt_requested) {
            request_interrupt(Interrupt::VBlank);
            vblank_interrupt_requested = true;

            frame_ready = true;

            std::swap(active_frame_buffer, displaying_frame_buffer);
        }
    }

    current_dot++;
    if (current_dot > 70224) {
        current_dot = 0;
    }
}

void GbE::CPU::dma_cycle(const uint8_t &machine_cycles)
{
    for (int i = 0; i < machine_cycles; i++) {
        change_memory(dma_dest_addr, peek_memory(dma_src_addr));
        dma_src_addr++;
        dma_dest_addr++;

        if (dma_dest_addr > 0xFE9F) {
            dma_started = false;
            dma_src_addr = 0;
            dma_dest_addr = 0;
            break;
        }
    }
}

void GbE::CPU::start_dma(const uint8_t &src)
{
    dma_src_addr = src << 8;
    dma_dest_addr = 0xFE00;
    dma_started = true;
}

uint8_t GbE::CPU::peek_memory(const uint16_t addr) const
{
    if (testing_memory != nullptr) [[unlikely]] {
        return testing_memory[addr];
    }

    if (addr <= 0x00FF && boot_rom_mapped && boot_rom != nullptr) return boot_rom[addr];
    else if (addr <= 0x3FFF) return rom[mapper.convert_rom0_addr(addr)];
    else if (addr <= 0x7FFF) return rom[mapper.convert_romN_addr(addr)];
    else if (addr <= 0x9FFF) return vram[addr - 0x8000];
    else if (addr <= 0xBFFF) return external_ram[mapper.convert_ram_addr(addr)];

    else if (addr <= 0xCFFF) return wram[addr - 0xC000];
    else if (addr <= 0xDFFF) return wram[addr - 0xC000];
    else if (addr <= 0xFDFF) return wram[addr - 0xE000];

    else if (addr <= 0xFE9F)  {
        if (oam_accessible) {
            return oam[addr - 0xFE00];
        } else {
            return 0xFF;
        }
    } 
    else if (addr <= 0xFEFF) return 0xFF; // not usable
    else if (addr <= 0xFF7F) {
        if (addr == 0xFF00) {
            return 0x3F; // joypad
        } else if (addr == 0xFF01) {
            return spi_byte;
        } else if (addr == 0xFF02) {
            return spi_control;
        } else if (addr == 0xFF04) {
            return div;
        } else if (addr == 0xFF05) {
            return tima;
        } else if (addr == 0xFF06) {
            return tma;  
        } else if (addr == 0xFF07) {
            return tac;
        } else if (addr == 0xFF0F) {
            return interrupt_flag;
        } else if (addr <= 0xFF26) {
            // audio
        } else if (addr <= 0xFF3F) {
            // wave pattern
        } else if (addr == 0xFF40) {
            return lcdc;
        } else if (addr == 0xFF41) {
            return STAT;
        } else if (addr == 0xFF42) {
            return scy;
        } else if (addr == 0xFF43) {
            return scx;
        } else if (addr == 0xFF44) {
            return LY;
        } else if (addr == 0xFF45) {
            return LYC;
        } else if (addr == 0xFF46) {
            return dma_src_addr;
        } else if (addr == 0xFF47) {
            return bgp;
        } else if (addr == 0xFF48) {
            return obp0;
        } else if (addr == 0xFF49) {
            return obp1;
        } else if (addr == 0xFF4A) {
            return wy;
        } else if (addr == 0xFF4B) {
            return wx;
        } else {
            return 0xFF;
        }
    }
    else if (addr <= 0xFFFE) return hram[addr - 0xFF80];
    else return interrupt_enable;

    return 0xFF;
}

void GbE::CPU::change_memory(const uint16_t addr, const uint8_t val)
{
    if (testing_memory != nullptr) [[unlikely]] {
        testing_memory[addr] = val;
        return;
    }

    if      (addr <= 0x3FFF) mapper.handle_rom0_write(addr, val);
    else if (addr <= 0x7FFF) mapper.handle_romN_write(addr, val);
    else if (addr <= 0x9FFF) vram[addr - 0x8000] = val;
    else if (addr <= 0xBFFF) {
        if (!mapper.handle_ram_write(addr, val))
            external_ram[mapper.convert_ram_addr(addr)] = val;
    }
    else if (addr <= 0xCFFF) wram[addr - 0xC000] = val;
    else if (addr <= 0xDFFF) wram[addr - 0xC000] = val;
    else if (addr <= 0xFDFF) wram[addr - 0xE000] = val;
    else if (addr <= 0xFE9F) oam[addr - 0xFE00] = val;
    else if (addr <= 0xFEFF) unknown();
    else if (addr <= 0xFF7F) {
        if (addr == 0xFF00) {
            // joypad
        } else if (addr == 0xFF01) {
            // serial transfer
            spi_byte = val;
        } else if (addr == 0xFF02) {
            if (val & 0x80) {
                std::cout << (char) spi_byte;
                spi_buffer.push_back(spi_byte);
            }
        } else if (addr == 0xFF04) {
            div = 0;
        } else if (addr == 0xFF05) {
            tima = val;
        } else if (addr == 0xFF06) {
            tma = val;  
        } else if (addr == 0xFF07) {
            tac = val;
        } else if (addr == 0xFF05) {
            tima = val;
            timer_counter = 0;
        } else if (addr == 0xFF06) {
            tma = val;
        } else if (addr == 0xFF07) {
            tac = val;

            if (tac & 0x04) {
                uint16_t freq[] = {1024, 16, 64, 256};
                timer_freq = freq[val ^ 0x04];
            }
        } else if (addr == 0xFF0F) {
            interrupt_flag = val & 0x1F;
        } else if (addr <= 0xFF26) {
            // audio
        } else if (addr <= 0xFF3F) {
            // wave pattern
        } else if (addr == 0xFF40) {// lcdc
            lcdc = val;
        } else if (addr == 0xFF41) {
            STAT = val;
        } else if (addr == 0xFF42) {
            scy = val;
        } else if (addr == 0xFF43) {
            scx = val;
        } else if (addr == 0xFF44) {
            // read only
        } else if (addr == 0xFF45) {
            LYC = val;
        } else if (addr == 0xFF46) {
            dma_src_addr = val;
        } else if (addr == 0xFF47) {
            bgp = val;
        } else if (addr == 0xFF48) {
            obp0 = val;
        } else if (addr == 0xFF49) {
            obp1 = val;
        } else if (addr == 0xFF4A) {
            wy = val;
        } else if (addr == 0xFF4B) {
            wx = val;
        } else if (addr <= 0xFF4F) {
            // [cgb] vram bank select
        } else if (addr <= 0xFF50) {
            if (val) { // disable boot rom
                boot_rom_mapped = false;
            }
        } else if (addr <= 0xFF55) {
            // [cgb] vram dma
        } else if (addr <= 0xFF69) {
            // [cgb] bg/obj palettes
        } else if (addr <= 0xFF70) {
            // [cgb] wram bank select
        } else {
            unknown();
        }
    }
    else if (addr <= 0xFFFE) hram[addr - 0xFF80] = val;
    else interrupt_enable = val;
}

uint8_t GbE::CPU::read_memory(const uint16_t addr)
{
    m_cycle();

    return peek_memory(addr);
}

void GbE::CPU::write_memory(const uint16_t addr, const uint8_t val)
{
    m_cycle();

    change_memory(addr, val);    
}

const std::vector<uint8_t> GbE::CPU::get_spi_buffer_data() const
{
    return spi_buffer;
}

uint16_t GbE::CPU::get_last_PC() const
{
    return last_PC;
}

uint8_t GbE::CPU::get_last_opcode() const
{
    return last_opcode;
}

uint8_t GbE::CPU::get_last_CB_opcode() const
{
    return last_cb_opcode;
}

bool GbE::CPU::isStopped() const
{
    return stopped;
}

bool GbE::CPU::isHalted() const
{
    return halted;
}

uint16_t GbE::CPU::get_PC() const
{
    return PC;
}

void GbE::CPU::set_PC(const uint16_t addr)
{
    PC = addr;
}

uint16_t GbE::CPU::get_SP() const
{
    return SP;
}

uint8_t GbE::CPU::get_flag(const Flag flag) const
{
    uint8_t flag_num = (uint8_t) flag;
    uint8_t mask = flag_mask(flag);

    return (registers[6] & mask) > 0;
}

void GbE::CPU::set_flag(const Flag flag, const bool val)
{
    uint8_t flag_num = (uint8_t) flag;

    uint8_t mask = flag_mask(flag);
    uint8_t val_mask = flag_mask(flag, val);

    registers[6] = (registers[6] & ~mask) ^ val_mask;
}

bool GbE::CPU::half_carry16(const uint16_t val1, const uint16_t val2) const
{
    return ((val1 & 0xFFF) + (val2 & 0xFFF)) > 0x0FFF;
}

bool GbE::CPU::half_carry8(const uint8_t val1, const uint8_t val2) const
{
    return ((val1 & 0xF) + (val2 & 0xF)) > 0x0F;
}

bool GbE::CPU::half_carry_sub8(const uint8_t val1, const uint8_t val2) const
{
    return ((uint8_t) ((val1 & 0xF) - (val2 & 0xF))) > 0x0F;
}

bool GbE::CPU::carry16(const uint16_t val1, const uint16_t val2) const
{
    return (((uint32_t) val1) + val2) > 0xFFFF;
}

bool GbE::CPU::carry8(const uint8_t val1, const uint8_t val2) const
{
    return (((uint16_t) val1) + val2) > 0xFF;
}

bool GbE::CPU::carry_sub8(const uint8_t val1, const uint8_t val2) const
{
    return val2 > val1;
}

uint8_t GbE::CPU::read_register8(const Reg8 reg)
{
    if (reg == Reg8::_HL) {
        uint16_t addr = read_register16(Reg16::HL);
        return read_memory(addr);
    }

    return registers[(uint8_t) reg];
}

void GbE::CPU::write_register8(const Reg8 reg, const uint8_t val)
{
    if (reg == Reg8::_HL) {
        uint16_t addr = read_register16(Reg16::HL);
        write_memory(addr, val);

        return;
    }

    registers[(uint8_t) reg] = val;
}

uint16_t GbE::CPU::read_register16(const Reg16 reg) const
{
    if (reg == Reg16::AF) {
        return ((uint16_t) (registers[7] << 8)) | (registers[6] & 0xF0);
    }

    uint8_t addr = ((uint8_t) reg) << 1;

    return ((uint16_t) (registers[addr] << 8)) | registers[addr+1];
}

void GbE::CPU::write_register16(const Reg16 reg, const uint16_t val)
{
    if (reg == Reg16::AF) {
        registers[7] = (uint8_t) (val >> 8);
        registers[6] = (uint8_t) (val & 0xF0);
        return;
    }

    uint8_t addr = ((uint8_t) reg) << 1;

    registers[addr] = (uint8_t) (val >> 8);
    registers[addr+1] = (uint8_t) (val & 0xFF);
}

uint16_t GbE::CPU::read_register16(const Reg16_SP reg) const
{
    if (reg == Reg16_SP::SP)
        return SP;

    uint8_t addr = ((uint8_t) reg) * 2;
    return ((uint16_t) (registers[addr] << 8)) | registers[addr+1];
}

void GbE::CPU::write_register16(const Reg16_SP reg, const uint16_t val)
{
    if (reg == Reg16_SP::SP) {
        SP = val;
        return;
    }

    uint8_t addr = ((uint8_t) reg) * 2;

    registers[addr] = (uint8_t) (val >> 8);
    registers[addr+1] = (uint8_t) (val & 0xFF);
}

uint8_t GbE::CPU::fetch()
{
    return read_memory(PC++);
}

int8_t GbE::CPU::fetch_signed()
{
    return (int8_t) read_memory(PC++);
}

uint16_t GbE::CPU::fetch16()
{
    uint16_t result = read_memory(PC++);
    result = result | (((uint16_t) read_memory(PC++)) << 8);

    return result;
}

int16_t GbE::CPU::fetch16_signed()
{
    int16_t result = read_memory(PC++);
    result = result | (((int16_t) read_memory(PC++)) << 8);

    return result;
}

void GbE::CPU::unknown() const {}

uint16_t GbE::CPU::pop16()
{
    uint16_t sp = read_register16(Reg16_SP::SP);
    uint8_t lower_byte = read_memory(sp);
    uint8_t higher_byte = read_memory(sp + 1);

    uint16_t result = ((uint16_t) higher_byte << 8) | lower_byte; 
    write_register16(Reg16_SP::SP, sp + 2);

    return result;
}

void GbE::CPU::push16(const uint16_t val)
{
    uint16_t sp = read_register16(Reg16_SP::SP);
    write_memory(sp - 1, (uint8_t) (val >> 8));
    write_memory(sp - 2, (uint8_t) (val & 0xFF));

    write_register16(Reg16_SP::SP, sp - 2);
}

void GbE::CPU::init_instruction_table()
{
    itab[0x08] = [this] { load_a16_SP(); };
    itab[0xF8] = [this] { load_HL_SP_s8(); };
    itab[0xF9] = [this] { load_SP_HL(); };
    itab[0xE0] = [this] { load_a8_A(); };
    itab[0xF0] = [this] { load_A_a8(); };
    itab[0xE2] = [this] { load_aC_A(); };
    itab[0xF2] = [this] { load_A_aC(); };
    itab[0xEA] = [this] { load_a16_A(); };
    itab[0xFA] = [this] { load_A_a16(); };

    itab[0xE8] = [this] { add_SP_s8(); };

    itab[0x18] = [this] { jr_s8(); };
    itab[0xC3] = [this] { jp_a16(); };
    itab[0xE9] = [this] { jp_HL(); };
    itab[0xCD] = [this] { call_a16(); };

    itab[0xC9] = [this] { ret(); };
    itab[0xD9] = [this] { reti(); };

    itab[0x00] = [this] { nop(); };
    itab[0x10] = [this] { stop(); };
    itab[0x27] = [this] { daa(); };
    itab[0x37] = [this] { scf(); };
    itab[0x2F] = [this] { cpl(); };
    itab[0x3F] = [this] { ccf(); };
    itab[0x76] = [this] { halt(); };
    itab[0xF3] = [this] { di(); };
    itab[0xFB] = [this] { ei(); };
    itab[0xCB] = [this] { cb(); };

    enum AType {ADD, ADC, SUB, SBC, AND, XOR, OR, CP};

    for (int i = 0; i < 256; i++) {
        /*
         *    INSTRUCTION LAYOUT:
         *
         *      2       3         3
         *   |  X  |    Y    |    Z   |
         *         |  P  | Q |
         *            2    1
         */

        uint8_t instructionCode = i;

        uint8_t x = instructionCode >> 6;
        uint8_t y = (instructionCode >> 3) & 0x7;
        uint8_t z = instructionCode & 0x7;

        uint8_t p = y >> 1;
        uint8_t q = y & 1;

        std::function<void()> inst;

        switch(x) {
        case 0x0:
            switch (z) {
            case 0x0:
                if (p == 0x2)
                    inst = [this, q] { jr_F_s8(GbE::CF::Z, q); };
                else if (p == 0x3)
                    inst = [this, q] { jr_F_s8(GbE::CF::C, q); };
                break;
            case 0x1:
                if (q)
                    inst = [this, p] { add_HL_reg16((Reg16_SP) p); };
                else
                    inst = [this, p] { load_reg16_d16((Reg16_SP) p); };
                break;
            case 0x2:
                if (q)
                    inst = [this, p] { load_A_aReg16((Reg16_Addr) p); };
                else
                    inst = [this, p] { load_aReg16_A((Reg16_Addr) p); };
                break;
            case 0x3:
                if (q)
                    inst = [this, p] { dec_reg16((Reg16_SP) p); };
                else
                    inst = [this, p] { inc_reg16((Reg16_SP) p); };
                break;
            case 0x4:
                inst = [this, y] { inc_reg8((Reg8) y); };
                break;
            case 0x5:
                inst = [this, y] { dec_reg8((Reg8) y); };
                break;
            case 0x6:
                inst = [this, y] { load_reg8_d8((Reg8) y); };
                break;
            case 0x7:
                if (p < 2) {
                    if (q)
                        inst = [this, p] { rra(p & 1); };
                    else
                        inst = [this, p] { rla(p & 1); };
                }
            }

            break;
        case 0x1:
            inst = [this, y, z] { load_reg8_reg8((Reg8) y, (Reg8) z); };

            break;
        case 0x2:
            switch (y) {
            case ADD:
                inst = [this, z] { add_reg8((Reg8) z); };
                break;
            case ADC:
                inst = [this, z] { adc_reg8((Reg8) z); };
                break;
            case SUB:
                inst = [this, z] { sub_reg8((Reg8) z); };
                break;
            case SBC:
                inst = [this, z] { sbc_reg8((Reg8) z); };
                break;
            case AND:
                inst = [this, z] { and_reg8((Reg8) z); };
                break;
            case XOR:
                inst = [this, z] { xor_reg8((Reg8) z); };
                break;
            case OR:
                inst = [this, z] { or_reg8((Reg8) z); };
                break;
            case CP:
                inst = [this, z] { cp_reg8((Reg8) z); };
                break;
            }

            break;
        case 0x3:
            switch(z) {
            case 0x0:
                if (p == 0x0)
                    inst = [this, q] { ret_F(GbE::CF::Z, q); };
                else if (p == 0x1)
                    inst = [this, q] { ret_F(GbE::CF::C, q); };
                break;
            case 0x1:
                if (q == 0x0)
                    inst = [this, p] { pop((Reg16) p); };
                break;
            case 0x2:
                if (p == 0x0)
                    inst = [this, q] { jp_F_a16(GbE::CF::Z, q); };
                else if (p == 0x1)
                    inst = [this, q] { jp_F_a16(GbE::CF::C, q); };
                break;
            case 0x4:
                if (p == 0x0)
                    inst = [this, q] { call_F_a16(GbE::CF::Z, q); };
                else if (p == 0x1)
                    inst = [this, q] { call_F_a16(GbE::CF::C, q); };
                break;
            case 0x5:
                if (q == 0x0)
                    inst = [this, p] { push((Reg16) p); };
                break;
            case 0x6:
                switch (y) {
                case ADD:
                    inst = [this] { add_d8(); };
                    break;
                case ADC:
                    inst = [this] { adc_d8(); };
                    break;
                case SUB:
                    inst = [this] { sub_d8(); };
                    break;
                case SBC:
                    inst = [this] { sbc_d8(); };
                    break;
                case AND:
                    inst = [this] { and_d8(); };
                    break;
                case XOR:
                    inst = [this] { xor_d8(); };
                    break;
                case OR:
                    inst = [this] { or_d8(); };
                    break;
                case CP:
                    inst = [this] { cp_d8(); };
                    break;
                }

                break;
            case 0x7:
                inst = [this, y] { rst_val(y); };
                break;
            }
            break;
        }

        if (inst && !itab[instructionCode]) {
            itab[instructionCode] = inst;
        }
    }

    uint8_t excluded[] = {0xD3, 0xE3, 0xE4, 0xF4, 0xDB, 0xEB, 0xEC, 0xFC, 0xDD, 0xED, 0xFD};

    for (int i = 0; i < 11; i++) {
        itab[excluded[i]] = [this] { unknown(); };
    }
}

void GbE::CPU::init_cb_instruction_table()
{
    for (int i = 0; i < 256; i++) {
        uint8_t inst = i;

        uint8_t x = inst >> 6;
        uint8_t y = (inst >> 3) & 0x7;
        uint8_t z = inst & 0x7;

        uint8_t p = y >> 1;
        uint8_t q = y & 1;

        std::function<void()> instruction;

        Reg8 reg = (Reg8) z;
        uint8_t bit = y;

        switch(x) {
        case 0x0:
            if (y < 4) {
                bool carry = p;

                if (q)
                    instruction = [this, reg, carry] { rr_reg8(reg, carry); };
                else
                    instruction = [this, reg, carry] { rl_reg8(reg, carry); };
            } else if (y < 6) {
                if (q)
                    instruction = [this, reg] { sra_reg8(reg); };
                else
                    instruction = [this, reg] { sla_reg8(reg); };
            } else {
                if (q)
                    instruction = [this, reg] { srl_reg8(reg); };
                else
                    instruction = [this, reg] { swap_reg8(reg); };
            }
            break;
        case 0x1:
            instruction = [this, reg, bit] { bit_reg8(reg, bit); };
            break;
        case 0x2:
            instruction = [this, reg, bit] { res_reg8(reg, bit); };
            break;
        case 0x3:
            instruction = [this, reg, bit] { set_reg8(reg, bit); };
            break;
        }

        cbtab[inst] = instruction;
    }
}

//===========Control==============
void GbE::CPU::nop()
{
    debug_push_inst("nop");
    // do nothing
}

void GbE::CPU::stop()
{
    debug_push_inst("stop");
    stopped = true;
    div = 0;

    fetch();
}

void GbE::CPU::halt()
{
    debug_push_inst("halt");
    halted = true;
}

void GbE::CPU::daa()
{
    debug_push_inst("daa");

    bool N_flag = get_flag(Flag::N);
    bool C_flag = get_flag(Flag::C);
    bool H_flag = get_flag(Flag::H);

    uint8_t A = read_register8(Reg8::A);

    if (!N_flag) {
        if (C_flag || A > 0x99) {
            set_flag(Flag::C, 1);
            A += 0x60;
        }
        if (H_flag || (A & 0x0f) > 0x09) {
            A += 0x6;
        }
    } else {
        if (C_flag) {
            A -= 0x60;
        }
        if (H_flag) {
            A -= 0x6;
        }
    }
    
    write_register8(Reg8::A, A);

    set_flag(Flag::Z, A == 0);
    set_flag(Flag::H, 0);
}

void GbE::CPU::scf()
{
    debug_push_inst("scf");
    set_flag(Flag::N, false);
    set_flag(Flag::H, false);
    set_flag(Flag::C, true);
}

void GbE::CPU::cpl()
{
    debug_push_inst("cpl");
    set_flag(Flag::N, true);
    set_flag(Flag::H, true);
    write_register8(Reg8::A, ~read_register8(Reg8::A));;
}

void GbE::CPU::ccf()
{
    debug_push_inst("ccf");
    set_flag(Flag::N, false);
    set_flag(Flag::H, false);
    set_flag(Flag::C, !get_flag(Flag::C));
}

void GbE::CPU::di()
{
    debug_push_inst("di");
    interrupt_master_enable = 0x00;
}

void GbE::CPU::ei()
{
    debug_push_inst("ei");
    ime_set = true;
}

void GbE::CPU::cb()
{
    uint8_t inst = fetch();

    cbtab[inst]();

    last_cb_opcode = inst;
}
//===========Control==============

//===========Bitwise==============
// if !carry then store the carry, else use it
void GbE::CPU::rla(const bool carry)
{
    if (carry) {
        debug_push_inst("rla");
    } else {
        debug_push_inst("rlca");
    }

    uint8_t data = read_register8(Reg8::A);
    uint8_t bit7 = data >> 7;
    uint8_t result = data << 1;
    if (carry) {
        result = result | get_flag(Flag::C);
    } else {
        result = result | bit7;
    }

    set_flag(Flag::Z, 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, bit7);

    write_register8(Reg8::A, result);
}

void GbE::CPU::rra(const bool carry)
{
    if (carry) {
        debug_push_inst("rra");
    } else {
        debug_push_inst("rrca");
    }

    uint8_t data = read_register8(Reg8::A);
    uint8_t bit0 = data << 7;
    uint8_t result = data >> 1;
    if (carry) {
        result = result | (get_flag(Flag::C) << 7);
    } else {
        result = result | bit0;
    }

    set_flag(Flag::Z, 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, bit0);

    write_register8(Reg8::A, result);
}

// prefixed by $CB
void GbE::CPU::rl_reg8(const Reg8 reg, const bool carry)
{
    if (carry) {
        debug_push_inst("rl");
    } else {
        debug_push_inst("rlc");
    }

    debug_push_reg(reg);

    uint8_t data = read_register8(reg);
    uint8_t bit7 = data >> 7;
    uint8_t result = data << 1;
    if (carry) {
        result = result | get_flag(Flag::C);
    } else {
        result = result | bit7;
    }

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, bit7);

    write_register8(reg, result);
}

void GbE::CPU::rr_reg8(const Reg8 reg, const bool carry)
{
    if (carry) {
        debug_push_inst("rr");
    } else {
        debug_push_inst("rrc");
    }

    debug_push_reg(reg);

    uint8_t data = read_register8(reg);
    uint8_t bit0 = data << 7;
    uint8_t result = data >> 1;
    if (carry) {
        result = result | (get_flag(Flag::C) << 7);
    } else {
        result = result | bit0;
    }

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, bit0);

    write_register8(reg, result);
}

void GbE::CPU::sla_reg8(const Reg8 reg)
{
    debug_push_inst("sla");

    debug_push_reg(reg);
    uint8_t data = read_register8(reg);
    uint8_t bit7 = data >> 7;
    data = data << 1;

    set_flag(Flag::Z, data == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, bit7);

    write_register8(reg, data);
}

void GbE::CPU::sra_reg8(const Reg8 reg)
{
    debug_push_inst("sra");

    debug_push_reg(reg);

    uint8_t data = read_register8(reg);
    uint8_t bit0 = data & 1;
    uint8_t bit7 = data & 0x80;

    data = (data >> 1) | bit7;

    set_flag(Flag::Z, data == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, bit0);

    write_register8(reg, data);
}

void GbE::CPU::srl_reg8(const Reg8 reg)
{
    debug_push_inst("srl");

    debug_push_reg(reg);
    uint8_t data = read_register8(reg);
    set_flag(Flag::C, data & 1);
    data = data >> 1;

    set_flag(Flag::Z, data == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);

    write_register8(reg, data);
}

void GbE::CPU::swap_reg8(const Reg8 reg) {
    debug_push_inst("swap");

    debug_push_reg(reg);
    uint8_t data = read_register8(reg);

    uint8_t high = data << 4;
    uint8_t low = data >> 4;

    data = high | low;

    set_flag(Flag::Z, data == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, 0);

    write_register8(reg, high | low);
}

void GbE::CPU::bit_reg8(const Reg8 reg, const uint8_t bit) {
    debug_push_inst("bit");

    debug_push_arg(std::to_string(bit));
    debug_push_reg(reg);
    uint8_t data = read_register8(reg);

    uint8_t mask = 1 << bit;

    bool bitData = (data & mask) > 0;

    set_flag(Flag::Z, !bitData);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 1);
}

void GbE::CPU::res_reg8(const Reg8 reg, const uint8_t bit) {
    debug_push_inst("res");

    debug_push_arg(std::to_string(bit));
    debug_push_reg(reg);
    uint8_t data = read_register8(reg);
    data = data & ~(1 << bit);

    write_register8(reg, data);
}

void GbE::CPU::set_reg8(const Reg8 reg, const uint8_t bit)
{
    debug_push_inst("set");

    debug_push_arg(std::to_string(bit));
    debug_push_reg(reg);
    uint8_t data = read_register8(reg);

    uint8_t mask = 1 << bit;

    data = (data & ~mask) ^ mask;
    write_register8(reg, data);
}
//===========Bitwise==============

//========16-bit loads============
void GbE::CPU::load_reg16_d16(const Reg16_SP reg) {
    debug_push_inst("ld");
    debug_push_reg(reg);

    uint16_t val = fetch16();

    debug_push_val_u16(val);

    write_register16(reg, val);
}

void GbE::CPU::load_HL_SP_s8() {
    debug_push_inst("ld");
    debug_push_reg(Reg16::HL);

    uint16_t SP = read_register16(Reg16_SP::SP);
    int8_t s8 = fetch_signed();
    debug_push_custom_arg(std::string("SP") 
        + (s8 >= 0 ? ("+" + std::to_string(s8)) 
                   : ("-" + std::to_string((~(s8 - 1))))));

    uint16_t result = SP + s8;

    uint8_t SP_low = (uint8_t) (SP & 0xFF);

    set_flag(Flag::Z, 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, half_carry8(SP_low, (uint8_t) ((int) s8)));
    set_flag(Flag::C,      carry8(SP_low, (uint8_t) ((int) s8)));

    write_register16(Reg16::HL, result);
}

void GbE::CPU::load_SP_HL() {
    debug_push_inst("ld");
    debug_push_reg(Reg16_SP::SP);
    debug_push_reg(Reg16::HL);

    write_register16(Reg16_SP::SP, read_register16(Reg16::HL));
}

void GbE::CPU::load_a16_SP()
{
    debug_push_inst("ld");
    uint16_t addr = fetch16();

    debug_push_addr16(addr);
    debug_push_reg(Reg16_SP::SP);

    uint16_t val = read_register16(Reg16_SP::SP);
    write_memory(addr, (uint8_t) val);
    write_memory(addr + 1, (uint8_t) (val >> 8));
}

void GbE::CPU::pop(const Reg16 reg)
{
    debug_push_inst("pop");
    debug_push_reg(reg);
    write_register16(reg, pop16());
}

void GbE::CPU::push(const Reg16 reg)
{
    debug_push_inst("push");
    debug_push_reg(reg);

    push16(read_register16(reg));
}
//========16-bit loads============

//=========8-bit loads============
void GbE::CPU::load_reg8_d8(const Reg8 reg)
{
    debug_push_inst("ld");
    debug_push_reg(reg);

    uint8_t data = fetch();
    debug_push_val_u8(data);

    write_register8(reg, data);
}

void GbE::CPU::load_aReg16_A(const Reg16_Addr reg)
{
    debug_push_inst("ld");
    debug_push_reg(reg);
    debug_push_reg(Reg8::A);

    uint16_t addr = 0;

    if (reg == Reg16_Addr::BC) {
        addr = read_register16(Reg16::BC);
    } else if (reg == Reg16_Addr::DE) {
        addr = read_register16(Reg16::DE);
    } else if (reg == Reg16_Addr::HLD) {
        addr = read_register16(Reg16::HL);
        write_register16(Reg16::HL, addr - 1);
    } else if (reg == Reg16_Addr::HLI) {
        addr = read_register16(Reg16::HL);
        write_register16(Reg16::HL, addr + 1);
    }

    write_memory(addr, read_register8(Reg8::A));
}

void GbE::CPU::load_A_aReg16(const Reg16_Addr reg)
{
    debug_push_inst("ld");
    debug_push_reg(Reg8::A);
    debug_push_reg(reg);

    uint16_t addr = 0;

    if (reg == Reg16_Addr::BC) {
        addr = read_register16(Reg16::BC);
    } else if (reg == Reg16_Addr::DE) {
        addr = read_register16(Reg16::DE);
    } else if (reg == Reg16_Addr::HLD) {
        addr = read_register16(Reg16::HL);
        write_register16(Reg16::HL, addr - 1);
    } else if (reg == Reg16_Addr::HLI) {
        addr = read_register16(Reg16::HL);
        write_register16(Reg16::HL, addr + 1);
    }

    write_register8(Reg8::A, read_memory(addr));
}

void GbE::CPU::load_reg8_reg8(const Reg8 reg1, const Reg8 reg2)
{
    debug_push_inst("ld");
    debug_push_reg(reg1);
    debug_push_reg(reg2);

    uint8_t data = read_register8(reg2);
    write_register8(reg1, data);
}

void GbE::CPU::load_a8_A()
{
    debug_push_inst("ld");

    uint8_t val = fetch();
    debug_push_addr8(val);
    debug_push_reg(Reg8::A);

    uint16_t addr = 0xFF00 + val;
    write_memory(addr, read_register8(Reg8::A));
}

void GbE::CPU::load_A_a8()
{
    debug_push_inst("ld");

    uint8_t val = fetch();
    debug_push_reg(Reg8::A);
    debug_push_addr8(val);

    uint16_t addr = 0xFF00 + val;
    write_register8(Reg8::A, read_memory(addr));
}

void GbE::CPU::load_aC_A()
{
    debug_push_inst("ld");
    debug_push_custom_arg("(C)");
    debug_push_reg(Reg8::A);

    uint16_t addr = 0xFF00 + read_register8(Reg8::C);
    write_memory(addr, read_register8(Reg8::A));
}

void GbE::CPU::load_A_aC()
{
    debug_push_inst("ld");
    debug_push_reg(Reg8::A);
    debug_push_custom_arg("(C)");

    uint16_t addr = 0xFF00 + read_register8(Reg8::C);
    write_register8(Reg8::A, read_memory(addr));
}

void GbE::CPU::load_a16_A()
{
    debug_push_inst("ld");

    uint16_t addr = fetch16();
    debug_push_addr16(addr);
    debug_push_reg(Reg8::A);

    write_memory(addr, read_register8(Reg8::A));
}

void GbE::CPU::load_A_a16()
{
    debug_push_inst("ld");

    uint16_t addr = fetch16();
    debug_push_reg(Reg8::A);
    debug_push_addr16(addr);

    write_register8(Reg8::A, read_memory(addr));
}
//=========8-bit loads============

//======16-bit arithmetic=========
void GbE::CPU::inc_reg16(const Reg16_SP reg16)
{
    debug_push_inst("inc");
    debug_push_reg(reg16);

    write_register16(reg16, read_register16(reg16) + 1);
}

void GbE::CPU::dec_reg16(const Reg16_SP reg16)
{
    debug_push_inst("dec");
    debug_push_reg(reg16);

    write_register16(reg16, read_register16(reg16) - 1);
}

void GbE::CPU::add_HL_reg16(const Reg16_SP reg16)
{
    debug_push_inst("add");
    debug_push_reg(Reg16::HL);
    debug_push_reg(reg16);

    uint16_t val1 = read_register16(Reg16::HL);
    uint16_t val2 = read_register16(reg16);
    uint16_t result = val1 + val2;

    set_flag(Flag::N, 0);
    set_flag(Flag::H, half_carry16(val1, val2));
    set_flag(Flag::C, carry16(val1, val2));

    write_register16(Reg16::HL, result);
}

void GbE::CPU::add_SP_s8()
{
    debug_push_inst("add");
    debug_push_reg(Reg16_SP::SP);

    uint16_t SP = read_register16(Reg16_SP::SP);
    int8_t s8 = fetch_signed();

    debug_push_val_s8(s8);

    uint16_t result = SP + s8;

    uint8_t SP_low = (uint8_t) (SP & 0xFF);

    set_flag(Flag::Z, 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, half_carry8(SP_low, (uint8_t) ((int) s8)));
    set_flag(Flag::C,      carry8(SP_low, (uint8_t) ((int) s8)));

    write_register16(Reg16_SP::SP, result);
}
//======16-bit arithmetic=========

//=======8-bit arithmetic=========
void GbE::CPU::inc_reg8(const Reg8 reg)
{
    debug_push_inst("inc");
    debug_push_reg(reg);

    uint8_t val = read_register8(reg);
    uint8_t result = val + 1;
    write_register8(reg, result);

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, half_carry8(val, 1));
}

void GbE::CPU::dec_reg8(const Reg8 reg)
{
    debug_push_inst("dec");
    debug_push_reg(reg);

    uint8_t val = read_register8(reg);
    uint8_t result = val - 1;
    write_register8(reg, result);

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 1);
    set_flag(Flag::H, half_carry_sub8(val, 1));
}

void GbE::CPU::add_reg8(const Reg8 reg)
{
    debug_push_inst("add");
    debug_push_reg(Reg8::A);
    debug_push_reg(reg);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);
    uint8_t result = val1 + val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, half_carry8(val1, val2));
    set_flag(Flag::C, carry8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::CPU::add_d8()
{
    debug_push_inst("add");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();

    debug_push_val_u8(val2);

    uint8_t result = val1 + val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, half_carry8(val1, val2));
    set_flag(Flag::C, carry8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::CPU::adc_reg8(const Reg8 reg)
{
    debug_push_inst("adc");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);

    bool C = get_flag(Flag::C);

    debug_push_reg(reg);
    uint8_t result = (uint8_t) (val1 + val2 + C);

    uint8_t half_carry = half_carry8(val1, val2);
    half_carry = half_carry | half_carry8(val1 + val2, C);

    uint8_t carry = carry8(val1, val2);
    carry = carry | carry8(val1 + val2, C);

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, half_carry);
    set_flag(Flag::C, carry);

    write_register8(Reg8::A, result);
}

void GbE::CPU::adc_d8()
{
    debug_push_inst("add");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();

    bool C = get_flag(Flag::C);

    debug_push_val_u8(val2);
    uint8_t result = val1 + val2 + C;

    uint8_t half_carry = half_carry8(val1, val2);
    half_carry = half_carry | half_carry8(val1 + val2, C);

    uint8_t carry = carry8(val1, val2);
    carry = carry | carry8(val1 + val2, C);

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, half_carry);
    set_flag(Flag::C, carry);

    write_register8(Reg8::A, result);
}

void GbE::CPU::sub_reg8(const Reg8 reg)
{
    debug_push_inst("sub");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);

    debug_push_reg(reg);
    uint8_t result = val1 - val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 1);
    set_flag(Flag::H, half_carry_sub8(val1, val2));
    set_flag(Flag::C, carry_sub8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::CPU::sub_d8()
{
    debug_push_inst("sub");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();

    debug_push_val_u8(val2);
    uint8_t result = val1 - val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 1);
    set_flag(Flag::H, half_carry_sub8(val1, val2));
    set_flag(Flag::C, carry_sub8(val1, val2));

    write_register8(Reg8::A, result);
}

void GbE::CPU::sbc_reg8(const Reg8 reg)
{
    debug_push_inst("sbc");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);

    bool C = get_flag(Flag::C);

    debug_push_reg(reg);
    uint8_t result = val1 - val2 - C;

    uint8_t half_carry = half_carry_sub8(val1, val2);
    half_carry = half_carry | half_carry_sub8(val1 - val2, C);

    uint8_t carry = carry_sub8(val1, val2);
    carry = carry | carry_sub8(val1 - val2, C);

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 1);
    set_flag(Flag::H, half_carry);
    set_flag(Flag::C, carry);

    write_register8(Reg8::A, result);
}

void GbE::CPU::sbc_d8()
{
    debug_push_inst("sbc");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();

    bool C = get_flag(Flag::C);

    debug_push_val_u8(val2);
    uint8_t result = val1 - val2 - C;

    uint8_t half_carry = half_carry_sub8(val1, val2);
    half_carry = half_carry | half_carry_sub8(val1 - val2, C);

    uint8_t carry = carry_sub8(val1, val2);
    carry = carry | carry_sub8(val1 - val2, C);

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 1);
    set_flag(Flag::H, half_carry);
    set_flag(Flag::C, carry);

    write_register8(Reg8::A, result);
}

void GbE::CPU::and_reg8(const Reg8 reg)
{
    debug_push_inst("and");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);

    debug_push_reg(reg);
    uint8_t result = val1 & val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 1);
    set_flag(Flag::C, 0);

    write_register8(Reg8::A, result);
}

void GbE::CPU::and_d8()
{
    debug_push_inst("and");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();

    debug_push_val_u8(val2);
    uint8_t result = val1 & val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 1);
    set_flag(Flag::C, 0);

    write_register8(Reg8::A, result);
}

void GbE::CPU::xor_reg8(const Reg8 reg)
{
    debug_push_inst("xor");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);

    debug_push_reg(reg);
    uint8_t result = val1 ^ val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, 0);

    write_register8(Reg8::A, result);
}

void GbE::CPU::xor_d8()
{
    debug_push_inst("xor");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();

    debug_push_val_u8(val2);
    uint8_t result = val1 ^ val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, 0);

    write_register8(Reg8::A, result);
}

void GbE::CPU::or_reg8(const Reg8 reg)
{
    debug_push_inst("or");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);

    debug_push_reg(reg);
    uint8_t result = val1 | val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, 0);

    write_register8(Reg8::A, result);
}

void GbE::CPU::or_d8()
{
    debug_push_inst("or");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();

    debug_push_val_u8(val2);
    uint8_t result = val1 | val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 0);
    set_flag(Flag::H, 0);
    set_flag(Flag::C, 0);

    write_register8(Reg8::A, result);
}

void GbE::CPU::cp_reg8(const Reg8 reg)
{
    debug_push_inst("cp");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = read_register8(reg);

    debug_push_reg(reg);
    uint8_t result = val1 - val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 1);
    set_flag(Flag::H, half_carry_sub8(val1, val2));
    set_flag(Flag::C, carry_sub8(val1, val2));
}

void GbE::CPU::cp_d8()
{
    debug_push_inst("cp");
    debug_push_reg(Reg8::A);

    uint8_t val1 = read_register8(Reg8::A);
    uint8_t val2 = fetch();

    debug_push_val_u8(val2);
    uint8_t result = val1 - val2;

    set_flag(Flag::Z, result == 0);
    set_flag(Flag::N, 1);
    set_flag(Flag::H, half_carry_sub8(val1, val2));
    set_flag(Flag::C, carry_sub8(val1, val2));
}
//=======8-bit arithmetic=========

//============Jumps===============
// invoke if flag == val
// relative jumps
void GbE::CPU::jr_F_s8(const GbE::CF flag, const bool val)
{
    debug_push_inst("jr");
    if (val)
        debug_push_flag(flag);
    else
        debug_push_flag_n(flag);

    int8_t s = fetch_signed();
    debug_push_val_s8(s);

    bool flagVal;
    if (flag == GbE::CF::Z) {
        flagVal = get_flag(Flag::Z);
    } else {
        flagVal = get_flag(Flag::C);
    }

    if (flagVal == val) {
        PC += s;
    }
}

void GbE::CPU::jr_s8()
{
    debug_push_inst("jr");

    int8_t val = fetch_signed();
    debug_push_val_s8(val);

    PC += val;

    if (val == -2) [[unlikely]] {
        stopped = true;
    }
}

// absolute jumps
void GbE::CPU::jp_F_a16(const GbE::CF flag, const bool val)
{
    debug_push_inst("jp");
    if (val)
        debug_push_flag(flag);
    else
        debug_push_flag_n(flag);

    uint16_t addr = fetch16();
    debug_push_addr16(addr);

    bool flagVal;
    if (flag == GbE::CF::Z) {
        flagVal = get_flag(Flag::Z);
    } else {
        flagVal = get_flag(Flag::C);
    }

    if (flagVal == val) {
        PC = addr;
    }
}

void GbE::CPU::jp_a16()
{
    debug_push_inst("jp");
    uint16_t val = fetch16();
    debug_push_addr16(val);

    PC = val;
}

void GbE::CPU::jp_HL()
{
    debug_push_inst("jp");
    debug_push_reg(Reg16::HL);
    PC = read_register16(Reg16::HL);
}

// calls
void GbE::CPU::call_F_a16(const GbE::CF flag, const bool val)
{
    debug_push_inst("call");
    if (val)
        debug_push_flag(flag);
    else
        debug_push_flag_n(flag);

    uint16_t addr = fetch16();
    debug_push_addr16(addr);

    bool flagVal;
    if (flag == GbE::CF::Z) {
        flagVal = get_flag(Flag::Z);
    } else {
        flagVal = get_flag(Flag::C);
    }

    if (flagVal == val) {
        push16(PC);
        PC = addr;
    }
}

void GbE::CPU::call_a16()
{
    debug_push_inst("call");
    uint16_t addr = fetch16();
    debug_push_addr16(addr);

    push16(PC);
    PC = addr;
}

// returns
void GbE::CPU::ret_F(const GbE::CF flag, const bool val)
{
    debug_push_inst("ret");
    if (val)
        debug_push_flag(flag);
    else
        debug_push_flag_n(flag);

    bool flagVal;
    if (flag == GbE::CF::Z) {
        flagVal = get_flag(Flag::Z);
    } else {
        flagVal = get_flag(Flag::C);
    }

    if (flagVal == val) {
        PC = pop16();
    }
}

void GbE::CPU::ret()
{
    debug_push_inst("ret");
    PC = pop16();
}

void GbE::CPU::reti()
{
    debug_push_inst("reti");
    interrupt_master_enable = 0x01;
    PC = pop16();
}

// resets
void GbE::CPU::rst_val(const uint8_t val)
{
    debug_push_inst("rst");
    debug_push_val_u8(val);

    push16(PC);

    uint16_t addr = ((val >> 1) << 4) | ((val & 1) << 3);
    PC = addr;
}
//============Jumps===============
