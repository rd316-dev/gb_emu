#include "gameboyemulator.h"

#include <iostream>
#include <string>

void unimplemented(std::string name)
{
    std::cerr << name << " is unimplemented" << std::endl;
}

GbE::GbE()
{

}

void GbE::load_boot_rom(size_t size, std::shared_ptr<uint8_t> boot_rom)
{
    if (size < 0xFF) {
        std::cerr << "error occurred while loading boot rom: boot rom size is lower than 256 bytes";
        return;
    }
}

void GbE::load_rom(size_t size, std::shared_ptr<uint8_t> rom)
{
    // minimal size for a rom with a valid header is 335 bytes
    // 0x14F is the last byte of header
    if (size < 0x14F) {
        std::cerr << "error occurred while loading rom: rom size is lower than 335 bytes" << std::endl;
        return;
    }

    this->rom_size = size;
    this->rom = rom;

    uint8_t* p = rom.get();

    // copy header to structure for easier read
    CartidgeHeader header;
    memcpy(&(header.entry_point),       p + 0x100, 4);
    memcpy(header.logo,                 p + 0x104, 48);
    memcpy(header.title,                p + 0x134, 16);
    memcpy(&(header.licensee_code),     p + 0x144, 2);
    memcpy(&(header.sgb_flag),          p + 0x146, 1);
    memcpy(&(header.cartridge_type),    p + 0x147, 1);
    memcpy(&(header.rom_size),          p + 0x148, 1);
    memcpy(&(header.ram_size),          p + 0x149, 1);
    memcpy(&(header.destination_code),  p + 0x14A, 1);
    memcpy(&(header.old_licensee_code), p + 0x14B, 1);
    memcpy(&(header.rom_version),       p + 0x14C, 1);
    memcpy(&(header.header_checksum),   p + 0x14D, 1);
    memcpy(&(header.global_checksum),   p + 0x14E, 2);
}

void GbE::execute()
{
    used_cycles = 0;

    uint8_t inst = fetch();
    itab[inst]();

    machine_cycle += used_cycles;

    int used = used_cycles;

    if (dma_started)
        dma_cycle(used);
    if (ppu_enabled)
        ppu_cycle(used);
}

void GbE::dma_cycle(uint8_t machine_cycles)
{
    for (int i = 0; i < machine_cycles; i++) {
        write_memory(dma_dest_addr, read_memory(dma_src_addr));
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

void GbE::ppu_cycle(uint8_t machine_cycles)
{
    int current_scanline = current_dot / 456;
    int scanline_dot = current_dot % 456;

    if (current_scanline <= 143) {
        if (scanline_dot <= 80) {
            // oam scan
        } else {
            if (fetcher_step == 0) {
                bool inside_window_x = (fetcher_x * 8) >= wx && lcdc.window_enabled;
                bool inside_window_y = (fetcher_y * 8) >= wy && lcdc.window_enabled;

                uint16_t addr = 0x9800;

                if ((inside_window_x && lcdc.window_tile_map_area) || (!inside_window_x && lcdc.bg_tile_map_area))
                    addr = 0x9C00;

                if (inside_window_x && inside_window_y) {
                    // todo: I don't understand
                } else {
                    int x = ((scx / 8) + fetcher_x) & 0x1F;
                }
            }
        }
    } else {
        // vblank
    }
}

void GbE::start_dma(uint8_t src)
{
    dma_src_addr = src << 8;
    dma_dest_addr = 0xFE00;
    dma_started = true;
}

void GbE::unknown() {}

void GbE::init_instruction_table()
{
    itab[0x08] = [this] { load_a16_SP(); };
    itab[0x08] = [this] { load_a16_SP(); };
    itab[0xF8] = [this] { load_HL_SP_s8(); };
    itab[0xE0] = [this] { load_a8_A(); };
    itab[0xF2] = [this] { load_aC_A(); };
    itab[0xEA] = [this] { load_a16_A(); };
    itab[0xFA] = [this] { load_A_a16(); };

    itab[0xE8] = [this] { add_SP_s8(); };

    itab[0x18] = [this] { jr_s8(); };
    itab[0xC3] = [this] { jp_a16(); };
    itab[0xE9] = [this] { jp_HL(); };
    itab[0xCD] = [this] { call_a16(); };
    itab[0xC8] = [this] { ret(); };
    itab[0xD8] = [this] { reti(); };

    itab[0x00] = [this] { nop(); };
    itab[0x10] = [this] { stop(); };
    itab[0x27] = [this] { daa(); };
    itab[0x37] = [this] { scf(); };
    itab[0x2F] = [this] { cpl(); };
    itab[0x3F] = [this] { ccf(); };
    itab[0x76] = [this] { halt(); };
    itab[0xF3] = [this] { di(); };
    itab[0xFA] = [this] { ei(); };
    itab[0xCB] = [this] { cb(); };

    enum AType {ADD, ADC, SUB, SBC, AND, XOR, OR, CP};

    for (int i = 0; i < 256; i++) {
        uint8_t inst = i;

        uint8_t x = inst >> 6;
        uint8_t y = (inst >> 3) & 0x7;
        uint8_t z = inst & 0x7;

        uint8_t p = y >> 1;
        uint8_t q = y & 1;

        std::function<void()> instruction;

        switch(x) {
        case 0x0:
            switch (z) {
            case 0x0:
                if (p == 0x2)
                    instruction = [this, q] { jr_F_s8(CF::Z, q); };
                else if (p == 0x3)
                    instruction = [this, q] { jr_F_s8(CF::C, q); };

                break;
            case 0x1:
                if (q)
                    instruction = [this, p] { add_HL_reg16((Reg16_SP) p); };
                else
                    instruction = [this, p] { load_reg16_d16((Reg16_SP) p); };
                break;
            case 0x2:
                if (q)
                    instruction = [this, p] { load_A_aReg16((Reg16_Addr) p); };
                else
                    instruction = [this, p] { load_aReg16_A((Reg16_Addr) p); };
                break;
            case 0x3:
                if (q)
                    instruction = [this, p] { dec_reg16((Reg16_SP) p); };
                else
                    instruction = [this, p] { inc_reg16((Reg16_SP) p); };
                break;
            case 0x4:
                instruction = [this, y] { inc_reg8((Reg8) y); };
                break;
            case 0x5:
                instruction = [this, y] { inc_reg8((Reg8) y); };
                break;
            case 0x6:
                instruction = [this, y] { load_reg8_d8((Reg8) y); };
                break;
            }

            break;
        case 0x1:
            instruction = [this, y, z] { load_reg8_reg8((Reg8) y, (Reg8) z); };

            break;
        case 0x2:
            switch (y) {
            case ADD:
                instruction = [this, z] { add_reg8((Reg8) z); };
                break;
            case ADC:
                instruction = [this, z] { adc_reg8((Reg8) z); };
                break;
            case SUB:
                instruction = [this, z] { sub_reg8((Reg8) z); };
                break;
            case SBC:
                instruction = [this, z] { sbc_reg8((Reg8) z); };
                break;
            case AND:
                instruction = [this, z] { and_reg8((Reg8) z); };
                break;
            case XOR:
                instruction = [this, z] { xor_reg8((Reg8) z); };
                break;
            case OR:
                instruction = [this, z] { or_reg8((Reg8) z); };
                break;
            case CP:
                instruction = [this, z] { cp_reg8((Reg8) z); };
                break;
            }

            break;
        case 0x3:
            switch(z) {
            case 0x0:
                if (p == 0x0)
                    instruction = [this, q] { ret_F(CF::Z, q); };
                else if (p == 0x1)
                    instruction = [this, q] { ret_F(CF::C, q); };
                break;
            case 0x1:
                if (q == 0x0)
                    instruction = [this, p] { pop((Reg16) p); };
                break;
            case 0x2:
                if (p == 0x0)
                    instruction = [this, q] { jp_F_a16(CF::Z, q); };
                else if (p == 0x1)
                    instruction = [this, q] { jp_F_a16(CF::C, q); };
                break;
            case 0x4:
                if (q == 0x0)
                    instruction = [this, p] { push((Reg16) p); };
                break;
            case 0x5:
                if (p == 0x0)
                    instruction = [this, q] { call_F_a16(CF::Z, q); };
                else if (p == 0x1)
                    instruction = [this, q] { call_F_a16(CF::C, q); };
                break;
            case 0x6:
                switch (y) {
                case ADD:
                    instruction = [this] { add_d8(); };
                    break;
                case ADC:
                    instruction = [this] { adc_d8(); };
                    break;
                case SUB:
                    instruction = [this] { sub_d8(); };
                    break;
                case SBC:
                    instruction = [this] { sbc_d8(); };
                    break;
                case AND:
                    instruction = [this] { and_d8(); };
                    break;
                case XOR:
                    instruction = [this] { xor_d8(); };
                    break;
                case OR:
                    instruction = [this] { or_d8(); };
                    break;
                case CP:
                    instruction = [this] { cp_d8(); };
                    break;
                }

                break;
            case 0x7:
                instruction = [this, y] { rst_val(y); };
                break;
            }
            break;
        }

        itab[inst] = instruction;
    }

    uint8_t excluded[] = {0xD3, 0xE3, 0xE4, 0xF4, 0xDB, 0xEB, 0xEC, 0xFC, 0xDD, 0xED, 0xFD};

    for (int i = 0; i < 11; i++) {
        itab[excluded[i]] = [this] { unknown(); };
    }

}

void GbE::init_cb_instruction_table()
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

uint8_t GbE::read_memory(uint16_t addr)
{
    used_cycles++;

    if (addr <= 0x00FF && boot_rom_mapped) return boot_rom.get()[addr];
    else if (addr <= 0x3FFF) return rom.get()[mapper.convert_rom0_addr(addr)];
    else if (addr <= 0x7FFF) return rom.get()[mapper.convert_romN_addr(addr)];
    else if (addr <= 0x9FFF) return vram.get()[addr - 0x8000];
    else if (addr <= 0xBFFF) return external_ram.get()[mapper.convert_ram_addr(addr)];
    else if (addr <= 0xCFFF) return wram.get()[addr - 0xC000];
    else if (addr <= 0xDFFF) return wram.get()[addr - 0xC000];
    else if (addr <= 0xFDFF) return wram.get()[addr - 0xC000];
    else if (addr <= 0xFE9F) return oam.get()[addr - 0xFE00];
    else if (addr <= 0xFEFF) return 0xFF; // not usable
    else if (addr <= 0xFF7F) {
        if (addr <= 0xFF00) {
            // joypad
        } else if (addr <= 0xFF02) {
            // serial transfer
        } else if (addr <= 0xFF07) {
            // timer and divider
        } else if (addr <= 0xFF26) {
            // audio
        } else if (addr <= 0xFF3F) {
            // wave pattern
        } else if (addr <= 0xFF4B) {
            // lcd
        } else if (addr <= 0xFF4F) {
            // [cgb] vram bank select
        } else if (addr <= 0xFF50) {
            // disable boot rom
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
    else if (addr <= 0xFFFE) return hram.get()[addr - 0xFF80];
    else return interrupt_enabled;

    return 0xFF;
}

void GbE::write_memory(uint16_t addr, uint8_t val)
{
    used_cycles++;

    if      (addr <= 0x3FFF) mapper.handle_rom0_write(addr, val);
    else if (addr <= 0x7FFF) mapper.handle_romN_write(addr, val);
    else if (addr <= 0x9FFF) vram.get()[addr - 0x8000] = val;
    else if (addr <= 0xBFFF) {
        if (!mapper.handle_ram_write(addr, val))
            external_ram.get()[mapper.convert_ram_addr(addr)] = val;
    }
    else if (addr <= 0xCFFF) wram.get()[addr - 0xC000] = val;
    else if (addr <= 0xDFFF) wram.get()[addr - 0xC000] = val;
    else if (addr <= 0xFDFF) wram.get()[addr - 0xC000] = val;
    else if (addr <= 0xFE9F) oam.get()[addr - 0xFE00] = val;
    else if (addr <= 0xFEFF) unknown();
    else if (addr <= 0xFF80) {
        if (addr <= 0xFF00) {
            // joypad
        } else if (addr <= 0xFF02) {
            // serial transfer
        } else if (addr <= 0xFF07) {
            // timer and divider
        } else if (addr <= 0xFF26) {
            // audio
        } else if (addr <= 0xFF3F) {
            // wave pattern
        } else if (addr <= 0xFF4B) {
            // lcd
            switch (addr) {
            case 0xFF40: // lcdc
                break;
            case 0xFF41: // stat
                break;
            case 0xFF42: // scx
                scx = val;
                break;
            case 0xFF43:
                scy = val; // scy
                break;
            case 0xFF44:
                break;
            case 0xFF45:
                break;
            case 0xFF46:
                start_dma(val);
                break;
            case 0xFF47:
                break;
            case 0xFF48:
                break;
            case 0xFF49:
                break;
            case 0xFF4A:
                break;
            case 0xFF4B:
                break;
            }
        } else if (addr <= 0xFF4F) {
            // [cgb] vram bank select
        } else if (addr <= 0xFF50) {
            // disable boot rom
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
    else if (addr <= 0xFFFF) hram.get()[addr - 0xFF80] = val;
    else                     interrupt_enabled = val;
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
    if (reg == Reg16_SP::SP)
        return SP;

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

void GbE::set_Z(bool val)
{
    // bit 7
    if (val)
        registers[6] = registers[6] | 0x80;
    else
        registers[6] = registers[6] ^ 0x80;
}

void GbE::set_N(bool val)
{
     // bit 6
    if (val)
        registers[6] = registers[6] | 0x40;
    else
        registers[6] = registers[6] ^ 0x40;
}

void GbE::set_H(bool val)
{
     // bit 5
    if (val)
        registers[6] = registers[6] | 0x20;
    else
        registers[6] = registers[6] ^ 0x20;
}

void GbE::set_C(bool val)
{
     // bit 4
    if (val)
        registers[6] = registers[6] | 0x10;
    else
        registers[6] = registers[6] ^ 0x10;
}

uint8_t GbE::get_Z()
{
    return registers[6] >> 7;
}

uint8_t GbE::get_N()
{
    return (registers[6] >> 6) & 1;
}

uint8_t GbE::get_H()
{
    return (registers[6] >> 5) & 1;
}

uint8_t GbE::get_C()
{
    return (registers[6] >> 4) & 1;
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
    write_register8(Reg8::A, ~read_register8(Reg8::A));;
}

void GbE::ccf()
{
    set_N(false);
    set_H(false);
    set_C(!get_C());
}

void GbE::di()
{
    interrupt_enabled = 0x00;
}

void GbE::ei()
{
    interrupt_enabled = 0x01;
}

void GbE::cb()
{
    uint8_t inst = fetch();
    cbtab[inst]();
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

void GbE::load_HL_SP_s8() {
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

void GbE::load_aReg16_A(Reg16_Addr reg)
{
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

void GbE::load_A_aReg16(Reg16_Addr reg)
{
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

void GbE::inc_reg8(Reg8 reg)
{
    write_register8(reg, read_register8(reg) + 1);
}

void GbE::dec_reg8(Reg8 reg)
{
    write_register8(reg, read_register8(reg) - 1);
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

void GbE::reti()
{
    ei();
    ret();
}

// resets
void GbE::rst_val(uint8_t val)
{
    push16(PC);
    PC = read_memory(val);
}
//============Jumps===============
