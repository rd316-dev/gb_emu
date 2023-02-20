#include "mbc1_mapper.h"

MBC1_Mapper::MBC1_Mapper(size_t rom_size, size_t ram_size)
{
    this->rom_size = rom_size;
    this->ram_size = ram_size;
}

bool MBC1_Mapper::handle_rom0_write(uint16_t addr, uint8_t val)
{
    if (addr <= 0x1FFF) {
        ram_enabled = ((val & 0x0F) == 0x0A);
    } else {
        uint8_t bank_value = val & 0x1F;

        if (!bank_value)
            bank_value = 0x01;

        rom_bank_number = (rom_bank_number ^ 0x1F) | bank_value;
        rom_offset = rom_bank_number * 0x4000;
    }

    return true;
}

bool MBC1_Mapper::handle_romN_write(uint16_t addr, uint8_t val)
{
    uint8_t ram_bank_number;

    if (addr <= 0x5FFF) {
        if (use_ram_banking) {
            ram_offset = val * 0x2000;
        } else {
            rom_bank_number = (rom_bank_number ^ 0x60) | ((val & 0x03) << 5);
            rom_offset = rom_bank_number * 0x4000;
        }
    } else {
        banking_mode = val;
    }

    return true;
}

bool MBC1_Mapper::handle_ram_write(uint16_t addr, uint8_t val)
{
    return !use_ram_banking;
}

uint16_t MBC1_Mapper::convert_rom0_addr(uint16_t address) const
{
    return address;
}

uint16_t MBC1_Mapper::convert_romN_addr(uint16_t address) const
{
    return (address - 0x4000) + rom_offset;
}

uint16_t MBC1_Mapper::convert_ram_addr(uint16_t address) const
{
    return (address - 0xA000) + ram_offset;
}
