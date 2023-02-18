#include "memorymapper.h"

MemoryMapper::MemoryMapper()
{

}

bool MemoryMapper::handle_rom0_write(uint16_t addr, uint8_t val)
{
    return true;
}

bool MemoryMapper::handle_romN_write(uint16_t addr, uint8_t val)
{
    return true;
}

bool MemoryMapper::handle_ram_write(uint16_t addr, uint8_t val)
{
    return true;
}

uint16_t MemoryMapper::convert_rom0_addr(uint16_t address)
{
    return address;
}

uint16_t MemoryMapper::convert_romN_addr(uint16_t address)
{
    return address;
}

uint16_t MemoryMapper::convert_ram_addr(uint16_t address)
{
    return (address - 0xA000);
}