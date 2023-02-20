#ifndef MEMORYMAPPER_H
#define MEMORYMAPPER_H


#include <cstdint>
class MemoryMapper
{
public:
    MemoryMapper();

    virtual bool handle_rom0_write(uint16_t address, uint8_t value);
    virtual bool handle_romN_write(uint16_t address, uint8_t value);
    virtual bool handle_ram_write(uint16_t address, uint8_t value);

    virtual uint16_t convert_rom0_addr(uint16_t address) const;
    virtual uint16_t convert_romN_addr(uint16_t address) const;
    virtual uint16_t convert_ram_addr(uint16_t address)  const;
};

#endif // MEMORYMAPPER_H
