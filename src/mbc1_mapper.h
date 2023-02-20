#ifndef MBC1_MAPPER_H
#define MBC1_MAPPER_H

#include "memorymapper.h"

class MBC1_Mapper : public MemoryMapper
{
public:
    MBC1_Mapper(size_t rom_size = 0, size_t ram_size = 0);

    bool handle_rom0_write(uint16_t address, uint8_t value) override;
    bool handle_romN_write(uint16_t address, uint8_t value) override;
    bool handle_ram_write(uint16_t address, uint8_t value) override;

    uint16_t convert_rom0_addr(uint16_t address) const override;
    uint16_t convert_romN_addr(uint16_t address) const override;
    uint16_t convert_ram_addr(uint16_t address) const override;

private:
    bool use_ram_banking = true;

    bool ram_enabled;
    bool banking_mode;

    uint8_t rom_bank_number = 1;

    uint16_t rom_offset = 0;
    uint16_t ram_offset = 0;

    size_t rom_size;
    size_t ram_size;
};

#endif // MBC1_MAPPER_H
