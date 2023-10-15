#include "utils.h"

#include <iomanip>
#include <iostream>

std::string utils::to_hex(uint32_t val, int length)
{
    std::stringstream stream;
    stream << std::setfill('0') << std::setw(length)
           << std::hex << val;

    return stream.str();
}

uint32_t utils::from_hex(const std::string &str)
{
    const std::string hex_chars = "0123456789ABCDEF";

    uint32_t val = 0;
    for (const auto &c : str) {
        uint8_t digit;
        bool found = false;

        for (int i = 0; i < hex_chars.size(); i++) {
            const char pc = hex_chars.at(i);
            if (c > 'F') {
                found = (c - 32) == pc;
            } else {
                found = c == pc;
            }

            if (found) {
                digit = i;
                break;
            }
        }

        if (!found) {
            std::cout << "error: invalid hex character \'" 
                      << c << "\' in string \'" << str << "\'" 
                      << std::endl;
            return 0;
        }

        val = (val << 4) | digit;
    }

    return val;
}