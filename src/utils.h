#ifndef UTILS_H
#define UTILS_H

#include <cstdint>
#include <string>

namespace utils {
std::string to_hex(uint32_t val, int length);

uint32_t from_hex(const std::string &str);
}

#endif