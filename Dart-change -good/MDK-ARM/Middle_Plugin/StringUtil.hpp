#ifndef _StringUtil_hpp
#define _StringUtil_hpp

#include <stdint.h>
#include <string>
#include <vector>

uint8_t hexCharToByte(char c);
std::vector<uint8_t> hexStringToBytes(const std::string &hex);
std::string hexStringToUtf8String(const std::string &hexStr);
std::string bytesToHexString(const std::vector<uint8_t> &bytes);
std::string utf8StringToHexString(const std::string &utf8Str);

#endif
