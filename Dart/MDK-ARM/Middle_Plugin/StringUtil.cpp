#include "InHpp.hpp"



/* ===============================函数声明=============================== */
// uint8_t hexCharToByte(char c);
// std::vector<uint8_t> hexStringToBytes(const std::string &hex);
// std::string hexStringToUtf8String(const std::string &hexStr);
// std::string bytesToHexString(const std::vector<uint8_t> &bytes);
// std::string utf8StringToHexString(const std::string &utf8Str);
/* ===============================函数声明=============================== */

//////////////////////////////////////////////////////////////////////////////////
// 将单个十六进制字符转换为字节
uint8_t hexCharToByte(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0; // 默认返回 0，表示无效输入
}

// 将十六进制字符串转换为字节数组
std::vector<uint8_t> hexStringToBytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i < hex.length(); i += 2) {
        uint8_t byte = (hexCharToByte(hex[i]) << 4) | hexCharToByte(hex[i + 1]);
        bytes.push_back(byte);
    }
    return bytes;
}

// 十六进制字符串 -> UTF-8 字符串（中文）
std::string hexStringToUtf8String(const std::string& hexStr) {
    std::vector<uint8_t> utf8Bytes = hexStringToBytes(hexStr);
    return std::string(utf8Bytes.begin(), utf8Bytes.end());
}

//////////////////////////////////////////////////////////////////////////////////
// 将单个字节转为两位十六进制字符串
std::string byteToHexString(unsigned char byte) {
    const char* hexDigits = "0123456789abcdef";
    std::string result;
    result += hexDigits[byte >> 4];     // 高四位
    result += hexDigits[byte & 0x0F];   // 低四位
    return result;
}

// UTF-8 字符串 -> 十六进制字符串（不使用 ostringstream）
std::string utf8StringToHexString(const std::string& utf8Str) {
    std::string hexString;
    for (size_t i = 0; i < utf8Str.size(); ++i) {
        unsigned char c = utf8Str[i];
        hexString += byteToHexString(c);
    }
    return hexString;
}
