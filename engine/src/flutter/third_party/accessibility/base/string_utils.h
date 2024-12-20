// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_STRING_UTILS_H_
#define BASE_STRING_UTILS_H_

#include <memory>
#include <string>
#include <vector>

namespace base {

constexpr char16_t kWhitespaceUTF16 = u' ';

// Return a C++ string given printf-like input.
template <typename... Args>
std::string StringPrintf(const std::string& format, Args... args) {
  // Calculate the buffer size.
  int size = snprintf(nullptr, 0, format.c_str(), args...) + 1;
  std::unique_ptr<char[]> buf = std::make_unique<char[]>(size);
  snprintf(buf.get(), size, format.c_str(), args...);
  return std::string(buf.get(), buf.get() + size - 1);
}

std::u16string ASCIIToUTF16(std::string src);
std::u16string UTF8ToUTF16(std::string src);
std::string UTF16ToUTF8(std::u16string src);

std::u16string NumberToString16(unsigned int number);
std::u16string NumberToString16(int32_t number);
std::u16string NumberToString16(float number);
std::u16string NumberToString16(double number);

std::string NumberToString(unsigned int number);
std::string NumberToString(int32_t number);
std::string NumberToString(float number);
std::string NumberToString(double number);

std::string ToUpperASCII(std::string str);
std::string ToLowerASCII(std::string str);

std::string JoinString(std::vector<std::string> tokens, std::string delimiter);
std::u16string JoinString(std::vector<std::u16string> tokens,
                          std::u16string delimiter);

void ReplaceChars(std::string in,
                  std::string from,
                  std::string to,
                  std::string* out);
void ReplaceChars(std::u16string in,
                  std::u16string from,
                  std::u16string to,
                  std::u16string* out);
bool LowerCaseEqualsASCII(std::string a, std::string b);
bool ContainsOnlyChars(std::u16string str, char16_t ch);

const std::string& EmptyString();

}  // namespace base

#endif  // BASE_STRING_UTILS_H_
