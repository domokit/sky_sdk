// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_STRING_CONVERSION_H_
#define FLUTTER_FML_STRING_CONVERSION_H_

#include <string>
#include <vector>

namespace fml {

// Returns a string joined by the given delimiter.
std::string Join(const std::vector<std::string>& vec, const char* delimiter);

// Returns a UTF-8 encoded equivalent of a UTF-16 encoded input string.
std::string Utf16ToUtf8(const std::u16string_view string);

// Returns a UTF-16 encoded equivalent of a UTF-8 encoded input string.
std::u16string Utf8ToUtf16(const std::string_view string);

}  // namespace fml

#endif  // FLUTTER_FML_STRING_CONVERSION_H_
