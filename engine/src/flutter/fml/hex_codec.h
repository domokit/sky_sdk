// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_HEX_CODEC_H_
#define FLUTTER_FML_HEX_CODEC_H_

#include <string>
#include <string_view>

namespace fml {

std::string HexEncode(std::string_view input);

}  // namespace fml

#endif  // FLUTTER_FML_HEX_CODEC_H_
