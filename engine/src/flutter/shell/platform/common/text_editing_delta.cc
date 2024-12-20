// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/text_editing_delta.h"

#include "flutter/fml/string_conversion.h"

namespace flutter {

TextEditingDelta::TextEditingDelta(const std::u16string& text_before_change,
                                   const TextRange& range,
                                   const std::u16string& text)
    : old_text_(text_before_change),
      delta_text_(text),
      delta_start_(range.start()),
      delta_end_(range.start() + range.length()) {}

TextEditingDelta::TextEditingDelta(const std::string& text_before_change,
                                   const TextRange& range,
                                   const std::string& text)
    : old_text_(fml::Utf8ToUtf16(text_before_change)),
      delta_text_(fml::Utf8ToUtf16(text)),
      delta_start_(range.start()),
      delta_end_(range.start() + range.length()) {}

TextEditingDelta::TextEditingDelta(const std::u16string& text)
    : old_text_(text), delta_text_(u""), delta_start_(-1), delta_end_(-1) {}

TextEditingDelta::TextEditingDelta(const std::string& text)
    : old_text_(fml::Utf8ToUtf16(text)),
      delta_text_(u""),
      delta_start_(-1),
      delta_end_(-1) {}

}  // namespace flutter
