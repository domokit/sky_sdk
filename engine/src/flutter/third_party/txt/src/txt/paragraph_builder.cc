/*
 * Copyright 2017 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "paragraph_builder.h"

#include "flutter/third_party/txt/src/skia/paragraph_builder_skia.h"
#include "paragraph_style.h"
#include "third_party/icu/source/common/unicode/unistr.h"

namespace txt {

//------------------------------------------------------------------------------
/// @brief      Creates a |ParagraphBuilder| based on Skia's text layout module.
///
/// @param[in]  style             The style to use for the paragraph.
/// @param[in]  font_collection   The font collection to use for the paragraph.
/// @param[in]  impeller_enabled  Whether Impeller is enabled in the runtime.
std::unique_ptr<ParagraphBuilder> ParagraphBuilder::CreateSkiaBuilder(
    const ParagraphStyle& style,
    std::shared_ptr<FontCollection> font_collection,
    const bool impeller_enabled) {
  return std::make_unique<ParagraphBuilderSkia>(style, font_collection,
                                                impeller_enabled);
}

}  // namespace txt
