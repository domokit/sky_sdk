// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_TYPOGRAPHY_CONTEXT_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_TYPOGRAPHY_CONTEXT_H_

#include <memory>

#include "flutter/third_party/skia/modules/skparagraph/include/TypefaceFontProvider.h"
#include "flutter/third_party/txt/src/txt/font_collection.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class TypographyContext final
    : public Object<TypographyContext,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerTypographyContext)> {
 public:
  TypographyContext();

  ~TypographyContext() override;

  TypographyContext(const TypographyContext&) = delete;

  TypographyContext& operator=(const TypographyContext&) = delete;

  bool IsValid() const;

  const std::shared_ptr<txt::FontCollection>& GetFontCollection() const;

  //----------------------------------------------------------------------------
  /// @brief      Registers custom font data. If an alias for the family name is
  ///             provided, subsequent lookups will need to use that same alias.
  ///             If not, the family name will be read from the font data.
  ///
  /// @param[in]  font_data          The font data
  /// @param[in]  family_name_alias  The family name alias
  ///
  /// @return     If the font data could be successfully registered.
  ///
  bool RegisterFont(std::unique_ptr<fml::Mapping> font_data,
                    const char* family_name_alias);

 private:
  std::shared_ptr<txt::FontCollection> collection_;
  sk_sp<skia::textlayout::TypefaceFontProvider> asset_font_manager_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_TYPOGRAPHY_CONTEXT_H_
