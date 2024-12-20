// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_TEXT_FONT_COLLECTION_H_
#define FLUTTER_LIB_UI_TEXT_FONT_COLLECTION_H_

#include <memory>
#include <vector>

#include "flutter/assets/asset_manager.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_ptr.h"
#include "third_party/tonic/typed_data/typed_list.h"
#include "txt/font_collection.h"

namespace flutter {

class FontCollection {
 public:
  FontCollection();

  virtual ~FontCollection();

  std::shared_ptr<txt::FontCollection> GetFontCollection() const;

  void SetupDefaultFontManager(uint32_t font_initialization_data);

  // Virtual for testing.
  virtual void RegisterFonts(
      const std::shared_ptr<AssetManager>& asset_manager);

  void RegisterTestFonts();

  static void LoadFontFromList(Dart_Handle font_data_handle,
                               Dart_Handle callback,
                               const std::string& family_name);

 private:
  std::shared_ptr<txt::FontCollection> collection_;
  sk_sp<txt::DynamicFontManager> dynamic_font_manager_;

  FML_DISALLOW_COPY_AND_ASSIGN(FontCollection);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_TEXT_FONT_COLLECTION_H_
