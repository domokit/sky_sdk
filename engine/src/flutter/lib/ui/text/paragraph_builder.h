// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_TEXT_PARAGRAPH_BUILDER_H_
#define FLUTTER_LIB_UI_TEXT_PARAGRAPH_BUILDER_H_

#include <memory>

#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/paint.h"
#include "flutter/lib/ui/text/paragraph.h"
#include "flutter/third_party/txt/src/txt/paragraph_builder.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

class Paragraph;

class ParagraphBuilder : public RefCountedDartWrappable<ParagraphBuilder> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(ParagraphBuilder);

 public:
  static void Create(Dart_Handle wrapper,
                     Dart_Handle encoded_handle,
                     Dart_Handle strutData,
                     const std::string& fontFamily,
                     const std::vector<std::string>& strutFontFamilies,
                     double fontSize,
                     double height,
                     const std::u16string& ellipsis,
                     const std::string& locale);

  ~ParagraphBuilder() override;

  void pushStyle(const tonic::Int32List& encoded,
                 const std::vector<std::string>& fontFamilies,
                 double fontSize,
                 double letterSpacing,
                 double wordSpacing,
                 double height,
                 double decorationThickness,
                 const std::string& locale,
                 Dart_Handle background_objects,
                 Dart_Handle background_data,
                 Dart_Handle foreground_objects,
                 Dart_Handle foreground_data,
                 Dart_Handle shadows_data,
                 Dart_Handle font_features_data,
                 Dart_Handle font_variations_data);

  void pop();

  Dart_Handle addText(const std::u16string& text);

  // Pushes the information required to leave an open space, where Flutter may
  // draw a custom placeholder into.
  //
  // Internally, this method adds a single object replacement character (0xFFFC)
  // and emplaces a new PlaceholderRun instance to the vector of inline
  // placeholders.
  void addPlaceholder(double width,
                      double height,
                      unsigned alignment,
                      double baseline_offset,
                      unsigned baseline);

  void build(Dart_Handle paragraph_handle);

 private:
  explicit ParagraphBuilder(Dart_Handle encoded,
                            Dart_Handle strutData,
                            const std::string& fontFamily,
                            const std::vector<std::string>& strutFontFamilies,
                            double fontSize,
                            double height,
                            const std::u16string& ellipsis,
                            const std::string& locale);

  std::unique_ptr<txt::ParagraphBuilder> m_paragraph_builder_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_TEXT_PARAGRAPH_BUILDER_H_
