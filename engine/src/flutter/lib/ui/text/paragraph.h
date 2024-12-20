// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_TEXT_PARAGRAPH_H_
#define FLUTTER_LIB_UI_TEXT_PARAGRAPH_H_

#include "flutter/fml/message_loop.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/canvas.h"
#include "flutter/third_party/txt/src/txt/paragraph.h"

namespace flutter {

class Paragraph : public RefCountedDartWrappable<Paragraph> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(Paragraph);

 public:
  static void Create(Dart_Handle paragraph_handle,
                     std::unique_ptr<txt::Paragraph> txt_paragraph) {
    auto paragraph = fml::MakeRefCounted<Paragraph>(std::move(txt_paragraph));
    paragraph->AssociateWithDartWrapper(paragraph_handle);
  }

  ~Paragraph() override;

  double width();
  double height();
  double longestLine();
  double minIntrinsicWidth();
  double maxIntrinsicWidth();
  double alphabeticBaseline();
  double ideographicBaseline();
  bool didExceedMaxLines();

  void layout(double width);
  void paint(Canvas* canvas, double x, double y);

  tonic::Float32List getRectsForRange(unsigned start,
                                      unsigned end,
                                      unsigned boxHeightStyle,
                                      unsigned boxWidthStyle);
  tonic::Float32List getRectsForPlaceholders();
  Dart_Handle getPositionForOffset(double dx, double dy);
  Dart_Handle getGlyphInfoAt(unsigned utf16Offset,
                             Dart_Handle constructor) const;
  Dart_Handle getClosestGlyphInfo(double dx,
                                  double dy,
                                  Dart_Handle constructor) const;
  Dart_Handle getWordBoundary(unsigned offset);
  Dart_Handle getLineBoundary(unsigned offset);
  tonic::Float64List computeLineMetrics() const;
  Dart_Handle getLineMetricsAt(int lineNumber, Dart_Handle constructor) const;
  size_t getNumberOfLines() const;
  int getLineNumberAt(size_t utf16Offset) const;

  void dispose();

 private:
  std::unique_ptr<txt::Paragraph> m_paragraph_;

  explicit Paragraph(std::unique_ptr<txt::Paragraph> paragraph);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_TEXT_PARAGRAPH_H_
