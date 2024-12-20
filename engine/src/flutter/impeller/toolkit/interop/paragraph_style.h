// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_PARAGRAPH_STYLE_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_PARAGRAPH_STYLE_H_

#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"
#include "impeller/toolkit/interop/paint.h"
#include "third_party/txt/src/txt/paragraph_style.h"

namespace impeller::interop {

class ParagraphStyle final
    : public Object<ParagraphStyle,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerParagraphStyle)> {
 public:
  explicit ParagraphStyle();

  ~ParagraphStyle() override;

  ParagraphStyle(const ParagraphStyle&) = delete;

  ParagraphStyle& operator=(const ParagraphStyle&) = delete;

  void SetForeground(ScopedObject<Paint> paint);

  void SetBackground(ScopedObject<Paint> paint);

  void SetFontWeight(txt::FontWeight weight);

  void SetFontStyle(txt::FontStyle style);

  void SetFontFamily(std::string family);

  void SetFontSize(double size);

  void SetHeight(double height);

  void SetTextAlignment(txt::TextAlign alignment);

  void SetTextDirection(txt::TextDirection direction);

  void SetMaxLines(size_t max_lines);

  void SetLocale(std::string locale);

  txt::TextStyle CreateTextStyle() const;

  const txt::ParagraphStyle& GetParagraphStyle() const;

 private:
  txt::ParagraphStyle style_;
  ScopedObject<Paint> foreground_;
  ScopedObject<Paint> background_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_PARAGRAPH_STYLE_H_
