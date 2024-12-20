// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_PARAGRAPH_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_PARAGRAPH_H_

#include "flutter/third_party/txt/src/txt/paragraph.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class Paragraph final
    : public Object<Paragraph,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerParagraph)> {
 public:
  explicit Paragraph(std::unique_ptr<txt::Paragraph> paragraph);

  ~Paragraph() override;

  Paragraph(const Paragraph&) = delete;

  Paragraph& operator=(const Paragraph&) = delete;

  Scalar GetMaxWidth() const;

  Scalar GetHeight() const;

  Scalar GetLongestLineWidth() const;

  Scalar GetMinIntrinsicWidth() const;

  Scalar GetMaxIntrinsicWidth() const;

  Scalar GetIdeographicBaseline() const;

  Scalar GetAlphabeticBaseline() const;

  uint32_t GetLineCount() const;

  const std::unique_ptr<txt::Paragraph>& GetHandle() const;

 private:
  std::unique_ptr<txt::Paragraph> paragraph_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_PARAGRAPH_H_
