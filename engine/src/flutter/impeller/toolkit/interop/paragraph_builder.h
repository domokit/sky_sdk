// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_PARAGRAPH_BUILDER_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_PARAGRAPH_BUILDER_H_

#include <memory>

#include "flutter/third_party/txt/src/txt/paragraph_builder.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"
#include "impeller/toolkit/interop/paragraph.h"
#include "impeller/toolkit/interop/paragraph_style.h"
#include "impeller/toolkit/interop/typography_context.h"

namespace impeller::interop {

class ParagraphBuilder final
    : public Object<ParagraphBuilder,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerParagraphBuilder)> {
 public:
  explicit ParagraphBuilder(ScopedObject<TypographyContext> context);

  ~ParagraphBuilder() override;

  ParagraphBuilder(const ParagraphBuilder&) = delete;

  ParagraphBuilder& operator=(const ParagraphBuilder&) = delete;

  bool IsValid() const;

  void PushStyle(const ParagraphStyle& style);

  void PopStyle();

  void AddText(const uint8_t* data, size_t byte_length);

  ScopedObject<Paragraph> Build(Scalar width) const;

 private:
  ScopedObject<TypographyContext> context_;
  mutable std::unique_ptr<txt::ParagraphBuilder> lazy_builder_;

  const std::unique_ptr<txt::ParagraphBuilder>& GetBuilder(
      const txt::ParagraphStyle& style) const;

  const std::unique_ptr<txt::ParagraphBuilder>& GetBuilder() const;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_PARAGRAPH_BUILDER_H_
