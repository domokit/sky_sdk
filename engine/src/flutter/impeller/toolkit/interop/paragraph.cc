// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/interop/paragraph.h"

namespace impeller::interop {

Paragraph::Paragraph(std::unique_ptr<txt::Paragraph> paragraph)
    : paragraph_(std::move(paragraph)) {}

Paragraph::~Paragraph() = default;

Scalar Paragraph::GetMaxWidth() const {
  return paragraph_->GetMaxWidth();
}

Scalar Paragraph::GetHeight() const {
  return paragraph_->GetHeight();
}

Scalar Paragraph::GetLongestLineWidth() const {
  return paragraph_->GetLongestLine();
}

Scalar Paragraph::GetMinIntrinsicWidth() const {
  return paragraph_->GetMinIntrinsicWidth();
}

Scalar Paragraph::GetMaxIntrinsicWidth() const {
  return paragraph_->GetMaxIntrinsicWidth();
}

Scalar Paragraph::GetIdeographicBaseline() const {
  return paragraph_->GetIdeographicBaseline();
}

Scalar Paragraph::GetAlphabeticBaseline() const {
  return paragraph_->GetAlphabeticBaseline();
}

uint32_t Paragraph::GetLineCount() const {
  return paragraph_->GetNumberOfLines();
}

const std::unique_ptr<txt::Paragraph>& Paragraph::GetHandle() const {
  return paragraph_;
}

}  // namespace impeller::interop
