// Copyright (c) 2011 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "base/win/enum_variant.h"

#include <wrl/client.h>

#include <algorithm>

#include "base/logging.h"

namespace base {
namespace win {

EnumVariant::EnumVariant(ULONG count) : current_index_(0) {
  for (ULONG i = 0; i < count; ++i)
    items_.emplace_back(ScopedVariant::kEmptyVariant);
}

EnumVariant::~EnumVariant() = default;

VARIANT* EnumVariant::ItemAt(ULONG index) {
  BASE_DCHECK(index < items_.size());
  // This is a hack to return a mutable pointer to the ScopedVariant, even
  // though the original intent of the AsInput method was to allow only readonly
  // access to the wrapped variant.
  return items_[index].AsInput();
}

HRESULT EnumVariant::Next(ULONG requested_count,
                          VARIANT* out_elements,
                          ULONG* out_elements_received) {
  if (!out_elements)
    return E_INVALIDARG;

  BASE_DCHECK(current_index_ <= items_.size());
  ULONG available_count = static_cast<ULONG>(items_.size()) - current_index_;
  ULONG count = std::min(requested_count, available_count);
  for (ULONG i = 0; i < count; ++i)
    out_elements[i] = items_[current_index_ + i].Copy();
  current_index_ += count;

  // The caller can choose not to get the number of received elements by setting
  // |out_elements_received| to nullptr.
  if (out_elements_received)
    *out_elements_received = count;

  return (count == requested_count ? S_OK : S_FALSE);
}

HRESULT EnumVariant::Skip(ULONG skip_count) {
  ULONG count = skip_count;
  if (current_index_ + count > static_cast<ULONG>(items_.size()))
    count = static_cast<ULONG>(items_.size()) - current_index_;

  current_index_ += count;
  return (count == skip_count ? S_OK : S_FALSE);
}

HRESULT EnumVariant::Reset() {
  current_index_ = 0;
  return S_OK;
}

HRESULT EnumVariant::Clone(IEnumVARIANT** out_cloned_object) {
  if (!out_cloned_object)
    return E_INVALIDARG;

  size_t count = items_.size();
  Microsoft::WRL::ComPtr<EnumVariant> other =
      Microsoft::WRL::Make<EnumVariant>(static_cast<ULONG>(count));
  for (size_t i = 0; i < count; ++i)
    other->items_[i] = static_cast<const VARIANT&>(items_[i]);

  other->Skip(current_index_);
  return other.CopyTo(IID_PPV_ARGS(out_cloned_object));
}

}  // namespace win
}  // namespace base
