// Copyright (c) 2011 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_WIN_ENUM_VARIANT_H_
#define BASE_WIN_ENUM_VARIANT_H_

#include <wrl/implements.h>

#include <memory>
#include <vector>

#include "base/win/scoped_variant.h"

namespace base {
namespace win {

// A simple implementation of IEnumVARIANT.
class BASE_EXPORT EnumVariant
    : public Microsoft::WRL::RuntimeClass<
          Microsoft::WRL::RuntimeClassFlags<Microsoft::WRL::ClassicCom>,
          IEnumVARIANT> {
 public:
  // The constructor allocates a vector of empty ScopedVariants of size |count|.
  // Use ItemAt to set the value of each item in the array.
  explicit EnumVariant(ULONG count);

  // IEnumVARIANT:
  IFACEMETHODIMP Next(ULONG requested_count,
                      VARIANT* out_elements,
                      ULONG* out_elements_received) override;
  IFACEMETHODIMP Skip(ULONG skip_count) override;
  IFACEMETHODIMP Reset() override;
  IFACEMETHODIMP Clone(IEnumVARIANT** out_cloned_object) override;

  // Returns a mutable pointer to the item at position |index|.
  VARIANT* ItemAt(ULONG index);

 private:
  ~EnumVariant() override;

  std::vector<ScopedVariant> items_;
  ULONG current_index_;
};

}  // namespace win
}  // namespace base

#endif  // BASE_WIN_ENUM_VARIANT_H_
