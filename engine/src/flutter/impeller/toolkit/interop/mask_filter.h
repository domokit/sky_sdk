// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_MASK_FILTER_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_MASK_FILTER_H_

#include "flutter/display_list/effects/dl_mask_filter.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class MaskFilter final
    : public Object<MaskFilter,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerMaskFilter)> {
 public:
  static ScopedObject<MaskFilter> MakeBlur(flutter::DlBlurStyle style,
                                           float sigma);

  explicit MaskFilter(std::shared_ptr<flutter::DlMaskFilter> mask_filter);

  ~MaskFilter() override;

  MaskFilter(const MaskFilter&) = delete;

  MaskFilter& operator=(const MaskFilter&) = delete;

  const std::shared_ptr<flutter::DlMaskFilter>& GetMaskFilter() const;

 public:
  std::shared_ptr<flutter::DlMaskFilter> mask_filter_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_MASK_FILTER_H_
