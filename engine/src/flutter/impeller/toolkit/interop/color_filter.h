// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_INTEROP_COLOR_FILTER_H_
#define FLUTTER_IMPELLER_TOOLKIT_INTEROP_COLOR_FILTER_H_

#include "flutter/display_list/effects/dl_color_filter.h"
#include "impeller/toolkit/interop/formats.h"
#include "impeller/toolkit/interop/impeller.h"
#include "impeller/toolkit/interop/object.h"

namespace impeller::interop {

class ColorFilter final
    : public Object<ColorFilter,
                    IMPELLER_INTERNAL_HANDLE_NAME(ImpellerColorFilter)> {
 public:
  static ScopedObject<ColorFilter> MakeBlend(Color color, BlendMode mode);

  static ScopedObject<ColorFilter> MakeMatrix(const float matrix[20]);

  explicit ColorFilter(std::shared_ptr<const flutter::DlColorFilter> filter);

  ~ColorFilter() override;

  ColorFilter(const ColorFilter&) = delete;

  ColorFilter& operator=(const ColorFilter&) = delete;

  const std::shared_ptr<const flutter::DlColorFilter>& GetColorFilter() const;

 private:
  std::shared_ptr<const flutter::DlColorFilter> filter_;
};

}  // namespace impeller::interop

#endif  // FLUTTER_IMPELLER_TOOLKIT_INTEROP_COLOR_FILTER_H_
