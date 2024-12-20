// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_IMAGE_FILTER_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_IMAGE_FILTER_H_

#include "display_list/effects/dl_image_filter.h"
#include "impeller/entity/contents/filters/filter_contents.h"

namespace impeller {

/// @brief  Generate a new FilterContents using this filter's configuration.
///
std::shared_ptr<FilterContents> WrapInput(const flutter::DlImageFilter* filter,
                                          const FilterInput::Ref& input);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_IMAGE_FILTER_H_
