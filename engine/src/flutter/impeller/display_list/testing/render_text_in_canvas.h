// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_TESTING_RENDER_TEXT_IN_CANVAS_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_TESTING_RENDER_TEXT_IN_CANVAS_H_

#include <memory>

#include "flutter/display_list/dl_canvas.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/effects/dl_mask_filter.h"
#include "impeller/typographer/backends/skia/text_frame_skia.h"

namespace flutter {
namespace testing {

struct TextRenderOptions {
  bool stroke = false;
  SkScalar font_size = 50;
  DlColor color = DlColor::kYellow();
  std::shared_ptr<DlMaskFilter> mask_filter;
  bool is_subpixel = false;
};

bool RenderTextInCanvasSkia(DlCanvas* canvas,
                            const std::string& text,
                            const std::string_view& font_fixture,
                            DlPoint position,
                            const TextRenderOptions& options = {});

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_TESTING_RENDER_TEXT_IN_CANVAS_H_
