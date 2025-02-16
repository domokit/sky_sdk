// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/testing/render_text_in_canvas.h"

#include "flutter/testing/testing.h"
#include "txt/platform.h"

namespace flutter {
namespace testing {

bool RenderTextInCanvasSkia(DlCanvas* canvas,
                            const std::string& text,
                            const std::string_view& font_fixture,
                            DlPoint position,
                            const TextRenderOptions& options) {
  auto c_font_fixture = std::string(font_fixture);
  auto mapping = flutter::testing::OpenFixtureAsSkData(c_font_fixture.c_str());
  if (!mapping) {
    return false;
  }
  sk_sp<SkFontMgr> font_mgr = txt::GetDefaultFontManager();
  SkFont sk_font(font_mgr->makeFromData(mapping), options.font_size);
  if (options.is_subpixel) {
    sk_font.setSubpixel(true);
  }
  auto blob = SkTextBlob::MakeFromString(text.c_str(), sk_font);
  if (!blob) {
    return false;
  }

  auto frame = impeller::MakeTextFrameFromTextBlobSkia(blob);

  DlPaint text_paint;
  text_paint.setColor(options.color);
  text_paint.setMaskFilter(options.mask_filter);
  // text_paint.mask_blur_descriptor = options.mask_blur_descriptor;
  // text_paint.stroke_width = 1;
  // text_paint.style =
  //     options.stroke ? Paint::Style::kStroke : Paint::Style::kFill;
  canvas->DrawTextFrame(frame, position.x, position.y, text_paint);
  return true;
}
}  // namespace testing
}  // namespace flutter
