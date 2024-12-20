// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_DL_PLAYGROUND_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_DL_PLAYGROUND_H_

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_builder.h"
#include "flutter/impeller/golden_tests/screenshot.h"
#include "impeller/playground/playground_test.h"
#include "third_party/skia/include/core/SkFont.h"

namespace impeller {

class DlPlayground : public PlaygroundTest {
 public:
  using DisplayListPlaygroundCallback =
      std::function<sk_sp<flutter::DisplayList>()>;

  DlPlayground();

  ~DlPlayground();

  bool OpenPlaygroundHere(flutter::DisplayListBuilder& builder);

  bool OpenPlaygroundHere(sk_sp<flutter::DisplayList> list);

  bool OpenPlaygroundHere(DisplayListPlaygroundCallback callback);

  std::unique_ptr<testing::Screenshot> MakeScreenshot(
      const sk_sp<flutter::DisplayList>& list);

  SkFont CreateTestFontOfSize(SkScalar scalar);

  SkFont CreateTestFont();

  sk_sp<flutter::DlImage> CreateDlImageForFixture(
      const char* fixture_name,
      bool enable_mipmapping = false) const;

 private:
  DlPlayground(const DlPlayground&) = delete;

  DlPlayground& operator=(const DlPlayground&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_DL_PLAYGROUND_H_
