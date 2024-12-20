// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_AIKS_PLAYGROUND_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_AIKS_PLAYGROUND_H_

#include "flutter/display_list/display_list.h"
#include "impeller/display_list/aiks_context.h"
#include "impeller/playground/playground_test.h"
#include "impeller/typographer/typographer_context.h"
#include "third_party/imgui/imgui.h"

namespace impeller {

class AiksPlayground : public PlaygroundTest {
 public:
  using AiksDlPlaygroundCallback = std::function<sk_sp<flutter::DisplayList>()>;

  AiksPlayground();

  ~AiksPlayground();

  void TearDown() override;

  void SetTypographerContext(
      std::shared_ptr<TypographerContext> typographer_context);

  bool OpenPlaygroundHere(const AiksDlPlaygroundCallback& callback);

  bool OpenPlaygroundHere(const sk_sp<flutter::DisplayList>& list);

  static bool ImGuiBegin(const char* name,
                         bool* p_open,
                         ImGuiWindowFlags flags);

 private:
  std::shared_ptr<TypographerContext> typographer_context_;

  AiksPlayground(const AiksPlayground&) = delete;

  AiksPlayground& operator=(const AiksPlayground&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_AIKS_PLAYGROUND_H_
