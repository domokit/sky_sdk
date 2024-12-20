// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_PLAYGROUND_IMGUI_IMGUI_IMPL_IMPELLER_H_
#define FLUTTER_IMPELLER_PLAYGROUND_IMGUI_IMGUI_IMPL_IMPELLER_H_

#include <memory>

#include "impeller/core/host_buffer.h"
#include "third_party/imgui/imgui.h"

namespace impeller {

class Context;
class RenderPass;

}  // namespace impeller

IMGUI_IMPL_API bool ImGui_ImplImpeller_Init(
    const std::shared_ptr<impeller::Context>& context);

IMGUI_IMPL_API void ImGui_ImplImpeller_Shutdown();

IMGUI_IMPL_API void ImGui_ImplImpeller_RenderDrawData(
    ImDrawData* draw_data,
    impeller::RenderPass& renderpass,
    impeller::HostBuffer& host_buffer);

#endif  // FLUTTER_IMPELLER_PLAYGROUND_IMGUI_IMGUI_IMPL_IMPELLER_H_
