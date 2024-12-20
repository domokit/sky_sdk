// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/runtime_stage/runtime_stage_playground.h"

#include <future>

#include "flutter/fml/make_copyable.h"
#include "flutter/testing/testing.h"
#include "impeller/core/shader_types.h"
#include "impeller/renderer/shader_library.h"

namespace impeller {

RuntimeStagePlayground::RuntimeStagePlayground() = default;

RuntimeStagePlayground::~RuntimeStagePlayground() = default;

bool RuntimeStagePlayground::RegisterStage(const RuntimeStage& stage) {
  std::promise<bool> registration;
  auto future = registration.get_future();
  const std::shared_ptr<ShaderLibrary>& library =
      GetContext()->GetShaderLibrary();
  GetContext()->GetShaderLibrary()->RegisterFunction(
      stage.GetEntrypoint(), ToShaderStage(stage.GetShaderStage()),
      stage.GetCodeMapping(),
      fml::MakeCopyable([reg = std::move(registration)](bool result) mutable {
        reg.set_value(result);
      }));
  return future.get();
}

}  // namespace impeller
