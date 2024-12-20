// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder_platform_message_response.h"

#include "flutter/fml/make_copyable.h"

namespace flutter {

EmbedderPlatformMessageResponse::EmbedderPlatformMessageResponse(
    fml::RefPtr<fml::TaskRunner> runner,
    const Callback& callback)
    : runner_(std::move(runner)), callback_(callback) {}

EmbedderPlatformMessageResponse::~EmbedderPlatformMessageResponse() = default;

// |PlatformMessageResponse|
void EmbedderPlatformMessageResponse::Complete(
    std::unique_ptr<fml::Mapping> data) {
  if (!data) {
    CompleteEmpty();
    return;
  }

  runner_->PostTask(
      // The static leak checker gets confused by the use of fml::MakeCopyable.
      // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
      fml::MakeCopyable([data = std::move(data), callback = callback_]() {
        callback(data->GetMapping(), data->GetSize());
      }));
}

// |PlatformMessageResponse|
void EmbedderPlatformMessageResponse::CompleteEmpty() {
  Complete(std::make_unique<fml::NonOwnedMapping>(nullptr, 0u));
}

}  // namespace flutter
