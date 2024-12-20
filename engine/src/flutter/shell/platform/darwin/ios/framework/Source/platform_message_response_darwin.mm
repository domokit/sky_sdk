// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/platform_message_response_darwin.h"

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

FLUTTER_ASSERT_ARC

namespace flutter {

PlatformMessageResponseDarwin::PlatformMessageResponseDarwin(
    PlatformMessageResponseCallback callback,
    fml::RefPtr<fml::TaskRunner> platform_task_runner)
    : callback_(callback), platform_task_runner_(std::move(platform_task_runner)) {}

PlatformMessageResponseDarwin::~PlatformMessageResponseDarwin() = default;

void PlatformMessageResponseDarwin::Complete(std::unique_ptr<fml::Mapping> data) {
  fml::RefPtr<PlatformMessageResponseDarwin> self(this);
  platform_task_runner_->PostTask(fml::MakeCopyable([self, data = std::move(data)]() mutable {
    self->callback_(CopyMappingPtrToNSData(std::move(data)));
  }));
}

void PlatformMessageResponseDarwin::CompleteEmpty() {
  fml::RefPtr<PlatformMessageResponseDarwin> self(this);
  platform_task_runner_->PostTask(fml::MakeCopyable([self]() mutable { self->callback_(nil); }));
}

}  // namespace flutter
