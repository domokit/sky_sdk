// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_CONTEXT_NOOP_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_CONTEXT_NOOP_H_

#import "flutter/shell/platform/darwin/ios/ios_context.h"

namespace flutter {

/// @brief A noop rendering context for usage on simulators without metal support.
class IOSContextNoop final : public IOSContext {
 public:
  IOSContextNoop();

  // |IOSContext|
  ~IOSContextNoop();

  // |IOSContext|
  sk_sp<GrDirectContext> CreateResourceContext() override;

  // |IOSContext|
  sk_sp<GrDirectContext> GetMainContext() const override;

  // |IOSContext|
  std::unique_ptr<GLContextResult> MakeCurrent() override;

  // |IOSContext|
  std::unique_ptr<Texture> CreateExternalTexture(int64_t texture_id,
                                                 NSObject<FlutterTexture>* texture) override;

  IOSRenderingBackend GetBackend() const override;

 private:
  IOSContextNoop(const IOSContextNoop&) = delete;

  IOSContextNoop& operator=(const IOSContextNoop&) = delete;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_CONTEXT_NOOP_H_
