// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_CONTEXT_SOFTWARE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_CONTEXT_SOFTWARE_H_

#include "flutter/fml/macros.h"
#import "flutter/shell/platform/darwin/ios/ios_context.h"

namespace flutter {

class IOSContextSoftware final : public IOSContext {
 public:
  IOSContextSoftware();

  // |IOSContext|
  ~IOSContextSoftware();

  // |IOSContext|
  sk_sp<GrDirectContext> CreateResourceContext() override;

  // |IOSContext|
  sk_sp<GrDirectContext> GetMainContext() const override;

  // |IOSContext|
  std::unique_ptr<GLContextResult> MakeCurrent() override;

  // |IOSContext|
  std::unique_ptr<Texture> CreateExternalTexture(int64_t texture_id,
                                                 NSObject<FlutterTexture>* texture) override;

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(IOSContextSoftware);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_CONTEXT_SOFTWARE_H_
