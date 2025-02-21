// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_H_

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformViews_Internal.h"

#include <memory>

#include "flutter/flow/embedded_views.h"
#include "flutter/flow/surface.h"
#include "flutter/fml/macros.h"

@class CALayer;

namespace flutter {

class IOSSurface {
 public:
  static std::unique_ptr<IOSSurface> Create(std::shared_ptr<IOSContext> context, CALayer* layer);

  std::shared_ptr<IOSContext> GetContext() const;

  virtual ~IOSSurface();

  virtual bool IsValid() const = 0;

  virtual void UpdateStorageSizeIfNecessary() = 0;

  virtual std::unique_ptr<Surface> CreateGPUSurface() = 0;

 protected:
  explicit IOSSurface(std::shared_ptr<IOSContext> ios_context);

 private:
  std::shared_ptr<IOSContext> ios_context_;

  FML_DISALLOW_COPY_AND_ASSIGN(IOSSurface);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_IOS_SURFACE_H_
