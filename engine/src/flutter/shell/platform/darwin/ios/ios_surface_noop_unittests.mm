// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <QuartzCore/QuartzCore.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/ios/ios_surface_noop.h"

#import "flutter/common/task_runners.h"
#import "flutter/fml/message_loop.h"
#import "flutter/fml/thread.h"
#import "flutter/lib/ui/window/platform_message.h"
#import "flutter/lib/ui/window/platform_message_response.h"
#import "flutter/shell/common/thread_host.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/ios_context_noop.h"

FLUTTER_ASSERT_ARC

@interface IOSSurfaceNoopTest : XCTestCase
@end

@implementation IOSSurfaceNoopTest
- (void)testCreateSurface {
  auto context = std::make_shared<flutter::IOSContextNoop>();
  flutter::IOSSurfaceNoop noop(context);

  XCTAssertTrue(noop.IsValid());
  XCTAssertTrue(!!noop.CreateGPUSurface());
}

@end
