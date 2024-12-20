// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <QuartzCore/QuartzCore.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#include "shell/platform/darwin/ios/ios_context_noop.h"
#include "shell/platform/darwin/ios/rendering_api_selection.h"

FLUTTER_ASSERT_ARC

@interface IOSContextNoopTest : XCTestCase
@end

@implementation IOSContextNoopTest
- (void)testCreateNoop {
  flutter::IOSContextNoop noop;

  XCTAssertTrue(noop.GetBackend() == flutter::IOSRenderingBackend::kImpeller);
}

@end
