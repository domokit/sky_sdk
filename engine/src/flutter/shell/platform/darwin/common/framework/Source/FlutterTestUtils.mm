// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/framework/Source/FlutterTestUtils.h"

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

FLUTTER_ASSERT_ARC

BOOL FLTThrowsObjcException(dispatch_block_t block) {
  @try {
    block();
  } @catch (...) {
    return YES;
  }
  return NO;
}
