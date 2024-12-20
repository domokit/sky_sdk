// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewControllerTestUtils.h"

namespace flutter::testing {

id CreateMockViewController() {
  {
    NSString* fixtures = @(testing::GetFixturesPath());
    FlutterDartProject* project = [[FlutterDartProject alloc]
        initWithAssetsPath:fixtures
               ICUDataPath:[fixtures stringByAppendingString:@"/icudtl.dat"]];
    FlutterViewController* viewController = [[FlutterViewController alloc] initWithProject:project];
    id viewControllerMock = OCMPartialMock(viewController);
    return viewControllerMock;
  }
}

}  // namespace flutter::testing
