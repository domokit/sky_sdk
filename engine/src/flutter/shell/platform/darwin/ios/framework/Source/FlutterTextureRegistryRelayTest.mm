// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextureRegistryRelay.h"

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterTexture.h"

FLUTTER_ASSERT_ARC

@interface FlutterTextureRegistryRelayTest : XCTestCase
@end

@implementation FlutterTextureRegistryRelayTest

- (void)testCreate {
  id textureRegistry = OCMProtocolMock(@protocol(FlutterTextureRegistry));
  FlutterTextureRegistryRelay* relay =
      [[FlutterTextureRegistryRelay alloc] initWithParent:textureRegistry];
  XCTAssertNotNil(relay);
  XCTAssertEqual(textureRegistry, relay.parent);
}

- (void)testRegisterTexture {
  id textureRegistry = OCMProtocolMock(@protocol(FlutterTextureRegistry));
  FlutterTextureRegistryRelay* relay =
      [[FlutterTextureRegistryRelay alloc] initWithParent:textureRegistry];
  id texture = OCMProtocolMock(@protocol(FlutterTexture));
  [relay registerTexture:texture];
  OCMVerify([textureRegistry registerTexture:texture]);
}

- (void)testTextureFrameAvailable {
  id textureRegistry = OCMProtocolMock(@protocol(FlutterTextureRegistry));
  FlutterTextureRegistryRelay* relay =
      [[FlutterTextureRegistryRelay alloc] initWithParent:textureRegistry];
  [relay textureFrameAvailable:0];
  OCMVerify([textureRegistry textureFrameAvailable:0]);
}

- (void)testUnregisterTexture {
  id textureRegistry = OCMProtocolMock(@protocol(FlutterTextureRegistry));
  FlutterTextureRegistryRelay* relay =
      [[FlutterTextureRegistryRelay alloc] initWithParent:textureRegistry];
  [relay unregisterTexture:0];
  OCMVerify([textureRegistry unregisterTexture:0]);
}

- (void)testRetainCycle {
  __weak FlutterEngine* weakEngine;
  NSObject<FlutterTextureRegistry>* strongRelay;
  @autoreleasepool {
    FlutterDartProject* project = [[FlutterDartProject alloc] init];
    FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"foobar" project:project];
    strongRelay = [engine textureRegistry];
    weakEngine = engine;
  }
  XCTAssertNil(weakEngine);
  XCTAssertNotNil(strongRelay);
}

@end
