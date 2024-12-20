// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/ios/framework/Source/connection_collection.h"

@interface ConnectionCollectionTest : XCTestCase
@end

@implementation ConnectionCollectionTest

- (void)testSimple {
  auto connections = std::make_unique<flutter::ConnectionCollection>();
  flutter::ConnectionCollection::Connection connection = connections->AquireConnection("foo");
  XCTAssertTrue(connections->CleanupConnection(connection) == "foo");
  XCTAssertTrue(connections->CleanupConnection(connection).empty());
}

@end
