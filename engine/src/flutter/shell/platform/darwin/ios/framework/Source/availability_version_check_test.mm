// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <tuple>

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/common/availability_version_check.h"

@interface AvailabilityVersionCheckTest : XCTestCase
@end

@implementation AvailabilityVersionCheckTest

- (void)testSimple {
  auto maybe_product_version = flutter::ProductVersionFromSystemVersionPList();
  XCTAssertTrue(maybe_product_version.has_value());
  if (maybe_product_version.has_value()) {
    auto product_version = maybe_product_version.value();
    XCTAssertTrue(product_version > std::make_tuple(0, 0, 0));
  }
}

@end
