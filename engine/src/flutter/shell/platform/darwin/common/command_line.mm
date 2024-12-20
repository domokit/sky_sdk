// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/command_line.h"

#import <Foundation/Foundation.h>

static_assert(__has_feature(objc_arc), "ARC must be enabled.");

namespace flutter {

fml::CommandLine CommandLineFromNSProcessInfo(NSProcessInfo* processInfoOrNil) {
  std::vector<std::string> args_vector;
  auto processInfo = processInfoOrNil ? processInfoOrNil : [NSProcessInfo processInfo];

  for (NSString* arg in processInfo.arguments) {
    args_vector.emplace_back(arg.UTF8String);
  }

  return fml::CommandLineFromIterators(args_vector.begin(), args_vector.end());
}

}  // namespace flutter
