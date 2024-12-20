// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_SOURCE_FLUTTERTESTUTILS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_SOURCE_FLUTTERTESTUTILS_H_

#import <Foundation/Foundation.h>

/// Returns YES if the block throws an exception.
BOOL FLTThrowsObjcException(dispatch_block_t block);

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_SOURCE_FLUTTERTESTUTILS_H_
