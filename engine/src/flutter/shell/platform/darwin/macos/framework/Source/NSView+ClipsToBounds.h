// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_NSVIEW_CLIPSTOBOUNDS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_NSVIEW_CLIPSTOBOUNDS_H_

#import <Cocoa/Cocoa.h>

@interface NSView (ClipsToBounds)
// This property is available since macOS 10.9 but only declared in macOS 14 SDK.
@property BOOL clipsToBounds API_AVAILABLE(macos(10.9));
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_NSVIEW_CLIPSTOBOUNDS_H_
