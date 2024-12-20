// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLUGINAPPLIFECYCLEDELEGATE_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLUGINAPPLIFECYCLEDELEGATE_INTERNAL_H_

@interface FlutterPluginAppLifeCycleDelegate ()

/**
 * Check whether the selector should be handled dynamically.
 */
- (BOOL)isSelectorAddedDynamically:(SEL)selector;

/**
 * Check whether there is at least one plugin responds to the selector.
 */
- (BOOL)hasPluginThatRespondsToSelector:(SEL)selector;

@end
;

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLUGINAPPLIFECYCLEDELEGATE_INTERNAL_H_
