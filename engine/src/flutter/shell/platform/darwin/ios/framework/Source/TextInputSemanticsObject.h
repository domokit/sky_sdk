// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_TEXTINPUTSEMANTICSOBJECT_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_TEXTINPUTSEMANTICSOBJECT_H_

#import <UIKit/UIKit.h>

#import "flutter/shell/platform/darwin/ios/framework/Source/SemanticsObject.h"

/**
 * An implementation of `SemanticsObject` specialized for expressing text
 * fields.
 *
 * Delegates to `FlutterTextInputView` when the object corresponds to a text
 * field that currently owns input focus. Delegates to
 * `FlutterInactiveTextInput` otherwise.
 */
@interface TextInputSemanticsObject : SemanticsObject <UITextInput>
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_TEXTINPUTSEMANTICSOBJECT_H_
