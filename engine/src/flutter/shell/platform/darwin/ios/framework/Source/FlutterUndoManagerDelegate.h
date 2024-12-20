// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERUNDOMANAGERDELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERUNDOMANAGERDELEGATE_H_

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

typedef NS_ENUM(NSInteger, FlutterUndoRedoDirection) {
  // NOLINTBEGIN(readability-identifier-naming)
  FlutterUndoRedoDirectionUndo,
  FlutterUndoRedoDirectionRedo,
  // NOLINTEND(readability-identifier-naming)
};

/**
 * Protocol for undo manager changes from the `FlutterUndoManagerPlugin`, typically a
 * `FlutterEngine`.
 */
@protocol FlutterUndoManagerDelegate <NSObject>

/**
 * The `NSUndoManager` that should be managed by the `FlutterUndoManagerPlugin`.
 * When the delegate is `FlutterEngine` this will be the `FlutterViewController`'s undo manager.
 */
@property(nonatomic, readonly, nullable) NSUndoManager* undoManager;

/**
 * Used to notify the active view when undo manager state (can redo/can undo)
 * changes, in order to force keyboards to update undo/redo buttons.
 */
@property(nonatomic, readonly, nullable) UIView<UITextInput>* activeTextInputView;

/**
 * Pass changes to the framework through the undo manager channel.
 */
- (void)handleUndoWithDirection:(FlutterUndoRedoDirection)direction;

@end
NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERUNDOMANAGERDELEGATE_H_
