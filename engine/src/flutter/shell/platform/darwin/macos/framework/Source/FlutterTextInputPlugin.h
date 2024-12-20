// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTEXTINPUTPLUGIN_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTEXTINPUTPLUGIN_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterBinaryMessenger.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"

@class FlutterTextField;

/**
 * A plugin to handle text input.
 *
 * Responsible for bridging the native macOS text input system with the Flutter framework text
 * editing classes, via system channels.
 *
 * This is not an FlutterPlugin since it needs access to FlutterViewController internals, so needs
 * to be managed differently.
 *
 * When accessibility is on, accessibility bridge creates a NSTextField, i.e. FlutterTextField,
 * for every text field in the Flutter. This plugin acts as a field editor for those NSTextField[s].
 */
@interface FlutterTextInputPlugin : NSTextView

/**
 * The NSTextField that currently has this plugin as its field editor.
 *
 * Must be nil if accessibility is off.
 */
@property(nonatomic, weak) FlutterTextField* client;

/**
 * Initializes a text input plugin that coordinates key event handling with |viewController|.
 */
- (instancetype)initWithViewController:(FlutterViewController*)viewController;

/**
 * Whether this plugin is the first responder of this NSWindow.
 *
 * When accessibility is on, this plugin is set as the first responder to act as the field
 * editor for FlutterTextFields.
 *
 * Returns false if accessibility is off.
 */
- (BOOL)isFirstResponder;

/**
 * Handles key down events received from the view controller, responding YES if
 * the event was handled.
 *
 * Note, the Apple docs suggest that clients should override essentially all the
 * mouse and keyboard event-handling methods of NSResponder. However, experimentation
 * indicates that only key events are processed by the native layer; Flutter processes
 * mouse events. Additionally, processing both keyUp and keyDown results in duplicate
 * processing of the same keys.
 */
- (BOOL)handleKeyEvent:(NSEvent*)event;

@end

// Private methods made visible for testing
@interface FlutterTextInputPlugin (TestMethods)
- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;
- (NSRect)firstRectForCharacterRange:(NSRange)range actualRange:(NSRangePointer)actualRange;
- (NSDictionary*)editingState;
@property(nonatomic) NSTextInputContext* textInputContext;
@property(readwrite, nonatomic) NSString* customRunLoopMode;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTEXTINPUTPLUGIN_H_
