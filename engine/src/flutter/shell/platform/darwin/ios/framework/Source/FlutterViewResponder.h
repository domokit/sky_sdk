// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERVIEWRESPONDER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERVIEWRESPONDER_H_

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

/**
 * Protocol to send touch events, typically a `FlutterViewController`.
 */
@protocol FlutterViewResponder <NSObject>

@property(nonatomic, strong) UIView* view;

/**
 * See `-[UIResponder touchesBegan:withEvent:]`
 */
- (void)touchesBegan:(NSSet*)touches withEvent:(UIEvent*)event;

/**
 * See `-[UIResponder touchesMoved:withEvent:]`
 */
- (void)touchesMoved:(NSSet*)touches withEvent:(UIEvent*)event;

/**
 * See `-[UIResponder touchesEnded:withEvent:]`
 */
- (void)touchesEnded:(NSSet*)touches withEvent:(UIEvent*)event;

/**
 * See `-[UIResponder touchesCancelled:withEvent:]`
 */
- (void)touchesCancelled:(NSSet*)touches withEvent:(UIEvent*)event;

/**
 * See `-[UIResponder touchesEstimatedPropertiesUpdated:]`
 */
- (void)touchesEstimatedPropertiesUpdated:(NSSet*)touches;

/**
 * Send touches to the Flutter Engine while forcing the change type to be cancelled.
 * The `phase`s in `touches` are ignored.
 */
- (void)forceTouchesCancelled:(NSSet*)touches;

@end
NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERVIEWRESPONDER_H_
