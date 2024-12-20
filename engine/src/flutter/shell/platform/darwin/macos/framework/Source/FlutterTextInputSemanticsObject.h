// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTEXTINPUTSEMANTICSOBJECT_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTEXTINPUTSEMANTICSOBJECT_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterPlatformNodeDelegateMac.h"

#include "flutter/fml/macros.h"
#include "flutter/third_party/accessibility/ax/platform/ax_platform_node_base.h"

@class FlutterTextField;
@class FlutterTextInputPlugin;

namespace flutter {

//------------------------------------------------------------------------------
/// The ax platform node for a text field.
class FlutterTextPlatformNode : public ui::AXPlatformNodeBase {
 public:
  //---------------------------------------------------------------------------
  /// @brief      Creates a FlutterTextPlatformNode that uses a
  ///             FlutterTextField as its NativeViewAccessible.
  /// @param[in]  delegate          The delegate that provides accessibility
  ///                               data.
  /// @param[in]  view_controller   The view_controller that is used for querying
  ///                               the information about FlutterView and
  ///                               FlutterTextInputPlugin.
  explicit FlutterTextPlatformNode(FlutterPlatformNodeDelegate* delegate,
                                   __weak FlutterViewController* view_controller);
  ~FlutterTextPlatformNode() override;

  //------------------------------------------------------------------------------
  /// @brief Gets the frame of this platform node relative to the view of
  ///        FlutterViewController. This is used by the FlutterTextField to get its
  ///        frame rect because the FlutterTextField is a subview of the
  ///        FlutterViewController.view.
  NSRect GetFrame();

  // |ui::AXPlatformNodeBase|
  gfx::NativeViewAccessible GetNativeViewAccessible() override;

 private:
  FlutterTextField* appkit_text_field_;
  __weak FlutterViewController* view_controller_;

  //------------------------------------------------------------------------------
  /// @brief Ensures the FlutterTextField is attached to the FlutterView. This
  ///        method returns true if the text field is succesfully attached. If
  ///        this method returns false, that means the FlutterTextField could not
  ///        be attached to the FlutterView. This can happen when the FlutterEngine
  ///        does not have a FlutterViewController or the FlutterView is not loaded
  ///        yet.
  bool EnsureAttachedToView();

  //------------------------------------------------------------------------------
  /// @brief Detaches the FlutterTextField from the FlutterView if it is not
  ///        already detached.
  void EnsureDetachedFromView();

  FML_DISALLOW_COPY_AND_ASSIGN(FlutterTextPlatformNode);
};

}  // namespace flutter

/**
 * An NSTextField implementation that represents the NativeViewAccessible for the
 * FlutterTextPlatformNode
 *
 * The NSAccessibility protocol does not provide full support for text editing. This
 * appkit text field is used to get around this problem. The FlutterTextPlatformNode
 * creates a hidden FlutterTextField, since VoiceOver only provides text editing
 * announcements for NSTextField subclasses.
 *
 * All of the text editing events in this native text field are redirected to the
 * FlutterTextInputPlugin.
 */
@interface FlutterTextField : NSTextField

/**
 * Initializes a FlutterTextField that uses the FlutterTextInputPlugin as its field editor.
 * The text field redirects all of the text editing events to the FlutterTextInputPlugin.
 */
- (instancetype)initWithPlatformNode:(flutter::FlutterTextPlatformNode*)node
                         fieldEditor:(FlutterTextInputPlugin*)plugin;

/**
 * Updates the string value and the selection of this text field.
 *
 * Calling this method is necessary for macOS to get notified about string and selection
 * changes.
 */
- (void)updateString:(NSString*)string withSelection:(NSRange)selection;

/**
 * Makes the field editor (plugin) current editor for this TextField, meaning
 * that the text field will start getting editing events.
 */
- (void)startEditing;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTEXTINPUTSEMANTICSOBJECT_H_
