// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputSemanticsObject.h"

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputPlugin.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"

#include "flutter/third_party/accessibility/ax/ax_action_data.h"
#include "flutter/third_party/accessibility/gfx/geometry/rect_conversions.h"
#include "flutter/third_party/accessibility/gfx/mac/coordinate_conversion.h"

#pragma mark - FlutterTextFieldCell
/**
 * A convenient class that can be used to set a custom field editor for an
 * NSTextField.
 *
 * The FlutterTextField uses this class set the FlutterTextInputPlugin as
 * its field editor.
 */
@interface FlutterTextFieldCell : NSTextFieldCell

/**
 * Initializes the NSCell for the input NSTextField.
 */
- (instancetype)initWithTextField:(NSTextField*)textField fieldEditor:(NSTextView*)editor;

@end

@implementation FlutterTextFieldCell {
  NSTextView* _editor;
}

#pragma mark - Private

- (instancetype)initWithTextField:(NSTextField*)textField fieldEditor:(NSTextView*)editor {
  self = [super initTextCell:textField.stringValue];
  if (self) {
    _editor = editor;
    [self setControlView:textField];
    // Read-only text fields are sent to the mac embedding as static
    // text. This text field must be editable and selectable at this
    // point.
    self.editable = YES;
    self.selectable = YES;
  }
  return self;
}

#pragma mark - NSCell

- (NSTextView*)fieldEditorForView:(NSView*)controlView {
  return _editor;
}

@end

#pragma mark - FlutterTextField

@implementation FlutterTextField {
  flutter::FlutterTextPlatformNode* _node;
  FlutterTextInputPlugin* _plugin;
}

#pragma mark - Public

- (instancetype)initWithPlatformNode:(flutter::FlutterTextPlatformNode*)node
                         fieldEditor:(FlutterTextInputPlugin*)plugin {
  self = [super initWithFrame:NSZeroRect];
  if (self) {
    _node = node;
    _plugin = plugin;
    [self setCell:[[FlutterTextFieldCell alloc] initWithTextField:self fieldEditor:plugin]];
  }
  return self;
}

- (void)updateString:(NSString*)string withSelection:(NSRange)selection {
  NSAssert(_plugin.client == self,
           @"Can't update FlutterTextField when it is not the first responder");
  if (![[self stringValue] isEqualToString:string]) {
    [self setStringValue:string];
  }
  if (!NSEqualRanges(_plugin.selectedRange, selection)) {
    [_plugin setSelectedRange:selection];
  }
}

#pragma mark - NSView

- (NSRect)frame {
  if (!_node) {
    return NSZeroRect;
  }
  return _node->GetFrame();
}

#pragma mark - NSAccessibilityProtocol

- (void)setAccessibilityFocused:(BOOL)isFocused {
  if (!_node) {
    return;
  }
  [super setAccessibilityFocused:isFocused];
  ui::AXActionData data;
  data.action = isFocused ? ax::mojom::Action::kFocus : ax::mojom::Action::kBlur;
  _node->GetDelegate()->AccessibilityPerformAction(data);
}

- (void)startEditing {
  if (!_plugin) {
    return;
  }
  if (self.currentEditor == _plugin) {
    return;
  }
  if (!_node) {
    return;
  }
  // Selecting text seems to be the only way to make the field editor
  // current editor.
  [self selectText:self];
  NSAssert(self.currentEditor == _plugin, @"Failed to set current editor");

  _plugin.client = self;

  // Restore previous selection.
  NSString* textValue = @(_node->GetStringAttribute(ax::mojom::StringAttribute::kValue).data());
  int start = _node->GetIntAttribute(ax::mojom::IntAttribute::kTextSelStart);
  int end = _node->GetIntAttribute(ax::mojom::IntAttribute::kTextSelEnd);
  NSAssert((start >= 0 && end >= 0) || (start == -1 && end == -1), @"selection is invalid");
  NSRange selection;
  if (start >= 0 && end >= 0) {
    selection = NSMakeRange(MIN(start, end), ABS(end - start));
  } else {
    // The native behavior is to place the cursor at the end of the string if
    // there is no selection.
    selection = NSMakeRange([self stringValue].length, 0);
  }
  [self updateString:textValue withSelection:selection];
}

- (void)setPlatformNode:(flutter::FlutterTextPlatformNode*)node {
  _node = node;
}

#pragma mark - NSObject

- (void)dealloc {
  if (_plugin.client == self) {
    _plugin.client = nil;
  }
}

@end

namespace flutter {

FlutterTextPlatformNode::FlutterTextPlatformNode(FlutterPlatformNodeDelegate* delegate,
                                                 __weak FlutterViewController* view_controller) {
  Init(delegate);
  view_controller_ = view_controller;
  appkit_text_field_ =
      [[FlutterTextField alloc] initWithPlatformNode:this
                                         fieldEditor:view_controller.textInputPlugin];
  appkit_text_field_.bezeled = NO;
  appkit_text_field_.drawsBackground = NO;
  appkit_text_field_.bordered = NO;
  appkit_text_field_.focusRingType = NSFocusRingTypeNone;
}

FlutterTextPlatformNode::~FlutterTextPlatformNode() {
  [appkit_text_field_ setPlatformNode:nil];
  EnsureDetachedFromView();
}

gfx::NativeViewAccessible FlutterTextPlatformNode::GetNativeViewAccessible() {
  if (EnsureAttachedToView()) {
    return appkit_text_field_;
  }
  return nil;
}

NSRect FlutterTextPlatformNode::GetFrame() {
  if (!view_controller_.viewLoaded) {
    return NSZeroRect;
  }
  FlutterPlatformNodeDelegate* delegate = static_cast<FlutterPlatformNodeDelegate*>(GetDelegate());
  bool offscreen;
  auto bridge_ptr = delegate->GetOwnerBridge().lock();
  if (!bridge_ptr) {
    return NSZeroRect;
  }
  gfx::RectF bounds = bridge_ptr->RelativeToGlobalBounds(delegate->GetAXNode(), offscreen, true);

  // Converts to NSRect to use NSView rect conversion.
  NSRect ns_local_bounds = NSMakeRect(bounds.x(), bounds.y(), bounds.width(), bounds.height());
  // The macOS XY coordinates start at bottom-left and increase toward top-right,
  // which is different from the Flutter's XY coordinates that start at top-left
  // increasing to bottom-right. Flip the y coordinate to convert from Flutter
  // coordinates to macOS coordinates.
  ns_local_bounds.origin.y = -ns_local_bounds.origin.y - ns_local_bounds.size.height;
  NSRect ns_view_bounds = [view_controller_.flutterView convertRectFromBacking:ns_local_bounds];
  return [view_controller_.flutterView convertRect:ns_view_bounds toView:nil];
}

bool FlutterTextPlatformNode::EnsureAttachedToView() {
  if (!view_controller_.viewLoaded) {
    return false;
  }
  if ([appkit_text_field_ isDescendantOf:view_controller_.view]) {
    return true;
  }
  [view_controller_.view addSubview:appkit_text_field_
                         positioned:NSWindowBelow
                         relativeTo:view_controller_.flutterView];
  return true;
}

void FlutterTextPlatformNode::EnsureDetachedFromView() {
  [appkit_text_field_ removeFromSuperview];
}

}  // namespace flutter
