// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputPlugin.h"

#import <Foundation/Foundation.h>
#import <objc/message.h>

#include <algorithm>
#include <memory>

#include "flutter/fml/platform/darwin/string_range_sanitization.h"
#include "flutter/shell/platform/common/text_editing_delta.h"
#include "flutter/shell/platform/common/text_input_model.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterCodecs.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputSemanticsObject.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/NSView+ClipsToBounds.h"

static NSString* const kTextInputChannel = @"flutter/textinput";

#pragma mark - TextInput channel method names
// See https://api.flutter.dev/flutter/services/SystemChannels/textInput-constant.html
static NSString* const kSetClientMethod = @"TextInput.setClient";
static NSString* const kShowMethod = @"TextInput.show";
static NSString* const kHideMethod = @"TextInput.hide";
static NSString* const kClearClientMethod = @"TextInput.clearClient";
static NSString* const kSetEditingStateMethod = @"TextInput.setEditingState";
static NSString* const kSetEditableSizeAndTransform = @"TextInput.setEditableSizeAndTransform";
static NSString* const kSetCaretRect = @"TextInput.setCaretRect";
static NSString* const kUpdateEditStateResponseMethod = @"TextInputClient.updateEditingState";
static NSString* const kUpdateEditStateWithDeltasResponseMethod =
    @"TextInputClient.updateEditingStateWithDeltas";
static NSString* const kPerformAction = @"TextInputClient.performAction";
static NSString* const kPerformSelectors = @"TextInputClient.performSelectors";
static NSString* const kMultilineInputType = @"TextInputType.multiline";

#pragma mark - TextInputConfiguration field names
static NSString* const kSecureTextEntry = @"obscureText";
static NSString* const kTextInputAction = @"inputAction";
static NSString* const kEnableDeltaModel = @"enableDeltaModel";
static NSString* const kTextInputType = @"inputType";
static NSString* const kTextInputTypeName = @"name";
static NSString* const kSelectionBaseKey = @"selectionBase";
static NSString* const kSelectionExtentKey = @"selectionExtent";
static NSString* const kSelectionAffinityKey = @"selectionAffinity";
static NSString* const kSelectionIsDirectionalKey = @"selectionIsDirectional";
static NSString* const kComposingBaseKey = @"composingBase";
static NSString* const kComposingExtentKey = @"composingExtent";
static NSString* const kTextKey = @"text";
static NSString* const kTransformKey = @"transform";
static NSString* const kAssociatedAutofillFields = @"fields";

// TextInputConfiguration.autofill and sub-field names
static NSString* const kAutofillProperties = @"autofill";
static NSString* const kAutofillId = @"uniqueIdentifier";
static NSString* const kAutofillEditingValue = @"editingValue";
static NSString* const kAutofillHints = @"hints";

// TextAffinity types
static NSString* const kTextAffinityDownstream = @"TextAffinity.downstream";
static NSString* const kTextAffinityUpstream = @"TextAffinity.upstream";

// TextInputAction types
static NSString* const kInputActionNewline = @"TextInputAction.newline";

#pragma mark - Enums
/**
 * The affinity of the current cursor position. If the cursor is at a position
 * representing a soft line break, the cursor may be drawn either at the end of
 * the current line (upstream) or at the beginning of the next (downstream).
 */
typedef NS_ENUM(NSUInteger, FlutterTextAffinity) {
  kFlutterTextAffinityUpstream,
  kFlutterTextAffinityDownstream
};

#pragma mark - Static functions

/*
 * Updates a range given base and extent fields.
 */
static flutter::TextRange RangeFromBaseExtent(NSNumber* base,
                                              NSNumber* extent,
                                              const flutter::TextRange& range) {
  if (base == nil || extent == nil) {
    return range;
  }
  if (base.intValue == -1 && extent.intValue == -1) {
    return flutter::TextRange(0, 0);
  }
  return flutter::TextRange([base unsignedLongValue], [extent unsignedLongValue]);
}

// Returns the autofill hint content type, if specified; otherwise nil.
static NSString* GetAutofillHint(NSDictionary* autofill) {
  NSArray<NSString*>* hints = autofill[kAutofillHints];
  return hints.count > 0 ? hints[0] : nil;
}

// Returns the text content type for the specified TextInputConfiguration.
// NSTextContentType is only available for macOS 11.0 and later.
static NSTextContentType GetTextContentType(NSDictionary* configuration)
    API_AVAILABLE(macos(11.0)) {
  // Check autofill hints.
  NSDictionary* autofill = configuration[kAutofillProperties];
  if (autofill) {
    NSString* hint = GetAutofillHint(autofill);
    if ([hint isEqualToString:@"username"]) {
      return NSTextContentTypeUsername;
    }
    if ([hint isEqualToString:@"password"]) {
      return NSTextContentTypePassword;
    }
    if ([hint isEqualToString:@"oneTimeCode"]) {
      return NSTextContentTypeOneTimeCode;
    }
  }
  // If no autofill hints, guess based on other attributes.
  if ([configuration[kSecureTextEntry] boolValue]) {
    return NSTextContentTypePassword;
  }
  return nil;
}

// Returns YES if configuration describes a field for which autocomplete should be enabled for
// the specified TextInputConfiguration. Autocomplete is enabled by default, but will be disabled
// if the field is password-related, or if the configuration contains no autofill settings.
static BOOL EnableAutocompleteForTextInputConfiguration(NSDictionary* configuration) {
  // Disable if obscureText is set.
  if ([configuration[kSecureTextEntry] boolValue]) {
    return NO;
  }

  // Disable if autofill properties are not set.
  NSDictionary* autofill = configuration[kAutofillProperties];
  if (autofill == nil) {
    return NO;
  }

  // Disable if autofill properties indicate a username/password.
  // See: https://github.com/flutter/flutter/issues/119824
  NSString* hint = GetAutofillHint(autofill);
  if ([hint isEqualToString:@"password"] || [hint isEqualToString:@"username"]) {
    return NO;
  }
  return YES;
}

// Returns YES if configuration describes a field for which autocomplete should be enabled.
// Autocomplete is enabled by default, but will be disabled if the field is password-related, or if
// the configuration contains no autofill settings.
//
// In the case where the current field is part of an AutofillGroup, the configuration will have
// a fields attribute with a list of TextInputConfigurations, one for each field. In the case where
// any field in the group disables autocomplete, we disable it for all.
static BOOL EnableAutocomplete(NSDictionary* configuration) {
  for (NSDictionary* field in configuration[kAssociatedAutofillFields]) {
    if (!EnableAutocompleteForTextInputConfiguration(field)) {
      return NO;
    }
  }

  // Check the top-level TextInputConfiguration.
  return EnableAutocompleteForTextInputConfiguration(configuration);
}

#pragma mark - NSEvent (KeyEquivalentMarker) protocol

@interface NSEvent (KeyEquivalentMarker)

// Internally marks that the event was received through performKeyEquivalent:.
// When text editing is active, keyboard events that have modifier keys pressed
// are received through performKeyEquivalent: instead of keyDown:. If such event
// is passed to TextInputContext but doesn't result in a text editing action it
// needs to be forwarded by FlutterKeyboardManager to the next responder.
- (void)markAsKeyEquivalent;

// Returns YES if the event is marked as a key equivalent.
- (BOOL)isKeyEquivalent;

@end

@implementation NSEvent (KeyEquivalentMarker)

// This field doesn't need a value because only its address is used as a unique identifier.
static char markerKey;

- (void)markAsKeyEquivalent {
  objc_setAssociatedObject(self, &markerKey, @true, OBJC_ASSOCIATION_RETAIN);
}

- (BOOL)isKeyEquivalent {
  return [objc_getAssociatedObject(self, &markerKey) boolValue] == YES;
}

@end

#pragma mark - FlutterTextInputPlugin private interface

/**
 * Private properties of FlutterTextInputPlugin.
 */
@interface FlutterTextInputPlugin ()

/**
 * A text input context, representing a connection to the Cocoa text input system.
 */
@property(nonatomic) NSTextInputContext* textInputContext;

/**
 * The channel used to communicate with Flutter.
 */
@property(nonatomic) FlutterMethodChannel* channel;

/**
 * The FlutterViewController to manage input for.
 */
@property(nonatomic, weak) FlutterViewController* flutterViewController;

/**
 * Whether the text input is shown in the view.
 *
 * Defaults to TRUE on startup.
 */
@property(nonatomic) BOOL shown;

/**
 * The current state of the keyboard and pressed keys.
 */
@property(nonatomic) uint64_t previouslyPressedFlags;

/**
 * The affinity for the current cursor position.
 */
@property FlutterTextAffinity textAffinity;

/**
 * ID of the text input client.
 */
@property(nonatomic, nonnull) NSNumber* clientID;

/**
 * Keyboard type of the client. See available options:
 * https://api.flutter.dev/flutter/services/TextInputType-class.html
 */
@property(nonatomic, nonnull) NSString* inputType;

/**
 * An action requested by the user on the input client. See available options:
 * https://api.flutter.dev/flutter/services/TextInputAction-class.html
 */
@property(nonatomic, nonnull) NSString* inputAction;

/**
 * Set to true if the last event fed to the input context produced a text editing command
 * or text output. It is reset to false at the beginning of every key event, and is only
 * used while processing this event.
 */
@property(nonatomic) BOOL eventProducedOutput;

/**
 * Whether to enable the sending of text input updates from the engine to the
 * framework as TextEditingDeltas rather than as one TextEditingValue.
 * For more information on the delta model, see:
 * https://master-api.flutter.dev/flutter/services/TextInputConfiguration/enableDeltaModel.html
 */
@property(nonatomic) BOOL enableDeltaModel;

/**
 * Used to gather multiple selectors performed in one run loop turn. These
 * will be all sent in one platform channel call so that the framework can process
 * them in single microtask.
 */
@property(nonatomic) NSMutableArray* pendingSelectors;

/**
 * Handles a Flutter system message on the text input channel.
 */
- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;

/**
 * Updates the text input model with state received from the framework via the
 * TextInput.setEditingState message.
 */
- (void)setEditingState:(NSDictionary*)state;

/**
 * Informs the Flutter framework of changes to the text input model's state by
 * sending the entire new state.
 */
- (void)updateEditState;

/**
 * Informs the Flutter framework of changes to the text input model's state by
 * sending only the difference.
 */
- (void)updateEditStateWithDelta:(const flutter::TextEditingDelta)delta;

/**
 * Updates the stringValue and selectedRange that stored in the NSTextView interface
 * that this plugin inherits from.
 *
 * If there is a FlutterTextField uses this plugin as its field editor, this method
 * will update the stringValue and selectedRange through the API of the FlutterTextField.
 */
- (void)updateTextAndSelection;

/**
 * Return the string representation of the current textAffinity as it should be
 * sent over the FlutterMethodChannel.
 */
- (NSString*)textAffinityString;

/**
 * Allow overriding run loop mode for test.
 */
@property(readwrite, nonatomic) NSString* customRunLoopMode;

@end

#pragma mark - FlutterTextInputPlugin

@implementation FlutterTextInputPlugin {
  /**
   * The currently active text input model.
   */
  std::unique_ptr<flutter::TextInputModel> _activeModel;

  /**
   * Transform for current the editable. Used to determine position of accent selection menu.
   */
  CATransform3D _editableTransform;

  /**
   * Current position of caret in local (editable) coordinates.
   */
  CGRect _caretRect;
}

- (instancetype)initWithViewController:(FlutterViewController*)viewController {
  // The view needs an empty frame otherwise it is visible on dark background.
  // https://github.com/flutter/flutter/issues/118504
  self = [super initWithFrame:NSZeroRect];
  self.clipsToBounds = YES;
  if (self != nil) {
    _flutterViewController = viewController;
    _channel = [FlutterMethodChannel methodChannelWithName:kTextInputChannel
                                           binaryMessenger:viewController.engine.binaryMessenger
                                                     codec:[FlutterJSONMethodCodec sharedInstance]];
    _shown = FALSE;
    // NSTextView does not support _weak reference, so this class has to
    // use __unsafe_unretained and manage the reference by itself.
    //
    // Since the dealloc removes the handler, the pointer should
    // be valid if the handler is ever called.
    __unsafe_unretained FlutterTextInputPlugin* unsafeSelf = self;
    [_channel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
      [unsafeSelf handleMethodCall:call result:result];
    }];
    _textInputContext = [[NSTextInputContext alloc] initWithClient:unsafeSelf];
    _previouslyPressedFlags = 0;

    // Initialize with the zero matrix which is not
    // an affine transform.
    _editableTransform = CATransform3D();
    _caretRect = CGRectNull;
  }
  return self;
}

- (BOOL)isFirstResponder {
  if (!self.flutterViewController.viewLoaded) {
    return false;
  }
  return [self.flutterViewController.view.window firstResponder] == self;
}

- (void)dealloc {
  [_channel setMethodCallHandler:nil];
}

#pragma mark - Private

- (void)resignAndRemoveFromSuperview {
  if (self.superview != nil) {
    [self.window makeFirstResponder:_flutterViewController.flutterView];
    [self removeFromSuperview];
  }
}

- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result {
  BOOL handled = YES;
  NSString* method = call.method;
  if ([method isEqualToString:kSetClientMethod]) {
    if (!call.arguments[0] || !call.arguments[1]) {
      result([FlutterError
          errorWithCode:@"error"
                message:@"Missing arguments"
                details:@"Missing arguments while trying to set a text input client"]);
      return;
    }
    NSNumber* clientID = call.arguments[0];
    if (clientID != nil) {
      NSDictionary* config = call.arguments[1];

      _clientID = clientID;
      _inputAction = config[kTextInputAction];
      _enableDeltaModel = [config[kEnableDeltaModel] boolValue];
      NSDictionary* inputTypeInfo = config[kTextInputType];
      _inputType = inputTypeInfo[kTextInputTypeName];
      self.textAffinity = kFlutterTextAffinityUpstream;
      self.automaticTextCompletionEnabled = EnableAutocomplete(config);
      if (@available(macOS 11.0, *)) {
        self.contentType = GetTextContentType(config);
      }

      _activeModel = std::make_unique<flutter::TextInputModel>();
    }
  } else if ([method isEqualToString:kShowMethod]) {
    // Ensure the plugin is in hierarchy. Only do this with accessibility disabled.
    // When accessibility is enabled cocoa will reparent the plugin inside
    // FlutterTextField in [FlutterTextField startEditing].
    if (_client == nil) {
      [_flutterViewController.view addSubview:self];
    }
    [self.window makeFirstResponder:self];
    _shown = TRUE;
  } else if ([method isEqualToString:kHideMethod]) {
    [self resignAndRemoveFromSuperview];
    _shown = FALSE;
  } else if ([method isEqualToString:kClearClientMethod]) {
    [self resignAndRemoveFromSuperview];
    // If there's an active mark region, commit it, end composing, and clear the IME's mark text.
    if (_activeModel && _activeModel->composing()) {
      _activeModel->CommitComposing();
      _activeModel->EndComposing();
    }
    [_textInputContext discardMarkedText];

    _clientID = nil;
    _inputAction = nil;
    _enableDeltaModel = NO;
    _inputType = nil;
    _activeModel = nullptr;
  } else if ([method isEqualToString:kSetEditingStateMethod]) {
    NSDictionary* state = call.arguments;
    [self setEditingState:state];
  } else if ([method isEqualToString:kSetEditableSizeAndTransform]) {
    NSDictionary* state = call.arguments;
    [self setEditableTransform:state[kTransformKey]];
  } else if ([method isEqualToString:kSetCaretRect]) {
    NSDictionary* rect = call.arguments;
    [self updateCaretRect:rect];
  } else {
    handled = NO;
  }
  result(handled ? nil : FlutterMethodNotImplemented);
}

- (void)setEditableTransform:(NSArray*)matrix {
  CATransform3D* transform = &_editableTransform;

  transform->m11 = [matrix[0] doubleValue];
  transform->m12 = [matrix[1] doubleValue];
  transform->m13 = [matrix[2] doubleValue];
  transform->m14 = [matrix[3] doubleValue];

  transform->m21 = [matrix[4] doubleValue];
  transform->m22 = [matrix[5] doubleValue];
  transform->m23 = [matrix[6] doubleValue];
  transform->m24 = [matrix[7] doubleValue];

  transform->m31 = [matrix[8] doubleValue];
  transform->m32 = [matrix[9] doubleValue];
  transform->m33 = [matrix[10] doubleValue];
  transform->m34 = [matrix[11] doubleValue];

  transform->m41 = [matrix[12] doubleValue];
  transform->m42 = [matrix[13] doubleValue];
  transform->m43 = [matrix[14] doubleValue];
  transform->m44 = [matrix[15] doubleValue];
}

- (void)updateCaretRect:(NSDictionary*)dictionary {
  NSAssert(dictionary[@"x"] != nil && dictionary[@"y"] != nil && dictionary[@"width"] != nil &&
               dictionary[@"height"] != nil,
           @"Expected a dictionary representing a CGRect, got %@", dictionary);
  _caretRect = CGRectMake([dictionary[@"x"] doubleValue], [dictionary[@"y"] doubleValue],
                          [dictionary[@"width"] doubleValue], [dictionary[@"height"] doubleValue]);
}

- (void)setEditingState:(NSDictionary*)state {
  NSString* selectionAffinity = state[kSelectionAffinityKey];
  if (selectionAffinity != nil) {
    _textAffinity = [selectionAffinity isEqualToString:kTextAffinityUpstream]
                        ? kFlutterTextAffinityUpstream
                        : kFlutterTextAffinityDownstream;
  }

  NSString* text = state[kTextKey];

  flutter::TextRange selected_range = RangeFromBaseExtent(
      state[kSelectionBaseKey], state[kSelectionExtentKey], _activeModel->selection());
  _activeModel->SetSelection(selected_range);

  flutter::TextRange composing_range = RangeFromBaseExtent(
      state[kComposingBaseKey], state[kComposingExtentKey], _activeModel->composing_range());

  const bool wasComposing = _activeModel->composing();
  _activeModel->SetText([text UTF8String], selected_range, composing_range);
  if (composing_range.collapsed() && wasComposing) {
    [_textInputContext discardMarkedText];
  }
  [_client startEditing];

  [self updateTextAndSelection];
}

- (NSDictionary*)editingState {
  if (_activeModel == nullptr) {
    return nil;
  }

  NSString* const textAffinity = [self textAffinityString];

  int composingBase = _activeModel->composing() ? _activeModel->composing_range().base() : -1;
  int composingExtent = _activeModel->composing() ? _activeModel->composing_range().extent() : -1;

  return @{
    kSelectionBaseKey : @(_activeModel->selection().base()),
    kSelectionExtentKey : @(_activeModel->selection().extent()),
    kSelectionAffinityKey : textAffinity,
    kSelectionIsDirectionalKey : @NO,
    kComposingBaseKey : @(composingBase),
    kComposingExtentKey : @(composingExtent),
    kTextKey : [NSString stringWithUTF8String:_activeModel->GetText().c_str()] ?: [NSNull null],
  };
}

- (void)updateEditState {
  if (_activeModel == nullptr) {
    return;
  }

  NSDictionary* state = [self editingState];
  [_channel invokeMethod:kUpdateEditStateResponseMethod arguments:@[ self.clientID, state ]];
  [self updateTextAndSelection];
}

- (void)updateEditStateWithDelta:(const flutter::TextEditingDelta)delta {
  NSUInteger selectionBase = _activeModel->selection().base();
  NSUInteger selectionExtent = _activeModel->selection().extent();
  int composingBase = _activeModel->composing() ? _activeModel->composing_range().base() : -1;
  int composingExtent = _activeModel->composing() ? _activeModel->composing_range().extent() : -1;

  NSString* const textAffinity = [self textAffinityString];

  NSDictionary* deltaToFramework = @{
    @"oldText" : @(delta.old_text().c_str()),
    @"deltaText" : @(delta.delta_text().c_str()),
    @"deltaStart" : @(delta.delta_start()),
    @"deltaEnd" : @(delta.delta_end()),
    @"selectionBase" : @(selectionBase),
    @"selectionExtent" : @(selectionExtent),
    @"selectionAffinity" : textAffinity,
    @"selectionIsDirectional" : @(false),
    @"composingBase" : @(composingBase),
    @"composingExtent" : @(composingExtent),
  };

  NSDictionary* deltas = @{
    @"deltas" : @[ deltaToFramework ],
  };

  [_channel invokeMethod:kUpdateEditStateWithDeltasResponseMethod
               arguments:@[ self.clientID, deltas ]];
  [self updateTextAndSelection];
}

- (void)updateTextAndSelection {
  NSAssert(_activeModel != nullptr, @"Flutter text model must not be null.");
  NSString* text = @(_activeModel->GetText().data());
  int start = _activeModel->selection().base();
  int extend = _activeModel->selection().extent();
  NSRange selection = NSMakeRange(MIN(start, extend), ABS(start - extend));
  // There may be a native text field client if VoiceOver is on.
  // In this case, this plugin has to update text and selection through
  // the client in order for VoiceOver to announce the text editing
  // properly.
  if (_client) {
    [_client updateString:text withSelection:selection];
  } else {
    self.string = text;
    [self setSelectedRange:selection];
  }
}

- (NSString*)textAffinityString {
  return (self.textAffinity == kFlutterTextAffinityUpstream) ? kTextAffinityUpstream
                                                             : kTextAffinityDownstream;
}

- (BOOL)handleKeyEvent:(NSEvent*)event {
  if (event.type == NSEventTypeKeyUp ||
      (event.type == NSEventTypeFlagsChanged && event.modifierFlags < _previouslyPressedFlags)) {
    return NO;
  }
  _previouslyPressedFlags = event.modifierFlags;
  if (!_shown) {
    return NO;
  }

  _eventProducedOutput = NO;
  BOOL res = [_textInputContext handleEvent:event];
  // NSTextInputContext#handleEvent returns YES if the context handles the event. One of the reasons
  // the event is handled is because it's a key equivalent. But a key equivalent might produce a
  // text command (indicated by calling doCommandBySelector) or might not (for example, Cmd+Q). In
  // the latter case, this command somehow has not been executed yet and Flutter must dispatch it to
  // the next responder. See https://github.com/flutter/flutter/issues/106354 .
  // The event is also not redispatched if there is IME composition active, because it might be
  // handled by the IME. See https://github.com/flutter/flutter/issues/134699

  // both NSEventModifierFlagNumericPad and NSEventModifierFlagFunction are set for arrow keys.
  bool is_navigation = event.modifierFlags & NSEventModifierFlagFunction &&
                       event.modifierFlags & NSEventModifierFlagNumericPad;
  bool is_navigation_in_ime = is_navigation && self.hasMarkedText;

  if (event.isKeyEquivalent && !is_navigation_in_ime && !_eventProducedOutput) {
    return NO;
  }
  return res;
}

#pragma mark -
#pragma mark NSResponder

- (void)keyDown:(NSEvent*)event {
  [self.flutterViewController keyDown:event];
}

- (void)keyUp:(NSEvent*)event {
  [self.flutterViewController keyUp:event];
}

- (BOOL)performKeyEquivalent:(NSEvent*)event {
  if ([_flutterViewController isDispatchingKeyEvent:event]) {
    // When NSWindow is nextResponder, keyboard manager will send to it
    // unhandled events (through [NSWindow keyDown:]). If event has both
    // control and cmd modifiers set (i.e. cmd+control+space - emoji picker)
    // NSWindow will then send this event as performKeyEquivalent: to first
    // responder, which is FlutterTextInputPlugin. If that's the case, the
    // plugin must not handle the event, otherwise the emoji picker would not
    // work (due to first responder returning YES from performKeyEquivalent:)
    // and there would be endless loop, because FlutterViewController will
    // send the event back to [keyboardManager handleEvent:].
    return NO;
  }
  [event markAsKeyEquivalent];
  [self.flutterViewController keyDown:event];
  return YES;
}

- (void)flagsChanged:(NSEvent*)event {
  [self.flutterViewController flagsChanged:event];
}

- (void)mouseDown:(NSEvent*)event {
  [self.flutterViewController mouseDown:event];
}

- (void)mouseUp:(NSEvent*)event {
  [self.flutterViewController mouseUp:event];
}

- (void)mouseDragged:(NSEvent*)event {
  [self.flutterViewController mouseDragged:event];
}

- (void)rightMouseDown:(NSEvent*)event {
  [self.flutterViewController rightMouseDown:event];
}

- (void)rightMouseUp:(NSEvent*)event {
  [self.flutterViewController rightMouseUp:event];
}

- (void)rightMouseDragged:(NSEvent*)event {
  [self.flutterViewController rightMouseDragged:event];
}

- (void)otherMouseDown:(NSEvent*)event {
  [self.flutterViewController otherMouseDown:event];
}

- (void)otherMouseUp:(NSEvent*)event {
  [self.flutterViewController otherMouseUp:event];
}

- (void)otherMouseDragged:(NSEvent*)event {
  [self.flutterViewController otherMouseDragged:event];
}

- (void)mouseMoved:(NSEvent*)event {
  [self.flutterViewController mouseMoved:event];
}

- (void)scrollWheel:(NSEvent*)event {
  [self.flutterViewController scrollWheel:event];
}

- (NSTextInputContext*)inputContext {
  return _textInputContext;
}

#pragma mark -
#pragma mark NSTextInputClient

- (void)insertTab:(id)sender {
  // Implementing insertTab: makes AppKit send tab as command, instead of
  // insertText with '\t'.
}

- (void)insertText:(id)string replacementRange:(NSRange)range {
  if (_activeModel == nullptr) {
    return;
  }

  _eventProducedOutput |= true;

  if (range.location != NSNotFound) {
    // The selected range can actually have negative numbers, since it can start
    // at the end of the range if the user selected the text going backwards.
    // Cast to a signed type to determine whether or not the selection is reversed.
    long signedLength = static_cast<long>(range.length);
    long location = range.location;
    long textLength = _activeModel->text_range().end();

    size_t base = std::clamp(location, 0L, textLength);
    size_t extent = std::clamp(location + signedLength, 0L, textLength);

    _activeModel->SetSelection(flutter::TextRange(base, extent));
  }

  flutter::TextRange oldSelection = _activeModel->selection();
  flutter::TextRange composingBeforeChange = _activeModel->composing_range();
  flutter::TextRange replacedRange(-1, -1);

  std::string textBeforeChange = _activeModel->GetText().c_str();
  std::string utf8String = [string UTF8String];
  _activeModel->AddText(utf8String);
  if (_activeModel->composing()) {
    replacedRange = composingBeforeChange;
    _activeModel->CommitComposing();
    _activeModel->EndComposing();
  } else {
    replacedRange = range.location == NSNotFound
                        ? flutter::TextRange(oldSelection.base(), oldSelection.extent())
                        : flutter::TextRange(range.location, range.location + range.length);
  }
  if (_enableDeltaModel) {
    [self updateEditStateWithDelta:flutter::TextEditingDelta(textBeforeChange, replacedRange,
                                                             utf8String)];
  } else {
    [self updateEditState];
  }
}

- (void)doCommandBySelector:(SEL)selector {
  _eventProducedOutput |= selector != NSSelectorFromString(@"noop:");
  if ([self respondsToSelector:selector]) {
    // Note: The more obvious [self performSelector...] doesn't give ARC enough information to
    // handle retain semantics properly. See https://stackoverflow.com/questions/7017281/ for more
    // information.
    IMP imp = [self methodForSelector:selector];
    void (*func)(id, SEL, id) = reinterpret_cast<void (*)(id, SEL, id)>(imp);
    func(self, selector, nil);
  }
  if (self.clientID == nil) {
    // The macOS may still call selector even if it is no longer a first responder.
    return;
  }

  if (selector == @selector(insertNewline:)) {
    // Already handled through text insertion (multiline) or action.
    return;
  }

  // Group multiple selectors received within a single run loop turn so that
  // the framework can process them in single microtask.
  NSString* name = NSStringFromSelector(selector);
  if (_pendingSelectors == nil) {
    _pendingSelectors = [NSMutableArray array];
  }
  [_pendingSelectors addObject:name];

  if (_pendingSelectors.count == 1) {
    __weak NSMutableArray* selectors = _pendingSelectors;
    __weak FlutterMethodChannel* channel = _channel;
    __weak NSNumber* clientID = self.clientID;

    CFStringRef runLoopMode = self.customRunLoopMode != nil
                                  ? (__bridge CFStringRef)self.customRunLoopMode
                                  : kCFRunLoopCommonModes;

    CFRunLoopPerformBlock(CFRunLoopGetMain(), runLoopMode, ^{
      if (selectors.count > 0) {
        [channel invokeMethod:kPerformSelectors arguments:@[ clientID, selectors ]];
        [selectors removeAllObjects];
      }
    });
  }
}

- (void)insertNewline:(id)sender {
  if (_activeModel == nullptr) {
    return;
  }
  if (_activeModel->composing()) {
    _activeModel->CommitComposing();
    _activeModel->EndComposing();
  }
  if ([self.inputType isEqualToString:kMultilineInputType] &&
      [self.inputAction isEqualToString:kInputActionNewline]) {
    [self insertText:@"\n" replacementRange:self.selectedRange];
  }
  [_channel invokeMethod:kPerformAction arguments:@[ self.clientID, self.inputAction ]];
}

- (void)setMarkedText:(id)string
        selectedRange:(NSRange)selectedRange
     replacementRange:(NSRange)replacementRange {
  if (_activeModel == nullptr) {
    return;
  }
  std::string textBeforeChange = _activeModel->GetText().c_str();
  if (!_activeModel->composing()) {
    _activeModel->BeginComposing();
  }

  if (replacementRange.location != NSNotFound) {
    // According to the NSTextInputClient documentation replacementRange is
    // computed from the beginning of the marked text. That doesn't seem to be
    // the case, because in situations where the replacementRange is actually
    // specified (i.e. when switching between characters equivalent after long
    // key press) the replacementRange is provided while there is no composition.
    _activeModel->SetComposingRange(
        flutter::TextRange(replacementRange.location,
                           replacementRange.location + replacementRange.length),
        0);
  }

  flutter::TextRange composingBeforeChange = _activeModel->composing_range();
  flutter::TextRange selectionBeforeChange = _activeModel->selection();

  // Input string may be NSString or NSAttributedString.
  BOOL isAttributedString = [string isKindOfClass:[NSAttributedString class]];
  const NSString* rawString = isAttributedString ? [string string] : string;
  _activeModel->UpdateComposingText(
      (const char16_t*)[rawString cStringUsingEncoding:NSUTF16StringEncoding],
      flutter::TextRange(selectedRange.location, selectedRange.location + selectedRange.length));

  if (_enableDeltaModel) {
    std::string marked_text = [rawString UTF8String];
    [self updateEditStateWithDelta:flutter::TextEditingDelta(textBeforeChange,
                                                             selectionBeforeChange.collapsed()
                                                                 ? composingBeforeChange
                                                                 : selectionBeforeChange,
                                                             marked_text)];
  } else {
    [self updateEditState];
  }
}

- (void)unmarkText {
  if (_activeModel == nullptr) {
    return;
  }
  _activeModel->CommitComposing();
  _activeModel->EndComposing();
  if (_enableDeltaModel) {
    [self updateEditStateWithDelta:flutter::TextEditingDelta(_activeModel->GetText().c_str())];
  } else {
    [self updateEditState];
  }
}

- (NSRange)markedRange {
  if (_activeModel == nullptr) {
    return NSMakeRange(NSNotFound, 0);
  }
  return NSMakeRange(
      _activeModel->composing_range().base(),
      _activeModel->composing_range().extent() - _activeModel->composing_range().base());
}

- (BOOL)hasMarkedText {
  return _activeModel != nullptr && _activeModel->composing_range().length() > 0;
}

- (NSAttributedString*)attributedSubstringForProposedRange:(NSRange)range
                                               actualRange:(NSRangePointer)actualRange {
  if (_activeModel == nullptr) {
    return nil;
  }
  NSString* text = [NSString stringWithUTF8String:_activeModel->GetText().c_str()];
  if (range.location >= text.length) {
    return nil;
  }
  range.length = std::min(range.length, text.length - range.location);
  if (actualRange != nil) {
    *actualRange = range;
  }
  NSString* substring = [text substringWithRange:range];
  return [[NSAttributedString alloc] initWithString:substring attributes:nil];
}

- (NSArray<NSString*>*)validAttributesForMarkedText {
  return @[];
}

// Returns the bounding CGRect of the transformed incomingRect, in screen
// coordinates.
- (CGRect)screenRectFromFrameworkTransform:(CGRect)incomingRect {
  CGPoint points[] = {
      incomingRect.origin,
      CGPointMake(incomingRect.origin.x, incomingRect.origin.y + incomingRect.size.height),
      CGPointMake(incomingRect.origin.x + incomingRect.size.width, incomingRect.origin.y),
      CGPointMake(incomingRect.origin.x + incomingRect.size.width,
                  incomingRect.origin.y + incomingRect.size.height)};

  CGPoint origin = CGPointMake(CGFLOAT_MAX, CGFLOAT_MAX);
  CGPoint farthest = CGPointMake(-CGFLOAT_MAX, -CGFLOAT_MAX);

  for (int i = 0; i < 4; i++) {
    const CGPoint point = points[i];

    CGFloat x = _editableTransform.m11 * point.x + _editableTransform.m21 * point.y +
                _editableTransform.m41;
    CGFloat y = _editableTransform.m12 * point.x + _editableTransform.m22 * point.y +
                _editableTransform.m42;

    const CGFloat w = _editableTransform.m14 * point.x + _editableTransform.m24 * point.y +
                      _editableTransform.m44;

    if (w == 0.0) {
      return CGRectZero;
    } else if (w != 1.0) {
      x /= w;
      y /= w;
    }

    origin.x = MIN(origin.x, x);
    origin.y = MIN(origin.y, y);
    farthest.x = MAX(farthest.x, x);
    farthest.y = MAX(farthest.y, y);
  }

  const NSView* fromView = self.flutterViewController.flutterView;
  const CGRect rectInWindow = [fromView
      convertRect:CGRectMake(origin.x, origin.y, farthest.x - origin.x, farthest.y - origin.y)
           toView:nil];
  NSWindow* window = fromView.window;
  return window ? [window convertRectToScreen:rectInWindow] : rectInWindow;
}

- (NSRect)firstRectForCharacterRange:(NSRange)range actualRange:(NSRangePointer)actualRange {
  // This only determines position of caret instead of any arbitrary range, but it's enough
  // to properly position accent selection popup
  return !self.flutterViewController.viewLoaded || CGRectEqualToRect(_caretRect, CGRectNull)
             ? CGRectZero
             : [self screenRectFromFrameworkTransform:_caretRect];
}

- (NSUInteger)characterIndexForPoint:(NSPoint)point {
  // TODO(cbracken): Implement.
  // Note: This function can't easily be implemented under the system-message architecture.
  return 0;
}

@end
