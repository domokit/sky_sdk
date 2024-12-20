// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"

#include <Carbon/Carbon.h>
#import <objc/message.h>

#include "flutter/common/constants.h"
#include "flutter/fml/platform/darwin/cf_utils.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterCodecs.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyPrimaryResponder.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyboardManager.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterRenderer.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputSemanticsObject.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterView.h"
#include "flutter/shell/platform/embedder/embedder.h"

#pragma mark - Static types and data.

namespace {
using flutter::KeyboardLayoutNotifier;
using flutter::LayoutClue;

// Use different device ID for mouse and pan/zoom events, since we can't differentiate the actual
// device (mouse v.s. trackpad).
static constexpr int32_t kMousePointerDeviceId = 0;
static constexpr int32_t kPointerPanZoomDeviceId = 1;

// A trackpad touch following inertial scrolling should cause an inertia cancel
// event to be issued. Use a window of 50 milliseconds after the scroll to account
// for delays in event propagation observed in macOS Ventura.
static constexpr double kTrackpadTouchInertiaCancelWindowMs = 0.050;

/**
 * State tracking for mouse events, to adapt between the events coming from the system and the
 * events that the embedding API expects.
 */
struct MouseState {
  /**
   * The currently pressed buttons, as represented in FlutterPointerEvent.
   */
  int64_t buttons = 0;

  /**
   * The accumulated gesture pan.
   */
  CGFloat delta_x = 0;
  CGFloat delta_y = 0;

  /**
   * The accumulated gesture zoom scale.
   */
  CGFloat scale = 0;

  /**
   * The accumulated gesture rotation.
   */
  CGFloat rotation = 0;

  /**
   * Whether or not a kAdd event has been sent (or sent again since the last kRemove if tracking is
   * enabled). Used to determine whether to send a kAdd event before sending an incoming mouse
   * event, since Flutter expects pointers to be added before events are sent for them.
   */
  bool flutter_state_is_added = false;

  /**
   * Whether or not a kDown has been sent since the last kAdd/kUp.
   */
  bool flutter_state_is_down = false;

  /**
   * Whether or not mouseExited: was received while a button was down. Cocoa's behavior when
   * dragging out of a tracked area is to send an exit, then keep sending drag events until the last
   * button is released. Flutter doesn't expect to receive events after a kRemove, so the kRemove
   * for the exit needs to be delayed until after the last mouse button is released. If cursor
   * returns back to the window while still dragging, the flag is cleared in mouseEntered:.
   */
  bool has_pending_exit = false;

  /*
   * Whether or not a kPanZoomStart has been sent since the last kAdd/kPanZoomEnd.
   */
  bool flutter_state_is_pan_zoom_started = false;

  /**
   * State of pan gesture.
   */
  NSEventPhase pan_gesture_phase = NSEventPhaseNone;

  /**
   * State of scale gesture.
   */
  NSEventPhase scale_gesture_phase = NSEventPhaseNone;

  /**
   * State of rotate gesture.
   */
  NSEventPhase rotate_gesture_phase = NSEventPhaseNone;

  /**
   * Time of last scroll momentum event.
   */
  NSTimeInterval last_scroll_momentum_changed_time = 0;

  /**
   * Resets all gesture state to default values.
   */
  void GestureReset() {
    delta_x = 0;
    delta_y = 0;
    scale = 0;
    rotation = 0;
    flutter_state_is_pan_zoom_started = false;
    pan_gesture_phase = NSEventPhaseNone;
    scale_gesture_phase = NSEventPhaseNone;
    rotate_gesture_phase = NSEventPhaseNone;
  }

  /**
   * Resets all state to default values.
   */
  void Reset() {
    flutter_state_is_added = false;
    flutter_state_is_down = false;
    has_pending_exit = false;
    buttons = 0;
  }
};

}  // namespace

#pragma mark - Private interface declaration.

/**
 * FlutterViewWrapper is a convenience class that wraps a FlutterView and provides
 * a mechanism to attach AppKit views such as FlutterTextField without affecting
 * the accessibility subtree of the wrapped FlutterView itself.
 *
 * The FlutterViewController uses this class to create its content view. When
 * any of the accessibility services (e.g. VoiceOver) is turned on, the accessibility
 * bridge creates FlutterTextFields that interact with the service. The bridge has to
 * attach the FlutterTextField somewhere in the view hierarchy in order for the
 * FlutterTextField to interact correctly with VoiceOver. Those FlutterTextFields
 * will be attached to this view so that they won't affect the accessibility subtree
 * of FlutterView.
 */
@interface FlutterViewWrapper : NSView

- (void)setBackgroundColor:(NSColor*)color;

@end

/**
 * Private interface declaration for FlutterViewController.
 */
@interface FlutterViewController () <FlutterViewDelegate>

/**
 * The tracking area used to generate hover events, if enabled.
 */
@property(nonatomic) NSTrackingArea* trackingArea;

/**
 * The current state of the mouse and the sent mouse events.
 */
@property(nonatomic) MouseState mouseState;

/**
 * Event monitor for keyUp events.
 */
@property(nonatomic) id keyUpMonitor;

/**
 * Pointer to a keyboard manager, a hub that manages how key events are
 * dispatched to various Flutter key responders, and whether the event is
 * propagated to the next NSResponder.
 */
@property(nonatomic, readonly, nonnull) FlutterKeyboardManager* keyboardManager;

@property(nonatomic) KeyboardLayoutNotifier keyboardLayoutNotifier;

@property(nonatomic) NSData* keyboardLayoutData;

/**
 * Starts running |engine|, including any initial setup.
 */
- (BOOL)launchEngine;

/**
 * Updates |trackingArea| for the current tracking settings, creating it with
 * the correct mode if tracking is enabled, or removing it if not.
 */
- (void)configureTrackingArea;

/**
 * Creates and registers keyboard related components.
 */
- (void)initializeKeyboard;

/**
 * Calls dispatchMouseEvent:phase: with a phase determined by self.mouseState.
 *
 * mouseState.buttons should be updated before calling this method.
 */
- (void)dispatchMouseEvent:(nonnull NSEvent*)event;

/**
 * Calls dispatchMouseEvent:phase: with a phase determined by event.phase.
 */
- (void)dispatchGestureEvent:(nonnull NSEvent*)event;

/**
 * Converts |event| to a FlutterPointerEvent with the given phase, and sends it to the engine.
 */
- (void)dispatchMouseEvent:(nonnull NSEvent*)event phase:(FlutterPointerPhase)phase;

/**
 * Called when the active keyboard input source changes.
 *
 * Input sources may be simple keyboard layouts, or more complex input methods involving an IME,
 * such as Chinese, Japanese, and Korean.
 */
- (void)onKeyboardLayoutChanged;

@end

#pragma mark - FlutterViewWrapper implementation.

/**
 * NotificationCenter callback invoked on kTISNotifySelectedKeyboardInputSourceChanged events.
 */
static void OnKeyboardLayoutChanged(CFNotificationCenterRef center,
                                    void* observer,
                                    CFStringRef name,
                                    const void* object,
                                    CFDictionaryRef userInfo) {
  FlutterViewController* controller = (__bridge FlutterViewController*)observer;
  if (controller != nil) {
    [controller onKeyboardLayoutChanged];
  }
}

@implementation FlutterViewWrapper {
  FlutterView* _flutterView;
  __weak FlutterViewController* _controller;
}

- (instancetype)initWithFlutterView:(FlutterView*)view
                         controller:(FlutterViewController*)controller {
  self = [super initWithFrame:NSZeroRect];
  if (self) {
    _flutterView = view;
    _controller = controller;
    view.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
    [self addSubview:view];
  }
  return self;
}

- (void)setBackgroundColor:(NSColor*)color {
  [_flutterView setBackgroundColor:color];
}

- (BOOL)performKeyEquivalent:(NSEvent*)event {
  // Do not intercept the event if flutterView is not first responder, otherwise this would
  // interfere with TextInputPlugin, which also handles key equivalents.
  //
  // Also do not intercept the event if key equivalent is a product of an event being
  // redispatched by the TextInputPlugin, in which case it needs to bubble up so that menus
  // can handle key equivalents.
  if (self.window.firstResponder != _flutterView || [_controller isDispatchingKeyEvent:event]) {
    return [super performKeyEquivalent:event];
  }
  [_flutterView keyDown:event];
  return YES;
}

- (NSArray*)accessibilityChildren {
  return @[ _flutterView ];
}

// TODO(cbracken): https://github.com/flutter/flutter/issues/154063
// Remove this whole method override when we drop support for macOS 12 (Monterey).
- (void)mouseDown:(NSEvent*)event {
  if (@available(macOS 13.3.1, *)) {
    [super mouseDown:event];
  } else {
    // Work around an AppKit bug where mouseDown/mouseUp are not called on the view controller if
    // the view is the content view of an NSPopover AND macOS's Reduced Transparency accessibility
    // setting is enabled.
    //
    // This simply calls mouseDown on the next responder in the responder chain as the default
    // implementation on NSResponder is documented to do.
    //
    // See: https://github.com/flutter/flutter/issues/115015
    // See: http://www.openradar.me/FB12050037
    // See: https://developer.apple.com/documentation/appkit/nsresponder/1524634-mousedown
    [self.nextResponder mouseDown:event];
  }
}

// TODO(cbracken): https://github.com/flutter/flutter/issues/154063
// Remove this workaround when we drop support for macOS 12 (Monterey).
- (void)mouseUp:(NSEvent*)event {
  if (@available(macOS 13.3.1, *)) {
    [super mouseUp:event];
  } else {
    // Work around an AppKit bug where mouseDown/mouseUp are not called on the view controller if
    // the view is the content view of an NSPopover AND macOS's Reduced Transparency accessibility
    // setting is enabled.
    //
    // This simply calls mouseUp on the next responder in the responder chain as the default
    // implementation on NSResponder is documented to do.
    //
    // See: https://github.com/flutter/flutter/issues/115015
    // See: http://www.openradar.me/FB12050037
    // See: https://developer.apple.com/documentation/appkit/nsresponder/1535349-mouseup
    [self.nextResponder mouseUp:event];
  }
}

@end

#pragma mark - FlutterViewController implementation.

@implementation FlutterViewController {
  // The project to run in this controller's engine.
  FlutterDartProject* _project;

  std::shared_ptr<flutter::AccessibilityBridgeMac> _bridge;

  // FlutterViewController does not actually uses the synchronizer, but only
  // passes it to FlutterView.
  FlutterThreadSynchronizer* _threadSynchronizer;
}

// Synthesize properties declared readonly.
@synthesize viewIdentifier = _viewIdentifier;

@dynamic accessibilityBridge;

/**
 * Performs initialization that's common between the different init paths.
 */
static void CommonInit(FlutterViewController* controller, FlutterEngine* engine) {
  if (!engine) {
    engine = [[FlutterEngine alloc] initWithName:@"io.flutter"
                                         project:controller->_project
                          allowHeadlessExecution:NO];
  }
  NSCAssert(controller.engine == nil,
            @"The FlutterViewController is unexpectedly attached to "
            @"engine %@ before initialization.",
            controller.engine);
  [engine addViewController:controller];
  NSCAssert(controller.engine != nil,
            @"The FlutterViewController unexpectedly stays unattached after initialization. "
            @"In unit tests, this is likely because either the FlutterViewController or "
            @"the FlutterEngine is mocked. Please subclass these classes instead.",
            controller.engine, controller.viewIdentifier);
  controller->_mouseTrackingMode = kFlutterMouseTrackingModeInKeyWindow;
  controller->_textInputPlugin = [[FlutterTextInputPlugin alloc] initWithViewController:controller];
  [controller initializeKeyboard];
  [controller notifySemanticsEnabledChanged];
  // macOS fires this message when changing IMEs.
  CFNotificationCenterRef cfCenter = CFNotificationCenterGetDistributedCenter();
  __weak FlutterViewController* weakSelf = controller;
  CFNotificationCenterAddObserver(cfCenter, (__bridge void*)weakSelf, OnKeyboardLayoutChanged,
                                  kTISNotifySelectedKeyboardInputSourceChanged, NULL,
                                  CFNotificationSuspensionBehaviorDeliverImmediately);
}

- (instancetype)initWithCoder:(NSCoder*)coder {
  self = [super initWithCoder:coder];
  NSAssert(self, @"Super init cannot be nil");

  CommonInit(self, nil);
  return self;
}

- (instancetype)initWithNibName:(NSString*)nibNameOrNil bundle:(NSBundle*)nibBundleOrNil {
  self = [super initWithNibName:nibNameOrNil bundle:nibBundleOrNil];
  NSAssert(self, @"Super init cannot be nil");

  CommonInit(self, nil);
  return self;
}

- (instancetype)initWithProject:(nullable FlutterDartProject*)project {
  self = [super initWithNibName:nil bundle:nil];
  NSAssert(self, @"Super init cannot be nil");

  _project = project;
  CommonInit(self, nil);
  return self;
}

- (instancetype)initWithEngine:(nonnull FlutterEngine*)engine
                       nibName:(nullable NSString*)nibName
                        bundle:(nullable NSBundle*)nibBundle {
  NSAssert(engine != nil, @"Engine is required");

  self = [super initWithNibName:nibName bundle:nibBundle];
  if (self) {
    CommonInit(self, engine);
  }

  return self;
}

- (BOOL)isDispatchingKeyEvent:(NSEvent*)event {
  return [_keyboardManager isDispatchingKeyEvent:event];
}

- (void)loadView {
  FlutterView* flutterView;
  id<MTLDevice> device = _engine.renderer.device;
  id<MTLCommandQueue> commandQueue = _engine.renderer.commandQueue;
  if (!device || !commandQueue) {
    NSLog(@"Unable to create FlutterView; no MTLDevice or MTLCommandQueue available.");
    return;
  }
  flutterView = [self createFlutterViewWithMTLDevice:device commandQueue:commandQueue];
  if (_backgroundColor != nil) {
    [flutterView setBackgroundColor:_backgroundColor];
  }
  FlutterViewWrapper* wrapperView = [[FlutterViewWrapper alloc] initWithFlutterView:flutterView
                                                                         controller:self];
  self.view = wrapperView;
  _flutterView = flutterView;
}

- (void)viewDidLoad {
  [self configureTrackingArea];
  [self.view setAllowedTouchTypes:NSTouchTypeMaskIndirect];
  [self.view setWantsRestingTouches:YES];
  [_engine viewControllerViewDidLoad:self];
}

- (void)viewWillAppear {
  [super viewWillAppear];
  if (!_engine.running) {
    [self launchEngine];
  }
  [self listenForMetaModifiedKeyUpEvents];
}

- (void)viewWillDisappear {
  // Per Apple's documentation, it is discouraged to call removeMonitor: in dealloc, and it's
  // recommended to be called earlier in the lifecycle.
  [NSEvent removeMonitor:_keyUpMonitor];
  _keyUpMonitor = nil;
}

- (void)dealloc {
  if ([self attached]) {
    [_engine removeViewController:self];
  }
  CFNotificationCenterRef cfCenter = CFNotificationCenterGetDistributedCenter();
  CFNotificationCenterRemoveEveryObserver(cfCenter, (__bridge void*)self);
}

#pragma mark - Public methods

- (void)setMouseTrackingMode:(FlutterMouseTrackingMode)mode {
  if (_mouseTrackingMode == mode) {
    return;
  }
  _mouseTrackingMode = mode;
  [self configureTrackingArea];
}

- (void)setBackgroundColor:(NSColor*)color {
  _backgroundColor = color;
  [_flutterView setBackgroundColor:_backgroundColor];
}

- (FlutterViewIdentifier)viewIdentifier {
  NSAssert([self attached], @"This view controller is not attached.");
  return _viewIdentifier;
}

- (void)onPreEngineRestart {
  [self initializeKeyboard];
}

- (void)notifySemanticsEnabledChanged {
  BOOL mySemanticsEnabled = !!_bridge;
  BOOL newSemanticsEnabled = _engine.semanticsEnabled;
  if (newSemanticsEnabled == mySemanticsEnabled) {
    return;
  }
  if (newSemanticsEnabled) {
    _bridge = [self createAccessibilityBridgeWithEngine:_engine];
  } else {
    // Remove the accessibility children from flutter view before resetting the bridge.
    _flutterView.accessibilityChildren = nil;
    _bridge.reset();
  }
  NSAssert(newSemanticsEnabled == !!_bridge, @"Failed to update semantics for the view.");
}

- (std::weak_ptr<flutter::AccessibilityBridgeMac>)accessibilityBridge {
  return _bridge;
}

- (void)setUpWithEngine:(FlutterEngine*)engine
         viewIdentifier:(FlutterViewIdentifier)viewIdentifier
     threadSynchronizer:(FlutterThreadSynchronizer*)threadSynchronizer {
  NSAssert(_engine == nil, @"Already attached to an engine %@.", _engine);
  _engine = engine;
  _viewIdentifier = viewIdentifier;
  _threadSynchronizer = threadSynchronizer;
  [_threadSynchronizer registerView:_viewIdentifier];
}

- (void)detachFromEngine {
  NSAssert(_engine != nil, @"Not attached to any engine.");
  [_threadSynchronizer deregisterView:_viewIdentifier];
  _threadSynchronizer = nil;
  _engine = nil;
}

- (BOOL)attached {
  return _engine != nil;
}

- (void)updateSemantics:(const FlutterSemanticsUpdate2*)update {
  // Semantics will be disabled when unfocusing application but the updateSemantics:
  // callback is received in next run loop turn.
  if (!_engine.semanticsEnabled) {
    return;
  }
  for (size_t i = 0; i < update->node_count; i++) {
    const FlutterSemanticsNode2* node = update->nodes[i];
    _bridge->AddFlutterSemanticsNodeUpdate(*node);
  }

  for (size_t i = 0; i < update->custom_action_count; i++) {
    const FlutterSemanticsCustomAction2* action = update->custom_actions[i];
    _bridge->AddFlutterSemanticsCustomActionUpdate(*action);
  }

  _bridge->CommitUpdates();

  // Accessibility tree can only be used when the view is loaded.
  if (!self.viewLoaded) {
    return;
  }
  // Attaches the accessibility root to the flutter view.
  auto root = _bridge->GetFlutterPlatformNodeDelegateFromID(0).lock();
  if (root) {
    if ([self.flutterView.accessibilityChildren count] == 0) {
      NSAccessibilityElement* native_root = root->GetNativeViewAccessible();
      self.flutterView.accessibilityChildren = @[ native_root ];
    }
  } else {
    self.flutterView.accessibilityChildren = nil;
  }
}

#pragma mark - Private methods

- (BOOL)launchEngine {
  if (![_engine runWithEntrypoint:nil]) {
    return NO;
  }
  return YES;
}

// macOS does not call keyUp: on a key while the command key is pressed. This results in a loss
// of a key event once the modified key is released. This method registers the
// ViewController as a listener for a keyUp event before it's handled by NSApplication, and should
// NOT modify the event to avoid any unexpected behavior.
- (void)listenForMetaModifiedKeyUpEvents {
  if (_keyUpMonitor != nil) {
    // It is possible for [NSViewController viewWillAppear] to be invoked multiple times
    // in a row. https://github.com/flutter/flutter/issues/105963
    return;
  }
  FlutterViewController* __weak weakSelf = self;
  _keyUpMonitor = [NSEvent
      addLocalMonitorForEventsMatchingMask:NSEventMaskKeyUp
                                   handler:^NSEvent*(NSEvent* event) {
                                     // Intercept keyUp only for events triggered on the current
                                     // view or textInputPlugin.
                                     NSResponder* firstResponder = [[event window] firstResponder];
                                     if (weakSelf.viewLoaded && weakSelf.flutterView &&
                                         (firstResponder == weakSelf.flutterView ||
                                          firstResponder == weakSelf.textInputPlugin) &&
                                         ([event modifierFlags] & NSEventModifierFlagCommand) &&
                                         ([event type] == NSEventTypeKeyUp)) {
                                       [weakSelf keyUp:event];
                                     }
                                     return event;
                                   }];
}

- (void)configureTrackingArea {
  if (!self.viewLoaded) {
    // The viewDidLoad will call configureTrackingArea again when
    // the view is actually loaded.
    return;
  }
  if (_mouseTrackingMode != kFlutterMouseTrackingModeNone && self.flutterView) {
    NSTrackingAreaOptions options = NSTrackingMouseEnteredAndExited | NSTrackingMouseMoved |
                                    NSTrackingInVisibleRect | NSTrackingEnabledDuringMouseDrag;
    switch (_mouseTrackingMode) {
      case kFlutterMouseTrackingModeInKeyWindow:
        options |= NSTrackingActiveInKeyWindow;
        break;
      case kFlutterMouseTrackingModeInActiveApp:
        options |= NSTrackingActiveInActiveApp;
        break;
      case kFlutterMouseTrackingModeAlways:
        options |= NSTrackingActiveAlways;
        break;
      default:
        NSLog(@"Error: Unrecognized mouse tracking mode: %ld", _mouseTrackingMode);
        return;
    }
    _trackingArea = [[NSTrackingArea alloc] initWithRect:NSZeroRect
                                                 options:options
                                                   owner:self
                                                userInfo:nil];
    [self.flutterView addTrackingArea:_trackingArea];
  } else if (_trackingArea) {
    [self.flutterView removeTrackingArea:_trackingArea];
    _trackingArea = nil;
  }
}

- (void)initializeKeyboard {
  // TODO(goderbauer): Seperate keyboard/textinput stuff into ViewController specific and Engine
  // global parts. Move the global parts to FlutterEngine.
  _keyboardManager = [[FlutterKeyboardManager alloc] initWithViewDelegate:self];
}

- (void)dispatchMouseEvent:(nonnull NSEvent*)event {
  FlutterPointerPhase phase = _mouseState.buttons == 0
                                  ? (_mouseState.flutter_state_is_down ? kUp : kHover)
                                  : (_mouseState.flutter_state_is_down ? kMove : kDown);
  [self dispatchMouseEvent:event phase:phase];
}

- (void)dispatchGestureEvent:(nonnull NSEvent*)event {
  if (event.phase == NSEventPhaseBegan || event.phase == NSEventPhaseMayBegin) {
    [self dispatchMouseEvent:event phase:kPanZoomStart];
  } else if (event.phase == NSEventPhaseChanged) {
    [self dispatchMouseEvent:event phase:kPanZoomUpdate];
  } else if (event.phase == NSEventPhaseEnded || event.phase == NSEventPhaseCancelled) {
    [self dispatchMouseEvent:event phase:kPanZoomEnd];
  } else if (event.phase == NSEventPhaseNone && event.momentumPhase == NSEventPhaseNone) {
    [self dispatchMouseEvent:event phase:kHover];
  } else {
    // Waiting until the first momentum change event is a workaround for an issue where
    // touchesBegan: is called unexpectedly while in low power mode within the interval between
    // momentum start and the first momentum change.
    if (event.momentumPhase == NSEventPhaseChanged) {
      _mouseState.last_scroll_momentum_changed_time = event.timestamp;
    }
    // Skip momentum update events, the framework will generate scroll momentum.
    NSAssert(event.momentumPhase != NSEventPhaseNone,
             @"Received gesture event with unexpected phase");
  }
}

- (void)dispatchMouseEvent:(NSEvent*)event phase:(FlutterPointerPhase)phase {
  NSAssert(self.viewLoaded, @"View must be loaded before it handles the mouse event");
  // There are edge cases where the system will deliver enter out of order relative to other
  // events (e.g., drag out and back in, release, then click; mouseDown: will be called before
  // mouseEntered:). Discard those events, since the add will already have been synthesized.
  if (_mouseState.flutter_state_is_added && phase == kAdd) {
    return;
  }

  // Multiple gesture recognizers could be active at once, we can't send multiple kPanZoomStart.
  // For example: rotation and magnification.
  if (phase == kPanZoomStart || phase == kPanZoomEnd) {
    if (event.type == NSEventTypeScrollWheel) {
      _mouseState.pan_gesture_phase = event.phase;
    } else if (event.type == NSEventTypeMagnify) {
      _mouseState.scale_gesture_phase = event.phase;
    } else if (event.type == NSEventTypeRotate) {
      _mouseState.rotate_gesture_phase = event.phase;
    }
  }
  if (phase == kPanZoomStart) {
    if (event.type == NSEventTypeScrollWheel) {
      // Ensure scroll inertia cancel event is not sent afterwards.
      _mouseState.last_scroll_momentum_changed_time = 0;
    }
    if (_mouseState.flutter_state_is_pan_zoom_started) {
      // Already started on a previous gesture type
      return;
    }
    _mouseState.flutter_state_is_pan_zoom_started = true;
  }
  if (phase == kPanZoomEnd) {
    if (!_mouseState.flutter_state_is_pan_zoom_started) {
      // NSEventPhaseCancelled is sometimes received at incorrect times in the state
      // machine, just ignore it here if it doesn't make sense
      // (we have no active gesture to cancel).
      NSAssert(event.phase == NSEventPhaseCancelled,
               @"Received gesture event with unexpected phase");
      return;
    }
    // NSEventPhase values are powers of two, we can use this to inspect merged phases.
    NSEventPhase all_gestures_fields = _mouseState.pan_gesture_phase |
                                       _mouseState.scale_gesture_phase |
                                       _mouseState.rotate_gesture_phase;
    NSEventPhase active_mask = NSEventPhaseBegan | NSEventPhaseChanged;
    if ((all_gestures_fields & active_mask) != 0) {
      // Even though this gesture type ended, a different type is still active.
      return;
    }
  }

  // If a pointer added event hasn't been sent, synthesize one using this event for the basic
  // information.
  if (!_mouseState.flutter_state_is_added && phase != kAdd) {
    // Only the values extracted for use in flutterEvent below matter, the rest are dummy values.
    NSEvent* addEvent = [NSEvent enterExitEventWithType:NSEventTypeMouseEntered
                                               location:event.locationInWindow
                                          modifierFlags:0
                                              timestamp:event.timestamp
                                           windowNumber:event.windowNumber
                                                context:nil
                                            eventNumber:0
                                         trackingNumber:0
                                               userData:NULL];
    [self dispatchMouseEvent:addEvent phase:kAdd];
  }

  NSPoint locationInView = [self.flutterView convertPoint:event.locationInWindow fromView:nil];
  NSPoint locationInBackingCoordinates = [self.flutterView convertPointToBacking:locationInView];
  int32_t device = kMousePointerDeviceId;
  FlutterPointerDeviceKind deviceKind = kFlutterPointerDeviceKindMouse;
  if (phase == kPanZoomStart || phase == kPanZoomUpdate || phase == kPanZoomEnd) {
    device = kPointerPanZoomDeviceId;
    deviceKind = kFlutterPointerDeviceKindTrackpad;
  }
  FlutterPointerEvent flutterEvent = {
      .struct_size = sizeof(flutterEvent),
      .phase = phase,
      .timestamp = static_cast<size_t>(event.timestamp * USEC_PER_SEC),
      .x = locationInBackingCoordinates.x,
      .y = -locationInBackingCoordinates.y,  // convertPointToBacking makes this negative.
      .device = device,
      .device_kind = deviceKind,
      // If a click triggered a synthesized kAdd, don't pass the buttons in that event.
      .buttons = phase == kAdd ? 0 : _mouseState.buttons,
      .view_id = static_cast<FlutterViewIdentifier>(_viewIdentifier),
  };

  if (phase == kPanZoomUpdate) {
    if (event.type == NSEventTypeScrollWheel) {
      _mouseState.delta_x += event.scrollingDeltaX * self.flutterView.layer.contentsScale;
      _mouseState.delta_y += event.scrollingDeltaY * self.flutterView.layer.contentsScale;
    } else if (event.type == NSEventTypeMagnify) {
      _mouseState.scale += event.magnification;
    } else if (event.type == NSEventTypeRotate) {
      _mouseState.rotation += event.rotation * (-M_PI / 180.0);
    }
    flutterEvent.pan_x = _mouseState.delta_x;
    flutterEvent.pan_y = _mouseState.delta_y;
    // Scale value needs to be normalized to range 0->infinity.
    flutterEvent.scale = pow(2.0, _mouseState.scale);
    flutterEvent.rotation = _mouseState.rotation;
  } else if (phase == kPanZoomEnd) {
    _mouseState.GestureReset();
  } else if (phase != kPanZoomStart && event.type == NSEventTypeScrollWheel) {
    flutterEvent.signal_kind = kFlutterPointerSignalKindScroll;

    double pixelsPerLine = 1.0;
    if (!event.hasPreciseScrollingDeltas) {
      // The scrollingDelta needs to be multiplied by the line height.
      // CGEventSourceGetPixelsPerLine() will return 10, which will result in
      // scrolling that is noticeably slower than in other applications.
      // Using 40.0 as the multiplier to match Chromium.
      // See https://source.chromium.org/chromium/chromium/src/+/main:ui/events/cocoa/events_mac.mm
      pixelsPerLine = 40.0;
    }
    double scaleFactor = self.flutterView.layer.contentsScale;
    // When mouse input is received while shift is pressed (regardless of
    // any other pressed keys), Mac automatically flips the axis. Other
    // platforms do not do this, so we flip it back to normalize the input
    // received by the framework. The keyboard+mouse-scroll mechanism is exposed
    // in the ScrollBehavior of the framework so developers can customize the
    // behavior.
    // At time of change, Apple does not expose any other type of API or signal
    // that the X/Y axes have been flipped.
    double scaledDeltaX = -event.scrollingDeltaX * pixelsPerLine * scaleFactor;
    double scaledDeltaY = -event.scrollingDeltaY * pixelsPerLine * scaleFactor;
    if (event.modifierFlags & NSShiftKeyMask) {
      flutterEvent.scroll_delta_x = scaledDeltaY;
      flutterEvent.scroll_delta_y = scaledDeltaX;
    } else {
      flutterEvent.scroll_delta_x = scaledDeltaX;
      flutterEvent.scroll_delta_y = scaledDeltaY;
    }
  }

  [_keyboardManager syncModifiersIfNeeded:event.modifierFlags timestamp:event.timestamp];
  [_engine sendPointerEvent:flutterEvent];

  // Update tracking of state as reported to Flutter.
  if (phase == kDown) {
    _mouseState.flutter_state_is_down = true;
  } else if (phase == kUp) {
    _mouseState.flutter_state_is_down = false;
    if (_mouseState.has_pending_exit) {
      [self dispatchMouseEvent:event phase:kRemove];
      _mouseState.has_pending_exit = false;
    }
  } else if (phase == kAdd) {
    _mouseState.flutter_state_is_added = true;
  } else if (phase == kRemove) {
    _mouseState.Reset();
  }
}

- (void)onAccessibilityStatusChanged:(BOOL)enabled {
  if (!enabled && self.viewLoaded && [_textInputPlugin isFirstResponder]) {
    // Normally TextInputPlugin, when editing, is child of FlutterViewWrapper.
    // When accessiblity is enabled the TextInputPlugin gets added as an indirect
    // child to FlutterTextField. When disabling the plugin needs to be reparented
    // back.
    [self.view addSubview:_textInputPlugin];
  }
}

- (std::shared_ptr<flutter::AccessibilityBridgeMac>)createAccessibilityBridgeWithEngine:
    (nonnull FlutterEngine*)engine {
  return std::make_shared<flutter::AccessibilityBridgeMac>(engine, self);
}

- (nonnull FlutterView*)createFlutterViewWithMTLDevice:(id<MTLDevice>)device
                                          commandQueue:(id<MTLCommandQueue>)commandQueue {
  return [[FlutterView alloc] initWithMTLDevice:device
                                   commandQueue:commandQueue
                                       delegate:self
                             threadSynchronizer:_threadSynchronizer
                                 viewIdentifier:_viewIdentifier];
}

- (void)onKeyboardLayoutChanged {
  _keyboardLayoutData = nil;
  if (_keyboardLayoutNotifier != nil) {
    _keyboardLayoutNotifier();
  }
}

- (NSString*)lookupKeyForAsset:(NSString*)asset {
  return [FlutterDartProject lookupKeyForAsset:asset];
}

- (NSString*)lookupKeyForAsset:(NSString*)asset fromPackage:(NSString*)package {
  return [FlutterDartProject lookupKeyForAsset:asset fromPackage:package];
}

#pragma mark - FlutterViewDelegate

/**
 * Responds to view reshape by notifying the engine of the change in dimensions.
 */
- (void)viewDidReshape:(NSView*)view {
  FML_DCHECK(view == _flutterView);
  [_engine updateWindowMetricsForViewController:self];
}

- (BOOL)viewShouldAcceptFirstResponder:(NSView*)view {
  FML_DCHECK(view == _flutterView);
  // Only allow FlutterView to become first responder if TextInputPlugin is
  // not active. Otherwise a mouse event inside FlutterView would cause the
  // TextInputPlugin to lose first responder status.
  return !_textInputPlugin.isFirstResponder;
}

#pragma mark - FlutterPluginRegistry

- (id<FlutterPluginRegistrar>)registrarForPlugin:(NSString*)pluginName {
  return [_engine registrarForPlugin:pluginName];
}

- (NSObject*)valuePublishedByPlugin:(NSString*)pluginKey {
  return [_engine valuePublishedByPlugin:pluginKey];
}

#pragma mark - FlutterKeyboardViewDelegate

/**
 * Returns the current Unicode layout data (kTISPropertyUnicodeKeyLayoutData).
 *
 * To use the returned data, convert it to CFDataRef first, finds its bytes
 * with CFDataGetBytePtr, then reinterpret it into const UCKeyboardLayout*.
 * It's returned in NSData* to enable auto reference count.
 */
static NSData* CurrentKeyboardLayoutData() {
  fml::CFRef<TISInputSourceRef> source(TISCopyCurrentKeyboardInputSource());
  CFTypeRef layout_data = TISGetInputSourceProperty(source, kTISPropertyUnicodeKeyLayoutData);
  if (layout_data == nil) {
    // TISGetInputSourceProperty returns null with Japanese keyboard layout.
    // Using TISCopyCurrentKeyboardLayoutInputSource to fix NULL return.
    // https://github.com/microsoft/node-native-keymap/blob/5f0699ded00179410a14c0e1b0e089fe4df8e130/src/keyboard_mac.mm#L91
    source.Reset(TISCopyCurrentKeyboardLayoutInputSource());
    layout_data = TISGetInputSourceProperty(source, kTISPropertyUnicodeKeyLayoutData);
  }
  return (__bridge NSData*)layout_data;
}

- (void)sendKeyEvent:(const FlutterKeyEvent&)event
            callback:(nullable FlutterKeyEventCallback)callback
            userData:(nullable void*)userData {
  [_engine sendKeyEvent:event callback:callback userData:userData];
}

- (id<FlutterBinaryMessenger>)getBinaryMessenger {
  return _engine.binaryMessenger;
}

- (BOOL)onTextInputKeyEvent:(nonnull NSEvent*)event {
  return [_textInputPlugin handleKeyEvent:event];
}

- (void)subscribeToKeyboardLayoutChange:(nullable KeyboardLayoutNotifier)callback {
  _keyboardLayoutNotifier = callback;
}

- (LayoutClue)lookUpLayoutForKeyCode:(uint16_t)keyCode shift:(BOOL)shift {
  if (_keyboardLayoutData == nil) {
    _keyboardLayoutData = CurrentKeyboardLayoutData();
  }
  const UCKeyboardLayout* layout = reinterpret_cast<const UCKeyboardLayout*>(
      CFDataGetBytePtr((__bridge CFDataRef)_keyboardLayoutData));

  UInt32 deadKeyState = 0;
  UniCharCount stringLength = 0;
  UniChar resultChar;

  UInt32 modifierState = ((shift ? shiftKey : 0) >> 8) & 0xFF;
  UInt32 keyboardType = LMGetKbdLast();

  bool isDeadKey = false;
  OSStatus status =
      UCKeyTranslate(layout, keyCode, kUCKeyActionDown, modifierState, keyboardType,
                     kUCKeyTranslateNoDeadKeysBit, &deadKeyState, 1, &stringLength, &resultChar);
  // For dead keys, press the same key again to get the printable representation of the key.
  if (status == noErr && stringLength == 0 && deadKeyState != 0) {
    isDeadKey = true;
    status =
        UCKeyTranslate(layout, keyCode, kUCKeyActionDown, modifierState, keyboardType,
                       kUCKeyTranslateNoDeadKeysBit, &deadKeyState, 1, &stringLength, &resultChar);
  }

  if (status == noErr && stringLength == 1 && !std::iscntrl(resultChar)) {
    return LayoutClue{resultChar, isDeadKey};
  }
  return LayoutClue{0, false};
}

- (nonnull NSDictionary*)getPressedState {
  return [_keyboardManager getPressedState];
}

#pragma mark - NSResponder

- (BOOL)acceptsFirstResponder {
  return YES;
}

- (void)keyDown:(NSEvent*)event {
  [_keyboardManager handleEvent:event];
}

- (void)keyUp:(NSEvent*)event {
  [_keyboardManager handleEvent:event];
}

- (void)flagsChanged:(NSEvent*)event {
  [_keyboardManager handleEvent:event];
}

- (void)mouseEntered:(NSEvent*)event {
  if (_mouseState.has_pending_exit) {
    _mouseState.has_pending_exit = false;
  } else {
    [self dispatchMouseEvent:event phase:kAdd];
  }
}

- (void)mouseExited:(NSEvent*)event {
  if (_mouseState.buttons != 0) {
    _mouseState.has_pending_exit = true;
    return;
  }
  [self dispatchMouseEvent:event phase:kRemove];
}

- (void)mouseDown:(NSEvent*)event {
  _mouseState.buttons |= kFlutterPointerButtonMousePrimary;
  [self dispatchMouseEvent:event];
}

- (void)mouseUp:(NSEvent*)event {
  _mouseState.buttons &= ~static_cast<uint64_t>(kFlutterPointerButtonMousePrimary);
  [self dispatchMouseEvent:event];
}

- (void)mouseDragged:(NSEvent*)event {
  [self dispatchMouseEvent:event];
}

- (void)rightMouseDown:(NSEvent*)event {
  _mouseState.buttons |= kFlutterPointerButtonMouseSecondary;
  [self dispatchMouseEvent:event];
}

- (void)rightMouseUp:(NSEvent*)event {
  _mouseState.buttons &= ~static_cast<uint64_t>(kFlutterPointerButtonMouseSecondary);
  [self dispatchMouseEvent:event];
}

- (void)rightMouseDragged:(NSEvent*)event {
  [self dispatchMouseEvent:event];
}

- (void)otherMouseDown:(NSEvent*)event {
  _mouseState.buttons |= (1 << event.buttonNumber);
  [self dispatchMouseEvent:event];
}

- (void)otherMouseUp:(NSEvent*)event {
  _mouseState.buttons &= ~static_cast<uint64_t>(1 << event.buttonNumber);
  [self dispatchMouseEvent:event];
}

- (void)otherMouseDragged:(NSEvent*)event {
  [self dispatchMouseEvent:event];
}

- (void)mouseMoved:(NSEvent*)event {
  [self dispatchMouseEvent:event];
}

- (void)scrollWheel:(NSEvent*)event {
  [self dispatchGestureEvent:event];
}

- (void)magnifyWithEvent:(NSEvent*)event {
  [self dispatchGestureEvent:event];
}

- (void)rotateWithEvent:(NSEvent*)event {
  [self dispatchGestureEvent:event];
}

- (void)swipeWithEvent:(NSEvent*)event {
  // Not needed, it's handled by scrollWheel.
}

- (void)touchesBeganWithEvent:(NSEvent*)event {
  NSTouch* touch = event.allTouches.anyObject;
  if (touch != nil) {
    if ((event.timestamp - _mouseState.last_scroll_momentum_changed_time) <
        kTrackpadTouchInertiaCancelWindowMs) {
      // The trackpad has been touched following a scroll momentum event.
      // A scroll inertia cancel message should be sent to the framework.
      NSPoint locationInView = [self.flutterView convertPoint:event.locationInWindow fromView:nil];
      NSPoint locationInBackingCoordinates =
          [self.flutterView convertPointToBacking:locationInView];
      FlutterPointerEvent flutterEvent = {
          .struct_size = sizeof(flutterEvent),
          .timestamp = static_cast<size_t>(event.timestamp * USEC_PER_SEC),
          .x = locationInBackingCoordinates.x,
          .y = -locationInBackingCoordinates.y,  // convertPointToBacking makes this negative.
          .device = kPointerPanZoomDeviceId,
          .signal_kind = kFlutterPointerSignalKindScrollInertiaCancel,
          .device_kind = kFlutterPointerDeviceKindTrackpad,
          .view_id = static_cast<FlutterViewIdentifier>(_viewIdentifier),
      };

      [_engine sendPointerEvent:flutterEvent];
      // Ensure no further scroll inertia cancel event will be sent.
      _mouseState.last_scroll_momentum_changed_time = 0;
    }
  }
}

@end
