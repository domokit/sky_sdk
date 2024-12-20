// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewController_Internal.h"

#import <os/log.h>
#include <memory>

#include "flutter/common/constants.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/platform/darwin/platform_version.h"
#include "flutter/runtime/ptrace_check.h"
#include "flutter/shell/common/thread_host.h"
#import "flutter/shell/platform/darwin/common/framework/Source/FlutterBinaryMessengerRelay.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterChannelKeyResponder.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEmbedderKeyResponder.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterKeyPrimaryResponder.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterKeyboardManager.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformViews_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputDelegate.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterView.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/UIViewController+FlutterScreenAndSceneIfLoaded.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/platform_message_response_darwin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/vsync_waiter_ios.h"
#import "flutter/shell/platform/darwin/ios/platform_view_ios.h"
#import "flutter/shell/platform/embedder/embedder.h"
#import "flutter/third_party/spring_animation/spring_animation.h"

FLUTTER_ASSERT_ARC

static constexpr int kMicrosecondsPerSecond = 1000 * 1000;
static constexpr CGFloat kScrollViewContentSize = 2.0;

static NSString* const kFlutterRestorationStateAppData = @"FlutterRestorationStateAppData";

NSNotificationName const FlutterSemanticsUpdateNotification = @"FlutterSemanticsUpdate";
NSNotificationName const FlutterViewControllerWillDealloc = @"FlutterViewControllerWillDealloc";
NSNotificationName const FlutterViewControllerHideHomeIndicator =
    @"FlutterViewControllerHideHomeIndicator";
NSNotificationName const FlutterViewControllerShowHomeIndicator =
    @"FlutterViewControllerShowHomeIndicator";

// Struct holding data to help adapt system mouse/trackpad events to embedder events.
typedef struct MouseState {
  // Current coordinate of the mouse cursor in physical device pixels.
  CGPoint location = CGPointZero;

  // Last reported translation for an in-flight pan gesture in physical device pixels.
  CGPoint last_translation = CGPointZero;
} MouseState;

// This is left a FlutterBinaryMessenger privately for now to give people a chance to notice the
// change. Unfortunately unless you have Werror turned on, incompatible pointers as arguments are
// just a warning.
@interface FlutterViewController () <FlutterBinaryMessenger, UIScrollViewDelegate>
// TODO(dkwingsmt): Make the view ID property public once the iOS shell
// supports multiple views.
// https://github.com/flutter/flutter/issues/138168
@property(nonatomic, readonly) int64_t viewIdentifier;

// We keep a separate reference to this and create it ahead of time because we want to be able to
// set up a shell along with its platform view before the view has to appear.
@property(nonatomic, strong) FlutterView* flutterView;
@property(nonatomic, strong) void (^flutterViewRenderedCallback)(void);

@property(nonatomic, assign) UIInterfaceOrientationMask orientationPreferences;
@property(nonatomic, assign) UIStatusBarStyle statusBarStyle;
@property(nonatomic, assign) BOOL initialized;
@property(nonatomic, assign) BOOL engineNeedsLaunch;

@property(nonatomic, readwrite, getter=isDisplayingFlutterUI) BOOL displayingFlutterUI;
@property(nonatomic, assign) BOOL isHomeIndicatorHidden;
@property(nonatomic, assign) BOOL isPresentingViewControllerAnimating;

// Internal state backing override of UIView.prefersStatusBarHidden.
@property(nonatomic, assign) BOOL flutterPrefersStatusBarHidden;

@property(nonatomic, strong) NSMutableSet<NSNumber*>* ongoingTouches;
// This scroll view is a workaround to accommodate iOS 13 and higher.  There isn't a way to get
// touches on the status bar to trigger scrolling to the top of a scroll view.  We place a
// UIScrollView with height zero and a content offset so we can get those events. See also:
// https://github.com/flutter/flutter/issues/35050
@property(nonatomic, strong) UIScrollView* scrollView;
@property(nonatomic, strong) UIView* keyboardAnimationView;
@property(nonatomic, strong) SpringAnimation* keyboardSpringAnimation;

/**
 * Whether we should ignore viewport metrics updates during rotation transition.
 */
@property(nonatomic, assign) BOOL shouldIgnoreViewportMetricsUpdatesDuringRotation;

/**
 * Keyboard animation properties
 */
@property(nonatomic, assign) CGFloat targetViewInsetBottom;
@property(nonatomic, assign) CGFloat originalViewInsetBottom;
@property(nonatomic, strong) VSyncClient* keyboardAnimationVSyncClient;
@property(nonatomic, assign) BOOL keyboardAnimationIsShowing;
@property(nonatomic, assign) fml::TimePoint keyboardAnimationStartTime;
@property(nonatomic, assign) BOOL isKeyboardInOrTransitioningFromBackground;

/// Timestamp after which a scroll inertia cancel event should be inferred.
@property(nonatomic, assign) NSTimeInterval scrollInertiaEventStartline;

/// When an iOS app is running in emulation on an Apple Silicon Mac, trackpad input goes through
/// a translation layer, and events are not received with precise deltas. Due to this, we can't
/// rely on checking for a stationary trackpad event. Fortunately, AppKit will send an event of
/// type UIEventTypeScroll following a scroll when inertia should stop. This field is needed to
/// estimate if such an event represents the natural end of scrolling inertia or a user-initiated
/// cancellation.
@property(nonatomic, assign) NSTimeInterval scrollInertiaEventAppKitDeadline;

/// VSyncClient for touch events delivery frame rate correction.
///
/// On promotion devices(eg: iPhone13 Pro), the delivery frame rate of touch events is 60HZ
/// but the frame rate of rendering is 120HZ, which is different and will leads jitter and laggy.
/// With this VSyncClient, it can correct the delivery frame rate of touch events to let it keep
/// the same with frame rate of rendering.
@property(nonatomic, strong) VSyncClient* touchRateCorrectionVSyncClient;

/*
 * Mouse and trackpad gesture recognizers
 */
// Mouse and trackpad hover
@property(nonatomic, strong)
    UIHoverGestureRecognizer* hoverGestureRecognizer API_AVAILABLE(ios(13.4));
// Mouse wheel scrolling
@property(nonatomic, strong)
    UIPanGestureRecognizer* discreteScrollingPanGestureRecognizer API_AVAILABLE(ios(13.4));
// Trackpad and Magic Mouse scrolling
@property(nonatomic, strong)
    UIPanGestureRecognizer* continuousScrollingPanGestureRecognizer API_AVAILABLE(ios(13.4));
// Trackpad pinching
@property(nonatomic, strong)
    UIPinchGestureRecognizer* pinchGestureRecognizer API_AVAILABLE(ios(13.4));
// Trackpad rotating
@property(nonatomic, strong)
    UIRotationGestureRecognizer* rotationGestureRecognizer API_AVAILABLE(ios(13.4));

/// Creates and registers plugins used by this view controller.
- (void)addInternalPlugins;
- (void)deregisterNotifications;

/// Called when the first frame has been rendered. Invokes any registered first-frame callback.
- (void)onFirstFrameRendered;

/// Handles updating viewport metrics on keyboard animation.
- (void)handleKeyboardAnimationCallbackWithTargetTime:(fml::TimePoint)targetTime;
@end

@implementation FlutterViewController {
  flutter::ViewportMetrics _viewportMetrics;
  MouseState _mouseState;
}

// Synthesize properties with an overridden getter/setter.
@synthesize viewOpaque = _viewOpaque;
@synthesize displayingFlutterUI = _displayingFlutterUI;

// TODO(dkwingsmt): https://github.com/flutter/flutter/issues/138168
// No backing ivar is currently required; when multiple views are supported, we'll need to
// synthesize the ivar and store the view identifier.
@dynamic viewIdentifier;

#pragma mark - Manage and override all designated initializers

- (instancetype)initWithEngine:(FlutterEngine*)engine
                       nibName:(nullable NSString*)nibName
                        bundle:(nullable NSBundle*)nibBundle {
  NSAssert(engine != nil, @"Engine is required");
  self = [super initWithNibName:nibName bundle:nibBundle];
  if (self) {
    _viewOpaque = YES;
    if (engine.viewController) {
      FML_LOG(ERROR) << "The supplied FlutterEngine " << [[engine description] UTF8String]
                     << " is already used with FlutterViewController instance "
                     << [[engine.viewController description] UTF8String]
                     << ". One instance of the FlutterEngine can only be attached to one "
                        "FlutterViewController at a time. Set FlutterEngine.viewController "
                        "to nil before attaching it to another FlutterViewController.";
    }
    _engine = engine;
    _engineNeedsLaunch = NO;
    _flutterView = [[FlutterView alloc] initWithDelegate:_engine
                                                  opaque:self.isViewOpaque
                                         enableWideGamut:engine.project.isWideGamutEnabled];
    _ongoingTouches = [[NSMutableSet alloc] init];

    // TODO(cbracken): https://github.com/flutter/flutter/issues/157140
    // Eliminate method calls in initializers and dealloc.
    [self performCommonViewControllerInitialization];
    [engine setViewController:self];
  }

  return self;
}

- (instancetype)initWithProject:(FlutterDartProject*)project
                        nibName:(NSString*)nibName
                         bundle:(NSBundle*)nibBundle {
  self = [super initWithNibName:nibName bundle:nibBundle];
  if (self) {
    // TODO(cbracken): https://github.com/flutter/flutter/issues/157140
    // Eliminate method calls in initializers and dealloc.
    [self sharedSetupWithProject:project initialRoute:nil];
  }

  return self;
}

- (instancetype)initWithProject:(FlutterDartProject*)project
                   initialRoute:(NSString*)initialRoute
                        nibName:(NSString*)nibName
                         bundle:(NSBundle*)nibBundle {
  self = [super initWithNibName:nibName bundle:nibBundle];
  if (self) {
    // TODO(cbracken): https://github.com/flutter/flutter/issues/157140
    // Eliminate method calls in initializers and dealloc.
    [self sharedSetupWithProject:project initialRoute:initialRoute];
  }

  return self;
}

- (instancetype)initWithNibName:(NSString*)nibNameOrNil bundle:(NSBundle*)nibBundleOrNil {
  return [self initWithProject:nil nibName:nil bundle:nil];
}

- (instancetype)initWithCoder:(NSCoder*)aDecoder {
  self = [super initWithCoder:aDecoder];
  return self;
}

- (void)awakeFromNib {
  [super awakeFromNib];
  if (!self.engine) {
    [self sharedSetupWithProject:nil initialRoute:nil];
  }
}

- (instancetype)init {
  return [self initWithProject:nil nibName:nil bundle:nil];
}

- (void)sharedSetupWithProject:(nullable FlutterDartProject*)project
                  initialRoute:(nullable NSString*)initialRoute {
  // Need the project to get settings for the view. Initializing it here means
  // the Engine class won't initialize it later.
  if (!project) {
    project = [[FlutterDartProject alloc] init];
  }
  FlutterView.forceSoftwareRendering = project.settings.enable_software_rendering;
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"io.flutter"
                                                      project:project
                                       allowHeadlessExecution:self.engineAllowHeadlessExecution
                                           restorationEnabled:self.restorationIdentifier != nil];
  if (!engine) {
    return;
  }

  _viewOpaque = YES;
  _engine = engine;
  _flutterView = [[FlutterView alloc] initWithDelegate:_engine
                                                opaque:_viewOpaque
                                       enableWideGamut:project.isWideGamutEnabled];
  [_engine createShell:nil libraryURI:nil initialRoute:initialRoute];
  _engineNeedsLaunch = YES;
  _ongoingTouches = [[NSMutableSet alloc] init];

  // TODO(cbracken): https://github.com/flutter/flutter/issues/157140
  // Eliminate method calls in initializers and dealloc.
  [self loadDefaultSplashScreenView];
  [self performCommonViewControllerInitialization];
}

- (BOOL)isViewOpaque {
  return _viewOpaque;
}

- (void)setViewOpaque:(BOOL)value {
  _viewOpaque = value;
  if (self.flutterView.layer.opaque != value) {
    self.flutterView.layer.opaque = value;
    [self.flutterView.layer setNeedsLayout];
  }
}

#pragma mark - Common view controller initialization tasks

- (void)performCommonViewControllerInitialization {
  if (_initialized) {
    return;
  }

  _initialized = YES;
  _orientationPreferences = UIInterfaceOrientationMaskAll;
  _statusBarStyle = UIStatusBarStyleDefault;

  // TODO(cbracken): https://github.com/flutter/flutter/issues/157140
  // Eliminate method calls in initializers and dealloc.
  [self setUpNotificationCenterObservers];
}

- (void)setUpNotificationCenterObservers {
  NSNotificationCenter* center = [NSNotificationCenter defaultCenter];
  [center addObserver:self
             selector:@selector(onOrientationPreferencesUpdated:)
                 name:@(flutter::kOrientationUpdateNotificationName)
               object:nil];

  [center addObserver:self
             selector:@selector(onPreferredStatusBarStyleUpdated:)
                 name:@(flutter::kOverlayStyleUpdateNotificationName)
               object:nil];

#if APPLICATION_EXTENSION_API_ONLY
  if (@available(iOS 13.0, *)) {
    [self setUpSceneLifecycleNotifications:center];
  } else {
    [self setUpApplicationLifecycleNotifications:center];
  }
#else
  [self setUpApplicationLifecycleNotifications:center];
#endif

  [center addObserver:self
             selector:@selector(keyboardWillChangeFrame:)
                 name:UIKeyboardWillChangeFrameNotification
               object:nil];

  [center addObserver:self
             selector:@selector(keyboardWillShowNotification:)
                 name:UIKeyboardWillShowNotification
               object:nil];

  [center addObserver:self
             selector:@selector(keyboardWillBeHidden:)
                 name:UIKeyboardWillHideNotification
               object:nil];

  [center addObserver:self
             selector:@selector(onAccessibilityStatusChanged:)
                 name:UIAccessibilityVoiceOverStatusDidChangeNotification
               object:nil];

  [center addObserver:self
             selector:@selector(onAccessibilityStatusChanged:)
                 name:UIAccessibilitySwitchControlStatusDidChangeNotification
               object:nil];

  [center addObserver:self
             selector:@selector(onAccessibilityStatusChanged:)
                 name:UIAccessibilitySpeakScreenStatusDidChangeNotification
               object:nil];

  [center addObserver:self
             selector:@selector(onAccessibilityStatusChanged:)
                 name:UIAccessibilityInvertColorsStatusDidChangeNotification
               object:nil];

  [center addObserver:self
             selector:@selector(onAccessibilityStatusChanged:)
                 name:UIAccessibilityReduceMotionStatusDidChangeNotification
               object:nil];

  [center addObserver:self
             selector:@selector(onAccessibilityStatusChanged:)
                 name:UIAccessibilityBoldTextStatusDidChangeNotification
               object:nil];

  [center addObserver:self
             selector:@selector(onAccessibilityStatusChanged:)
                 name:UIAccessibilityDarkerSystemColorsStatusDidChangeNotification
               object:nil];

  if (@available(iOS 13.0, *)) {
    [center addObserver:self
               selector:@selector(onAccessibilityStatusChanged:)
                   name:UIAccessibilityOnOffSwitchLabelsDidChangeNotification
                 object:nil];
  }

  [center addObserver:self
             selector:@selector(onUserSettingsChanged:)
                 name:UIContentSizeCategoryDidChangeNotification
               object:nil];

  [center addObserver:self
             selector:@selector(onHideHomeIndicatorNotification:)
                 name:FlutterViewControllerHideHomeIndicator
               object:nil];

  [center addObserver:self
             selector:@selector(onShowHomeIndicatorNotification:)
                 name:FlutterViewControllerShowHomeIndicator
               object:nil];
}

- (void)setUpSceneLifecycleNotifications:(NSNotificationCenter*)center API_AVAILABLE(ios(13.0)) {
  [center addObserver:self
             selector:@selector(sceneBecameActive:)
                 name:UISceneDidActivateNotification
               object:nil];

  [center addObserver:self
             selector:@selector(sceneWillResignActive:)
                 name:UISceneWillDeactivateNotification
               object:nil];

  [center addObserver:self
             selector:@selector(sceneWillDisconnect:)
                 name:UISceneDidDisconnectNotification
               object:nil];

  [center addObserver:self
             selector:@selector(sceneDidEnterBackground:)
                 name:UISceneDidEnterBackgroundNotification
               object:nil];

  [center addObserver:self
             selector:@selector(sceneWillEnterForeground:)
                 name:UISceneWillEnterForegroundNotification
               object:nil];
}

- (void)setUpApplicationLifecycleNotifications:(NSNotificationCenter*)center {
  [center addObserver:self
             selector:@selector(applicationBecameActive:)
                 name:UIApplicationDidBecomeActiveNotification
               object:nil];

  [center addObserver:self
             selector:@selector(applicationWillResignActive:)
                 name:UIApplicationWillResignActiveNotification
               object:nil];

  [center addObserver:self
             selector:@selector(applicationWillTerminate:)
                 name:UIApplicationWillTerminateNotification
               object:nil];

  [center addObserver:self
             selector:@selector(applicationDidEnterBackground:)
                 name:UIApplicationDidEnterBackgroundNotification
               object:nil];

  [center addObserver:self
             selector:@selector(applicationWillEnterForeground:)
                 name:UIApplicationWillEnterForegroundNotification
               object:nil];
}

- (void)setInitialRoute:(NSString*)route {
  [self.engine.navigationChannel invokeMethod:@"setInitialRoute" arguments:route];
}

- (void)popRoute {
  [self.engine.navigationChannel invokeMethod:@"popRoute" arguments:nil];
}

- (void)pushRoute:(NSString*)route {
  [self.engine.navigationChannel invokeMethod:@"pushRoute" arguments:route];
}

#pragma mark - Loading the view

static UIView* GetViewOrPlaceholder(UIView* existing_view) {
  if (existing_view) {
    return existing_view;
  }

  auto placeholder = [[UIView alloc] init];

  placeholder.autoresizingMask = UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight;
  if (@available(iOS 13.0, *)) {
    placeholder.backgroundColor = UIColor.systemBackgroundColor;
  } else {
    placeholder.backgroundColor = UIColor.whiteColor;
  }
  placeholder.autoresizesSubviews = YES;

  // Only add the label when we know we have failed to enable tracing (and it was necessary).
  // Otherwise, a spurious warning will be shown in cases where an engine cannot be initialized for
  // other reasons.
  if (flutter::GetTracingResult() == flutter::TracingResult::kDisabled) {
    auto messageLabel = [[UILabel alloc] init];
    messageLabel.numberOfLines = 0u;
    messageLabel.textAlignment = NSTextAlignmentCenter;
    messageLabel.autoresizingMask =
        UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight;
    messageLabel.text =
        @"In iOS 14+, debug mode Flutter apps can only be launched from Flutter tooling, "
        @"IDEs with Flutter plugins or from Xcode.\n\nAlternatively, build in profile or release "
        @"modes to enable launching from the home screen.";
    [placeholder addSubview:messageLabel];
  }

  return placeholder;
}

- (void)loadView {
  self.view = GetViewOrPlaceholder(self.flutterView);
  self.view.multipleTouchEnabled = YES;
  self.view.autoresizingMask = UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight;

  [self installSplashScreenViewIfNecessary];

  // Create and set up the scroll view.
  UIScrollView* scrollView = [[UIScrollView alloc] init];
  scrollView.autoresizingMask = UIViewAutoresizingFlexibleWidth;
  // The color shouldn't matter since it is offscreen.
  scrollView.backgroundColor = UIColor.whiteColor;
  scrollView.delegate = self;
  // This is an arbitrary small size.
  scrollView.contentSize = CGSizeMake(kScrollViewContentSize, kScrollViewContentSize);
  // This is an arbitrary offset that is not CGPointZero.
  scrollView.contentOffset = CGPointMake(kScrollViewContentSize, kScrollViewContentSize);

  [self.view addSubview:scrollView];
  self.scrollView = scrollView;
}

- (flutter::PointerData)generatePointerDataForFake {
  flutter::PointerData pointer_data;
  pointer_data.Clear();
  pointer_data.kind = flutter::PointerData::DeviceKind::kTouch;
  // `UITouch.timestamp` is defined as seconds since system startup. Synthesized events can get this
  // time with `NSProcessInfo.systemUptime`. See
  // https://developer.apple.com/documentation/uikit/uitouch/1618144-timestamp?language=objc
  pointer_data.time_stamp = [[NSProcessInfo processInfo] systemUptime] * kMicrosecondsPerSecond;
  return pointer_data;
}

static void SendFakeTouchEvent(UIScreen* screen,
                               FlutterEngine* engine,
                               CGPoint location,
                               flutter::PointerData::Change change) {
  const CGFloat scale = screen.scale;
  flutter::PointerData pointer_data = [[engine viewController] generatePointerDataForFake];
  pointer_data.physical_x = location.x * scale;
  pointer_data.physical_y = location.y * scale;
  auto packet = std::make_unique<flutter::PointerDataPacket>(/*count=*/1);
  pointer_data.change = change;
  packet->SetPointerData(0, pointer_data);
  [engine dispatchPointerDataPacket:std::move(packet)];
}

- (BOOL)scrollViewShouldScrollToTop:(UIScrollView*)scrollView {
  if (!self.engine) {
    return NO;
  }
  CGPoint statusBarPoint = CGPointZero;
  UIScreen* screen = self.flutterScreenIfViewLoaded;
  if (screen) {
    SendFakeTouchEvent(screen, self.engine, statusBarPoint, flutter::PointerData::Change::kDown);
    SendFakeTouchEvent(screen, self.engine, statusBarPoint, flutter::PointerData::Change::kUp);
  }
  return NO;
}

#pragma mark - Managing launch views

- (void)installSplashScreenViewIfNecessary {
  // Show the launch screen view again on top of the FlutterView if available.
  // This launch screen view will be removed once the first Flutter frame is rendered.
  if (self.splashScreenView && (self.isBeingPresented || self.isMovingToParentViewController)) {
    [self.splashScreenView removeFromSuperview];
    self.splashScreenView = nil;
    return;
  }

  // Use the property getter to initialize the default value.
  UIView* splashScreenView = self.splashScreenView;
  if (splashScreenView == nil) {
    return;
  }
  splashScreenView.frame = self.view.bounds;
  [self.view addSubview:splashScreenView];
}

+ (BOOL)automaticallyNotifiesObserversOfDisplayingFlutterUI {
  return NO;
}

- (void)setDisplayingFlutterUI:(BOOL)displayingFlutterUI {
  if (_displayingFlutterUI != displayingFlutterUI) {
    if (displayingFlutterUI == YES) {
      if (!self.viewIfLoaded.window) {
        return;
      }
    }
    [self willChangeValueForKey:@"displayingFlutterUI"];
    _displayingFlutterUI = displayingFlutterUI;
    [self didChangeValueForKey:@"displayingFlutterUI"];
  }
}

- (void)callViewRenderedCallback {
  self.displayingFlutterUI = YES;
  if (self.flutterViewRenderedCallback) {
    self.flutterViewRenderedCallback();
    self.flutterViewRenderedCallback = nil;
  }
}

- (void)removeSplashScreenWithCompletion:(dispatch_block_t _Nullable)onComplete {
  NSAssert(self.splashScreenView, @"The splash screen view must not be nil");
  UIView* splashScreen = self.splashScreenView;
  // setSplashScreenView calls this method. Assign directly to ivar to avoid an infinite loop.
  _splashScreenView = nil;
  [UIView animateWithDuration:0.2
      animations:^{
        splashScreen.alpha = 0;
      }
      completion:^(BOOL finished) {
        [splashScreen removeFromSuperview];
        if (onComplete) {
          onComplete();
        }
      }];
}

- (void)onFirstFrameRendered {
  if (self.splashScreenView) {
    __weak FlutterViewController* weakSelf = self;
    [self removeSplashScreenWithCompletion:^{
      [weakSelf callViewRenderedCallback];
    }];
  } else {
    [self callViewRenderedCallback];
  }
}

- (void)installFirstFrameCallback {
  if (!self.engine) {
    return;
  }
  __weak FlutterViewController* weakSelf = self;
  [self.engine installFirstFrameCallback:^{
    [weakSelf onFirstFrameRendered];
  }];
}

#pragma mark - Properties

- (int64_t)viewIdentifier {
  // TODO(dkwingsmt): Fill the view ID property with the correct value once the
  // iOS shell supports multiple views.
  return flutter::kFlutterImplicitViewId;
}

- (BOOL)loadDefaultSplashScreenView {
  NSString* launchscreenName =
      [[[NSBundle mainBundle] infoDictionary] objectForKey:@"UILaunchStoryboardName"];
  if (launchscreenName == nil) {
    return NO;
  }
  UIView* splashView = [self splashScreenFromStoryboard:launchscreenName];
  if (!splashView) {
    splashView = [self splashScreenFromXib:launchscreenName];
  }
  if (!splashView) {
    return NO;
  }
  self.splashScreenView = splashView;
  return YES;
}

- (UIView*)splashScreenFromStoryboard:(NSString*)name {
  UIStoryboard* storyboard = nil;
  @try {
    storyboard = [UIStoryboard storyboardWithName:name bundle:nil];
  } @catch (NSException* exception) {
    return nil;
  }
  if (storyboard) {
    UIViewController* splashScreenViewController = [storyboard instantiateInitialViewController];
    return splashScreenViewController.view;
  }
  return nil;
}

- (UIView*)splashScreenFromXib:(NSString*)name {
  NSArray* objects = nil;
  @try {
    objects = [[NSBundle mainBundle] loadNibNamed:name owner:self options:nil];
  } @catch (NSException* exception) {
    return nil;
  }
  if ([objects count] != 0) {
    UIView* view = [objects objectAtIndex:0];
    return view;
  }
  return nil;
}

- (void)setSplashScreenView:(UIView*)view {
  if (view == _splashScreenView) {
    return;
  }

  // Special case: user wants to remove the splash screen view.
  if (!view) {
    if (_splashScreenView) {
      [self removeSplashScreenWithCompletion:nil];
    }
    return;
  }

  _splashScreenView = view;
  _splashScreenView.autoresizingMask =
      UIViewAutoresizingFlexibleWidth | UIViewAutoresizingFlexibleHeight;
}

- (void)setFlutterViewDidRenderCallback:(void (^)(void))callback {
  _flutterViewRenderedCallback = callback;
}

#pragma mark - Surface creation and teardown updates

- (void)surfaceUpdated:(BOOL)appeared {
  if (!self.engine) {
    return;
  }

  // NotifyCreated/NotifyDestroyed are synchronous and require hops between the UI and raster
  // thread.
  if (appeared) {
    [self installFirstFrameCallback];
    self.platformViewsController.flutterView = self.flutterView;
    self.platformViewsController.flutterViewController = self;
    [self.engine notifyViewCreated];
  } else {
    self.displayingFlutterUI = NO;
    [self.engine notifyViewDestroyed];
    self.platformViewsController.flutterView = nil;
    self.platformViewsController.flutterViewController = nil;
  }
}

#pragma mark - UIViewController lifecycle notifications

- (void)viewDidLoad {
  TRACE_EVENT0("flutter", "viewDidLoad");

  if (self.engine && self.engineNeedsLaunch) {
    [self.engine launchEngine:nil libraryURI:nil entrypointArgs:nil];
    [self.engine setViewController:self];
    self.engineNeedsLaunch = NO;
  } else if (self.engine.viewController == self) {
    [self.engine attachView];
  }

  // Register internal plugins.
  [self addInternalPlugins];

  // Create a vsync client to correct delivery frame rate of touch events if needed.
  [self createTouchRateCorrectionVSyncClientIfNeeded];

  if (@available(iOS 13.4, *)) {
    _hoverGestureRecognizer =
        [[UIHoverGestureRecognizer alloc] initWithTarget:self action:@selector(hoverEvent:)];
    _hoverGestureRecognizer.delegate = self;
    [self.flutterView addGestureRecognizer:_hoverGestureRecognizer];

    _discreteScrollingPanGestureRecognizer =
        [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(discreteScrollEvent:)];
    _discreteScrollingPanGestureRecognizer.allowedScrollTypesMask = UIScrollTypeMaskDiscrete;
    // Disallowing all touch types. If touch events are allowed here, touches to the screen will be
    // consumed by the UIGestureRecognizer instead of being passed through to flutter via
    // touchesBegan. Trackpad and mouse scrolls are sent by the platform as scroll events rather
    // than touch events, so they will still be received.
    _discreteScrollingPanGestureRecognizer.allowedTouchTypes = @[];
    _discreteScrollingPanGestureRecognizer.delegate = self;
    [self.flutterView addGestureRecognizer:_discreteScrollingPanGestureRecognizer];
    _continuousScrollingPanGestureRecognizer =
        [[UIPanGestureRecognizer alloc] initWithTarget:self
                                                action:@selector(continuousScrollEvent:)];
    _continuousScrollingPanGestureRecognizer.allowedScrollTypesMask = UIScrollTypeMaskContinuous;
    _continuousScrollingPanGestureRecognizer.allowedTouchTypes = @[];
    _continuousScrollingPanGestureRecognizer.delegate = self;
    [self.flutterView addGestureRecognizer:_continuousScrollingPanGestureRecognizer];
    _pinchGestureRecognizer =
        [[UIPinchGestureRecognizer alloc] initWithTarget:self action:@selector(pinchEvent:)];
    _pinchGestureRecognizer.allowedTouchTypes = @[];
    _pinchGestureRecognizer.delegate = self;
    [self.flutterView addGestureRecognizer:_pinchGestureRecognizer];
    _rotationGestureRecognizer = [[UIRotationGestureRecognizer alloc] init];
    _rotationGestureRecognizer.allowedTouchTypes = @[];
    _rotationGestureRecognizer.delegate = self;
    [self.flutterView addGestureRecognizer:_rotationGestureRecognizer];
  }

  [super viewDidLoad];
}

- (void)addInternalPlugins {
  self.keyboardManager = [[FlutterKeyboardManager alloc] init];
  __weak FlutterViewController* weakSelf = self;
  FlutterSendKeyEvent sendEvent =
      ^(const FlutterKeyEvent& event, FlutterKeyEventCallback callback, void* userData) {
        [weakSelf.engine sendKeyEvent:event callback:callback userData:userData];
      };
  [self.keyboardManager
      addPrimaryResponder:[[FlutterEmbedderKeyResponder alloc] initWithSendEvent:sendEvent]];
  FlutterChannelKeyResponder* responder =
      [[FlutterChannelKeyResponder alloc] initWithChannel:self.engine.keyEventChannel];
  [self.keyboardManager addPrimaryResponder:responder];
  FlutterTextInputPlugin* textInputPlugin = self.engine.textInputPlugin;
  if (textInputPlugin != nil) {
    [self.keyboardManager addSecondaryResponder:textInputPlugin];
  }
  if (self.engine.viewController == self) {
    [textInputPlugin setUpIndirectScribbleInteraction:self];
  }
}

- (void)removeInternalPlugins {
  self.keyboardManager = nil;
}

- (void)viewWillAppear:(BOOL)animated {
  TRACE_EVENT0("flutter", "viewWillAppear");
  if (self.engine.viewController == self) {
    // Send platform settings to Flutter, e.g., platform brightness.
    [self onUserSettingsChanged:nil];

    // Only recreate surface on subsequent appearances when viewport metrics are known.
    // First time surface creation is done on viewDidLayoutSubviews.
    if (_viewportMetrics.physical_width) {
      [self surfaceUpdated:YES];
    }
    [self.engine.lifecycleChannel sendMessage:@"AppLifecycleState.inactive"];
    [self.engine.restorationPlugin markRestorationComplete];
  }

  [super viewWillAppear:animated];
}

- (void)viewDidAppear:(BOOL)animated {
  TRACE_EVENT0("flutter", "viewDidAppear");
  if (self.engine.viewController == self) {
    [self onUserSettingsChanged:nil];
    [self onAccessibilityStatusChanged:nil];
    BOOL stateIsActive = YES;
#if APPLICATION_EXTENSION_API_ONLY
    if (@available(iOS 13.0, *)) {
      stateIsActive = self.flutterWindowSceneIfViewLoaded.activationState ==
                      UISceneActivationStateForegroundActive;
    }
#else
    stateIsActive = UIApplication.sharedApplication.applicationState == UIApplicationStateActive;
#endif
    if (stateIsActive) {
      [self.engine.lifecycleChannel sendMessage:@"AppLifecycleState.resumed"];
    }
  }
  [super viewDidAppear:animated];
}

- (void)viewWillDisappear:(BOOL)animated {
  TRACE_EVENT0("flutter", "viewWillDisappear");
  if (self.engine.viewController == self) {
    [self.engine.lifecycleChannel sendMessage:@"AppLifecycleState.inactive"];
  }
  [super viewWillDisappear:animated];
}

- (void)viewDidDisappear:(BOOL)animated {
  TRACE_EVENT0("flutter", "viewDidDisappear");
  if (self.engine.viewController == self) {
    [self invalidateKeyboardAnimationVSyncClient];
    [self ensureViewportMetricsIsCorrect];
    [self surfaceUpdated:NO];
    [self.engine.lifecycleChannel sendMessage:@"AppLifecycleState.paused"];
    [self flushOngoingTouches];
    [self.engine notifyLowMemory];
  }

  [super viewDidDisappear:animated];
}

- (void)viewWillTransitionToSize:(CGSize)size
       withTransitionCoordinator:(id<UIViewControllerTransitionCoordinator>)coordinator {
  [super viewWillTransitionToSize:size withTransitionCoordinator:coordinator];

  // We delay the viewport metrics update for half of rotation transition duration, to address
  // a bug with distorted aspect ratio.
  // See: https://github.com/flutter/flutter/issues/16322
  //
  // This approach does not fully resolve all distortion problem. But instead, it reduces the
  // rotation distortion roughly from 4x to 2x. The most distorted frames occur in the middle
  // of the transition when it is rotating the fastest, making it hard to notice.

  NSTimeInterval transitionDuration = coordinator.transitionDuration;
  // Do not delay viewport metrics update if zero transition duration.
  if (transitionDuration == 0) {
    return;
  }

  __weak FlutterViewController* weakSelf = self;
  _shouldIgnoreViewportMetricsUpdatesDuringRotation = YES;
  dispatch_after(dispatch_time(DISPATCH_TIME_NOW,
                               static_cast<int64_t>(transitionDuration / 2.0 * NSEC_PER_SEC)),
                 dispatch_get_main_queue(), ^{
                   FlutterViewController* strongSelf = weakSelf;
                   if (!strongSelf) {
                     return;
                   }

                   // `viewWillTransitionToSize` is only called after the previous rotation is
                   // complete. So there won't be race condition for this flag.
                   strongSelf.shouldIgnoreViewportMetricsUpdatesDuringRotation = NO;
                   [strongSelf updateViewportMetricsIfNeeded];
                 });
}

- (void)flushOngoingTouches {
  if (self.engine && self.ongoingTouches.count > 0) {
    auto packet = std::make_unique<flutter::PointerDataPacket>(self.ongoingTouches.count);
    size_t pointer_index = 0;
    // If the view controller is going away, we want to flush cancel all the ongoing
    // touches to the framework so nothing gets orphaned.
    for (NSNumber* device in self.ongoingTouches) {
      // Create fake PointerData to balance out each previously started one for the framework.
      flutter::PointerData pointer_data = [self generatePointerDataForFake];

      pointer_data.change = flutter::PointerData::Change::kCancel;
      pointer_data.device = device.longLongValue;
      pointer_data.pointer_identifier = 0;
      pointer_data.view_id = self.viewIdentifier;

      // Anything we put here will be arbitrary since there are no touches.
      pointer_data.physical_x = 0;
      pointer_data.physical_y = 0;
      pointer_data.physical_delta_x = 0.0;
      pointer_data.physical_delta_y = 0.0;
      pointer_data.pressure = 1.0;
      pointer_data.pressure_max = 1.0;

      packet->SetPointerData(pointer_index++, pointer_data);
    }

    [self.ongoingTouches removeAllObjects];
    [self.engine dispatchPointerDataPacket:std::move(packet)];
  }
}

- (void)deregisterNotifications {
  [[NSNotificationCenter defaultCenter] postNotificationName:FlutterViewControllerWillDealloc
                                                      object:self
                                                    userInfo:nil];
  [[NSNotificationCenter defaultCenter] removeObserver:self];
}

- (void)dealloc {
  // TODO(cbracken): https://github.com/flutter/flutter/issues/157140
  // Eliminate method calls in initializers and dealloc.
  [self removeInternalPlugins];
  [self deregisterNotifications];

  [self invalidateKeyboardAnimationVSyncClient];
  [self invalidateTouchRateCorrectionVSyncClient];

  // TODO(cbracken): https://github.com/flutter/flutter/issues/156222
  // Ensure all delegates are weak and remove this.
  _scrollView.delegate = nil;
  _hoverGestureRecognizer.delegate = nil;
  _discreteScrollingPanGestureRecognizer.delegate = nil;
  _continuousScrollingPanGestureRecognizer.delegate = nil;
  _pinchGestureRecognizer.delegate = nil;
  _rotationGestureRecognizer.delegate = nil;
}

#pragma mark - Application lifecycle notifications

- (void)applicationBecameActive:(NSNotification*)notification {
  TRACE_EVENT0("flutter", "applicationBecameActive");
  [self appOrSceneBecameActive];
}

- (void)applicationWillResignActive:(NSNotification*)notification {
  TRACE_EVENT0("flutter", "applicationWillResignActive");
  [self appOrSceneWillResignActive];
}

- (void)applicationWillTerminate:(NSNotification*)notification {
  [self appOrSceneWillTerminate];
}

- (void)applicationDidEnterBackground:(NSNotification*)notification {
  TRACE_EVENT0("flutter", "applicationDidEnterBackground");
  [self appOrSceneDidEnterBackground];
}

- (void)applicationWillEnterForeground:(NSNotification*)notification {
  TRACE_EVENT0("flutter", "applicationWillEnterForeground");
  [self appOrSceneWillEnterForeground];
}

#pragma mark - Scene lifecycle notifications

- (void)sceneBecameActive:(NSNotification*)notification API_AVAILABLE(ios(13.0)) {
  TRACE_EVENT0("flutter", "sceneBecameActive");
  [self appOrSceneBecameActive];
}

- (void)sceneWillResignActive:(NSNotification*)notification API_AVAILABLE(ios(13.0)) {
  TRACE_EVENT0("flutter", "sceneWillResignActive");
  [self appOrSceneWillResignActive];
}

- (void)sceneWillDisconnect:(NSNotification*)notification API_AVAILABLE(ios(13.0)) {
  [self appOrSceneWillTerminate];
}

- (void)sceneDidEnterBackground:(NSNotification*)notification API_AVAILABLE(ios(13.0)) {
  TRACE_EVENT0("flutter", "sceneDidEnterBackground");
  [self appOrSceneDidEnterBackground];
}

- (void)sceneWillEnterForeground:(NSNotification*)notification API_AVAILABLE(ios(13.0)) {
  TRACE_EVENT0("flutter", "sceneWillEnterForeground");
  [self appOrSceneWillEnterForeground];
}

#pragma mark - Lifecycle shared

- (void)appOrSceneBecameActive {
  self.isKeyboardInOrTransitioningFromBackground = NO;
  if (_viewportMetrics.physical_width) {
    [self surfaceUpdated:YES];
  }
  [self performSelector:@selector(goToApplicationLifecycle:)
             withObject:@"AppLifecycleState.resumed"
             afterDelay:0.0f];
}

- (void)appOrSceneWillResignActive {
  [NSObject cancelPreviousPerformRequestsWithTarget:self
                                           selector:@selector(goToApplicationLifecycle:)
                                             object:@"AppLifecycleState.resumed"];
  [self goToApplicationLifecycle:@"AppLifecycleState.inactive"];
}

- (void)appOrSceneWillTerminate {
  [self goToApplicationLifecycle:@"AppLifecycleState.detached"];
  [self.engine destroyContext];
}

- (void)appOrSceneDidEnterBackground {
  self.isKeyboardInOrTransitioningFromBackground = YES;
  [self surfaceUpdated:NO];
  [self goToApplicationLifecycle:@"AppLifecycleState.paused"];
}

- (void)appOrSceneWillEnterForeground {
  [self goToApplicationLifecycle:@"AppLifecycleState.inactive"];
}

// Make this transition only while this current view controller is visible.
- (void)goToApplicationLifecycle:(nonnull NSString*)state {
  // Accessing self.view will create the view. Instead use viewIfLoaded
  // to check whether the view is attached to window.
  if (self.viewIfLoaded.window) {
    [self.engine.lifecycleChannel sendMessage:state];
  }
}

#pragma mark - Touch event handling

static flutter::PointerData::Change PointerDataChangeFromUITouchPhase(UITouchPhase phase) {
  switch (phase) {
    case UITouchPhaseBegan:
      return flutter::PointerData::Change::kDown;
    case UITouchPhaseMoved:
    case UITouchPhaseStationary:
      // There is no EVENT_TYPE_POINTER_STATIONARY. So we just pass a move type
      // with the same coordinates
      return flutter::PointerData::Change::kMove;
    case UITouchPhaseEnded:
      return flutter::PointerData::Change::kUp;
    case UITouchPhaseCancelled:
      return flutter::PointerData::Change::kCancel;
    default:
      // TODO(53695): Handle the `UITouchPhaseRegion`... enum values.
      FML_DLOG(INFO) << "Unhandled touch phase: " << phase;
      break;
  }

  return flutter::PointerData::Change::kCancel;
}

static flutter::PointerData::DeviceKind DeviceKindFromTouchType(UITouch* touch) {
  switch (touch.type) {
    case UITouchTypeDirect:
    case UITouchTypeIndirect:
      return flutter::PointerData::DeviceKind::kTouch;
    case UITouchTypeStylus:
      return flutter::PointerData::DeviceKind::kStylus;
    case UITouchTypeIndirectPointer:
      return flutter::PointerData::DeviceKind::kMouse;
    default:
      FML_DLOG(INFO) << "Unhandled touch type: " << touch.type;
      break;
  }

  return flutter::PointerData::DeviceKind::kTouch;
}

// Dispatches the UITouches to the engine. Usually, the type of change of the touch is determined
// from the UITouch's phase. However, FlutterAppDelegate fakes touches to ensure that touch events
// in the status bar area are available to framework code. The change type (optional) of the faked
// touch is specified in the second argument.
- (void)dispatchTouches:(NSSet*)touches
    pointerDataChangeOverride:(flutter::PointerData::Change*)overridden_change
                        event:(UIEvent*)event {
  if (!self.engine) {
    return;
  }

  // If the UIApplicationSupportsIndirectInputEvents in Info.plist returns YES, then the platform
  // dispatches indirect pointer touches (trackpad clicks) as UITouch with a type of
  // UITouchTypeIndirectPointer and different identifiers for each click. They are translated into
  // Flutter pointer events with type of kMouse and different device IDs. These devices must be
  // terminated with kRemove events when the touches end, otherwise they will keep triggering hover
  // events.
  //
  // If the UIApplicationSupportsIndirectInputEvents in Info.plist returns NO, then the platform
  // dispatches indirect pointer touches (trackpad clicks) as UITouch with a type of
  // UITouchTypeIndirectPointer and different identifiers for each click. They are translated into
  // Flutter pointer events with type of kTouch and different device IDs. Removing these devices is
  // neither necessary nor harmful.
  //
  // Therefore Flutter always removes these devices. The touches_to_remove_count tracks how many
  // remove events are needed in this group of touches to properly allocate space for the packet.
  // The remove event of a touch is synthesized immediately after its normal event.
  //
  // See also:
  // https://developer.apple.com/documentation/uikit/pointer_interactions?language=objc
  // https://developer.apple.com/documentation/bundleresources/information_property_list/uiapplicationsupportsindirectinputevents?language=objc
  NSUInteger touches_to_remove_count = 0;
  for (UITouch* touch in touches) {
    if (touch.phase == UITouchPhaseEnded || touch.phase == UITouchPhaseCancelled) {
      touches_to_remove_count++;
    }
  }

  // Activate or pause the correction of delivery frame rate of touch events.
  [self triggerTouchRateCorrectionIfNeeded:touches];

  const CGFloat scale = self.flutterScreenIfViewLoaded.scale;
  auto packet =
      std::make_unique<flutter::PointerDataPacket>(touches.count + touches_to_remove_count);

  size_t pointer_index = 0;

  for (UITouch* touch in touches) {
    CGPoint windowCoordinates = [touch locationInView:self.view];

    flutter::PointerData pointer_data;
    pointer_data.Clear();

    constexpr int kMicrosecondsPerSecond = 1000 * 1000;
    pointer_data.time_stamp = touch.timestamp * kMicrosecondsPerSecond;

    pointer_data.change = overridden_change != nullptr
                              ? *overridden_change
                              : PointerDataChangeFromUITouchPhase(touch.phase);

    pointer_data.kind = DeviceKindFromTouchType(touch);

    pointer_data.device = reinterpret_cast<int64_t>(touch);

    pointer_data.view_id = self.viewIdentifier;

    // Pointer will be generated in pointer_data_packet_converter.cc.
    pointer_data.pointer_identifier = 0;

    pointer_data.physical_x = windowCoordinates.x * scale;
    pointer_data.physical_y = windowCoordinates.y * scale;

    // Delta will be generated in pointer_data_packet_converter.cc.
    pointer_data.physical_delta_x = 0.0;
    pointer_data.physical_delta_y = 0.0;

    NSNumber* deviceKey = [NSNumber numberWithLongLong:pointer_data.device];
    // Track touches that began and not yet stopped so we can flush them
    // if the view controller goes away.
    switch (pointer_data.change) {
      case flutter::PointerData::Change::kDown:
        [self.ongoingTouches addObject:deviceKey];
        break;
      case flutter::PointerData::Change::kCancel:
      case flutter::PointerData::Change::kUp:
        [self.ongoingTouches removeObject:deviceKey];
        break;
      case flutter::PointerData::Change::kHover:
      case flutter::PointerData::Change::kMove:
        // We're only tracking starts and stops.
        break;
      case flutter::PointerData::Change::kAdd:
      case flutter::PointerData::Change::kRemove:
        // We don't use kAdd/kRemove.
        break;
      case flutter::PointerData::Change::kPanZoomStart:
      case flutter::PointerData::Change::kPanZoomUpdate:
      case flutter::PointerData::Change::kPanZoomEnd:
        // We don't send pan/zoom events here
        break;
    }

    // pressure_min is always 0.0
    pointer_data.pressure = touch.force;
    pointer_data.pressure_max = touch.maximumPossibleForce;
    pointer_data.radius_major = touch.majorRadius;
    pointer_data.radius_min = touch.majorRadius - touch.majorRadiusTolerance;
    pointer_data.radius_max = touch.majorRadius + touch.majorRadiusTolerance;

    // iOS Documentation: altitudeAngle
    // A value of 0 radians indicates that the stylus is parallel to the surface. The value of
    // this property is Pi/2 when the stylus is perpendicular to the surface.
    //
    // PointerData Documentation: tilt
    // The angle of the stylus, in radians in the range:
    //    0 <= tilt <= pi/2
    // giving the angle of the axis of the stylus, relative to the axis perpendicular to the input
    // surface (thus 0.0 indicates the stylus is orthogonal to the plane of the input surface,
    // while pi/2 indicates that the stylus is flat on that surface).
    //
    // Discussion:
    // The ranges are the same. Origins are swapped.
    pointer_data.tilt = M_PI_2 - touch.altitudeAngle;

    // iOS Documentation: azimuthAngleInView:
    // With the tip of the stylus touching the screen, the value of this property is 0 radians
    // when the cap end of the stylus (that is, the end opposite of the tip) points along the
    // positive x axis of the device's screen. The azimuth angle increases as the user swings the
    // cap end of the stylus in a clockwise direction around the tip.
    //
    // PointerData Documentation: orientation
    // The angle of the stylus, in radians in the range:
    //    -pi < orientation <= pi
    // giving the angle of the axis of the stylus projected onto the input surface, relative to
    // the positive y-axis of that surface (thus 0.0 indicates the stylus, if projected onto that
    // surface, would go from the contact point vertically up in the positive y-axis direction, pi
    // would indicate that the stylus would go down in the negative y-axis direction; pi/4 would
    // indicate that the stylus goes up and to the right, -pi/2 would indicate that the stylus
    // goes to the left, etc).
    //
    // Discussion:
    // Sweep direction is the same. Phase of M_PI_2.
    pointer_data.orientation = [touch azimuthAngleInView:nil] - M_PI_2;

    if (@available(iOS 13.4, *)) {
      if (event != nullptr) {
        pointer_data.buttons = (((event.buttonMask & UIEventButtonMaskPrimary) > 0)
                                    ? flutter::PointerButtonMouse::kPointerButtonMousePrimary
                                    : 0) |
                               (((event.buttonMask & UIEventButtonMaskSecondary) > 0)
                                    ? flutter::PointerButtonMouse::kPointerButtonMouseSecondary
                                    : 0);
      }
    }

    packet->SetPointerData(pointer_index++, pointer_data);

    if (touch.phase == UITouchPhaseEnded || touch.phase == UITouchPhaseCancelled) {
      flutter::PointerData remove_pointer_data = pointer_data;
      remove_pointer_data.change = flutter::PointerData::Change::kRemove;
      packet->SetPointerData(pointer_index++, remove_pointer_data);
    }
  }

  [self.engine dispatchPointerDataPacket:std::move(packet)];
}

- (void)touchesBegan:(NSSet*)touches withEvent:(UIEvent*)event {
  [self dispatchTouches:touches pointerDataChangeOverride:nullptr event:event];
}

- (void)touchesMoved:(NSSet*)touches withEvent:(UIEvent*)event {
  [self dispatchTouches:touches pointerDataChangeOverride:nullptr event:event];
}

- (void)touchesEnded:(NSSet*)touches withEvent:(UIEvent*)event {
  [self dispatchTouches:touches pointerDataChangeOverride:nullptr event:event];
}

- (void)touchesCancelled:(NSSet*)touches withEvent:(UIEvent*)event {
  [self dispatchTouches:touches pointerDataChangeOverride:nullptr event:event];
}

- (void)forceTouchesCancelled:(NSSet*)touches {
  flutter::PointerData::Change cancel = flutter::PointerData::Change::kCancel;
  [self dispatchTouches:touches pointerDataChangeOverride:&cancel event:nullptr];
}

#pragma mark - Touch events rate correction

- (void)createTouchRateCorrectionVSyncClientIfNeeded {
  if (_touchRateCorrectionVSyncClient != nil) {
    return;
  }

  double displayRefreshRate = DisplayLinkManager.displayRefreshRate;
  const double epsilon = 0.1;
  if (displayRefreshRate < 60.0 + epsilon) {  // displayRefreshRate <= 60.0

    // If current device's max frame rate is not larger than 60HZ, the delivery rate of touch events
    // is the same with render vsync rate. So it is unnecessary to create
    // _touchRateCorrectionVSyncClient to correct touch callback's rate.
    return;
  }

  auto callback = [](std::unique_ptr<flutter::FrameTimingsRecorder> recorder) {
    // Do nothing in this block. Just trigger system to callback touch events with correct rate.
  };
  _touchRateCorrectionVSyncClient =
      [[VSyncClient alloc] initWithTaskRunner:self.engine.platformTaskRunner callback:callback];
  _touchRateCorrectionVSyncClient.allowPauseAfterVsync = NO;
}

- (void)triggerTouchRateCorrectionIfNeeded:(NSSet*)touches {
  if (_touchRateCorrectionVSyncClient == nil) {
    // If the _touchRateCorrectionVSyncClient is not created, means current devices doesn't
    // need to correct the touch rate. So just return.
    return;
  }

  // As long as there is a touch's phase is UITouchPhaseBegan or UITouchPhaseMoved,
  // activate the correction. Otherwise pause the correction.
  BOOL isUserInteracting = NO;
  for (UITouch* touch in touches) {
    if (touch.phase == UITouchPhaseBegan || touch.phase == UITouchPhaseMoved) {
      isUserInteracting = YES;
      break;
    }
  }

  if (isUserInteracting && self.engine.viewController == self) {
    [_touchRateCorrectionVSyncClient await];
  } else {
    [_touchRateCorrectionVSyncClient pause];
  }
}

- (void)invalidateTouchRateCorrectionVSyncClient {
  [_touchRateCorrectionVSyncClient invalidate];
  _touchRateCorrectionVSyncClient = nil;
}

#pragma mark - Handle view resizing

- (void)updateViewportMetricsIfNeeded {
  if (_shouldIgnoreViewportMetricsUpdatesDuringRotation) {
    return;
  }
  if (self.engine.viewController == self) {
    [self.engine updateViewportMetrics:_viewportMetrics];
  }
}

- (void)viewDidLayoutSubviews {
  CGRect viewBounds = self.view.bounds;
  CGFloat scale = self.flutterScreenIfViewLoaded.scale;

  // Purposefully place this not visible.
  self.scrollView.frame = CGRectMake(0.0, 0.0, viewBounds.size.width, 0.0);
  self.scrollView.contentOffset = CGPointMake(kScrollViewContentSize, kScrollViewContentSize);

  // First time since creation that the dimensions of its view is known.
  bool firstViewBoundsUpdate = !_viewportMetrics.physical_width;
  _viewportMetrics.device_pixel_ratio = scale;
  [self setViewportMetricsSize];
  [self setViewportMetricsPaddings];
  [self updateViewportMetricsIfNeeded];

  // There is no guarantee that UIKit will layout subviews when the application/scene is active.
  // Creating the surface when inactive will cause GPU accesses from the background. Only wait for
  // the first frame to render when the application/scene is actually active.
  bool applicationOrSceneIsActive = YES;
#if APPLICATION_EXTENSION_API_ONLY
  if (@available(iOS 13.0, *)) {
    applicationOrSceneIsActive = self.flutterWindowSceneIfViewLoaded.activationState ==
                                 UISceneActivationStateForegroundActive;
  }
#else
  applicationOrSceneIsActive =
      [UIApplication sharedApplication].applicationState == UIApplicationStateActive;
#endif

  // This must run after updateViewportMetrics so that the surface creation tasks are queued after
  // the viewport metrics update tasks.
  if (firstViewBoundsUpdate && applicationOrSceneIsActive && self.engine) {
    [self surfaceUpdated:YES];
#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG
    NSTimeInterval timeout = 0.2;
#else
    NSTimeInterval timeout = 0.1;
#endif
    [self.engine
        waitForFirstFrameSync:timeout
                     callback:^(BOOL didTimeout) {
                       if (didTimeout) {
                         FML_LOG(INFO)
                             << "Timeout waiting for the first frame to render. This may happen in "
                                "unoptimized builds. If this is a release build, you should load a "
                                "less complex frame to avoid the timeout.";
                       }
                     }];
  }
}

- (void)viewSafeAreaInsetsDidChange {
  [self setViewportMetricsPaddings];
  [self updateViewportMetricsIfNeeded];
  [super viewSafeAreaInsetsDidChange];
}

// Set _viewportMetrics physical size.
- (void)setViewportMetricsSize {
  UIScreen* screen = self.flutterScreenIfViewLoaded;
  if (!screen) {
    return;
  }

  CGFloat scale = screen.scale;
  _viewportMetrics.physical_width = self.view.bounds.size.width * scale;
  _viewportMetrics.physical_height = self.view.bounds.size.height * scale;
}

// Set _viewportMetrics physical paddings.
//
// Viewport paddings represent the iOS safe area insets.
- (void)setViewportMetricsPaddings {
  UIScreen* screen = self.flutterScreenIfViewLoaded;
  if (!screen) {
    return;
  }

  CGFloat scale = screen.scale;
  _viewportMetrics.physical_padding_top = self.view.safeAreaInsets.top * scale;
  _viewportMetrics.physical_padding_left = self.view.safeAreaInsets.left * scale;
  _viewportMetrics.physical_padding_right = self.view.safeAreaInsets.right * scale;
  _viewportMetrics.physical_padding_bottom = self.view.safeAreaInsets.bottom * scale;
}

#pragma mark - Keyboard events

- (void)keyboardWillShowNotification:(NSNotification*)notification {
  // Immediately prior to a docked keyboard being shown or when a keyboard goes from
  // undocked/floating to docked, this notification is triggered. This notification also happens
  // when Minimized/Expanded Shortcuts bar is dropped after dragging (the keyboard's end frame will
  // be CGRectZero).
  [self handleKeyboardNotification:notification];
}

- (void)keyboardWillChangeFrame:(NSNotification*)notification {
  // Immediately prior to a change in keyboard frame, this notification is triggered.
  // Sometimes when the keyboard is being hidden or undocked, this notification's keyboard's end
  // frame is not yet entirely out of screen, which is why we also use
  // UIKeyboardWillHideNotification.
  [self handleKeyboardNotification:notification];
}

- (void)keyboardWillBeHidden:(NSNotification*)notification {
  // When keyboard is hidden or undocked, this notification will be triggered.
  // This notification might not occur when the keyboard is changed from docked to floating, which
  // is why we also use UIKeyboardWillChangeFrameNotification.
  [self handleKeyboardNotification:notification];
}

- (void)handleKeyboardNotification:(NSNotification*)notification {
  // See https://flutter.dev/go/ios-keyboard-calculating-inset for more details
  // on why notifications are used and how things are calculated.
  if ([self shouldIgnoreKeyboardNotification:notification]) {
    return;
  }

  NSDictionary* info = notification.userInfo;
  CGRect beginKeyboardFrame = [info[UIKeyboardFrameBeginUserInfoKey] CGRectValue];
  CGRect keyboardFrame = [info[UIKeyboardFrameEndUserInfoKey] CGRectValue];
  FlutterKeyboardMode keyboardMode = [self calculateKeyboardAttachMode:notification];
  CGFloat calculatedInset = [self calculateKeyboardInset:keyboardFrame keyboardMode:keyboardMode];

  // Avoid double triggering startKeyBoardAnimation.
  if (self.targetViewInsetBottom == calculatedInset) {
    return;
  }

  self.targetViewInsetBottom = calculatedInset;
  NSTimeInterval duration = [info[UIKeyboardAnimationDurationUserInfoKey] doubleValue];

  // Flag for simultaneous compounding animation calls.
  // This captures animation calls made while the keyboard animation is currently animating. If the
  // new animation is in the same direction as the current animation, this flag lets the current
  // animation continue with an updated targetViewInsetBottom instead of starting a new keyboard
  // animation. This allows for smoother keyboard animation interpolation.
  BOOL keyboardWillShow = beginKeyboardFrame.origin.y > keyboardFrame.origin.y;
  BOOL keyboardAnimationIsCompounding =
      self.keyboardAnimationIsShowing == keyboardWillShow && _keyboardAnimationVSyncClient != nil;

  // Mark keyboard as showing or hiding.
  self.keyboardAnimationIsShowing = keyboardWillShow;

  if (!keyboardAnimationIsCompounding) {
    [self startKeyBoardAnimation:duration];
  } else if (self.keyboardSpringAnimation) {
    self.keyboardSpringAnimation.toValue = self.targetViewInsetBottom;
  }
}

- (BOOL)shouldIgnoreKeyboardNotification:(NSNotification*)notification {
  // Don't ignore UIKeyboardWillHideNotification notifications.
  // Even if the notification is triggered in the background or by a different app/view controller,
  // we want to always handle this notification to avoid inaccurate inset when in a mulitasking mode
  // or when switching between apps.
  if (notification.name == UIKeyboardWillHideNotification) {
    return NO;
  }

  // Ignore notification when keyboard's dimensions and position are all zeroes for
  // UIKeyboardWillChangeFrameNotification. This happens when keyboard is dragged. Do not ignore if
  // the notification is UIKeyboardWillShowNotification, as CGRectZero for that notfication only
  // occurs when Minimized/Expanded Shortcuts Bar is dropped after dragging, which we later use to
  // categorize it as floating.
  NSDictionary* info = notification.userInfo;
  CGRect keyboardFrame = [info[UIKeyboardFrameEndUserInfoKey] CGRectValue];
  if (notification.name == UIKeyboardWillChangeFrameNotification &&
      CGRectEqualToRect(keyboardFrame, CGRectZero)) {
    return YES;
  }

  // When keyboard's height or width is set to 0, don't ignore. This does not happen
  // often but can happen sometimes when switching between multitasking modes.
  if (CGRectIsEmpty(keyboardFrame)) {
    return NO;
  }

  // Ignore keyboard notifications related to other apps or view controllers.
  if ([self isKeyboardNotificationForDifferentView:notification]) {
    return YES;
  }

  if (@available(iOS 13.0, *)) {
    // noop
  } else {
    // If OS version is less than 13, ignore notification if the app is in the background
    // or is transitioning from the background. In older versions, when switching between
    // apps with the keyboard open in the secondary app, notifications are sent when
    // the app is in the background/transitioning from background as if they belong
    // to the app and as if the keyboard is showing even though it is not.
    if (self.isKeyboardInOrTransitioningFromBackground) {
      return YES;
    }
  }

  return NO;
}

- (BOOL)isKeyboardNotificationForDifferentView:(NSNotification*)notification {
  NSDictionary* info = notification.userInfo;
  // Keyboard notifications related to other apps.
  // If the UIKeyboardIsLocalUserInfoKey key doesn't exist (this should not happen after iOS 8),
  // proceed as if it was local so that the notification is not ignored.
  id isLocal = info[UIKeyboardIsLocalUserInfoKey];
  if (isLocal && ![isLocal boolValue]) {
    return YES;
  }
  return self.engine.viewController != self;
}

- (FlutterKeyboardMode)calculateKeyboardAttachMode:(NSNotification*)notification {
  // There are multiple types of keyboard: docked, undocked, split, split docked,
  // floating, expanded shortcuts bar, minimized shortcuts bar. This function will categorize
  // the keyboard as one of the following modes: docked, floating, or hidden.
  // Docked mode includes docked, split docked, expanded shortcuts bar (when opening via click),
  // and minimized shortcuts bar (when opened via click).
  // Floating includes undocked, split, floating, expanded shortcuts bar (when dragged and dropped),
  // and minimized shortcuts bar (when dragged and dropped).
  NSDictionary* info = notification.userInfo;
  CGRect keyboardFrame = [info[UIKeyboardFrameEndUserInfoKey] CGRectValue];

  if (notification.name == UIKeyboardWillHideNotification) {
    return FlutterKeyboardModeHidden;
  }

  // If keyboard's dimensions and position are all zeroes, that means it's a Minimized/Expanded
  // Shortcuts Bar that has been dropped after dragging, which we categorize as floating.
  if (CGRectEqualToRect(keyboardFrame, CGRectZero)) {
    return FlutterKeyboardModeFloating;
  }
  // If keyboard's width or height are 0, it's hidden.
  if (CGRectIsEmpty(keyboardFrame)) {
    return FlutterKeyboardModeHidden;
  }

  CGRect screenRect = self.flutterScreenIfViewLoaded.bounds;
  CGRect adjustedKeyboardFrame = keyboardFrame;
  adjustedKeyboardFrame.origin.y += [self calculateMultitaskingAdjustment:screenRect
                                                            keyboardFrame:keyboardFrame];

  // If the keyboard is partially or fully showing within the screen, it's either docked or
  // floating. Sometimes with custom keyboard extensions, the keyboard's position may be off by a
  // small decimal amount (which is why CGRectIntersectRect can't be used). Round to compare.
  CGRect intersection = CGRectIntersection(adjustedKeyboardFrame, screenRect);
  CGFloat intersectionHeight = CGRectGetHeight(intersection);
  CGFloat intersectionWidth = CGRectGetWidth(intersection);
  if (round(intersectionHeight) > 0 && intersectionWidth > 0) {
    // If the keyboard is above the bottom of the screen, it's floating.
    CGFloat screenHeight = CGRectGetHeight(screenRect);
    CGFloat adjustedKeyboardBottom = CGRectGetMaxY(adjustedKeyboardFrame);
    if (round(adjustedKeyboardBottom) < screenHeight) {
      return FlutterKeyboardModeFloating;
    }
    return FlutterKeyboardModeDocked;
  }
  return FlutterKeyboardModeHidden;
}

- (CGFloat)calculateMultitaskingAdjustment:(CGRect)screenRect keyboardFrame:(CGRect)keyboardFrame {
  // In Slide Over mode, the keyboard's frame does not include the space
  // below the app, even though the keyboard may be at the bottom of the screen.
  // To handle, shift the Y origin by the amount of space below the app.
  if (self.viewIfLoaded.traitCollection.userInterfaceIdiom == UIUserInterfaceIdiomPad &&
      self.viewIfLoaded.traitCollection.horizontalSizeClass == UIUserInterfaceSizeClassCompact &&
      self.viewIfLoaded.traitCollection.verticalSizeClass == UIUserInterfaceSizeClassRegular) {
    CGFloat screenHeight = CGRectGetHeight(screenRect);
    CGFloat keyboardBottom = CGRectGetMaxY(keyboardFrame);

    // Stage Manager mode will also meet the above parameters, but it does not handle
    // the keyboard positioning the same way, so skip if keyboard is at bottom of page.
    if (screenHeight == keyboardBottom) {
      return 0;
    }
    CGRect viewRectRelativeToScreen =
        [self.viewIfLoaded convertRect:self.viewIfLoaded.frame
                     toCoordinateSpace:self.flutterScreenIfViewLoaded.coordinateSpace];
    CGFloat viewBottom = CGRectGetMaxY(viewRectRelativeToScreen);
    CGFloat offset = screenHeight - viewBottom;
    if (offset > 0) {
      return offset;
    }
  }
  return 0;
}

- (CGFloat)calculateKeyboardInset:(CGRect)keyboardFrame keyboardMode:(NSInteger)keyboardMode {
  // Only docked keyboards will have an inset.
  if (keyboardMode == FlutterKeyboardModeDocked) {
    // Calculate how much of the keyboard intersects with the view.
    CGRect viewRectRelativeToScreen =
        [self.viewIfLoaded convertRect:self.viewIfLoaded.frame
                     toCoordinateSpace:self.flutterScreenIfViewLoaded.coordinateSpace];
    CGRect intersection = CGRectIntersection(keyboardFrame, viewRectRelativeToScreen);
    CGFloat portionOfKeyboardInView = CGRectGetHeight(intersection);

    // The keyboard is treated as an inset since we want to effectively reduce the window size by
    // the keyboard height. The Dart side will compute a value accounting for the keyboard-consuming
    // bottom padding.
    CGFloat scale = self.flutterScreenIfViewLoaded.scale;
    return portionOfKeyboardInView * scale;
  }
  return 0;
}

- (void)startKeyBoardAnimation:(NSTimeInterval)duration {
  // If current physical_view_inset_bottom == targetViewInsetBottom, do nothing.
  if (_viewportMetrics.physical_view_inset_bottom == self.targetViewInsetBottom) {
    return;
  }

  // When this method is called for the first time,
  // initialize the keyboardAnimationView to get animation interpolation during animation.
  if (!self.keyboardAnimationView) {
    UIView* keyboardAnimationView = [[UIView alloc] init];
    keyboardAnimationView.hidden = YES;
    self.keyboardAnimationView = keyboardAnimationView;
  }

  if (!self.keyboardAnimationView.superview) {
    [self.view addSubview:self.keyboardAnimationView];
  }

  // Remove running animation when start another animation.
  [self.keyboardAnimationView.layer removeAllAnimations];

  // Set animation begin value and DisplayLink tracking values.
  self.keyboardAnimationView.frame =
      CGRectMake(0, _viewportMetrics.physical_view_inset_bottom, 0, 0);
  self.keyboardAnimationStartTime = fml::TimePoint().Now();
  self.originalViewInsetBottom = _viewportMetrics.physical_view_inset_bottom;

  // Invalidate old vsync client if old animation is not completed.
  [self invalidateKeyboardAnimationVSyncClient];

  __weak FlutterViewController* weakSelf = self;
  [self setUpKeyboardAnimationVsyncClient:^(fml::TimePoint targetTime) {
    [weakSelf handleKeyboardAnimationCallbackWithTargetTime:targetTime];
  }];
  VSyncClient* currentVsyncClient = _keyboardAnimationVSyncClient;

  [UIView animateWithDuration:duration
      animations:^{
        FlutterViewController* strongSelf = weakSelf;
        if (!strongSelf) {
          return;
        }

        // Set end value.
        strongSelf.keyboardAnimationView.frame = CGRectMake(0, self.targetViewInsetBottom, 0, 0);

        // Setup keyboard animation interpolation.
        CAAnimation* keyboardAnimation =
            [strongSelf.keyboardAnimationView.layer animationForKey:@"position"];
        [strongSelf setUpKeyboardSpringAnimationIfNeeded:keyboardAnimation];
      }
      completion:^(BOOL finished) {
        if (_keyboardAnimationVSyncClient == currentVsyncClient) {
          FlutterViewController* strongSelf = weakSelf;
          if (!strongSelf) {
            return;
          }

          // Indicates the vsync client captured by this block is the original one, which also
          // indicates the animation has not been interrupted from its beginning. Moreover,
          // indicates the animation is over and there is no more to execute.
          [strongSelf invalidateKeyboardAnimationVSyncClient];
          [strongSelf removeKeyboardAnimationView];
          [strongSelf ensureViewportMetricsIsCorrect];
        }
      }];
}

- (void)setUpKeyboardSpringAnimationIfNeeded:(CAAnimation*)keyboardAnimation {
  // If keyboard animation is null or not a spring animation, fallback to DisplayLink tracking.
  if (keyboardAnimation == nil || ![keyboardAnimation isKindOfClass:[CASpringAnimation class]]) {
    _keyboardSpringAnimation = nil;
    return;
  }

  // Setup keyboard spring animation details for spring curve animation calculation.
  CASpringAnimation* keyboardCASpringAnimation = (CASpringAnimation*)keyboardAnimation;
  _keyboardSpringAnimation =
      [[SpringAnimation alloc] initWithStiffness:keyboardCASpringAnimation.stiffness
                                         damping:keyboardCASpringAnimation.damping
                                            mass:keyboardCASpringAnimation.mass
                                 initialVelocity:keyboardCASpringAnimation.initialVelocity
                                       fromValue:self.originalViewInsetBottom
                                         toValue:self.targetViewInsetBottom];
}

- (void)handleKeyboardAnimationCallbackWithTargetTime:(fml::TimePoint)targetTime {
  // If the view controller's view is not loaded, bail out.
  if (!self.isViewLoaded) {
    return;
  }
  // If the view for tracking keyboard animation is nil, means it is not
  // created, bail out.
  if (!self.keyboardAnimationView) {
    return;
  }
  // If keyboardAnimationVSyncClient is nil, means the animation ends.
  // And should bail out.
  if (!self.keyboardAnimationVSyncClient) {
    return;
  }

  if (!self.keyboardAnimationView.superview) {
    // Ensure the keyboardAnimationView is in view hierarchy when animation running.
    [self.view addSubview:self.keyboardAnimationView];
  }

  if (!self.keyboardSpringAnimation) {
    if (self.keyboardAnimationView.layer.presentationLayer) {
      self->_viewportMetrics.physical_view_inset_bottom =
          self.keyboardAnimationView.layer.presentationLayer.frame.origin.y;
      [self updateViewportMetricsIfNeeded];
    }
  } else {
    fml::TimeDelta timeElapsed = targetTime - self.keyboardAnimationStartTime;
    self->_viewportMetrics.physical_view_inset_bottom =
        [self.keyboardSpringAnimation curveFunction:timeElapsed.ToSecondsF()];
    [self updateViewportMetricsIfNeeded];
  }
}

- (void)setUpKeyboardAnimationVsyncClient:
    (FlutterKeyboardAnimationCallback)keyboardAnimationCallback {
  if (!keyboardAnimationCallback) {
    return;
  }
  NSAssert(_keyboardAnimationVSyncClient == nil,
           @"_keyboardAnimationVSyncClient must be nil when setting up.");

  // Make sure the new viewport metrics get sent after the begin frame event has processed.
  FlutterKeyboardAnimationCallback animationCallback = [keyboardAnimationCallback copy];
  auto uiCallback = [animationCallback](std::unique_ptr<flutter::FrameTimingsRecorder> recorder) {
    fml::TimeDelta frameInterval = recorder->GetVsyncTargetTime() - recorder->GetVsyncStartTime();
    fml::TimePoint targetTime = recorder->GetVsyncTargetTime() + frameInterval;
    dispatch_async(dispatch_get_main_queue(), ^(void) {
      animationCallback(targetTime);
    });
  };

  _keyboardAnimationVSyncClient = [[VSyncClient alloc] initWithTaskRunner:self.engine.uiTaskRunner
                                                                 callback:uiCallback];
  _keyboardAnimationVSyncClient.allowPauseAfterVsync = NO;
  [_keyboardAnimationVSyncClient await];
}

- (void)invalidateKeyboardAnimationVSyncClient {
  [_keyboardAnimationVSyncClient invalidate];
  _keyboardAnimationVSyncClient = nil;
}

- (void)removeKeyboardAnimationView {
  if (self.keyboardAnimationView.superview != nil) {
    [self.keyboardAnimationView removeFromSuperview];
  }
}

- (void)ensureViewportMetricsIsCorrect {
  if (_viewportMetrics.physical_view_inset_bottom != self.targetViewInsetBottom) {
    // Make sure the `physical_view_inset_bottom` is the target value.
    _viewportMetrics.physical_view_inset_bottom = self.targetViewInsetBottom;
    [self updateViewportMetricsIfNeeded];
  }
}

- (void)handlePressEvent:(FlutterUIPressProxy*)press
              nextAction:(void (^)())next API_AVAILABLE(ios(13.4)) {
  if (@available(iOS 13.4, *)) {
  } else {
    next();
    return;
  }
  [self.keyboardManager handlePress:press nextAction:next];
}

- (void)sendDeepLinkToFramework:(NSURL*)url completionHandler:(void (^)(BOOL success))completion {
  __weak FlutterViewController* weakSelf = self;
  [self.engine
      waitForFirstFrame:3.0
               callback:^(BOOL didTimeout) {
                 if (didTimeout) {
                   FML_LOG(ERROR) << "Timeout waiting for the first frame when launching an URL.";
                   completion(NO);
                 } else {
                   // invove the method and get the result
                   [weakSelf.engine.navigationChannel
                       invokeMethod:@"pushRouteInformation"
                          arguments:@{
                            @"location" : url.absoluteString ?: [NSNull null],
                          }
                             result:^(id _Nullable result) {
                               BOOL success =
                                   [result isKindOfClass:[NSNumber class]] && [result boolValue];
                               if (!success) {
                                 // Logging the error if the result is not successful
                                 FML_LOG(ERROR) << "Failed to handle route information in Flutter.";
                               }
                               completion(success);
                             }];
                 }
               }];
}

// The documentation for presses* handlers (implemented below) is entirely
// unclear about how to handle the case where some, but not all, of the presses
// are handled here. I've elected to call super separately for each of the
// presses that aren't handled, but it's not clear if this is correct. It may be
// that iOS intends for us to either handle all or none of the presses, and pass
// the original set to super. I have not yet seen multiple presses in the set in
// the wild, however, so I suspect that the API is built for a tvOS remote or
// something, and perhaps only one ever appears in the set on iOS from a
// keyboard.
//
// We define separate superPresses* overrides to avoid implicitly capturing self in the blocks
// passed to the presses* methods below.

- (void)superPressesBegan:(NSSet<UIPress*>*)presses withEvent:(UIPressesEvent*)event {
  [super pressesBegan:presses withEvent:event];
}

- (void)superPressesChanged:(NSSet<UIPress*>*)presses withEvent:(UIPressesEvent*)event {
  [super pressesChanged:presses withEvent:event];
}

- (void)superPressesEnded:(NSSet<UIPress*>*)presses withEvent:(UIPressesEvent*)event {
  [super pressesEnded:presses withEvent:event];
}

- (void)superPressesCancelled:(NSSet<UIPress*>*)presses withEvent:(UIPressesEvent*)event {
  [super pressesCancelled:presses withEvent:event];
}

// If you substantially change these presses overrides, consider also changing
// the similar ones in FlutterTextInputPlugin. They need to be overridden in
// both places to capture keys both inside and outside of a text field, but have
// slightly different implementations.

- (void)pressesBegan:(NSSet<UIPress*>*)presses
           withEvent:(UIPressesEvent*)event API_AVAILABLE(ios(9.0)) {
  if (@available(iOS 13.4, *)) {
    __weak FlutterViewController* weakSelf = self;
    for (UIPress* press in presses) {
      [self handlePressEvent:[[FlutterUIPressProxy alloc] initWithPress:press withEvent:event]
                  nextAction:^() {
                    [weakSelf superPressesBegan:[NSSet setWithObject:press] withEvent:event];
                  }];
    }
  } else {
    [super pressesBegan:presses withEvent:event];
  }
}

- (void)pressesChanged:(NSSet<UIPress*>*)presses
             withEvent:(UIPressesEvent*)event API_AVAILABLE(ios(9.0)) {
  if (@available(iOS 13.4, *)) {
    __weak FlutterViewController* weakSelf = self;
    for (UIPress* press in presses) {
      [self handlePressEvent:[[FlutterUIPressProxy alloc] initWithPress:press withEvent:event]
                  nextAction:^() {
                    [weakSelf superPressesChanged:[NSSet setWithObject:press] withEvent:event];
                  }];
    }
  } else {
    [super pressesChanged:presses withEvent:event];
  }
}

- (void)pressesEnded:(NSSet<UIPress*>*)presses
           withEvent:(UIPressesEvent*)event API_AVAILABLE(ios(9.0)) {
  if (@available(iOS 13.4, *)) {
    __weak FlutterViewController* weakSelf = self;
    for (UIPress* press in presses) {
      [self handlePressEvent:[[FlutterUIPressProxy alloc] initWithPress:press withEvent:event]
                  nextAction:^() {
                    [weakSelf superPressesEnded:[NSSet setWithObject:press] withEvent:event];
                  }];
    }
  } else {
    [super pressesEnded:presses withEvent:event];
  }
}

- (void)pressesCancelled:(NSSet<UIPress*>*)presses
               withEvent:(UIPressesEvent*)event API_AVAILABLE(ios(9.0)) {
  if (@available(iOS 13.4, *)) {
    __weak FlutterViewController* weakSelf = self;
    for (UIPress* press in presses) {
      [self handlePressEvent:[[FlutterUIPressProxy alloc] initWithPress:press withEvent:event]
                  nextAction:^() {
                    [weakSelf superPressesCancelled:[NSSet setWithObject:press] withEvent:event];
                  }];
    }
  } else {
    [super pressesCancelled:presses withEvent:event];
  }
}

#pragma mark - Orientation updates

- (void)onOrientationPreferencesUpdated:(NSNotification*)notification {
  // Notifications may not be on the iOS UI thread
  __weak FlutterViewController* weakSelf = self;
  dispatch_async(dispatch_get_main_queue(), ^{
    NSDictionary* info = notification.userInfo;
    NSNumber* update = info[@(flutter::kOrientationUpdateNotificationKey)];
    if (update == nil) {
      return;
    }
    [weakSelf performOrientationUpdate:update.unsignedIntegerValue];
  });
}

- (void)requestGeometryUpdateForWindowScenes:(NSSet<UIScene*>*)windowScenes
    API_AVAILABLE(ios(16.0)) {
  for (UIScene* windowScene in windowScenes) {
    FML_DCHECK([windowScene isKindOfClass:[UIWindowScene class]]);
    UIWindowSceneGeometryPreferencesIOS* preference = [[UIWindowSceneGeometryPreferencesIOS alloc]
        initWithInterfaceOrientations:self.orientationPreferences];
    [(UIWindowScene*)windowScene
        requestGeometryUpdateWithPreferences:preference
                                errorHandler:^(NSError* error) {
                                  os_log_error(OS_LOG_DEFAULT,
                                               "Failed to change device orientation: %@", error);
                                }];
    [self setNeedsUpdateOfSupportedInterfaceOrientations];
  }
}

- (void)performOrientationUpdate:(UIInterfaceOrientationMask)new_preferences {
  if (new_preferences != self.orientationPreferences) {
    self.orientationPreferences = new_preferences;

    if (@available(iOS 16.0, *)) {
      NSSet<UIScene*>* scenes =
#if APPLICATION_EXTENSION_API_ONLY
          self.flutterWindowSceneIfViewLoaded
              ? [NSSet setWithObject:self.flutterWindowSceneIfViewLoaded]
              : [NSSet set];
#else
          [UIApplication.sharedApplication.connectedScenes
              filteredSetUsingPredicate:[NSPredicate predicateWithBlock:^BOOL(
                                                         id scene, NSDictionary* bindings) {
                return [scene isKindOfClass:[UIWindowScene class]];
              }]];
#endif
      [self requestGeometryUpdateForWindowScenes:scenes];
    } else {
      UIInterfaceOrientationMask currentInterfaceOrientation = 0;
      if (@available(iOS 13.0, *)) {
        UIWindowScene* windowScene = self.flutterWindowSceneIfViewLoaded;
        if (!windowScene) {
          FML_LOG(WARNING)
              << "Accessing the interface orientation when the window scene is unavailable.";
          return;
        }
        currentInterfaceOrientation = 1 << windowScene.interfaceOrientation;
      } else {
#if APPLICATION_EXTENSION_API_ONLY
        FML_LOG(ERROR) << "Application based status bar orentiation update is not supported in "
                          "app extension. Orientation: "
                       << currentInterfaceOrientation;
#else
        currentInterfaceOrientation = 1 << [[UIApplication sharedApplication] statusBarOrientation];
#endif
      }
      if (!(self.orientationPreferences & currentInterfaceOrientation)) {
        [UIViewController attemptRotationToDeviceOrientation];
        // Force orientation switch if the current orientation is not allowed
        if (self.orientationPreferences & UIInterfaceOrientationMaskPortrait) {
          // This is no official API but more like a workaround / hack (using
          // key-value coding on a read-only property). This might break in
          // the future, but currently it´s the only way to force an orientation change
          [[UIDevice currentDevice] setValue:@(UIInterfaceOrientationPortrait)
                                      forKey:@"orientation"];
        } else if (self.orientationPreferences & UIInterfaceOrientationMaskPortraitUpsideDown) {
          [[UIDevice currentDevice] setValue:@(UIInterfaceOrientationPortraitUpsideDown)
                                      forKey:@"orientation"];
        } else if (self.orientationPreferences & UIInterfaceOrientationMaskLandscapeLeft) {
          [[UIDevice currentDevice] setValue:@(UIInterfaceOrientationLandscapeLeft)
                                      forKey:@"orientation"];
        } else if (self.orientationPreferences & UIInterfaceOrientationMaskLandscapeRight) {
          [[UIDevice currentDevice] setValue:@(UIInterfaceOrientationLandscapeRight)
                                      forKey:@"orientation"];
        }
      }
    }
  }
}

- (void)onHideHomeIndicatorNotification:(NSNotification*)notification {
  self.isHomeIndicatorHidden = YES;
}

- (void)onShowHomeIndicatorNotification:(NSNotification*)notification {
  self.isHomeIndicatorHidden = NO;
}

- (void)setIsHomeIndicatorHidden:(BOOL)hideHomeIndicator {
  if (hideHomeIndicator != _isHomeIndicatorHidden) {
    _isHomeIndicatorHidden = hideHomeIndicator;
    [self setNeedsUpdateOfHomeIndicatorAutoHidden];
  }
}

- (BOOL)prefersHomeIndicatorAutoHidden {
  return self.isHomeIndicatorHidden;
}

- (BOOL)shouldAutorotate {
  return YES;
}

- (NSUInteger)supportedInterfaceOrientations {
  return self.orientationPreferences;
}

#pragma mark - Accessibility

- (void)onAccessibilityStatusChanged:(NSNotification*)notification {
  if (!self.engine) {
    return;
  }
  BOOL enabled = NO;
  int32_t flags = self.accessibilityFlags;
#if TARGET_OS_SIMULATOR
  // There doesn't appear to be any way to determine whether the accessibility
  // inspector is enabled on the simulator. We conservatively always turn on the
  // accessibility bridge in the simulator, but never assistive technology.
  enabled = YES;
#else
  _isVoiceOverRunning = UIAccessibilityIsVoiceOverRunning();
  enabled = _isVoiceOverRunning || UIAccessibilityIsSwitchControlRunning();
  if (enabled) {
    flags |= static_cast<int32_t>(flutter::AccessibilityFeatureFlag::kAccessibleNavigation);
  }
  enabled |= UIAccessibilityIsSpeakScreenEnabled();
#endif
  [self.engine enableSemantics:enabled withFlags:flags];
}

- (int32_t)accessibilityFlags {
  int32_t flags = 0;
  if (UIAccessibilityIsInvertColorsEnabled()) {
    flags |= static_cast<int32_t>(flutter::AccessibilityFeatureFlag::kInvertColors);
  }
  if (UIAccessibilityIsReduceMotionEnabled()) {
    flags |= static_cast<int32_t>(flutter::AccessibilityFeatureFlag::kReduceMotion);
  }
  if (UIAccessibilityIsBoldTextEnabled()) {
    flags |= static_cast<int32_t>(flutter::AccessibilityFeatureFlag::kBoldText);
  }
  if (UIAccessibilityDarkerSystemColorsEnabled()) {
    flags |= static_cast<int32_t>(flutter::AccessibilityFeatureFlag::kHighContrast);
  }
  if ([FlutterViewController accessibilityIsOnOffSwitchLabelsEnabled]) {
    flags |= static_cast<int32_t>(flutter::AccessibilityFeatureFlag::kOnOffSwitchLabels);
  }

  return flags;
}

- (BOOL)accessibilityPerformEscape {
  FlutterMethodChannel* navigationChannel = self.engine.navigationChannel;
  if (navigationChannel) {
    [self popRoute];
    return YES;
  }
  return NO;
}

+ (BOOL)accessibilityIsOnOffSwitchLabelsEnabled {
  if (@available(iOS 13, *)) {
    return UIAccessibilityIsOnOffSwitchLabelsEnabled();
  } else {
    return NO;
  }
}

#pragma mark - Set user settings

- (void)traitCollectionDidChange:(UITraitCollection*)previousTraitCollection {
  [super traitCollectionDidChange:previousTraitCollection];
  [self onUserSettingsChanged:nil];
}

- (void)onUserSettingsChanged:(NSNotification*)notification {
  [self.engine.settingsChannel sendMessage:@{
    @"textScaleFactor" : @(self.textScaleFactor),
    @"alwaysUse24HourFormat" : @(FlutterHourFormat.isAlwaysUse24HourFormat),
    @"platformBrightness" : self.brightnessMode,
    @"platformContrast" : self.contrastMode,
    @"nativeSpellCheckServiceDefined" : @YES,
    @"supportsShowingSystemContextMenu" : @(self.supportsShowingSystemContextMenu)
  }];
}

- (CGFloat)textScaleFactor {
#if APPLICATION_EXTENSION_API_ONLY
  FML_LOG(WARNING) << "Dynamic content size update is not supported in app extension.";
  return 1.0;
#else
  UIContentSizeCategory category = [UIApplication sharedApplication].preferredContentSizeCategory;
  // The delta is computed by approximating Apple's typography guidelines:
  // https://developer.apple.com/ios/human-interface-guidelines/visual-design/typography/
  //
  // Specifically:
  // Non-accessibility sizes for "body" text are:
  const CGFloat xs = 14;
  const CGFloat s = 15;
  const CGFloat m = 16;
  const CGFloat l = 17;
  const CGFloat xl = 19;
  const CGFloat xxl = 21;
  const CGFloat xxxl = 23;

  // Accessibility sizes for "body" text are:
  const CGFloat ax1 = 28;
  const CGFloat ax2 = 33;
  const CGFloat ax3 = 40;
  const CGFloat ax4 = 47;
  const CGFloat ax5 = 53;

  // We compute the scale as relative difference from size L (large, the default size), where
  // L is assumed to have scale 1.0.
  if ([category isEqualToString:UIContentSizeCategoryExtraSmall]) {
    return xs / l;
  } else if ([category isEqualToString:UIContentSizeCategorySmall]) {
    return s / l;
  } else if ([category isEqualToString:UIContentSizeCategoryMedium]) {
    return m / l;
  } else if ([category isEqualToString:UIContentSizeCategoryLarge]) {
    return 1.0;
  } else if ([category isEqualToString:UIContentSizeCategoryExtraLarge]) {
    return xl / l;
  } else if ([category isEqualToString:UIContentSizeCategoryExtraExtraLarge]) {
    return xxl / l;
  } else if ([category isEqualToString:UIContentSizeCategoryExtraExtraExtraLarge]) {
    return xxxl / l;
  } else if ([category isEqualToString:UIContentSizeCategoryAccessibilityMedium]) {
    return ax1 / l;
  } else if ([category isEqualToString:UIContentSizeCategoryAccessibilityLarge]) {
    return ax2 / l;
  } else if ([category isEqualToString:UIContentSizeCategoryAccessibilityExtraLarge]) {
    return ax3 / l;
  } else if ([category isEqualToString:UIContentSizeCategoryAccessibilityExtraExtraLarge]) {
    return ax4 / l;
  } else if ([category isEqualToString:UIContentSizeCategoryAccessibilityExtraExtraExtraLarge]) {
    return ax5 / l;
  } else {
    return 1.0;
  }
#endif
}

- (BOOL)supportsShowingSystemContextMenu {
  if (@available(iOS 16.0, *)) {
    return YES;
  } else {
    return NO;
  }
}

// The brightness mode of the platform, e.g., light or dark, expressed as a string that
// is understood by the Flutter framework. See the settings
// system channel for more information.
- (NSString*)brightnessMode {
  if (@available(iOS 13, *)) {
    UIUserInterfaceStyle style = self.traitCollection.userInterfaceStyle;

    if (style == UIUserInterfaceStyleDark) {
      return @"dark";
    } else {
      return @"light";
    }
  } else {
    return @"light";
  }
}

// The contrast mode of the platform, e.g., normal or high, expressed as a string that is
// understood by the Flutter framework. See the settings system channel for more
// information.
- (NSString*)contrastMode {
  if (@available(iOS 13, *)) {
    UIAccessibilityContrast contrast = self.traitCollection.accessibilityContrast;

    if (contrast == UIAccessibilityContrastHigh) {
      return @"high";
    } else {
      return @"normal";
    }
  } else {
    return @"normal";
  }
}

#pragma mark - Status bar style

- (UIStatusBarStyle)preferredStatusBarStyle {
  return self.statusBarStyle;
}

- (void)onPreferredStatusBarStyleUpdated:(NSNotification*)notification {
  // Notifications may not be on the iOS UI thread
  __weak FlutterViewController* weakSelf = self;
  dispatch_async(dispatch_get_main_queue(), ^{
    FlutterViewController* strongSelf = weakSelf;
    if (!strongSelf) {
      return;
    }

    NSDictionary* info = notification.userInfo;
    NSNumber* update = info[@(flutter::kOverlayStyleUpdateNotificationKey)];
    if (update == nil) {
      return;
    }

    UIStatusBarStyle style = static_cast<UIStatusBarStyle>(update.integerValue);
    if (style != strongSelf.statusBarStyle) {
      strongSelf.statusBarStyle = style;
      [strongSelf setNeedsStatusBarAppearanceUpdate];
    }
  });
}

- (void)setPrefersStatusBarHidden:(BOOL)hidden {
  if (hidden != self.flutterPrefersStatusBarHidden) {
    self.flutterPrefersStatusBarHidden = hidden;
    [self setNeedsStatusBarAppearanceUpdate];
  }
}

- (BOOL)prefersStatusBarHidden {
  return self.flutterPrefersStatusBarHidden;
}

#pragma mark - Platform views

- (FlutterPlatformViewsController*)platformViewsController {
  return self.engine.platformViewsController;
}

- (NSObject<FlutterBinaryMessenger>*)binaryMessenger {
  return self.engine.binaryMessenger;
}

#pragma mark - FlutterBinaryMessenger

- (void)sendOnChannel:(NSString*)channel message:(NSData*)message {
  [self.engine.binaryMessenger sendOnChannel:channel message:message];
}

- (void)sendOnChannel:(NSString*)channel
              message:(NSData*)message
          binaryReply:(FlutterBinaryReply)callback {
  NSAssert(channel, @"The channel must not be null");
  [self.engine.binaryMessenger sendOnChannel:channel message:message binaryReply:callback];
}

- (NSObject<FlutterTaskQueue>*)makeBackgroundTaskQueue {
  return [self.engine.binaryMessenger makeBackgroundTaskQueue];
}

- (FlutterBinaryMessengerConnection)setMessageHandlerOnChannel:(NSString*)channel
                                          binaryMessageHandler:
                                              (FlutterBinaryMessageHandler)handler {
  return [self setMessageHandlerOnChannel:channel binaryMessageHandler:handler taskQueue:nil];
}

- (FlutterBinaryMessengerConnection)
    setMessageHandlerOnChannel:(NSString*)channel
          binaryMessageHandler:(FlutterBinaryMessageHandler _Nullable)handler
                     taskQueue:(NSObject<FlutterTaskQueue>* _Nullable)taskQueue {
  NSAssert(channel, @"The channel must not be null");
  return [self.engine.binaryMessenger setMessageHandlerOnChannel:channel
                                            binaryMessageHandler:handler
                                                       taskQueue:taskQueue];
}

- (void)cleanUpConnection:(FlutterBinaryMessengerConnection)connection {
  [self.engine.binaryMessenger cleanUpConnection:connection];
}

#pragma mark - FlutterTextureRegistry

- (int64_t)registerTexture:(NSObject<FlutterTexture>*)texture {
  return [self.engine.textureRegistry registerTexture:texture];
}

- (void)unregisterTexture:(int64_t)textureId {
  [self.engine.textureRegistry unregisterTexture:textureId];
}

- (void)textureFrameAvailable:(int64_t)textureId {
  [self.engine.textureRegistry textureFrameAvailable:textureId];
}

- (NSString*)lookupKeyForAsset:(NSString*)asset {
  return [FlutterDartProject lookupKeyForAsset:asset];
}

- (NSString*)lookupKeyForAsset:(NSString*)asset fromPackage:(NSString*)package {
  return [FlutterDartProject lookupKeyForAsset:asset fromPackage:package];
}

- (id<FlutterPluginRegistry>)pluginRegistry {
  return self.engine;
}

+ (BOOL)isUIAccessibilityIsVoiceOverRunning {
  return UIAccessibilityIsVoiceOverRunning();
}

#pragma mark - FlutterPluginRegistry

- (NSObject<FlutterPluginRegistrar>*)registrarForPlugin:(NSString*)pluginKey {
  return [self.engine registrarForPlugin:pluginKey];
}

- (BOOL)hasPlugin:(NSString*)pluginKey {
  return [self.engine hasPlugin:pluginKey];
}

- (NSObject*)valuePublishedByPlugin:(NSString*)pluginKey {
  return [self.engine valuePublishedByPlugin:pluginKey];
}

- (void)presentViewController:(UIViewController*)viewControllerToPresent
                     animated:(BOOL)flag
                   completion:(void (^)(void))completion {
  self.isPresentingViewControllerAnimating = YES;
  __weak FlutterViewController* weakSelf = self;
  [super presentViewController:viewControllerToPresent
                      animated:flag
                    completion:^{
                      weakSelf.isPresentingViewControllerAnimating = NO;
                      if (completion) {
                        completion();
                      }
                    }];
}

- (BOOL)isPresentingViewController {
  return self.presentedViewController != nil || self.isPresentingViewControllerAnimating;
}

- (flutter::PointerData)updateMousePointerDataFrom:(UIGestureRecognizer*)gestureRecognizer
    API_AVAILABLE(ios(13.4)) {
  CGPoint location = [gestureRecognizer locationInView:self.view];
  CGFloat scale = self.flutterScreenIfViewLoaded.scale;
  _mouseState.location = {location.x * scale, location.y * scale};
  flutter::PointerData pointer_data;
  pointer_data.Clear();
  pointer_data.time_stamp = [[NSProcessInfo processInfo] systemUptime] * kMicrosecondsPerSecond;
  pointer_data.physical_x = _mouseState.location.x;
  pointer_data.physical_y = _mouseState.location.y;
  return pointer_data;
}

- (BOOL)gestureRecognizer:(UIGestureRecognizer*)gestureRecognizer
    shouldRecognizeSimultaneouslyWithGestureRecognizer:(UIGestureRecognizer*)otherGestureRecognizer
    API_AVAILABLE(ios(13.4)) {
  return YES;
}

- (BOOL)gestureRecognizer:(UIGestureRecognizer*)gestureRecognizer
       shouldReceiveEvent:(UIEvent*)event API_AVAILABLE(ios(13.4)) {
  if (gestureRecognizer == _continuousScrollingPanGestureRecognizer &&
      event.type == UIEventTypeScroll) {
    // Events with type UIEventTypeScroll are only received when running on macOS under emulation.
    flutter::PointerData pointer_data = [self updateMousePointerDataFrom:gestureRecognizer];
    pointer_data.device = reinterpret_cast<int64_t>(_continuousScrollingPanGestureRecognizer);
    pointer_data.kind = flutter::PointerData::DeviceKind::kTrackpad;
    pointer_data.signal_kind = flutter::PointerData::SignalKind::kScrollInertiaCancel;
    pointer_data.view_id = self.viewIdentifier;

    if (event.timestamp < self.scrollInertiaEventAppKitDeadline) {
      // Only send the event if it occured before the expected natural end of gesture momentum.
      // If received after the deadline, it's not likely the event is from a user-initiated cancel.
      auto packet = std::make_unique<flutter::PointerDataPacket>(1);
      packet->SetPointerData(/*i=*/0, pointer_data);
      [self.engine dispatchPointerDataPacket:std::move(packet)];
      self.scrollInertiaEventAppKitDeadline = 0;
    }
  }
  // This method is also called for UITouches, should return YES to process all touches.
  return YES;
}

- (void)hoverEvent:(UIHoverGestureRecognizer*)recognizer API_AVAILABLE(ios(13.4)) {
  CGPoint oldLocation = _mouseState.location;

  flutter::PointerData pointer_data = [self updateMousePointerDataFrom:recognizer];
  pointer_data.device = reinterpret_cast<int64_t>(recognizer);
  pointer_data.kind = flutter::PointerData::DeviceKind::kMouse;
  pointer_data.view_id = self.viewIdentifier;

  switch (_hoverGestureRecognizer.state) {
    case UIGestureRecognizerStateBegan:
      pointer_data.change = flutter::PointerData::Change::kAdd;
      break;
    case UIGestureRecognizerStateChanged:
      pointer_data.change = flutter::PointerData::Change::kHover;
      break;
    case UIGestureRecognizerStateEnded:
    case UIGestureRecognizerStateCancelled:
      pointer_data.change = flutter::PointerData::Change::kRemove;
      break;
    default:
      // Sending kHover is the least harmful thing to do here
      // But this state is not expected to ever be reached.
      pointer_data.change = flutter::PointerData::Change::kHover;
      break;
  }

  NSTimeInterval time = [NSProcessInfo processInfo].systemUptime;
  BOOL isRunningOnMac = NO;
  if (@available(iOS 14.0, *)) {
    // This "stationary pointer" heuristic is not reliable when running within macOS.
    // We instead receive a scroll cancel event directly from AppKit.
    // See gestureRecognizer:shouldReceiveEvent:
    isRunningOnMac = [NSProcessInfo processInfo].iOSAppOnMac;
  }
  if (!isRunningOnMac && CGPointEqualToPoint(oldLocation, _mouseState.location) &&
      time > self.scrollInertiaEventStartline) {
    // iPadOS reports trackpad movements events with high (sub-pixel) precision. When an event
    // is received with the same position as the previous one, it can only be from a finger
    // making or breaking contact with the trackpad surface.
    auto packet = std::make_unique<flutter::PointerDataPacket>(2);
    packet->SetPointerData(/*i=*/0, pointer_data);
    flutter::PointerData inertia_cancel = pointer_data;
    inertia_cancel.device = reinterpret_cast<int64_t>(_continuousScrollingPanGestureRecognizer);
    inertia_cancel.kind = flutter::PointerData::DeviceKind::kTrackpad;
    inertia_cancel.signal_kind = flutter::PointerData::SignalKind::kScrollInertiaCancel;
    inertia_cancel.view_id = self.viewIdentifier;
    packet->SetPointerData(/*i=*/1, inertia_cancel);
    [self.engine dispatchPointerDataPacket:std::move(packet)];
    self.scrollInertiaEventStartline = DBL_MAX;
  } else {
    auto packet = std::make_unique<flutter::PointerDataPacket>(1);
    packet->SetPointerData(/*i=*/0, pointer_data);
    [self.engine dispatchPointerDataPacket:std::move(packet)];
  }
}

- (void)discreteScrollEvent:(UIPanGestureRecognizer*)recognizer API_AVAILABLE(ios(13.4)) {
  CGPoint translation = [recognizer translationInView:self.view];
  const CGFloat scale = self.flutterScreenIfViewLoaded.scale;

  translation.x *= scale;
  translation.y *= scale;

  flutter::PointerData pointer_data = [self updateMousePointerDataFrom:recognizer];
  pointer_data.device = reinterpret_cast<int64_t>(recognizer);
  pointer_data.kind = flutter::PointerData::DeviceKind::kMouse;
  pointer_data.signal_kind = flutter::PointerData::SignalKind::kScroll;
  pointer_data.scroll_delta_x = (translation.x - _mouseState.last_translation.x);
  pointer_data.scroll_delta_y = -(translation.y - _mouseState.last_translation.y);
  pointer_data.view_id = self.viewIdentifier;

  // The translation reported by UIPanGestureRecognizer is the total translation
  // generated by the pan gesture since the gesture began. We need to be able
  // to keep track of the last translation value in order to generate the deltaX
  // and deltaY coordinates for each subsequent scroll event.
  if (recognizer.state != UIGestureRecognizerStateEnded) {
    _mouseState.last_translation = translation;
  } else {
    _mouseState.last_translation = CGPointZero;
  }

  auto packet = std::make_unique<flutter::PointerDataPacket>(1);
  packet->SetPointerData(/*i=*/0, pointer_data);
  [self.engine dispatchPointerDataPacket:std::move(packet)];
}

- (void)continuousScrollEvent:(UIPanGestureRecognizer*)recognizer API_AVAILABLE(ios(13.4)) {
  CGPoint translation = [recognizer translationInView:self.view];
  const CGFloat scale = self.flutterScreenIfViewLoaded.scale;

  flutter::PointerData pointer_data = [self updateMousePointerDataFrom:recognizer];
  pointer_data.device = reinterpret_cast<int64_t>(recognizer);
  pointer_data.kind = flutter::PointerData::DeviceKind::kTrackpad;
  pointer_data.view_id = self.viewIdentifier;
  switch (recognizer.state) {
    case UIGestureRecognizerStateBegan:
      pointer_data.change = flutter::PointerData::Change::kPanZoomStart;
      break;
    case UIGestureRecognizerStateChanged:
      pointer_data.change = flutter::PointerData::Change::kPanZoomUpdate;
      pointer_data.pan_x = translation.x * scale;
      pointer_data.pan_y = translation.y * scale;
      pointer_data.pan_delta_x = 0;  // Delta will be generated in pointer_data_packet_converter.cc.
      pointer_data.pan_delta_y = 0;  // Delta will be generated in pointer_data_packet_converter.cc.
      pointer_data.scale = 1;
      break;
    case UIGestureRecognizerStateEnded:
    case UIGestureRecognizerStateCancelled:
      self.scrollInertiaEventStartline =
          [[NSProcessInfo processInfo] systemUptime] +
          0.1;  // Time to lift fingers off trackpad (experimentally determined)
      // When running an iOS app on an Apple Silicon Mac, AppKit will send an event
      // of type UIEventTypeScroll when trackpad scroll momentum has ended. This event
      // is sent whether the momentum ended normally or was cancelled by a trackpad touch.
      // Since Flutter scrolling inertia will likely not match the system inertia, we should
      // only send a PointerScrollInertiaCancel event for user-initiated cancellations.
      // The following (curve-fitted) calculation provides a cutoff point after which any
      // UIEventTypeScroll event will likely be from the system instead of the user.
      // See https://github.com/flutter/engine/pull/34929.
      self.scrollInertiaEventAppKitDeadline =
          [[NSProcessInfo processInfo] systemUptime] +
          (0.1821 * log(fmax([recognizer velocityInView:self.view].x,
                             [recognizer velocityInView:self.view].y))) -
          0.4825;
      pointer_data.change = flutter::PointerData::Change::kPanZoomEnd;
      break;
    default:
      // continuousScrollEvent: should only ever be triggered with the above phases
      NSAssert(NO, @"Trackpad pan event occured with unexpected phase 0x%lx",
               (long)recognizer.state);
      break;
  }

  auto packet = std::make_unique<flutter::PointerDataPacket>(1);
  packet->SetPointerData(/*i=*/0, pointer_data);
  [self.engine dispatchPointerDataPacket:std::move(packet)];
}

- (void)pinchEvent:(UIPinchGestureRecognizer*)recognizer API_AVAILABLE(ios(13.4)) {
  flutter::PointerData pointer_data = [self updateMousePointerDataFrom:recognizer];
  pointer_data.device = reinterpret_cast<int64_t>(recognizer);
  pointer_data.kind = flutter::PointerData::DeviceKind::kTrackpad;
  pointer_data.view_id = self.viewIdentifier;
  switch (recognizer.state) {
    case UIGestureRecognizerStateBegan:
      pointer_data.change = flutter::PointerData::Change::kPanZoomStart;
      break;
    case UIGestureRecognizerStateChanged:
      pointer_data.change = flutter::PointerData::Change::kPanZoomUpdate;
      pointer_data.scale = recognizer.scale;
      pointer_data.rotation = _rotationGestureRecognizer.rotation;
      break;
    case UIGestureRecognizerStateEnded:
    case UIGestureRecognizerStateCancelled:
      pointer_data.change = flutter::PointerData::Change::kPanZoomEnd;
      break;
    default:
      // pinchEvent: should only ever be triggered with the above phases
      NSAssert(NO, @"Trackpad pinch event occured with unexpected phase 0x%lx",
               (long)recognizer.state);
      break;
  }

  auto packet = std::make_unique<flutter::PointerDataPacket>(1);
  packet->SetPointerData(/*i=*/0, pointer_data);
  [self.engine dispatchPointerDataPacket:std::move(packet)];
}

#pragma mark - State Restoration

- (void)encodeRestorableStateWithCoder:(NSCoder*)coder {
  NSData* restorationData = [self.engine.restorationPlugin restorationData];
  [coder encodeBytes:(const unsigned char*)restorationData.bytes
              length:restorationData.length
              forKey:kFlutterRestorationStateAppData];
  [super encodeRestorableStateWithCoder:coder];
}

- (void)decodeRestorableStateWithCoder:(NSCoder*)coder {
  NSUInteger restorationDataLength;
  const unsigned char* restorationBytes = [coder decodeBytesForKey:kFlutterRestorationStateAppData
                                                    returnedLength:&restorationDataLength];
  NSData* restorationData = [NSData dataWithBytes:restorationBytes length:restorationDataLength];
  [self.engine.restorationPlugin setRestorationData:restorationData];
}

- (FlutterRestorationPlugin*)restorationPlugin {
  return self.engine.restorationPlugin;
}

@end
