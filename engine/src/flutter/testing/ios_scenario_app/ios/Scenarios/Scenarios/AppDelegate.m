// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "AppDelegate.h"

#import "ContinuousTexture.h"
#import "FlutterEngine+ScenariosTest.h"
#import "ScreenBeforeFlutter.h"
#import "TextPlatformView.h"

// A UIViewController that sets YES for its preferedStatusBarHidden property.
// StatusBar includes current time, which is non-deterministic. This ViewController
// removes the StatusBar to make the screenshot deterministic.
@interface NoStatusBarViewController : UIViewController

@end

@implementation NoStatusBarViewController
- (BOOL)prefersStatusBarHidden {
  return YES;
}
@end

// The FlutterViewController version of NoStatusBarViewController
@interface NoStatusBarFlutterViewController : FlutterViewController

@end

@implementation NoStatusBarFlutterViewController
- (BOOL)prefersStatusBarHidden {
  return YES;
}
@end

@implementation AppDelegate

- (BOOL)application:(UIApplication*)application
    didFinishLaunchingWithOptions:(NSDictionary*)launchOptions {
  NSArray<NSString*>* processArguments = NSProcessInfo.processInfo.arguments;
  if ([processArguments containsObject:@"--enable-software-rendering"]) {
    @throw @"--enable-software-rendering is unsupported in iOS scenario tests";
  }

  self.window = [[UIWindow alloc] initWithFrame:[[UIScreen mainScreen] bounds]];
  if ([processArguments containsObject:@"--maskview-blocking"]) {
    self.window.tintColor = UIColor.systemPinkColor;
  }
  NSDictionary<NSString*, NSString*>* launchArgsMap = @{
    // The golden test args should match `GoldenTestManager`.
    @"--locale-initialization" : @"locale_initialization",
    @"--platform-view" : @"platform_view",
    @"--platform-view-no-overlay-intersection" : @"platform_view_no_overlay_intersection",
    @"--platform-view-two-intersecting-overlays" : @"platform_view_two_intersecting_overlays",
    @"--platform-view-partial-intersection" : @"platform_view_partial_intersection",
    @"--platform-view-one-overlay-two-intersecting-overlays" :
        @"platform_view_one_overlay_two_intersecting_overlays",
    @"--platform-view-multiple-without-overlays" : @"platform_view_multiple_without_overlays",
    @"--platform-view-max-overlays" : @"platform_view_max_overlays",
    @"--platform-view-surrounding-layers-fractional-coordinate" :
        @"platform_view_surrounding_layers_fractional_coordinate",
    @"--platform-view-partial-intersection-fractional-coordinate" :
        @"platform_view_partial_intersection_fractional_coordinate",
    @"--platform-view-multiple" : @"platform_view_multiple",
    @"--platform-view-multiple-background-foreground" :
        @"platform_view_multiple_background_foreground",
    @"--platform-view-cliprect" : @"platform_view_cliprect",
    @"--platform-view-cliprect-multiple-clips" : @"platform_view_cliprect_multiple_clips",
    @"--platform-view-cliprrect" : @"platform_view_cliprrect",
    @"--platform-view-cliprrect-multiple-clips" : @"platform_view_cliprrect_multiple_clips",
    @"--platform-view-large-cliprrect" : @"platform_view_large_cliprrect",
    @"--platform-view-large-cliprrect-multiple-clips" :
        @"platform_view_large_cliprrect_multiple_clips",
    @"--platform-view-clippath" : @"platform_view_clippath",
    @"--platform-view-clippath-multiple-clips" : @"platform_view_clippath_multiple_clips",
    @"--platform-view-cliprrect-with-transform" : @"platform_view_cliprrect_with_transform",
    @"--platform-view-cliprrect-with-transform-multiple-clips" :
        @"platform_view_cliprrect_with_transform_multiple_clips",
    @"--platform-view-large-cliprrect-with-transform" :
        @"platform_view_large_cliprrect_with_transform",
    @"--platform-view-large-cliprrect-with-transform-multiple-clips" :
        @"platform_view_large_cliprrect_with_transform_multiple_clips",
    @"--platform-view-cliprect-with-transform" : @"platform_view_cliprect_with_transform",
    @"--platform-view-cliprect-with-transform-multiple-clips" :
        @"platform_view_cliprect_with_transform_multiple_clips",
    @"--platform-view-clippath-with-transform" : @"platform_view_clippath_with_transform",
    @"--platform-view-clippath-with-transform-multiple-clips" :
        @"platform_view_clippath_with_transform_multiple_clips",
    @"--platform-view-transform" : @"platform_view_transform",
    @"--platform-view-opacity" : @"platform_view_opacity",
    @"--platform-view-with-other-backdrop-filter" : @"platform_view_with_other_backdrop_filter",
    @"--two-platform-views-with-other-backdrop-filter" :
        @"two_platform_views_with_other_backdrop_filter",
    @"--platform-view-with-negative-backdrop-filter" :
        @"platform_view_with_negative_backdrop_filter",
    @"--platform-view-rotate" : @"platform_view_rotate",
    @"--non-full-screen-flutter-view-platform-view" : @"non_full_screen_flutter_view_platform_view",
    @"--gesture-reject-after-touches-ended" : @"platform_view_gesture_reject_after_touches_ended",
    @"--gesture-reject-eager" : @"platform_view_gesture_reject_eager",
    @"--gesture-accept" : @"platform_view_gesture_accept",
    @"--gesture-accept-with-overlapping-platform-views" :
        @"platform_view_gesture_accept_with_overlapping_platform_views",
    @"--tap-status-bar" : @"tap_status_bar",
    @"--animated-color-square" : @"animated_color_square",
    @"--solid-blue" : @"solid_blue",
    @"--platform-view-with-continuous-texture" : @"platform_view_with_continuous_texture",
    @"--bogus-font-text" : @"bogus_font_text",
    @"--spawn-engine-works" : @"spawn_engine_works",
    @"--pointer-events" : @"pointer_events",
    @"--platform-view-scrolling-under-widget" : @"platform_view_scrolling_under_widget",
    @"--platform-views-with-clips-scrolling" : @"platform_views_with_clips_scrolling",
    @"--platform-views-with-clips-scrolling-multiple-clips" :
        @"platform_views_with_clips_scrolling_multiple_clips",
    @"--platform-view-cliprect-after-moved" : @"platform_view_cliprect_after_moved",
    @"--platform-view-cliprect-after-moved-multiple-clips" :
        @"platform_view_cliprect_after_moved_multiple_clips",
    @"--two-platform-view-clip-rect" : @"two_platform_view_clip_rect",
    @"--two-platform-view-clip-rect-multiple-clips" : @"two_platform_view_clip_rect_multiple_clips",
    @"--two-platform-view-clip-rrect" : @"two_platform_view_clip_rrect",
    @"--two-platform-view-clip-rrect-multiple-clips" :
        @"two_platform_view_clip_rrect_multiple_clips",
    @"--two-platform-view-clip-path" : @"two_platform_view_clip_path",
    @"--two-platform-view-clip-path-multiple-clips" : @"two_platform_view_clip_path_multiple_clips",
    @"--darwin-system-font" : @"darwin_system_font",
  };
  __block NSString* flutterViewControllerTestName = nil;
  [launchArgsMap
      enumerateKeysAndObjectsUsingBlock:^(NSString* argument, NSString* testName, BOOL* stop) {
        if ([processArguments containsObject:argument]) {
          flutterViewControllerTestName = testName;
          *stop = YES;
        }
      }];
  if (flutterViewControllerTestName) {
    [self setupFlutterViewControllerTest:flutterViewControllerTestName];
  } else if ([processArguments containsObject:@"--screen-before-flutter"]) {
    self.window.rootViewController = [[ScreenBeforeFlutter alloc] initWithEngineRunCompletion:nil];
  } else {
    self.window.rootViewController = [[UIViewController alloc] init];
  }

  [self.window makeKeyAndVisible];
  if ([processArguments containsObject:@"--with-continuous-texture"]) {
    [ContinuousTexture
        registerWithRegistrar:[self registrarForPlugin:@"com.constant.firing.texture"]];
  }
  return [super application:application didFinishLaunchingWithOptions:launchOptions];
}

- (FlutterEngine*)engineForTest:(NSString*)scenarioIdentifier {
  if ([scenarioIdentifier isEqualToString:@"spawn_engine_works"]) {
    FlutterEngine* spawner = [[FlutterEngine alloc] initWithName:@"FlutterControllerTest"
                                                         project:nil];
    [spawner run];
    return [spawner spawnWithEntrypoint:nil libraryURI:nil initialRoute:nil entrypointArgs:nil];
  } else {
    FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"FlutterControllerTest"
                                                        project:nil];
    [engine run];
    return engine;
  }
}

- (FlutterViewController*)flutterViewControllerForTest:(NSString*)scenarioIdentifier
                                            withEngine:(FlutterEngine*)engine {
  if ([scenarioIdentifier isEqualToString:@"tap_status_bar"]) {
    return [[FlutterViewController alloc] initWithEngine:engine nibName:nil bundle:nil];
  } else {
    return [[NoStatusBarFlutterViewController alloc] initWithEngine:engine nibName:nil bundle:nil];
  }
}

- (void)setupFlutterViewControllerTest:(NSString*)scenarioIdentifier {
  FlutterEngine* engine = [self engineForTest:scenarioIdentifier];
  FlutterViewController* flutterViewController =
      [self flutterViewControllerForTest:scenarioIdentifier withEngine:engine];
  flutterViewController.view.accessibilityIdentifier = @"flutter_view";

  [engine.binaryMessenger
      setMessageHandlerOnChannel:@"waiting_for_status"
            binaryMessageHandler:^(NSData* _Nullable message, FlutterBinaryReply _Nonnull reply) {
              FlutterMethodChannel* channel = [FlutterMethodChannel
                  methodChannelWithName:@"driver"
                        binaryMessenger:engine.binaryMessenger
                                  codec:[FlutterJSONMethodCodec sharedInstance]];
              [channel invokeMethod:@"set_scenario" arguments:@{@"name" : scenarioIdentifier}];
            }];
  // Can be used to synchronize timing in the test for a signal from Dart.
  [engine.binaryMessenger
      setMessageHandlerOnChannel:@"display_data"
            binaryMessageHandler:^(NSData* _Nullable message, FlutterBinaryReply _Nonnull reply) {
              NSDictionary* dict = [NSJSONSerialization JSONObjectWithData:message
                                                                   options:0
                                                                     error:nil];
              UITextField* text = [[UITextField alloc] initWithFrame:CGRectMake(0, 400, 300, 100)];
              text.text = dict[@"data"];
              [flutterViewController.view addSubview:text];
            }];

  TextPlatformViewFactory* textPlatformViewFactory =
      [[TextPlatformViewFactory alloc] initWithMessenger:engine.binaryMessenger];
  NSObject<FlutterPluginRegistrar>* registrar =
      [engine registrarForPlugin:@"scenarios/TextPlatformViewPlugin"];
  [registrar registerViewFactory:textPlatformViewFactory
                                withId:@"scenarios/textPlatformView"
      gestureRecognizersBlockingPolicy:FlutterPlatformViewGestureRecognizersBlockingPolicyEager];
  [registrar registerViewFactory:textPlatformViewFactory
                                withId:@"scenarios/textPlatformView_blockPolicyUntilTouchesEnded"
      gestureRecognizersBlockingPolicy:
          FlutterPlatformViewGestureRecognizersBlockingPolicyWaitUntilTouchesEnded];

  UIViewController* rootViewController = flutterViewController;
  // Make Flutter View's origin x/y not 0.
  if ([scenarioIdentifier isEqualToString:@"non_full_screen_flutter_view_platform_view"]) {
    rootViewController = [[NoStatusBarViewController alloc] init];
    [rootViewController.view addSubview:flutterViewController.view];
    flutterViewController.view.frame = CGRectMake(150, 150, 500, 500);
  }

  self.window.rootViewController = rootViewController;
}

@end
