// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformPlugin.h"

#import <AudioToolbox/AudioToolbox.h>
#import <Foundation/Foundation.h>
#import <UIKit/UIApplication.h>
#import <UIKit/UIKit.h>

#include "flutter/fml/logging.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/UIViewController+FlutterScreenAndSceneIfLoaded.h"

FLUTTER_ASSERT_ARC

namespace {

constexpr char kTextPlainFormat[] = "text/plain";
const UInt32 kKeyPressClickSoundId = 1306;

#if not APPLICATION_EXTENSION_API_ONLY
NSString* const kSearchURLPrefix = @"x-web-search://?";
#endif

}  // namespace

namespace flutter {

// TODO(abarth): Move these definitions from system_chrome_impl.cc to here.
const char* const kOrientationUpdateNotificationName =
    "io.flutter.plugin.platform.SystemChromeOrientationNotificationName";
const char* const kOrientationUpdateNotificationKey =
    "io.flutter.plugin.platform.SystemChromeOrientationNotificationKey";
const char* const kOverlayStyleUpdateNotificationName =
    "io.flutter.plugin.platform.SystemChromeOverlayNotificationName";
const char* const kOverlayStyleUpdateNotificationKey =
    "io.flutter.plugin.platform.SystemChromeOverlayNotificationKey";

}  // namespace flutter

using namespace flutter;

static void SetStatusBarHiddenForSharedApplication(BOOL hidden) {
#if not APPLICATION_EXTENSION_API_ONLY
  [UIApplication sharedApplication].statusBarHidden = hidden;
#else
  FML_LOG(WARNING) << "Application based status bar styling is not available in app extension.";
#endif
}

static void SetStatusBarStyleForSharedApplication(UIStatusBarStyle style) {
#if not APPLICATION_EXTENSION_API_ONLY
  // Note: -[UIApplication setStatusBarStyle] is deprecated in iOS9
  // in favor of delegating to the view controller.
  [[UIApplication sharedApplication] setStatusBarStyle:style];
#else
  FML_LOG(WARNING) << "Application based status bar styling is not available in app extension.";
#endif
}

@interface FlutterPlatformPlugin ()

/**
 * @brief Whether the status bar appearance is based on the style preferred for this ViewController.
 *
 *        The default value is YES.
 *        Explicitly add `UIViewControllerBasedStatusBarAppearance` as `false` in
 *        info.plist makes this value to be false.
 */
@property(nonatomic, assign) BOOL enableViewControllerBasedStatusBarAppearance;
@property(nonatomic, weak) FlutterEngine* engine;

/**
 * @brief Used to detect whether or not this device supports live text input from the camera.
 */
@property(nonatomic, strong) UITextField* textField;
@end

@implementation FlutterPlatformPlugin

- (instancetype)initWithEngine:(FlutterEngine*)engine {
  FML_DCHECK(engine) << "engine must be set";
  self = [super init];

  if (self) {
    _engine = engine;
    NSObject* infoValue = [[NSBundle mainBundle]
        objectForInfoDictionaryKey:@"UIViewControllerBasedStatusBarAppearance"];
#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG
    if (infoValue != nil && ![infoValue isKindOfClass:[NSNumber class]]) {
      FML_LOG(ERROR) << "The value of UIViewControllerBasedStatusBarAppearance in info.plist must "
                        "be a Boolean type.";
    }
#endif
    _enableViewControllerBasedStatusBarAppearance =
        (infoValue == nil || [(NSNumber*)infoValue boolValue]);
  }

  return self;
}

- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result {
  NSString* method = call.method;
  id args = call.arguments;
  if ([method isEqualToString:@"SystemSound.play"]) {
    [self playSystemSound:args];
    result(nil);
  } else if ([method isEqualToString:@"HapticFeedback.vibrate"]) {
    [self vibrateHapticFeedback:args];
    result(nil);
  } else if ([method isEqualToString:@"SystemChrome.setPreferredOrientations"]) {
    [self setSystemChromePreferredOrientations:args];
    result(nil);
  } else if ([method isEqualToString:@"SystemChrome.setApplicationSwitcherDescription"]) {
    [self setSystemChromeApplicationSwitcherDescription:args];
    result(nil);
  } else if ([method isEqualToString:@"SystemChrome.setEnabledSystemUIOverlays"]) {
    [self setSystemChromeEnabledSystemUIOverlays:args];
    result(nil);
  } else if ([method isEqualToString:@"SystemChrome.setEnabledSystemUIMode"]) {
    [self setSystemChromeEnabledSystemUIMode:args];
    result(nil);
  } else if ([method isEqualToString:@"SystemChrome.restoreSystemUIOverlays"]) {
    [self restoreSystemChromeSystemUIOverlays];
    result(nil);
  } else if ([method isEqualToString:@"SystemChrome.setSystemUIOverlayStyle"]) {
    [self setSystemChromeSystemUIOverlayStyle:args];
    result(nil);
  } else if ([method isEqualToString:@"SystemNavigator.pop"]) {
    NSNumber* isAnimated = args;
    [self popSystemNavigator:isAnimated.boolValue];
    result(nil);
  } else if ([method isEqualToString:@"Clipboard.getData"]) {
    result([self getClipboardData:args]);
  } else if ([method isEqualToString:@"Clipboard.setData"]) {
    [self setClipboardData:args];
    result(nil);
  } else if ([method isEqualToString:@"Clipboard.hasStrings"]) {
    result([self clipboardHasStrings]);
  } else if ([method isEqualToString:@"LiveText.isLiveTextInputAvailable"]) {
    result(@([self isLiveTextInputAvailable]));
  } else if ([method isEqualToString:@"SearchWeb.invoke"]) {
    [self searchWeb:args];
    result(nil);
  } else if ([method isEqualToString:@"LookUp.invoke"]) {
    [self showLookUpViewController:args];
    result(nil);
  } else if ([method isEqualToString:@"Share.invoke"]) {
    [self showShareViewController:args];
    result(nil);
  } else if ([method isEqualToString:@"ContextMenu.showSystemContextMenu"]) {
    [self showSystemContextMenu:args];
    result(nil);
  } else if ([method isEqualToString:@"ContextMenu.hideSystemContextMenu"]) {
    [self hideSystemContextMenu];
    result(nil);
  } else {
    result(FlutterMethodNotImplemented);
  }
}

- (void)showSystemContextMenu:(NSDictionary*)args {
  if (@available(iOS 16.0, *)) {
    FlutterTextInputPlugin* textInputPlugin = [self.engine textInputPlugin];
    BOOL shownEditMenu = [textInputPlugin showEditMenu:args];
    if (!shownEditMenu) {
      FML_LOG(ERROR) << "Only text input supports system context menu for now. Ensure the system "
                        "context menu is shown with an active text input connection. See "
                        "https://github.com/flutter/flutter/issues/143033.";
    }
  }
}

- (void)hideSystemContextMenu {
  if (@available(iOS 16.0, *)) {
    FlutterTextInputPlugin* textInputPlugin = [self.engine textInputPlugin];
    [textInputPlugin hideEditMenu];
  }
}

- (void)showShareViewController:(NSString*)content {
  UIViewController* engineViewController = [self.engine viewController];

  NSArray* itemsToShare = @[ content ?: [NSNull null] ];
  UIActivityViewController* activityViewController =
      [[UIActivityViewController alloc] initWithActivityItems:itemsToShare
                                        applicationActivities:nil];

  if (UI_USER_INTERFACE_IDIOM() == UIUserInterfaceIdiomPad) {
    // On iPad, the share screen is presented in a popover view, and requires a
    // sourceView and sourceRect
    FlutterTextInputPlugin* _textInputPlugin = [self.engine textInputPlugin];
    UITextRange* range = _textInputPlugin.textInputView.selectedTextRange;

    // firstRectForRange cannot be used here as it's current implementation does
    // not always return the full rect of the range.
    CGRect firstRect = [(FlutterTextInputView*)_textInputPlugin.textInputView
        caretRectForPosition:(FlutterTextPosition*)range.start];
    CGRect transformedFirstRect = [(FlutterTextInputView*)_textInputPlugin.textInputView
        localRectFromFrameworkTransform:firstRect];
    CGRect lastRect = [(FlutterTextInputView*)_textInputPlugin.textInputView
        caretRectForPosition:(FlutterTextPosition*)range.end];
    CGRect transformedLastRect = [(FlutterTextInputView*)_textInputPlugin.textInputView
        localRectFromFrameworkTransform:lastRect];

    activityViewController.popoverPresentationController.sourceView = engineViewController.view;
    // In case of RTL Language, get the minimum x coordinate
    activityViewController.popoverPresentationController.sourceRect =
        CGRectMake(fmin(transformedFirstRect.origin.x, transformedLastRect.origin.x),
                   transformedFirstRect.origin.y,
                   abs(transformedLastRect.origin.x - transformedFirstRect.origin.x),
                   transformedFirstRect.size.height);
  }

  [engineViewController presentViewController:activityViewController animated:YES completion:nil];
}

- (void)searchWeb:(NSString*)searchTerm {
#if APPLICATION_EXTENSION_API_ONLY
  FML_LOG(WARNING) << "SearchWeb.invoke is not availabe in app extension.";
#else
  NSString* escapedText = [searchTerm
      stringByAddingPercentEncodingWithAllowedCharacters:[NSCharacterSet
                                                             URLHostAllowedCharacterSet]];
  NSString* searchURL = [NSString stringWithFormat:@"%@%@", kSearchURLPrefix, escapedText];

  [[UIApplication sharedApplication] openURL:[NSURL URLWithString:searchURL]
                                     options:@{}
                           completionHandler:nil];
#endif
}

- (void)playSystemSound:(NSString*)soundType {
  if ([soundType isEqualToString:@"SystemSoundType.click"]) {
    // All feedback types are specific to Android and are treated as equal on
    // iOS.
    AudioServicesPlaySystemSound(kKeyPressClickSoundId);
  }
}

- (void)vibrateHapticFeedback:(NSString*)feedbackType {
  if (!feedbackType) {
    AudioServicesPlaySystemSound(kSystemSoundID_Vibrate);
    return;
  }

  if ([@"HapticFeedbackType.lightImpact" isEqualToString:feedbackType]) {
    [[[UIImpactFeedbackGenerator alloc] initWithStyle:UIImpactFeedbackStyleLight] impactOccurred];
  } else if ([@"HapticFeedbackType.mediumImpact" isEqualToString:feedbackType]) {
    [[[UIImpactFeedbackGenerator alloc] initWithStyle:UIImpactFeedbackStyleMedium] impactOccurred];
  } else if ([@"HapticFeedbackType.heavyImpact" isEqualToString:feedbackType]) {
    [[[UIImpactFeedbackGenerator alloc] initWithStyle:UIImpactFeedbackStyleHeavy] impactOccurred];
  } else if ([@"HapticFeedbackType.selectionClick" isEqualToString:feedbackType]) {
    [[[UISelectionFeedbackGenerator alloc] init] selectionChanged];
  }
}

- (void)setSystemChromePreferredOrientations:(NSArray*)orientations {
  UIInterfaceOrientationMask mask = 0;

  if (orientations.count == 0) {
    mask |= UIInterfaceOrientationMaskAll;
  } else {
    for (NSString* orientation in orientations) {
      if ([orientation isEqualToString:@"DeviceOrientation.portraitUp"]) {
        mask |= UIInterfaceOrientationMaskPortrait;
      } else if ([orientation isEqualToString:@"DeviceOrientation.portraitDown"]) {
        mask |= UIInterfaceOrientationMaskPortraitUpsideDown;
      } else if ([orientation isEqualToString:@"DeviceOrientation.landscapeLeft"]) {
        mask |= UIInterfaceOrientationMaskLandscapeLeft;
      } else if ([orientation isEqualToString:@"DeviceOrientation.landscapeRight"]) {
        mask |= UIInterfaceOrientationMaskLandscapeRight;
      }
    }
  }

  if (!mask) {
    return;
  }
  [[NSNotificationCenter defaultCenter]
      postNotificationName:@(kOrientationUpdateNotificationName)
                    object:nil
                  userInfo:@{@(kOrientationUpdateNotificationKey) : @(mask)}];
}

- (void)setSystemChromeApplicationSwitcherDescription:(NSDictionary*)object {
  // No counterpart on iOS but is a benign operation. So no asserts.
}

- (void)setSystemChromeEnabledSystemUIOverlays:(NSArray*)overlays {
  BOOL statusBarShouldBeHidden = ![overlays containsObject:@"SystemUiOverlay.top"];
  if ([overlays containsObject:@"SystemUiOverlay.bottom"]) {
    [[NSNotificationCenter defaultCenter]
        postNotificationName:FlutterViewControllerShowHomeIndicator
                      object:nil];
  } else {
    [[NSNotificationCenter defaultCenter]
        postNotificationName:FlutterViewControllerHideHomeIndicator
                      object:nil];
  }
  if (self.enableViewControllerBasedStatusBarAppearance) {
    [self.engine viewController].prefersStatusBarHidden = statusBarShouldBeHidden;
  } else {
    // Checks if the top status bar should be visible. This platform ignores all
    // other overlays

    // We opt out of view controller based status bar visibility since we want
    // to be able to modify this on the fly. The key used is
    // UIViewControllerBasedStatusBarAppearance.
    SetStatusBarHiddenForSharedApplication(statusBarShouldBeHidden);
  }
}

- (void)setSystemChromeEnabledSystemUIMode:(NSString*)mode {
  BOOL edgeToEdge = [mode isEqualToString:@"SystemUiMode.edgeToEdge"];
  if (self.enableViewControllerBasedStatusBarAppearance) {
    [self.engine viewController].prefersStatusBarHidden = !edgeToEdge;
  } else {
    // Checks if the top status bar should be visible, reflected by edge to edge setting. This
    // platform ignores all other system ui modes.

    // We opt out of view controller based status bar visibility since we want
    // to be able to modify this on the fly. The key used is
    // UIViewControllerBasedStatusBarAppearance.
    SetStatusBarHiddenForSharedApplication(!edgeToEdge);
  }
  [[NSNotificationCenter defaultCenter]
      postNotificationName:edgeToEdge ? FlutterViewControllerShowHomeIndicator
                                      : FlutterViewControllerHideHomeIndicator
                    object:nil];
}

- (void)restoreSystemChromeSystemUIOverlays {
  // Nothing to do on iOS.
}

- (void)setSystemChromeSystemUIOverlayStyle:(NSDictionary*)message {
  NSString* brightness = message[@"statusBarBrightness"];
  if (brightness == (id)[NSNull null]) {
    return;
  }

  UIStatusBarStyle statusBarStyle;
  if ([brightness isEqualToString:@"Brightness.dark"]) {
    statusBarStyle = UIStatusBarStyleLightContent;
  } else if ([brightness isEqualToString:@"Brightness.light"]) {
    if (@available(iOS 13, *)) {
      statusBarStyle = UIStatusBarStyleDarkContent;
    } else {
      statusBarStyle = UIStatusBarStyleDefault;
    }
  } else {
    return;
  }

  if (self.enableViewControllerBasedStatusBarAppearance) {
    // This notification is respected by the iOS embedder.
    [[NSNotificationCenter defaultCenter]
        postNotificationName:@(kOverlayStyleUpdateNotificationName)
                      object:nil
                    userInfo:@{@(kOverlayStyleUpdateNotificationKey) : @(statusBarStyle)}];
  } else {
    SetStatusBarStyleForSharedApplication(statusBarStyle);
  }
}

- (void)popSystemNavigator:(BOOL)isAnimated {
  // Apple's human user guidelines say not to terminate iOS applications. However, if the
  // root view of the app is a navigation controller, it is instructed to back up a level
  // in the navigation hierarchy.
  // It's also possible in an Add2App scenario that the FlutterViewController was presented
  // outside the context of a UINavigationController, and still wants to be popped.

  FlutterViewController* engineViewController = [self.engine viewController];
  UINavigationController* navigationController = [engineViewController navigationController];
  if (navigationController) {
    [navigationController popViewControllerAnimated:isAnimated];
  } else {
    UIViewController* rootViewController = nil;
#if APPLICATION_EXTENSION_API_ONLY
    if (@available(iOS 15.0, *)) {
      rootViewController =
          [engineViewController flutterWindowSceneIfViewLoaded].keyWindow.rootViewController;
    } else {
      FML_LOG(WARNING)
          << "rootViewController is not available in application extension prior to iOS 15.0.";
    }
#else
    rootViewController = [UIApplication sharedApplication].keyWindow.rootViewController;
#endif
    if (engineViewController != rootViewController) {
      [engineViewController dismissViewControllerAnimated:isAnimated completion:nil];
    }
  }
}

- (NSDictionary*)getClipboardData:(NSString*)format {
  UIPasteboard* pasteboard = [UIPasteboard generalPasteboard];
  if (!format || [format isEqualToString:@(kTextPlainFormat)]) {
    NSString* stringInPasteboard = pasteboard.string;
    // The pasteboard may contain an item but it may not be a string (an image for instance).
    return stringInPasteboard == nil ? nil : @{@"text" : stringInPasteboard};
  }
  return nil;
}

- (void)setClipboardData:(NSDictionary*)data {
  UIPasteboard* pasteboard = [UIPasteboard generalPasteboard];
  id copyText = data[@"text"];
  if ([copyText isKindOfClass:[NSString class]]) {
    pasteboard.string = copyText;
  } else {
    pasteboard.string = @"null";
  }
}

- (NSDictionary*)clipboardHasStrings {
  return @{@"value" : @([UIPasteboard generalPasteboard].hasStrings)};
}

- (BOOL)isLiveTextInputAvailable {
  return [[self textField] canPerformAction:@selector(captureTextFromCamera:) withSender:nil];
}

- (void)showLookUpViewController:(NSString*)term {
  UIViewController* engineViewController = [self.engine viewController];
  UIReferenceLibraryViewController* referenceLibraryViewController =
      [[UIReferenceLibraryViewController alloc] initWithTerm:term];
  [engineViewController presentViewController:referenceLibraryViewController
                                     animated:YES
                                   completion:nil];
}

- (UITextField*)textField {
  if (_textField == nil) {
    _textField = [[UITextField alloc] init];
  }
  return _textField;
}

@end
