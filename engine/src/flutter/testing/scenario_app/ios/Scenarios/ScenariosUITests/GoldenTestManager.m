// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "GoldenTestManager.h"
#import <XCTest/XCTest.h>

@interface GoldenTestManager ()

@property(readwrite, strong, nonatomic) GoldenImage* goldenImage;

@end

@implementation GoldenTestManager

NSDictionary* launchArgsMap;
const double kDefaultRmseThreshold = 0.5;

- (instancetype)initWithLaunchArg:(NSString*)launchArg {
  self = [super init];
  if (self) {
    // The launchArgsMap should match the one in the `PlatformVieGoldenTestManager`.
    static NSDictionary<NSString*, NSString*>* launchArgsMap;
    static dispatch_once_t onceToken;
    dispatch_once(&onceToken, ^{
      launchArgsMap = @{
        @"--platform-view" : @"platform_view",
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
        @"--non-full-screen-flutter-view-platform-view" :
            @"non_full_screen_flutter_view_platform_view",
        @"--bogus-font-text" : @"bogus_font_text",
        @"--spawn-engine-works" : @"spawn_engine_works",
        @"--platform-view-cliprect-after-moved" : @"platform_view_cliprect_after_moved",
        @"--platform-view-cliprect-after-moved-multiple-clips" :
            @"platform_view_cliprect_after_moved_multiple_clips",
        @"--two-platform-view-clip-rect" : @"two_platform_view_clip_rect",
        @"--two-platform-view-clip-rect-multiple-clips" :
            @"two_platform_view_clip_rect_multiple_clips",
        @"--two-platform-view-clip-rrect" : @"two_platform_view_clip_rrect",
        @"--two-platform-view-clip-rrect-multiple-clips" :
            @"two_platform_view_clip_rrect_multiple_clips",
        @"--two-platform-view-clip-path" : @"two_platform_view_clip_path",
        @"--two-platform-view-clip-path-multiple-clips" :
            @"two_platform_view_clip_path_multiple_clips",
        @"--app-extension" : @"app_extension",
        @"--darwin-system-font" : @"darwin_system_font",
      };
    });
    _identifier = launchArgsMap[launchArg];

    NSString* impeller = @"impeller_";
    NSNumber* enableImpeller = [[NSBundle bundleWithIdentifier:@"dev.flutter.Scenarios"]
        objectForInfoDictionaryKey:@"FLTEnableImpeller"];
    if (enableImpeller != nil && !enableImpeller.boolValue) {
      impeller = @"";
      NSLog(@"Testing Skia: FLTEnableImpeller is NO");
    } else {
      NSLog(@"Testing Impeller");
    }

    NSString* prefix = [NSString stringWithFormat:@"golden_%@_%@", _identifier, impeller];
    _goldenImage = [[GoldenImage alloc] initWithGoldenNamePrefix:prefix];
    _launchArg = launchArg;
  }
  return self;
}

- (void)checkGoldenForTest:(XCTestCase*)test rmesThreshold:(double)rmesThreshold {
  XCUIScreenshot* screenshot = [[XCUIScreen mainScreen] screenshot];
  if (!_goldenImage.image) {
    XCTAttachment* attachment = [XCTAttachment attachmentWithScreenshot:screenshot];
    attachment.name = [_goldenImage.goldenName stringByAppendingString:@"_new.png"];
    attachment.lifetime = XCTAttachmentLifetimeKeepAlways;
    [test addAttachment:attachment];
    // Instead of XCTFail because that definition changed between Xcode 11 and 12 whereas this impl
    // is stable.
    _XCTPrimitiveFail(test,
                      @"This test will fail - no golden named %@ found. "
                      @"Follow the steps in the README to add a new golden.",
                      _goldenImage.goldenName);
  }

  if (![_goldenImage compareGoldenToImage:screenshot.image rmesThreshold:rmesThreshold]) {
    XCTAttachment* screenshotAttachment = [XCTAttachment attachmentWithImage:screenshot.image];
    screenshotAttachment.name = [_goldenImage.goldenName stringByAppendingString:@"_actual.png"];
    screenshotAttachment.lifetime = XCTAttachmentLifetimeKeepAlways;
    [test addAttachment:screenshotAttachment];

    _XCTPrimitiveFail(test,
                      @"Goldens do not match. Follow the steps in the "
                      @"README to update golden named %@ if needed.",
                      _goldenImage.goldenName);
  }
}

@end
