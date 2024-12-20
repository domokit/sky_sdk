// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterMenuPlugin.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterMenuPlugin_Internal.h"

#import "flutter/shell/platform/common/platform_provided_menu.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterPluginMacOS.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterPluginRegistrarMacOS.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterDartProject_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputPlugin.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputSemanticsObject.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"
#include "flutter/testing/autoreleasepool_test.h"
#include "flutter/testing/testing.h"
#include "gtest/gtest.h"

@interface FakePluginRegistrar : NSObject <FlutterPluginRegistrar>
@property(nonatomic, readonly) id<FlutterPlugin> plugin;
@property(nonatomic, readonly) FlutterMethodChannel* channel;
@end

@implementation FakePluginRegistrar

// Synthesize properties declared in FlutterPluginRegistrar protocol.
@synthesize messenger;
@synthesize textures;
@synthesize view;

- (void)addMethodCallDelegate:(nonnull id<FlutterPlugin>)delegate
                      channel:(nonnull FlutterMethodChannel*)channel {
  _plugin = delegate;
  _channel = channel;
  [_channel setMethodCallHandler:^(FlutterMethodCall* call, FlutterResult result) {
    [delegate handleMethodCall:call result:result];
  }];
}

- (void)addApplicationDelegate:(nonnull NSObject<FlutterAppLifecycleDelegate>*)delegate {
}

- (void)registerViewFactory:(nonnull NSObject<FlutterPlatformViewFactory>*)factory
                     withId:(nonnull NSString*)factoryId {
}

- (void)publish:(nonnull NSObject*)value {
}

- (nonnull NSString*)lookupKeyForAsset:(nonnull NSString*)asset {
  return @"";
}

- (nonnull NSString*)lookupKeyForAsset:(nonnull NSString*)asset
                           fromPackage:(nonnull NSString*)package {
  return @"";
}
@end

namespace flutter::testing {

// FlutterMenuPluginTest is an AutoreleasePoolTest that allocates an NSView.
//
// This supports the use of NSApplication features that rely on the assumption of a view, such as
// when modifying the application menu bar, or even accessing the NSApplication.localizedName
// property.
//
// See: https://github.com/flutter/flutter/issues/104748#issuecomment-1159336728
class FlutterMenuPluginTest : public AutoreleasePoolTest {
 public:
  FlutterMenuPluginTest();
  ~FlutterMenuPluginTest() = default;

 private:
  NSView* view_;
};

FlutterMenuPluginTest::FlutterMenuPluginTest() {
  view_ = [[NSView alloc] initWithFrame:NSZeroRect];
  view_.wantsLayer = YES;
}

TEST_F(FlutterMenuPluginTest, TestSetMenu) {
  // Build a simulation of the default main menu.
  NSMenu* mainMenu = [[NSMenu alloc] init];
  NSMenuItem* appNameMenu = [[NSMenuItem alloc] initWithTitle:NSLocalizedString(@"APP_NAME", nil)
                                                       action:nil
                                                keyEquivalent:@""];
  NSMenu* submenu =
      [[NSMenu alloc] initWithTitle:NSLocalizedString(@"Prexisting APP_NAME menu", nil)];
  [submenu addItem:[[NSMenuItem alloc] initWithTitle:NSLocalizedString(@"About APP_NAME", nil)
                                              action:nil
                                       keyEquivalent:@""]];
  appNameMenu.submenu = submenu;
  [mainMenu addItem:appNameMenu];
  [NSApp setMainMenu:mainMenu];

  FakePluginRegistrar* registrar = [[FakePluginRegistrar alloc] init];
  [FlutterMenuPlugin registerWithRegistrar:registrar];
  FlutterMenuPlugin* plugin = [registrar plugin];

  NSDictionary* testMenus = @{
    @"0" : @[
      @{
        @"id" : [NSNumber numberWithInt:1],
        @"label" : @"APP_NAME",
        @"enabled" : @(YES),
        @"children" : @[
          @{
            @"id" : [NSNumber numberWithInt:3],
            @"platformProvidedMenu" : @(static_cast<int>(flutter::PlatformProvidedMenu::kQuit)),
            @"enabled" : @(YES),
          },
          @{
            @"id" : [NSNumber numberWithInt:2],
            @"label" : @"APP_NAME Info",
            @"enabled" : @(YES),
            @"shortcutTrigger" : [NSNumber numberWithUnsignedLongLong:0x61],
            @"shortcutModifiers" : [NSNumber numberWithUnsignedInt:0],
          },
        ],
      },
      @{
        @"id" : [NSNumber numberWithInt:4],
        @"label" : @"Help for APP_NAME",
        @"enabled" : @(YES),
        @"children" : @[
          @{
            @"id" : [NSNumber numberWithInt:5],
            @"label" : @"Help me!",
            @"enabled" : @(YES),
          },
          @{
            @"id" : [NSNumber numberWithInt:6],
            @"label" : @"",
            @"enabled" : @(NO),
            @"isDivider" : @(YES),
          },
          @{
            @"id" : [NSNumber numberWithInt:7],
            @"label" : @"Search help",
            @"enabled" : @(NO),
          },
        ],
      },
    ],
  };

  __block id available = @NO;
  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"Menu.isPluginAvailable"
                                                             arguments:nil]
                    result:^(id _Nullable result) {
                      available = result;
                    }];

  EXPECT_TRUE(available);

  [plugin handleMethodCall:[FlutterMethodCall methodCallWithMethodName:@"Menu.setMenus"
                                                             arguments:testMenus]
                    result:^(id _Nullable result){
                    }];

  EXPECT_EQ([NSApp.mainMenu numberOfItems], 2);
  NSMenuItem* firstMenu = [NSApp.mainMenu itemAtIndex:0];
  EXPECT_TRUE([[firstMenu title] isEqualToString:@"flutter_desktop_darwin_unittests"]);
  EXPECT_EQ([firstMenu tag], 1);
  EXPECT_TRUE([firstMenu isEnabled]);
  EXPECT_FALSE([firstMenu isHidden]);
  EXPECT_TRUE([[firstMenu keyEquivalent] isEqualToString:@"\0"]);

  EXPECT_EQ([[firstMenu submenu] numberOfItems], 1);
  NSMenuItem* firstItem = [[firstMenu submenu] itemAtIndex:0];
  EXPECT_TRUE([[firstItem title] isEqualToString:@"flutter_desktop_darwin_unittests Info"]);
  EXPECT_TRUE([[firstItem keyEquivalent] isEqualToString:@"a"]);
  EXPECT_TRUE([firstItem isEnabled]);
  EXPECT_FALSE([firstItem isHidden]);
  EXPECT_TRUE(
      [NSStringFromSelector([firstItem action]) isEqualToString:@"flutterMenuItemSelected:"]);
  EXPECT_EQ([firstItem tag], 2);

  NSMenuItem* secondMenu = [NSApp.mainMenu itemAtIndex:1];
  EXPECT_TRUE([[secondMenu title] isEqualToString:@"Help for flutter_desktop_darwin_unittests"]);
  EXPECT_EQ([secondMenu tag], 4);
  EXPECT_TRUE([secondMenu isEnabled]);
  EXPECT_FALSE([secondMenu isHidden]);

  EXPECT_EQ([[secondMenu submenu] numberOfItems], 3);
  NSMenuItem* secondMenuFirst = [[secondMenu submenu] itemAtIndex:0];
  EXPECT_TRUE([[secondMenuFirst title] isEqualToString:@"Help me!"]);
  EXPECT_TRUE([secondMenuFirst isEnabled]);
  EXPECT_FALSE([secondMenuFirst isHidden]);
  EXPECT_TRUE(
      [NSStringFromSelector([secondMenuFirst action]) isEqualToString:@"flutterMenuItemSelected:"]);
  EXPECT_EQ([secondMenuFirst tag], 5);

  NSMenuItem* secondMenuDivider = [[secondMenu submenu] itemAtIndex:1];
  EXPECT_TRUE([[secondMenuDivider title] isEqualToString:@""]);
  EXPECT_TRUE([[secondMenuDivider keyEquivalent] isEqualToString:@""]);
  EXPECT_FALSE([secondMenuDivider isEnabled]);
  EXPECT_FALSE([secondMenuDivider isHidden]);
  EXPECT_EQ([secondMenuDivider action], nil);
  EXPECT_EQ([secondMenuDivider tag], 0);

  NSMenuItem* secondMenuLast = [[secondMenu submenu] itemAtIndex:2];
  EXPECT_TRUE([[secondMenuLast title] isEqualToString:@"Search help"]);
  EXPECT_FALSE([secondMenuLast isEnabled]);
  EXPECT_FALSE([secondMenuLast isHidden]);
  EXPECT_TRUE(
      [NSStringFromSelector([secondMenuLast action]) isEqualToString:@"flutterMenuItemSelected:"]);
  EXPECT_EQ([secondMenuLast tag], 7);
}

}  // namespace flutter::testing
