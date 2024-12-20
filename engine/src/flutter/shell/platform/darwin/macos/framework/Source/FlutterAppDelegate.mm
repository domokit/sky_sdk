// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterAppDelegate.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterAppDelegate_Internal.h"

#import <AppKit/AppKit.h>

#include "flutter/fml/logging.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterAppLifecycleDelegate.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterAppLifecycleDelegate_Internal.h"
#include "flutter/shell/platform/embedder/embedder.h"

@interface FlutterAppDelegate ()

/**
 * Returns the display name of the application as set in the Info.plist.
 */
- (NSString*)applicationName;

@property(nonatomic) FlutterAppLifecycleRegistrar* lifecycleRegistrar;
@end

@implementation FlutterAppDelegate

- (instancetype)init {
  if (self = [super init]) {
    _terminationHandler = nil;
    _lifecycleRegistrar = [[FlutterAppLifecycleRegistrar alloc] init];
  }
  return self;
}

- (void)applicationWillFinishLaunching:(NSNotification*)notification {
  // Update UI elements to match the application name.
  NSString* applicationName = [self applicationName];
  _mainFlutterWindow.title = applicationName;
  for (NSMenuItem* menuItem in _applicationMenu.itemArray) {
    menuItem.title = [menuItem.title stringByReplacingOccurrencesOfString:@"APP_NAME"
                                                               withString:applicationName];
  }
}

#pragma mark - Delegate handling

- (void)addApplicationLifecycleDelegate:(NSObject<FlutterAppLifecycleDelegate>*)delegate {
  [self.lifecycleRegistrar addDelegate:delegate];
}

- (void)removeApplicationLifecycleDelegate:(NSObject<FlutterAppLifecycleDelegate>*)delegate {
  [self.lifecycleRegistrar removeDelegate:delegate];
}

#pragma mark Private Methods

- (NSString*)applicationName {
  NSString* applicationName =
      [NSBundle.mainBundle objectForInfoDictionaryKey:@"CFBundleDisplayName"];
  if (!applicationName) {
    applicationName = [NSBundle.mainBundle objectForInfoDictionaryKey:@"CFBundleName"];
  }
  return applicationName;
}

#pragma mark NSApplicationDelegate

- (void)application:(NSApplication*)application openURLs:(NSArray<NSURL*>*)urls {
  for (NSObject<FlutterAppLifecycleDelegate>* delegate in self.lifecycleRegistrar.delegates) {
    if ([delegate respondsToSelector:@selector(handleOpenURLs:)] &&
        [delegate handleOpenURLs:urls]) {
      return;
    }
  }
}

- (NSApplicationTerminateReply)applicationShouldTerminate:(NSApplication* _Nonnull)sender {
  // If the framework has already told us to terminate, terminate immediately.
  if ([self terminationHandler] == nil || [[self terminationHandler] shouldTerminate]) {
    return NSTerminateNow;
  }

  // Send a termination request to the framework.
  FlutterEngineTerminationHandler* terminationHandler = [self terminationHandler];
  [terminationHandler requestApplicationTermination:sender
                                           exitType:kFlutterAppExitTypeCancelable
                                             result:nil];

  // Cancel termination to allow the framework to handle the request asynchronously. When the
  // termination request returns from the app, if termination is desired, this method will be
  // reinvoked with self.terminationHandler.shouldTerminate set to YES.
  return NSTerminateCancel;
}

@end
