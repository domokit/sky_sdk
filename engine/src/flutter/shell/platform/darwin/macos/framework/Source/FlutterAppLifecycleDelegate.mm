// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterAppLifecycleDelegate.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterAppLifecycleDelegate_Internal.h"

#include <AppKit/AppKit.h>
#include <AppKit/NSApplication.h>
#include <Foundation/Foundation.h>
#include <objc/message.h>

#include "flutter/fml/logging.h"
#include "flutter/fml/paths.h"

@implementation FlutterAppLifecycleRegistrar

- (void)addObserverFor:(NSString*)name selector:(SEL)selector {
  [[NSNotificationCenter defaultCenter] addObserver:self selector:selector name:name object:nil];
}

- (instancetype)init {
  if (self = [super init]) {
// Using a macro to avoid errors where the notification doesn't match the
// selector.
#ifdef OBSERVE_NOTIFICATION
#error OBSERVE_NOTIFICATION ALREADY DEFINED!
#else
#define OBSERVE_NOTIFICATION(SELECTOR) \
  [self addObserverFor:NSApplication##SELECTOR##Notification selector:@selector(handle##SELECTOR:)]
#endif

    OBSERVE_NOTIFICATION(WillFinishLaunching);
    OBSERVE_NOTIFICATION(DidFinishLaunching);
    OBSERVE_NOTIFICATION(WillBecomeActive);
    OBSERVE_NOTIFICATION(DidBecomeActive);
    OBSERVE_NOTIFICATION(WillResignActive);
    OBSERVE_NOTIFICATION(DidResignActive);
    OBSERVE_NOTIFICATION(WillTerminate);
    OBSERVE_NOTIFICATION(WillHide);
    OBSERVE_NOTIFICATION(DidHide);
    OBSERVE_NOTIFICATION(WillUnhide);
    OBSERVE_NOTIFICATION(DidUnhide);
    OBSERVE_NOTIFICATION(DidChangeScreenParameters);
    OBSERVE_NOTIFICATION(DidChangeOcclusionState);

#undef OBSERVE_NOTIFICATION

    _delegates = [NSPointerArray weakObjectsPointerArray];
  }
  return self;
}

static BOOL IsPowerOfTwo(NSUInteger x) {
  return x != 0 && (x & (x - 1)) == 0;
}

- (void)addDelegate:(NSObject<FlutterAppLifecycleDelegate>*)delegate {
  [_delegates addPointer:(__bridge void*)delegate];
  if (IsPowerOfTwo([_delegates count])) {
    [_delegates compact];
  }
}

- (void)removeDelegate:(NSObject<FlutterAppLifecycleDelegate>*)delegate {
  NSUInteger index = [[_delegates allObjects] indexOfObject:delegate];
  if (index != NSNotFound) {
    [_delegates removePointerAtIndex:index];
  }
}

// This isn't done via performSelector because that can cause leaks due to the
// selector not being known. Using a macro to avoid mismatch errors between the
// notification and the selector.
#ifdef DISTRIBUTE_NOTIFICATION
#error DISTRIBUTE_NOTIFICATION ALREADY DEFINED!
#else
#define DISTRIBUTE_NOTIFICATION(SELECTOR)                                  \
  -(void)handle##SELECTOR : (NSNotification*)notification {                \
    for (NSObject<FlutterAppLifecycleDelegate> * delegate in _delegates) { \
      if ([delegate respondsToSelector:@selector(handle##SELECTOR:)]) {    \
        [delegate handle##SELECTOR:notification];                          \
      }                                                                    \
    }                                                                      \
  }
#endif

DISTRIBUTE_NOTIFICATION(WillFinishLaunching)
DISTRIBUTE_NOTIFICATION(DidFinishLaunching)
DISTRIBUTE_NOTIFICATION(WillBecomeActive)
DISTRIBUTE_NOTIFICATION(DidBecomeActive)
DISTRIBUTE_NOTIFICATION(WillResignActive)
DISTRIBUTE_NOTIFICATION(DidResignActive)
DISTRIBUTE_NOTIFICATION(WillTerminate)
DISTRIBUTE_NOTIFICATION(WillHide)
DISTRIBUTE_NOTIFICATION(WillUnhide)
DISTRIBUTE_NOTIFICATION(DidHide)
DISTRIBUTE_NOTIFICATION(DidUnhide)
DISTRIBUTE_NOTIFICATION(DidChangeScreenParameters)
DISTRIBUTE_NOTIFICATION(DidChangeOcclusionState)

#undef DISTRIBUTE_NOTIFICATION

@end
