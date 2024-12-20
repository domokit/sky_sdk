// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterEngineGroup.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterEngine_Internal.h"

FLUTTER_ASSERT_ARC

@implementation FlutterEngineGroupOptions
@end

@interface FlutterEngineGroup ()
@property(nonatomic, copy) NSString* name;
@property(nonatomic, strong) NSPointerArray* engines;
@property(nonatomic, copy) FlutterDartProject* project;
@property(nonatomic, assign) NSUInteger enginesCreatedCount;
@end

@implementation FlutterEngineGroup

- (instancetype)initWithName:(NSString*)name project:(nullable FlutterDartProject*)project {
  self = [super init];
  if (self) {
    _name = [name copy];
    _engines = [NSPointerArray weakObjectsPointerArray];
    _project = project;
  }
  return self;
}

- (FlutterEngine*)makeEngineWithEntrypoint:(nullable NSString*)entrypoint
                                libraryURI:(nullable NSString*)libraryURI {
  return [self makeEngineWithEntrypoint:entrypoint libraryURI:libraryURI initialRoute:nil];
}

- (FlutterEngine*)makeEngineWithEntrypoint:(nullable NSString*)entrypoint
                                libraryURI:(nullable NSString*)libraryURI
                              initialRoute:(nullable NSString*)initialRoute {
  FlutterEngineGroupOptions* options = [[FlutterEngineGroupOptions alloc] init];
  options.entrypoint = entrypoint;
  options.libraryURI = libraryURI;
  options.initialRoute = initialRoute;
  return [self makeEngineWithOptions:options];
}

- (FlutterEngine*)makeEngineWithOptions:(nullable FlutterEngineGroupOptions*)options {
  NSString* entrypoint = options.entrypoint;
  NSString* libraryURI = options.libraryURI;
  NSString* initialRoute = options.initialRoute;
  NSArray<NSString*>* entrypointArgs = options.entrypointArgs;

  FlutterEngine* engine;
  // NSPointerArray is clever and assumes that unless a mutation operation has occurred on it that
  // has set one of its values to nil, nothing could have changed and it can skip compaction.
  // That's reasonable behaviour on a regular NSPointerArray but not for a weakObjectPointerArray.
  // As a workaround, we mutate it first. See: http://www.openradar.me/15396578
  [self.engines addPointer:nil];
  [self.engines compact];
  if (self.engines.count <= 0) {
    engine = [self makeEngine];
    [engine runWithEntrypoint:entrypoint
                   libraryURI:libraryURI
                 initialRoute:initialRoute
               entrypointArgs:entrypointArgs];
  } else {
    FlutterEngine* spawner = (__bridge FlutterEngine*)[self.engines pointerAtIndex:0];
    engine = [spawner spawnWithEntrypoint:entrypoint
                               libraryURI:libraryURI
                             initialRoute:initialRoute
                           entrypointArgs:entrypointArgs];
  }
  [self.engines addPointer:(__bridge void*)engine];

  return engine;
}

- (FlutterEngine*)makeEngine {
  NSString* engineName =
      [NSString stringWithFormat:@"%@.%lu", self.name, ++self.enginesCreatedCount];
  return [[FlutterEngine alloc] initWithName:engineName project:self.project];
}

@end
