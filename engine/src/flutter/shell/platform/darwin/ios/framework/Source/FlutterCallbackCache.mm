// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterCallbackCache_Internal.h"

#include "flutter/fml/logging.h"
#include "flutter/lib/ui/plugins/callback_cache.h"

FLUTTER_ASSERT_ARC

@implementation FlutterCallbackInformation
@end

@implementation FlutterCallbackCache

+ (FlutterCallbackInformation*)lookupCallbackInformation:(int64_t)handle {
  auto info = flutter::DartCallbackCache::GetCallbackInformation(handle);
  if (info == nullptr) {
    return nil;
  }
  FlutterCallbackInformation* new_info = [[FlutterCallbackInformation alloc] init];
  new_info.callbackName = [NSString stringWithUTF8String:info->name.c_str()];
  new_info.callbackClassName = [NSString stringWithUTF8String:info->class_name.c_str()];
  new_info.callbackLibraryPath = [NSString stringWithUTF8String:info->library_path.c_str()];
  return new_info;
}

+ (void)setCachePath:(NSString*)path {
  FML_DCHECK(path != nil);
  flutter::DartCallbackCache::SetCachePath([path UTF8String]);
  NSString* cache_path =
      [NSString stringWithUTF8String:flutter::DartCallbackCache::GetCachePath().c_str()];
  // Set the "Do Not Backup" flag to ensure that the cache isn't moved off disk in
  // low-memory situations.
  if (![[NSFileManager defaultManager] fileExistsAtPath:cache_path]) {
    [[NSFileManager defaultManager] createFileAtPath:cache_path contents:nil attributes:nil];
    NSError* error = nil;
    NSURL* URL = [NSURL fileURLWithPath:cache_path];
    BOOL success = [URL setResourceValue:[NSNumber numberWithBool:YES]
                                  forKey:NSURLIsExcludedFromBackupKey
                                   error:&error];
    if (!success) {
      NSLog(@"Error excluding %@ from backup %@", [URL lastPathComponent], error);
    }
  }
}

@end
