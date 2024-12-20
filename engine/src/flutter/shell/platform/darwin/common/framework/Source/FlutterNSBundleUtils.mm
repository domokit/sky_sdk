// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/common/framework/Source/FlutterNSBundleUtils.h"

#include <Foundation/Foundation.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

FLUTTER_ASSERT_ARC

NSString* const kDefaultAssetPath = @"Frameworks/App.framework/flutter_assets";

static NSString* GetFlutterAssetsPathFromBundle(NSBundle* bundle, NSString* relativeAssetsPath);

NSBundle* FLTFrameworkBundleInternal(NSString* flutterFrameworkBundleID, NSURL* searchURL) {
  NSDirectoryEnumerator<NSURL*>* frameworkEnumerator = [NSFileManager.defaultManager
                 enumeratorAtURL:searchURL
      includingPropertiesForKeys:nil
                         options:NSDirectoryEnumerationSkipsSubdirectoryDescendants |
                                 NSDirectoryEnumerationSkipsHiddenFiles
                    // Skip directories where errors are encountered.
                    errorHandler:nil];

  for (NSURL* candidate in frameworkEnumerator) {
    NSBundle* flutterFrameworkBundle = [NSBundle bundleWithURL:candidate];
    if ([flutterFrameworkBundle.bundleIdentifier isEqualToString:flutterFrameworkBundleID]) {
      return flutterFrameworkBundle;
    }
  }
  return nil;
}

NSBundle* FLTGetApplicationBundle() {
  NSBundle* mainBundle = NSBundle.mainBundle;
  // App extension bundle is in <AppName>.app/PlugIns/Extension.appex.
  if ([mainBundle.bundleURL.pathExtension isEqualToString:@"appex"]) {
    // Up two levels.
    return [NSBundle bundleWithURL:mainBundle.bundleURL.URLByDeletingLastPathComponent
                                       .URLByDeletingLastPathComponent];
  }
  return mainBundle;
}

NSBundle* FLTFrameworkBundleWithIdentifier(NSString* flutterFrameworkBundleID) {
  NSBundle* appBundle = FLTGetApplicationBundle();
  NSBundle* flutterFrameworkBundle =
      FLTFrameworkBundleInternal(flutterFrameworkBundleID, appBundle.privateFrameworksURL);
  if (flutterFrameworkBundle == nil) {
    // Fallback to slow implementation.
    flutterFrameworkBundle = [NSBundle bundleWithIdentifier:flutterFrameworkBundleID];
  }
  if (flutterFrameworkBundle == nil) {
    flutterFrameworkBundle = NSBundle.mainBundle;
  }
  return flutterFrameworkBundle;
}

NSString* FLTAssetPath(NSBundle* bundle) {
  return [bundle objectForInfoDictionaryKey:@"FLTAssetsPath"] ?: kDefaultAssetPath;
}

NSString* FLTAssetsPathFromBundle(NSBundle* bundle) {
  NSString* relativeAssetsPath = FLTAssetPath(bundle);
  NSString* flutterAssetsPath = GetFlutterAssetsPathFromBundle(bundle, relativeAssetsPath);
  if (flutterAssetsPath.length == 0) {
    flutterAssetsPath = GetFlutterAssetsPathFromBundle(NSBundle.mainBundle, relativeAssetsPath);
  }
  return flutterAssetsPath;
}

static NSString* GetFlutterAssetsPathFromBundle(NSBundle* bundle, NSString* relativeAssetsPath) {
  // Use the raw path solution so that asset path can be returned from unloaded bundles.
  // See https://github.com/flutter/engine/pull/46073
  NSString* assetsPath = [bundle pathForResource:relativeAssetsPath ofType:nil];
  if (assetsPath.length == 0) {
    // In app extension, using full relative path (kDefaultAssetPath)
    // returns nil when the app bundle is not loaded. Try to use
    // the sub folder name, which can successfully return a valid path.
    assetsPath = [bundle pathForResource:@"flutter_assets" ofType:nil];
  }
  return assetsPath;
}
