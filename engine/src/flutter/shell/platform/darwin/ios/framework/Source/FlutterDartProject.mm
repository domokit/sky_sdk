// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterDartProject_Internal.h"

#import <Metal/Metal.h>
#import <UIKit/UIKit.h>

#include <syslog.h>

#include "flutter/common/constants.h"
#include "flutter/fml/build_config.h"
#include "flutter/shell/common/switches.h"
#include "flutter/shell/platform/darwin/common/command_line.h"

FLUTTER_ASSERT_ARC

extern "C" {
#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG
// Used for debugging dart:* sources.
extern const uint8_t kPlatformStrongDill[];
extern const intptr_t kPlatformStrongDillSize;
#endif
}

static const char* kApplicationKernelSnapshotFileName = "kernel_blob.bin";

static BOOL DoesHardwareSupportWideGamut() {
  static BOOL result = NO;
  static dispatch_once_t once_token = 0;
  dispatch_once(&once_token, ^{
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (@available(iOS 13.0, *)) {
      // MTLGPUFamilyApple2 = A9/A10
      result = [device supportsFamily:MTLGPUFamilyApple2];
    } else {
      // A9/A10 on iOS 10+
      result = [device supportsFeatureSet:MTLFeatureSet_iOS_GPUFamily3_v2];
    }
  });
  return result;
}

flutter::Settings FLTDefaultSettingsForBundle(NSBundle* bundle, NSProcessInfo* processInfoOrNil) {
  auto command_line = flutter::CommandLineFromNSProcessInfo(processInfoOrNil);

  // Precedence:
  // 1. Settings from the specified NSBundle (except for enable-impeller).
  // 2. Settings passed explicitly via command-line arguments.
  // 3. Settings from the NSBundle with the default bundle ID.
  // 4. Settings from the main NSBundle and default values.

  NSBundle* mainBundle = FLTGetApplicationBundle();
  NSBundle* engineBundle = [NSBundle bundleForClass:[FlutterDartProject class]];

  bool hasExplicitBundle = bundle != nil;
  if (bundle == nil) {
    bundle = FLTFrameworkBundleWithIdentifier([FlutterDartProject defaultBundleIdentifier]);
  }

  auto settings = flutter::SettingsFromCommandLine(command_line);

  settings.task_observer_add = [](intptr_t key, const fml::closure& callback) {
    fml::MessageLoop::GetCurrent().AddTaskObserver(key, callback);
  };

  settings.task_observer_remove = [](intptr_t key) {
    fml::MessageLoop::GetCurrent().RemoveTaskObserver(key);
  };

  settings.log_message_callback = [](const std::string& tag, const std::string& message) {
    // TODO(cbracken): replace this with os_log-based approach.
    // https://github.com/flutter/flutter/issues/44030
    std::stringstream stream;
    if (!tag.empty()) {
      stream << tag << ": ";
    }
    stream << message;
    std::string log = stream.str();
    syslog(LOG_ALERT, "%.*s", (int)log.size(), log.c_str());
  };

  settings.enable_platform_isolates = true;

  // The command line arguments may not always be complete. If they aren't, attempt to fill in
  // defaults.

  // Flutter ships the ICU data file in the bundle of the engine. Look for it there.
  if (settings.icu_data_path.empty()) {
    NSString* icuDataPath = [engineBundle pathForResource:@"icudtl" ofType:@"dat"];
    if (icuDataPath.length > 0) {
      settings.icu_data_path = icuDataPath.UTF8String;
    }
  }

  if (flutter::DartVM::IsRunningPrecompiledCode()) {
    if (hasExplicitBundle) {
      NSString* executablePath = bundle.executablePath;
      if ([[NSFileManager defaultManager] fileExistsAtPath:executablePath]) {
        settings.application_library_path.push_back(executablePath.UTF8String);
      }
    }

    // No application bundle specified.  Try a known location from the main bundle's Info.plist.
    if (settings.application_library_path.empty()) {
      NSString* libraryName = [mainBundle objectForInfoDictionaryKey:@"FLTLibraryPath"];
      NSString* libraryPath = [mainBundle pathForResource:libraryName ofType:@""];
      if (libraryPath.length > 0) {
        NSString* executablePath = [NSBundle bundleWithPath:libraryPath].executablePath;
        if (executablePath.length > 0) {
          settings.application_library_path.push_back(executablePath.UTF8String);
        }
      }
    }

    // In case the application bundle is still not specified, look for the App.framework in the
    // Frameworks directory.
    if (settings.application_library_path.empty()) {
      NSString* applicationFrameworkPath = [mainBundle pathForResource:@"Frameworks/App.framework"
                                                                ofType:@""];
      if (applicationFrameworkPath.length > 0) {
        NSString* executablePath =
            [NSBundle bundleWithPath:applicationFrameworkPath].executablePath;
        if (executablePath.length > 0) {
          settings.application_library_path.push_back(executablePath.UTF8String);
        }
      }
    }
  }

  // Checks to see if the flutter assets directory is already present.
  if (settings.assets_path.empty()) {
    NSString* assetsPath = FLTAssetsPathFromBundle(bundle);

    if (assetsPath.length == 0) {
      NSLog(@"Failed to find assets path for \"%@\"", bundle);
    } else {
      settings.assets_path = assetsPath.UTF8String;

      // Check if there is an application kernel snapshot in the assets directory we could
      // potentially use.  Looking for the snapshot makes sense only if we have a VM that can use
      // it.
      if (!flutter::DartVM::IsRunningPrecompiledCode()) {
        NSURL* applicationKernelSnapshotURL =
            [NSURL URLWithString:@(kApplicationKernelSnapshotFileName)
                   relativeToURL:[NSURL fileURLWithPath:assetsPath]];
        NSError* error;
        if ([applicationKernelSnapshotURL checkResourceIsReachableAndReturnError:&error]) {
          settings.application_kernel_asset = applicationKernelSnapshotURL.path.UTF8String;
        } else {
          NSLog(@"Failed to find snapshot at %@: %@", applicationKernelSnapshotURL.path, error);
        }
      }
    }
  }

  // Domain network configuration
  // Disabled in https://github.com/flutter/flutter/issues/72723.
  // Re-enable in https://github.com/flutter/flutter/issues/54448.
  settings.may_insecurely_connect_to_all_domains = true;
  settings.domain_network_policy = "";

  // Whether to enable wide gamut colors.
#if TARGET_OS_SIMULATOR
  // As of Xcode 14.1, the wide gamut surface pixel formats are not supported by
  // the simulator.
  settings.enable_wide_gamut = false;
  // Removes unused function warning.
  (void)DoesHardwareSupportWideGamut;
#else
  NSNumber* nsEnableWideGamut = [mainBundle objectForInfoDictionaryKey:@"FLTEnableWideGamut"];
  BOOL enableWideGamut =
      (nsEnableWideGamut ? nsEnableWideGamut.boolValue : YES) && DoesHardwareSupportWideGamut();
  settings.enable_wide_gamut = enableWideGamut;
#endif

  settings.warn_on_impeller_opt_out = true;

  NSNumber* enableTraceSystrace = [mainBundle objectForInfoDictionaryKey:@"FLTTraceSystrace"];
  // Change the default only if the option is present.
  if (enableTraceSystrace != nil) {
    settings.trace_systrace = enableTraceSystrace.boolValue;
  }

  NSNumber* enableDartAsserts = [mainBundle objectForInfoDictionaryKey:@"FLTEnableDartAsserts"];
  if (enableDartAsserts != nil) {
    settings.dart_flags.push_back("--enable-asserts");
  }

  NSNumber* enableDartProfiling = [mainBundle objectForInfoDictionaryKey:@"FLTEnableDartProfiling"];
  // Change the default only if the option is present.
  if (enableDartProfiling != nil) {
    settings.enable_dart_profiling = enableDartProfiling.boolValue;
  }

  // Leak Dart VM settings, set whether leave or clean up the VM after the last shell shuts down.
  NSNumber* leakDartVM = [mainBundle objectForInfoDictionaryKey:@"FLTLeakDartVM"];
  // It will change the default leak_vm value in settings only if the key exists.
  if (leakDartVM != nil) {
    settings.leak_vm = leakDartVM.boolValue;
  }

  NSNumber* enableMergedPlatformUIThread =
      [mainBundle objectForInfoDictionaryKey:@"FLTEnableMergedPlatformUIThread"];
  if (enableMergedPlatformUIThread != nil) {
    settings.merged_platform_ui_thread = enableMergedPlatformUIThread.boolValue;
  }

#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG
  // There are no ownership concerns here as all mappings are owned by the
  // embedder and not the engine.
  auto make_mapping_callback = [](const uint8_t* mapping, size_t size) {
    return [mapping, size]() { return std::make_unique<fml::NonOwnedMapping>(mapping, size); };
  };

  settings.dart_library_sources_kernel =
      make_mapping_callback(kPlatformStrongDill, kPlatformStrongDillSize);
#endif  // FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG

  // If we even support setting this e.g. from the command line or the plist,
  // we should let the user override it.
  // Otherwise, we want to set this to a value that will avoid having the OS
  // kill us. On most iOS devices, that happens somewhere near half
  // the available memory.
  // The VM expects this value to be in megabytes.
  if (settings.old_gen_heap_size <= 0) {
    settings.old_gen_heap_size = std::round([NSProcessInfo processInfo].physicalMemory * .48 /
                                            flutter::kMegaByteSizeInBytes);
  }

  // This is the formula Android uses.
  // https://android.googlesource.com/platform/frameworks/base/+/39ae5bac216757bc201490f4c7b8c0f63006c6cd/libs/hwui/renderthread/CacheManager.cpp#45
  CGFloat scale = [UIScreen mainScreen].scale;
  CGFloat screenWidth = [UIScreen mainScreen].bounds.size.width * scale;
  CGFloat screenHeight = [UIScreen mainScreen].bounds.size.height * scale;
  settings.resource_cache_max_bytes_threshold = screenWidth * screenHeight * 12 * 4;

  // Whether to enable ios embedder api.
  NSNumber* enable_embedder_api =
      [mainBundle objectForInfoDictionaryKey:@"FLTEnableIOSEmbedderAPI"];
  // Change the default only if the option is present.
  if (enable_embedder_api) {
    settings.enable_embedder_api = enable_embedder_api.boolValue;
  }

  return settings;
}

@implementation FlutterDartProject {
  flutter::Settings _settings;
}

// This property is marked unavailable on iOS in the common header.
// That doesn't seem to be enough to prevent this property from being synthesized.
// Mark dynamic to avoid warnings.
@dynamic dartEntrypointArguments;

#pragma mark - Override base class designated initializers

- (instancetype)init {
  return [self initWithPrecompiledDartBundle:nil];
}

#pragma mark - Designated initializers

- (instancetype)initWithPrecompiledDartBundle:(nullable NSBundle*)bundle {
  self = [super init];

  if (self) {
    _settings = FLTDefaultSettingsForBundle(bundle);
  }

  return self;
}

- (instancetype)initWithSettings:(const flutter::Settings&)settings {
  self = [self initWithPrecompiledDartBundle:nil];

  if (self) {
    _settings = settings;
  }

  return self;
}

#pragma mark - PlatformData accessors

- (const flutter::PlatformData)defaultPlatformData {
  flutter::PlatformData PlatformData;
  PlatformData.lifecycle_state = std::string("AppLifecycleState.detached");
  return PlatformData;
}

#pragma mark - Settings accessors

- (const flutter::Settings&)settings {
  return _settings;
}

- (flutter::RunConfiguration)runConfiguration {
  return [self runConfigurationForEntrypoint:nil];
}

- (flutter::RunConfiguration)runConfigurationForEntrypoint:(nullable NSString*)entrypointOrNil {
  return [self runConfigurationForEntrypoint:entrypointOrNil libraryOrNil:nil];
}

- (flutter::RunConfiguration)runConfigurationForEntrypoint:(nullable NSString*)entrypointOrNil
                                              libraryOrNil:(nullable NSString*)dartLibraryOrNil {
  return [self runConfigurationForEntrypoint:entrypointOrNil
                                libraryOrNil:dartLibraryOrNil
                              entrypointArgs:nil];
}

- (flutter::RunConfiguration)runConfigurationForEntrypoint:(nullable NSString*)entrypointOrNil
                                              libraryOrNil:(nullable NSString*)dartLibraryOrNil
                                            entrypointArgs:
                                                (nullable NSArray<NSString*>*)entrypointArgs {
  auto config = flutter::RunConfiguration::InferFromSettings(_settings);
  if (dartLibraryOrNil && entrypointOrNil) {
    config.SetEntrypointAndLibrary(std::string([entrypointOrNil UTF8String]),
                                   std::string([dartLibraryOrNil UTF8String]));

  } else if (entrypointOrNil) {
    config.SetEntrypoint(std::string([entrypointOrNil UTF8String]));
  }

  if (entrypointArgs.count) {
    std::vector<std::string> cppEntrypointArgs;
    for (NSString* arg in entrypointArgs) {
      cppEntrypointArgs.push_back(std::string([arg UTF8String]));
    }
    config.SetEntrypointArgs(std::move(cppEntrypointArgs));
  }

  return config;
}

#pragma mark - Assets-related utilities

+ (NSString*)flutterAssetsName:(NSBundle*)bundle {
  if (bundle == nil) {
    bundle = FLTFrameworkBundleWithIdentifier([FlutterDartProject defaultBundleIdentifier]);
  }
  return FLTAssetPath(bundle);
}

+ (NSString*)domainNetworkPolicy:(NSDictionary*)appTransportSecurity {
  // https://developer.apple.com/documentation/bundleresources/information_property_list/nsapptransportsecurity/nsexceptiondomains
  NSDictionary* exceptionDomains = appTransportSecurity[@"NSExceptionDomains"];
  if (exceptionDomains == nil) {
    return @"";
  }
  NSMutableArray* networkConfigArray = [[NSMutableArray alloc] init];
  for (NSString* domain in exceptionDomains) {
    NSDictionary* domainConfiguration = exceptionDomains[domain];
    // Default value is false.
    bool includesSubDomains = [domainConfiguration[@"NSIncludesSubdomains"] boolValue];
    bool allowsCleartextCommunication =
        [domainConfiguration[@"NSExceptionAllowsInsecureHTTPLoads"] boolValue];
    [networkConfigArray addObject:@[
      domain, includesSubDomains ? @YES : @NO, allowsCleartextCommunication ? @YES : @NO
    ]];
  }
  NSData* jsonData = [NSJSONSerialization dataWithJSONObject:networkConfigArray
                                                     options:0
                                                       error:NULL];
  return [[NSString alloc] initWithData:jsonData encoding:NSUTF8StringEncoding];
}

+ (bool)allowsArbitraryLoads:(NSDictionary*)appTransportSecurity {
  return [appTransportSecurity[@"NSAllowsArbitraryLoads"] boolValue];
}

+ (NSString*)lookupKeyForAsset:(NSString*)asset {
  return [self lookupKeyForAsset:asset fromBundle:nil];
}

+ (NSString*)lookupKeyForAsset:(NSString*)asset fromBundle:(nullable NSBundle*)bundle {
  NSString* flutterAssetsName = [FlutterDartProject flutterAssetsName:bundle];
  return [NSString stringWithFormat:@"%@/%@", flutterAssetsName, asset];
}

+ (NSString*)lookupKeyForAsset:(NSString*)asset fromPackage:(NSString*)package {
  return [self lookupKeyForAsset:asset fromPackage:package fromBundle:nil];
}

+ (NSString*)lookupKeyForAsset:(NSString*)asset
                   fromPackage:(NSString*)package
                    fromBundle:(nullable NSBundle*)bundle {
  return [self lookupKeyForAsset:[NSString stringWithFormat:@"packages/%@/%@", package, asset]
                      fromBundle:bundle];
}

+ (NSString*)defaultBundleIdentifier {
  return @"io.flutter.flutter.app";
}

- (BOOL)isWideGamutEnabled {
  return _settings.enable_wide_gamut;
}

- (BOOL)isImpellerEnabled {
  return _settings.enable_impeller;
}

@end
