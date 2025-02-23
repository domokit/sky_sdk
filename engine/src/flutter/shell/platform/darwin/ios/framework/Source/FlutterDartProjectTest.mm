// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#include "flutter/common/constants.h"
#include "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterDartProject_Internal.h"

FLUTTER_ASSERT_ARC

@interface FlutterDartProjectTest : XCTestCase
@end

@implementation FlutterDartProjectTest

- (void)setUp {
}

- (void)tearDown {
}

- (void)testOldGenHeapSizeSetting {
  FlutterDartProject* project = [[FlutterDartProject alloc] init];
  int64_t old_gen_heap_size =
      std::round([NSProcessInfo processInfo].physicalMemory * .48 / flutter::kMegaByteSizeInBytes);
  XCTAssertEqual(project.settings.old_gen_heap_size, old_gen_heap_size);
}

- (void)testResourceCacheMaxBytesThresholdSetting {
  FlutterDartProject* project = [[FlutterDartProject alloc] init];
  CGFloat scale = [UIScreen mainScreen].scale;
  CGFloat screenWidth = [UIScreen mainScreen].bounds.size.width * scale;
  CGFloat screenHeight = [UIScreen mainScreen].bounds.size.height * scale;
  size_t resource_cache_max_bytes_threshold = screenWidth * screenHeight * 12 * 4;
  XCTAssertEqual(project.settings.resource_cache_max_bytes_threshold,
                 resource_cache_max_bytes_threshold);
}

- (void)testMainBundleSettingsAreCorrectlyParsed {
  NSBundle* mainBundle = [NSBundle mainBundle];
  NSDictionary* appTransportSecurity =
      [mainBundle objectForInfoDictionaryKey:@"NSAppTransportSecurity"];
  XCTAssertTrue([FlutterDartProject allowsArbitraryLoads:appTransportSecurity]);
  XCTAssertEqualObjects(
      @"[[\"invalid-site.com\",true,false],[\"sub.invalid-site.com\",false,false]]",
      [FlutterDartProject domainNetworkPolicy:appTransportSecurity]);
}

- (void)testLeakDartVMSettingsAreCorrectlyParsed {
  // The FLTLeakDartVM's value is defined in Info.plist
  NSBundle* mainBundle = [NSBundle mainBundle];
  NSNumber* leakDartVM = [mainBundle objectForInfoDictionaryKey:@"FLTLeakDartVM"];
  XCTAssertEqual(leakDartVM.boolValue, NO);

  auto settings = FLTDefaultSettingsForBundle();
  // Check settings.leak_vm value is same as the value defined in Info.plist.
  XCTAssertEqual(settings.leak_vm, NO);
}

- (void)testFLTFrameworkBundleInternalWhenBundleIsNotPresent {
  NSBundle* found =
      FLTFrameworkBundleInternal(@"doesNotExist", NSBundle.mainBundle.privateFrameworksURL);
  XCTAssertNil(found);
}

- (void)testFLTFrameworkBundleInternalWhenBundleIsPresent {
  NSString* presentBundleID = @"io.flutter.flutter";
  NSBundle* found =
      FLTFrameworkBundleInternal(presentBundleID, NSBundle.mainBundle.privateFrameworksURL);
  XCTAssertNotNil(found);
}

- (void)testFLTGetApplicationBundleWhenCurrentTargetIsNotExtension {
  NSBundle* bundle = FLTGetApplicationBundle();
  XCTAssertEqual(bundle, [NSBundle mainBundle]);
}

- (void)testFLTGetApplicationBundleWhenCurrentTargetIsExtension {
  id mockMainBundle = OCMPartialMock([NSBundle mainBundle]);
  NSURL* url = [[NSBundle mainBundle].bundleURL URLByAppendingPathComponent:@"foo/ext.appex"];
  OCMStub([mockMainBundle bundleURL]).andReturn(url);
  NSBundle* bundle = FLTGetApplicationBundle();
  [mockMainBundle stopMocking];
  XCTAssertEqualObjects(bundle.bundleURL, [NSBundle mainBundle].bundleURL);
}

- (void)testFLTAssetsURLFromBundle {
  {
    // Found asset path in info.plist
    id mockBundle = OCMClassMock([NSBundle class]);
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(@"foo/assets");
    NSString* resultAssetsPath = @"path/to/foo/assets";
    OCMStub([mockBundle pathForResource:@"foo/assets" ofType:nil]).andReturn(resultAssetsPath);
    NSString* path = FLTAssetsPathFromBundle(mockBundle);
    XCTAssertEqualObjects(path, @"path/to/foo/assets");
  }
  {
    // Found asset path in info.plist, is not overriden by main bundle
    id mockBundle = OCMClassMock([NSBundle class]);
    id mockMainBundle = OCMPartialMock(NSBundle.mainBundle);
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(@"foo/assets");
    OCMStub([mockMainBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(nil);
    NSString* resultAssetsPath = @"path/to/foo/assets";
    OCMStub([mockBundle pathForResource:@"foo/assets" ofType:nil]).andReturn(resultAssetsPath);
    NSString* path = FLTAssetsPathFromBundle(mockBundle);
    XCTAssertEqualObjects(path, @"path/to/foo/assets");
    [mockMainBundle stopMocking];
  }
  {
    // No asset path in info.plist, defaults to main bundle
    id mockBundle = OCMClassMock([NSBundle class]);
    id mockMainBundle = OCMPartialMock([NSBundle mainBundle]);
    NSString* resultAssetsPath = @"path/to/foo/assets";
    OCMStub([mockBundle pathForResource:@"Frameworks/App.framework/flutter_assets" ofType:nil])
        .andReturn(nil);
    OCMStub([mockMainBundle pathForResource:@"Frameworks/App.framework/flutter_assets" ofType:nil])
        .andReturn(resultAssetsPath);
    NSString* path = FLTAssetsPathFromBundle(mockBundle);
    XCTAssertEqualObjects(path, @"path/to/foo/assets");
    [mockMainBundle stopMocking];
  }
}

- (void)testFLTAssetPathReturnsTheCorrectValue {
  {
    // Found assets path in info.plist
    id mockBundle = OCMClassMock([NSBundle class]);
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(@"foo/assets");
    XCTAssertEqualObjects(FLTAssetPath(mockBundle), @"foo/assets");
  }
  {
    // No assets path in info.plist, use default value
    id mockBundle = OCMClassMock([NSBundle class]);
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(nil);
    XCTAssertEqualObjects(FLTAssetPath(mockBundle), kDefaultAssetPath);
  }
}

- (void)testLookUpForAssets {
  {
    id mockBundle = OCMPartialMock([NSBundle mainBundle]);
    // Found assets path in info.plist
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(@"foo/assets");
    NSString* assetsPath = [FlutterDartProject lookupKeyForAsset:@"bar"];
    // This is testing public API, changing this assert is likely to break plugins.
    XCTAssertEqualObjects(assetsPath, @"foo/assets/bar");
    [mockBundle stopMocking];
  }
  {
    id mockBundle = OCMPartialMock([NSBundle mainBundle]);
    // No assets path in info.plist, use default value
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(nil);
    NSString* assetsPath = [FlutterDartProject lookupKeyForAsset:@"bar"];
    // This is testing public API, changing this assert is likely to break plugins.
    XCTAssertEqualObjects(assetsPath, @"Frameworks/App.framework/flutter_assets/bar");
    [mockBundle stopMocking];
  }
}

- (void)testLookUpForAssetsFromBundle {
  {
    id mockBundle = OCMClassMock([NSBundle class]);
    // Found assets path in info.plist
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(@"foo/assets");
    NSString* assetsPath = [FlutterDartProject lookupKeyForAsset:@"bar" fromBundle:mockBundle];
    // This is testing public API, changing this assert is likely to break plugins.
    XCTAssertEqualObjects(assetsPath, @"foo/assets/bar");
  }
  {
    // No assets path in info.plist, use default value
    id mockBundle = OCMClassMock([NSBundle class]);
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(nil);
    NSString* assetsPath = [FlutterDartProject lookupKeyForAsset:@"bar" fromBundle:mockBundle];
    // This is testing public API, changing this assert is likely to break plugins.
    XCTAssertEqualObjects(assetsPath, @"Frameworks/App.framework/flutter_assets/bar");
  }
}

- (void)testLookUpForAssetsFromPackage {
  {
    id mockBundle = OCMPartialMock([NSBundle mainBundle]);
    // Found assets path in info.plist
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(@"foo/assets");
    NSString* assetsPath = [FlutterDartProject lookupKeyForAsset:@"bar" fromPackage:@"bar_package"];
    // This is testing public API, changing this assert is likely to break plugins.
    XCTAssertEqualObjects(assetsPath, @"foo/assets/packages/bar_package/bar");
    [mockBundle stopMocking];
  }
  {
    id mockBundle = OCMPartialMock([NSBundle mainBundle]);
    // No assets path in info.plist, use default value
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(nil);
    NSString* assetsPath = [FlutterDartProject lookupKeyForAsset:@"bar" fromPackage:@"bar_package"];
    // This is testing public API, changing this assert is likely to break plugins.
    XCTAssertEqualObjects(assetsPath,
                          @"Frameworks/App.framework/flutter_assets/packages/bar_package/bar");
    [mockBundle stopMocking];
  }
}

- (void)testLookUpForAssetsFromPackageFromBundle {
  {
    id mockBundle = OCMClassMock([NSBundle class]);
    // Found assets path in info.plist
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(@"foo/assets");
    NSString* assetsPath = [FlutterDartProject lookupKeyForAsset:@"bar"
                                                     fromPackage:@"bar_package"
                                                      fromBundle:mockBundle];
    // This is testing public API, changing this assert is likely to break plugins.
    XCTAssertEqualObjects(assetsPath, @"foo/assets/packages/bar_package/bar");
  }
  {
    id mockBundle = OCMClassMock([NSBundle class]);
    // No assets path in info.plist, use default value
    OCMStub([mockBundle objectForInfoDictionaryKey:@"FLTAssetsPath"]).andReturn(nil);
    NSString* assetsPath = [FlutterDartProject lookupKeyForAsset:@"bar"
                                                     fromPackage:@"bar_package"
                                                      fromBundle:mockBundle];
    // This is testing public API, changing this assert is likely to break plugins.
    XCTAssertEqualObjects(assetsPath,
                          @"Frameworks/App.framework/flutter_assets/packages/bar_package/bar");
  }
}

- (void)testEnableDartAssertsCommandLineArgument {
  id mockMainBundle = OCMPartialMock([NSBundle mainBundle]);
  OCMStub([mockMainBundle objectForInfoDictionaryKey:@"FLTEnableDartAsserts"]).andReturn(@"YES");
  id mockProcessInfo = OCMPartialMock([NSProcessInfo processInfo]);
  NSArray* arguments = @[ @"process_name" ];
  OCMStub([mockProcessInfo arguments]).andReturn(arguments);

  auto settings = FLTDefaultSettingsForBundle(nil, mockProcessInfo);

  XCTAssertEqual(settings.dart_flags.size(), 1u);
  XCTAssertEqual(settings.dart_flags[0], "--enable-asserts");
  [mockMainBundle stopMocking];
}

- (void)testEnableTraceSystraceSettingIsCorrectlyParsed {
  NSBundle* mainBundle = [NSBundle mainBundle];
  NSNumber* enableTraceSystrace = [mainBundle objectForInfoDictionaryKey:@"FLTTraceSystrace"];
  XCTAssertNotNil(enableTraceSystrace);
  XCTAssertEqual(enableTraceSystrace.boolValue, NO);
  auto settings = FLTDefaultSettingsForBundle();
  XCTAssertEqual(settings.trace_systrace, NO);
}

- (void)testEnableDartProflingSettingIsCorrectlyParsed {
  NSBundle* mainBundle = [NSBundle mainBundle];
  NSNumber* enableTraceSystrace = [mainBundle objectForInfoDictionaryKey:@"FLTEnableDartProfiling"];
  XCTAssertNotNil(enableTraceSystrace);
  XCTAssertEqual(enableTraceSystrace.boolValue, NO);
  auto settings = FLTDefaultSettingsForBundle();
  XCTAssertEqual(settings.trace_systrace, NO);
}

- (void)testEmptySettingsAreCorrect {
  XCTAssertFalse([FlutterDartProject allowsArbitraryLoads:[[NSDictionary alloc] init]]);
  XCTAssertEqualObjects(@"", [FlutterDartProject domainNetworkPolicy:[[NSDictionary alloc] init]]);
}

- (void)testAllowsArbitraryLoads {
  XCTAssertFalse([FlutterDartProject allowsArbitraryLoads:@{@"NSAllowsArbitraryLoads" : @false}]);
  XCTAssertTrue([FlutterDartProject allowsArbitraryLoads:@{@"NSAllowsArbitraryLoads" : @true}]);
}

- (void)testProperlyFormedExceptionDomains {
  NSDictionary* domainInfoOne = @{
    @"NSIncludesSubdomains" : @false,
    @"NSExceptionAllowsInsecureHTTPLoads" : @true,
    @"NSExceptionMinimumTLSVersion" : @"4.0"
  };
  NSDictionary* domainInfoTwo = @{
    @"NSIncludesSubdomains" : @true,
    @"NSExceptionAllowsInsecureHTTPLoads" : @false,
    @"NSExceptionMinimumTLSVersion" : @"4.0"
  };
  NSDictionary* domainInfoThree = @{
    @"NSIncludesSubdomains" : @false,
    @"NSExceptionAllowsInsecureHTTPLoads" : @true,
    @"NSExceptionMinimumTLSVersion" : @"4.0"
  };
  NSDictionary* exceptionDomains = @{
    @"domain.name" : domainInfoOne,
    @"sub.domain.name" : domainInfoTwo,
    @"sub.two.domain.name" : domainInfoThree
  };
  NSDictionary* appTransportSecurity = @{@"NSExceptionDomains" : exceptionDomains};
  XCTAssertEqualObjects(@"[[\"domain.name\",false,true],[\"sub.domain.name\",true,false],"
                        @"[\"sub.two.domain.name\",false,true]]",
                        [FlutterDartProject domainNetworkPolicy:appTransportSecurity]);
}

- (void)testExceptionDomainsWithMissingInfo {
  NSDictionary* domainInfoOne = @{@"NSExceptionMinimumTLSVersion" : @"4.0"};
  NSDictionary* domainInfoTwo = @{
    @"NSIncludesSubdomains" : @true,
  };
  NSDictionary* domainInfoThree = @{};
  NSDictionary* exceptionDomains = @{
    @"domain.name" : domainInfoOne,
    @"sub.domain.name" : domainInfoTwo,
    @"sub.two.domain.name" : domainInfoThree
  };
  NSDictionary* appTransportSecurity = @{@"NSExceptionDomains" : exceptionDomains};
  XCTAssertEqualObjects(@"[[\"domain.name\",false,false],[\"sub.domain.name\",true,false],"
                        @"[\"sub.two.domain.name\",false,false]]",
                        [FlutterDartProject domainNetworkPolicy:appTransportSecurity]);
}

@end
