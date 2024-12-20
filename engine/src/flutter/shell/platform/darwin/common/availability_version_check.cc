// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/darwin/common/availability_version_check.h"

#include <cstdint>
#include <optional>
#include <tuple>

#include <CoreFoundation/CoreFoundation.h>
#include <dispatch/dispatch.h>
#include <dlfcn.h>

#include "flutter/fml/build_config.h"
#include "flutter/fml/file.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/platform/darwin/cf_utils.h"

// The implementation of _availability_version_check defined in this file is
// based on the code in the clang-rt library at:
//
// https://github.com/llvm/llvm-project/blob/e315bf25a843582de39257e1345408a10dc08224/compiler-rt/lib/builtins/os_version_check.c
//
// Flutter provides its own implementation due to an issue introduced in recent
// versions of Clang following Clang 18 in which the clang-rt library declares
// weak linkage against the _availability_version_check symbol. This declaration
// causes apps to be rejected from the App Store. When Flutter statically links
// the implementation below, the weak linkage is satisfied at Engine build time,
// the symbol is no longer exposed from the Engine dylib, and apps will then
// not be rejected from the App Store.
//
// The implementation of _availability_version_check can delegate to the
// dynamically looked-up symbol on recent iOS versions, but the lookup will fail
// on iOS 11 and 12. When the lookup fails, the current OS version must be
// retrieved from a plist file at a well-known path. The logic for this below is
// copied from the clang-rt implementation and adapted for the Engine.

// See more context in https://github.com/flutter/flutter/issues/132130 and
// https://github.com/flutter/engine/pull/44711.

// TODO(zanderso): Remove this after Clang 18 rolls into Xcode.
// https://github.com/flutter/flutter/issues/133203.

#define CF_PROPERTY_LIST_IMMUTABLE 0

namespace flutter {

// This function parses the platform's version information out of a plist file
// at a well-known path. It parses the plist file using CoreFoundation functions
// to match the implementation in the clang-rt library.
std::optional<ProductVersion> ProductVersionFromSystemVersionPList() {
  std::string plist_path = "/System/Library/CoreServices/SystemVersion.plist";
#if FML_OS_IOS_SIMULATOR
  char* plist_path_prefix = getenv("IPHONE_SIMULATOR_ROOT");
  if (!plist_path_prefix) {
    FML_DLOG(ERROR) << "Failed to getenv IPHONE_SIMULATOR_ROOT";
    return std::nullopt;
  }
  plist_path = std::string(plist_path_prefix) + plist_path;
#endif  // FML_OS_IOS_SIMULATOR

  auto plist_mapping = fml::FileMapping::CreateReadOnly(plist_path);

  // Get the file buffer into CF's format. We pass in a null allocator here *
  // because we free PListBuf ourselves
  auto file_contents = fml::CFRef<CFDataRef>(CFDataCreateWithBytesNoCopy(
      nullptr, plist_mapping->GetMapping(),
      static_cast<CFIndex>(plist_mapping->GetSize()), kCFAllocatorNull));
  if (!file_contents) {
    FML_DLOG(ERROR) << "Failed to CFDataCreateWithBytesNoCopyFunc";
    return std::nullopt;
  }

  auto plist = fml::CFRef<CFDictionaryRef>(
      reinterpret_cast<CFDictionaryRef>(CFPropertyListCreateWithData(
          nullptr, file_contents, CF_PROPERTY_LIST_IMMUTABLE, nullptr,
          nullptr)));
  if (!plist) {
    FML_DLOG(ERROR) << "Failed to CFPropertyListCreateWithDataFunc or "
                       "CFPropertyListCreateFromXMLDataFunc";
    return std::nullopt;
  }

  auto product_version =
      fml::CFRef<CFStringRef>(CFStringCreateWithCStringNoCopy(
          nullptr, "ProductVersion", kCFStringEncodingASCII, kCFAllocatorNull));
  if (!product_version) {
    FML_DLOG(ERROR) << "Failed to CFStringCreateWithCStringNoCopyFunc";
    return std::nullopt;
  }
  CFTypeRef opaque_value = CFDictionaryGetValue(plist, product_version);
  if (!opaque_value || CFGetTypeID(opaque_value) != CFStringGetTypeID()) {
    FML_DLOG(ERROR) << "Failed to CFDictionaryGetValueFunc";
    return std::nullopt;
  }

  char version_str[32];
  if (!CFStringGetCString(reinterpret_cast<CFStringRef>(opaque_value),
                          version_str, sizeof(version_str),
                          kCFStringEncodingUTF8)) {
    FML_DLOG(ERROR) << "Failed to CFStringGetCStringFunc";
    return std::nullopt;
  }

  int32_t major = 0;
  int32_t minor = 0;
  int32_t subminor = 0;
  int matches = sscanf(version_str, "%d.%d.%d", &major, &minor, &subminor);
  // A major version number is sufficient. The minor and subminor numbers might
  // not be present.
  if (matches < 1) {
    FML_DLOG(ERROR) << "Failed to match product version string: "
                    << version_str;
    return std::nullopt;
  }

  return ProductVersion{major, minor, subminor};
}

bool IsEncodedVersionLessThanOrSame(uint32_t encoded_lhs, ProductVersion rhs) {
  // Parse the values out of encoded_lhs, then compare against rhs.
  const int32_t major = (encoded_lhs >> 16) & 0xffff;
  const int32_t minor = (encoded_lhs >> 8) & 0xff;
  const int32_t subminor = encoded_lhs & 0xff;
  auto lhs = ProductVersion{major, minor, subminor};

  return lhs <= rhs;
}

}  // namespace flutter

namespace {

// The host's OS version when the dynamic lookup of _availability_version_check
// has failed.
static flutter::ProductVersion g_version;

typedef uint32_t dyld_platform_t;

typedef struct {
  dyld_platform_t platform;
  uint32_t version;
} dyld_build_version_t;

typedef bool (*AvailabilityVersionCheckFn)(uint32_t count,
                                           dyld_build_version_t versions[]);

AvailabilityVersionCheckFn AvailabilityVersionCheck;

dispatch_once_t DispatchOnceCounter;

void InitializeAvailabilityCheck(void* unused) {
  if (AvailabilityVersionCheck) {
    return;
  }
  AvailabilityVersionCheck = reinterpret_cast<AvailabilityVersionCheckFn>(
      dlsym(RTLD_DEFAULT, "_availability_version_check"));
  if (AvailabilityVersionCheck) {
    return;
  }

  // If _availability_version_check can't be dynamically loaded, then version
  // information must be parsed out of a system plist file.
  auto product_version = flutter::ProductVersionFromSystemVersionPList();
  if (product_version.has_value()) {
    g_version = product_version.value();
  } else {
    // If reading version info out of the system plist file fails, then
    // fall back to the minimum version that Flutter supports.
#if FML_OS_IOS || FML_OS_IOS_SIMULATOR
    g_version = std::make_tuple(11, 0, 0);
#elif FML_OS_MACOSX
    g_version = std::make_tuple(10, 14, 0);
#endif  // FML_OS_MACOSX
  }
}

extern "C" bool _availability_version_check(uint32_t count,
                                            dyld_build_version_t versions[]) {
  dispatch_once_f(&DispatchOnceCounter, NULL, InitializeAvailabilityCheck);
  if (AvailabilityVersionCheck) {
    return AvailabilityVersionCheck(count, versions);
  }

  if (count == 0) {
    return true;
  }

  // This function is called in only one place in the clang-rt implementation
  // where there is only one element in the array.
  return flutter::IsEncodedVersionLessThanOrSame(versions[0].version,
                                                 g_version);
}

}  // namespace
