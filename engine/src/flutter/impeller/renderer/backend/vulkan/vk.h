// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_VK_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_VK_H_

#include "flutter/fml/build_config.h"
#include "flutter/fml/logging.h"

#define VK_NO_PROTOTYPES

#if FML_OS_IOS

// #ifndef VK_USE_PLATFORM_IOS_MVK
// #define VK_USE_PLATFORM_IOS_MVK
// #endif  // VK_USE_PLATFORM_IOS_MVK

#ifndef VK_USE_PLATFORM_METAL_EXT
#define VK_USE_PLATFORM_METAL_EXT
#endif  // VK_USE_PLATFORM_METAL_EXT

#elif FML_OS_MACOSX

// #ifndef VK_USE_PLATFORM_MACOS_MVK
// #define VK_USE_PLATFORM_MACOS_MVK
// #endif  // VK_USE_PLATFORM_MACOS_MVK

#ifndef VK_USE_PLATFORM_METAL_EXT
#define VK_USE_PLATFORM_METAL_EXT
#endif  // VK_USE_PLATFORM_METAL_EXT

#elif FML_OS_ANDROID

#ifndef VK_USE_PLATFORM_ANDROID_KHR
#define VK_USE_PLATFORM_ANDROID_KHR
#endif  // VK_USE_PLATFORM_ANDROID_KHR

#elif FML_OS_LINUX

// Nothing for now.

#elif FML_OS_WIN

#ifndef VK_USE_PLATFORM_WIN32_KHR
#define VK_USE_PLATFORM_WIN32_KHR
#endif  // VK_USE_PLATFORM_WIN32_KHR

#elif OS_FUCHSIA

#ifndef VK_USE_PLATFORM_ANDROID_KHR
#define VK_USE_PLATFORM_ANDROID_KHR
#endif  // VK_USE_PLATFORM_ANDROID_KHR

#endif  // FML_OS

#if !defined(NDEBUG)
#define VULKAN_HPP_ASSERT FML_CHECK
#else
#define VULKAN_HPP_ASSERT(ignored) \
  {}
#endif

#define VULKAN_HPP_NAMESPACE impeller::vk
#define VULKAN_HPP_ASSERT_ON_RESULT(ignored) \
  { [[maybe_unused]] auto res = (ignored); }
#define VULKAN_HPP_NO_EXCEPTIONS

#include "vulkan/vulkan.hpp"  // IWYU pragma: keep.

static_assert(VK_HEADER_VERSION >= 215, "Vulkan headers must not be too old.");

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_VULKAN_VK_H_
