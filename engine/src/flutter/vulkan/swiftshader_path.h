// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_VULKAN_SWIFTSHADER_PATH_H_
#define FLUTTER_VULKAN_SWIFTSHADER_PATH_H_

#ifndef VULKAN_SO_PATH
#if FML_OS_MACOSX
#define VULKAN_SO_PATH "libvk_swiftshader.dylib"
#elif FML_OS_WIN
#define VULKAN_SO_PATH "vk_swiftshader.dll"
#else
#define VULKAN_SO_PATH "libvk_swiftshader.so"
#endif  // !FML_OS_MACOSX && !FML_OS_WIN
#endif  // VULKAN_SO_PATH

#endif  // FLUTTER_VULKAN_SWIFTSHADER_PATH_H_
