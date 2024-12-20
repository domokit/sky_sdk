// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_EXPORT_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_EXPORT_H_

#ifdef FLUTTER_DESKTOP_LIBRARY

// Add visibility/export annotations when building the library.
#ifdef _WIN32
#define FLUTTER_EXPORT __declspec(dllexport)
#else
#define FLUTTER_EXPORT __attribute__((visibility("default")))
#endif

#else  // FLUTTER_DESKTOP_LIBRARY

// Add import annotations when consuming the library.
#ifdef _WIN32
#define FLUTTER_EXPORT __declspec(dllimport)
#else
#define FLUTTER_EXPORT
#endif

#endif  // FLUTTER_DESKTOP_LIBRARY

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_EXPORT_H_
