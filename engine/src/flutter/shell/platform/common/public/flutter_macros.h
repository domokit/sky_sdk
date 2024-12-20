// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_MACROS_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_MACROS_H_

#ifdef FLUTTER_DESKTOP_LIBRARY

// Do not add deprecation annotations when building the library.
#define FLUTTER_DEPRECATED(message)

#else  // FLUTTER_DESKTOP_LIBRARY

// Add deprecation warning for users of the library.
#ifdef _WIN32
#define FLUTTER_DEPRECATED(message) __declspec(deprecated(message))
#else
#define FLUTTER_DEPRECATED(message) __attribute__((deprecated(message)))
#endif

#endif  // FLUTTER_DESKTOP_LIBRARY

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_MACROS_H_
