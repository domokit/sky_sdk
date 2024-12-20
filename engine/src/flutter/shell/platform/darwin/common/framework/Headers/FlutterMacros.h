// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERMACROS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERMACROS_H_

#if defined(FLUTTER_FRAMEWORK)

#define FLUTTER_DARWIN_EXPORT __attribute__((visibility("default")))

#else  // defined(FLUTTER_SDK)

#define FLUTTER_DARWIN_EXPORT

#endif  // defined(FLUTTER_SDK)

#ifndef NS_ASSUME_NONNULL_BEGIN
#define NS_ASSUME_NONNULL_BEGIN _Pragma("clang assume_nonnull begin")
#define NS_ASSUME_NONNULL_END _Pragma("clang assume_nonnull end")
#endif  // defined(NS_ASSUME_NONNULL_BEGIN)

/**
 * Indicates that the API has been deprecated for the specified reason. Code
 * that uses the deprecated API will continue to work as before. However, the
 * API will soon become unavailable and users are encouraged to immediately take
 * the appropriate action mentioned in the deprecation message and the BREAKING
 * CHANGES section present in the Flutter.h umbrella header.
 */
#define FLUTTER_DEPRECATED(msg) __attribute__((__deprecated__(msg)))

/**
 * Indicates that the previously deprecated API is now unavailable. Code that
 * uses the API will not work and the declaration of the API is only a stub
 * meant to display the given message detailing the actions for the user to take
 * immediately.
 */
#define FLUTTER_UNAVAILABLE(msg) __attribute__((__unavailable__(msg)))

#if __has_feature(objc_arc)
#define FLUTTER_ASSERT_ARC
#define FLUTTER_ASSERT_NOT_ARC #error ARC must be disabled !
#else
#define FLUTTER_ASSERT_ARC #error ARC must be enabled !
#define FLUTTER_ASSERT_NOT_ARC
#endif

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_COMMON_FRAMEWORK_HEADERS_FLUTTERMACROS_H_
