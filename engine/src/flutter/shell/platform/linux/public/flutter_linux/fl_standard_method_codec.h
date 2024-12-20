// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_STANDARD_METHOD_CODEC_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_STANDARD_METHOD_CODEC_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <gmodule.h>

#include "fl_method_codec.h"
#include "fl_standard_message_codec.h"

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlStandardMethodCodec,
                     fl_standard_method_codec,
                     FL,
                     STANDARD_METHOD_CODEC,
                     FlMethodCodec)

/**
 * FlStandardMethodCodec:
 *
 * #FlStandardMethodCodec is an #FlMethodCodec that implements method calls
 * using the Flutter standard message encoding. It should be used with a
 * #FlMethodChannel.
 *
 * #FlStandardMethodCodec matches the StandardMethodCodec class in the Flutter
 * services library.
 */

/**
 * fl_standard_method_codec_new:
 *
 * Creates an #FlStandardMethodCodec.
 *
 * Returns: a new #FlStandardMethodCodec.
 */
FlStandardMethodCodec* fl_standard_method_codec_new();

/**
 * fl_standard_method_codec_new:
 * @message_codec: A #FlMessageCodec.
 *
 * Creates an #FlStandardMethodCodec with a custom message codec.
 *
 * Returns: a new #FlStandardMethodCodec.
 */
FlStandardMethodCodec* fl_standard_method_codec_new_with_message_codec(
    FlStandardMessageCodec* message_codec);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_STANDARD_METHOD_CODEC_H_
