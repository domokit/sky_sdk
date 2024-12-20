// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_JSON_METHOD_CODEC_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_JSON_METHOD_CODEC_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <gmodule.h>

#include "fl_method_codec.h"

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlJsonMethodCodec,
                     fl_json_method_codec,
                     FL,
                     JSON_METHOD_CODEC,
                     FlMethodCodec)

/**
 * FlJsonMethodCodec:
 *
 * #FlJsonMessageCodec is an #FlMethodCodec that implements method calls using
 * the Flutter JSON message encoding. It should be used with an
 * #FlMethodChannel.
 *
 * #FlJsonMethodCodec matches the JSONMethodCodec class in the Flutter services
 * library.
 */

/**
 * fl_json_method_codec_new:
 *
 * Creates an #FlJsonMethodCodec.
 *
 * Returns: a new #FlJsonMethodCodec.
 */
FlJsonMethodCodec* fl_json_method_codec_new();

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_JSON_METHOD_CODEC_H_
