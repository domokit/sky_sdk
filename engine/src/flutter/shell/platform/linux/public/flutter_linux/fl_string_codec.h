// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_STRING_CODEC_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_STRING_CODEC_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <gmodule.h>

#include "fl_message_codec.h"

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlStringCodec,
                     fl_string_codec,
                     FL,
                     STRING_CODEC,
                     FlMessageCodec)

/**
 * FlStringCodec:
 *
 * #FlStringCodec is an #FlMessageCodec that implements the Flutter string
 * message encoding. This only encodes and decodes #FlValue of type
 * #FL_VALUE_TYPE_STRING, other types #FlValues will generate an error during
 * encoding.
 *
 * #FlStringCodec matches the StringCodec class in the Flutter services library.
 */

/**
 * fl_string_codec_new:
 *
 * Creates an #FlStringCodec.
 *
 * Returns: a new #FlStringCodec.
 */
FlStringCodec* fl_string_codec_new();

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_STRING_CODEC_H_
