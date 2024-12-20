// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_BINARY_CODEC_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_BINARY_CODEC_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <gmodule.h>

#include "fl_message_codec.h"

G_BEGIN_DECLS

G_MODULE_EXPORT
G_DECLARE_FINAL_TYPE(FlBinaryCodec,
                     fl_binary_codec,
                     FL,
                     BINARY_CODEC,
                     FlMessageCodec)

/**
 * FlBinaryCodec:
 *
 * #FlBinaryCodec is an #FlMessageCodec that implements the Flutter binary
 * message encoding. This only encodes and decodes #FlValue of type
 * #FL_VALUE_TYPE_UINT8_LIST, other types #FlValues will generate an error
 * during encoding.
 *
 * #FlBinaryCodec matches the BinaryCodec class in the Flutter services
 * library.
 */

/**
 * fl_binary_codec_new:
 *
 * Creates an #FlBinaryCodec.
 *
 * Returns: a new #FlBinaryCodec.
 */
FlBinaryCodec* fl_binary_codec_new();

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_BINARY_CODEC_H_
