// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_MESSAGE_CODEC_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_MESSAGE_CODEC_H_

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

#include <glib-object.h>
#include <gmodule.h>

#include "fl_value.h"

G_BEGIN_DECLS

/**
 * FlMessageCodecError:
 * @FL_MESSAGE_CODEC_ERROR_FAILED: Codec failed due to an unspecified error.
 * @FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA: Codec ran out of data reading a value.
 * @FL_MESSAGE_CODEC_ERROR_ADDITIONAL_DATA: Additional data encountered in
 * message.
 * @FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE: Codec encountered an unsupported
 * #FlValue.
 *
 * Errors for #FlMessageCodec objects to set on failures.
 */
#define FL_MESSAGE_CODEC_ERROR fl_message_codec_error_quark()

typedef enum {
  FL_MESSAGE_CODEC_ERROR_FAILED,
  FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA,
  FL_MESSAGE_CODEC_ERROR_ADDITIONAL_DATA,
  FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE,
} FlMessageCodecError;

G_MODULE_EXPORT
GQuark fl_message_codec_error_quark(void) G_GNUC_CONST;

G_MODULE_EXPORT
G_DECLARE_DERIVABLE_TYPE(FlMessageCodec,
                         fl_message_codec,
                         FL,
                         MESSAGE_CODEC,
                         GObject)

/**
 * FlMessageCodec:
 *
 * #FlMessageCodec is a message encoding/decoding mechanism that operates on
 * #FlValue objects. Both operations returns errors if the conversion fails.
 * Such situations should be treated as programming errors.
 *
 * #FlMessageCodec matches the MethodCodec class in the Flutter services
 * library.
 */

struct _FlMessageCodecClass {
  GObjectClass parent_class;

  /**
   * FlMessageCodec::encode_message:
   * @codec: an #FlMessageCodec.
   * @message: message to encode or %NULL to encode the null value.
   * @error: (allow-none): #GError location to store the error occurring, or
   * %NULL.
   *
   * Virtual method to encode a message. A subclass must implement this method.
   * If the subclass cannot handle the type of @message then it must generate a
   * FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE error.
   *
   * Returns: a binary message or %NULL on error.
   */
  GBytes* (*encode_message)(FlMessageCodec* codec,
                            FlValue* message,
                            GError** error);

  /**
   * FlMessageCodec::decode_message:
   * @codec: an #FlMessageCodec.
   * @message: binary message to decode.
   * @error: (allow-none): #GError location to store the error occurring, or
   * %NULL.
   *
   * Virtual method to decode a message. A subclass must implement this method.
   * If @message is too small then a #FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA error
   * must be generated. If @message is too large then a
   * #FL_MESSAGE_CODEC_ERROR_ADDITIONAL_DATA error must be generated.
   *
   * Returns: an #FlValue or %NULL on error.
   */
  FlValue* (*decode_message)(FlMessageCodec* codec,
                             GBytes* message,
                             GError** error);
};

/**
 * fl_message_codec_encode_message:
 * @codec: an #FlMessageCodec.
 * @buffer: buffer to write to.
 * @message: message to encode or %NULL to encode the null value.
 * @error: (allow-none): #GError location to store the error occurring, or
 * %NULL.
 *
 * Encodes a message into a binary representation.
 *
 * Returns: a binary encoded message or %NULL on error.
 */
GBytes* fl_message_codec_encode_message(FlMessageCodec* codec,
                                        FlValue* message,
                                        GError** error);

/**
 * fl_message_codec_decode_message:
 * @codec: an #FlMessageCodec.
 * @message: binary message to decode.
 * @error: (allow-none): #GError location to store the error occurring, or
 * %NULL.
 *
 * Decodes a message from a binary encoding.
 *
 * Returns: an #FlValue or %NULL on error.
 */
FlValue* fl_message_codec_decode_message(FlMessageCodec* codec,
                                         GBytes* message,
                                         GError** error);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_MESSAGE_CODEC_H_
