// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_FL_METHOD_CODEC_PRIVATE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_FL_METHOD_CODEC_PRIVATE_H_

#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_codec.h"
#include "flutter/shell/platform/linux/public/flutter_linux/fl_method_response.h"

G_BEGIN_DECLS

/**
 * fl_method_codec_encode_method_call:
 * @codec: an #FlMethodCodec.
 * @name: method name.
 * @args: (allow-none): method arguments, or %NULL.
 * @error: (allow-none): #GError location to store the error occurring, or
 * %NULL.
 *
 * Encodes a method call.
 *
 * Returns: (transfer full): a binary encoding of this method call or %NULL if
 * not able to encode.
 */
GBytes* fl_method_codec_encode_method_call(FlMethodCodec* codec,
                                           const gchar* name,
                                           FlValue* args,
                                           GError** error);

/**
 * fl_method_codec_decode_method_call:
 * @codec: an #FlMethodCodec.
 * @message: message to decode.
 * @name: (transfer full): location to write method name or %NULL if not
 * required.
 * @args: (transfer full): location to write method arguments, or %NULL if not
 * required.
 * @error: (allow-none): #GError location to store the error occurring, or
 * %NULL.
 *
 * Decodes a method call.
 *
 * Returns: %TRUE if successfully decoded.
 */
gboolean fl_method_codec_decode_method_call(FlMethodCodec* codec,
                                            GBytes* message,
                                            gchar** name,
                                            FlValue** args,
                                            GError** error);

/**
 * fl_method_codec_encode_success_envelope:
 * @codec: an #FlMethodCodec.
 * @result: (allow-none): method result, or %NULL.
 * @error: (allow-none): #GError location to store the error occurring, or
 * %NULL.
 *
 * Encodes a successful response to a method call.
 *
 * Returns: (transfer full): a binary encoding of this response or %NULL if not
 * able to encode.
 */
GBytes* fl_method_codec_encode_success_envelope(FlMethodCodec* codec,
                                                FlValue* result,
                                                GError** error);

/**
 * fl_method_codec_encode_error_envelope:
 * @codec: an #FlMethodCodec.
 * @code: an error code.
 * @message: (allow-none): an error message or %NULL.
 * @details: (allow-none): error details, or %NULL.
 * @error: (allow-none): #GError location to store the error occurring, or
 * %NULL.
 *
 * Encodes an error response to a method call.
 *
 * Returns: (transfer full): a binary encoding of this response or %NULL if not
 * able to encode.
 */
GBytes* fl_method_codec_encode_error_envelope(FlMethodCodec* codec,
                                              const gchar* code,
                                              const gchar* message,
                                              FlValue* details,
                                              GError** error);

/**
 * fl_method_codec_encode_response:
 * @codec: an #FlMethodCodec.
 * @response: response to encode.
 * @error: (allow-none): #GError location to store the error occurring, or
 * %NULL.
 *
 * Encodes a response to a method call.
 *
 * Returns: a new #FlMethodResponse or %NULL on error.
 */
GBytes* fl_method_codec_encode_response(FlMethodCodec* codec,
                                        FlMethodResponse* response,
                                        GError** error);

/**
 * fl_method_codec_decode_response:
 * @codec: an #FlMethodCodec.
 * @message: message to decode.
 * @error: (allow-none): #GError location to store the error occurring, or
 * %NULL.
 *
 * Decodes a response to a method call.
 *
 * Returns: a new #FlMethodResponse or %NULL on error.
 */
FlMethodResponse* fl_method_codec_decode_response(FlMethodCodec* codec,
                                                  GBytes* message,
                                                  GError** error);

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_FL_METHOD_CODEC_PRIVATE_H_
