// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_standard_message_codec.h"

#include <gmodule.h>

#include <cstring>

// See lib/src/services/message_codecs.dart in Flutter source for description of
// encoding.

// Type values.
static constexpr int kValueNull = 0;
static constexpr int kValueTrue = 1;
static constexpr int kValueFalse = 2;
static constexpr int kValueInt32 = 3;
static constexpr int kValueInt64 = 4;
static constexpr int kValueFloat64 = 6;
static constexpr int kValueString = 7;
static constexpr int kValueUint8List = 8;
static constexpr int kValueInt32List = 9;
static constexpr int kValueInt64List = 10;
static constexpr int kValueFloat64List = 11;
static constexpr int kValueList = 12;
static constexpr int kValueMap = 13;
static constexpr int kValueFloat32List = 14;

G_DEFINE_TYPE(FlStandardMessageCodec,
              fl_standard_message_codec,
              fl_message_codec_get_type())

// Functions to write standard C number types.

static void write_uint8(GByteArray* buffer, uint8_t value) {
  g_byte_array_append(buffer, &value, sizeof(uint8_t));
}

static void write_uint16(GByteArray* buffer, uint16_t value) {
  g_byte_array_append(buffer, reinterpret_cast<uint8_t*>(&value),
                      sizeof(uint16_t));
}

static void write_uint32(GByteArray* buffer, uint32_t value) {
  g_byte_array_append(buffer, reinterpret_cast<uint8_t*>(&value),
                      sizeof(uint32_t));
}

static void write_int32(GByteArray* buffer, int32_t value) {
  g_byte_array_append(buffer, reinterpret_cast<uint8_t*>(&value),
                      sizeof(int32_t));
}

static void write_int64(GByteArray* buffer, int64_t value) {
  g_byte_array_append(buffer, reinterpret_cast<uint8_t*>(&value),
                      sizeof(int64_t));
}

static void write_float64(GByteArray* buffer, double value) {
  g_byte_array_append(buffer, reinterpret_cast<uint8_t*>(&value),
                      sizeof(double));
}

// Write padding bytes to align to @align multiple of bytes.
static void write_align(GByteArray* buffer, guint align) {
  while (buffer->len % align != 0) {
    write_uint8(buffer, 0);
  }
}

// Checks there is enough data in @buffer to be read.
static gboolean check_size(GBytes* buffer,
                           size_t offset,
                           size_t required,
                           GError** error) {
  if (offset + required > g_bytes_get_size(buffer)) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                FL_MESSAGE_CODEC_ERROR_OUT_OF_DATA, "Unexpected end of data");
    return FALSE;
  }
  return TRUE;
}

// Skip bytes to align next read on @align multiple of bytes.
static gboolean read_align(GBytes* buffer,
                           size_t* offset,
                           size_t align,
                           GError** error) {
  if ((*offset) % align == 0) {
    return TRUE;
  }

  size_t required = align - (*offset) % align;
  if (!check_size(buffer, *offset, required, error)) {
    return FALSE;
  }

  (*offset) += required;
  return TRUE;
}

// Gets a pointer to the given offset in @buffer.
static const uint8_t* get_data(GBytes* buffer, size_t* offset) {
  return static_cast<const uint8_t*>(g_bytes_get_data(buffer, nullptr)) +
         *offset;
}

// Reads an unsigned 8 bit number from @buffer and writes it to @value.
// Returns TRUE if successful, otherwise sets an error.
static gboolean read_uint8(GBytes* buffer,
                           size_t* offset,
                           uint8_t* value,
                           GError** error) {
  if (!check_size(buffer, *offset, sizeof(uint8_t), error)) {
    return FALSE;
  }

  *value = get_data(buffer, offset)[0];
  (*offset)++;
  return TRUE;
}

// Reads an unsigned 16 bit integer from @buffer and writes it to @value.
// Returns TRUE if successful, otherwise sets an error.
static gboolean read_uint16(GBytes* buffer,
                            size_t* offset,
                            uint16_t* value,
                            GError** error) {
  if (!check_size(buffer, *offset, sizeof(uint16_t), error)) {
    return FALSE;
  }

  *value = reinterpret_cast<const uint16_t*>(get_data(buffer, offset))[0];
  *offset += sizeof(uint16_t);
  return TRUE;
}

// Reads an unsigned 32 bit integer from @buffer and writes it to @value.
// Returns TRUE if successful, otherwise sets an error.
static gboolean read_uint32(GBytes* buffer,
                            size_t* offset,
                            uint32_t* value,
                            GError** error) {
  if (!check_size(buffer, *offset, sizeof(uint32_t), error)) {
    return FALSE;
  }

  *value = reinterpret_cast<const uint32_t*>(get_data(buffer, offset))[0];
  *offset += sizeof(uint32_t);
  return TRUE;
}

// Reads a #FL_VALUE_TYPE_INT stored as a signed 32 bit integer from @buffer.
// Returns a new #FlValue of type #FL_VALUE_TYPE_INT if successful or %NULL on
// error.
static FlValue* read_int32_value(GBytes* buffer,
                                 size_t* offset,
                                 GError** error) {
  if (!check_size(buffer, *offset, sizeof(int32_t), error)) {
    return nullptr;
  }

  FlValue* value = fl_value_new_int(
      reinterpret_cast<const int32_t*>(get_data(buffer, offset))[0]);
  *offset += sizeof(int32_t);
  return value;
}

// Reads a #FL_VALUE_TYPE_INT stored as a signed 64 bit integer from @buffer.
// Returns a new #FlValue of type #FL_VALUE_TYPE_INT if successful or %NULL on
// error.
static FlValue* read_int64_value(GBytes* buffer,
                                 size_t* offset,
                                 GError** error) {
  if (!check_size(buffer, *offset, sizeof(int64_t), error)) {
    return nullptr;
  }

  FlValue* value = fl_value_new_int(
      reinterpret_cast<const int64_t*>(get_data(buffer, offset))[0]);
  *offset += sizeof(int64_t);
  return value;
}

// Reads a 64 bit floating point number from @buffer and writes it to @value.
// Returns a new #FlValue of type #FL_VALUE_TYPE_FLOAT if successful or %NULL on
// error.
static FlValue* read_float64_value(GBytes* buffer,
                                   size_t* offset,
                                   GError** error) {
  if (!read_align(buffer, offset, 8, error)) {
    return nullptr;
  }
  if (!check_size(buffer, *offset, sizeof(double), error)) {
    return nullptr;
  }

  FlValue* value = fl_value_new_float(
      reinterpret_cast<const double*>(get_data(buffer, offset))[0]);
  *offset += sizeof(double);
  return value;
}

// Reads an UTF-8 text string from @buffer in standard codec format.
// Returns a new #FlValue of type #FL_VALUE_TYPE_STRING if successful or %NULL
// on error.
static FlValue* read_string_value(FlStandardMessageCodec* self,
                                  GBytes* buffer,
                                  size_t* offset,
                                  GError** error) {
  uint32_t length;
  if (!fl_standard_message_codec_read_size(self, buffer, offset, &length,
                                           error)) {
    return nullptr;
  }
  if (!check_size(buffer, *offset, length, error)) {
    return nullptr;
  }
  FlValue* value = fl_value_new_string_sized(
      reinterpret_cast<const gchar*>(get_data(buffer, offset)), length);
  *offset += length;
  return value;
}

// Reads an unsigned 8 bit list from @buffer in standard codec format.
// Returns a new #FlValue of type #FL_VALUE_TYPE_UINT8_LIST if successful or
// %NULL on error.
static FlValue* read_uint8_list_value(FlStandardMessageCodec* self,
                                      GBytes* buffer,
                                      size_t* offset,
                                      GError** error) {
  uint32_t length;
  if (!fl_standard_message_codec_read_size(self, buffer, offset, &length,
                                           error)) {
    return nullptr;
  }
  if (!check_size(buffer, *offset, sizeof(uint8_t) * length, error)) {
    return nullptr;
  }
  FlValue* value = fl_value_new_uint8_list(get_data(buffer, offset), length);
  *offset += length;
  return value;
}

// Reads a signed 32 bit list from @buffer in standard codec format.
// Returns a new #FlValue of type #FL_VALUE_TYPE_INT32_LIST if successful or
// %NULL on error.
static FlValue* read_int32_list_value(FlStandardMessageCodec* self,
                                      GBytes* buffer,
                                      size_t* offset,
                                      GError** error) {
  uint32_t length;
  if (!fl_standard_message_codec_read_size(self, buffer, offset, &length,
                                           error)) {
    return nullptr;
  }
  if (!read_align(buffer, offset, 4, error)) {
    return nullptr;
  }
  if (!check_size(buffer, *offset, sizeof(int32_t) * length, error)) {
    return nullptr;
  }
  FlValue* value = fl_value_new_int32_list(
      reinterpret_cast<const int32_t*>(get_data(buffer, offset)), length);
  *offset += sizeof(int32_t) * length;
  return value;
}

// Reads a signed 64 bit list from @buffer in standard codec format.
// Returns a new #FlValue of type #FL_VALUE_TYPE_INT64_LIST if successful or
// %NULL on error.
static FlValue* read_int64_list_value(FlStandardMessageCodec* self,
                                      GBytes* buffer,
                                      size_t* offset,
                                      GError** error) {
  uint32_t length;
  if (!fl_standard_message_codec_read_size(self, buffer, offset, &length,
                                           error)) {
    return nullptr;
  }
  if (!read_align(buffer, offset, 8, error)) {
    return nullptr;
  }
  if (!check_size(buffer, *offset, sizeof(int64_t) * length, error)) {
    return nullptr;
  }
  FlValue* value = fl_value_new_int64_list(
      reinterpret_cast<const int64_t*>(get_data(buffer, offset)), length);
  *offset += sizeof(int64_t) * length;
  return value;
}

// Reads a 32 bit floating point number list from @buffer in standard codec
// format. Returns a new #FlValue of type #FL_VALUE_TYPE_FLOAT32_LIST if
// successful or %NULL on error.
static FlValue* read_float32_list_value(FlStandardMessageCodec* self,
                                        GBytes* buffer,
                                        size_t* offset,
                                        GError** error) {
  uint32_t length;
  if (!fl_standard_message_codec_read_size(self, buffer, offset, &length,
                                           error)) {
    return nullptr;
  }
  if (!read_align(buffer, offset, 4, error)) {
    return nullptr;
  }
  if (!check_size(buffer, *offset, sizeof(float) * length, error)) {
    return nullptr;
  }
  FlValue* value = fl_value_new_float32_list(
      reinterpret_cast<const float*>(get_data(buffer, offset)), length);
  *offset += sizeof(float) * length;
  return value;
}

// Reads a floating point number list from @buffer in standard codec format.
// Returns a new #FlValue of type #FL_VALUE_TYPE_FLOAT_LIST if successful or
// %NULL on error.
static FlValue* read_float64_list_value(FlStandardMessageCodec* self,
                                        GBytes* buffer,
                                        size_t* offset,
                                        GError** error) {
  uint32_t length;
  if (!fl_standard_message_codec_read_size(self, buffer, offset, &length,
                                           error)) {
    return nullptr;
  }
  if (!read_align(buffer, offset, 8, error)) {
    return nullptr;
  }
  if (!check_size(buffer, *offset, sizeof(double) * length, error)) {
    return nullptr;
  }
  FlValue* value = fl_value_new_float_list(
      reinterpret_cast<const double*>(get_data(buffer, offset)), length);
  *offset += sizeof(double) * length;
  return value;
}

// Reads a list from @buffer in standard codec format.
// Returns a new #FlValue of type #FL_VALUE_TYPE_LIST if successful or %NULL on
// error.
static FlValue* read_list_value(FlStandardMessageCodec* self,
                                GBytes* buffer,
                                size_t* offset,
                                GError** error) {
  uint32_t length;
  if (!fl_standard_message_codec_read_size(self, buffer, offset, &length,
                                           error)) {
    return nullptr;
  }

  g_autoptr(FlValue) list = fl_value_new_list();
  for (size_t i = 0; i < length; i++) {
    g_autoptr(FlValue) child =
        fl_standard_message_codec_read_value(self, buffer, offset, error);
    if (child == nullptr) {
      return nullptr;
    }
    fl_value_append(list, child);
  }

  return fl_value_ref(list);
}

// Reads a map from @buffer in standard codec format.
// Returns a new #FlValue of type #FL_VALUE_TYPE_MAP if successful or %NULL on
// error.
static FlValue* read_map_value(FlStandardMessageCodec* self,
                               GBytes* buffer,
                               size_t* offset,
                               GError** error) {
  uint32_t length;
  if (!fl_standard_message_codec_read_size(self, buffer, offset, &length,
                                           error)) {
    return nullptr;
  }

  g_autoptr(FlValue) map = fl_value_new_map();
  for (size_t i = 0; i < length; i++) {
    g_autoptr(FlValue) key =
        fl_standard_message_codec_read_value(self, buffer, offset, error);
    if (key == nullptr) {
      return nullptr;
    }
    g_autoptr(FlValue) value =
        fl_standard_message_codec_read_value(self, buffer, offset, error);
    if (value == nullptr) {
      return nullptr;
    }
    fl_value_set(map, key, value);
  }

  return fl_value_ref(map);
}

// Implements FlMessageCodec::encode_message.
static GBytes* fl_standard_message_codec_encode_message(FlMessageCodec* codec,
                                                        FlValue* message,
                                                        GError** error) {
  FlStandardMessageCodec* self =
      reinterpret_cast<FlStandardMessageCodec*>(codec);

  g_autoptr(GByteArray) buffer = g_byte_array_new();
  if (!fl_standard_message_codec_write_value(self, buffer, message, error)) {
    return nullptr;
  }
  return g_byte_array_free_to_bytes(
      static_cast<GByteArray*>(g_steal_pointer(&buffer)));
}

// Implements FlMessageCodec::decode_message.
static FlValue* fl_standard_message_codec_decode_message(FlMessageCodec* codec,
                                                         GBytes* message,
                                                         GError** error) {
  if (g_bytes_get_size(message) == 0) {
    return fl_value_new_null();
  }

  FlStandardMessageCodec* self =
      reinterpret_cast<FlStandardMessageCodec*>(codec);

  size_t offset = 0;
  g_autoptr(FlValue) value =
      fl_standard_message_codec_read_value(self, message, &offset, error);
  if (value == nullptr) {
    return nullptr;
  }

  if (offset != g_bytes_get_size(message)) {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                FL_MESSAGE_CODEC_ERROR_ADDITIONAL_DATA,
                "Unused %zi bytes after standard message",
                g_bytes_get_size(message) - offset);
    return nullptr;
  }

  return fl_value_ref(value);
}

// Implements FlStandardMessageCodec::write_value.
static gboolean fl_standard_message_codec_real_write_value(
    FlStandardMessageCodec* self,
    GByteArray* buffer,
    FlValue* value,
    GError** error) {
  if (value == nullptr) {
    write_uint8(buffer, kValueNull);
    return TRUE;
  }

  switch (fl_value_get_type(value)) {
    case FL_VALUE_TYPE_NULL:
      write_uint8(buffer, kValueNull);
      return TRUE;
    case FL_VALUE_TYPE_BOOL:
      if (fl_value_get_bool(value)) {
        write_uint8(buffer, kValueTrue);
      } else {
        write_uint8(buffer, kValueFalse);
      }
      return TRUE;
    case FL_VALUE_TYPE_INT: {
      int64_t v = fl_value_get_int(value);
      if (v >= INT32_MIN && v <= INT32_MAX) {
        write_uint8(buffer, kValueInt32);
        write_int32(buffer, v);
      } else {
        write_uint8(buffer, kValueInt64);
        write_int64(buffer, v);
      }
      return TRUE;
    }
    case FL_VALUE_TYPE_FLOAT:
      write_uint8(buffer, kValueFloat64);
      write_align(buffer, 8);
      write_float64(buffer, fl_value_get_float(value));
      return TRUE;
    case FL_VALUE_TYPE_STRING: {
      write_uint8(buffer, kValueString);
      const char* text = fl_value_get_string(value);
      size_t length = strlen(text);
      fl_standard_message_codec_write_size(self, buffer, length);
      g_byte_array_append(buffer, reinterpret_cast<const uint8_t*>(text),
                          length);
      return TRUE;
    }
    case FL_VALUE_TYPE_UINT8_LIST: {
      write_uint8(buffer, kValueUint8List);
      size_t length = fl_value_get_length(value);
      fl_standard_message_codec_write_size(self, buffer, length);
      g_byte_array_append(buffer, fl_value_get_uint8_list(value),
                          sizeof(uint8_t) * length);
      return TRUE;
    }
    case FL_VALUE_TYPE_INT32_LIST: {
      write_uint8(buffer, kValueInt32List);
      size_t length = fl_value_get_length(value);
      fl_standard_message_codec_write_size(self, buffer, length);
      write_align(buffer, 4);
      g_byte_array_append(
          buffer,
          reinterpret_cast<const uint8_t*>(fl_value_get_int32_list(value)),
          sizeof(int32_t) * length);
      return TRUE;
    }
    case FL_VALUE_TYPE_INT64_LIST: {
      write_uint8(buffer, kValueInt64List);
      size_t length = fl_value_get_length(value);
      fl_standard_message_codec_write_size(self, buffer, length);
      write_align(buffer, 8);
      g_byte_array_append(
          buffer,
          reinterpret_cast<const uint8_t*>(fl_value_get_int64_list(value)),
          sizeof(int64_t) * length);
      return TRUE;
    }
    case FL_VALUE_TYPE_FLOAT32_LIST: {
      write_uint8(buffer, kValueFloat32List);
      size_t length = fl_value_get_length(value);
      fl_standard_message_codec_write_size(self, buffer, length);
      write_align(buffer, 4);
      g_byte_array_append(
          buffer,
          reinterpret_cast<const uint8_t*>(fl_value_get_float32_list(value)),
          sizeof(float) * length);
      return TRUE;
    }
    case FL_VALUE_TYPE_FLOAT_LIST: {
      write_uint8(buffer, kValueFloat64List);
      size_t length = fl_value_get_length(value);
      fl_standard_message_codec_write_size(self, buffer, length);
      write_align(buffer, 8);
      g_byte_array_append(
          buffer,
          reinterpret_cast<const uint8_t*>(fl_value_get_float_list(value)),
          sizeof(double) * length);
      return TRUE;
    }
    case FL_VALUE_TYPE_LIST:
      write_uint8(buffer, kValueList);
      fl_standard_message_codec_write_size(self, buffer,
                                           fl_value_get_length(value));
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        if (!fl_standard_message_codec_write_value(
                self, buffer, fl_value_get_list_value(value, i), error)) {
          return FALSE;
        }
      }
      return TRUE;
    case FL_VALUE_TYPE_MAP:
      write_uint8(buffer, kValueMap);
      fl_standard_message_codec_write_size(self, buffer,
                                           fl_value_get_length(value));
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        if (!fl_standard_message_codec_write_value(
                self, buffer, fl_value_get_map_key(value, i), error) ||
            !fl_standard_message_codec_write_value(
                self, buffer, fl_value_get_map_value(value, i), error)) {
          return FALSE;
        }
      }
      return TRUE;
    case FL_VALUE_TYPE_CUSTOM:
      g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                  FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE,
                  "Custom value not implemented");
      return FALSE;
  }

  g_set_error(error, FL_MESSAGE_CODEC_ERROR,
              FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE,
              "Unexpected FlValue type %d", fl_value_get_type(value));
  return FALSE;
}

// Implements FlStandardMessageCodec::read_value_of_type.
static FlValue* fl_standard_message_codec_read_value_of_type(
    FlStandardMessageCodec* self,
    GBytes* buffer,
    size_t* offset,
    int type,
    GError** error) {
  g_autoptr(FlValue) value = nullptr;
  if (type == kValueNull) {
    return fl_value_new_null();
  } else if (type == kValueTrue) {
    return fl_value_new_bool(TRUE);
  } else if (type == kValueFalse) {
    return fl_value_new_bool(FALSE);
  } else if (type == kValueInt32) {
    value = read_int32_value(buffer, offset, error);
  } else if (type == kValueInt64) {
    value = read_int64_value(buffer, offset, error);
  } else if (type == kValueFloat64) {
    value = read_float64_value(buffer, offset, error);
  } else if (type == kValueString) {
    value = read_string_value(self, buffer, offset, error);
  } else if (type == kValueUint8List) {
    value = read_uint8_list_value(self, buffer, offset, error);
  } else if (type == kValueInt32List) {
    value = read_int32_list_value(self, buffer, offset, error);
  } else if (type == kValueInt64List) {
    value = read_int64_list_value(self, buffer, offset, error);
  } else if (type == kValueFloat32List) {
    value = read_float32_list_value(self, buffer, offset, error);
  } else if (type == kValueFloat64List) {
    value = read_float64_list_value(self, buffer, offset, error);
  } else if (type == kValueList) {
    value = read_list_value(self, buffer, offset, error);
  } else if (type == kValueMap) {
    value = read_map_value(self, buffer, offset, error);
  } else {
    g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE,
                "Unexpected standard codec type %02x", type);
    return nullptr;
  }

  return value == nullptr ? nullptr : fl_value_ref(value);
}

static void fl_standard_message_codec_class_init(
    FlStandardMessageCodecClass* klass) {
  FL_MESSAGE_CODEC_CLASS(klass)->encode_message =
      fl_standard_message_codec_encode_message;
  FL_MESSAGE_CODEC_CLASS(klass)->decode_message =
      fl_standard_message_codec_decode_message;
  klass->write_value = fl_standard_message_codec_real_write_value;
  klass->read_value_of_type = fl_standard_message_codec_read_value_of_type;
}

static void fl_standard_message_codec_init(FlStandardMessageCodec* self) {}

G_MODULE_EXPORT FlStandardMessageCodec* fl_standard_message_codec_new() {
  return static_cast<FlStandardMessageCodec*>(
      g_object_new(fl_standard_message_codec_get_type(), nullptr));
}

G_MODULE_EXPORT void fl_standard_message_codec_write_size(
    FlStandardMessageCodec* codec,
    GByteArray* buffer,
    uint32_t size) {
  if (size < 254) {
    write_uint8(buffer, size);
  } else if (size <= 0xffff) {
    write_uint8(buffer, 254);
    write_uint16(buffer, size);
  } else {
    write_uint8(buffer, 255);
    write_uint32(buffer, size);
  }
}

G_MODULE_EXPORT gboolean fl_standard_message_codec_read_size(
    FlStandardMessageCodec* codec,
    GBytes* buffer,
    size_t* offset,
    uint32_t* value,
    GError** error) {
  uint8_t value8;
  if (!read_uint8(buffer, offset, &value8, error)) {
    return FALSE;
  }

  if (value8 == 255) {
    if (!read_uint32(buffer, offset, value, error)) {
      return FALSE;
    }
  } else if (value8 == 254) {
    uint16_t value16;
    if (!read_uint16(buffer, offset, &value16, error)) {
      return FALSE;
    }
    *value = value16;
  } else {
    *value = value8;
  }

  return TRUE;
}

G_MODULE_EXPORT gboolean fl_standard_message_codec_write_value(
    FlStandardMessageCodec* self,
    GByteArray* buffer,
    FlValue* value,
    GError** error) {
  return FL_STANDARD_MESSAGE_CODEC_GET_CLASS(self)->write_value(self, buffer,
                                                                value, error);
}

G_MODULE_EXPORT FlValue* fl_standard_message_codec_read_value(
    FlStandardMessageCodec* self,
    GBytes* buffer,
    size_t* offset,
    GError** error) {
  uint8_t type;
  if (!read_uint8(buffer, offset, &type, error)) {
    return nullptr;
  }

  return FL_STANDARD_MESSAGE_CODEC_GET_CLASS(self)->read_value_of_type(
      self, buffer, offset, type, error);
}
