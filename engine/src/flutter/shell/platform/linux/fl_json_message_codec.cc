// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_json_message_codec.h"

#include <gmodule.h>

#include <cstring>

#include "rapidjson/reader.h"
#include "rapidjson/writer.h"

G_DEFINE_QUARK(fl_json_message_codec_error_quark, fl_json_message_codec_error)

struct _FlJsonMessageCodec {
  FlMessageCodec parent_instance;
};

G_DEFINE_TYPE(FlJsonMessageCodec,
              fl_json_message_codec,
              fl_message_codec_get_type())

// Recursively writes #FlValue objects using rapidjson.
static gboolean write_value(rapidjson::Writer<rapidjson::StringBuffer>& writer,
                            FlValue* value,
                            GError** error) {
  if (value == nullptr) {
    writer.Null();
    return TRUE;
  }

  switch (fl_value_get_type(value)) {
    case FL_VALUE_TYPE_NULL:
      writer.Null();
      break;
    case FL_VALUE_TYPE_BOOL:
      writer.Bool(fl_value_get_bool(value));
      break;
    case FL_VALUE_TYPE_INT:
      writer.Int64(fl_value_get_int(value));
      break;
    case FL_VALUE_TYPE_FLOAT:
      writer.Double(fl_value_get_float(value));
      break;
    case FL_VALUE_TYPE_STRING:
      writer.String(fl_value_get_string(value));
      break;
    case FL_VALUE_TYPE_UINT8_LIST: {
      writer.StartArray();
      const uint8_t* data = fl_value_get_uint8_list(value);
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        writer.Int(data[i]);
      }
      writer.EndArray();
      break;
    }
    case FL_VALUE_TYPE_INT32_LIST: {
      writer.StartArray();
      const int32_t* data = fl_value_get_int32_list(value);
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        writer.Int(data[i]);
      }
      writer.EndArray();
      break;
    }
    case FL_VALUE_TYPE_INT64_LIST: {
      writer.StartArray();
      const int64_t* data = fl_value_get_int64_list(value);
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        writer.Int64(data[i]);
      }
      writer.EndArray();
      break;
    }
    case FL_VALUE_TYPE_FLOAT_LIST: {
      writer.StartArray();
      const double* data = fl_value_get_float_list(value);
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        writer.Double(data[i]);
      }
      writer.EndArray();
      break;
    }
    case FL_VALUE_TYPE_LIST: {
      writer.StartArray();
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        if (!write_value(writer, fl_value_get_list_value(value, i), error)) {
          return FALSE;
        }
      }
      writer.EndArray();
      break;
    }
    case FL_VALUE_TYPE_MAP: {
      writer.StartObject();
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        FlValue* key = fl_value_get_map_key(value, i);
        if (fl_value_get_type(key) != FL_VALUE_TYPE_STRING) {
          g_set_error(error, FL_JSON_MESSAGE_CODEC_ERROR,
                      FL_JSON_MESSAGE_CODEC_ERROR_INVALID_OBJECT_KEY_TYPE,
                      "Invalid object key type");
          return FALSE;
        }
        writer.Key(fl_value_get_string(key));
        if (!write_value(writer, fl_value_get_map_value(value, i), error)) {
          return FALSE;
        }
      }
      writer.EndObject();
      break;
    }
    default:
      g_set_error(error, FL_MESSAGE_CODEC_ERROR,
                  FL_MESSAGE_CODEC_ERROR_UNSUPPORTED_TYPE,
                  "Unexpected FlValue type %d", fl_value_get_type(value));
      return FALSE;
  }

  return TRUE;
}

// Handler to parse JSON using rapidjson in SAX mode.
struct FlValueHandler {
  GPtrArray* stack;
  FlValue* key;
  GError* error;

  FlValueHandler() {
    stack = g_ptr_array_new_with_free_func(
        reinterpret_cast<GDestroyNotify>(fl_value_unref));
    key = nullptr;
    error = nullptr;
  }

  ~FlValueHandler() {
    g_ptr_array_unref(stack);
    if (key != nullptr) {
      fl_value_unref(key);
    }
    if (error != nullptr) {
      g_error_free(error);
    }
  }

  // Gets the current head of the stack.
  FlValue* get_head() {
    if (stack->len == 0) {
      return nullptr;
    }
    return static_cast<FlValue*>(g_ptr_array_index(stack, stack->len - 1));
  }

  // Pushes a value onto the stack.
  void push(FlValue* value) { g_ptr_array_add(stack, fl_value_ref(value)); }

  // Pops the stack.
  void pop() { g_ptr_array_remove_index(stack, stack->len - 1); }

  // Adds a new value to the stack.
  bool add(FlValue* value) {
    g_autoptr(FlValue) owned_value = value;
    FlValue* head = get_head();
    if (head == nullptr) {
      push(owned_value);
    } else if (fl_value_get_type(head) == FL_VALUE_TYPE_LIST) {
      fl_value_append(head, owned_value);
    } else if (fl_value_get_type(head) == FL_VALUE_TYPE_MAP) {
      fl_value_set_take(head, key, fl_value_ref(owned_value));
      key = nullptr;
    } else {
      g_set_error(&error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                  "Can't add value to non container");
      return false;
    }

    if (fl_value_get_type(owned_value) == FL_VALUE_TYPE_LIST ||
        fl_value_get_type(owned_value) == FL_VALUE_TYPE_MAP) {
      push(value);
    }

    return true;
  }

  // The following implements the rapidjson SAX API.

  bool Null() { return add(fl_value_new_null()); }

  bool Bool(bool b) { return add(fl_value_new_bool(b)); }

  bool Int(int i) { return add(fl_value_new_int(i)); }

  bool Uint(unsigned i) { return add(fl_value_new_int(i)); }

  bool Int64(int64_t i) { return add(fl_value_new_int(i)); }

  bool Uint64(uint64_t i) {
    // For some reason (bug in rapidjson?) this is not returned in Int64.
    if (i == G_MAXINT64) {
      return add(fl_value_new_int(i));
    } else {
      return add(fl_value_new_float(i));
    }
  }

  bool Double(double d) { return add(fl_value_new_float(d)); }

  bool RawNumber(const char* str, rapidjson::SizeType length, bool copy) {
    g_set_error(&error, FL_MESSAGE_CODEC_ERROR, FL_MESSAGE_CODEC_ERROR_FAILED,
                "RawNumber not supported");
    return false;
  }

  bool String(const char* str, rapidjson::SizeType length, bool copy) {
    FlValue* v = fl_value_new_string_sized(str, length);
    return add(v);
  }

  bool StartObject() { return add(fl_value_new_map()); }

  bool Key(const char* str, rapidjson::SizeType length, bool copy) {
    if (key != nullptr) {
      fl_value_unref(key);
    }
    key = fl_value_new_string_sized(str, length);
    return true;
  }

  bool EndObject(rapidjson::SizeType memberCount) {
    pop();
    return true;
  }

  bool StartArray() { return add(fl_value_new_list()); }

  bool EndArray(rapidjson::SizeType elementCount) {
    pop();
    return true;
  }
};

// Implements FlMessageCodec:encode_message.
static GBytes* fl_json_message_codec_encode_message(FlMessageCodec* codec,
                                                    FlValue* message,
                                                    GError** error) {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

  if (!write_value(writer, message, error)) {
    return nullptr;
  }

  const gchar* text = buffer.GetString();
  return g_bytes_new(text, strlen(text));
}

// Implements FlMessageCodec:decode_message.
static FlValue* fl_json_message_codec_decode_message(FlMessageCodec* codec,
                                                     GBytes* message,
                                                     GError** error) {
  gsize data_length;
  const gchar* data =
      static_cast<const char*>(g_bytes_get_data(message, &data_length));
  if (!g_utf8_validate(data, data_length, nullptr)) {
    g_set_error(error, FL_JSON_MESSAGE_CODEC_ERROR,
                FL_JSON_MESSAGE_CODEC_ERROR_INVALID_UTF8,
                "Message is not valid UTF8");
    return nullptr;
  }

  FlValueHandler handler;
  rapidjson::Reader reader;
  rapidjson::MemoryStream ss(data, data_length);
  if (!reader.Parse(ss, handler)) {
    if (handler.error != nullptr) {
      g_propagate_error(error, handler.error);
      handler.error = nullptr;
    } else {
      g_set_error(error, FL_JSON_MESSAGE_CODEC_ERROR,
                  FL_JSON_MESSAGE_CODEC_ERROR_INVALID_JSON,
                  "Message is not valid JSON");
    }
    return nullptr;
  }

  FlValue* value = handler.get_head();
  if (value == nullptr) {
    g_set_error(error, FL_JSON_MESSAGE_CODEC_ERROR,
                FL_JSON_MESSAGE_CODEC_ERROR_INVALID_JSON,
                "Message is not valid JSON");
    return nullptr;
  }

  return fl_value_ref(value);
}

static void fl_json_message_codec_class_init(FlJsonMessageCodecClass* klass) {
  FL_MESSAGE_CODEC_CLASS(klass)->encode_message =
      fl_json_message_codec_encode_message;
  FL_MESSAGE_CODEC_CLASS(klass)->decode_message =
      fl_json_message_codec_decode_message;
}

static void fl_json_message_codec_init(FlJsonMessageCodec* self) {}

G_MODULE_EXPORT FlJsonMessageCodec* fl_json_message_codec_new() {
  return static_cast<FlJsonMessageCodec*>(
      g_object_new(fl_json_message_codec_get_type(), nullptr));
}

G_MODULE_EXPORT gchar* fl_json_message_codec_encode(FlJsonMessageCodec* codec,
                                                    FlValue* value,
                                                    GError** error) {
  g_return_val_if_fail(FL_IS_JSON_CODEC(codec), nullptr);

  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

  if (!write_value(writer, value, error)) {
    return nullptr;
  }

  return g_strdup(buffer.GetString());
}

G_MODULE_EXPORT FlValue* fl_json_message_codec_decode(FlJsonMessageCodec* codec,
                                                      const gchar* text,
                                                      GError** error) {
  g_return_val_if_fail(FL_IS_JSON_CODEC(codec), nullptr);

  g_autoptr(GBytes) data = g_bytes_new_static(text, strlen(text));
  g_autoptr(FlValue) value = fl_json_message_codec_decode_message(
      FL_MESSAGE_CODEC(codec), data, error);
  if (value == nullptr) {
    return nullptr;
  }

  return fl_value_ref(value);
}
