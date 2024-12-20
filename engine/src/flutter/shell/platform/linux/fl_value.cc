// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_value.h"

#include <gmodule.h>

#include <cstring>

struct _FlValue {
  FlValueType type;
  int ref_count;
};

typedef struct {
  FlValue parent;
  bool value;
} FlValueBool;

typedef struct {
  FlValue parent;
  int64_t value;
} FlValueInt;

typedef struct {
  FlValue parent;
  double value;
} FlValueDouble;

typedef struct {
  FlValue parent;
  gchar* value;
} FlValueString;

typedef struct {
  FlValue parent;
  uint8_t* values;
  size_t values_length;
} FlValueUint8List;

typedef struct {
  FlValue parent;
  int32_t* values;
  size_t values_length;
} FlValueInt32List;

typedef struct {
  FlValue parent;
  int64_t* values;
  size_t values_length;
} FlValueInt64List;

typedef struct {
  FlValue parent;
  float* values;
  size_t values_length;
} FlValueFloat32List;

typedef struct {
  FlValue parent;
  double* values;
  size_t values_length;
} FlValueFloatList;

typedef struct {
  FlValue parent;
  GPtrArray* values;
} FlValueList;

typedef struct {
  FlValue parent;
  GPtrArray* keys;
  GPtrArray* values;
} FlValueMap;

typedef struct {
  FlValue parent;
  int type;
  gconstpointer value;
  GDestroyNotify destroy_notify;
} FlValueCustom;

static FlValue* fl_value_new(FlValueType type, size_t size) {
  FlValue* self = static_cast<FlValue*>(g_malloc0(size));
  self->type = type;
  self->ref_count = 1;
  return self;
}

// Helper function to match GDestroyNotify type.
static void fl_value_destroy(gpointer value) {
  fl_value_unref(static_cast<FlValue*>(value));
}

// Finds the index of a key in a FlValueMap.
// FIXME(robert-ancell) This is highly inefficient, and should be optimized if
// necessary.
static ssize_t fl_value_lookup_index(FlValue* self, FlValue* key) {
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_MAP, -1);

  for (size_t i = 0; i < fl_value_get_length(self); i++) {
    FlValue* k = fl_value_get_map_key(self, i);
    if (fl_value_equal(k, key)) {
      return i;
    }
  }
  return -1;
}

// Converts an integer to a string and adds it to the buffer.
static void int_to_string(int64_t value, GString* buffer) {
  g_string_append_printf(buffer, "%" G_GINT64_FORMAT, value);
}

// Converts a floating point number to a string and adds it to the buffer.
static void float_to_string(double value, GString* buffer) {
  g_string_append_printf(buffer, "%.16f", value);

  // Strip trailing zeros.
  int zero_count = 0;
  for (int i = buffer->len - 1; i >= 0; i--) {
    // Leave one zero after a decimal point.
    if (buffer->str[i] == '.') {
      zero_count = zero_count == 0 ? 0 : zero_count - 1;
      break;
    }
    if (buffer->str[i] != '0') {
      break;
    }
    zero_count++;
  }
  g_string_truncate(buffer, buffer->len - zero_count);
}

static void value_to_string(FlValue* value, GString* buffer) {
  switch (value->type) {
    case FL_VALUE_TYPE_NULL:
      g_string_append(buffer, "null");
      return;
    case FL_VALUE_TYPE_BOOL:
      if (fl_value_get_bool(value)) {
        g_string_append(buffer, "true");
      } else {
        g_string_append(buffer, "false");
      }
      return;
    case FL_VALUE_TYPE_INT:
      int_to_string(fl_value_get_int(value), buffer);
      return;
    case FL_VALUE_TYPE_FLOAT:
      float_to_string(fl_value_get_float(value), buffer);
      return;
    case FL_VALUE_TYPE_STRING: {
      g_string_append(buffer, fl_value_get_string(value));
      return;
    }
    case FL_VALUE_TYPE_UINT8_LIST: {
      g_string_append(buffer, "[");
      const uint8_t* values = fl_value_get_uint8_list(value);
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        if (i != 0) {
          g_string_append(buffer, ", ");
        }
        int_to_string(values[i], buffer);
      }
      g_string_append(buffer, "]");
      return;
    }
    case FL_VALUE_TYPE_INT32_LIST: {
      g_string_append(buffer, "[");
      const int32_t* values = fl_value_get_int32_list(value);
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        if (i != 0) {
          g_string_append(buffer, ", ");
        }
        int_to_string(values[i], buffer);
      }
      g_string_append(buffer, "]");
      return;
    }
    case FL_VALUE_TYPE_INT64_LIST: {
      g_string_append(buffer, "[");
      const int64_t* values = fl_value_get_int64_list(value);
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        if (i != 0) {
          g_string_append(buffer, ", ");
        }
        int_to_string(values[i], buffer);
      }
      g_string_append(buffer, "]");
      return;
    }
    case FL_VALUE_TYPE_FLOAT32_LIST: {
      g_string_append(buffer, "[");
      const float* values = fl_value_get_float32_list(value);
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        if (i != 0) {
          g_string_append(buffer, ", ");
        }
        float_to_string(values[i], buffer);
      }
      g_string_append(buffer, "]");
      return;
    }
    case FL_VALUE_TYPE_FLOAT_LIST: {
      g_string_append(buffer, "[");
      const double* values = fl_value_get_float_list(value);
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        if (i != 0) {
          g_string_append(buffer, ", ");
        }
        float_to_string(values[i], buffer);
      }
      g_string_append(buffer, "]");
      return;
    }
    case FL_VALUE_TYPE_LIST: {
      g_string_append(buffer, "[");
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        if (i != 0) {
          g_string_append(buffer, ", ");
        }
        value_to_string(fl_value_get_list_value(value, i), buffer);
      }
      g_string_append(buffer, "]");
      return;
    }
    case FL_VALUE_TYPE_MAP: {
      g_string_append(buffer, "{");
      for (size_t i = 0; i < fl_value_get_length(value); i++) {
        if (i != 0) {
          g_string_append(buffer, ", ");
        }
        value_to_string(fl_value_get_map_key(value, i), buffer);
        g_string_append(buffer, ": ");
        value_to_string(fl_value_get_map_value(value, i), buffer);
      }
      g_string_append(buffer, "}");
      return;
    }
    case FL_VALUE_TYPE_CUSTOM:
      g_string_append_printf(buffer, "(custom %d)",
                             fl_value_get_custom_type(value));
      return;
    default:
      g_string_append_printf(buffer, "<unknown type %d>", value->type);
  }
}

G_MODULE_EXPORT FlValue* fl_value_new_null() {
  return fl_value_new(FL_VALUE_TYPE_NULL, sizeof(FlValue));
}

G_MODULE_EXPORT FlValue* fl_value_new_bool(bool value) {
  FlValueBool* self = reinterpret_cast<FlValueBool*>(
      fl_value_new(FL_VALUE_TYPE_BOOL, sizeof(FlValueBool)));
  self->value = value ? true : false;
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_int(int64_t value) {
  FlValueInt* self = reinterpret_cast<FlValueInt*>(
      fl_value_new(FL_VALUE_TYPE_INT, sizeof(FlValueInt)));
  self->value = value;
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_float(double value) {
  FlValueDouble* self = reinterpret_cast<FlValueDouble*>(
      fl_value_new(FL_VALUE_TYPE_FLOAT, sizeof(FlValueDouble)));
  self->value = value;
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_string(const gchar* value) {
  FlValueString* self = reinterpret_cast<FlValueString*>(
      fl_value_new(FL_VALUE_TYPE_STRING, sizeof(FlValueString)));
  self->value = g_strdup(value);
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_string_sized(const gchar* value,
                                                   size_t value_length) {
  FlValueString* self = reinterpret_cast<FlValueString*>(
      fl_value_new(FL_VALUE_TYPE_STRING, sizeof(FlValueString)));
  self->value =
      value_length == 0 ? g_strdup("") : g_strndup(value, value_length);
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_uint8_list(const uint8_t* data,
                                                 size_t data_length) {
  FlValueUint8List* self = reinterpret_cast<FlValueUint8List*>(
      fl_value_new(FL_VALUE_TYPE_UINT8_LIST, sizeof(FlValueUint8List)));
  self->values_length = data_length;
  self->values = static_cast<uint8_t*>(g_malloc(sizeof(uint8_t) * data_length));
  memcpy(self->values, data, sizeof(uint8_t) * data_length);
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_uint8_list_from_bytes(GBytes* data) {
  gsize length;
  const uint8_t* d =
      static_cast<const uint8_t*>(g_bytes_get_data(data, &length));
  return fl_value_new_uint8_list(d, length);
}

G_MODULE_EXPORT FlValue* fl_value_new_int32_list(const int32_t* data,
                                                 size_t data_length) {
  FlValueInt32List* self = reinterpret_cast<FlValueInt32List*>(
      fl_value_new(FL_VALUE_TYPE_INT32_LIST, sizeof(FlValueInt32List)));
  self->values_length = data_length;
  self->values = static_cast<int32_t*>(g_malloc(sizeof(int32_t) * data_length));
  memcpy(self->values, data, sizeof(int32_t) * data_length);
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_int64_list(const int64_t* data,
                                                 size_t data_length) {
  FlValueInt64List* self = reinterpret_cast<FlValueInt64List*>(
      fl_value_new(FL_VALUE_TYPE_INT64_LIST, sizeof(FlValueInt64List)));
  self->values_length = data_length;
  self->values = static_cast<int64_t*>(g_malloc(sizeof(int64_t) * data_length));
  memcpy(self->values, data, sizeof(int64_t) * data_length);
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_float32_list(const float* data,
                                                   size_t data_length) {
  FlValueFloat32List* self = reinterpret_cast<FlValueFloat32List*>(
      fl_value_new(FL_VALUE_TYPE_FLOAT32_LIST, sizeof(FlValueFloat32List)));
  self->values_length = data_length;
  self->values = static_cast<float*>(g_malloc(sizeof(float) * data_length));
  memcpy(self->values, data, sizeof(float) * data_length);
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_float_list(const double* data,
                                                 size_t data_length) {
  FlValueFloatList* self = reinterpret_cast<FlValueFloatList*>(
      fl_value_new(FL_VALUE_TYPE_FLOAT_LIST, sizeof(FlValueFloatList)));
  self->values_length = data_length;
  self->values = static_cast<double*>(g_malloc(sizeof(double) * data_length));
  memcpy(self->values, data, sizeof(double) * data_length);
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_list() {
  FlValueList* self = reinterpret_cast<FlValueList*>(
      fl_value_new(FL_VALUE_TYPE_LIST, sizeof(FlValueList)));
  self->values = g_ptr_array_new_with_free_func(fl_value_destroy);
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_list_from_strv(
    const gchar* const* str_array) {
  g_return_val_if_fail(str_array != nullptr, nullptr);
  g_autoptr(FlValue) value = fl_value_new_list();
  for (int i = 0; str_array[i] != nullptr; i++) {
    fl_value_append_take(value, fl_value_new_string(str_array[i]));
  }
  return fl_value_ref(value);
}

G_MODULE_EXPORT FlValue* fl_value_new_map() {
  FlValueMap* self = reinterpret_cast<FlValueMap*>(
      fl_value_new(FL_VALUE_TYPE_MAP, sizeof(FlValueMap)));
  self->keys = g_ptr_array_new_with_free_func(fl_value_destroy);
  self->values = g_ptr_array_new_with_free_func(fl_value_destroy);
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_custom(int type,
                                             gconstpointer value,
                                             GDestroyNotify destroy_notify) {
  FlValueCustom* self = reinterpret_cast<FlValueCustom*>(
      fl_value_new(FL_VALUE_TYPE_CUSTOM, sizeof(FlValueCustom)));
  self->type = type;
  self->value = value;
  self->destroy_notify = destroy_notify;
  return reinterpret_cast<FlValue*>(self);
}

G_MODULE_EXPORT FlValue* fl_value_new_custom_object(int type, GObject* object) {
  return fl_value_new_custom(type, g_object_ref(object), g_object_unref);
}

G_MODULE_EXPORT FlValue* fl_value_new_custom_object_take(int type,
                                                         GObject* object) {
  return fl_value_new_custom(type, object, g_object_unref);
}

G_MODULE_EXPORT FlValue* fl_value_ref(FlValue* self) {
  g_return_val_if_fail(self != nullptr, nullptr);
  self->ref_count++;
  return self;
}

G_MODULE_EXPORT void fl_value_unref(FlValue* self) {
  g_return_if_fail(self != nullptr);
  g_return_if_fail(self->ref_count > 0);
  self->ref_count--;
  if (self->ref_count != 0) {
    return;
  }

  switch (self->type) {
    case FL_VALUE_TYPE_STRING: {
      FlValueString* v = reinterpret_cast<FlValueString*>(self);
      g_free(v->value);
      break;
    }
    case FL_VALUE_TYPE_UINT8_LIST: {
      FlValueUint8List* v = reinterpret_cast<FlValueUint8List*>(self);
      g_free(v->values);
      break;
    }
    case FL_VALUE_TYPE_INT32_LIST: {
      FlValueInt32List* v = reinterpret_cast<FlValueInt32List*>(self);
      g_free(v->values);
      break;
    }
    case FL_VALUE_TYPE_INT64_LIST: {
      FlValueInt64List* v = reinterpret_cast<FlValueInt64List*>(self);
      g_free(v->values);
      break;
    }
    case FL_VALUE_TYPE_FLOAT32_LIST: {
      FlValueFloat32List* v = reinterpret_cast<FlValueFloat32List*>(self);
      g_free(v->values);
      break;
    }
    case FL_VALUE_TYPE_FLOAT_LIST: {
      FlValueFloatList* v = reinterpret_cast<FlValueFloatList*>(self);
      g_free(v->values);
      break;
    }
    case FL_VALUE_TYPE_LIST: {
      FlValueList* v = reinterpret_cast<FlValueList*>(self);
      g_ptr_array_unref(v->values);
      break;
    }
    case FL_VALUE_TYPE_MAP: {
      FlValueMap* v = reinterpret_cast<FlValueMap*>(self);
      g_ptr_array_unref(v->keys);
      g_ptr_array_unref(v->values);
      break;
    }
    case FL_VALUE_TYPE_CUSTOM: {
      FlValueCustom* v = reinterpret_cast<FlValueCustom*>(self);
      if (v->destroy_notify != nullptr) {
        v->destroy_notify(const_cast<gpointer>(v->value));
      }
      break;
    }
    case FL_VALUE_TYPE_NULL:
    case FL_VALUE_TYPE_BOOL:
    case FL_VALUE_TYPE_INT:
    case FL_VALUE_TYPE_FLOAT:
      break;
  }
  g_free(self);
}

G_MODULE_EXPORT FlValueType fl_value_get_type(FlValue* self) {
  g_return_val_if_fail(self != nullptr, FL_VALUE_TYPE_NULL);
  return self->type;
}

G_MODULE_EXPORT bool fl_value_equal(FlValue* a, FlValue* b) {
  g_return_val_if_fail(a != nullptr, false);
  g_return_val_if_fail(b != nullptr, false);

  if (a->type != b->type) {
    return false;
  }

  switch (a->type) {
    case FL_VALUE_TYPE_NULL:
      return true;
    case FL_VALUE_TYPE_BOOL:
      return fl_value_get_bool(a) == fl_value_get_bool(b);
    case FL_VALUE_TYPE_INT:
      return fl_value_get_int(a) == fl_value_get_int(b);
    case FL_VALUE_TYPE_FLOAT:
      return fl_value_get_float(a) == fl_value_get_float(b);
    case FL_VALUE_TYPE_STRING: {
      FlValueString* a_ = reinterpret_cast<FlValueString*>(a);
      FlValueString* b_ = reinterpret_cast<FlValueString*>(b);
      return g_strcmp0(a_->value, b_->value) == 0;
    }
    case FL_VALUE_TYPE_UINT8_LIST: {
      if (fl_value_get_length(a) != fl_value_get_length(b)) {
        return false;
      }
      const uint8_t* values_a = fl_value_get_uint8_list(a);
      const uint8_t* values_b = fl_value_get_uint8_list(b);
      for (size_t i = 0; i < fl_value_get_length(a); i++) {
        if (values_a[i] != values_b[i]) {
          return false;
        }
      }
      return true;
    }
    case FL_VALUE_TYPE_INT32_LIST: {
      if (fl_value_get_length(a) != fl_value_get_length(b)) {
        return false;
      }
      const int32_t* values_a = fl_value_get_int32_list(a);
      const int32_t* values_b = fl_value_get_int32_list(b);
      for (size_t i = 0; i < fl_value_get_length(a); i++) {
        if (values_a[i] != values_b[i]) {
          return false;
        }
      }
      return true;
    }
    case FL_VALUE_TYPE_INT64_LIST: {
      if (fl_value_get_length(a) != fl_value_get_length(b)) {
        return false;
      }
      const int64_t* values_a = fl_value_get_int64_list(a);
      const int64_t* values_b = fl_value_get_int64_list(b);
      for (size_t i = 0; i < fl_value_get_length(a); i++) {
        if (values_a[i] != values_b[i]) {
          return false;
        }
      }
      return true;
    }
    case FL_VALUE_TYPE_FLOAT32_LIST: {
      if (fl_value_get_length(a) != fl_value_get_length(b)) {
        return false;
      }
      const float* values_a = fl_value_get_float32_list(a);
      const float* values_b = fl_value_get_float32_list(b);
      for (size_t i = 0; i < fl_value_get_length(a); i++) {
        if (values_a[i] != values_b[i]) {
          return false;
        }
      }
      return true;
    }
    case FL_VALUE_TYPE_FLOAT_LIST: {
      if (fl_value_get_length(a) != fl_value_get_length(b)) {
        return false;
      }
      const double* values_a = fl_value_get_float_list(a);
      const double* values_b = fl_value_get_float_list(b);
      for (size_t i = 0; i < fl_value_get_length(a); i++) {
        if (values_a[i] != values_b[i]) {
          return false;
        }
      }
      return true;
    }
    case FL_VALUE_TYPE_LIST: {
      if (fl_value_get_length(a) != fl_value_get_length(b)) {
        return false;
      }
      for (size_t i = 0; i < fl_value_get_length(a); i++) {
        if (!fl_value_equal(fl_value_get_list_value(a, i),
                            fl_value_get_list_value(b, i))) {
          return false;
        }
      }
      return true;
    }
    case FL_VALUE_TYPE_MAP: {
      if (fl_value_get_length(a) != fl_value_get_length(b)) {
        return false;
      }
      for (size_t i = 0; i < fl_value_get_length(a); i++) {
        FlValue* key = fl_value_get_map_key(a, i);
        FlValue* value_b = fl_value_lookup(b, key);
        if (value_b == nullptr) {
          return false;
        }
        FlValue* value_a = fl_value_get_map_value(a, i);
        if (!fl_value_equal(value_a, value_b)) {
          return false;
        }
      }
      return true;
    }
    case FL_VALUE_TYPE_CUSTOM:
      return false;
  }
}

G_MODULE_EXPORT void fl_value_append(FlValue* self, FlValue* value) {
  g_return_if_fail(self != nullptr);
  g_return_if_fail(self->type == FL_VALUE_TYPE_LIST);
  g_return_if_fail(value != nullptr);

  fl_value_append_take(self, fl_value_ref(value));
}

G_MODULE_EXPORT void fl_value_append_take(FlValue* self, FlValue* value) {
  g_return_if_fail(self != nullptr);
  g_return_if_fail(self->type == FL_VALUE_TYPE_LIST);
  g_return_if_fail(value != nullptr);

  FlValueList* v = reinterpret_cast<FlValueList*>(self);
  g_ptr_array_add(v->values, value);
}

G_MODULE_EXPORT void fl_value_set(FlValue* self, FlValue* key, FlValue* value) {
  g_return_if_fail(self != nullptr);
  g_return_if_fail(self->type == FL_VALUE_TYPE_MAP);
  g_return_if_fail(key != nullptr);
  g_return_if_fail(value != nullptr);

  fl_value_set_take(self, fl_value_ref(key), fl_value_ref(value));
}

G_MODULE_EXPORT void fl_value_set_take(FlValue* self,
                                       FlValue* key,
                                       FlValue* value) {
  g_return_if_fail(self != nullptr);
  g_return_if_fail(self->type == FL_VALUE_TYPE_MAP);
  g_return_if_fail(key != nullptr);
  g_return_if_fail(value != nullptr);

  FlValueMap* v = reinterpret_cast<FlValueMap*>(self);
  ssize_t index = fl_value_lookup_index(self, key);
  if (index < 0) {
    g_ptr_array_add(v->keys, key);
    g_ptr_array_add(v->values, value);
  } else {
    fl_value_destroy(v->keys->pdata[index]);
    v->keys->pdata[index] = key;
    fl_value_destroy(v->values->pdata[index]);
    v->values->pdata[index] = value;
  }
}

G_MODULE_EXPORT void fl_value_set_string(FlValue* self,
                                         const gchar* key,
                                         FlValue* value) {
  g_return_if_fail(self != nullptr);
  g_return_if_fail(self->type == FL_VALUE_TYPE_MAP);
  g_return_if_fail(key != nullptr);
  g_return_if_fail(value != nullptr);

  fl_value_set_take(self, fl_value_new_string(key), fl_value_ref(value));
}

G_MODULE_EXPORT void fl_value_set_string_take(FlValue* self,
                                              const gchar* key,
                                              FlValue* value) {
  g_return_if_fail(self != nullptr);
  g_return_if_fail(self->type == FL_VALUE_TYPE_MAP);
  g_return_if_fail(key != nullptr);
  g_return_if_fail(value != nullptr);

  fl_value_set_take(self, fl_value_new_string(key), value);
}

G_MODULE_EXPORT bool fl_value_get_bool(FlValue* self) {
  g_return_val_if_fail(self != nullptr, FALSE);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_BOOL, FALSE);
  FlValueBool* v = reinterpret_cast<FlValueBool*>(self);
  return v->value;
}

G_MODULE_EXPORT int64_t fl_value_get_int(FlValue* self) {
  g_return_val_if_fail(self != nullptr, 0);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_INT, 0);
  FlValueInt* v = reinterpret_cast<FlValueInt*>(self);
  return v->value;
}

G_MODULE_EXPORT double fl_value_get_float(FlValue* self) {
  g_return_val_if_fail(self != nullptr, 0.0);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_FLOAT, 0.0);
  FlValueDouble* v = reinterpret_cast<FlValueDouble*>(self);
  return v->value;
}

G_MODULE_EXPORT const gchar* fl_value_get_string(FlValue* self) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_STRING, nullptr);
  FlValueString* v = reinterpret_cast<FlValueString*>(self);
  return v->value;
}

G_MODULE_EXPORT const uint8_t* fl_value_get_uint8_list(FlValue* self) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_UINT8_LIST, nullptr);
  FlValueUint8List* v = reinterpret_cast<FlValueUint8List*>(self);
  return v->values;
}

G_MODULE_EXPORT const int32_t* fl_value_get_int32_list(FlValue* self) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_INT32_LIST, nullptr);
  FlValueInt32List* v = reinterpret_cast<FlValueInt32List*>(self);
  return v->values;
}

G_MODULE_EXPORT const int64_t* fl_value_get_int64_list(FlValue* self) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_INT64_LIST, nullptr);
  FlValueInt64List* v = reinterpret_cast<FlValueInt64List*>(self);
  return v->values;
}

G_MODULE_EXPORT const float* fl_value_get_float32_list(FlValue* self) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_FLOAT32_LIST, nullptr);
  FlValueFloat32List* v = reinterpret_cast<FlValueFloat32List*>(self);
  return v->values;
}

G_MODULE_EXPORT const double* fl_value_get_float_list(FlValue* self) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_FLOAT_LIST, nullptr);
  FlValueFloatList* v = reinterpret_cast<FlValueFloatList*>(self);
  return v->values;
}

G_MODULE_EXPORT size_t fl_value_get_length(FlValue* self) {
  g_return_val_if_fail(self != nullptr, 0);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_UINT8_LIST ||
                           self->type == FL_VALUE_TYPE_INT32_LIST ||
                           self->type == FL_VALUE_TYPE_INT64_LIST ||
                           self->type == FL_VALUE_TYPE_FLOAT32_LIST ||
                           self->type == FL_VALUE_TYPE_FLOAT_LIST ||
                           self->type == FL_VALUE_TYPE_LIST ||
                           self->type == FL_VALUE_TYPE_MAP,
                       0);

  switch (self->type) {
    case FL_VALUE_TYPE_UINT8_LIST: {
      FlValueUint8List* v = reinterpret_cast<FlValueUint8List*>(self);
      return v->values_length;
    }
    case FL_VALUE_TYPE_INT32_LIST: {
      FlValueInt32List* v = reinterpret_cast<FlValueInt32List*>(self);
      return v->values_length;
    }
    case FL_VALUE_TYPE_INT64_LIST: {
      FlValueInt64List* v = reinterpret_cast<FlValueInt64List*>(self);
      return v->values_length;
    }
    case FL_VALUE_TYPE_FLOAT32_LIST: {
      FlValueFloat32List* v = reinterpret_cast<FlValueFloat32List*>(self);
      return v->values_length;
    }
    case FL_VALUE_TYPE_FLOAT_LIST: {
      FlValueFloatList* v = reinterpret_cast<FlValueFloatList*>(self);
      return v->values_length;
    }
    case FL_VALUE_TYPE_LIST: {
      FlValueList* v = reinterpret_cast<FlValueList*>(self);
      return v->values->len;
    }
    case FL_VALUE_TYPE_MAP: {
      FlValueMap* v = reinterpret_cast<FlValueMap*>(self);
      return v->keys->len;
    }
    case FL_VALUE_TYPE_NULL:
    case FL_VALUE_TYPE_BOOL:
    case FL_VALUE_TYPE_INT:
    case FL_VALUE_TYPE_FLOAT:
    case FL_VALUE_TYPE_STRING:
    case FL_VALUE_TYPE_CUSTOM:
      return 0;
  }

  return 0;
}

G_MODULE_EXPORT FlValue* fl_value_get_list_value(FlValue* self, size_t index) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_LIST, nullptr);

  FlValueList* v = reinterpret_cast<FlValueList*>(self);
  return static_cast<FlValue*>(g_ptr_array_index(v->values, index));
}

G_MODULE_EXPORT FlValue* fl_value_get_map_key(FlValue* self, size_t index) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_MAP, nullptr);

  FlValueMap* v = reinterpret_cast<FlValueMap*>(self);
  return static_cast<FlValue*>(g_ptr_array_index(v->keys, index));
}

G_MODULE_EXPORT FlValue* fl_value_get_map_value(FlValue* self, size_t index) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_MAP, nullptr);

  FlValueMap* v = reinterpret_cast<FlValueMap*>(self);
  return static_cast<FlValue*>(g_ptr_array_index(v->values, index));
}

G_MODULE_EXPORT FlValue* fl_value_lookup(FlValue* self, FlValue* key) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_MAP, nullptr);

  ssize_t index = fl_value_lookup_index(self, key);
  if (index < 0) {
    return nullptr;
  }
  return fl_value_get_map_value(self, index);
}

G_MODULE_EXPORT FlValue* fl_value_lookup_string(FlValue* self,
                                                const gchar* key) {
  g_return_val_if_fail(self != nullptr, nullptr);
  FlValue* string_key = fl_value_new_string(key);
  FlValue* value = fl_value_lookup(self, string_key);
  // Explicit unref used because the g_autoptr is triggering a false positive
  // with clang-tidy.
  fl_value_unref(string_key);
  return value;
}

G_MODULE_EXPORT int fl_value_get_custom_type(FlValue* self) {
  g_return_val_if_fail(self != nullptr, -1);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_CUSTOM, -1);

  FlValueCustom* v = reinterpret_cast<FlValueCustom*>(self);
  return v->type;
}

G_MODULE_EXPORT gconstpointer fl_value_get_custom_value(FlValue* self) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_CUSTOM, nullptr);

  FlValueCustom* v = reinterpret_cast<FlValueCustom*>(self);
  return v->value;
}

G_MODULE_EXPORT GObject* fl_value_get_custom_value_object(FlValue* self) {
  g_return_val_if_fail(self != nullptr, nullptr);
  g_return_val_if_fail(self->type == FL_VALUE_TYPE_CUSTOM, nullptr);

  FlValueCustom* v = reinterpret_cast<FlValueCustom*>(self);
  return G_OBJECT(v->value);
}

G_MODULE_EXPORT gchar* fl_value_to_string(FlValue* value) {
  GString* buffer = g_string_new("");
  value_to_string(value, buffer);
  return g_string_free(buffer, FALSE);
}
