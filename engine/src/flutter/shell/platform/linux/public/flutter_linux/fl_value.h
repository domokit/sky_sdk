// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_VALUE_H_
#define FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_VALUE_H_

#include <glib-object.h>
#include <glib.h>
#include <stdbool.h>
#include <stdint.h>

#if !defined(__FLUTTER_LINUX_INSIDE__) && !defined(FLUTTER_LINUX_COMPILATION)
#error "Only <flutter_linux/flutter_linux.h> can be included directly."
#endif

G_BEGIN_DECLS

/**
 * FlValue:
 *
 * #FlValue is an object that contains the data types used in the platform
 * channel used by Flutter.
 *
 * In Dart the values are represented as follows:
 * - #FL_VALUE_TYPE_NULL: Null
 * - #FL_VALUE_TYPE_BOOL: bool
 * - #FL_VALUE_TYPE_INT: num
 * - #FL_VALUE_TYPE_FLOAT: num
 * - #FL_VALUE_TYPE_STRING: String
 * - #FL_VALUE_TYPE_UINT8_LIST: Uint8List
 * - #FL_VALUE_TYPE_INT32_LIST: Int32List
 * - #FL_VALUE_TYPE_INT64_LIST: Int64List
 * - #FL_VALUE_TYPE_FLOAT32_LIST: Float32List
 * - #FL_VALUE_TYPE_FLOAT_LIST: Float64List
 * - #FL_VALUE_TYPE_LIST: List<dynamic>
 * - #FL_VALUE_TYPE_MAP: Map<dynamic>
 * - #FL_VALUE_TYPE_CUSTOM: (custom type)
 *
 * See #FlMessageCodec to encode and decode these values.
 */
typedef struct _FlValue FlValue;

/**
 * FlValueType:
 * @FL_VALUE_TYPE_NULL: The null value.
 * @FL_VALUE_TYPE_BOOL: A boolean.
 * @FL_VALUE_TYPE_INT: A 64 bit signed integer.
 * @FL_VALUE_TYPE_FLOAT: A 64 bit floating point number.
 * @FL_VALUE_TYPE_STRING: UTF-8 text.
 * @FL_VALUE_TYPE_UINT8_LIST: An ordered list of unsigned 8 bit integers.
 * @FL_VALUE_TYPE_INT32_LIST: An ordered list of 32 bit integers.
 * @FL_VALUE_TYPE_INT64_LIST: An ordered list of 64 bit integers.
 * @FL_VALUE_TYPE_FLOAT_LIST: An ordered list of floating point numbers.
 * @FL_VALUE_TYPE_LIST: An ordered list of #FlValue objects.
 * @FL_VALUE_TYPE_MAP: A map of #FlValue objects keyed by #FlValue object.
 * @FL_VALUE_TYPE_FLOAT32_LIST: An ordered list of 32bit floating point numbers.
 * @FL_VALUE_TYPE_CUSTOM: A custom value.
 *
 * Types of #FlValue.
 */
typedef enum {
  // Parts of the public API, so fixing the names is a breaking change.
  FL_VALUE_TYPE_NULL,
  FL_VALUE_TYPE_BOOL,
  FL_VALUE_TYPE_INT,
  FL_VALUE_TYPE_FLOAT,
  FL_VALUE_TYPE_STRING,
  FL_VALUE_TYPE_UINT8_LIST,
  FL_VALUE_TYPE_INT32_LIST,
  FL_VALUE_TYPE_INT64_LIST,
  FL_VALUE_TYPE_FLOAT_LIST,
  FL_VALUE_TYPE_LIST,
  FL_VALUE_TYPE_MAP,
  FL_VALUE_TYPE_FLOAT32_LIST,
  FL_VALUE_TYPE_CUSTOM,
} FlValueType;

/**
 * fl_value_new_null:
 *
 * Creates an #FlValue that contains a null value. The equivalent Dart type is
 * null.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_null();

/**
 * fl_value_new_bool:
 * @value: the value.
 *
 * Creates an #FlValue that contains a boolean value. The equivalent Dart type
 * is a bool.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_bool(bool value);

/**
 * fl_value_new_int:
 * @value: the value.
 *
 * Creates an #FlValue that contains an integer number. The equivalent Dart type
 * is a num.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_int(int64_t value);

/**
 * fl_value_new_float:
 * @value: the value.
 *
 * Creates an #FlValue that contains a floating point number. The equivalent
 * Dart type is a num.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_float(double value);

/**
 * fl_value_new_string:
 * @value: a %NULL-terminated UTF-8 string.
 *
 * Creates an #FlValue that contains UTF-8 text. The equivalent Dart type is a
 * String.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_string(const gchar* value);

/**
 * fl_value_new_string_sized:
 * @value: a buffer containing UTF-8 text. It does not require a nul terminator.
 * @value_length: the number of bytes to use from @value.
 *
 * Creates an #FlValue that contains UTF-8 text. The equivalent Dart type is a
 * String.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_string_sized(const gchar* value, size_t value_length);

/**
 * fl_value_new_uint8_list:
 * @value: an array of unsigned 8 bit integers.
 * @value_length: number of elements in @value.
 *
 * Creates an ordered list containing 8 bit unsigned integers. The data is
 * copied. The equivalent Dart type is a Uint8List.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_uint8_list(const uint8_t* value, size_t value_length);

/**
 * fl_value_new_uint8_list_from_bytes:
 * @value: a #GBytes.
 *
 * Creates an ordered list containing 8 bit unsigned integers. The data is
 * copied. The equivalent Dart type is a Uint8List.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_uint8_list_from_bytes(GBytes* value);

/**
 * fl_value_new_int32_list:
 * @value: an array of signed 32 bit integers.
 * @value_length: number of elements in @value.
 *
 * Creates an ordered list containing 32 bit integers. The equivalent Dart type
 * is a Int32List.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_int32_list(const int32_t* value, size_t value_length);

/**
 * fl_value_new_int64_list:
 * @value: an array of signed 64 bit integers.
 * @value_length: number of elements in @value.
 *
 * Creates an ordered list containing 64 bit integers. The equivalent Dart type
 * is a Int64List.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_int64_list(const int64_t* value, size_t value_length);

/**
 * fl_value_new_float32_list:
 * @value: an array of floating point numbers.
 * @value_length: number of elements in @value.
 *
 * Creates an ordered list containing 32 bit floating point numbers.
 * The equivalent Dart type is a Float32List.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_float32_list(const float* value, size_t value_length);

/**
 * fl_value_new_float_list:
 * @value: an array of floating point numbers.
 * @value_length: number of elements in @value.
 *
 * Creates an ordered list containing floating point numbers. The equivalent
 * Dart type is a Float64List.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_float_list(const double* value, size_t value_length);

/**
 * fl_value_new_list:
 *
 * Creates an ordered list. Children can be added to the list using
 * fl_value_append(). The children are accessed using fl_value_get_length()
 * and fl_value_get_list_value(). The equivalent Dart type is a List<dynamic>.
 *
 * The following example shows a simple list of values:
 *
 * |[<!-- language="C" -->
 *   g_autoptr(FlValue) value = fl_value_new_list ();
 *   fl_value_append_take (value, fl_value_new_string ("one");
 *   fl_value_append_take (value, fl_value_new_int (2);
 *   fl_value_append_take (value, fl_value_new_float (3.0);
 * ]|
 *
 * This value can be decoded using:
 *
 * |[<!-- language="C" -->
 *   g_assert (fl_value_get_type (value) == FL_VALUE_TYPE_LIST);
 *   for (size_t i = 0; i < fl_value_get_length (value); i++) {
 *     FlValue *child = fl_value_get_list_value (value, i);
 *     process_value (child);
 *   }
 * ]|
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_list();

/**
 * fl_value_new_list_from_strv:
 * @value: a %NULL-terminated array of strings.
 *
 * Creates an ordered list containing #FlString values.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_list_from_strv(const gchar* const* value);

/**
 * fl_value_new_map:
 *
 * Creates an ordered associative array. Children can be added to the map
 * using fl_value_set(), fl_value_set_take(), fl_value_set_string(),
 * fl_value_set_string_take(). The children are accessed using
 * fl_value_get_length(), fl_value_get_map_key(), fl_value_get_map_value(),
 * fl_value_lookup() and fl_value_lookup_string(). The equivalent Dart type is a
 * Map<dynamic>.
 *
 * The following example shows how to create a map of values keyed by strings:
 *
 * |[<!-- language="C" -->
 *   g_autoptr(FlValue) value = fl_value_new_map ();
 *   fl_value_set_string_take (value, "name", fl_value_new_string ("Gandalf"));
 *   fl_value_set_string_take (value, "occupation",
 *                             fl_value_new_string ("Wizard"));
 *   fl_value_set_string_take (value, "age", fl_value_new_int (2019));
 * ]|
 *
 * This value can be decoded using:
 * |[<!-- language="C" -->
 *   g_assert (fl_value_get_type (value) == FL_VALUE_TYPE_MAP);
 *   FlValue *name = fl_value_lookup_string (value, "name");
 *   g_assert (fl_value_get_type (name) == FL_VALUE_TYPE_STRING);
 *   FlValue *age = fl_value_lookup_string (value, "age");
 *   g_assert (fl_value_get_type (age) == FL_VALUE_TYPE_INT);
 *   g_message ("Next customer is %s (%d years old)",
 *              fl_value_get_string (name),
 *              fl_value_get_int (age));
 * ]|
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_map();

/**
 * fl_value_new_custom:
 * @type: an ID for this type.
 * @value: pointer to the custom value.
 * @destroy_notify: function to call when @value is no longer required.
 *
 * Creates a new custom data type. The Dart side of the channel must have
 * equivalent custom code to access this object.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_custom(int type,
                             gconstpointer value,
                             GDestroyNotify destroy_notify);

/**
 * fl_value_new_custom_object:
 * @type: an ID for this type.
 * @object: the custom object.
 *
 * Creates a new custom data type. The Dart side of the channel must have
 * equivalent custom code to access this object.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_custom_object(int type, GObject* object);

/**
 * fl_value_new_custom_object_take:
 * @type: an ID for this type.
 * @object: (transfer full): the custom object.
 *
 * Creates a new custom data type. The Dart side of the channel must have
 * equivalent custom code to access this object. Ownership of @object is taken.
 *
 * Returns: a new #FlValue.
 */
FlValue* fl_value_new_custom_object_take(int type, GObject* object);

/**
 * fl_value_ref:
 * @value: an #FlValue.
 *
 * Increases the reference count of an #FlValue.
 *
 * Returns: the value that was referenced.
 */
FlValue* fl_value_ref(FlValue* value);

/**
 * fl_value_unref:
 * @value: an #FlValue.
 *
 * Decreases the reference count of an #FlValue. When the reference count hits
 * zero @value is destroyed and no longer valid.
 */
void fl_value_unref(FlValue* value);

/**
 * fl_value_get_type:
 * @value: an #FlValue.
 *
 * Gets the type of @value.
 *
 * Returns: an #FlValueType.
 */
FlValueType fl_value_get_type(FlValue* value);

/**
 * fl_value_equal:
 * @a: an #FlValue.
 * @b: an #FlValue.
 *
 * Compares two #FlValue to see if they are equivalent. Two values are
 * considered equivalent if they are of the same type and their data is the same
 * including any child values. For values of type #FL_VALUE_TYPE_MAP the order
 * of the values does not matter.
 *
 * Returns: %TRUE if both values are equivalent.
 */
bool fl_value_equal(FlValue* a, FlValue* b);

/**
 * fl_value_append:
 * @value: an #FlValue of type #FL_VALUE_TYPE_LIST.
 * @child: an #FlValue.
 *
 * Adds @child to the end of @value. Calling this with an #FlValue that is not
 * of type #FL_VALUE_TYPE_LIST is a programming error.
 */
void fl_value_append(FlValue* value, FlValue* child);

/**
 * fl_value_append_take:
 * @value: an #FlValue of type #FL_VALUE_TYPE_LIST.
 * @child: (transfer full): an #FlValue.
 *
 * Adds @child to the end of @value. Ownership of @child is taken by @value.
 * Calling this with an #FlValue that is not of type #FL_VALUE_TYPE_LIST is a
 * programming error.
 */
void fl_value_append_take(FlValue* value, FlValue* child);

/**
 * fl_value_set:
 * @value: an #FlValue of type #FL_VALUE_TYPE_MAP.
 * @key: an #FlValue.
 * @child_value: an #FlValue.
 *
 * Sets @key in @value to @child_value. If an existing value was in the map with
 * the same key it is replaced. Calling this with an #FlValue that is not of
 * type #FL_VALUE_TYPE_MAP is a programming error.
 */
void fl_value_set(FlValue* value, FlValue* key, FlValue* child_value);

/**
 * fl_value_set_take:
 * @value: an #FlValue of type #FL_VALUE_TYPE_MAP.
 * @key: (transfer full): an #FlValue.
 * @child_value: (transfer full): an #FlValue.
 *
 * Sets @key in @value to @child_value. Ownership of both @key and @child_value
 * is taken by @value. If an existing value was in the map with the same key it
 * is replaced. Calling this with an #FlValue that is not of type
 * #FL_VALUE_TYPE_MAP is a programming error.
 */
void fl_value_set_take(FlValue* value, FlValue* key, FlValue* child_value);

/**
 * fl_value_set_string:
 * @value: an #FlValue of type #FL_VALUE_TYPE_MAP.
 * @key: a UTF-8 text key.
 * @child_value: an #FlValue.
 *
 * Sets a value in the map with a text key. If an existing value was in the map
 * with the same key it is replaced. Calling this with an #FlValue that is not
 * of type #FL_VALUE_TYPE_MAP is a programming error.
 */
void fl_value_set_string(FlValue* value,
                         const gchar* key,
                         FlValue* child_value);

/**
 * fl_value_set_string_take:
 * @value: an #FlValue of type #FL_VALUE_TYPE_MAP.
 * @key: a UTF-8 text key.
 * @child_value: (transfer full): an #FlValue.
 *
 * Sets a value in the map with a text key, taking ownership of the value. If an
 * existing value was in the map with the same key it is replaced. Calling this
 * with an #FlValue that is not of type #FL_VALUE_TYPE_MAP is a programming
 * error.
 */
void fl_value_set_string_take(FlValue* value,
                              const gchar* key,
                              FlValue* child_value);

/**
 * fl_value_get_bool:
 * @value: an #FlValue of type #FL_VALUE_TYPE_BOOL.
 *
 * Gets the boolean value of @value. Calling this with an #FlValue that is
 * not of type #FL_VALUE_TYPE_BOOL is a programming error.
 *
 * Returns: a boolean value.
 */
bool fl_value_get_bool(FlValue* value);

/**
 * fl_value_get_int:
 * @value: an #FlValue of type #FL_VALUE_TYPE_INT.
 *
 * Gets the integer number of @value. Calling this with an #FlValue that is
 * not of type #FL_VALUE_TYPE_INT is a programming error.
 *
 * Returns: an integer number.
 */
int64_t fl_value_get_int(FlValue* value);

/**
 * fl_value_get_float:
 * @value: an #FlValue of type #FL_VALUE_TYPE_FLOAT.
 *
 * Gets the floating point number of @value. Calling this with an #FlValue
 * that is not of type #FL_VALUE_TYPE_FLOAT is a programming error.
 *
 * Returns: a floating point number.
 */
double fl_value_get_float(FlValue* value);

/**
 * fl_value_get_string:
 * @value: an #FlValue of type #FL_VALUE_TYPE_STRING.
 *
 * Gets the UTF-8 text contained in @value. Calling this with an #FlValue
 * that is not of type #FL_VALUE_TYPE_STRING is a programming error.
 *
 * Returns: a UTF-8 encoded string.
 */
const gchar* fl_value_get_string(FlValue* value);

/**
 * fl_value_get_length:
 * @value: an #FlValue of type #FL_VALUE_TYPE_UINT8_LIST,
 * #FL_VALUE_TYPE_INT32_LIST, #FL_VALUE_TYPE_INT64_LIST,
 * #FL_VALUE_TYPE_FLOAT32_LIST, #FL_VALUE_TYPE_FLOAT_LIST, #FL_VALUE_TYPE_LIST
 * or #FL_VALUE_TYPE_MAP.
 *
 * Gets the number of elements @value contains. This is only valid for list
 * and map types. Calling this with other types is a programming error.
 *
 * Returns: the number of elements inside @value.
 */
size_t fl_value_get_length(FlValue* value);

/**
 * fl_value_get_uint8_list:
 * @value: an #FlValue of type #FL_VALUE_TYPE_UINT8_LIST.
 *
 * Gets the array of unisigned 8 bit integers @value contains. The data
 * contains fl_value_get_length() elements. Calling this with an #FlValue that
 * is not of type #FL_VALUE_TYPE_UINT8_LIST is a programming error.
 *
 * Returns: an array of unsigned 8 bit integers.
 */
const uint8_t* fl_value_get_uint8_list(FlValue* value);

/**
 * fl_value_get_int32_list:
 * @value: an #FlValue of type #FL_VALUE_TYPE_INT32_LIST.
 *
 * Gets the array of 32 bit integers @value contains. The data contains
 * fl_value_get_length() elements. Calling this with an #FlValue that is not of
 * type #FL_VALUE_TYPE_INT32_LIST is a programming error.
 *
 * Returns: an array of 32 bit integers.
 */
const int32_t* fl_value_get_int32_list(FlValue* value);

/**
 * fl_value_get_int64_list:
 * @value: an #FlValue of type #FL_VALUE_TYPE_INT64_LIST.
 *
 * Gets the array of 64 bit integers @value contains. The data contains
 * fl_value_get_length() elements. Calling this with an #FlValue that is not of
 * type #FL_VALUE_TYPE_INT64_LIST is a programming error.
 *
 * Returns: an array of 64 bit integers.
 */
const int64_t* fl_value_get_int64_list(FlValue* value);

/**
 * fl_value_get_float32_list:
 * @value: an #FlValue of type #FL_VALUE_TYPE_FLOAT32_LIST.
 *
 * Gets the array of floating point numbers @value contains. The data
 * contains fl_value_get_length() elements. Calling this with an #FlValue that
 * is not of type #FL_VALUE_TYPE_FLOAT32_LIST is a programming error.
 *
 * Returns: an array of floating point numbers.
 */
const float* fl_value_get_float32_list(FlValue* value);

/**
 * fl_value_get_float_list:
 * @value: an #FlValue of type #FL_VALUE_TYPE_FLOAT_LIST.
 *
 * Gets the array of floating point numbers @value contains. The data
 * contains fl_value_get_length() elements. Calling this with an #FlValue that
 * is not of type #FL_VALUE_TYPE_FLOAT_LIST is a programming error.
 *
 * Returns: an array of floating point numbers.
 */
const double* fl_value_get_float_list(FlValue* value);

/**
 * fl_value_get_list_value:
 * @value: an #FlValue of type #FL_VALUE_TYPE_LIST.
 * @index: an index in the list.
 *
 * Gets a child element of the list. It is a programming error to request an
 * index that is outside the size of the list as returned from
 * fl_value_get_length(). Calling this with an #FlValue that is not of type
 * #FL_VALUE_TYPE_LIST is a programming error.
 *
 * Returns: an #FlValue.
 */
FlValue* fl_value_get_list_value(FlValue* value, size_t index);

/**
 * fl_value_get_map_key:
 * @value: an #FlValue of type #FL_VALUE_TYPE_MAP.
 * @index: an index in the map.
 *
 * Gets a key from the map. It is a programming error to request an index that
 * is outside the size of the list as returned from fl_value_get_length().
 * Calling this with an #FlValue that is not of type #FL_VALUE_TYPE_MAP is a
 * programming error.
 *
 * Returns: an #FlValue.
 */
FlValue* fl_value_get_map_key(FlValue* value, size_t index);

/**
 * fl_value_get_map_value:
 * @value: an #FlValue of type #FL_VALUE_TYPE_MAP.
 * @index: an index in the map.
 *
 * Gets a value from the map. It is a programming error to request an index that
 * is outside the size of the list as returned from fl_value_get_length().
 * Calling this with an #FlValue that is not of type #FL_VALUE_TYPE_MAP is a
 * programming error.
 *
 * Returns: an #FlValue.
 */
FlValue* fl_value_get_map_value(FlValue* value, size_t index);

/**
 * fl_value_lookup:
 * @value: an #FlValue of type #FL_VALUE_TYPE_MAP.
 * @key: a key value.
 *
 * Gets the map entry that matches @key. Keys are checked using
 * fl_value_equal(). Calling this with an #FlValue that is not of type
 * #FL_VALUE_TYPE_MAP is a programming error.
 *
 * Map lookups are not optimized for performance - if you have a large map or
 * need frequent access you should copy the data into another structure, e.g.
 * #GHashTable.
 *
 * Returns: (allow-none): the value with this key or %NULL if not one present.
 */
FlValue* fl_value_lookup(FlValue* value, FlValue* key);

/**
 * fl_value_lookup_string:
 * @value: an #FlValue of type #FL_VALUE_TYPE_MAP.
 * @key: a key value.
 *
 * Gets the map entry that matches @key. Keys are checked using
 * fl_value_equal(). Calling this with an #FlValue that is not of type
 * #FL_VALUE_TYPE_MAP is a programming error.
 *
 * Map lookups are not optimized for performance - if you have a large map or
 * need frequent access you should copy the data into another structure, e.g.
 * #GHashTable.
 *
 * Returns: (allow-none): the value with this key or %NULL if not one present.
 */
FlValue* fl_value_lookup_string(FlValue* value, const gchar* key);

/**
 * fl_value_get_custom_type:
 * @value: an #FlValue of type #FL_VALUE_TYPE_CUSTOM.
 *
 * Gets the type ID for this custom type.
 *
 * Returns: a type ID.
 */
int fl_value_get_custom_type(FlValue* value);

/**
 * fl_value_get_custom_value:
 * @value: an #FlValue of type #FL_VALUE_TYPE_CUSTOM.
 *
 * Gets the address of the custom value.
 *
 * Returns: a pointer to the custom value.
 */
gconstpointer fl_value_get_custom_value(FlValue* value);

/**
 * fl_value_get_custom_value_object:
 * @value: an #FlValue of type #FL_VALUE_TYPE_CUSTOM.
 *
 * Gets the custom value as an object.
 *
 * Returns: an object.
 */
GObject* fl_value_get_custom_value_object(FlValue* value);

/**
 * fl_value_to_string:
 * @value: an #FlValue.
 *
 * Converts an #FlValue to a text representation, suitable for logging purposes.
 * The text is formatted to be the equivalent of Dart toString() methods.
 *
 * Returns: UTF-8 text.
 */
gchar* fl_value_to_string(FlValue* value);

G_DEFINE_AUTOPTR_CLEANUP_FUNC(FlValue, fl_value_unref)

G_END_DECLS

#endif  // FLUTTER_SHELL_PLATFORM_LINUX_PUBLIC_FLUTTER_LINUX_FL_VALUE_H_
