// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_value.h"

#include <gmodule.h>

#include "gtest/gtest.h"

TEST(FlDartProjectTest, Null) {
  g_autoptr(FlValue) value = fl_value_new_null();
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_NULL);
}

TEST(FlValueTest, NullEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_null();
  g_autoptr(FlValue) value2 = fl_value_new_null();
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, NullToString) {
  g_autoptr(FlValue) value = fl_value_new_null();
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "null");
}

TEST(FlValueTest, BoolTrue) {
  g_autoptr(FlValue) value = fl_value_new_bool(TRUE);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_BOOL);
  EXPECT_TRUE(fl_value_get_bool(value));
}

TEST(FlValueTest, BoolFalse) {
  g_autoptr(FlValue) value = fl_value_new_bool(FALSE);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_BOOL);
  EXPECT_FALSE(fl_value_get_bool(value));
}

TEST(FlValueTest, BoolEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_bool(TRUE);
  g_autoptr(FlValue) value2 = fl_value_new_bool(TRUE);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, BoolNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_bool(TRUE);
  g_autoptr(FlValue) value2 = fl_value_new_bool(FALSE);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, BoolTrueToString) {
  g_autoptr(FlValue) value = fl_value_new_bool(TRUE);
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "true");
}

TEST(FlValueTest, BoolFalseToString) {
  g_autoptr(FlValue) value = fl_value_new_bool(FALSE);
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "false");
}

TEST(FlValueTest, IntZero) {
  g_autoptr(FlValue) value = fl_value_new_int(0);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), 0);
}

TEST(FlValueTest, IntOne) {
  g_autoptr(FlValue) value = fl_value_new_int(1);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), 1);
}

TEST(FlValueTest, IntMinusOne) {
  g_autoptr(FlValue) value = fl_value_new_int(-1);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), -1);
}

TEST(FlValueTest, IntMin) {
  g_autoptr(FlValue) value = fl_value_new_int(G_MININT64);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), G_MININT64);
}

TEST(FlValueTest, IntMax) {
  g_autoptr(FlValue) value = fl_value_new_int(G_MAXINT64);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(value), G_MAXINT64);
}

TEST(FlValueTest, IntEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_int(42);
  g_autoptr(FlValue) value2 = fl_value_new_int(42);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, IntNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_int(42);
  g_autoptr(FlValue) value2 = fl_value_new_int(99);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, IntToString) {
  g_autoptr(FlValue) value = fl_value_new_int(42);
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "42");
}

TEST(FlValueTest, FloatZero) {
  g_autoptr(FlValue) value = fl_value_new_float(0.0);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_float(value), 0.0);
}

TEST(FlValueTest, FloatOne) {
  g_autoptr(FlValue) value = fl_value_new_float(1.0);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_float(value), 1.0);
}

TEST(FlValueTest, FloatMinusOne) {
  g_autoptr(FlValue) value = fl_value_new_float(-1.0);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_float(value), -1.0);
}

TEST(FlValueTest, FloatPi) {
  g_autoptr(FlValue) value = fl_value_new_float(M_PI);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_float(value), M_PI);
}

TEST(FlValueTest, FloatEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_float(M_PI);
  g_autoptr(FlValue) value2 = fl_value_new_float(M_PI);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, FloatNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_float(M_PI);
  g_autoptr(FlValue) value2 = fl_value_new_float(M_E);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, FloatToString) {
  g_autoptr(FlValue) value = fl_value_new_float(M_PI);
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "3.1415926535897931");
}

TEST(FlValueTest, String) {
  g_autoptr(FlValue) value = fl_value_new_string("hello");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(value), "hello");
}

TEST(FlValueTest, StringEmpty) {
  g_autoptr(FlValue) value = fl_value_new_string("");
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(value), "");
}

TEST(FlValueTest, StringSized) {
  g_autoptr(FlValue) value = fl_value_new_string_sized("hello", 2);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(value), "he");
}

TEST(FlValueTest, StringSizedNullptr) {
  g_autoptr(FlValue) value = fl_value_new_string_sized(nullptr, 0);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(value), "");
}

TEST(FlValueTest, StringSizedZeroLength) {
  g_autoptr(FlValue) value = fl_value_new_string_sized("hello", 0);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(value), "");
}

TEST(FlValueTest, StringEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_string("hello");
  g_autoptr(FlValue) value2 = fl_value_new_string("hello");
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, StringNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_string("hello");
  g_autoptr(FlValue) value2 = fl_value_new_string("world");
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, StringToString) {
  g_autoptr(FlValue) value = fl_value_new_string("hello");
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "hello");
}

TEST(FlValueTest, Uint8List) {
  uint8_t data[] = {0x00, 0x01, 0xFE, 0xFF};
  g_autoptr(FlValue) value = fl_value_new_uint8_list(data, 4);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_UINT8_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(4));
  EXPECT_EQ(fl_value_get_uint8_list(value)[0], 0x00);
  EXPECT_EQ(fl_value_get_uint8_list(value)[1], 0x01);
  EXPECT_EQ(fl_value_get_uint8_list(value)[2], 0xFE);
  EXPECT_EQ(fl_value_get_uint8_list(value)[3], 0xFF);
}

TEST(FlValueTest, Uint8ListNullptr) {
  g_autoptr(FlValue) value = fl_value_new_uint8_list(nullptr, 0);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_UINT8_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlValueTest, Uint8ListEqual) {
  uint8_t data1[] = {1, 2, 3};
  g_autoptr(FlValue) value1 = fl_value_new_uint8_list(data1, 3);
  uint8_t data2[] = {1, 2, 3};
  g_autoptr(FlValue) value2 = fl_value_new_uint8_list(data2, 3);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Uint8ListEmptyEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_uint8_list(nullptr, 0);
  g_autoptr(FlValue) value2 = fl_value_new_uint8_list(nullptr, 0);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Uint8ListNotEqualSameSize) {
  uint8_t data1[] = {1, 2, 3};
  g_autoptr(FlValue) value1 = fl_value_new_uint8_list(data1, 3);
  uint8_t data2[] = {1, 2, 4};
  g_autoptr(FlValue) value2 = fl_value_new_uint8_list(data2, 3);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Uint8ListNotEqualDifferentSize) {
  uint8_t data1[] = {1, 2, 3};
  g_autoptr(FlValue) value1 = fl_value_new_uint8_list(data1, 3);
  uint8_t data2[] = {1, 2, 3, 4};
  g_autoptr(FlValue) value2 = fl_value_new_uint8_list(data2, 4);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Uint8ListEmptyNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_uint8_list(nullptr, 0);
  uint8_t data[] = {1, 2, 3};
  g_autoptr(FlValue) value2 = fl_value_new_uint8_list(data, 3);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Uint8ListToString) {
  uint8_t data[] = {0x00, 0x01, 0xFE, 0xFF};
  g_autoptr(FlValue) value = fl_value_new_uint8_list(data, 4);
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "[0, 1, 254, 255]");
}

TEST(FlValueTest, Int32List) {
  int32_t data[] = {0, -1, G_MAXINT32, G_MININT32};
  g_autoptr(FlValue) value = fl_value_new_int32_list(data, 4);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT32_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(4));
  EXPECT_EQ(fl_value_get_int32_list(value)[0], 0);
  EXPECT_EQ(fl_value_get_int32_list(value)[1], -1);
  EXPECT_EQ(fl_value_get_int32_list(value)[2], G_MAXINT32);
  EXPECT_EQ(fl_value_get_int32_list(value)[3], G_MININT32);
}

TEST(FlValueTest, Int32ListNullptr) {
  g_autoptr(FlValue) value = fl_value_new_int32_list(nullptr, 0);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT32_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlValueTest, Int32ListEqual) {
  int32_t data1[] = {0, G_MAXINT32, G_MININT32};
  g_autoptr(FlValue) value1 = fl_value_new_int32_list(data1, 3);
  int32_t data2[] = {0, G_MAXINT32, G_MININT32};
  g_autoptr(FlValue) value2 = fl_value_new_int32_list(data2, 3);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int32ListEmptyEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_int32_list(nullptr, 0);
  g_autoptr(FlValue) value2 = fl_value_new_int32_list(nullptr, 0);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int32ListNotEqualSameSize) {
  int32_t data1[] = {0, G_MAXINT32, G_MININT32};
  g_autoptr(FlValue) value1 = fl_value_new_int32_list(data1, 3);
  int32_t data2[] = {0, G_MININT32, G_MAXINT32};
  g_autoptr(FlValue) value2 = fl_value_new_int32_list(data2, 3);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int32ListNotEqualDifferentSize) {
  int32_t data1[] = {0, G_MAXINT32, G_MININT32};
  g_autoptr(FlValue) value1 = fl_value_new_int32_list(data1, 3);
  int32_t data2[] = {0, G_MAXINT32, G_MININT32, -1};
  g_autoptr(FlValue) value2 = fl_value_new_int32_list(data2, 4);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int32ListEmptyNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_int32_list(nullptr, 0);
  int32_t data[] = {0, G_MAXINT32, G_MININT32};
  g_autoptr(FlValue) value2 = fl_value_new_int32_list(data, 3);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int32ListToString) {
  int32_t data[] = {0, G_MAXINT32, G_MININT32};
  g_autoptr(FlValue) value = fl_value_new_int32_list(data, 3);
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "[0, 2147483647, -2147483648]");
}

TEST(FlValueTest, Int64List) {
  int64_t data[] = {0, -1, G_MAXINT64, G_MININT64};
  g_autoptr(FlValue) value = fl_value_new_int64_list(data, 4);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT64_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(4));
  EXPECT_EQ(fl_value_get_int64_list(value)[0], 0);
  EXPECT_EQ(fl_value_get_int64_list(value)[1], -1);
  EXPECT_EQ(fl_value_get_int64_list(value)[2], G_MAXINT64);
  EXPECT_EQ(fl_value_get_int64_list(value)[3], G_MININT64);
}

TEST(FlValueTest, Int64ListNullptr) {
  g_autoptr(FlValue) value = fl_value_new_int64_list(nullptr, 0);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_INT64_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlValueTest, Int64ListEqual) {
  int64_t data1[] = {0, G_MAXINT64, G_MININT64};
  g_autoptr(FlValue) value1 = fl_value_new_int64_list(data1, 3);
  int64_t data2[] = {0, G_MAXINT64, G_MININT64};
  g_autoptr(FlValue) value2 = fl_value_new_int64_list(data2, 3);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int64ListEmptyEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_int64_list(nullptr, 0);
  g_autoptr(FlValue) value2 = fl_value_new_int64_list(nullptr, 0);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int64ListNotEqualSameSize) {
  int64_t data1[] = {0, G_MAXINT64, G_MININT64};
  g_autoptr(FlValue) value1 = fl_value_new_int64_list(data1, 3);
  int64_t data2[] = {0, G_MININT64, G_MAXINT64};
  g_autoptr(FlValue) value2 = fl_value_new_int64_list(data2, 3);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int64ListNotEqualDifferentSize) {
  int64_t data1[] = {0, G_MAXINT64, G_MININT64};
  g_autoptr(FlValue) value1 = fl_value_new_int64_list(data1, 3);
  int64_t data2[] = {0, G_MAXINT64, G_MININT64, -1};
  g_autoptr(FlValue) value2 = fl_value_new_int64_list(data2, 4);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int64ListEmptyNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_int64_list(nullptr, 0);
  int64_t data[] = {0, G_MAXINT64, G_MININT64};
  g_autoptr(FlValue) value2 = fl_value_new_int64_list(data, 3);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int64ListToString) {
  int64_t data[] = {0, G_MAXINT64, G_MININT64};
  g_autoptr(FlValue) value = fl_value_new_int64_list(data, 3);
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "[0, 9223372036854775807, -9223372036854775808]");
}

TEST(FlValueTest, FloatList) {
  double data[] = {0.0, -1.0, M_PI};
  g_autoptr(FlValue) value = fl_value_new_float_list(data, 3);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(3));
  EXPECT_EQ(fl_value_get_float_list(value)[0], 0);
  EXPECT_EQ(fl_value_get_float_list(value)[1], -1.0);
  EXPECT_EQ(fl_value_get_float_list(value)[2], M_PI);
}

TEST(FlValueTest, FloatListNullptr) {
  g_autoptr(FlValue) value = fl_value_new_float_list(nullptr, 0);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_FLOAT_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlValueTest, FloatListEqual) {
  double data1[] = {0, -0.5, M_PI};
  g_autoptr(FlValue) value1 = fl_value_new_float_list(data1, 3);
  double data2[] = {0, -0.5, M_PI};
  g_autoptr(FlValue) value2 = fl_value_new_float_list(data2, 3);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, FloatListEmptyEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_float_list(nullptr, 0);
  g_autoptr(FlValue) value2 = fl_value_new_float_list(nullptr, 0);
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, FloatListNotEqualSameSize) {
  double data1[] = {0, -0.5, M_PI};
  g_autoptr(FlValue) value1 = fl_value_new_float_list(data1, 3);
  double data2[] = {0, -0.5, M_E};
  g_autoptr(FlValue) value2 = fl_value_new_float_list(data2, 3);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, FloatListNotEqualDifferentSize) {
  double data1[] = {0, -0.5, M_PI};
  g_autoptr(FlValue) value1 = fl_value_new_float_list(data1, 3);
  double data2[] = {0, -0.5, M_PI, 42};
  g_autoptr(FlValue) value2 = fl_value_new_float_list(data2, 4);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, FloatListEmptyNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_float_list(nullptr, 0);
  double data[] = {0, -0.5, M_PI};
  g_autoptr(FlValue) value2 = fl_value_new_float_list(data, 3);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, FloatListToString) {
  double data[] = {0, -0.5, M_PI};
  g_autoptr(FlValue) value = fl_value_new_float_list(data, 3);
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "[0.0, -0.5, 3.1415926535897931]");
}

TEST(FlValueTest, ListEmpty) {
  g_autoptr(FlValue) value = fl_value_new_list();
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlValueTest, ListAdd) {
  g_autoptr(FlValue) value = fl_value_new_list();
  g_autoptr(FlValue) child = fl_value_new_null();
  fl_value_append(value, child);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(1));
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 0)),
            FL_VALUE_TYPE_NULL);
}

TEST(FlValueTest, ListAddTake) {
  g_autoptr(FlValue) value = fl_value_new_list();
  fl_value_append_take(value, fl_value_new_null());
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(1));
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 0)),
            FL_VALUE_TYPE_NULL);
}

TEST(FlValueTest, ListChildTypes) {
  g_autoptr(FlValue) value = fl_value_new_list();
  fl_value_append_take(value, fl_value_new_null());
  fl_value_append_take(value, fl_value_new_bool(TRUE));
  fl_value_append_take(value, fl_value_new_int(42));
  fl_value_append_take(value, fl_value_new_float(M_PI));
  fl_value_append_take(value, fl_value_new_uint8_list(nullptr, 0));
  fl_value_append_take(value, fl_value_new_int32_list(nullptr, 0));
  fl_value_append_take(value, fl_value_new_int64_list(nullptr, 0));
  fl_value_append_take(value, fl_value_new_float_list(nullptr, 0));
  fl_value_append_take(value, fl_value_new_list());
  fl_value_append_take(value, fl_value_new_map());
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(10));
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 0)),
            FL_VALUE_TYPE_NULL);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 1)),
            FL_VALUE_TYPE_BOOL);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 2)),
            FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 3)),
            FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 4)),
            FL_VALUE_TYPE_UINT8_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 5)),
            FL_VALUE_TYPE_INT32_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 6)),
            FL_VALUE_TYPE_INT64_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 7)),
            FL_VALUE_TYPE_FLOAT_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 8)),
            FL_VALUE_TYPE_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_list_value(value, 9)),
            FL_VALUE_TYPE_MAP);
}

TEST(FlValueTest, ListStrv) {
  g_auto(GStrv) words = g_strsplit("hello:world", ":", -1);
  g_autoptr(FlValue) value = fl_value_new_list_from_strv(words);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(2));
  ASSERT_EQ(fl_value_get_type(fl_value_get_list_value(value, 0)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_list_value(value, 0)), "hello");
  ASSERT_EQ(fl_value_get_type(fl_value_get_list_value(value, 1)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_list_value(value, 1)), "world");
}

TEST(FlValueTest, ListStrvEmpty) {
  g_auto(GStrv) words = g_strsplit("", ":", -1);
  g_autoptr(FlValue) value = fl_value_new_list_from_strv(words);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_LIST);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlValueTest, ListEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_list();
  fl_value_append_take(value1, fl_value_new_int(1));
  fl_value_append_take(value1, fl_value_new_int(2));
  fl_value_append_take(value1, fl_value_new_int(3));
  g_autoptr(FlValue) value2 = fl_value_new_list();
  fl_value_append_take(value2, fl_value_new_int(1));
  fl_value_append_take(value2, fl_value_new_int(2));
  fl_value_append_take(value2, fl_value_new_int(3));
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, ListEmptyEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_list();
  g_autoptr(FlValue) value2 = fl_value_new_list();
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, ListNotEqualSameSize) {
  g_autoptr(FlValue) value1 = fl_value_new_list();
  fl_value_append_take(value1, fl_value_new_int(1));
  fl_value_append_take(value1, fl_value_new_int(2));
  fl_value_append_take(value1, fl_value_new_int(3));
  g_autoptr(FlValue) value2 = fl_value_new_list();
  fl_value_append_take(value2, fl_value_new_int(1));
  fl_value_append_take(value2, fl_value_new_int(2));
  fl_value_append_take(value2, fl_value_new_int(4));
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, ListNotEqualDifferentSize) {
  g_autoptr(FlValue) value1 = fl_value_new_list();
  fl_value_append_take(value1, fl_value_new_int(1));
  fl_value_append_take(value1, fl_value_new_int(2));
  fl_value_append_take(value1, fl_value_new_int(3));
  g_autoptr(FlValue) value2 = fl_value_new_list();
  fl_value_append_take(value2, fl_value_new_int(1));
  fl_value_append_take(value2, fl_value_new_int(2));
  fl_value_append_take(value2, fl_value_new_int(3));
  fl_value_append_take(value2, fl_value_new_int(4));
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, ListEmptyNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_list();
  g_autoptr(FlValue) value2 = fl_value_new_list();
  fl_value_append_take(value2, fl_value_new_int(1));
  fl_value_append_take(value2, fl_value_new_int(2));
  fl_value_append_take(value2, fl_value_new_int(3));
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, ListToString) {
  g_autoptr(FlValue) value = fl_value_new_list();
  fl_value_append_take(value, fl_value_new_null());
  fl_value_append_take(value, fl_value_new_bool(TRUE));
  fl_value_append_take(value, fl_value_new_int(42));
  fl_value_append_take(value, fl_value_new_float(M_PI));
  fl_value_append_take(value, fl_value_new_uint8_list(nullptr, 0));
  fl_value_append_take(value, fl_value_new_int32_list(nullptr, 0));
  fl_value_append_take(value, fl_value_new_int64_list(nullptr, 0));
  fl_value_append_take(value, fl_value_new_float_list(nullptr, 0));
  fl_value_append_take(value, fl_value_new_list());
  fl_value_append_take(value, fl_value_new_map());
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text,
               "[null, true, 42, 3.1415926535897931, [], [], [], [], [], {}]");
}

TEST(FlValueTest, MapEmpty) {
  g_autoptr(FlValue) value = fl_value_new_map();
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(0));
}

TEST(FlValueTest, MapSet) {
  g_autoptr(FlValue) value = fl_value_new_map();
  g_autoptr(FlValue) k = fl_value_new_string("count");
  g_autoptr(FlValue) v = fl_value_new_int(42);
  fl_value_set(value, k, v);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(1));
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 0)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 0)), "count");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 0)),
            FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(fl_value_get_map_value(value, 0)), 42);
}

TEST(FlValueTest, MapSetTake) {
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_take(value, fl_value_new_string("count"), fl_value_new_int(42));
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(1));
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 0)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 0)), "count");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 0)),
            FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(fl_value_get_map_value(value, 0)), 42);
}

TEST(FlValueTest, MapSetString) {
  g_autoptr(FlValue) value = fl_value_new_map();
  g_autoptr(FlValue) v = fl_value_new_int(42);
  fl_value_set_string(value, "count", v);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(1));
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 0)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 0)), "count");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 0)),
            FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(fl_value_get_map_value(value, 0)), 42);
}

TEST(FlValueTest, MapSetStringTake) {
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_string_take(value, "count", fl_value_new_int(42));
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(1));
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_key(value, 0)),
            FL_VALUE_TYPE_STRING);
  EXPECT_STREQ(fl_value_get_string(fl_value_get_map_key(value, 0)), "count");
  ASSERT_EQ(fl_value_get_type(fl_value_get_map_value(value, 0)),
            FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(fl_value_get_map_value(value, 0)), 42);
}

TEST(FlValueTest, MapLookup) {
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_string_take(value, "one", fl_value_new_int(1));
  fl_value_set_string_take(value, "two", fl_value_new_int(2));
  fl_value_set_string_take(value, "three", fl_value_new_int(3));
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  g_autoptr(FlValue) two_key = fl_value_new_string("two");
  FlValue* v = fl_value_lookup(value, two_key);
  ASSERT_NE(v, nullptr);
  ASSERT_EQ(fl_value_get_type(v), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(v), 2);
  g_autoptr(FlValue) four_key = fl_value_new_string("four");
  v = fl_value_lookup(value, four_key);
  ASSERT_EQ(v, nullptr);
}

TEST(FlValueTest, MapLookupString) {
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_string_take(value, "one", fl_value_new_int(1));
  fl_value_set_string_take(value, "two", fl_value_new_int(2));
  fl_value_set_string_take(value, "three", fl_value_new_int(3));
  FlValue* v = fl_value_lookup_string(value, "two");
  ASSERT_NE(v, nullptr);
  ASSERT_EQ(fl_value_get_type(v), FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_int(v), 2);
  v = fl_value_lookup_string(value, "four");
  ASSERT_EQ(v, nullptr);
}

TEST(FlValueTest, MapValueypes) {
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_take(value, fl_value_new_string("null"), fl_value_new_null());
  fl_value_set_take(value, fl_value_new_string("bool"),
                    fl_value_new_bool(TRUE));
  fl_value_set_take(value, fl_value_new_string("int"), fl_value_new_int(42));
  fl_value_set_take(value, fl_value_new_string("float"),
                    fl_value_new_float(M_PI));
  fl_value_set_take(value, fl_value_new_string("uint8_list"),
                    fl_value_new_uint8_list(nullptr, 0));
  fl_value_set_take(value, fl_value_new_string("int32_list"),
                    fl_value_new_int32_list(nullptr, 0));
  fl_value_set_take(value, fl_value_new_string("int64_list"),
                    fl_value_new_int64_list(nullptr, 0));
  fl_value_set_take(value, fl_value_new_string("float_list"),
                    fl_value_new_float_list(nullptr, 0));
  fl_value_set_take(value, fl_value_new_string("list"), fl_value_new_list());
  fl_value_set_take(value, fl_value_new_string("map"), fl_value_new_map());
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(10));
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_value(value, 0)),
            FL_VALUE_TYPE_NULL);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_value(value, 1)),
            FL_VALUE_TYPE_BOOL);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_value(value, 2)),
            FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_value(value, 3)),
            FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_value(value, 4)),
            FL_VALUE_TYPE_UINT8_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_value(value, 5)),
            FL_VALUE_TYPE_INT32_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_value(value, 6)),
            FL_VALUE_TYPE_INT64_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_value(value, 7)),
            FL_VALUE_TYPE_FLOAT_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_value(value, 8)),
            FL_VALUE_TYPE_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_value(value, 9)),
            FL_VALUE_TYPE_MAP);
}

TEST(FlValueTest, MapKeyTypes) {
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_take(value, fl_value_new_null(), fl_value_new_string("null"));
  fl_value_set_take(value, fl_value_new_bool(TRUE),
                    fl_value_new_string("bool"));
  fl_value_set_take(value, fl_value_new_int(42), fl_value_new_string("int"));
  fl_value_set_take(value, fl_value_new_float(M_PI),
                    fl_value_new_string("float"));
  fl_value_set_take(value, fl_value_new_uint8_list(nullptr, 0),
                    fl_value_new_string("uint8_list"));
  fl_value_set_take(value, fl_value_new_int32_list(nullptr, 0),
                    fl_value_new_string("int32_list"));
  fl_value_set_take(value, fl_value_new_int64_list(nullptr, 0),
                    fl_value_new_string("int64_list"));
  fl_value_set_take(value, fl_value_new_float_list(nullptr, 0),
                    fl_value_new_string("float_list"));
  fl_value_set_take(value, fl_value_new_list(), fl_value_new_string("list"));
  fl_value_set_take(value, fl_value_new_map(), fl_value_new_string("map"));
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_MAP);
  ASSERT_EQ(fl_value_get_length(value), static_cast<size_t>(10));
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_key(value, 0)),
            FL_VALUE_TYPE_NULL);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_key(value, 1)),
            FL_VALUE_TYPE_BOOL);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_key(value, 2)),
            FL_VALUE_TYPE_INT);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_key(value, 3)),
            FL_VALUE_TYPE_FLOAT);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_key(value, 4)),
            FL_VALUE_TYPE_UINT8_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_key(value, 5)),
            FL_VALUE_TYPE_INT32_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_key(value, 6)),
            FL_VALUE_TYPE_INT64_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_key(value, 7)),
            FL_VALUE_TYPE_FLOAT_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_key(value, 8)),
            FL_VALUE_TYPE_LIST);
  EXPECT_EQ(fl_value_get_type(fl_value_get_map_key(value, 9)),
            FL_VALUE_TYPE_MAP);
}

TEST(FlValueTest, MapEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_map();
  fl_value_set_string_take(value1, "one", fl_value_new_int(1));
  fl_value_set_string_take(value1, "two", fl_value_new_int(2));
  fl_value_set_string_take(value1, "three", fl_value_new_int(3));
  g_autoptr(FlValue) value2 = fl_value_new_map();
  fl_value_set_string_take(value2, "one", fl_value_new_int(1));
  fl_value_set_string_take(value2, "two", fl_value_new_int(2));
  fl_value_set_string_take(value2, "three", fl_value_new_int(3));
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, MapEqualDifferentOrder) {
  g_autoptr(FlValue) value1 = fl_value_new_map();
  fl_value_set_string_take(value1, "one", fl_value_new_int(1));
  fl_value_set_string_take(value1, "two", fl_value_new_int(2));
  fl_value_set_string_take(value1, "three", fl_value_new_int(3));
  g_autoptr(FlValue) value2 = fl_value_new_map();
  fl_value_set_string_take(value2, "one", fl_value_new_int(1));
  fl_value_set_string_take(value2, "three", fl_value_new_int(3));
  fl_value_set_string_take(value2, "two", fl_value_new_int(2));
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, MapEmptyEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_map();
  g_autoptr(FlValue) value2 = fl_value_new_map();
  EXPECT_TRUE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, MapNotEqualSameSizeDifferentKeys) {
  g_autoptr(FlValue) value1 = fl_value_new_map();
  fl_value_set_string_take(value1, "one", fl_value_new_int(1));
  fl_value_set_string_take(value1, "two", fl_value_new_int(2));
  fl_value_set_string_take(value1, "three", fl_value_new_int(3));
  g_autoptr(FlValue) value2 = fl_value_new_map();
  fl_value_set_string_take(value2, "one", fl_value_new_int(1));
  fl_value_set_string_take(value2, "two", fl_value_new_int(2));
  fl_value_set_string_take(value2, "four", fl_value_new_int(3));
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, MapNotEqualSameSizeDifferentValues) {
  g_autoptr(FlValue) value1 = fl_value_new_map();
  fl_value_set_string_take(value1, "one", fl_value_new_int(1));
  fl_value_set_string_take(value1, "two", fl_value_new_int(2));
  fl_value_set_string_take(value1, "three", fl_value_new_int(3));
  g_autoptr(FlValue) value2 = fl_value_new_map();
  fl_value_set_string_take(value2, "one", fl_value_new_int(1));
  fl_value_set_string_take(value2, "two", fl_value_new_int(2));
  fl_value_set_string_take(value2, "three", fl_value_new_int(4));
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, MapNotEqualDifferentSize) {
  g_autoptr(FlValue) value1 = fl_value_new_map();
  fl_value_set_string_take(value1, "one", fl_value_new_int(1));
  fl_value_set_string_take(value1, "two", fl_value_new_int(2));
  fl_value_set_string_take(value1, "three", fl_value_new_int(3));
  g_autoptr(FlValue) value2 = fl_value_new_map();
  fl_value_set_string_take(value2, "one", fl_value_new_int(1));
  fl_value_set_string_take(value2, "two", fl_value_new_int(2));
  fl_value_set_string_take(value2, "three", fl_value_new_int(3));
  fl_value_set_string_take(value2, "four", fl_value_new_int(4));
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, MapEmptyNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_map();
  g_autoptr(FlValue) value2 = fl_value_new_map();
  fl_value_set_string_take(value2, "one", fl_value_new_int(1));
  fl_value_set_string_take(value2, "two", fl_value_new_int(2));
  fl_value_set_string_take(value2, "three", fl_value_new_int(3));
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, MapToString) {
  g_autoptr(FlValue) value = fl_value_new_map();
  fl_value_set_take(value, fl_value_new_string("null"), fl_value_new_null());
  fl_value_set_take(value, fl_value_new_string("bool"),
                    fl_value_new_bool(TRUE));
  fl_value_set_take(value, fl_value_new_string("int"), fl_value_new_int(42));
  fl_value_set_take(value, fl_value_new_string("float"),
                    fl_value_new_float(M_PI));
  fl_value_set_take(value, fl_value_new_string("uint8_list"),
                    fl_value_new_uint8_list(nullptr, 0));
  fl_value_set_take(value, fl_value_new_string("int32_list"),
                    fl_value_new_int32_list(nullptr, 0));
  fl_value_set_take(value, fl_value_new_string("int64_list"),
                    fl_value_new_int64_list(nullptr, 0));
  fl_value_set_take(value, fl_value_new_string("float_list"),
                    fl_value_new_float_list(nullptr, 0));
  fl_value_set_take(value, fl_value_new_string("list"), fl_value_new_list());
  fl_value_set_take(value, fl_value_new_string("map"), fl_value_new_map());
  fl_value_set_take(value, fl_value_new_null(), fl_value_new_string("null"));
  fl_value_set_take(value, fl_value_new_bool(TRUE),
                    fl_value_new_string("bool"));
  fl_value_set_take(value, fl_value_new_int(42), fl_value_new_string("int"));
  fl_value_set_take(value, fl_value_new_float(M_PI),
                    fl_value_new_string("float"));
  fl_value_set_take(value, fl_value_new_uint8_list(nullptr, 0),
                    fl_value_new_string("uint8_list"));
  fl_value_set_take(value, fl_value_new_int32_list(nullptr, 0),
                    fl_value_new_string("int32_list"));
  fl_value_set_take(value, fl_value_new_int64_list(nullptr, 0),
                    fl_value_new_string("int64_list"));
  fl_value_set_take(value, fl_value_new_float_list(nullptr, 0),
                    fl_value_new_string("float_list"));
  fl_value_set_take(value, fl_value_new_list(), fl_value_new_string("list"));
  fl_value_set_take(value, fl_value_new_map(), fl_value_new_string("map"));
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text,
               "{null: null, bool: true, int: 42, float: 3.1415926535897931, "
               "uint8_list: [], int32_list: [], int64_list: [], float_list: "
               "[], list: [], map: {}, null: null, true: bool, 42: int, "
               "3.1415926535897931: float, []: uint8_list, []: int32_list, []: "
               "int64_list, []: float_list, []: list, {}: map}");
}

TEST(FlDartProjectTest, Custom) {
  g_autoptr(FlValue) value =
      fl_value_new_custom(128, g_strdup("Hello World"), g_free);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_CUSTOM);
  ASSERT_EQ(fl_value_get_custom_type(value), 128);
  ASSERT_STREQ(reinterpret_cast<const gchar*>(fl_value_get_custom_value(value)),
               "Hello World");
}

TEST(FlDartProjectTest, CustomNoDestroyNotify) {
  g_autoptr(FlValue) value = fl_value_new_custom(128, "Hello World", nullptr);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_CUSTOM);
  ASSERT_EQ(fl_value_get_custom_type(value), 128);
  ASSERT_STREQ(reinterpret_cast<const gchar*>(fl_value_get_custom_value(value)),
               "Hello World");
}

TEST(FlDartProjectTest, CustomObject) {
  g_autoptr(GObject) v = G_OBJECT(g_object_new(G_TYPE_OBJECT, nullptr));
  g_autoptr(FlValue) value = fl_value_new_custom_object(128, v);
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_CUSTOM);
  ASSERT_EQ(fl_value_get_custom_type(value), 128);
  ASSERT_TRUE(G_IS_OBJECT(fl_value_get_custom_value_object(value)));
}

TEST(FlDartProjectTest, CustomObjectTake) {
  g_autoptr(FlValue) value = fl_value_new_custom_object_take(
      128, G_OBJECT(g_object_new(G_TYPE_OBJECT, nullptr)));
  ASSERT_EQ(fl_value_get_type(value), FL_VALUE_TYPE_CUSTOM);
  ASSERT_EQ(fl_value_get_custom_type(value), 128);
  ASSERT_TRUE(G_IS_OBJECT(fl_value_get_custom_value_object(value)));
}

TEST(FlValueTest, CustomEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_custom(128, "Hello World", nullptr);
  g_autoptr(FlValue) value2 = fl_value_new_custom(128, "Hello World", nullptr);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, CustomToString) {
  g_autoptr(FlValue) value = fl_value_new_custom(128, nullptr, nullptr);
  g_autofree gchar* text = fl_value_to_string(value);
  EXPECT_STREQ(text, "(custom 128)");
}

TEST(FlValueTest, EqualSameObject) {
  g_autoptr(FlValue) value = fl_value_new_null();
  EXPECT_TRUE(fl_value_equal(value, value));
}

TEST(FlValueTest, NullIntNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_null();
  g_autoptr(FlValue) value2 = fl_value_new_int(0);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, NullBoolNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_bool(FALSE);
  g_autoptr(FlValue) value2 = fl_value_new_int(0);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, StringUint8ListNotEqual) {
  uint8_t data[] = {'h', 'e', 'l', 'l', 'o'};
  g_autoptr(FlValue) value1 = fl_value_new_uint8_list(data, 5);
  g_autoptr(FlValue) value2 = fl_value_new_string("hello");
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Uint8ListInt32ListNotEqual) {
  uint8_t data8[] = {0, 1, 2, 3, 4};
  int32_t data32[] = {0, 1, 2, 3, 4};
  g_autoptr(FlValue) value1 = fl_value_new_uint8_list(data8, 5);
  g_autoptr(FlValue) value2 = fl_value_new_int32_list(data32, 5);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int32ListInt64ListNotEqual) {
  int32_t data32[] = {0, 1, 2, 3, 4};
  int64_t data64[] = {0, 1, 2, 3, 4};
  g_autoptr(FlValue) value1 = fl_value_new_int32_list(data32, 5);
  g_autoptr(FlValue) value2 = fl_value_new_int64_list(data64, 5);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, Int64ListFloatListNotEqual) {
  int64_t data64[] = {0, 1, 2, 3, 4};
  double dataf[] = {0.0, 1.0, 2.0, 3.0, 4.0};
  g_autoptr(FlValue) value1 = fl_value_new_int64_list(data64, 5);
  g_autoptr(FlValue) value2 = fl_value_new_float_list(dataf, 5);
  EXPECT_FALSE(fl_value_equal(value1, value2));
}

TEST(FlValueTest, ListMapNotEqual) {
  g_autoptr(FlValue) value1 = fl_value_new_list();
  g_autoptr(FlValue) value2 = fl_value_new_map();
  EXPECT_FALSE(fl_value_equal(value1, value2));
}
