// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/client_wrapper/include/flutter/encodable_value.h"

#include <limits>

#include "gtest/gtest.h"

namespace flutter {

TEST(EncodableValueTest, Null) {
  EncodableValue value;
  value.IsNull();
}

TEST(EncodableValueTest, Bool) {
  EncodableValue value(false);

  EXPECT_FALSE(std::get<bool>(value));
  value = true;
  EXPECT_TRUE(std::get<bool>(value));
}

TEST(EncodableValueTest, Int) {
  EncodableValue value(42);

  EXPECT_EQ(std::get<int32_t>(value), 42);
  value = std::numeric_limits<int32_t>::max();
  EXPECT_EQ(std::get<int32_t>(value), std::numeric_limits<int32_t>::max());
}

// Test the int/long convenience wrapper.
TEST(EncodableValueTest, LongValue) {
  const EncodableValue int_value(std::numeric_limits<int32_t>::max());
  EXPECT_EQ(int_value.LongValue(), std::numeric_limits<int32_t>::max());
  const EncodableValue long_value(std::numeric_limits<int64_t>::max());
  EXPECT_EQ(long_value.LongValue(), std::numeric_limits<int64_t>::max());
}

TEST(EncodableValueTest, Long) {
  EncodableValue value(INT64_C(42));

  EXPECT_EQ(std::get<int64_t>(value), 42);
  value = std::numeric_limits<int64_t>::max();
  EXPECT_EQ(std::get<int64_t>(value), std::numeric_limits<int64_t>::max());
}

TEST(EncodableValueTest, Double) {
  EncodableValue value(3.14);

  EXPECT_EQ(std::get<double>(value), 3.14);
  value = std::numeric_limits<double>::max();
  EXPECT_EQ(std::get<double>(value), std::numeric_limits<double>::max());
}

TEST(EncodableValueTest, String) {
  std::string hello("Hello, world!");
  EncodableValue value(hello);

  EXPECT_EQ(std::get<std::string>(value), hello);
  value = std::string("Goodbye");
  EXPECT_EQ(std::get<std::string>(value), "Goodbye");
}

// Explicitly verify that the overrides to prevent char*->bool conversions work.
TEST(EncodableValueTest, CString) {
  const char* hello = "Hello, world!";
  EncodableValue value(hello);

  EXPECT_EQ(std::get<std::string>(value), hello);
  value = "Goodbye";
  EXPECT_EQ(std::get<std::string>(value), "Goodbye");
}

TEST(EncodableValueTest, UInt8List) {
  std::vector<uint8_t> data = {0, 2};
  EncodableValue value(data);

  auto& list_value = std::get<std::vector<uint8_t>>(value);
  list_value.push_back(std::numeric_limits<uint8_t>::max());
  EXPECT_EQ(list_value[0], 0);
  EXPECT_EQ(list_value[1], 2);

  ASSERT_EQ(list_value.size(), 3u);
  EXPECT_EQ(data.size(), 2u);
  EXPECT_EQ(list_value[2], std::numeric_limits<uint8_t>::max());
}

TEST(EncodableValueTest, Int32List) {
  std::vector<int32_t> data = {-10, 2};
  EncodableValue value(data);

  auto& list_value = std::get<std::vector<int32_t>>(value);
  list_value.push_back(std::numeric_limits<int32_t>::max());
  EXPECT_EQ(list_value[0], -10);
  EXPECT_EQ(list_value[1], 2);

  ASSERT_EQ(list_value.size(), 3u);
  EXPECT_EQ(data.size(), 2u);
  EXPECT_EQ(list_value[2], std::numeric_limits<int32_t>::max());
}

TEST(EncodableValueTest, Int64List) {
  std::vector<int64_t> data = {-10, 2};
  EncodableValue value(data);

  auto& list_value = std::get<std::vector<int64_t>>(value);
  list_value.push_back(std::numeric_limits<int64_t>::max());
  EXPECT_EQ(list_value[0], -10);
  EXPECT_EQ(list_value[1], 2);

  ASSERT_EQ(list_value.size(), 3u);
  EXPECT_EQ(data.size(), 2u);
  EXPECT_EQ(list_value[2], std::numeric_limits<int64_t>::max());
}

TEST(EncodableValueTest, DoubleList) {
  std::vector<double> data = {-10.0, 2.0};
  EncodableValue value(data);

  auto& list_value = std::get<std::vector<double>>(value);
  list_value.push_back(std::numeric_limits<double>::max());
  EXPECT_EQ(list_value[0], -10.0);
  EXPECT_EQ(list_value[1], 2.0);

  ASSERT_EQ(list_value.size(), 3u);
  EXPECT_EQ(data.size(), 2u);
  EXPECT_EQ(list_value[2], std::numeric_limits<double>::max());
}

TEST(EncodableValueTest, List) {
  EncodableList encodables = {
      EncodableValue(1),
      EncodableValue(2.0),
      EncodableValue("Three"),
  };
  EncodableValue value(encodables);

  auto& list_value = std::get<EncodableList>(value);
  EXPECT_EQ(std::get<int32_t>(list_value[0]), 1);
  EXPECT_EQ(std::get<double>(list_value[1]), 2.0);
  EXPECT_EQ(std::get<std::string>(list_value[2]), "Three");

  // Ensure that it's a modifiable copy of the original array.
  list_value.push_back(EncodableValue(true));
  ASSERT_EQ(list_value.size(), 4u);
  EXPECT_EQ(encodables.size(), 3u);
  EXPECT_EQ(std::get<bool>(std::get<EncodableList>(value)[3]), true);
}

TEST(EncodableValueTest, Map) {
  EncodableMap encodables = {
      {EncodableValue(), EncodableValue(std::vector<int32_t>{1, 2, 3})},
      {EncodableValue(1), EncodableValue(INT64_C(10000))},
      {EncodableValue("two"), EncodableValue(7)},
  };
  EncodableValue value(encodables);

  auto& map_value = std::get<EncodableMap>(value);
  EXPECT_EQ(
      std::holds_alternative<std::vector<int32_t>>(map_value[EncodableValue()]),
      true);
  EXPECT_EQ(std::get<int64_t>(map_value[EncodableValue(1)]), INT64_C(10000));
  EXPECT_EQ(std::get<int32_t>(map_value[EncodableValue("two")]), 7);

  // Ensure that it's a modifiable copy of the original map.
  map_value[EncodableValue(true)] = EncodableValue(false);
  ASSERT_EQ(map_value.size(), 4u);
  EXPECT_EQ(encodables.size(), 3u);
  EXPECT_EQ(std::get<bool>(map_value[EncodableValue(true)]), false);
}

// Tests that the < operator meets the requirements of using EncodableValue as
// a map key.
TEST(EncodableValueTest, Comparison) {
  EncodableList values = {
      // Null
      EncodableValue(),
      // Bool
      EncodableValue(true),
      EncodableValue(false),
      // Int
      EncodableValue(-7),
      EncodableValue(0),
      EncodableValue(100),
      // Long
      EncodableValue(INT64_C(-7)),
      EncodableValue(INT64_C(0)),
      EncodableValue(INT64_C(100)),
      // Double
      EncodableValue(-7.0),
      EncodableValue(0.0),
      EncodableValue(100.0),
      // String
      EncodableValue("one"),
      EncodableValue("two"),
      // ByteList
      EncodableValue(std::vector<uint8_t>{0, 1}),
      EncodableValue(std::vector<uint8_t>{0, 10}),
      // IntList
      EncodableValue(std::vector<int32_t>{0, 1}),
      EncodableValue(std::vector<int32_t>{0, 100}),
      // LongList
      EncodableValue(std::vector<int64_t>{0, INT64_C(1)}),
      EncodableValue(std::vector<int64_t>{0, INT64_C(100)}),
      // DoubleList
      EncodableValue(std::vector<double>{0, INT64_C(1)}),
      EncodableValue(std::vector<double>{0, INT64_C(100)}),
      // List
      EncodableValue(EncodableList{EncodableValue(), EncodableValue(true)}),
      EncodableValue(EncodableList{EncodableValue(), EncodableValue(1.0)}),
      // Map
      EncodableValue(EncodableMap{{EncodableValue(), EncodableValue(true)},
                                  {EncodableValue(7), EncodableValue(7.0)}}),
      EncodableValue(
          EncodableMap{{EncodableValue(), EncodableValue(1.0)},
                       {EncodableValue("key"), EncodableValue("value")}}),
      // FloatList
      EncodableValue(std::vector<float>{0, 1}),
      EncodableValue(std::vector<float>{0, 100}),
  };

  for (size_t i = 0; i < values.size(); ++i) {
    const auto& a = values[i];
    for (size_t j = 0; j < values.size(); ++j) {
      const auto& b = values[j];
      if (i == j) {
        // Identical objects should always be equal.
        EXPECT_FALSE(a < b);
        EXPECT_FALSE(b < a);
      } else {
        // All other comparisons should be consistent, but the direction doesn't
        // matter.
        EXPECT_NE(a < b, b < a) << "Indexes: " << i << ", " << j;
      }
    }

    // Copies should always be equal.
    EncodableValue copy(a);
    EXPECT_FALSE(a < copy || copy < a);
  }
}

// Tests that structures are deep-copied.
TEST(EncodableValueTest, DeepCopy) {
  EncodableList original = {
      EncodableValue(EncodableMap{
          {EncodableValue(), EncodableValue(std::vector<int32_t>{1, 2, 3})},
          {EncodableValue(1), EncodableValue(INT64_C(0000))},
          {EncodableValue("two"), EncodableValue(7)},
      }),
      EncodableValue(EncodableList{
          EncodableValue(),
          EncodableValue(),
          EncodableValue(
              EncodableMap{{EncodableValue("a"), EncodableValue("b")}}),
      }),
  };

  EncodableValue copy(original);
  ASSERT_TRUE(std::holds_alternative<EncodableList>(copy));

  // Spot-check innermost collection values.
  auto& root_list = std::get<EncodableList>(copy);
  auto& first_child = std::get<EncodableMap>(root_list[0]);
  EXPECT_EQ(std::get<int32_t>(first_child[EncodableValue("two")]), 7);
  auto& second_child = std::get<EncodableList>(root_list[1]);
  auto& innermost_map = std::get<EncodableMap>(second_child[2]);
  EXPECT_EQ(std::get<std::string>(innermost_map[EncodableValue("a")]), "b");

  // Modify those values in the original structure.
  first_child[EncodableValue("two")] = EncodableValue();
  innermost_map[EncodableValue("a")] = 99;

  // Re-check innermost collection values of the original to ensure that they
  // haven't changed.
  first_child = std::get<EncodableMap>(original[0]);
  EXPECT_EQ(std::get<int32_t>(first_child[EncodableValue("two")]), 7);
  second_child = std::get<EncodableList>(original[1]);
  innermost_map = std::get<EncodableMap>(second_child[2]);
  EXPECT_EQ(std::get<std::string>(innermost_map[EncodableValue("a")]), "b");
}

// Simple class for testing custom encodable values
class TestCustomValue {
 public:
  TestCustomValue() : x_(0), y_(0) {}
  TestCustomValue(int x, int y) : x_(x), y_(y) {}
  ~TestCustomValue() = default;

  int x() const { return x_; }
  int y() const { return y_; }

 private:
  int x_;
  int y_;
};

TEST(EncodableValueTest, TypeIndexesCorrect) {
  // Null
  EXPECT_EQ(EncodableValue().index(), 0u);
  // Bool
  EXPECT_EQ(EncodableValue(true).index(), 1u);
  // Int32
  EXPECT_EQ(EncodableValue(100).index(), 2u);
  // Int64
  EXPECT_EQ(EncodableValue(INT64_C(100)).index(), 3u);
  // Double
  EXPECT_EQ(EncodableValue(7.0).index(), 4u);
  // String
  EXPECT_EQ(EncodableValue("one").index(), 5u);
  // ByteList
  EXPECT_EQ(EncodableValue(std::vector<uint8_t>()).index(), 6u);
  // IntList
  EXPECT_EQ(EncodableValue(std::vector<int32_t>()).index(), 7u);
  // LongList
  EXPECT_EQ(EncodableValue(std::vector<int64_t>()).index(), 8u);
  // DoubleList
  EXPECT_EQ(EncodableValue(std::vector<double>()).index(), 9u);
  // List
  EXPECT_EQ(EncodableValue(EncodableList()).index(), 10u);
  // Map
  EXPECT_EQ(EncodableValue(EncodableMap()).index(), 11u);
  // Custom type
  TestCustomValue customValue;
  EXPECT_EQ(((EncodableValue)CustomEncodableValue(customValue)).index(), 12u);
  // FloatList
  EXPECT_EQ(EncodableValue(std::vector<float>()).index(), 13u);
}  // namespace flutter

}  // namespace flutter
