// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/json_message_codec.h"

#include <limits>
#include <map>
#include <vector>

#include "gtest/gtest.h"

namespace flutter {

namespace {

// Validates round-trip encoding and decoding of |value|.
static void CheckEncodeDecode(const rapidjson::Document& value) {
  const JsonMessageCodec& codec = JsonMessageCodec::GetInstance();
  auto encoded = codec.EncodeMessage(value);
  ASSERT_TRUE(encoded);
  auto decoded = codec.DecodeMessage(*encoded);
  EXPECT_EQ(value, *decoded);
}

}  // namespace

// Tests that a JSON document with various data types round-trips correctly.
TEST(JsonMessageCodec, EncodeDecode) {
  // NOLINTNEXTLINE(clang-analyzer-core.NullDereference)
  rapidjson::Document array(rapidjson::kArrayType);
  auto& allocator = array.GetAllocator();

  array.PushBack("string", allocator);

  rapidjson::Value map(rapidjson::kObjectType);
  map.AddMember("a", -7, allocator);
  map.AddMember("b", std::numeric_limits<int>::max(), allocator);
  map.AddMember("c", 3.14159, allocator);
  map.AddMember("d", true, allocator);
  map.AddMember("e", rapidjson::Value(), allocator);
  array.PushBack(map, allocator);

  CheckEncodeDecode(array);
}

}  // namespace flutter
