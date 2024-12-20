// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/json_method_codec.h"

#include "flutter/shell/platform/common/client_wrapper/include/flutter/method_result_functions.h"
#include "gtest/gtest.h"

namespace flutter {

namespace {

// Returns true if the given method calls have the same method name, and their
// arguments have equivalent values.
bool MethodCallsAreEqual(const MethodCall<rapidjson::Document>& a,
                         const MethodCall<rapidjson::Document>& b) {
  if (a.method_name() != b.method_name()) {
    return false;
  }
  // Treat nullptr and Null as equivalent.
  if ((!a.arguments() || a.arguments()->IsNull()) &&
      (!b.arguments() || b.arguments()->IsNull())) {
    return true;
  }
  return *a.arguments() == *b.arguments();
}

}  // namespace

TEST(JsonMethodCodec, HandlesMethodCallsWithNullArguments) {
  const JsonMethodCodec& codec = JsonMethodCodec::GetInstance();
  MethodCall<rapidjson::Document> call("hello", nullptr);
  auto encoded = codec.EncodeMethodCall(call);
  ASSERT_TRUE(encoded);
  std::unique_ptr<MethodCall<rapidjson::Document>> decoded =
      codec.DecodeMethodCall(*encoded);
  ASSERT_TRUE(decoded);
  EXPECT_TRUE(MethodCallsAreEqual(call, *decoded));
}

TEST(JsonMethodCodec, HandlesMethodCallsWithArgument) {
  const JsonMethodCodec& codec = JsonMethodCodec::GetInstance();

  auto arguments = std::make_unique<rapidjson::Document>(rapidjson::kArrayType);
  auto& allocator = arguments->GetAllocator();
  arguments->PushBack(42, allocator);
  arguments->PushBack("world", allocator);
  MethodCall<rapidjson::Document> call("hello", std::move(arguments));
  auto encoded = codec.EncodeMethodCall(call);
  ASSERT_TRUE(encoded);
  std::unique_ptr<MethodCall<rapidjson::Document>> decoded =
      codec.DecodeMethodCall(*encoded);
  ASSERT_TRUE(decoded);
  EXPECT_TRUE(MethodCallsAreEqual(call, *decoded));
}

TEST(JsonMethodCodec, HandlesSuccessEnvelopesWithNullResult) {
  const JsonMethodCodec& codec = JsonMethodCodec::GetInstance();
  auto encoded = codec.EncodeSuccessEnvelope();
  ASSERT_TRUE(encoded);
  std::vector<uint8_t> bytes = {'[', 'n', 'u', 'l', 'l', ']'};
  EXPECT_EQ(*encoded, bytes);

  bool decoded_successfully = false;
  MethodResultFunctions<rapidjson::Document> result_handler(
      [&decoded_successfully](const rapidjson::Document* result) {
        decoded_successfully = true;
        EXPECT_EQ(result, nullptr);
      },
      nullptr, nullptr);
  codec.DecodeAndProcessResponseEnvelope(encoded->data(), encoded->size(),
                                         &result_handler);
  EXPECT_TRUE(decoded_successfully);
}

TEST(JsonMethodCodec, HandlesSuccessEnvelopesWithResult) {
  const JsonMethodCodec& codec = JsonMethodCodec::GetInstance();
  rapidjson::Document result;
  result.SetInt(42);
  auto encoded = codec.EncodeSuccessEnvelope(&result);
  ASSERT_TRUE(encoded);
  std::vector<uint8_t> bytes = {'[', '4', '2', ']'};
  EXPECT_EQ(*encoded, bytes);

  bool decoded_successfully = false;
  MethodResultFunctions<rapidjson::Document> result_handler(
      [&decoded_successfully](const rapidjson::Document* result) {
        decoded_successfully = true;
        EXPECT_EQ(result->GetInt(), 42);
      },
      nullptr, nullptr);
  codec.DecodeAndProcessResponseEnvelope(encoded->data(), encoded->size(),
                                         &result_handler);
  EXPECT_TRUE(decoded_successfully);
}

TEST(JsonMethodCodec, HandlesErrorEnvelopesWithNulls) {
  const JsonMethodCodec& codec = JsonMethodCodec::GetInstance();
  auto encoded = codec.EncodeErrorEnvelope("errorCode");
  ASSERT_TRUE(encoded);
  std::vector<uint8_t> bytes = {
      '[', '"', 'e', 'r', 'r', 'o', 'r', 'C', 'o', 'd', 'e',
      '"', ',', '"', '"', ',', 'n', 'u', 'l', 'l', ']',
  };
  EXPECT_EQ(*encoded, bytes);

  bool decoded_successfully = false;
  MethodResultFunctions<rapidjson::Document> result_handler(
      nullptr,
      [&decoded_successfully](const std::string& code,
                              const std::string& message,
                              const rapidjson::Document* details) {
        decoded_successfully = true;
        EXPECT_EQ(code, "errorCode");
        EXPECT_EQ(message, "");
        EXPECT_EQ(details, nullptr);
      },
      nullptr);
  codec.DecodeAndProcessResponseEnvelope(encoded->data(), encoded->size(),
                                         &result_handler);
  EXPECT_TRUE(decoded_successfully);
}

TEST(JsonMethodCodec, HandlesErrorEnvelopesWithDetails) {
  const JsonMethodCodec& codec = JsonMethodCodec::GetInstance();
  // NOLINTNEXTLINE(clang-analyzer-core.NullDereference)
  rapidjson::Document details(rapidjson::kArrayType);
  auto& allocator = details.GetAllocator();
  details.PushBack("a", allocator);
  details.PushBack(42, allocator);
  auto encoded =
      codec.EncodeErrorEnvelope("errorCode", "something failed", &details);
  ASSERT_NE(encoded.get(), nullptr);
  std::vector<uint8_t> bytes = {
      '[', '"', 'e', 'r', 'r', 'o', 'r', 'C', 'o', 'd', 'e', '"', ',', '"',
      's', 'o', 'm', 'e', 't', 'h', 'i', 'n', 'g', ' ', 'f', 'a', 'i', 'l',
      'e', 'd', '"', ',', '[', '"', 'a', '"', ',', '4', '2', ']', ']',
  };
  EXPECT_EQ(*encoded, bytes);

  bool decoded_successfully = false;
  MethodResultFunctions<rapidjson::Document> result_handler(
      nullptr,
      [&decoded_successfully](const std::string& code,
                              const std::string& message,
                              const rapidjson::Document* details) {
        decoded_successfully = true;
        EXPECT_EQ(code, "errorCode");
        EXPECT_EQ(message, "something failed");
        EXPECT_TRUE(details->IsArray());
        EXPECT_EQ(std::string((*details)[0].GetString()), "a");
        EXPECT_EQ((*details)[1].GetInt(), 42);
      },
      nullptr);
  codec.DecodeAndProcessResponseEnvelope(encoded->data(), encoded->size(),
                                         &result_handler);
  EXPECT_TRUE(decoded_successfully);
}

}  // namespace flutter
