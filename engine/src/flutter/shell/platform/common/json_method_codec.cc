// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/common/json_method_codec.h"

#include "flutter/shell/platform/common/json_message_codec.h"

namespace flutter {

namespace {

// Keys used in MethodCall encoding.
constexpr char kMessageMethodKey[] = "method";
constexpr char kMessageArgumentsKey[] = "args";

// Returns a new document containing only |element|, which must be an element
// in |document|. This is a move rather than a copy, so it is efficient but
// destructive to the data in |document|.
std::unique_ptr<rapidjson::Document> ExtractElement(
    rapidjson::Document* document,
    rapidjson::Value* subtree) {
  auto extracted = std::make_unique<rapidjson::Document>();
  // Pull the subtree up to the root of the document.
  document->Swap(*subtree);
  // Swap the entire document into |extracted|. Unlike the swap above this moves
  // the allocator ownership, so the data won't be deleted when |document| is
  // destroyed.
  extracted->Swap(*document);
  return extracted;
}

}  // namespace

// static
const JsonMethodCodec& JsonMethodCodec::GetInstance() {
  static JsonMethodCodec sInstance;
  return sInstance;
}

std::unique_ptr<MethodCall<rapidjson::Document>>
JsonMethodCodec::DecodeMethodCallInternal(const uint8_t* message,
                                          size_t message_size) const {
  std::unique_ptr<rapidjson::Document> json_message =
      JsonMessageCodec::GetInstance().DecodeMessage(message, message_size);
  if (!json_message) {
    return nullptr;
  }

  auto method_name_iter = json_message->FindMember(kMessageMethodKey);
  if (method_name_iter == json_message->MemberEnd()) {
    return nullptr;
  }
  if (!method_name_iter->value.IsString()) {
    return nullptr;
  }
  std::string method_name(method_name_iter->value.GetString());
  auto arguments_iter = json_message->FindMember(kMessageArgumentsKey);
  std::unique_ptr<rapidjson::Document> arguments;
  if (arguments_iter != json_message->MemberEnd()) {
    arguments = ExtractElement(json_message.get(), &(arguments_iter->value));
  }
  return std::make_unique<MethodCall<rapidjson::Document>>(
      method_name, std::move(arguments));
}

std::unique_ptr<std::vector<uint8_t>> JsonMethodCodec::EncodeMethodCallInternal(
    const MethodCall<rapidjson::Document>& method_call) const {
  // TODO(stuartmorgan): Consider revisiting the codec APIs to avoid the need
  // to copy everything when doing encoding (e.g., by having a version that
  // takes owership of the object to encode, so that it can be moved instead).
  rapidjson::Document message(rapidjson::kObjectType);
  auto& allocator = message.GetAllocator();
  rapidjson::Value name(method_call.method_name(), allocator);
  rapidjson::Value arguments;
  if (method_call.arguments()) {
    arguments.CopyFrom(*method_call.arguments(), allocator);
  }
  message.AddMember(kMessageMethodKey, name, allocator);
  message.AddMember(kMessageArgumentsKey, arguments, allocator);

  return JsonMessageCodec::GetInstance().EncodeMessage(message);
}

std::unique_ptr<std::vector<uint8_t>>
JsonMethodCodec::EncodeSuccessEnvelopeInternal(
    const rapidjson::Document* result) const {
  rapidjson::Document envelope;
  envelope.SetArray();
  rapidjson::Value result_value;
  if (result) {
    result_value.CopyFrom(*result, envelope.GetAllocator());
  }
  envelope.PushBack(result_value, envelope.GetAllocator());

  return JsonMessageCodec::GetInstance().EncodeMessage(envelope);
}

std::unique_ptr<std::vector<uint8_t>>
JsonMethodCodec::EncodeErrorEnvelopeInternal(
    const std::string& error_code,
    const std::string& error_message,
    const rapidjson::Document* error_details) const {
  // NOLINTNEXTLINE(clang-analyzer-core.NullDereference)
  rapidjson::Document envelope(rapidjson::kArrayType);
  auto& allocator = envelope.GetAllocator();
  envelope.PushBack(rapidjson::Value(error_code, allocator), allocator);
  envelope.PushBack(rapidjson::Value(error_message, allocator), allocator);
  rapidjson::Value details_value;
  if (error_details) {
    details_value.CopyFrom(*error_details, allocator);
  }
  envelope.PushBack(details_value, allocator);

  return JsonMessageCodec::GetInstance().EncodeMessage(envelope);
}

bool JsonMethodCodec::DecodeAndProcessResponseEnvelopeInternal(
    const uint8_t* response,
    size_t response_size,
    MethodResult<rapidjson::Document>* result) const {
  std::unique_ptr<rapidjson::Document> json_response =
      JsonMessageCodec::GetInstance().DecodeMessage(response, response_size);
  if (!json_response) {
    return false;
  }
  if (!json_response->IsArray()) {
    return false;
  }
  switch (json_response->Size()) {
    case 1: {
      std::unique_ptr<rapidjson::Document> value =
          ExtractElement(json_response.get(), &((*json_response)[0]));
      if (value->IsNull()) {
        result->Success();
      } else {
        result->Success(*value);
      }
      return true;
    }
    case 3: {
      std::string code = (*json_response)[0].GetString();
      std::string message = (*json_response)[1].GetString();
      std::unique_ptr<rapidjson::Document> details =
          ExtractElement(json_response.get(), &((*json_response)[2]));
      if (details->IsNull()) {
        result->Error(code, message);
      } else {
        result->Error(code, message, *details);
      }
      return true;
    }
    default:
      return false;
  }
}

}  // namespace flutter
