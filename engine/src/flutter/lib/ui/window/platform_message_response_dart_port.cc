// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/window/platform_message_response_dart_port.h"

#include <array>
#include <utility>

#include "flutter/common/task_runners.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/trace_event.h"
#include "third_party/dart/runtime/include/dart_native_api.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_state.h"
#include "third_party/tonic/logging/dart_invoke.h"
#include "third_party/tonic/typed_data/dart_byte_data.h"

namespace flutter {

PlatformMessageResponseDartPort::PlatformMessageResponseDartPort(
    Dart_Port send_port,
    int64_t identifier,
    const std::string& channel)
    : send_port_(send_port), identifier_(identifier), channel_(channel) {
  FML_DCHECK(send_port != ILLEGAL_PORT);
}

void PlatformMessageResponseDartPort::Complete(
    std::unique_ptr<fml::Mapping> data) {
  is_complete_ = true;
  Dart_CObject response_identifier = {
      .type = Dart_CObject_kInt64,
  };
  response_identifier.value.as_int64 = identifier_;
  Dart_CObject response_data = {
      .type = Dart_CObject_kTypedData,
  };
  response_data.value.as_typed_data.type = Dart_TypedData_kUint8;
  response_data.value.as_typed_data.length = data->GetSize();
  response_data.value.as_typed_data.values = data->GetMapping();

  std::array<Dart_CObject*, 2> response_values = {&response_identifier,
                                                  &response_data};

  Dart_CObject response = {
      .type = Dart_CObject_kArray,
  };
  response.value.as_array.length = response_values.size();
  response.value.as_array.values = response_values.data();

  bool did_send = Dart_PostCObject(send_port_, &response);
  FML_CHECK(did_send);
}

void PlatformMessageResponseDartPort::CompleteEmpty() {
  is_complete_ = true;
  Dart_CObject response = {
      .type = Dart_CObject_kNull,
  };
  bool did_send = Dart_PostCObject(send_port_, &response);
  FML_CHECK(did_send);
}

}  // namespace flutter
