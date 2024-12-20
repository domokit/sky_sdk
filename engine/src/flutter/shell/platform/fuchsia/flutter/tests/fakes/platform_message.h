// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_PLATFORM_MESSAGE_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_PLATFORM_MESSAGE_H_

#include <gtest/gtest.h>
#include <optional>

#include "flutter/lib/ui/window/platform_message.h"
#include "third_party/rapidjson/include/rapidjson/document.h"

using PlatformMessageResponse = flutter::PlatformMessageResponse;
using PlatformMessage = flutter::PlatformMessage;

namespace flutter_runner::testing {

class FakePlatformMessageResponse : public PlatformMessageResponse {
 public:
  static fml::RefPtr<FakePlatformMessageResponse> Create() {
    return fml::AdoptRef(new FakePlatformMessageResponse());
  }

  void ExpectCompleted(std::string expected) {
    EXPECT_TRUE(is_complete_);
    if (is_complete_) {
      EXPECT_EQ(expected, response_);
    }
  }

  bool IsCompleted() { return is_complete_; }

  std::unique_ptr<PlatformMessage> WithMessage(std::string channel,
                                               std::string message) {
    return std::make_unique<PlatformMessage>(
        channel,
        fml::MallocMapping::Copy(message.c_str(),
                                 message.c_str() + message.size()),
        fml::RefPtr<FakePlatformMessageResponse>(this));
  }

  void Complete(std::unique_ptr<fml::Mapping> data) override {
    response_ =
        std::string(data->GetMapping(), data->GetMapping() + data->GetSize());
    FinalizeComplete();
  };

  void CompleteEmpty() override { FinalizeComplete(); };

 private:
  // Private constructors.
  FakePlatformMessageResponse() {}

  void FinalizeComplete() {
    EXPECT_FALSE(std::exchange(is_complete_, true))
        << "Platform message responses can only be completed once!";
  }

  std::string response_;
};

}  // namespace flutter_runner::testing

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_FLUTTER_TESTS_FAKES_PLATFORM_MESSAGE_H_
