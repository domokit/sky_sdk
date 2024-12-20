// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/windows/testing/windows_test_context.h"

#include "flutter/fml/platform/win/wstring_conversion.h"

namespace flutter {
namespace testing {

WindowsTestContext::WindowsTestContext(std::string_view assets_path)
    : assets_path_(fml::Utf8ToWideString(assets_path)),
      native_resolver_(std::make_shared<TestDartNativeResolver>()) {
  isolate_create_callbacks_.push_back(
      [weak_resolver =
           std::weak_ptr<TestDartNativeResolver>{native_resolver_}]() {
        if (auto resolver = weak_resolver.lock()) {
          resolver->SetNativeResolverForIsolate();
        }
      });
}

WindowsTestContext::~WindowsTestContext() = default;

const std::wstring& WindowsTestContext::GetAssetsPath() const {
  return assets_path_;
}

const std::wstring& WindowsTestContext::GetIcuDataPath() const {
  return icu_data_path_;
}

const std::wstring& WindowsTestContext::GetAotLibraryPath() const {
  return aot_library_path_;
}

void WindowsTestContext::AddNativeFunction(std::string_view name,
                                           Dart_NativeFunction function) {
  native_resolver_->AddNativeCallback(std::string{name}, function);
}

fml::closure WindowsTestContext::GetRootIsolateCallback() {
  return [this]() {
    for (auto closure : this->isolate_create_callbacks_) {
      closure();
    }
  };
}

}  // namespace testing
}  // namespace flutter
