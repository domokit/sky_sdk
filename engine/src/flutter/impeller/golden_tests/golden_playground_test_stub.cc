// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/golden_tests/golden_playground_test.h"

namespace impeller {

GoldenPlaygroundTest::GoldenPlaygroundTest() = default;

GoldenPlaygroundTest::~GoldenPlaygroundTest() = default;

void GoldenPlaygroundTest::SetTypographerContext(
    std::shared_ptr<TypographerContext> typographer_context) {
  typographer_context_ = std::move(typographer_context);
};

void GoldenPlaygroundTest::TearDown() {}

void GoldenPlaygroundTest::SetUp() {
  GTEST_SKIP() << "GoldenPlaygroundTest doesn't support this backend type.";
}

PlaygroundBackend GoldenPlaygroundTest::GetBackend() const {
  return GetParam();
}

bool GoldenPlaygroundTest::OpenPlaygroundHere(
    const AiksDlPlaygroundCallback& callback) {
  return false;
}

bool GoldenPlaygroundTest::OpenPlaygroundHere(
    const sk_sp<flutter::DisplayList>& list) {
  return false;
}

std::shared_ptr<Texture> GoldenPlaygroundTest::CreateTextureForFixture(
    const char* fixture_name,
    bool enable_mipmapping) const {
  return nullptr;
}

sk_sp<flutter::DlImage> GoldenPlaygroundTest::CreateDlImageForFixture(
    const char* fixture_name,
    bool enable_mipmapping) const {
  return nullptr;
}

RuntimeStage::Map GoldenPlaygroundTest::OpenAssetAsRuntimeStage(
    const char* asset_name) const {
  return {};
}

std::shared_ptr<Context> GoldenPlaygroundTest::GetContext() const {
  return nullptr;
}

Point GoldenPlaygroundTest::GetContentScale() const {
  return Point();
}

Scalar GoldenPlaygroundTest::GetSecondsElapsed() const {
  return Scalar();
}

ISize GoldenPlaygroundTest::GetWindowSize() const {
  return ISize();
}

void GoldenPlaygroundTest::SetWindowSize(ISize size) {}

fml::Status GoldenPlaygroundTest::SetCapabilities(
    const std::shared_ptr<Capabilities>& capabilities) {
  return fml::Status(
      fml::StatusCode::kUnimplemented,
      "GoldenPlaygroundTest-Stub doesn't support SetCapabilities.");
}

std::unique_ptr<testing::Screenshot> GoldenPlaygroundTest::MakeScreenshot(
    const sk_sp<flutter::DisplayList>& list) {
  return nullptr;
}

}  // namespace impeller
