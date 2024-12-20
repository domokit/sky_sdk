// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/android/android_context_gl_impeller.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

using ::testing::_;
using ::testing::AllOf;
using ::testing::ByMove;
using ::testing::Field;
using ::testing::Matcher;
using ::testing::Return;

using ::impeller::egl::Config;
using ::impeller::egl::ConfigDescriptor;

namespace {
class MockDisplay : public impeller::egl::Display {
 public:
  MOCK_METHOD(bool, IsValid, (), (const, override));
  MOCK_METHOD(std::unique_ptr<Config>,
              ChooseConfig,
              (ConfigDescriptor),
              (const, override));
};

bool GetEGLConfigForSurface(EGLint surface_bit, EGLConfig* result) {
  EGLint attributes[] = {
      // clang-format off
      EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
      EGL_SURFACE_TYPE,    surface_bit,
      EGL_RED_SIZE,        8,
      EGL_GREEN_SIZE,      8,
      EGL_BLUE_SIZE,       8,
      EGL_ALPHA_SIZE,      8,
      EGL_DEPTH_SIZE,      24,
      EGL_STENCIL_SIZE,    8,
      EGL_NONE,
      // clang-format on
  };
  EGLint config_count = 0;
  return eglChooseConfig(eglGetDisplay(EGL_DEFAULT_DISPLAY), attributes, result,
                         1, &config_count);
}

}  // namespace

class AndroidContextGLImpellerTest : public ::testing::Test {
 public:
  AndroidContextGLImpellerTest() {}

  void SetUp() override {
    EGLDisplay egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    ASSERT_TRUE(eglInitialize(egl_display, nullptr, nullptr));
  }
};

TEST_F(AndroidContextGLImpellerTest, MSAAFirstAttempt) {
  EGLConfig window_egl_config, pbuffer_egl_config;
  ASSERT_TRUE(GetEGLConfigForSurface(EGL_WINDOW_BIT, &window_egl_config));
  ASSERT_TRUE(GetEGLConfigForSurface(EGL_PBUFFER_BIT, &pbuffer_egl_config));

  auto display = std::make_unique<MockDisplay>();
  EXPECT_CALL(*display, IsValid).WillRepeatedly(Return(true));
  auto first_result =
      std::make_unique<Config>(ConfigDescriptor(), window_egl_config);
  auto second_result =
      std::make_unique<Config>(ConfigDescriptor(), pbuffer_egl_config);
  EXPECT_CALL(
      *display,
      ChooseConfig(Matcher<ConfigDescriptor>(AllOf(
          Field(&ConfigDescriptor::samples, impeller::egl::Samples::kFour),
          Field(&ConfigDescriptor::surface_type,
                impeller::egl::SurfaceType::kWindow)))))
      .WillOnce(Return(ByMove(std::move(first_result))));
  EXPECT_CALL(*display, ChooseConfig(Matcher<ConfigDescriptor>(
                            Field(&ConfigDescriptor::surface_type,
                                  impeller::egl::SurfaceType::kPBuffer))))
      .WillOnce(Return(ByMove(std::move(second_result))));
  ON_CALL(*display, ChooseConfig(_))
      .WillByDefault(Return(ByMove(std::unique_ptr<Config>())));
  auto context =
      std::make_unique<AndroidContextGLImpeller>(std::move(display), true);
  ASSERT_TRUE(context);
}

TEST_F(AndroidContextGLImpellerTest, FallbackForEmulator) {
  EGLConfig window_egl_config, pbuffer_egl_config;
  ASSERT_TRUE(GetEGLConfigForSurface(EGL_WINDOW_BIT, &window_egl_config));
  ASSERT_TRUE(GetEGLConfigForSurface(EGL_PBUFFER_BIT, &pbuffer_egl_config));

  auto display = std::make_unique<MockDisplay>();
  EXPECT_CALL(*display, IsValid).WillRepeatedly(Return(true));
  std::unique_ptr<Config> first_result;
  std::unique_ptr<Config> second_result;
  auto third_result =
      std::make_unique<Config>(ConfigDescriptor(), window_egl_config);
  auto fourth_result =
      std::make_unique<Config>(ConfigDescriptor(), pbuffer_egl_config);
  EXPECT_CALL(
      *display,
      ChooseConfig(Matcher<ConfigDescriptor>(AllOf(
          Field(&ConfigDescriptor::api, impeller::egl::API::kOpenGLES3),
          Field(&ConfigDescriptor::samples, impeller::egl::Samples::kFour),
          Field(&ConfigDescriptor::surface_type,
                impeller::egl::SurfaceType::kWindow)))))
      .WillOnce(Return(ByMove(std::move(first_result))));
  EXPECT_CALL(
      *display,
      ChooseConfig(Matcher<ConfigDescriptor>(AllOf(
          Field(&ConfigDescriptor::api, impeller::egl::API::kOpenGLES2),
          Field(&ConfigDescriptor::samples, impeller::egl::Samples::kFour),
          Field(&ConfigDescriptor::surface_type,
                impeller::egl::SurfaceType::kWindow)))))
      .WillOnce(Return(ByMove(std::move(second_result))));
  EXPECT_CALL(
      *display,
      ChooseConfig(Matcher<ConfigDescriptor>(
          AllOf(Field(&ConfigDescriptor::samples, impeller::egl::Samples::kOne),
                Field(&ConfigDescriptor::surface_type,
                      impeller::egl::SurfaceType::kWindow)))))
      .WillOnce(Return(ByMove(std::move(third_result))));
  EXPECT_CALL(*display, ChooseConfig(Matcher<ConfigDescriptor>(
                            Field(&ConfigDescriptor::surface_type,
                                  impeller::egl::SurfaceType::kPBuffer))))
      .WillOnce(Return(ByMove(std::move(fourth_result))));
  ON_CALL(*display, ChooseConfig(_))
      .WillByDefault(Return(ByMove(std::unique_ptr<Config>())));
  auto context =
      std::make_unique<AndroidContextGLImpeller>(std::move(display), true);
  ASSERT_TRUE(context);
}

}  // namespace testing
}  // namespace flutter
