// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_CONTEXT_METAL_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_CONTEXT_METAL_H_

#include "flutter/shell/platform/embedder/tests/embedder_test_context.h"
#include "flutter/testing/test_metal_context.h"
#include "flutter/testing/test_metal_surface.h"

namespace flutter {
namespace testing {

class EmbedderTestContextMetal : public EmbedderTestContext {
 public:
  using TestExternalTextureCallback =
      std::function<bool(int64_t texture_id,
                         size_t w,
                         size_t h,
                         FlutterMetalExternalTexture* output)>;

  using NextDrawableCallback =
      std::function<FlutterMetalTexture(const FlutterFrameInfo* frame_info)>;

  using PresentCallback = std::function<bool(int64_t texture_id)>;

  explicit EmbedderTestContextMetal(std::string assets_path = "");

  ~EmbedderTestContextMetal() override;

  // |EmbedderTestContext|
  EmbedderTestContextType GetContextType() const override;

  // |EmbedderTestContext|
  size_t GetSurfacePresentCount() const override;

  void SetExternalTextureCallback(
      TestExternalTextureCallback external_texture_frame_callback);

  // Override the default handling for Present.
  void SetPresentCallback(PresentCallback present_callback);

  bool Present(int64_t texture_id);

  bool PopulateExternalTexture(int64_t texture_id,
                               size_t w,
                               size_t h,
                               FlutterMetalExternalTexture* output);

  TestMetalContext* GetTestMetalContext();

  TestMetalSurface* GetTestMetalSurface();

  // Override the default handling for GetNextDrawable.
  void SetNextDrawableCallback(NextDrawableCallback next_drawable_callback);

  FlutterMetalTexture GetNextDrawable(const FlutterFrameInfo* frame_info);

 private:
  // |EmbedderTestContext|
  void SetSurface(SkISize surface_size) override;

  // |EmbedderTestContext|
  void SetupCompositor() override;

  TestExternalTextureCallback external_texture_frame_callback_ = nullptr;
  SkISize surface_size_ = SkISize::MakeEmpty();
  std::unique_ptr<TestMetalContext> metal_context_;
  std::unique_ptr<TestMetalSurface> metal_surface_;
  size_t present_count_ = 0;
  PresentCallback present_callback_ = nullptr;
  NextDrawableCallback next_drawable_callback_ = nullptr;

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderTestContextMetal);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_CONTEXT_METAL_H_
