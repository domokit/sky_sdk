// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_CONTEXT_GL_H_
#define FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_CONTEXT_GL_H_

#include "flutter/shell/platform/embedder/tests/embedder_test_context.h"

#include "flutter/testing/test_gl_context.h"
#include "flutter/testing/test_gl_surface.h"

namespace flutter {
namespace testing {

class EmbedderTestContextGL : public EmbedderTestContext {
 public:
  using GLGetFBOCallback = std::function<void(FlutterFrameInfo frame_info)>;
  using GLPopulateExistingDamageCallback =
      std::function<void(intptr_t id, FlutterDamage* existing_damage)>;
  using GLPresentCallback =
      std::function<void(FlutterPresentInfo present_info)>;

  explicit EmbedderTestContextGL(std::string assets_path = "");

  ~EmbedderTestContextGL() override;

  size_t GetSurfacePresentCount() const override;

  // |EmbedderTestContext|
  EmbedderTestContextType GetContextType() const override;

  // Used to explicitly set an `open_gl.fbo_callback`. Using this method will
  // cause your test to fail since the ctor for this class sets
  // `open_gl.fbo_callback_with_frame_info`. This method exists as a utility to
  // explicitly test this behavior.
  void SetOpenGLFBOCallBack();

  // Used to explicitly set an `open_gl.present`. Using this method will cause
  // your test to fail since the ctor for this class sets
  // `open_gl.present_with_info`. This method exists as a utility to explicitly
  // test this behavior.
  void SetOpenGLPresentCallBack();

  //----------------------------------------------------------------------------
  /// @brief      Sets a callback that will be invoked (on the raster task
  ///             runner) when the engine asks the embedder for a new FBO ID at
  ///             the updated size.
  ///
  /// @attention  The callback will be invoked on the raster task runner. The
  ///             callback can be set on the tests host thread.
  ///
  /// @param[in]  callback  The callback to set. The previous callback will be
  ///                       un-registered.
  ///
  void SetGLGetFBOCallback(const GLGetFBOCallback& callback);

  void SetGLPopulateExistingDamageCallback(
      GLPopulateExistingDamageCallback callback);

  uint32_t GetWindowFBOId() const;

  //----------------------------------------------------------------------------
  /// @brief      Sets a callback that will be invoked (on the raster task
  ///             runner) when the engine presents an fbo that was given by the
  ///             embedder.
  ///
  /// @attention  The callback will be invoked on the raster task runner. The
  ///             callback can be set on the tests host thread.
  ///
  /// @param[in]  callback  The callback to set. The previous callback will be
  ///                       un-registered.
  ///
  void SetGLPresentCallback(GLPresentCallback callback);

  void GLPopulateExistingDamage(const intptr_t id,
                                FlutterDamage* existing_damage);

  void* GLGetProcAddress(const char* name);

 private:
  // |EmbedderTestContext|
  void SetSurface(SkISize surface_size) override;

  // |EmbedderTestContext|
  void SetupCompositor() override;

  std::shared_ptr<TestEGLContext> egl_context_;
  std::unique_ptr<TestGLSurface> gl_surface_;
  size_t gl_surface_present_count_ = 0;
  std::mutex gl_callback_mutex_;
  GLGetFBOCallback gl_get_fbo_callback_;
  GLPresentCallback gl_present_callback_;
  GLPopulateExistingDamageCallback gl_populate_existing_damage_callback_;

  bool GLMakeCurrent();

  bool GLClearCurrent();

  bool GLPresent(FlutterPresentInfo present_info);

  uint32_t GLGetFramebuffer(FlutterFrameInfo frame_info);

  bool GLMakeResourceCurrent();

  FML_DISALLOW_COPY_AND_ASSIGN(EmbedderTestContextGL);
};

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_EMBEDDER_TESTS_EMBEDDER_TEST_CONTEXT_GL_H_
