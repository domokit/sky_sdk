// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_PLAYGROUND_BACKEND_GLES_PLAYGROUND_IMPL_GLES_H_
#define FLUTTER_IMPELLER_PLAYGROUND_BACKEND_GLES_PLAYGROUND_IMPL_GLES_H_

#include "impeller/playground/playground_impl.h"

namespace impeller {

class PlaygroundImplGLES final : public PlaygroundImpl {
 public:
  explicit PlaygroundImplGLES(PlaygroundSwitches switches);

  ~PlaygroundImplGLES();

  fml::Status SetCapabilities(
      const std::shared_ptr<Capabilities>& capabilities) override;

 private:
  class ReactorWorker;

  static void DestroyWindowHandle(WindowHandle handle);
  using UniqueHandle = std::unique_ptr<void, decltype(&DestroyWindowHandle)>;
  UniqueHandle handle_;
  std::shared_ptr<ReactorWorker> worker_;
  const bool use_angle_;
  void* angle_glesv2_;

  // |PlaygroundImpl|
  std::shared_ptr<Context> GetContext() const override;

  // |PlaygroundImpl|
  WindowHandle GetWindowHandle() const override;

  // |PlaygroundImpl|
  std::unique_ptr<Surface> AcquireSurfaceFrame(
      std::shared_ptr<Context> context) override;

  // |PlaygroundImpl|
  Playground::GLProcAddressResolver CreateGLProcAddressResolver()
      const override;

  PlaygroundImplGLES(const PlaygroundImplGLES&) = delete;

  PlaygroundImplGLES& operator=(const PlaygroundImplGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_PLAYGROUND_BACKEND_GLES_PLAYGROUND_IMPL_GLES_H_
