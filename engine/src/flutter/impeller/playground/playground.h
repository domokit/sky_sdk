// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_PLAYGROUND_PLAYGROUND_H_
#define FLUTTER_IMPELLER_PLAYGROUND_PLAYGROUND_H_

#include <chrono>
#include <memory>

#include "flutter/fml/status.h"
#include "flutter/fml/time/time_delta.h"
#include "impeller/core/host_buffer.h"
#include "impeller/core/runtime_types.h"
#include "impeller/core/texture.h"
#include "impeller/geometry/point.h"
#include "impeller/playground/image/compressed_image.h"
#include "impeller/playground/image/decompressed_image.h"
#include "impeller/playground/switches.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/runtime_stage/runtime_stage.h"

namespace impeller {

class PlaygroundImpl;

enum class PlaygroundBackend {
  kMetal,
  kOpenGLES,
  kVulkan,
};

constexpr inline RuntimeStageBackend PlaygroundBackendToRuntimeStageBackend(
    PlaygroundBackend backend) {
  switch (backend) {
    case PlaygroundBackend::kMetal:
      return RuntimeStageBackend::kMetal;
    case PlaygroundBackend::kOpenGLES:
      return RuntimeStageBackend::kOpenGLES;
    case PlaygroundBackend::kVulkan:
      return RuntimeStageBackend::kVulkan;
  }
  FML_UNREACHABLE();
}

std::string PlaygroundBackendToString(PlaygroundBackend backend);

class Playground {
 public:
  using SinglePassCallback = std::function<bool(RenderPass& pass)>;

  explicit Playground(PlaygroundSwitches switches);

  virtual ~Playground();

  static bool ShouldOpenNewPlaygrounds();

  void SetupContext(PlaygroundBackend backend,
                    const PlaygroundSwitches& switches);

  void SetupWindow();

  void TeardownWindow();

  bool IsPlaygroundEnabled() const;

  Point GetCursorPosition() const;

  ISize GetWindowSize() const;

  Point GetContentScale() const;

  /// @brief Get the amount of time elapsed from the start of the playground's
  /// execution.
  Scalar GetSecondsElapsed() const;

  std::shared_ptr<Context> GetContext() const;

  std::shared_ptr<Context> MakeContext() const;

  using RenderCallback = std::function<bool(RenderTarget& render_target)>;

  bool OpenPlaygroundHere(const RenderCallback& render_callback);

  bool OpenPlaygroundHere(SinglePassCallback pass_callback);

  static std::shared_ptr<CompressedImage> LoadFixtureImageCompressed(
      std::shared_ptr<fml::Mapping> mapping);

  static std::optional<DecompressedImage> DecodeImageRGBA(
      const std::shared_ptr<CompressedImage>& compressed);

  static std::shared_ptr<Texture> CreateTextureForMapping(
      const std::shared_ptr<Context>& context,
      std::shared_ptr<fml::Mapping> mapping,
      bool enable_mipmapping = false);

  std::shared_ptr<Texture> CreateTextureForFixture(
      const char* fixture_name,
      bool enable_mipmapping = false) const;

  std::shared_ptr<Texture> CreateTextureCubeForFixture(
      std::array<const char*, 6> fixture_names) const;

  static bool SupportsBackend(PlaygroundBackend backend);

  virtual std::unique_ptr<fml::Mapping> OpenAssetAsMapping(
      std::string asset_name) const = 0;

  virtual std::string GetWindowTitle() const = 0;

  [[nodiscard]] fml::Status SetCapabilities(
      const std::shared_ptr<Capabilities>& capabilities);

  /// Returns true if `OpenPlaygroundHere` will actually render anything.
  bool WillRenderSomething() const;

  using GLProcAddressResolver = std::function<void*(const char* proc_name)>;
  GLProcAddressResolver CreateGLProcAddressResolver() const;

  using VKProcAddressResolver =
      std::function<void*(void* instance, const char* proc_name)>;
  VKProcAddressResolver CreateVKProcAddressResolver() const;

 protected:
  const PlaygroundSwitches switches_;

  virtual bool ShouldKeepRendering() const;

  void SetWindowSize(ISize size);

 private:
  fml::TimeDelta start_time_;
  std::unique_ptr<PlaygroundImpl> impl_;
  std::shared_ptr<Context> context_;
  Point cursor_position_;
  ISize window_size_ = ISize{1024, 768};
  std::shared_ptr<HostBuffer> host_buffer_;

  void SetCursorPosition(Point pos);

  Playground(const Playground&) = delete;

  Playground& operator=(const Playground&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_PLAYGROUND_PLAYGROUND_H_
