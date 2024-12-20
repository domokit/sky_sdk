// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_RENDER_TARGET_H_
#define FLUTTER_IMPELLER_RENDERER_RENDER_TARGET_H_

#include <functional>
#include <map>
#include <optional>

#include "flutter/fml/hash_combine.h"
#include "impeller/core/allocator.h"
#include "impeller/core/formats.h"
#include "impeller/geometry/size.h"

namespace impeller {

class Context;

struct RenderTargetConfig {
  ISize size = ISize{0, 0};
  size_t mip_count = 0;
  bool has_msaa = false;
  bool has_depth_stencil = false;

  constexpr bool operator==(const RenderTargetConfig& o) const {
    return size == o.size && mip_count == o.mip_count &&
           has_msaa == o.has_msaa && has_depth_stencil == o.has_depth_stencil;
  }

  constexpr size_t Hash() const {
    return fml::HashCombine(size.width, size.height, mip_count, has_msaa,
                            has_depth_stencil);
  }
};

class RenderTarget final {
 public:
  struct AttachmentConfig {
    StorageMode storage_mode;
    LoadAction load_action;
    StoreAction store_action;
    Color clear_color;
  };

  struct AttachmentConfigMSAA {
    StorageMode storage_mode;
    StorageMode resolve_storage_mode;
    LoadAction load_action;
    StoreAction store_action;
    Color clear_color;
  };

  static constexpr AttachmentConfig kDefaultColorAttachmentConfig = {
      .storage_mode = StorageMode::kDevicePrivate,
      .load_action = LoadAction::kClear,
      .store_action = StoreAction::kStore,
      .clear_color = Color::BlackTransparent()};

  static constexpr AttachmentConfigMSAA kDefaultColorAttachmentConfigMSAA = {
      .storage_mode = StorageMode::kDeviceTransient,
      .resolve_storage_mode = StorageMode::kDevicePrivate,
      .load_action = LoadAction::kClear,
      .store_action = StoreAction::kMultisampleResolve,
      .clear_color = Color::BlackTransparent()};

  static constexpr AttachmentConfig kDefaultStencilAttachmentConfig = {
      .storage_mode = StorageMode::kDeviceTransient,
      .load_action = LoadAction::kClear,
      .store_action = StoreAction::kDontCare,
      .clear_color = Color::BlackTransparent()};

  RenderTarget();

  ~RenderTarget();

  bool IsValid() const;

  void SetupDepthStencilAttachments(
      const Context& context,
      Allocator& allocator,
      ISize size,
      bool msaa,
      std::string_view label = "Offscreen",
      RenderTarget::AttachmentConfig stencil_attachment_config =
          RenderTarget::kDefaultStencilAttachmentConfig,
      const std::shared_ptr<Texture>& depth_stencil_texture = nullptr);

  SampleCount GetSampleCount() const;

  bool HasColorAttachment(size_t index) const;

  ISize GetRenderTargetSize() const;

  std::shared_ptr<Texture> GetRenderTargetTexture() const;

  PixelFormat GetRenderTargetPixelFormat() const;

  std::optional<ISize> GetColorAttachmentSize(size_t index) const;

  RenderTarget& SetColorAttachment(const ColorAttachment& attachment,
                                   size_t index);

  /// @brief Get the color attachment at [index].
  ///
  /// This function does not validate whether or not the attachment was
  /// previously defined and will return a default constructed attachment
  /// if none is set.
  ColorAttachment GetColorAttachment(size_t index) const;

  RenderTarget& SetDepthAttachment(std::optional<DepthAttachment> attachment);

  RenderTarget& SetStencilAttachment(
      std::optional<StencilAttachment> attachment);

  size_t GetMaxColorAttacmentBindIndex() const;

  const std::optional<DepthAttachment>& GetDepthAttachment() const;

  const std::optional<StencilAttachment>& GetStencilAttachment() const;

  size_t GetTotalAttachmentCount() const;

  bool IterateAllColorAttachments(
      const std::function<bool(size_t index,
                               const ColorAttachment& attachment)>& iterator)
      const;

  void IterateAllAttachments(
      const std::function<bool(const Attachment& attachment)>& iterator) const;

  std::string ToString() const;

  RenderTargetConfig ToConfig() const;

 private:
  std::optional<ColorAttachment> color0_;
  std::optional<DepthAttachment> depth_;
  std::optional<StencilAttachment> stencil_;
  // Note: color0 is stored in the color0_ field above and not in this map,
  // to avoid heap allocations for the commonly created render target formats
  // in Flutter.
  std::map<size_t, ColorAttachment> colors_;
};

/// @brief a wrapper around the impeller [Allocator] instance that can be used
///        to provide caching of allocated render target textures.
class RenderTargetAllocator {
 public:
  explicit RenderTargetAllocator(std::shared_ptr<Allocator> allocator);

  virtual ~RenderTargetAllocator() = default;

  virtual RenderTarget CreateOffscreen(
      const Context& context,
      ISize size,
      int mip_count,
      std::string_view label = "Offscreen",
      RenderTarget::AttachmentConfig color_attachment_config =
          RenderTarget::kDefaultColorAttachmentConfig,
      std::optional<RenderTarget::AttachmentConfig> stencil_attachment_config =
          RenderTarget::kDefaultStencilAttachmentConfig,
      const std::shared_ptr<Texture>& existing_color_texture = nullptr,
      const std::shared_ptr<Texture>& existing_depth_stencil_texture = nullptr);

  virtual RenderTarget CreateOffscreenMSAA(
      const Context& context,
      ISize size,
      int mip_count,
      std::string_view label = "Offscreen MSAA",
      RenderTarget::AttachmentConfigMSAA color_attachment_config =
          RenderTarget::kDefaultColorAttachmentConfigMSAA,
      std::optional<RenderTarget::AttachmentConfig> stencil_attachment_config =
          RenderTarget::kDefaultStencilAttachmentConfig,
      const std::shared_ptr<Texture>& existing_color_msaa_texture = nullptr,
      const std::shared_ptr<Texture>& existing_color_resolve_texture = nullptr,
      const std::shared_ptr<Texture>& existing_depth_stencil_texture = nullptr);

  /// @brief Mark the beginning of a frame workload.
  ///
  ///       This may be used to reset any tracking state on whether or not a
  ///       particular texture instance is still in use.
  virtual void Start();

  /// @brief Mark the end of a frame workload.
  ///
  ///        This may be used to deallocate any unused textures.
  virtual void End();

 private:
  std::shared_ptr<Allocator> allocator_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_RENDER_TARGET_H_
