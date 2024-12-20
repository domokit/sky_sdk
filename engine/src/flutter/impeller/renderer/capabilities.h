// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_CAPABILITIES_H_
#define FLUTTER_IMPELLER_RENDERER_CAPABILITIES_H_

#include <memory>

#include "impeller/core/formats.h"

namespace impeller {

class Capabilities {
 public:
  virtual ~Capabilities();

  /// @brief  Whether the context backend supports attaching offscreen MSAA
  ///         color/stencil textures.
  virtual bool SupportsOffscreenMSAA() const = 0;

  /// @brief  Whether the context backend supports multisampled rendering to
  ///         the on-screen surface without requiring an explicit resolve of
  ///         the MSAA color attachment.
  virtual bool SupportsImplicitResolvingMSAA() const = 0;

  /// @brief  Whether the context backend supports binding Shader Storage Buffer
  ///         Objects (SSBOs) to pipelines.
  virtual bool SupportsSSBO() const = 0;

  /// @brief  Whether the context backend supports blitting from one texture
  ///         region to another texture region (via the relevant
  ///         `BlitPass::AddCopy` overloads).
  virtual bool SupportsTextureToTextureBlits() const = 0;

  /// @brief  Whether the context backend is able to support pipelines with
  ///         shaders that read from the framebuffer (i.e. pixels that have been
  ///         written by previous draw calls in the current render pass).
  ///
  ///         Example of reading from the first color attachment in a GLSL
  ///         shader:
  ///         ```
  ///         uniform subpassInput subpass_input;
  ///
  ///         out vec4 frag_color;
  ///
  ///         void main() {
  ///           vec4 color = subpassLoad(subpass_input);
  ///           // Invert the colors drawn to the framebuffer.
  ///           frag_color = vec4(vec3(1) - color.rgb, color.a);
  ///         }
  ///         ```
  virtual bool SupportsFramebufferFetch() const = 0;

  /// @brief  Whether the context backend supports `ComputePass`.
  virtual bool SupportsCompute() const = 0;

  /// @brief  Whether the context backend supports configuring `ComputePass`
  ///         command subgroups.
  virtual bool SupportsComputeSubgroups() const = 0;

  /// @brief  Whether the context backend supports binding the current
  ///         `RenderPass` attachments. This is supported if the backend can
  ///         guarantee that attachment textures will not be mutated until the
  ///         render pass has fully completed.
  ///
  ///         This is possible because many mobile graphics cards track
  ///         `RenderPass` attachment state in intermediary tile memory prior to
  ///         Storing the pass in the heap allocated attachments on DRAM.
  ///         Metal's hazard tracking and Vulkan's barriers are granular enough
  ///         to allow for safely accessing attachment textures prior to storage
  ///         in the same `RenderPass`.
  virtual bool SupportsReadFromResolve() const = 0;

  /// @brief  Whether the context backend supports `SamplerAddressMode::Decal`.
  virtual bool SupportsDecalSamplerAddressMode() const = 0;

  /// @brief  Whether the context backend supports allocating
  ///         `StorageMode::kDeviceTransient` (aka "memoryless") textures, which
  ///         are temporary textures kept in tile memory for the duration of the
  ///         `RenderPass` it's attached to.
  ///
  ///         This feature is especially useful for MSAA and stencils.
  virtual bool SupportsDeviceTransientTextures() const = 0;

  /// @brief Whether the primitive type TriangleFan is supported by the backend.
  virtual bool SupportsTriangleFan() const = 0;

  /// @brief Whether primitive restart is supported.
  virtual bool SupportsPrimitiveRestart() const = 0;

  /// @brief  Returns a supported `PixelFormat` for textures that store
  ///         4-channel colors (red/green/blue/alpha).
  virtual PixelFormat GetDefaultColorFormat() const = 0;

  /// @brief  Returns a supported `PixelFormat` for textures that store stencil
  ///         information. May include a depth channel if a stencil-only format
  ///         is not available.
  virtual PixelFormat GetDefaultStencilFormat() const = 0;

  /// @brief  Returns a supported `PixelFormat` for textures that store both a
  ///         stencil and depth component. This will never return a depth-only
  ///         or stencil-only texture.
  ///         Returns `PixelFormat::kUnknown` if no suitable depth+stencil
  ///         format was found.
  virtual PixelFormat GetDefaultDepthStencilFormat() const = 0;

  /// @brief Returns the default pixel format for the alpha bitmap glyph atlas.
  ///
  ///        Some backends may use Red channel while others use grey. This
  ///        should not have any impact
  virtual PixelFormat GetDefaultGlyphAtlasFormat() const = 0;

  /// @brief Return the maximum size of a render pass attachment.
  ///
  /// Note that this may be smaller than the maximum allocatable texture size.
  virtual ISize GetMaximumRenderPassAttachmentSize() const = 0;

 protected:
  Capabilities();

  Capabilities(const Capabilities&) = delete;

  Capabilities& operator=(const Capabilities&) = delete;
};

class CapabilitiesBuilder {
 public:
  CapabilitiesBuilder();

  ~CapabilitiesBuilder();

  CapabilitiesBuilder& SetSupportsOffscreenMSAA(bool value);

  CapabilitiesBuilder& SetSupportsSSBO(bool value);

  CapabilitiesBuilder& SetSupportsTextureToTextureBlits(bool value);

  CapabilitiesBuilder& SetSupportsFramebufferFetch(bool value);

  CapabilitiesBuilder& SetSupportsCompute(bool value);

  CapabilitiesBuilder& SetSupportsComputeSubgroups(bool value);

  CapabilitiesBuilder& SetSupportsReadFromResolve(bool value);

  CapabilitiesBuilder& SetDefaultColorFormat(PixelFormat value);

  CapabilitiesBuilder& SetDefaultStencilFormat(PixelFormat value);

  CapabilitiesBuilder& SetDefaultDepthStencilFormat(PixelFormat value);

  CapabilitiesBuilder& SetSupportsDecalSamplerAddressMode(bool value);

  CapabilitiesBuilder& SetSupportsDeviceTransientTextures(bool value);

  CapabilitiesBuilder& SetDefaultGlyphAtlasFormat(PixelFormat value);

  CapabilitiesBuilder& SetSupportsTriangleFan(bool value);

  CapabilitiesBuilder& SetMaximumRenderPassAttachmentSize(ISize size);

  std::unique_ptr<Capabilities> Build();

 private:
  bool supports_offscreen_msaa_ = false;
  bool supports_ssbo_ = false;
  bool supports_texture_to_texture_blits_ = false;
  bool supports_framebuffer_fetch_ = false;
  bool supports_compute_ = false;
  bool supports_compute_subgroups_ = false;
  bool supports_read_from_resolve_ = false;
  bool supports_decal_sampler_address_mode_ = false;
  bool supports_device_transient_textures_ = false;
  bool supports_triangle_fan_ = false;
  std::optional<PixelFormat> default_color_format_ = std::nullopt;
  std::optional<PixelFormat> default_stencil_format_ = std::nullopt;
  std::optional<PixelFormat> default_depth_stencil_format_ = std::nullopt;
  std::optional<PixelFormat> default_glyph_atlas_format_ = std::nullopt;
  std::optional<ISize> default_maximum_render_pass_attachment_size_ =
      std::nullopt;

  CapabilitiesBuilder(const CapabilitiesBuilder&) = delete;

  CapabilitiesBuilder& operator=(const CapabilitiesBuilder&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_CAPABILITIES_H_
