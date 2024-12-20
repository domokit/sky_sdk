// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BLIT_PASS_H_
#define FLUTTER_IMPELLER_RENDERER_BLIT_PASS_H_

#include <cstdint>
#include <string>

#include "impeller/core/device_buffer.h"
#include "impeller/core/texture.h"

namespace impeller {

class HostBuffer;
class Allocator;

//------------------------------------------------------------------------------
/// @brief      Blit passes encode blit into the underlying command buffer.
///
///             Blit passes can be obtained from the command buffer in which
///             the pass is meant to encode commands into.
///
/// @see        `CommandBuffer`
///
class BlitPass {
 public:
  virtual ~BlitPass();

  virtual bool IsValid() const = 0;

  void SetLabel(std::string_view label);

  //----------------------------------------------------------------------------
  /// @brief      If the texture is not already in a shader read internal
  ///             state, then convert it to that state.
  ///
  ///             This API is only used by Vulkan.
  virtual bool ConvertTextureToShaderRead(
      const std::shared_ptr<Texture>& texture);

  //----------------------------------------------------------------------------
  /// @brief      Resize the [source] texture into the [destination] texture.
  ///
  ///             On Metal platforms, [destination] is required to be non-lossy
  ///             and have the Shader read capability.
  virtual bool ResizeTexture(const std::shared_ptr<Texture>& source,
                             const std::shared_ptr<Texture>& destination) = 0;

  //----------------------------------------------------------------------------
  /// @brief      Record a command to copy the contents of one texture to
  ///             another texture. The blit area is limited by the intersection
  ///             of the texture coverage with respect the source region and
  ///             destination origin.
  ///
  /// @param[in]  source              The texture to read for copying.
  /// @param[in]  destination         The texture to overwrite using the source
  ///                                 contents.
  /// @param[in]  source_region       The optional region of the source texture
  ///                                 to use for copying. If not specified, the
  ///                                 full size of the source texture is used.
  /// @param[in]  destination_origin  The origin to start writing to in the
  ///                                 destination texture.
  /// @param[in]  label               The optional debug label to give the
  ///                                 command.
  ///
  /// @return     If the command was valid for subsequent commitment.
  ///
  bool AddCopy(std::shared_ptr<Texture> source,
               std::shared_ptr<Texture> destination,
               std::optional<IRect> source_region = std::nullopt,
               IPoint destination_origin = {},
               std::string_view label = "");

  //----------------------------------------------------------------------------
  /// @brief      Record a command to copy the contents of a texture to a
  ///             buffer.
  ///
  /// @param[in]  source              The texture to read for copying.
  /// @param[in]  destination         The buffer to overwrite using the source
  ///                                 contents.
  /// @param[in]  source_region       The optional region of the source texture
  ///                                 to use for copying. If not specified, the
  ///                                 full size of the source texture is used.
  /// @param[in]  destination_origin  The origin to start writing to in the
  ///                                 destination texture.
  /// @param[in]  label               The optional debug label to give the
  ///                                 command.
  ///
  /// @return     If the command was valid for subsequent commitment.
  ///
  bool AddCopy(std::shared_ptr<Texture> source,
               std::shared_ptr<DeviceBuffer> destination,
               std::optional<IRect> source_region = std::nullopt,
               size_t destination_offset = 0,
               std::string_view label = "");

  //----------------------------------------------------------------------------
  /// @brief      Record a command to copy the contents of a buffer to a
  ///             texture.
  ///
  /// @param[in]  source              The buffer view to read for copying.
  /// @param[in]  destination         The texture to overwrite using the source
  ///                                 contents.
  /// @param[in]  destination_region  The offset to start writing to in the
  ///                                 destination texture. If not provided, this
  ///                                 defaults to the entire texture.
  /// @param[in]  label               The optional debug label to give the
  ///                                 command.
  /// @param[in]  mip_level           The mip level to write to.
  /// @param[in]  slice               For cubemap textures, the slice to write
  ///                                 data to.
  /// @param[in]  convert_to_read     Whether to convert the texture to a shader
  ///                                 read state. Defaults to true.
  ///
  /// @return     If the command was valid for subsequent commitment.
  ///
  /// If a region smaller than the texture size is provided, the
  /// contents are treated as containing tightly packed pixel data of
  /// that region. Only the portion of the texture in this region is
  /// replaced and existing data is preserved.
  ///
  /// For example, to replace the top left 10 x 10 region of a larger
  /// 100 x 100 texture, the region is {0, 0, 10, 10} and the expected
  /// buffer size in bytes is 100 x bpp.
  bool AddCopy(BufferView source,
               std::shared_ptr<Texture> destination,
               std::optional<IRect> destination_region = std::nullopt,
               std::string_view label = "",
               uint32_t mip_level = 0,
               uint32_t slice = 0,
               bool convert_to_read = true);

  //----------------------------------------------------------------------------
  /// @brief      Record a command to generate all mip levels for a texture.
  ///
  /// @param[in]  texture  The texture to generate mipmaps for.
  /// @param[in]  label    The optional debug label to give the command.
  ///
  /// @return     If the command was valid for subsequent commitment.
  ///
  bool GenerateMipmap(std::shared_ptr<Texture> texture,
                      std::string_view label = "");

  //----------------------------------------------------------------------------
  /// @brief      Encode the recorded commands to the underlying command buffer.
  ///
  /// @param      transients_allocator  The transients allocator.
  ///
  /// @return     If the commands were encoded to the underlying command
  ///             buffer.
  ///
  virtual bool EncodeCommands(
      const std::shared_ptr<Allocator>& transients_allocator) const = 0;

 protected:
  explicit BlitPass();

  virtual void OnSetLabel(std::string_view label) = 0;

  virtual bool OnCopyTextureToTextureCommand(
      std::shared_ptr<Texture> source,
      std::shared_ptr<Texture> destination,
      IRect source_region,
      IPoint destination_origin,
      std::string_view label) = 0;

  virtual bool OnCopyTextureToBufferCommand(
      std::shared_ptr<Texture> source,
      std::shared_ptr<DeviceBuffer> destination,
      IRect source_region,
      size_t destination_offset,
      std::string_view label) = 0;

  virtual bool OnCopyBufferToTextureCommand(
      BufferView source,
      std::shared_ptr<Texture> destination,
      IRect destination_region,
      std::string_view label,
      uint32_t mip_level,
      uint32_t slice,
      bool convert_to_read) = 0;

  virtual bool OnGenerateMipmapCommand(std::shared_ptr<Texture> texture,
                                       std::string_view label) = 0;

 private:
  BlitPass(const BlitPass&) = delete;

  BlitPass& operator=(const BlitPass&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BLIT_PASS_H_
