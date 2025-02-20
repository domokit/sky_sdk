// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/blit_command_gles.h"

#include "flutter/fml/closure.h"
#include "fml/trace_event.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/geometry/point.h"
#include "impeller/renderer/backend/gles/device_buffer_gles.h"
#include "impeller/renderer/backend/gles/reactor_gles.h"
#include "impeller/renderer/backend/gles/texture_gles.h"

namespace impeller {

namespace {
static void FlipImage(uint8_t* buffer,
                      size_t width,
                      size_t height,
                      size_t stride) {
  if (buffer == nullptr || stride == 0) {
    return;
  }

  const auto byte_width = width * stride;

  for (size_t top = 0; top < height; top++) {
    size_t bottom = height - top - 1;
    if (top >= bottom) {
      break;
    }
    auto* top_row = buffer + byte_width * top;
    auto* bottom_row = buffer + byte_width * bottom;
    std::swap_ranges(top_row, top_row + byte_width, bottom_row);
  }
}
}  // namespace

BlitEncodeGLES::~BlitEncodeGLES() = default;

static void DeleteFBO(const ProcTableGLES& gl, GLuint fbo, GLenum type) {
  if (fbo != GL_NONE) {
    gl.BindFramebuffer(type, GL_NONE);
    gl.DeleteFramebuffers(1u, &fbo);
  }
};

static std::optional<GLuint> ConfigureFBO(
    const ProcTableGLES& gl,
    const std::shared_ptr<Texture>& texture,
    GLenum fbo_type) {
  auto handle = TextureGLES::Cast(texture.get())->GetGLHandle();
  if (!handle.has_value()) {
    return std::nullopt;
  }

  if (TextureGLES::Cast(*texture).IsWrapped()) {
    // The texture is attached to the default FBO, so there's no need to
    // create/configure one.
    gl.BindFramebuffer(fbo_type, 0);
    return 0;
  }

  GLuint fbo;
  gl.GenFramebuffers(1u, &fbo);
  gl.BindFramebuffer(fbo_type, fbo);

  if (!TextureGLES::Cast(*texture).SetAsFramebufferAttachment(
          fbo_type, TextureGLES::AttachmentType::kColor0)) {
    VALIDATION_LOG << "Could not attach texture to framebuffer.";
    DeleteFBO(gl, fbo, fbo_type);
    return std::nullopt;
  }

  if (gl.CheckFramebufferStatus(fbo_type) != GL_FRAMEBUFFER_COMPLETE) {
    VALIDATION_LOG << "Could not create a complete framebuffer.";
    DeleteFBO(gl, fbo, fbo_type);
    return std::nullopt;
  }

  return fbo;
};

BlitCopyTextureToTextureCommandGLES::~BlitCopyTextureToTextureCommandGLES() =
    default;

std::string BlitCopyTextureToTextureCommandGLES::GetLabel() const {
  return label;
}

bool BlitCopyTextureToTextureCommandGLES::Encode(
    const ReactorGLES& reactor) const {
  const auto& gl = reactor.GetProcTable();

  // glBlitFramebuffer is a GLES3 proc. Since we target GLES2, we need to
  // emulate the blit when it's not available in the driver.
  if (!gl.BlitFramebuffer.IsAvailable()) {
    // TODO(157064): Emulate the blit using a raster draw call here.
    VALIDATION_LOG << "Texture blit fallback not implemented yet for GLES2.";
    return false;
  }

  GLuint read_fbo = GL_NONE;
  GLuint draw_fbo = GL_NONE;
  fml::ScopedCleanupClosure delete_fbos([&gl, &read_fbo, &draw_fbo]() {
    DeleteFBO(gl, read_fbo, GL_READ_FRAMEBUFFER);
    DeleteFBO(gl, draw_fbo, GL_DRAW_FRAMEBUFFER);
  });

  {
    auto read = ConfigureFBO(gl, source, GL_READ_FRAMEBUFFER);
    if (!read.has_value()) {
      return false;
    }
    read_fbo = read.value();
  }

  {
    auto draw = ConfigureFBO(gl, destination, GL_DRAW_FRAMEBUFFER);
    if (!draw.has_value()) {
      return false;
    }
    draw_fbo = draw.value();
  }

  gl.Disable(GL_SCISSOR_TEST);
  gl.Disable(GL_DEPTH_TEST);
  gl.Disable(GL_STENCIL_TEST);

  gl.BlitFramebuffer(source_region.GetX(),       // srcX0
                     source_region.GetY(),       // srcY0
                     source_region.GetWidth(),   // srcX1
                     source_region.GetHeight(),  // srcY1
                     destination_origin.x,       // dstX0
                     destination_origin.y,       // dstY0
                     source_region.GetWidth(),   // dstX1
                     source_region.GetHeight(),  // dstY1
                     GL_COLOR_BUFFER_BIT,        // mask
                     GL_NEAREST                  // filter
  );

  return true;
};

namespace {
struct TexImage2DData {
  GLint internal_format = 0;
  GLenum external_format = GL_NONE;
  GLenum type = GL_NONE;
  BufferView buffer_view;

  explicit TexImage2DData(PixelFormat pixel_format) {
    switch (pixel_format) {
      case PixelFormat::kA8UNormInt:
        internal_format = GL_ALPHA;
        external_format = GL_ALPHA;
        type = GL_UNSIGNED_BYTE;
        break;
      case PixelFormat::kR8UNormInt:
        internal_format = GL_RED;
        external_format = GL_RED;
        type = GL_UNSIGNED_BYTE;
        break;
      case PixelFormat::kR8G8B8A8UNormInt:
      case PixelFormat::kB8G8R8A8UNormInt:
      case PixelFormat::kR8G8B8A8UNormIntSRGB:
      case PixelFormat::kB8G8R8A8UNormIntSRGB:
        internal_format = GL_RGBA;
        external_format = GL_RGBA;
        type = GL_UNSIGNED_BYTE;
        break;
      case PixelFormat::kR32G32B32A32Float:
        internal_format = GL_RGBA;
        external_format = GL_RGBA;
        type = GL_FLOAT;
        break;
      case PixelFormat::kR16G16B16A16Float:
        internal_format = GL_RGBA;
        external_format = GL_RGBA;
        type = GL_HALF_FLOAT;
        break;
      case PixelFormat::kS8UInt:
        // Pure stencil textures are only available in OpenGL 4.4+, which is
        // ~0% of mobile devices. Instead, we use a depth-stencil texture and
        // only use the stencil component.
        //
        // https://registry.khronos.org/OpenGL-Refpages/gl4/html/glTexImage2D.xhtml
      case PixelFormat::kD24UnormS8Uint:
        internal_format = GL_DEPTH_STENCIL;
        external_format = GL_DEPTH_STENCIL;
        type = GL_UNSIGNED_INT_24_8;
        break;
      case PixelFormat::kUnknown:
      case PixelFormat::kD32FloatS8UInt:
      case PixelFormat::kR8G8UNormInt:
      case PixelFormat::kB10G10R10XRSRGB:
      case PixelFormat::kB10G10R10XR:
      case PixelFormat::kB10G10R10A10XR:
        return;
    }
    is_valid_ = true;
  }

  TexImage2DData(PixelFormat pixel_format, BufferView p_buffer_view)
      : TexImage2DData(pixel_format) {
    buffer_view = std::move(p_buffer_view);
  }

  bool IsValid() const { return is_valid_; }

 private:
  bool is_valid_ = false;
};
}  // namespace

BlitCopyBufferToTextureCommandGLES::~BlitCopyBufferToTextureCommandGLES() =
    default;

std::string BlitCopyBufferToTextureCommandGLES::GetLabel() const {
  return label;
}

bool BlitCopyBufferToTextureCommandGLES::Encode(
    const ReactorGLES& reactor) const {
  TextureGLES& texture_gles = TextureGLES::Cast(*destination);

  if (texture_gles.GetType() != TextureGLES::Type::kTexture) {
    VALIDATION_LOG << "Incorrect texture usage flags for setting contents on "
                      "this texture object.";
    return false;
  }

  if (texture_gles.IsWrapped()) {
    VALIDATION_LOG << "Cannot set the contents of a wrapped texture.";
    return false;
  }

  const auto& tex_descriptor = texture_gles.GetTextureDescriptor();

  if (tex_descriptor.size.IsEmpty()) {
    return true;
  }

  if (!tex_descriptor.IsValid() ||
      source.GetRange().length !=
          BytesPerPixelForPixelFormat(tex_descriptor.format) *
              destination_region.Area()) {
    return false;
  }

  destination->SetCoordinateSystem(TextureCoordinateSystem::kUploadFromHost);

  GLenum texture_type;
  GLenum texture_target;
  switch (tex_descriptor.type) {
    case TextureType::kTexture2D:
      texture_type = GL_TEXTURE_2D;
      texture_target = GL_TEXTURE_2D;
      break;
    case TextureType::kTexture2DMultisample:
      VALIDATION_LOG << "Multisample texture uploading is not supported for "
                        "the OpenGLES backend.";
      return false;
    case TextureType::kTextureCube:
      texture_type = GL_TEXTURE_CUBE_MAP;
      texture_target = GL_TEXTURE_CUBE_MAP_POSITIVE_X + slice;
      break;
    case TextureType::kTextureExternalOES:
      texture_type = GL_TEXTURE_EXTERNAL_OES;
      texture_target = GL_TEXTURE_EXTERNAL_OES;
      break;
  }

  TexImage2DData data = TexImage2DData(tex_descriptor.format, source);
  if (!data.IsValid()) {
    VALIDATION_LOG << "Invalid texture format.";
    return false;
  }

  auto gl_handle = texture_gles.GetGLHandle();
  if (!gl_handle.has_value()) {
    VALIDATION_LOG
        << "Texture was collected before it could be uploaded to the GPU.";
    return false;
  }
  const auto& gl = reactor.GetProcTable();
  gl.BindTexture(texture_type, gl_handle.value());
  const GLvoid* tex_data = data.buffer_view.GetBuffer()->OnGetContents() +
                           data.buffer_view.GetRange().offset;

  // GL_INVALID_OPERATION if the texture array has not been
  // defined by a previous glTexImage2D operation.
  if (!texture_gles.IsSliceInitialized(slice)) {
    gl.TexImage2D(texture_target,              // target
                  mip_level,                   // LOD level
                  data.internal_format,        // internal format
                  tex_descriptor.size.width,   // width
                  tex_descriptor.size.height,  // height
                  0u,                          // border
                  data.external_format,        // external format
                  data.type,                   // type
                  nullptr                      // data
    );
    texture_gles.MarkSliceInitialized(slice);
  }

  {
    gl.PixelStorei(GL_UNPACK_ALIGNMENT, 1);
    gl.TexSubImage2D(texture_target,                  // target
                     mip_level,                       // LOD level
                     destination_region.GetX(),       // xoffset
                     destination_region.GetY(),       // yoffset
                     destination_region.GetWidth(),   // width
                     destination_region.GetHeight(),  // height
                     data.external_format,            // external format
                     data.type,                       // type
                     tex_data                         // data

    );
  }
  return true;
}

BlitCopyTextureToBufferCommandGLES::~BlitCopyTextureToBufferCommandGLES() =
    default;

std::string BlitCopyTextureToBufferCommandGLES::GetLabel() const {
  return label;
}

bool BlitCopyTextureToBufferCommandGLES::Encode(
    const ReactorGLES& reactor) const {
  if (source->GetTextureDescriptor().format != PixelFormat::kR8G8B8A8UNormInt) {
    VALIDATION_LOG << "Only textures with pixel format RGBA are supported yet.";
    return false;
  }

  const auto& gl = reactor.GetProcTable();
  TextureCoordinateSystem coord_system = source->GetCoordinateSystem();

  GLuint read_fbo = GL_NONE;
  fml::ScopedCleanupClosure delete_fbos(
      [&gl, &read_fbo]() { DeleteFBO(gl, read_fbo, GL_READ_FRAMEBUFFER); });

  {
    auto read = ConfigureFBO(gl, source, GL_READ_FRAMEBUFFER);
    if (!read.has_value()) {
      return false;
    }
    read_fbo = read.value();
  }

  DeviceBufferGLES::Cast(*destination)
      .UpdateBufferData([&gl, this, coord_system,
                         rows = source->GetSize().height](uint8_t* data,

                                                          size_t length) {
        gl.ReadPixels(source_region.GetX(), source_region.GetY(),
                      source_region.GetWidth(), source_region.GetHeight(),
                      GL_RGBA, GL_UNSIGNED_BYTE, data + destination_offset);
        switch (coord_system) {
          case TextureCoordinateSystem::kUploadFromHost:
            break;
          case TextureCoordinateSystem::kRenderToTexture:
            // The texture is upside down, and must be inverted when copying
            // byte data out.
            FlipImage(data + destination_offset, source_region.GetWidth(),
                      source_region.GetHeight(), 4);
        }
      });

  return true;
};

BlitGenerateMipmapCommandGLES::~BlitGenerateMipmapCommandGLES() = default;

std::string BlitGenerateMipmapCommandGLES::GetLabel() const {
  return label;
}

bool BlitGenerateMipmapCommandGLES::Encode(const ReactorGLES& reactor) const {
  auto texture_gles = TextureGLES::Cast(texture.get());
  if (!texture_gles->GenerateMipmap()) {
    return false;
  }

  return true;
};

//////  BlitResizeTextureCommandGLES
//////////////////////////////////////////////////////

BlitResizeTextureCommandGLES::~BlitResizeTextureCommandGLES() = default;

std::string BlitResizeTextureCommandGLES::GetLabel() const {
  return "Resize Texture";
}

bool BlitResizeTextureCommandGLES::Encode(const ReactorGLES& reactor) const {
  const auto& gl = reactor.GetProcTable();

  // glBlitFramebuffer is a GLES3 proc. Since we target GLES2, we need to
  // emulate the blit when it's not available in the driver.
  if (!gl.BlitFramebuffer.IsAvailable()) {
    // TODO(157064): Emulate the blit using a raster draw call here.
    VALIDATION_LOG << "Texture blit fallback not implemented yet for GLES2.";
    return false;
  }

  destination->SetCoordinateSystem(source->GetCoordinateSystem());

  GLuint read_fbo = GL_NONE;
  GLuint draw_fbo = GL_NONE;
  fml::ScopedCleanupClosure delete_fbos([&gl, &read_fbo, &draw_fbo]() {
    DeleteFBO(gl, read_fbo, GL_READ_FRAMEBUFFER);
    DeleteFBO(gl, draw_fbo, GL_DRAW_FRAMEBUFFER);
  });

  {
    auto read = ConfigureFBO(gl, source, GL_READ_FRAMEBUFFER);
    if (!read.has_value()) {
      return false;
    }
    read_fbo = read.value();
  }

  {
    auto draw = ConfigureFBO(gl, destination, GL_DRAW_FRAMEBUFFER);
    if (!draw.has_value()) {
      return false;
    }
    draw_fbo = draw.value();
  }

  gl.Disable(GL_SCISSOR_TEST);
  gl.Disable(GL_DEPTH_TEST);
  gl.Disable(GL_STENCIL_TEST);

  const IRect source_region = IRect::MakeSize(source->GetSize());
  const IRect destination_region = IRect::MakeSize(destination->GetSize());

  gl.BlitFramebuffer(source_region.GetX(),            // srcX0
                     source_region.GetY(),            // srcY0
                     source_region.GetWidth(),        // srcX1
                     source_region.GetHeight(),       // srcY1
                     destination_region.GetX(),       // dstX0
                     destination_region.GetY(),       // dstY0
                     destination_region.GetWidth(),   // dstX1
                     destination_region.GetHeight(),  // dstY1
                     GL_COLOR_BUFFER_BIT,             // mask
                     GL_LINEAR                        // filter
  );

  return true;
}

}  // namespace impeller
