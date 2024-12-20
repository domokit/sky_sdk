// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/texture_gles.h"

#include <optional>
#include <utility>

#include "flutter/fml/logging.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/trace_event.h"
#include "impeller/base/allocation.h"
#include "impeller/base/strings.h"
#include "impeller/base/validation.h"
#include "impeller/core/formats.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/renderer/backend/gles/formats_gles.h"

namespace impeller {

namespace {
static bool IsDepthStencilFormat(PixelFormat format) {
  switch (format) {
    case PixelFormat::kS8UInt:
    case PixelFormat::kD24UnormS8Uint:
    case PixelFormat::kD32FloatS8UInt:
      return true;
    case PixelFormat::kUnknown:
    case PixelFormat::kA8UNormInt:
    case PixelFormat::kR8UNormInt:
    case PixelFormat::kR8G8UNormInt:
    case PixelFormat::kR8G8B8A8UNormInt:
    case PixelFormat::kR8G8B8A8UNormIntSRGB:
    case PixelFormat::kB8G8R8A8UNormInt:
    case PixelFormat::kB8G8R8A8UNormIntSRGB:
    case PixelFormat::kR32G32B32A32Float:
    case PixelFormat::kR16G16B16A16Float:
    case PixelFormat::kB10G10R10XR:
    case PixelFormat::kB10G10R10XRSRGB:
    case PixelFormat::kB10G10R10A10XR:
      return false;
  }
  FML_UNREACHABLE();
}

static TextureGLES::Type GetTextureTypeFromDescriptor(
    const TextureDescriptor& desc) {
  const auto usage = static_cast<TextureUsageMask>(desc.usage);
  const auto render_target = TextureUsage::kRenderTarget;
  const auto is_msaa = desc.sample_count == SampleCount::kCount4;
  if (usage == render_target && IsDepthStencilFormat(desc.format)) {
    return is_msaa ? TextureGLES::Type::kRenderBufferMultisampled
                   : TextureGLES::Type::kRenderBuffer;
  }
  return is_msaa ? TextureGLES::Type::kTextureMultisampled
                 : TextureGLES::Type::kTexture;
}

struct TexImage2DData {
  GLint internal_format = 0;
  GLenum external_format = GL_NONE;
  GLenum type = GL_NONE;
  std::shared_ptr<const fml::Mapping> data;

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

  TexImage2DData(PixelFormat pixel_format,
                 std::shared_ptr<const fml::Mapping> mapping)
      : TexImage2DData(pixel_format) {
    data = std::move(mapping);
  }

  bool IsValid() const { return is_valid_; }

 private:
  bool is_valid_ = false;
};
}  // namespace

HandleType ToHandleType(TextureGLES::Type type) {
  switch (type) {
    case TextureGLES::Type::kTexture:
    case TextureGLES::Type::kTextureMultisampled:
      return HandleType::kTexture;
    case TextureGLES::Type::kRenderBuffer:
    case TextureGLES::Type::kRenderBufferMultisampled:
      return HandleType::kRenderBuffer;
  }
  FML_UNREACHABLE();
}

std::shared_ptr<TextureGLES> TextureGLES::WrapFBO(
    std::shared_ptr<ReactorGLES> reactor,
    TextureDescriptor desc,
    GLuint fbo) {
  auto texture = std::shared_ptr<TextureGLES>(
      new TextureGLES(std::move(reactor), desc, fbo, std::nullopt));
  if (!texture->IsValid()) {
    return nullptr;
  }
  return texture;
}

std::shared_ptr<TextureGLES> TextureGLES::WrapTexture(
    std::shared_ptr<ReactorGLES> reactor,
    TextureDescriptor desc,
    HandleGLES external_handle) {
  if (external_handle.IsDead()) {
    VALIDATION_LOG << "Cannot wrap a dead handle.";
    return nullptr;
  }
  if (external_handle.GetType() != HandleType::kTexture) {
    VALIDATION_LOG << "Cannot wrap a non-texture handle.";
    return nullptr;
  }
  auto texture = std::shared_ptr<TextureGLES>(
      new TextureGLES(std::move(reactor), desc, std::nullopt, external_handle));
  if (!texture->IsValid()) {
    return nullptr;
  }
  return texture;
}

std::shared_ptr<TextureGLES> TextureGLES::CreatePlaceholder(
    std::shared_ptr<ReactorGLES> reactor,
    TextureDescriptor desc) {
  return TextureGLES::WrapFBO(std::move(reactor), desc, 0u);
}

TextureGLES::TextureGLES(std::shared_ptr<ReactorGLES> reactor,
                         TextureDescriptor desc)
    : TextureGLES(std::move(reactor),  //
                  desc,                //
                  std::nullopt,        //
                  std::nullopt         //
      ) {}

TextureGLES::TextureGLES(std::shared_ptr<ReactorGLES> reactor,
                         TextureDescriptor desc,
                         std::optional<GLuint> fbo,
                         std::optional<HandleGLES> external_handle)
    : Texture(desc),
      reactor_(std::move(reactor)),
      type_(GetTextureTypeFromDescriptor(GetTextureDescriptor())),
      handle_(external_handle.has_value()
                  ? external_handle.value()
                  : reactor_->CreateUntrackedHandle(ToHandleType(type_))),
      is_wrapped_(fbo.has_value() || external_handle.has_value()),
      wrapped_fbo_(fbo) {
  // Ensure the texture descriptor itself is valid.
  if (!GetTextureDescriptor().IsValid()) {
    VALIDATION_LOG << "Invalid texture descriptor.";
    return;
  }
  // Ensure the texture doesn't exceed device capabilities.
  const auto tex_size = GetTextureDescriptor().size;
  const auto max_size =
      reactor_->GetProcTable().GetCapabilities()->max_texture_size;
  if (tex_size.Max(max_size) != max_size) {
    VALIDATION_LOG << "Texture of size " << tex_size
                   << " would exceed max supported size of " << max_size << ".";
    return;
  }

  is_valid_ = true;
}

// |Texture|
TextureGLES::~TextureGLES() {
  reactor_->CollectHandle(handle_);
  if (cached_fbo_ != GL_NONE) {
    reactor_->GetProcTable().DeleteFramebuffers(1, &cached_fbo_);
  }
}

// |Texture|
bool TextureGLES::IsValid() const {
  return is_valid_;
}

// |Texture|
void TextureGLES::SetLabel(std::string_view label) {
#ifdef IMPELLER_DEBUG
  reactor_->SetDebugLabel(handle_, label);
#endif  // IMPELLER_DEBUG
}

// |Texture|
void TextureGLES::SetLabel(std::string_view label, std::string_view trailing) {
#ifdef IMPELLER_DEBUG
  if (reactor_->CanSetDebugLabels()) {
    reactor_->SetDebugLabel(handle_,
                            SPrintF("%s %s", label.data(), trailing.data()));
  }
#endif  // IMPELLER_DEBUG
}

// |Texture|
bool TextureGLES::OnSetContents(const uint8_t* contents,
                                size_t length,
                                size_t slice) {
  return OnSetContents(CreateMappingWithCopy(contents, Bytes{length}), slice);
}

// |Texture|
bool TextureGLES::OnSetContents(std::shared_ptr<const fml::Mapping> mapping,
                                size_t slice) {
  if (!mapping) {
    return false;
  }

  if (mapping->GetSize() == 0u) {
    return true;
  }

  if (mapping->GetMapping() == nullptr) {
    return false;
  }

  if (GetType() != Type::kTexture) {
    VALIDATION_LOG << "Incorrect texture usage flags for setting contents on "
                      "this texture object.";
    return false;
  }

  if (is_wrapped_) {
    VALIDATION_LOG << "Cannot set the contents of a wrapped texture.";
    return false;
  }

  const auto& tex_descriptor = GetTextureDescriptor();

  if (tex_descriptor.size.IsEmpty()) {
    return true;
  }

  if (!tex_descriptor.IsValid() ||
      mapping->GetSize() < tex_descriptor.GetByteSizeOfBaseMipLevel()) {
    return false;
  }

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

  auto data = std::make_shared<TexImage2DData>(tex_descriptor.format,
                                               std::move(mapping));
  if (!data || !data->IsValid()) {
    VALIDATION_LOG << "Invalid texture format.";
    return false;
  }

  ReactorGLES::Operation texture_upload = [handle = handle_,            //
                                           data,                        //
                                           size = tex_descriptor.size,  //
                                           texture_type,                //
                                           texture_target               //
  ](const auto& reactor) {
    auto gl_handle = reactor.GetGLHandle(handle);
    if (!gl_handle.has_value()) {
      VALIDATION_LOG
          << "Texture was collected before it could be uploaded to the GPU.";
      return;
    }
    const auto& gl = reactor.GetProcTable();
    gl.BindTexture(texture_type, gl_handle.value());
    const GLvoid* tex_data = nullptr;
    if (data->data) {
      tex_data = data->data->GetMapping();
    }

    {
      TRACE_EVENT1("impeller", "TexImage2DUpload", "Bytes",
                   std::to_string(data->data->GetSize()).c_str());
      gl.PixelStorei(GL_UNPACK_ALIGNMENT, 1);
      gl.TexImage2D(texture_target,         // target
                    0u,                     // LOD level
                    data->internal_format,  // internal format
                    size.width,             // width
                    size.height,            // height
                    0u,                     // border
                    data->external_format,  // external format
                    data->type,             // type
                    tex_data                // data
      );
    }
  };

  slices_initialized_ = reactor_->AddOperation(texture_upload);
  return slices_initialized_[0];
}

// |Texture|
ISize TextureGLES::GetSize() const {
  return GetTextureDescriptor().size;
}

static std::optional<GLenum> ToRenderBufferFormat(PixelFormat format) {
  switch (format) {
    case PixelFormat::kB8G8R8A8UNormInt:
    case PixelFormat::kR8G8B8A8UNormInt:
      return GL_RGBA4;
    case PixelFormat::kR32G32B32A32Float:
      return GL_RGBA32F;
    case PixelFormat::kR16G16B16A16Float:
      return GL_RGBA16F;
    case PixelFormat::kS8UInt:
      return GL_STENCIL_INDEX8;
    case PixelFormat::kD24UnormS8Uint:
      return GL_DEPTH24_STENCIL8;
    case PixelFormat::kD32FloatS8UInt:
      return GL_DEPTH32F_STENCIL8;
    case PixelFormat::kUnknown:
    case PixelFormat::kA8UNormInt:
    case PixelFormat::kR8UNormInt:
    case PixelFormat::kR8G8UNormInt:
    case PixelFormat::kR8G8B8A8UNormIntSRGB:
    case PixelFormat::kB8G8R8A8UNormIntSRGB:
    case PixelFormat::kB10G10R10XRSRGB:
    case PixelFormat::kB10G10R10XR:
    case PixelFormat::kB10G10R10A10XR:
      return std::nullopt;
  }
  FML_UNREACHABLE();
}

void TextureGLES::InitializeContentsIfNecessary() const {
  if (!IsValid() || slices_initialized_[0]) {
    return;
  }
  slices_initialized_[0] = true;

  if (is_wrapped_) {
    return;
  }

  auto size = GetSize();

  if (size.IsEmpty()) {
    return;
  }

  const auto& gl = reactor_->GetProcTable();
  std::optional<GLuint> handle = reactor_->GetGLHandle(handle_);
  if (!handle.has_value()) {
    VALIDATION_LOG << "Could not initialize the contents of texture.";
    return;
  }

  switch (type_) {
    case Type::kTexture:
    case Type::kTextureMultisampled: {
      TexImage2DData tex_data(GetTextureDescriptor().format);
      if (!tex_data.IsValid()) {
        VALIDATION_LOG << "Invalid format for texture image.";
        return;
      }
      gl.BindTexture(GL_TEXTURE_2D, handle.value());
      {
        TRACE_EVENT0("impeller", "TexImage2DInitialization");
        gl.TexImage2D(GL_TEXTURE_2D,  // target
                      0u,             // LOD level (base mip level size checked)
                      tex_data.internal_format,  // internal format
                      size.width,                // width
                      size.height,               // height
                      0u,                        // border
                      tex_data.external_format,  // format
                      tex_data.type,             // type
                      nullptr                    // data
        );
      }
    } break;
    case Type::kRenderBuffer:
    case Type::kRenderBufferMultisampled: {
      auto render_buffer_format =
          ToRenderBufferFormat(GetTextureDescriptor().format);
      if (!render_buffer_format.has_value()) {
        VALIDATION_LOG << "Invalid format for render-buffer image.";
        return;
      }
      gl.BindRenderbuffer(GL_RENDERBUFFER, handle.value());
      {
        TRACE_EVENT0("impeller", "RenderBufferStorageInitialization");
        if (type_ == Type::kRenderBufferMultisampled) {
          gl.RenderbufferStorageMultisampleEXT(
              GL_RENDERBUFFER,               // target
              4,                             // samples
              render_buffer_format.value(),  // internal format
              size.width,                    // width
              size.height                    // height
          );
        } else {
          gl.RenderbufferStorage(
              GL_RENDERBUFFER,               // target
              render_buffer_format.value(),  // internal format
              size.width,                    // width
              size.height                    // height
          );
        }
      }
    } break;
  }
}

std::optional<GLuint> TextureGLES::GetGLHandle() const {
  if (!IsValid()) {
    return std::nullopt;
  }
  return reactor_->GetGLHandle(handle_);
}

bool TextureGLES::Bind() const {
  auto handle = GetGLHandle();
  if (!handle.has_value()) {
    return false;
  }
  const auto& gl = reactor_->GetProcTable();

  if (fence_.has_value()) {
    std::optional<GLsync> fence = reactor_->GetGLFence(fence_.value());
    if (fence.has_value()) {
      gl.WaitSync(fence.value(), 0, GL_TIMEOUT_IGNORED);
    }
    reactor_->CollectHandle(fence_.value());
    fence_ = std::nullopt;
  }

  switch (type_) {
    case Type::kTexture:
    case Type::kTextureMultisampled: {
      const auto target = ToTextureTarget(GetTextureDescriptor().type);
      if (!target.has_value()) {
        VALIDATION_LOG << "Could not bind texture of this type.";
        return false;
      }
      gl.BindTexture(target.value(), handle.value());
    } break;
    case Type::kRenderBuffer:
    case Type::kRenderBufferMultisampled:
      gl.BindRenderbuffer(GL_RENDERBUFFER, handle.value());
      break;
  }
  InitializeContentsIfNecessary();
  return true;
}

void TextureGLES::MarkContentsInitialized() {
  for (size_t i = 0; i < slices_initialized_.size(); i++) {
    slices_initialized_[i] = true;
  }
}

void TextureGLES::MarkSliceInitialized(size_t slice) const {
  slices_initialized_[slice] = true;
}

bool TextureGLES::IsSliceInitialized(size_t slice) const {
  return slices_initialized_[slice];
}

bool TextureGLES::GenerateMipmap() {
  if (!IsValid()) {
    return false;
  }

  auto type = GetTextureDescriptor().type;
  switch (type) {
    case TextureType::kTexture2D:
      break;
    case TextureType::kTexture2DMultisample:
      VALIDATION_LOG << "Generating mipmaps for multisample textures is not "
                        "supported in the GLES backend.";
      return false;
    case TextureType::kTextureCube:
      break;
    case TextureType::kTextureExternalOES:
      break;
  }

  if (!Bind()) {
    return false;
  }

  auto handle = GetGLHandle();
  if (!handle.has_value()) {
    return false;
  }

  const auto& gl = reactor_->GetProcTable();
  gl.GenerateMipmap(ToTextureType(type));
  mipmap_generated_ = true;
  return true;
}

TextureGLES::Type TextureGLES::GetType() const {
  return type_;
}

static GLenum ToAttachmentType(TextureGLES::AttachmentType point) {
  switch (point) {
    case TextureGLES::AttachmentType::kColor0:
      return GL_COLOR_ATTACHMENT0;
    case TextureGLES::AttachmentType::kDepth:
      return GL_DEPTH_ATTACHMENT;
    case TextureGLES::AttachmentType::kStencil:
      return GL_STENCIL_ATTACHMENT;
  }
}

bool TextureGLES::SetAsFramebufferAttachment(
    GLenum target,
    AttachmentType attachment_type) const {
  if (!IsValid()) {
    return false;
  }
  InitializeContentsIfNecessary();
  auto handle = GetGLHandle();
  if (!handle.has_value()) {
    return false;
  }
  const auto& gl = reactor_->GetProcTable();

  switch (type_) {
    case Type::kTexture:
      gl.FramebufferTexture2D(target,                             // target
                              ToAttachmentType(attachment_type),  // attachment
                              GL_TEXTURE_2D,                      // textarget
                              handle.value(),                     // texture
                              0                                   // level
      );
      break;
    case Type::kTextureMultisampled:
      gl.FramebufferTexture2DMultisampleEXT(
          target,                             // target
          ToAttachmentType(attachment_type),  // attachment
          GL_TEXTURE_2D,                      // textarget
          handle.value(),                     // texture
          0,                                  // level
          4                                   // samples
      );
      break;
    case Type::kRenderBuffer:
    case Type::kRenderBufferMultisampled:
      gl.FramebufferRenderbuffer(
          target,                             // target
          ToAttachmentType(attachment_type),  // attachment
          GL_RENDERBUFFER,                    // render-buffer target
          handle.value()                      // render-buffer
      );
      break;
  }

  return true;
}

// |Texture|
Scalar TextureGLES::GetYCoordScale() const {
  switch (GetCoordinateSystem()) {
    case TextureCoordinateSystem::kUploadFromHost:
      return 1.0;
    case TextureCoordinateSystem::kRenderToTexture:
      return -1.0;
  }
  FML_UNREACHABLE();
}

bool TextureGLES::IsWrapped() const {
  return is_wrapped_;
}

std::optional<GLuint> TextureGLES::GetFBO() const {
  return wrapped_fbo_;
}

void TextureGLES::SetFence(HandleGLES fence) {
  FML_DCHECK(!fence_.has_value());
  fence_ = fence;
}

// Visible for testing.
std::optional<HandleGLES> TextureGLES::GetSyncFence() const {
  return fence_;
}

void TextureGLES::SetCachedFBO(GLuint fbo) {
  cached_fbo_ = fbo;
}

GLuint TextureGLES::GetCachedFBO() const {
  return cached_fbo_;
}

}  // namespace impeller
