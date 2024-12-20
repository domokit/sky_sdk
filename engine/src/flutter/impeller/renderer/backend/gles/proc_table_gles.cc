// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/proc_table_gles.h"

#include <sstream>

#include "impeller/base/allocation.h"
#include "impeller/base/comparable.h"
#include "impeller/base/strings.h"
#include "impeller/base/validation.h"
#include "impeller/renderer/backend/gles/capabilities_gles.h"
#include "impeller/renderer/capabilities.h"

namespace impeller {

const char* GLErrorToString(GLenum value) {
  switch (value) {
    case GL_NO_ERROR:
      return "GL_NO_ERROR";
    case GL_INVALID_ENUM:
      return "GL_INVALID_ENUM";
    case GL_INVALID_VALUE:
      return "GL_INVALID_VALUE";
    case GL_INVALID_OPERATION:
      return "GL_INVALID_OPERATION";
    case GL_INVALID_FRAMEBUFFER_OPERATION:
      return "GL_INVALID_FRAMEBUFFER_OPERATION";
    case GL_FRAMEBUFFER_COMPLETE:
      return "GL_FRAMEBUFFER_COMPLETE";
    case GL_OUT_OF_MEMORY:
      return "GL_OUT_OF_MEMORY";
  }
  return "Unknown.";
}

bool GLErrorIsFatal(GLenum value) {
  switch (value) {
    case GL_NO_ERROR:
      return false;
    case GL_INVALID_ENUM:
    case GL_INVALID_VALUE:
    case GL_INVALID_OPERATION:
    case GL_INVALID_FRAMEBUFFER_OPERATION:
    case GL_OUT_OF_MEMORY:
      return true;
  }
  return false;
}

ProcTableGLES::Resolver WrappedResolver(
    const ProcTableGLES::Resolver& resolver) {
  return [resolver](const char* function_name) -> void* {
    auto resolved = resolver(function_name);
    if (resolved) {
      return resolved;
    }
    // If there are certain known suffixes (usually for extensions), strip them
    // out and try to resolve the same proc addresses again.
    auto function = std::string{function_name};
    if (function.find("KHR", function.size() - 3) != std::string::npos) {
      auto truncated = function.substr(0u, function.size() - 3);
      return resolver(truncated.c_str());
    }
    if (function.find("EXT", function.size() - 3) != std::string::npos) {
      auto truncated = function.substr(0u, function.size() - 3);
      return resolver(truncated.c_str());
    }
    return nullptr;
  };
}

ProcTableGLES::ProcTableGLES(  // NOLINT(google-readability-function-size)
    Resolver resolver) {
  // The reason this constructor has anywhere near enough code to tip off
  // `google-readability-function-size` is the proc macros, so ignore the lint.

  if (!resolver) {
    return;
  }

  resolver = WrappedResolver(resolver);

  auto error_fn = reinterpret_cast<PFNGLGETERRORPROC>(resolver("glGetError"));
  if (!error_fn) {
    VALIDATION_LOG << "Could not resolve " << "glGetError";
    return;
  }

#define IMPELLER_PROC(proc_ivar)                                \
  if (auto fn_ptr = resolver(proc_ivar.name)) {                 \
    proc_ivar.function =                                        \
        reinterpret_cast<decltype(proc_ivar.function)>(fn_ptr); \
    proc_ivar.error_fn = error_fn;                              \
  } else {                                                      \
    VALIDATION_LOG << "Could not resolve " << proc_ivar.name;   \
    return;                                                     \
  }

  FOR_EACH_IMPELLER_PROC(IMPELLER_PROC);

  description_ = std::make_unique<DescriptionGLES>(*this);

  if (!description_->IsValid()) {
    return;
  }

  if (description_->IsES()) {
    FOR_EACH_IMPELLER_ES_ONLY_PROC(IMPELLER_PROC);
  } else {
    FOR_EACH_IMPELLER_DESKTOP_ONLY_PROC(IMPELLER_PROC);
  }

#undef IMPELLER_PROC

#define IMPELLER_PROC(proc_ivar)                                \
  if (auto fn_ptr = resolver(proc_ivar.name)) {                 \
    proc_ivar.function =                                        \
        reinterpret_cast<decltype(proc_ivar.function)>(fn_ptr); \
    proc_ivar.error_fn = error_fn;                              \
  }

  if (description_->GetGlVersion().IsAtLeast(Version(3))) {
    FOR_EACH_IMPELLER_GLES3_PROC(IMPELLER_PROC);
  }

  FOR_EACH_IMPELLER_EXT_PROC(IMPELLER_PROC);

#undef IMPELLER_PROC

  if (!description_->HasDebugExtension()) {
    PushDebugGroupKHR.Reset();
    PopDebugGroupKHR.Reset();
    ObjectLabelKHR.Reset();
  } else {
    GetIntegerv(GL_MAX_LABEL_LENGTH_KHR, &debug_label_max_length_);
  }

  if (!description_->HasExtension("GL_EXT_discard_framebuffer")) {
    DiscardFramebufferEXT.Reset();
  }

  capabilities_ = std::make_shared<CapabilitiesGLES>(*this);

  is_valid_ = true;
}

ProcTableGLES::~ProcTableGLES() = default;

bool ProcTableGLES::IsValid() const {
  return is_valid_;
}

void ProcTableGLES::ShaderSourceMapping(
    GLuint shader,
    const fml::Mapping& mapping,
    const std::vector<Scalar>& defines) const {
  if (defines.empty()) {
    const GLchar* sources[] = {
        reinterpret_cast<const GLchar*>(mapping.GetMapping())};
    const GLint lengths[] = {static_cast<GLint>(mapping.GetSize())};
    ShaderSource(shader, 1u, sources, lengths);
    return;
  }
  const auto& shader_source = ComputeShaderWithDefines(mapping, defines);
  if (!shader_source.has_value()) {
    VALIDATION_LOG << "Failed to append constant data to shader";
    return;
  }

  const GLchar* sources[] = {
      reinterpret_cast<const GLchar*>(shader_source->c_str())};
  const GLint lengths[] = {static_cast<GLint>(shader_source->size())};
  ShaderSource(shader, 1u, sources, lengths);
}

// Visible For testing.
std::optional<std::string> ProcTableGLES::ComputeShaderWithDefines(
    const fml::Mapping& mapping,
    const std::vector<Scalar>& defines) const {
  std::string shader_source = std::string{
      reinterpret_cast<const char*>(mapping.GetMapping()), mapping.GetSize()};

  // Look for the first newline after the '#version' header, which impellerc
  // will always emit as the first line of a compiled shader.
  size_t index = shader_source.find('\n');
  if (index == std::string::npos) {
    VALIDATION_LOG << "Failed to append constant data to shader";
    return std::nullopt;
  }

  std::stringstream ss;
  ss << std::fixed;
  for (auto i = 0u; i < defines.size(); i++) {
    ss << "#define SPIRV_CROSS_CONSTANT_ID_" << i << " " << defines[i] << '\n';
  }
  auto define_string = ss.str();
  shader_source.insert(index + 1, define_string);
  return shader_source;
}

const DescriptionGLES* ProcTableGLES::GetDescription() const {
  return description_.get();
}

const std::shared_ptr<const CapabilitiesGLES>& ProcTableGLES::GetCapabilities()
    const {
  return capabilities_;
}

static const char* FramebufferStatusToString(GLenum status) {
  switch (status) {
    case GL_FRAMEBUFFER_COMPLETE:
      return "GL_FRAMEBUFFER_COMPLETE";
    case GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT:
      return "GL_FRAMEBUFFER_INCOMPLETE_ATTACHMENT";
#if GL_ES_VERSION_2_0
    case GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS:
      return "GL_FRAMEBUFFER_INCOMPLETE_DIMENSIONS";
#endif
    case GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT:
      return "GL_FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT";
    case GL_FRAMEBUFFER_UNSUPPORTED:
      return "GL_FRAMEBUFFER_UNSUPPORTED";
    case GL_INVALID_ENUM:
      return "GL_INVALID_ENUM";
  }

  return "Unknown FBO Error Status";
}

static const char* AttachmentTypeString(GLint type) {
  switch (type) {
    case GL_RENDERBUFFER:
      return "GL_RENDERBUFFER";
    case GL_TEXTURE:
      return "GL_TEXTURE";
    case GL_NONE:
      return "GL_NONE";
  }

  return "Unknown Type";
}

static std::string DescribeFramebufferAttachment(const ProcTableGLES& gl,
                                                 GLenum attachment) {
  GLint type = GL_NONE;
  gl.GetFramebufferAttachmentParameteriv(
      GL_FRAMEBUFFER,                         // target
      attachment,                             // attachment
      GL_FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE,  // parameter name
      &type                                   // parameter
  );

  if (type != GL_NONE) {
    GLint object = GL_NONE;
    gl.GetFramebufferAttachmentParameteriv(
        GL_FRAMEBUFFER,                         // target
        attachment,                             // attachment
        GL_FRAMEBUFFER_ATTACHMENT_OBJECT_NAME,  // parameter name
        &object                                 // parameter
    );
    std::stringstream stream;
    stream << AttachmentTypeString(type) << "(" << object << ")";
    return stream.str();
  }

  return "No Attachment";
}

std::string ProcTableGLES::DescribeCurrentFramebuffer() const {
  GLint framebuffer = GL_NONE;
  GetIntegerv(GL_FRAMEBUFFER_BINDING, &framebuffer);
  if (framebuffer == GL_NONE) {
    return "The default framebuffer (FBO0) was bound.";
  }
  if (IsFramebuffer(framebuffer) == GL_FALSE) {
    return SPrintF("The framebuffer binding (%d) was not a valid framebuffer.",
                   framebuffer);
  }

  GLenum status = CheckFramebufferStatus(GL_FRAMEBUFFER);
  std::stringstream stream;
  stream << "FBO "
         << ((framebuffer == GL_NONE) ? "(Default)"
                                      : std::to_string(framebuffer))
         << ": " << FramebufferStatusToString(status) << std::endl;
  if (IsCurrentFramebufferComplete()) {
    stream << "Framebuffer is complete." << std::endl;
  } else {
    stream << "Framebuffer is incomplete." << std::endl;
  }
  stream << "Description: " << std::endl;
  stream << "Color Attachment: "
         << DescribeFramebufferAttachment(*this, GL_COLOR_ATTACHMENT0)
         << std::endl;
  stream << "Depth Attachment: "
         << DescribeFramebufferAttachment(*this, GL_DEPTH_ATTACHMENT)
         << std::endl;
  stream << "Stencil Attachment: "
         << DescribeFramebufferAttachment(*this, GL_STENCIL_ATTACHMENT)
         << std::endl;
  return stream.str();
}

bool ProcTableGLES::IsCurrentFramebufferComplete() const {
  GLint framebuffer = GL_NONE;
  GetIntegerv(GL_FRAMEBUFFER_BINDING, &framebuffer);
  if (IsFramebuffer(framebuffer) == GL_FALSE) {
    // The default framebuffer is always complete.
    return true;
  }
  GLenum status = CheckFramebufferStatus(GL_FRAMEBUFFER);
  return status == GL_FRAMEBUFFER_COMPLETE;
}

static std::optional<GLenum> ToDebugIdentifier(DebugResourceType type) {
  switch (type) {
    case DebugResourceType::kTexture:
      return GL_TEXTURE;
    case DebugResourceType::kBuffer:
      return GL_BUFFER_KHR;
    case DebugResourceType::kProgram:
      return GL_PROGRAM_KHR;
    case DebugResourceType::kShader:
      return GL_SHADER_KHR;
    case DebugResourceType::kRenderBuffer:
      return GL_RENDERBUFFER;
    case DebugResourceType::kFrameBuffer:
      return GL_FRAMEBUFFER;
    case DebugResourceType::kFence:
      return GL_SYNC_FENCE;
  }
  FML_UNREACHABLE();
}

static bool ResourceIsLive(const ProcTableGLES& gl,
                           DebugResourceType type,
                           GLint name) {
  switch (type) {
    case DebugResourceType::kTexture:
      return gl.IsTexture(name);
    case DebugResourceType::kBuffer:
      return gl.IsBuffer(name);
    case DebugResourceType::kProgram:
      return gl.IsProgram(name);
    case DebugResourceType::kShader:
      return gl.IsShader(name);
    case DebugResourceType::kRenderBuffer:
      return gl.IsRenderbuffer(name);
    case DebugResourceType::kFrameBuffer:
      return gl.IsFramebuffer(name);
    case DebugResourceType::kFence:
      return true;
  }
  FML_UNREACHABLE();
}

bool ProcTableGLES::SupportsDebugLabels() const {
  if (debug_label_max_length_ <= 0) {
    return false;
  }
  if (!ObjectLabelKHR.IsAvailable()) {
    return false;
  }
  return true;
}

bool ProcTableGLES::SetDebugLabel(DebugResourceType type,
                                  GLint name,
                                  std::string_view label) const {
  if (!SupportsDebugLabels()) {
    return true;
  }
  if (!ResourceIsLive(*this, type, name)) {
    return false;
  }
  const auto identifier = ToDebugIdentifier(type);
  const auto label_length =
      std::min<GLsizei>(debug_label_max_length_ - 1, label.size());
  if (!identifier.has_value()) {
    return true;
  }
  ObjectLabelKHR(identifier.value(),  // identifier
                 name,                // name
                 label_length,        // length
                 label.data()         // label
  );
  return true;
}

void ProcTableGLES::PushDebugGroup(const std::string& label) const {
#ifdef IMPELLER_DEBUG
  if (debug_label_max_length_ <= 0) {
    return;
  }

  UniqueID id;
  const auto label_length =
      std::min<GLsizei>(debug_label_max_length_ - 1, label.size());
  PushDebugGroupKHR(GL_DEBUG_SOURCE_APPLICATION_KHR,  // source
                    static_cast<GLuint>(id.id),       // id
                    label_length,                     // length
                    label.data()                      // message
  );
#endif  // IMPELLER_DEBUG
}

void ProcTableGLES::PopDebugGroup() const {
#ifdef IMPELLER_DEBUG
  if (debug_label_max_length_ <= 0) {
    return;
  }

  PopDebugGroupKHR();
#endif  // IMPELLER_DEBUG
}

std::string ProcTableGLES::GetProgramInfoLogString(GLuint program) const {
  GLint length = 0;
  GetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
  if (length <= 0) {
    return "";
  }

  length = std::min<GLint>(length, 1024);
  Allocation allocation;
  if (!allocation.Truncate(Bytes{length}, false)) {
    return "";
  }
  GetProgramInfoLog(program,  // program
                    length,   // max length
                    &length,  // length written (excluding NULL terminator)
                    reinterpret_cast<GLchar*>(allocation.GetBuffer())  // buffer
  );
  if (length <= 0) {
    return "";
  }
  return std::string{reinterpret_cast<const char*>(allocation.GetBuffer()),
                     static_cast<size_t>(length)};
}

}  // namespace impeller
