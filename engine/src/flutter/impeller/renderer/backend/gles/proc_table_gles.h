// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_PROC_TABLE_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_PROC_TABLE_GLES_H_

#include <functional>
#include <string>

#include "flutter/fml/logging.h"
#include "flutter/fml/mapping.h"
#include "impeller/renderer/backend/gles/capabilities_gles.h"
#include "impeller/renderer/backend/gles/description_gles.h"
#include "impeller/renderer/backend/gles/gles.h"

namespace impeller {

const char* GLErrorToString(GLenum value);
bool GLErrorIsFatal(GLenum value);

struct AutoErrorCheck {
  const PFNGLGETERRORPROC error_fn;

  // TODO(135922) Change to string_view.
  const char* name;

  AutoErrorCheck(PFNGLGETERRORPROC error, const char* name)
      : error_fn(error), name(name) {}

  ~AutoErrorCheck() {
    if (error_fn) {
      auto error = error_fn();
      if (error == GL_NO_ERROR) {
        return;
      }
      if (GLErrorIsFatal(error)) {
        FML_LOG(FATAL) << "Fatal GL Error " << GLErrorToString(error) << "("
                       << error << ")" << " encountered on call to " << name;
      } else {
        FML_LOG(ERROR) << "GL Error " << GLErrorToString(error) << "(" << error
                       << ")" << " encountered on call to " << name;
      }
    }
  }
};

template <class Type>
void BuildGLArgumentsStream(std::stringstream& stream, Type arg) {
  stream << arg;
}

constexpr void BuildGLArgumentsStream(std::stringstream& stream) {}

template <class Type, class... Rest>
void BuildGLArgumentsStream(std::stringstream& stream,
                            Type arg,
                            Rest... other_args) {
  BuildGLArgumentsStream(stream, arg);
  stream << ", ";
  BuildGLArgumentsStream(stream, other_args...);
}

template <class... Type>
[[nodiscard]] std::string BuildGLArguments(Type... args) {
  std::stringstream stream;
  stream << "(";
  BuildGLArgumentsStream(stream, args...);
  stream << ")";
  return stream.str();
}

template <class T>
struct GLProc {
  using GLFunctionType = T;

  //----------------------------------------------------------------------------
  /// The name of the GL function.
  ///
  // TODO(135922) Change to string_view.
  const char* name = nullptr;

  //----------------------------------------------------------------------------
  /// The pointer to the GL function.
  ///
  GLFunctionType* function = nullptr;

  //----------------------------------------------------------------------------
  /// An optional error function. If present, all calls will be followed by an
  /// error check.
  ///
  PFNGLGETERRORPROC error_fn = nullptr;

  //----------------------------------------------------------------------------
  /// Whether the OpenGL call and its arguments should be logged.
  ///
  /// Only works in IMPELLER_DEBUG and for environments where traditional
  /// tracing is hard. Expect log spam and only use during development.
  ///
  bool log_calls = false;

  //----------------------------------------------------------------------------
  /// @brief      Call the GL function with the appropriate parameters. Lookup
  ///             the documentation for the GL function being called to
  ///             understand the arguments and return types. The arguments
  ///             types must match and will be type checked.
  ///
  template <class... Args>
  auto operator()(Args&&... args) const {
#if defined(IMPELLER_DEBUG) && !defined(NDEBUG)
    AutoErrorCheck error(error_fn, name);
    // We check for the existence of extensions, and reset the function pointer
    // but it's still called unconditionally below, and will segfault. This
    // validation log will at least give us a hint as to what's going on.
    FML_CHECK(IsAvailable()) << "GL function " << name << " is not available. "
                             << "This is likely due to a missing extension.";
    if (log_calls) {
      FML_LOG(IMPORTANT) << name
                         << BuildGLArguments(std::forward<Args>(args)...);
    }
#endif  // defined(IMPELLER_DEBUG) && !defined(NDEBUG)
    return function(std::forward<Args>(args)...);
  }

  constexpr bool IsAvailable() const { return function != nullptr; }

  void Reset() {
    function = nullptr;
    error_fn = nullptr;
  }
};

#define FOR_EACH_IMPELLER_PROC(PROC)         \
  PROC(ActiveTexture);                       \
  PROC(AttachShader);                        \
  PROC(BindAttribLocation);                  \
  PROC(BindBuffer);                          \
  PROC(BindFramebuffer);                     \
  PROC(BindRenderbuffer);                    \
  PROC(BindTexture);                         \
  PROC(BindVertexArray);                     \
  PROC(BlendEquationSeparate);               \
  PROC(BlendFuncSeparate);                   \
  PROC(BufferData);                          \
  PROC(BufferSubData);                       \
  PROC(CheckFramebufferStatus);              \
  PROC(Clear);                               \
  PROC(ClearColor);                          \
  PROC(ClearStencil);                        \
  PROC(ColorMask);                           \
  PROC(CompileShader);                       \
  PROC(CreateProgram);                       \
  PROC(CreateShader);                        \
  PROC(CullFace);                            \
  PROC(DeleteBuffers);                       \
  PROC(DeleteFramebuffers);                  \
  PROC(DeleteProgram);                       \
  PROC(DeleteRenderbuffers);                 \
  PROC(DeleteShader);                        \
  PROC(DeleteTextures);                      \
  PROC(DeleteVertexArrays);                  \
  PROC(DepthFunc);                           \
  PROC(DepthMask);                           \
  PROC(DetachShader);                        \
  PROC(Disable);                             \
  PROC(DisableVertexAttribArray);            \
  PROC(DrawArrays);                          \
  PROC(DrawElements);                        \
  PROC(Enable);                              \
  PROC(EnableVertexAttribArray);             \
  PROC(Finish);                              \
  PROC(Flush);                               \
  PROC(FramebufferRenderbuffer);             \
  PROC(FramebufferTexture2D);                \
  PROC(FrontFace);                           \
  PROC(GenBuffers);                          \
  PROC(GenerateMipmap);                      \
  PROC(GenFramebuffers);                     \
  PROC(GenRenderbuffers);                    \
  PROC(GenTextures);                         \
  PROC(GenVertexArrays);                     \
  PROC(GetActiveUniform);                    \
  PROC(GetBooleanv);                         \
  PROC(GetFloatv);                           \
  PROC(GetFramebufferAttachmentParameteriv); \
  PROC(GetIntegerv);                         \
  PROC(GetProgramInfoLog);                   \
  PROC(GetProgramiv);                        \
  PROC(GetShaderInfoLog);                    \
  PROC(GetShaderiv);                         \
  PROC(GetString);                           \
  PROC(GetStringi);                          \
  PROC(GetUniformLocation);                  \
  PROC(IsBuffer);                            \
  PROC(IsFramebuffer);                       \
  PROC(IsProgram);                           \
  PROC(IsRenderbuffer);                      \
  PROC(IsShader);                            \
  PROC(IsTexture);                           \
  PROC(LinkProgram);                         \
  PROC(PixelStorei);                         \
  PROC(RenderbufferStorage);                 \
  PROC(Scissor);                             \
  PROC(ShaderBinary);                        \
  PROC(ShaderSource);                        \
  PROC(StencilFuncSeparate);                 \
  PROC(StencilMaskSeparate);                 \
  PROC(StencilOpSeparate);                   \
  PROC(TexImage2D);                          \
  PROC(TexSubImage2D);                       \
  PROC(TexParameteri);                       \
  PROC(TexParameterfv);                      \
  PROC(Uniform1fv);                          \
  PROC(Uniform1i);                           \
  PROC(Uniform2fv);                          \
  PROC(Uniform3fv);                          \
  PROC(Uniform4fv);                          \
  PROC(UniformMatrix4fv);                    \
  PROC(UseProgram);                          \
  PROC(VertexAttribPointer);                 \
  PROC(Viewport);                            \
  PROC(GetShaderSource);                     \
  PROC(ReadPixels);

// Calls specific to OpenGLES.
void(glClearDepthf)(GLfloat depth);
void(glDepthRangef)(GLfloat n, GLfloat f);

#define FOR_EACH_IMPELLER_ES_ONLY_PROC(PROC) \
  PROC(ClearDepthf);                         \
  PROC(DepthRangef);

// Calls specific to desktop GL.
void(glClearDepth)(GLdouble depth);
void(glDepthRange)(GLdouble n, GLdouble f);

#define FOR_EACH_IMPELLER_DESKTOP_ONLY_PROC(PROC) \
  PROC(ClearDepth);                               \
  PROC(DepthRange);

#define FOR_EACH_IMPELLER_GLES3_PROC(PROC) \
  PROC(FenceSync);                         \
  PROC(DeleteSync);                        \
  PROC(GetActiveUniformBlockiv);           \
  PROC(GetActiveUniformBlockName);         \
  PROC(GetUniformBlockIndex);              \
  PROC(UniformBlockBinding);               \
  PROC(BindBufferRange);                   \
  PROC(WaitSync);                          \
  PROC(BlitFramebuffer);

#define FOR_EACH_IMPELLER_EXT_PROC(PROC)    \
  PROC(DebugMessageControlKHR);             \
  PROC(DiscardFramebufferEXT);              \
  PROC(FramebufferTexture2DMultisampleEXT); \
  PROC(PushDebugGroupKHR);                  \
  PROC(PopDebugGroupKHR);                   \
  PROC(ObjectLabelKHR);                     \
  PROC(RenderbufferStorageMultisampleEXT);  \
  PROC(GenQueriesEXT);                      \
  PROC(DeleteQueriesEXT);                   \
  PROC(GetQueryObjectui64vEXT);             \
  PROC(BeginQueryEXT);                      \
  PROC(EndQueryEXT);                        \
  PROC(GetQueryObjectuivEXT);

enum class DebugResourceType {
  kTexture,
  kBuffer,
  kProgram,
  kShader,
  kRenderBuffer,
  kFrameBuffer,
  kFence,
};

class ProcTableGLES {
 public:
  using Resolver = std::function<void*(const char* function_name)>;
  explicit ProcTableGLES(Resolver resolver);
  ProcTableGLES(ProcTableGLES&& other) = default;

  ~ProcTableGLES();

#define IMPELLER_PROC(name) \
  GLProc<decltype(gl##name)> name = {"gl" #name, nullptr};

  FOR_EACH_IMPELLER_PROC(IMPELLER_PROC);
  FOR_EACH_IMPELLER_ES_ONLY_PROC(IMPELLER_PROC);
  FOR_EACH_IMPELLER_DESKTOP_ONLY_PROC(IMPELLER_PROC);
  FOR_EACH_IMPELLER_GLES3_PROC(IMPELLER_PROC);
  FOR_EACH_IMPELLER_EXT_PROC(IMPELLER_PROC);

#undef IMPELLER_PROC

  bool IsValid() const;

  /// @brief Set the source for the attached [shader].
  ///
  /// Optionally, [defines] may contain a string value that will be
  /// append to the shader source after the version marker. This can be used to
  /// support static specialization. For example, setting "#define Foo 1".
  void ShaderSourceMapping(GLuint shader,
                           const fml::Mapping& mapping,
                           const std::vector<Scalar>& defines = {}) const;

  const DescriptionGLES* GetDescription() const;

  const std::shared_ptr<const CapabilitiesGLES>& GetCapabilities() const;

  std::string DescribeCurrentFramebuffer() const;

  std::string GetProgramInfoLogString(GLuint program) const;

  bool IsCurrentFramebufferComplete() const;

  bool SupportsDebugLabels() const;

  bool SetDebugLabel(DebugResourceType type,
                     GLint name,
                     std::string_view label) const;

  void PushDebugGroup(const std::string& string) const;

  void PopDebugGroup() const;

  // Visible For testing.
  std::optional<std::string> ComputeShaderWithDefines(
      const fml::Mapping& mapping,
      const std::vector<Scalar>& defines) const;

 private:
  bool is_valid_ = false;
  std::unique_ptr<DescriptionGLES> description_;
  std::shared_ptr<const CapabilitiesGLES> capabilities_;
  GLint debug_label_max_length_ = 0;

  ProcTableGLES(const ProcTableGLES&) = delete;

  ProcTableGLES& operator=(const ProcTableGLES&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_PROC_TABLE_GLES_H_
