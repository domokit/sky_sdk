// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/toolkit/glvk/trampoline.h"

#include <array>

#include "flutter/fml/closure.h"
#include "flutter/fml/trace_event.h"
#include "impeller/base/timing.h"
#include "impeller/base/validation.h"
#include "impeller/geometry/point.h"
#include "impeller/renderer/backend/gles/formats_gles.h"
#include "impeller/toolkit/android/proc_table.h"
#include "impeller/toolkit/egl/image.h"

namespace impeller::glvk {

static GLuint kAttributeIndexPosition = 0u;
static GLuint kAttributeIndexTexCoord = 1u;

static constexpr const char* kVertShader = R"IMPELLER_SHADER(#version 100

precision mediump float;

attribute vec2 aPosition;
attribute vec2 aTexCoord;

varying vec2 vTexCoord;

void main() {
  gl_Position = vec4(aPosition, 0.0, 1.0);
  vTexCoord = aTexCoord;

}
)IMPELLER_SHADER";

static constexpr const char* kFragShader = R"IMPELLER_SHADER(#version 100

#extension GL_OES_EGL_image_external : require

precision mediump float;

uniform samplerExternalOES uTexture;
uniform mat4 uUVTransformation;

varying vec2 vTexCoord;

void main() {
  vec2 texture_coords = (uUVTransformation * vec4(vTexCoord, 0, 1)).xy;
  gl_FragColor = texture2D(uTexture, texture_coords);
}

)IMPELLER_SHADER";

Trampoline::Trampoline() {
  auto egl_display = std::make_unique<egl::Display>();
  if (!egl_display->IsValid()) {
    VALIDATION_LOG
        << "Could not create EGL display for external texture interop.";
    return;
  }

  egl::ConfigDescriptor egl_config_desc;
  egl_config_desc.api = egl::API::kOpenGLES2;
  egl_config_desc.samples = egl::Samples::kOne;
  egl_config_desc.color_format = egl::ColorFormat::kRGBA8888;
  egl_config_desc.stencil_bits = egl::StencilBits::kZero;
  egl_config_desc.depth_bits = egl::DepthBits::kZero;
  egl_config_desc.surface_type = egl::SurfaceType::kPBuffer;
  auto egl_config = egl_display->ChooseConfig(egl_config_desc);
  if (!egl_config) {
    VALIDATION_LOG
        << "Could not choose EGL config for external texture interop.";
    return;
  }

  auto egl_surface = egl_display->CreatePixelBufferSurface(*egl_config, 1u, 1u);
  auto egl_context = egl_display->CreateContext(*egl_config, nullptr);

  if (!egl_surface || !egl_context) {
    VALIDATION_LOG << "Could not create EGL surface and/or context for "
                      "external texture interop.";
    return;
  }

  // Make the context current so the proc addresses can be resolved.
  if (!egl_context->MakeCurrent(*egl_surface)) {
    VALIDATION_LOG << "Could not make the context current.";
    return;
  }

  auto gl = std::make_unique<ProcTable>(egl::CreateProcAddressResolver());

  if (!gl->IsValid()) {
    egl_context->ClearCurrent();
    VALIDATION_LOG << "Could not setup trampoline proc table.";
    return;
  }

  // Generate program object.
  auto vert_shader = gl->CreateShader(GL_VERTEX_SHADER);
  auto frag_shader = gl->CreateShader(GL_FRAGMENT_SHADER);

  GLint vert_shader_size = strlen(kVertShader);
  GLint frag_shader_size = strlen(kFragShader);

  gl->ShaderSource(vert_shader, 1u, &kVertShader, &vert_shader_size);
  gl->ShaderSource(frag_shader, 1u, &kFragShader, &frag_shader_size);

  gl->CompileShader(vert_shader);
  gl->CompileShader(frag_shader);

  GLint vert_status = GL_FALSE;
  GLint frag_status = GL_FALSE;

  gl->GetShaderiv(vert_shader, GL_COMPILE_STATUS, &vert_status);
  gl->GetShaderiv(frag_shader, GL_COMPILE_STATUS, &frag_status);

  FML_CHECK(vert_status == GL_TRUE);
  FML_CHECK(frag_status == GL_TRUE);

  program_ = gl->CreateProgram();
  gl->AttachShader(program_, vert_shader);
  gl->AttachShader(program_, frag_shader);

  gl->BindAttribLocation(program_, kAttributeIndexPosition, "aPosition");
  gl->BindAttribLocation(program_, kAttributeIndexTexCoord, "aTexCoord");

  gl->LinkProgram(program_);

  GLint link_status = GL_FALSE;
  gl->GetProgramiv(program_, GL_LINK_STATUS, &link_status);
  FML_CHECK(link_status == GL_TRUE);

  texture_uniform_location_ = gl->GetUniformLocation(program_, "uTexture");
  uv_transformation_location_ =
      gl->GetUniformLocation(program_, "uUVTransformation");

  gl->DeleteShader(vert_shader);
  gl->DeleteShader(frag_shader);

  egl_context->ClearCurrent();

  gl_ = std::move(gl);
  egl_display_ = std::move(egl_display);
  egl_context_ = std::move(egl_context);
  egl_surface_ = std::move(egl_surface);
  is_valid_ = true;
}

Trampoline::~Trampoline() {
  if (!is_valid_) {
    return;
  }
  auto context = MakeCurrentContext();
  gl_->DeleteProgram(program_);
}

bool Trampoline::IsValid() const {
  return is_valid_;
}

static UniqueEGLImageKHR CreateEGLImageFromAHBTexture(
    const EGLDisplay& display,
    const AHBTextureSourceVK& to_texture) {
  if (!android::GetProcTable().eglGetNativeClientBufferANDROID.IsAvailable()) {
    VALIDATION_LOG << "Could not get native client buffer.";
    return {};
  }

  EGLClientBuffer client_buffer =
      android::GetProcTable().eglGetNativeClientBufferANDROID(
          to_texture.GetBackingStore()->GetHandle());

  if (!client_buffer) {
    VALIDATION_LOG
        << "Could not get client buffer from Android hardware buffer.";
    return {};
  }

  auto image = ::eglCreateImageKHR(display,                    //
                                   EGL_NO_CONTEXT,             //
                                   EGL_NATIVE_BUFFER_ANDROID,  //
                                   client_buffer,              //
                                   nullptr                     //
  );
  if (image == NULL) {
    VALIDATION_LOG << "Could not create EGL Image.";
    return {};
  }

  return UniqueEGLImageKHR(EGLImageKHRWithDisplay{image, display});
}

bool Trampoline::BlitTextureOpenGLToVulkan(
    const GLTextureInfo& src_texture,
    const AHBTextureSourceVK& dst_texture) const {
  TRACE_EVENT0("impeller", __FUNCTION__);
  if (!is_valid_) {
    return false;
  }

  FML_DCHECK(egl_context_->IsCurrent());

  auto dst_egl_image =
      CreateEGLImageFromAHBTexture(egl_display_->GetHandle(), dst_texture);
  if (!dst_egl_image.is_valid()) {
    VALIDATION_LOG << "Could not create EGL image from AHB texture.";
    return false;
  }

  const auto& gl = *gl_;

  GLuint dst_gl_texture = GL_NONE;
  gl.GenTextures(1u, &dst_gl_texture);
  gl.BindTexture(GL_TEXTURE_2D, dst_gl_texture);
  gl.EGLImageTargetTexture2DOES(GL_TEXTURE_2D, dst_egl_image.get().image);

  GLuint offscreen_fbo = GL_NONE;
  gl.GenFramebuffers(1u, &offscreen_fbo);
  gl.BindFramebuffer(GL_FRAMEBUFFER, offscreen_fbo);
  gl.FramebufferTexture2D(GL_FRAMEBUFFER,        //
                          GL_COLOR_ATTACHMENT0,  //
                          GL_TEXTURE_2D,         //
                          dst_gl_texture,        //
                          0                      //
  );

  FML_CHECK(gl.CheckFramebufferStatus(GL_FRAMEBUFFER) ==
            GL_FRAMEBUFFER_COMPLETE);

  gl.Disable(GL_BLEND);
  gl.Disable(GL_SCISSOR_TEST);
  gl.Disable(GL_DITHER);
  gl.Disable(GL_CULL_FACE);
  gl.ColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

  gl.ClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  gl.Clear(GL_COLOR_BUFFER_BIT);

  const auto& fb_size = dst_texture.GetTextureDescriptor().size;
  gl.Viewport(0, 0, fb_size.width, fb_size.height);

  gl.UseProgram(program_);

  struct VertexData {
    Point position;
    Point tex_coord;
  };

  // The vertex coordinates assume OpenGL NDC because that's the API we are
  // using to draw the quad. But the texture will be sampled in Vulkan so the
  // texture coordinate system assumes Vulkan convention.
  //
  // See the following help link for an overview of the different coordinate
  // systems:
  // https://github.com/flutter/engine/blob/5810b3fc791f4bb82b9a454014310990eddc1181/impeller/docs/coordinate_system.md
  static constexpr const VertexData kVertData[] = {
      {{-1, -1}, {0, 1}},  // bottom left
      {{-1, +1}, {0, 0}},  // top left
      {{+1, +1}, {1, 0}},  // top right
      {{+1, -1}, {1, 1}},  // bottom right
  };

  // This is tedious but we assume no vertex array objects (VAO) are available
  // because of ES 2 versioning constraints.
  GLuint vertex_buffer = GL_NONE;
  gl.GenBuffers(1u, &vertex_buffer);
  gl.BindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
  gl.BufferData(GL_ARRAY_BUFFER, sizeof(kVertData), kVertData, GL_STATIC_DRAW);
  gl.EnableVertexAttribArray(kAttributeIndexPosition);
  gl.EnableVertexAttribArray(kAttributeIndexTexCoord);
  gl.VertexAttribPointer(
      kAttributeIndexPosition, 2, GL_FLOAT, GL_FALSE, sizeof(VertexData),
      reinterpret_cast<void*>(offsetof(VertexData, position)));
  gl.VertexAttribPointer(
      kAttributeIndexTexCoord, 2, GL_FLOAT, GL_FALSE, sizeof(VertexData),
      reinterpret_cast<void*>(offsetof(VertexData, tex_coord)));

  gl.ActiveTexture(GL_TEXTURE0);
  gl.BindTexture(src_texture.target, src_texture.texture);
  gl.TexParameteri(src_texture.target, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  gl.TexParameteri(src_texture.target, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  gl.TexParameteri(src_texture.target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  gl.TexParameteri(src_texture.target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  gl.Uniform1i(texture_uniform_location_, 0u);

  auto gl_uv_transformation = src_texture.uv_transformation;

  gl.UniformMatrix4fv(uv_transformation_location_, 1u, GL_FALSE,
                      reinterpret_cast<GLfloat*>(&gl_uv_transformation));

  gl.DrawArrays(GL_TRIANGLE_FAN, 0, 4);

  gl.UseProgram(GL_NONE);

  gl.Flush();

  gl.DeleteFramebuffers(1u, &offscreen_fbo);
  gl.DeleteTextures(1u, &dst_gl_texture);
  gl.DeleteBuffers(1u, &vertex_buffer);

  // Theoretically, this does nothing because the surface is a 1x1 pbuffer
  // surface. But frame capture tools use this to denote a frame boundary in
  // OpenGL. So add this as a debugging aid anyway.
  eglSwapBuffers(egl_display_->GetHandle(), egl_surface_->GetHandle());

  return true;
}

AutoTrampolineContext Trampoline::MakeCurrentContext() const {
  FML_DCHECK(is_valid_);
  return AutoTrampolineContext{*this};
}

AutoTrampolineContext::AutoTrampolineContext(const Trampoline& trampoline)
    : context_(trampoline.egl_context_.get()),
      surface_(trampoline.egl_surface_.get()) {
  if (!context_->IsCurrent() && !context_->MakeCurrent(*surface_)) {
    VALIDATION_LOG << "Could not make context current.";
  }
};

AutoTrampolineContext::~AutoTrampolineContext() {
  if (!context_->ClearCurrent()) {
    VALIDATION_LOG << "Could not clear current context.";
  }
}

}  // namespace impeller::glvk
