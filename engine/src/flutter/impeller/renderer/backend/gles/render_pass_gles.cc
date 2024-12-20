// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/render_pass_gles.h"

#include <cstdint>

#include "flutter/fml/trace_event.h"
#include "fml/closure.h"
#include "fml/logging.h"
#include "impeller/base/validation.h"
#include "impeller/core/buffer_view.h"
#include "impeller/core/formats.h"
#include "impeller/renderer/backend/gles/buffer_bindings_gles.h"
#include "impeller/renderer/backend/gles/context_gles.h"
#include "impeller/renderer/backend/gles/device_buffer_gles.h"
#include "impeller/renderer/backend/gles/formats_gles.h"
#include "impeller/renderer/backend/gles/gpu_tracer_gles.h"
#include "impeller/renderer/backend/gles/pipeline_gles.h"
#include "impeller/renderer/backend/gles/texture_gles.h"
#include "impeller/renderer/command.h"

namespace impeller {

RenderPassGLES::RenderPassGLES(std::shared_ptr<const Context> context,
                               const RenderTarget& target,
                               std::shared_ptr<ReactorGLES> reactor)
    : RenderPass(std::move(context), target),
      reactor_(std::move(reactor)),
      is_valid_(reactor_ && reactor_->IsValid()) {}

// |RenderPass|
RenderPassGLES::~RenderPassGLES() = default;

// |RenderPass|
bool RenderPassGLES::IsValid() const {
  return is_valid_;
}

// |RenderPass|
void RenderPassGLES::OnSetLabel(std::string_view label) {
  label_ = label;
}

void ConfigureBlending(const ProcTableGLES& gl,
                       const ColorAttachmentDescriptor* color) {
  if (color->blending_enabled) {
    gl.Enable(GL_BLEND);
    gl.BlendFuncSeparate(
        ToBlendFactor(color->src_color_blend_factor),  // src color
        ToBlendFactor(color->dst_color_blend_factor),  // dst color
        ToBlendFactor(color->src_alpha_blend_factor),  // src alpha
        ToBlendFactor(color->dst_alpha_blend_factor)   // dst alpha
    );
    gl.BlendEquationSeparate(
        ToBlendOperation(color->color_blend_op),  // mode color
        ToBlendOperation(color->alpha_blend_op)   // mode alpha
    );
  } else {
    gl.Disable(GL_BLEND);
  }

  {
    const auto is_set = [](ColorWriteMask mask,
                           ColorWriteMask check) -> GLboolean {
      return (mask & check) ? GL_TRUE : GL_FALSE;
    };

    gl.ColorMask(
        is_set(color->write_mask, ColorWriteMaskBits::kRed),    // red
        is_set(color->write_mask, ColorWriteMaskBits::kGreen),  // green
        is_set(color->write_mask, ColorWriteMaskBits::kBlue),   // blue
        is_set(color->write_mask, ColorWriteMaskBits::kAlpha)   // alpha
    );
  }
}

void ConfigureStencil(GLenum face,
                      const ProcTableGLES& gl,
                      const StencilAttachmentDescriptor& stencil,
                      uint32_t stencil_reference) {
  gl.StencilOpSeparate(
      face,                                    // face
      ToStencilOp(stencil.stencil_failure),    // stencil fail
      ToStencilOp(stencil.depth_failure),      // depth fail
      ToStencilOp(stencil.depth_stencil_pass)  // depth stencil pass
  );
  gl.StencilFuncSeparate(face,                                        // face
                         ToCompareFunction(stencil.stencil_compare),  // func
                         stencil_reference,                           // ref
                         stencil.read_mask                            // mask
  );
  gl.StencilMaskSeparate(face, stencil.write_mask);
}

void ConfigureStencil(const ProcTableGLES& gl,
                      const PipelineDescriptor& pipeline,
                      uint32_t stencil_reference) {
  if (!pipeline.HasStencilAttachmentDescriptors()) {
    gl.Disable(GL_STENCIL_TEST);
    return;
  }

  gl.Enable(GL_STENCIL_TEST);
  const auto& front = pipeline.GetFrontStencilAttachmentDescriptor();
  const auto& back = pipeline.GetBackStencilAttachmentDescriptor();

  if (front.has_value() && back.has_value() && front == back) {
    ConfigureStencil(GL_FRONT_AND_BACK, gl, *front, stencil_reference);
    return;
  }
  if (front.has_value()) {
    ConfigureStencil(GL_FRONT, gl, *front, stencil_reference);
  }
  if (back.has_value()) {
    ConfigureStencil(GL_BACK, gl, *back, stencil_reference);
  }
}

//------------------------------------------------------------------------------
/// @brief      Encapsulates data that will be needed in the reactor for the
///             encoding of commands for this render pass.
///
struct RenderPassData {
  Viewport viewport;

  Color clear_color;
  uint32_t clear_stencil = 0u;
  Scalar clear_depth = 1.0;

  std::shared_ptr<Texture> color_attachment;
  std::shared_ptr<Texture> depth_attachment;
  std::shared_ptr<Texture> stencil_attachment;

  bool clear_color_attachment = true;
  bool clear_depth_attachment = true;
  bool clear_stencil_attachment = true;

  bool discard_color_attachment = true;
  bool discard_depth_attachment = true;
  bool discard_stencil_attachment = true;

  std::string label;
};

static bool BindVertexBuffer(const ProcTableGLES& gl,
                             BufferBindingsGLES* vertex_desc_gles,
                             const BufferView& vertex_buffer_view,
                             size_t buffer_index) {
  if (!vertex_buffer_view) {
    return false;
  }

  const DeviceBuffer* vertex_buffer = vertex_buffer_view.GetBuffer();

  if (!vertex_buffer) {
    return false;
  }

  const auto& vertex_buffer_gles = DeviceBufferGLES::Cast(*vertex_buffer);
  if (!vertex_buffer_gles.BindAndUploadDataIfNecessary(
          DeviceBufferGLES::BindingType::kArrayBuffer)) {
    return false;
  }

  //--------------------------------------------------------------------------
  /// Bind the vertex attributes associated with vertex buffer.
  ///
  if (!vertex_desc_gles->BindVertexAttributes(
          gl, buffer_index, vertex_buffer_view.GetRange().offset)) {
    return false;
  }

  return true;
}

void RenderPassGLES::ResetGLState(const ProcTableGLES& gl) {
  gl.Disable(GL_SCISSOR_TEST);
  gl.Disable(GL_DEPTH_TEST);
  gl.Disable(GL_STENCIL_TEST);
  gl.Disable(GL_CULL_FACE);
  gl.Disable(GL_BLEND);
  gl.Disable(GL_DITHER);
  gl.ColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
  gl.DepthMask(GL_TRUE);
  gl.StencilMaskSeparate(GL_FRONT, 0xFFFFFFFF);
  gl.StencilMaskSeparate(GL_BACK, 0xFFFFFFFF);
}

[[nodiscard]] bool EncodeCommandsInReactor(
    const RenderPassData& pass_data,
    const ReactorGLES& reactor,
    const std::vector<Command>& commands,
    const std::vector<BufferView>& vertex_buffers,
    const std::vector<TextureAndSampler>& bound_textures,
    const std::vector<BufferResource>& bound_buffers,
    const std::shared_ptr<GPUTracerGLES>& tracer) {
  TRACE_EVENT0("impeller", "RenderPassGLES::EncodeCommandsInReactor");

  const auto& gl = reactor.GetProcTable();
#ifdef IMPELLER_DEBUG
  tracer->MarkFrameStart(gl);

  fml::ScopedCleanupClosure pop_pass_debug_marker(
      [&gl]() { gl.PopDebugGroup(); });
  if (!pass_data.label.empty()) {
    gl.PushDebugGroup(pass_data.label);
  } else {
    pop_pass_debug_marker.Release();
  }
#endif  // IMPELLER_DEBUG

  GLuint fbo = GL_NONE;
  TextureGLES& color_gles = TextureGLES::Cast(*pass_data.color_attachment);
  const bool is_default_fbo = color_gles.IsWrapped();

  if (is_default_fbo) {
    if (color_gles.GetFBO().has_value()) {
      // NOLINTNEXTLINE(bugprone-unchecked-optional-access)
      gl.BindFramebuffer(GL_FRAMEBUFFER, *color_gles.GetFBO());
    }
  } else {
    // Create and bind an offscreen FBO.
    GLuint cached_fbo = color_gles.GetCachedFBO();
    if (cached_fbo != GL_NONE) {
      fbo = cached_fbo;
      gl.BindFramebuffer(GL_FRAMEBUFFER, fbo);
    } else {
      gl.GenFramebuffers(1u, &fbo);
      color_gles.SetCachedFBO(fbo);
      gl.BindFramebuffer(GL_FRAMEBUFFER, fbo);

      if (!color_gles.SetAsFramebufferAttachment(
              GL_FRAMEBUFFER, TextureGLES::AttachmentType::kColor0)) {
        return false;
      }

      if (auto depth = TextureGLES::Cast(pass_data.depth_attachment.get())) {
        if (!depth->SetAsFramebufferAttachment(
                GL_FRAMEBUFFER, TextureGLES::AttachmentType::kDepth)) {
          return false;
        }
      }
      if (auto stencil =
              TextureGLES::Cast(pass_data.stencil_attachment.get())) {
        if (!stencil->SetAsFramebufferAttachment(
                GL_FRAMEBUFFER, TextureGLES::AttachmentType::kStencil)) {
          return false;
        }
      }

      auto status = gl.CheckFramebufferStatus(GL_FRAMEBUFFER);
      if (status != GL_FRAMEBUFFER_COMPLETE) {
        VALIDATION_LOG << "Could not create a complete frambuffer: "
                       << DebugToFramebufferError(status);
        return false;
      }
    }
  }

  gl.ClearColor(pass_data.clear_color.red,    // red
                pass_data.clear_color.green,  // green
                pass_data.clear_color.blue,   // blue
                pass_data.clear_color.alpha   // alpha
  );
  if (pass_data.depth_attachment) {
    if (gl.DepthRangef.IsAvailable()) {
      gl.ClearDepthf(pass_data.clear_depth);
    } else {
      gl.ClearDepth(pass_data.clear_depth);
    }
  }
  if (pass_data.stencil_attachment) {
    gl.ClearStencil(pass_data.clear_stencil);
  }

  GLenum clear_bits = 0u;
  if (pass_data.clear_color_attachment) {
    clear_bits |= GL_COLOR_BUFFER_BIT;
  }
  if (pass_data.clear_depth_attachment) {
    clear_bits |= GL_DEPTH_BUFFER_BIT;
  }
  if (pass_data.clear_stencil_attachment) {
    clear_bits |= GL_STENCIL_BUFFER_BIT;
  }

  RenderPassGLES::ResetGLState(gl);

  gl.Clear(clear_bits);

  // Both the viewport and scissor are specified in framebuffer coordinates.
  // Impeller's framebuffer coordinate system is top left origin, but OpenGL's
  // is bottom left origin, so we convert the coordinates here.
  ISize target_size = pass_data.color_attachment->GetSize();

  //--------------------------------------------------------------------------
  /// Setup the viewport.
  ///
  const auto& viewport = pass_data.viewport;
  gl.Viewport(viewport.rect.GetX(),  // x
              target_size.height - viewport.rect.GetY() -
                  viewport.rect.GetHeight(),  // y
              viewport.rect.GetWidth(),       // width
              viewport.rect.GetHeight()       // height
  );
  if (pass_data.depth_attachment) {
    if (gl.DepthRangef.IsAvailable()) {
      gl.DepthRangef(viewport.depth_range.z_near, viewport.depth_range.z_far);
    } else {
      gl.DepthRange(viewport.depth_range.z_near, viewport.depth_range.z_far);
    }
  }

  CullMode current_cull_mode = CullMode::kNone;
  WindingOrder current_winding_order = WindingOrder::kClockwise;
  gl.FrontFace(GL_CW);

  for (const auto& command : commands) {
#ifdef IMPELLER_DEBUG
    fml::ScopedCleanupClosure pop_cmd_debug_marker(
        [&gl]() { gl.PopDebugGroup(); });
    if (!command.label.empty()) {
      gl.PushDebugGroup(command.label);
    } else {
      pop_cmd_debug_marker.Release();
    }
#endif  // IMPELLER_DEBUG

    const auto& pipeline = PipelineGLES::Cast(*command.pipeline);

    const auto* color_attachment =
        pipeline.GetDescriptor().GetLegacyCompatibleColorAttachment();
    if (!color_attachment) {
      VALIDATION_LOG
          << "Color attachment is too complicated for a legacy renderer.";
      return false;
    }

    //--------------------------------------------------------------------------
    /// Configure blending.
    ///
    ConfigureBlending(gl, color_attachment);

    //--------------------------------------------------------------------------
    /// Setup stencil.
    ///
    ConfigureStencil(gl, pipeline.GetDescriptor(), command.stencil_reference);

    //--------------------------------------------------------------------------
    /// Configure depth.
    ///
    if (auto depth =
            pipeline.GetDescriptor().GetDepthStencilAttachmentDescriptor();
        depth.has_value()) {
      gl.Enable(GL_DEPTH_TEST);
      gl.DepthFunc(ToCompareFunction(depth->depth_compare));
      gl.DepthMask(depth->depth_write_enabled ? GL_TRUE : GL_FALSE);
    } else {
      gl.Disable(GL_DEPTH_TEST);
    }

    //--------------------------------------------------------------------------
    /// Setup the viewport.
    ///
    if (command.viewport.has_value()) {
      gl.Viewport(viewport.rect.GetX(),  // x
                  target_size.height - viewport.rect.GetY() -
                      viewport.rect.GetHeight(),  // y
                  viewport.rect.GetWidth(),       // width
                  viewport.rect.GetHeight()       // height
      );
      if (pass_data.depth_attachment) {
        if (gl.DepthRangef.IsAvailable()) {
          gl.DepthRangef(viewport.depth_range.z_near,
                         viewport.depth_range.z_far);
        } else {
          gl.DepthRange(viewport.depth_range.z_near,
                        viewport.depth_range.z_far);
        }
      }
    }

    //--------------------------------------------------------------------------
    /// Setup the scissor rect.
    ///
    if (command.scissor.has_value()) {
      const auto& scissor = command.scissor.value();
      gl.Enable(GL_SCISSOR_TEST);
      gl.Scissor(
          scissor.GetX(),                                             // x
          target_size.height - scissor.GetY() - scissor.GetHeight(),  // y
          scissor.GetWidth(),                                         // width
          scissor.GetHeight()                                         // height
      );
    }

    //--------------------------------------------------------------------------
    /// Setup culling.
    ///
    CullMode pipeline_cull_mode = pipeline.GetDescriptor().GetCullMode();
    if (current_cull_mode != pipeline_cull_mode) {
      switch (pipeline_cull_mode) {
        case CullMode::kNone:
          gl.Disable(GL_CULL_FACE);
          break;
        case CullMode::kFrontFace:
          gl.Enable(GL_CULL_FACE);
          gl.CullFace(GL_FRONT);
          break;
        case CullMode::kBackFace:
          gl.Enable(GL_CULL_FACE);
          gl.CullFace(GL_BACK);
          break;
      }
      current_cull_mode = pipeline_cull_mode;
    }

    //--------------------------------------------------------------------------
    /// Setup winding order.
    ///
    WindingOrder pipeline_winding_order =
        pipeline.GetDescriptor().GetWindingOrder();
    if (current_winding_order != pipeline_winding_order) {
      switch (pipeline.GetDescriptor().GetWindingOrder()) {
        case WindingOrder::kClockwise:
          gl.FrontFace(GL_CW);
          break;
        case WindingOrder::kCounterClockwise:
          gl.FrontFace(GL_CCW);
          break;
      }
      current_winding_order = pipeline_winding_order;
    }

    BufferBindingsGLES* vertex_desc_gles = pipeline.GetBufferBindings();

    //--------------------------------------------------------------------------
    /// Bind vertex buffers.
    ///
    /// Note: There is no need to run `RenderPass::ValidateVertexBuffers` or
    ///       `RenderPass::ValidateIndexBuffer` here, as validation already runs
    ///       when the vertex/index buffers are set on the command.
    ///
    for (size_t i = 0; i < command.vertex_buffers.length; i++) {
      if (!BindVertexBuffer(gl, vertex_desc_gles,
                            vertex_buffers[i + command.vertex_buffers.offset],
                            i)) {
        return false;
      }
    }

    //--------------------------------------------------------------------------
    /// Bind the pipeline program.
    ///
    if (!pipeline.BindProgram()) {
      return false;
    }

    //--------------------------------------------------------------------------
    /// Bind uniform data.
    ///
    if (!vertex_desc_gles->BindUniformData(
            gl,                                        //
            bound_textures,                            //
            bound_buffers,                             //
            /*texture_range=*/command.bound_textures,  //
            /*buffer_range=*/command.bound_buffers     //
            )) {
      return false;
    }

    //--------------------------------------------------------------------------
    /// Determine the primitive type.
    ///
    // GLES doesn't support setting the fill mode, so override the primitive
    // with GL_LINE_STRIP to somewhat emulate PolygonMode::kLine. This isn't
    // correct; full triangle outlines won't be drawn and disconnected
    // geometry may appear connected. However this can still be useful for
    // wireframe debug views.
    GLenum mode =
        pipeline.GetDescriptor().GetPolygonMode() == PolygonMode::kLine
            ? GL_LINE_STRIP
            : ToMode(pipeline.GetDescriptor().GetPrimitiveType());

    //--------------------------------------------------------------------------
    /// Finally! Invoke the draw call.
    ///
    if (command.index_type == IndexType::kNone) {
      gl.DrawArrays(mode, command.base_vertex, command.element_count);
    } else {
      // Bind the index buffer if necessary.
      auto index_buffer_view = command.index_buffer;
      const DeviceBuffer* index_buffer = index_buffer_view.GetBuffer();
      const auto& index_buffer_gles = DeviceBufferGLES::Cast(*index_buffer);
      if (!index_buffer_gles.BindAndUploadDataIfNecessary(
              DeviceBufferGLES::BindingType::kElementArrayBuffer)) {
        return false;
      }
      gl.DrawElements(mode,                             // mode
                      command.element_count,            // count
                      ToIndexType(command.index_type),  // type
                      reinterpret_cast<const GLvoid*>(static_cast<GLsizei>(
                          index_buffer_view.GetRange().offset))  // indices
      );
    }

    //--------------------------------------------------------------------------
    /// Unbind vertex attribs.
    ///
    if (!vertex_desc_gles->UnbindVertexAttributes(gl)) {
      return false;
    }
  }

  if (gl.DiscardFramebufferEXT.IsAvailable()) {
    std::array<GLenum, 3> attachments;
    size_t attachment_count = 0;

    // TODO(130048): discarding stencil or depth on the default fbo causes Angle
    // to discard the entire render target. Until we know the reason, default to
    // storing.
    bool angle_safe = gl.GetCapabilities()->IsANGLE() ? !is_default_fbo : true;

    if (pass_data.discard_color_attachment) {
      attachments[attachment_count++] =
          (is_default_fbo ? GL_COLOR_EXT : GL_COLOR_ATTACHMENT0);
    }
    if (pass_data.discard_depth_attachment && angle_safe) {
      attachments[attachment_count++] =
          (is_default_fbo ? GL_DEPTH_EXT : GL_DEPTH_ATTACHMENT);
    }

    if (pass_data.discard_stencil_attachment && angle_safe) {
      attachments[attachment_count++] =
          (is_default_fbo ? GL_STENCIL_EXT : GL_STENCIL_ATTACHMENT);
    }
    gl.DiscardFramebufferEXT(GL_FRAMEBUFFER,     // target
                             attachment_count,   // attachments to discard
                             attachments.data()  // size
    );
  }

#ifdef IMPELLER_DEBUG
  if (is_default_fbo) {
    tracer->MarkFrameEnd(gl);
  }
#endif  // IMPELLER_DEBUG

  return true;
}

// |RenderPass|
bool RenderPassGLES::OnEncodeCommands(const Context& context) const {
  if (!IsValid()) {
    return false;
  }
  const auto& render_target = GetRenderTarget();
  if (!render_target.HasColorAttachment(0u)) {
    return false;
  }
  const ColorAttachment& color0 = render_target.GetColorAttachment(0);
  const std::optional<DepthAttachment>& depth0 =
      render_target.GetDepthAttachment();
  const std::optional<StencilAttachment>& stencil0 =
      render_target.GetStencilAttachment();

  auto pass_data = std::make_shared<RenderPassData>();
  pass_data->label = label_;
  pass_data->viewport.rect = Rect::MakeSize(GetRenderTargetSize());

  //----------------------------------------------------------------------------
  /// Setup color data.
  ///
  pass_data->color_attachment = color0.texture;
  pass_data->clear_color = color0.clear_color;
  pass_data->clear_color_attachment = CanClearAttachment(color0.load_action);
  pass_data->discard_color_attachment =
      CanDiscardAttachmentWhenDone(color0.store_action);

  // When we are using EXT_multisampled_render_to_texture, it is implicitly
  // resolved when we bind the texture to the framebuffer. We don't need to
  // discard the attachment when we are done.
  if (color0.resolve_texture) {
    FML_DCHECK(context.GetCapabilities()->SupportsImplicitResolvingMSAA());
    pass_data->discard_color_attachment = false;
  }

  //----------------------------------------------------------------------------
  /// Setup depth data.
  ///
  if (depth0.has_value()) {
    pass_data->depth_attachment = depth0->texture;
    pass_data->clear_depth = depth0->clear_depth;
    pass_data->clear_depth_attachment = CanClearAttachment(depth0->load_action);
    pass_data->discard_depth_attachment =
        CanDiscardAttachmentWhenDone(depth0->store_action);
  }

  //----------------------------------------------------------------------------
  /// Setup stencil data.
  ///
  if (stencil0.has_value()) {
    pass_data->stencil_attachment = stencil0->texture;
    pass_data->clear_stencil = stencil0->clear_stencil;
    pass_data->clear_stencil_attachment =
        CanClearAttachment(stencil0->load_action);
    pass_data->discard_stencil_attachment =
        CanDiscardAttachmentWhenDone(stencil0->store_action);
  }

  return reactor_->AddOperation(
      [pass_data = std::move(pass_data), render_pass = shared_from_this(),
       tracer =
           ContextGLES::Cast(context).GetGPUTracer()](const auto& reactor) {
        auto result = EncodeCommandsInReactor(
            /*pass_data=*/*pass_data,                         //
            /*reactor=*/reactor,                              //
            /*commands=*/render_pass->commands_,              //
            /*vertex_buffers=*/render_pass->vertex_buffers_,  //
            /*bound_textures=*/render_pass->bound_textures_,  //
            /*bound_buffers=*/render_pass->bound_buffers_,    //
            /*tracer=*/tracer                                 //
        );
        FML_CHECK(result)
            << "Must be able to encode GL commands without error.";
      },
      /*defer=*/true);
}

}  // namespace impeller
