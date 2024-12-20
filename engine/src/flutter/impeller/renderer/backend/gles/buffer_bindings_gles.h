// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_BUFFER_BINDINGS_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_BUFFER_BINDINGS_GLES_H_

#include <vector>

#include "flutter/third_party/abseil-cpp/absl/container/flat_hash_map.h"
#include "impeller/core/shader_types.h"
#include "impeller/renderer/backend/gles/device_buffer_gles.h"
#include "impeller/renderer/backend/gles/gles.h"
#include "impeller/renderer/backend/gles/proc_table_gles.h"
#include "impeller/renderer/command.h"

namespace impeller {

namespace testing {
FML_TEST_CLASS(BufferBindingsGLESTest, BindUniformData);
}  // namespace testing

//------------------------------------------------------------------------------
/// @brief      Sets up stage bindings for single draw call in the OpenGLES
///             backend.
///
class BufferBindingsGLES {
 public:
  BufferBindingsGLES();

  ~BufferBindingsGLES();

  bool RegisterVertexStageInput(
      const ProcTableGLES& gl,
      const std::vector<ShaderStageIOSlot>& inputs,
      const std::vector<ShaderStageBufferLayout>& layouts);

  bool ReadUniformsBindings(const ProcTableGLES& gl, GLuint program);

  bool BindVertexAttributes(const ProcTableGLES& gl,
                            size_t binding,
                            size_t vertex_offset);

  bool BindUniformData(const ProcTableGLES& gl,
                       const std::vector<TextureAndSampler>& bound_textures,
                       const std::vector<BufferResource>& bound_buffers,
                       Range texture_range,
                       Range buffer_range);

  bool UnbindVertexAttributes(const ProcTableGLES& gl);

 private:
  FML_FRIEND_TEST(testing::BufferBindingsGLESTest, BindUniformData);
  //----------------------------------------------------------------------------
  /// @brief      The arguments to glVertexAttribPointer.
  ///
  struct VertexAttribPointer {
    GLuint index = 0u;
    GLint size = 4;
    GLenum type = GL_FLOAT;
    GLenum normalized = GL_FALSE;
    GLsizei stride = 0u;
    GLsizei offset = 0u;
  };
  std::vector<std::vector<VertexAttribPointer>> vertex_attrib_arrays_;

  absl::flat_hash_map<std::string, GLint> uniform_locations_;
  absl::flat_hash_map<std::string, std::pair<GLint, GLuint>> ubo_locations_;

  using BindingMap = absl::flat_hash_map<std::string, std::vector<GLint>>;
  BindingMap binding_map_ = {};
  GLuint vertex_array_object_ = 0;
  GLuint program_handle_ = GL_NONE;
  bool use_ubo_ = false;

  const std::vector<GLint>& ComputeUniformLocations(
      const ShaderMetadata* metadata);

  bool ReadUniformsBindingsV2(const ProcTableGLES& gl, GLuint program);

  bool ReadUniformsBindingsV3(const ProcTableGLES& gl, GLuint program);

  GLint ComputeTextureLocation(const ShaderMetadata* metadata);

  bool BindUniformBuffer(const ProcTableGLES& gl, const BufferResource& buffer);

  bool BindUniformBufferV2(const ProcTableGLES& gl,
                           const BufferView& buffer,
                           const ShaderMetadata* metadata,
                           const DeviceBufferGLES& device_buffer_gles);

  bool BindUniformBufferV3(const ProcTableGLES& gl,
                           const BufferView& buffer,
                           const ShaderMetadata* metadata,
                           const DeviceBufferGLES& device_buffer_gles);

  std::optional<size_t> BindTextures(
      const ProcTableGLES& gl,
      const std::vector<TextureAndSampler>& bound_textures,
      Range texture_range,
      ShaderStage stage,
      size_t unit_start_index = 0);

  BufferBindingsGLES(const BufferBindingsGLES&) = delete;

  BufferBindingsGLES& operator=(const BufferBindingsGLES&) = delete;

  // For testing.
  void SetUniformBindings(
      absl::flat_hash_map<std::string, GLint> uniform_locations) {
    uniform_locations_ = std::move(uniform_locations);
  }
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_BUFFER_BINDINGS_GLES_H_
