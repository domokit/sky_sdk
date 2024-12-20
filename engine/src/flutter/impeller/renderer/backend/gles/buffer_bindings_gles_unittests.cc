// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"  // IWYU pragma: keep
#include "gtest/gtest.h"
#include "impeller/renderer/backend/gles/buffer_bindings_gles.h"
#include "impeller/renderer/backend/gles/device_buffer_gles.h"
#include "impeller/renderer/backend/gles/test/mock_gles.h"
#include "impeller/renderer/command.h"

namespace impeller {
namespace testing {

using ::testing::_;

TEST(BufferBindingsGLESTest, BindUniformData) {
  BufferBindingsGLES bindings;
  absl::flat_hash_map<std::string, GLint> uniform_bindings;
  uniform_bindings["SHADERMETADATA.FOOBAR"] = 1;
  bindings.SetUniformBindings(std::move(uniform_bindings));
  auto mock_gles_impl = std::make_unique<MockGLESImpl>();

  EXPECT_CALL(*mock_gles_impl, Uniform1fv(_, _, _)).Times(1);

  std::shared_ptr<MockGLES> mock_gl = MockGLES::Init(std::move(mock_gles_impl));
  std::vector<BufferResource> bound_buffers;
  std::vector<TextureAndSampler> bound_textures;

  ShaderMetadata shader_metadata = {
      .name = "shader_metadata",
      .members = {ShaderStructMemberMetadata{.type = ShaderType::kFloat,
                                             .name = "foobar",
                                             .offset = 0,
                                             .size = sizeof(float),
                                             .byte_length = sizeof(float)}}};
  std::shared_ptr<ReactorGLES> reactor;
  std::shared_ptr<Allocation> backing_store = std::make_shared<Allocation>();
  ASSERT_TRUE(backing_store->Truncate(Bytes{sizeof(float)}));
  DeviceBufferGLES device_buffer(DeviceBufferDescriptor{.size = sizeof(float)},
                                 reactor, backing_store);
  BufferView buffer_view(&device_buffer, Range(0, sizeof(float)));
  bound_buffers.push_back(BufferResource(&shader_metadata, buffer_view));

  EXPECT_TRUE(bindings.BindUniformData(mock_gl->GetProcTable(), bound_textures,
                                       bound_buffers, Range{0, 0},
                                       Range{0, 1}));
}

}  // namespace testing
}  // namespace impeller
