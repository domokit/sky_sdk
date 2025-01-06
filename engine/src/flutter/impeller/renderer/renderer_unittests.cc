// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/logging.h"
#include "flutter/fml/time/time_point.h"
#include "impeller/core/device_buffer_descriptor.h"
#include "impeller/core/formats.h"
#include "impeller/core/host_buffer.h"
#include "impeller/core/sampler_descriptor.h"
#include "impeller/fixtures/array.frag.h"
#include "impeller/fixtures/array.vert.h"
#include "impeller/fixtures/baby.frag.h"
#include "impeller/fixtures/baby.vert.h"
#include "impeller/fixtures/box_fade.frag.h"
#include "impeller/fixtures/box_fade.vert.h"
#include "impeller/fixtures/colors.frag.h"
#include "impeller/fixtures/colors.vert.h"
#include "impeller/fixtures/impeller.frag.h"
#include "impeller/fixtures/impeller.vert.h"
#include "impeller/fixtures/inactive_uniforms.frag.h"
#include "impeller/fixtures/inactive_uniforms.vert.h"
#include "impeller/fixtures/instanced_draw.frag.h"
#include "impeller/fixtures/instanced_draw.vert.h"
#include "impeller/fixtures/mipmaps.frag.h"
#include "impeller/fixtures/mipmaps.vert.h"
#include "impeller/fixtures/planet.frag.h"
#include "impeller/fixtures/planet.vert.h"
#include "impeller/fixtures/sepia.frag.h"
#include "impeller/fixtures/sepia.vert.h"
#include "impeller/fixtures/swizzle.frag.h"
#include "impeller/fixtures/texture.frag.h"
#include "impeller/fixtures/texture.vert.h"
#include "impeller/playground/playground.h"
#include "impeller/playground/playground_test.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/pipeline_builder.h"
#include "impeller/renderer/pipeline_library.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/render_target.h"
#include "impeller/renderer/vertex_buffer_builder.h"
#include "third_party/imgui/imgui.h"

// TODO(zanderso): https://github.com/flutter/flutter/issues/127701
// NOLINTBEGIN(bugprone-unchecked-optional-access)

namespace impeller {
namespace testing {

using RendererTest = PlaygroundTest;
INSTANTIATE_PLAYGROUND_SUITE(RendererTest);

TEST_P(RendererTest, CanCreateBoxPrimitive) {
  using VS = BoxFadeVertexShader;
  using FS = BoxFadeFragmentShader;
  auto context = GetContext();
  ASSERT_TRUE(context);
  using BoxPipelineBuilder = PipelineBuilder<VS, FS>;
  auto desc = BoxPipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(desc.has_value());
  desc->SetSampleCount(SampleCount::kCount4);
  desc->SetStencilAttachmentDescriptors(std::nullopt);

  // Vertex buffer.
  VertexBufferBuilder<VS::PerVertexData> vertex_builder;
  vertex_builder.SetLabel("Box");
  vertex_builder.AddVertices({
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 100, 0.0}, {1.0, 0.0}},  // 2
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 800, 0.0}, {0.0, 1.0}},  // 4
  });
  auto bridge = CreateTextureForFixture("bay_bridge.jpg");
  auto boston = CreateTextureForFixture("boston.jpg");
  ASSERT_TRUE(bridge && boston);
  raw_ptr<const Sampler> sampler = context->GetSamplerLibrary()->GetSampler({});
  ASSERT_TRUE(sampler);

  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  SinglePassCallback callback = [&](RenderPass& pass) {
    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    static bool wireframe;
    ImGui::Checkbox("Wireframe", &wireframe);
    ImGui::End();

    desc->SetPolygonMode(wireframe ? PolygonMode::kLine : PolygonMode::kFill);
    auto pipeline = context->GetPipelineLibrary()->GetPipeline(desc).Get();

    assert(pipeline && pipeline->IsValid());

    pass.SetCommandLabel("Box");
    pass.SetPipeline(pipeline);
    pass.SetVertexBuffer(
        vertex_builder.CreateVertexBuffer(*context->GetResourceAllocator()));

    VS::UniformBuffer uniforms;
    EXPECT_EQ(pass.GetOrthographicTransform(),
              Matrix::MakeOrthographic(pass.GetRenderTargetSize()));
    uniforms.mvp =
        pass.GetOrthographicTransform() * Matrix::MakeScale(GetContentScale());
    VS::BindUniformBuffer(pass, host_buffer->EmplaceUniform(uniforms));

    FS::FrameInfo frame_info;
    frame_info.current_time = GetSecondsElapsed();
    frame_info.cursor_position = GetCursorPosition();
    frame_info.window_size.x = GetWindowSize().width;
    frame_info.window_size.y = GetWindowSize().height;

    FS::BindFrameInfo(pass, host_buffer->EmplaceUniform(frame_info));
    FS::BindContents1(pass, boston, sampler);
    FS::BindContents2(pass, bridge, sampler);

    host_buffer->Reset();
    return pass.Draw().ok();
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, BabysFirstTriangle) {
  auto context = GetContext();
  ASSERT_TRUE(context);

  // Declare a shorthand for the shaders we are going to use.
  using VS = BabyVertexShader;
  using FS = BabyFragmentShader;

  // Create a pipeline descriptor that uses the shaders together and default
  // initializes the fixed function state.
  //
  // If the vertex shader outputs disagree with the fragment shader inputs, this
  // will be a compile time error.
  auto desc = PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(desc.has_value());

  // Modify the descriptor for our environment. This is specific to our test.
  desc->SetSampleCount(SampleCount::kCount4);
  desc->SetStencilAttachmentDescriptors(std::nullopt);

  // Create a pipeline from our descriptor. This is expensive to do. So just do
  // it once.
  auto pipeline = context->GetPipelineLibrary()->GetPipeline(desc).Get();

  // Create a host side buffer to build the vertex and uniform information.
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());

  // Specify the vertex buffer information.
  VertexBufferBuilder<VS::PerVertexData> vertex_buffer_builder;
  vertex_buffer_builder.AddVertices({
      {{-0.5, -0.5}, Color::Red(), Color::Green()},
      {{0.0, 0.5}, Color::Green(), Color::Blue()},
      {{0.5, -0.5}, Color::Blue(), Color::Red()},
  });

  auto vertex_buffer = vertex_buffer_builder.CreateVertexBuffer(
      *context->GetResourceAllocator());

  SinglePassCallback callback = [&](RenderPass& pass) {
    pass.SetPipeline(pipeline);
    pass.SetVertexBuffer(vertex_buffer);

    FS::FragInfo frag_info;
    frag_info.time = fml::TimePoint::Now().ToEpochDelta().ToSecondsF();

    auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                          context->GetIdleWaiter());
    FS::BindFragInfo(pass, host_buffer->EmplaceUniform(frag_info));

    return pass.Draw().ok();
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, CanRenderPerspectiveCube) {
  using VS = ColorsVertexShader;
  using FS = ColorsFragmentShader;
  auto context = GetContext();
  ASSERT_TRUE(context);
  auto desc = PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(desc.has_value());
  desc->SetCullMode(CullMode::kBackFace);
  desc->SetWindingOrder(WindingOrder::kCounterClockwise);
  desc->SetSampleCount(SampleCount::kCount4);
  desc->ClearStencilAttachments();

  // Setup the vertex layout to take two bindings. The first for positions and
  // the second for colors.
  auto vertex_desc = std::make_shared<VertexDescriptor>();
  ShaderStageIOSlot position_slot = VS::kInputPosition;
  ShaderStageIOSlot color_slot = VS::kInputColor;
  position_slot.binding = 0;
  position_slot.offset = 0;
  color_slot.binding = 1;
  color_slot.offset = 0;
  const std::vector<ShaderStageIOSlot> io_slots = {position_slot, color_slot};
  const std::vector<ShaderStageBufferLayout> layouts = {
      ShaderStageBufferLayout{.stride = 12u, .binding = 0},
      ShaderStageBufferLayout{.stride = 16u, .binding = 1}};
  vertex_desc->RegisterDescriptorSetLayouts(VS::kDescriptorSetLayouts);
  vertex_desc->RegisterDescriptorSetLayouts(FS::kDescriptorSetLayouts);
  vertex_desc->SetStageInputs(io_slots, layouts);
  desc->SetVertexDescriptor(std::move(vertex_desc));
  auto pipeline =
      context->GetPipelineLibrary()->GetPipeline(std::move(desc)).Get();
  ASSERT_TRUE(pipeline);

  struct Cube {
    Vector3 positions[8] = {
        // -Z
        {-1, -1, -1},
        {1, -1, -1},
        {1, 1, -1},
        {-1, 1, -1},
        // +Z
        {-1, -1, 1},
        {1, -1, 1},
        {1, 1, 1},
        {-1, 1, 1},
    };
    Color colors[8] = {
        Color::Red(),   Color::Yellow(), Color::Green(), Color::Blue(),
        Color::Green(), Color::Blue(),   Color::Red(),   Color::Yellow(),
    };
    uint16_t indices[36] = {
        1, 5, 2, 2, 5, 6,  // +X
        4, 0, 7, 7, 0, 3,  // -X
        4, 5, 0, 0, 5, 1,  // +Y
        3, 2, 7, 7, 2, 6,  // -Y
        5, 4, 6, 6, 4, 7,  // +Z
        0, 1, 3, 3, 1, 2,  // -Z
    };
  } cube;

  auto device_buffer = context->GetResourceAllocator()->CreateBufferWithCopy(
      reinterpret_cast<uint8_t*>(&cube), sizeof(cube));

  raw_ptr<const Sampler> sampler = context->GetSamplerLibrary()->GetSampler({});
  ASSERT_TRUE(sampler);

  Vector3 euler_angles;
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  SinglePassCallback callback = [&](RenderPass& pass) {
    static Degrees fov_y(60);
    static Scalar distance = 10;

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat("Field of view", &fov_y.degrees, 0, 180);
    ImGui::SliderFloat("Camera distance", &distance, 0, 30);
    ImGui::End();

    pass.SetCommandLabel("Perspective Cube");
    pass.SetPipeline(pipeline);

    std::array<BufferView, 2> vertex_buffers = {
        BufferView(device_buffer,
                   Range(offsetof(Cube, positions), sizeof(Cube::positions))),
        BufferView(device_buffer,
                   Range(offsetof(Cube, colors), sizeof(Cube::colors))),
    };

    BufferView index_buffer(
        device_buffer, Range(offsetof(Cube, indices), sizeof(Cube::indices)));
    pass.SetVertexBuffer(vertex_buffers.data(), vertex_buffers.size());
    pass.SetElementCount(36);
    pass.SetIndexBuffer(index_buffer, IndexType::k16bit);

    VS::UniformBuffer uniforms;
    Scalar time = GetSecondsElapsed();
    euler_angles = Vector3(0.19 * time, 0.7 * time, 0.43 * time);

    uniforms.mvp =
        Matrix::MakePerspective(fov_y, pass.GetRenderTargetSize(), 0, 10) *
        Matrix::MakeTranslation({0, 0, distance}) *
        Matrix::MakeRotationX(Radians(euler_angles.x)) *
        Matrix::MakeRotationY(Radians(euler_angles.y)) *
        Matrix::MakeRotationZ(Radians(euler_angles.z));
    VS::BindUniformBuffer(pass, host_buffer->EmplaceUniform(uniforms));

    host_buffer->Reset();
    return pass.Draw().ok();
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, CanRenderMultiplePrimitives) {
  using VS = BoxFadeVertexShader;
  using FS = BoxFadeFragmentShader;
  auto context = GetContext();
  ASSERT_TRUE(context);
  using BoxPipelineBuilder = PipelineBuilder<VS, FS>;
  auto desc = BoxPipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(desc.has_value());
  desc->SetSampleCount(SampleCount::kCount4);
  desc->SetStencilAttachmentDescriptors(std::nullopt);
  auto box_pipeline =
      context->GetPipelineLibrary()->GetPipeline(std::move(desc)).Get();
  ASSERT_TRUE(box_pipeline);

  // Vertex buffer.
  VertexBufferBuilder<VS::PerVertexData> vertex_builder;
  vertex_builder.SetLabel("Box");
  vertex_builder.AddVertices({
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 100, 0.0}, {1.0, 0.0}},  // 2
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 800, 0.0}, {0.0, 1.0}},  // 4
  });
  auto vertex_buffer =
      vertex_builder.CreateVertexBuffer(*context->GetResourceAllocator());
  ASSERT_TRUE(vertex_buffer);

  auto bridge = CreateTextureForFixture("bay_bridge.jpg");
  auto boston = CreateTextureForFixture("boston.jpg");
  ASSERT_TRUE(bridge && boston);
  raw_ptr<const Sampler> sampler = context->GetSamplerLibrary()->GetSampler({});
  ASSERT_TRUE(sampler);

  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  SinglePassCallback callback = [&](RenderPass& pass) {
    for (size_t i = 0; i < 1; i++) {
      for (size_t j = 0; j < 1; j++) {
        pass.SetCommandLabel("Box");
        pass.SetPipeline(box_pipeline);
        pass.SetVertexBuffer(vertex_buffer);

        FS::FrameInfo frame_info;
        frame_info.current_time = GetSecondsElapsed();
        frame_info.cursor_position = GetCursorPosition();
        frame_info.window_size.x = GetWindowSize().width;
        frame_info.window_size.y = GetWindowSize().height;

        FS::BindFrameInfo(pass, host_buffer->EmplaceUniform(frame_info));
        FS::BindContents1(pass, boston, sampler);
        FS::BindContents2(pass, bridge, sampler);

        VS::UniformBuffer uniforms;
        EXPECT_EQ(pass.GetOrthographicTransform(),
                  Matrix::MakeOrthographic(pass.GetRenderTargetSize()));
        uniforms.mvp = pass.GetOrthographicTransform() *
                       Matrix::MakeScale(GetContentScale()) *
                       Matrix::MakeTranslation({i * 50.0f, j * 50.0f, 0.0f});
        VS::BindUniformBuffer(pass, host_buffer->EmplaceUniform(uniforms));
        if (!pass.Draw().ok()) {
          return false;
        }
      }
    }

    host_buffer->Reset();
    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, CanRenderToTexture) {
  using VS = BoxFadeVertexShader;
  using FS = BoxFadeFragmentShader;
  auto context = GetContext();
  ASSERT_TRUE(context);
  using BoxPipelineBuilder = PipelineBuilder<VS, FS>;
  auto pipeline_desc =
      BoxPipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  pipeline_desc->SetSampleCount(SampleCount::kCount1);
  pipeline_desc->ClearDepthAttachment();
  pipeline_desc->SetStencilPixelFormat(PixelFormat::kS8UInt);

  ASSERT_TRUE(pipeline_desc.has_value());
  auto box_pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_desc).Get();
  ASSERT_TRUE(box_pipeline);
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());

  VertexBufferBuilder<VS::PerVertexData> vertex_builder;
  vertex_builder.SetLabel("Box");
  vertex_builder.AddVertices({
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 100, 0.0}, {1.0, 0.0}},  // 2
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 800, 0.0}, {0.0, 1.0}},  // 4
  });
  auto vertex_buffer =
      vertex_builder.CreateVertexBuffer(*context->GetResourceAllocator());
  ASSERT_TRUE(vertex_buffer);

  auto bridge = CreateTextureForFixture("bay_bridge.jpg");
  auto boston = CreateTextureForFixture("boston.jpg");
  ASSERT_TRUE(bridge && boston);
  raw_ptr<const Sampler> sampler = context->GetSamplerLibrary()->GetSampler({});
  ASSERT_TRUE(sampler);

  std::shared_ptr<RenderPass> r2t_pass;
  auto cmd_buffer = context->CreateCommandBuffer();
  ASSERT_TRUE(cmd_buffer);
  {
    ColorAttachment color0;
    color0.load_action = LoadAction::kClear;
    color0.store_action = StoreAction::kStore;

    TextureDescriptor texture_descriptor;
    ASSERT_NE(pipeline_desc->GetColorAttachmentDescriptor(0u), nullptr);
    texture_descriptor.format =
        pipeline_desc->GetColorAttachmentDescriptor(0u)->format;
    texture_descriptor.storage_mode = StorageMode::kHostVisible;
    texture_descriptor.size = {400, 400};
    texture_descriptor.mip_count = 1u;
    texture_descriptor.usage = TextureUsage::kRenderTarget;

    color0.texture =
        context->GetResourceAllocator()->CreateTexture(texture_descriptor);

    ASSERT_TRUE(color0.IsValid());

    color0.texture->SetLabel("r2t_target");

    StencilAttachment stencil0;
    stencil0.load_action = LoadAction::kClear;
    stencil0.store_action = StoreAction::kDontCare;
    TextureDescriptor stencil_texture_desc;
    stencil_texture_desc.storage_mode = StorageMode::kDeviceTransient;
    stencil_texture_desc.size = texture_descriptor.size;
    stencil_texture_desc.format = PixelFormat::kS8UInt;
    stencil_texture_desc.usage = TextureUsage::kRenderTarget;
    stencil0.texture =
        context->GetResourceAllocator()->CreateTexture(stencil_texture_desc);

    RenderTarget r2t_desc;
    r2t_desc.SetColorAttachment(color0, 0u);
    r2t_desc.SetStencilAttachment(stencil0);
    r2t_pass = cmd_buffer->CreateRenderPass(r2t_desc);
    ASSERT_TRUE(r2t_pass && r2t_pass->IsValid());
  }

  r2t_pass->SetCommandLabel("Box");
  r2t_pass->SetPipeline(box_pipeline);
  r2t_pass->SetVertexBuffer(vertex_buffer);

  FS::FrameInfo frame_info;
  frame_info.current_time = GetSecondsElapsed();
  frame_info.cursor_position = GetCursorPosition();
  frame_info.window_size.x = GetWindowSize().width;
  frame_info.window_size.y = GetWindowSize().height;

  FS::BindFrameInfo(*r2t_pass, host_buffer->EmplaceUniform(frame_info));
  FS::BindContents1(*r2t_pass, boston, sampler);
  FS::BindContents2(*r2t_pass, bridge, sampler);

  VS::UniformBuffer uniforms;
  uniforms.mvp = Matrix::MakeOrthographic(ISize{1024, 768}) *
                 Matrix::MakeTranslation({50.0f, 50.0f, 0.0f});
  VS::BindUniformBuffer(*r2t_pass, host_buffer->EmplaceUniform(uniforms));
  ASSERT_TRUE(r2t_pass->Draw().ok());
  ASSERT_TRUE(r2t_pass->EncodeCommands());
}

TEST_P(RendererTest, CanRenderInstanced) {
  if (GetParam() == PlaygroundBackend::kOpenGLES) {
    GTEST_SKIP() << "Instancing is not supported on OpenGL.";
  }
  using VS = InstancedDrawVertexShader;
  using FS = InstancedDrawFragmentShader;

  VertexBufferBuilder<VS::PerVertexData> builder;
  builder.AddVertices({
      VS::PerVertexData{Point{10, 10}},
      VS::PerVertexData{Point{10, 110}},
      VS::PerVertexData{Point{110, 10}},
      VS::PerVertexData{Point{10, 110}},
      VS::PerVertexData{Point{110, 10}},
      VS::PerVertexData{Point{110, 110}},
  });

  ASSERT_NE(GetContext(), nullptr);
  auto pipeline =
      GetContext()
          ->GetPipelineLibrary()
          ->GetPipeline(PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(
                            *GetContext())
                            ->SetSampleCount(SampleCount::kCount4)
                            .SetStencilAttachmentDescriptors(std::nullopt))

          .Get();
  ASSERT_TRUE(pipeline && pipeline->IsValid());

  static constexpr size_t kInstancesCount = 5u;
  VS::InstanceInfo<kInstancesCount> instances;
  for (size_t i = 0; i < kInstancesCount; i++) {
    instances.colors[i] = Color::Random();
  }

  auto host_buffer = HostBuffer::Create(GetContext()->GetResourceAllocator(),
                                        GetContext()->GetIdleWaiter());
  ASSERT_TRUE(OpenPlaygroundHere([&](RenderPass& pass) -> bool {
    pass.SetPipeline(pipeline);
    pass.SetCommandLabel("InstancedDraw");

    VS::FrameInfo frame_info;
    EXPECT_EQ(pass.GetOrthographicTransform(),
              Matrix::MakeOrthographic(pass.GetRenderTargetSize()));
    frame_info.mvp =
        pass.GetOrthographicTransform() * Matrix::MakeScale(GetContentScale());
    VS::BindFrameInfo(pass, host_buffer->EmplaceUniform(frame_info));
    VS::BindInstanceInfo(pass, host_buffer->EmplaceStorageBuffer(instances));
    pass.SetVertexBuffer(builder.CreateVertexBuffer(*host_buffer));

    pass.SetInstanceCount(kInstancesCount);
    pass.Draw();

    host_buffer->Reset();
    return true;
  }));
}

TEST_P(RendererTest, CanBlitTextureToTexture) {
  if (GetBackend() == PlaygroundBackend::kOpenGLES) {
    GTEST_SKIP() << "Mipmap test shader not supported on GLES.";
  }
  auto context = GetContext();
  ASSERT_TRUE(context);

  using VS = MipmapsVertexShader;
  using FS = MipmapsFragmentShader;
  auto desc = PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(desc.has_value());
  desc->SetSampleCount(SampleCount::kCount4);
  desc->SetStencilAttachmentDescriptors(std::nullopt);
  auto mipmaps_pipeline =
      context->GetPipelineLibrary()->GetPipeline(std::move(desc)).Get();
  ASSERT_TRUE(mipmaps_pipeline);

  TextureDescriptor texture_desc;
  texture_desc.storage_mode = StorageMode::kHostVisible;
  texture_desc.format = PixelFormat::kR8G8B8A8UNormInt;
  texture_desc.size = {800, 600};
  texture_desc.mip_count = 1u;
  texture_desc.usage = TextureUsage::kRenderTarget | TextureUsage::kShaderRead;
  auto texture = context->GetResourceAllocator()->CreateTexture(texture_desc);
  ASSERT_TRUE(texture);

  auto bridge = CreateTextureForFixture("bay_bridge.jpg");
  auto boston = CreateTextureForFixture("boston.jpg");
  ASSERT_TRUE(bridge && boston);
  raw_ptr<const Sampler> sampler = context->GetSamplerLibrary()->GetSampler({});
  ASSERT_TRUE(sampler);

  // Vertex buffer.
  VertexBufferBuilder<VS::PerVertexData> vertex_builder;
  vertex_builder.SetLabel("Box");
  auto size = Point(boston->GetSize());
  vertex_builder.AddVertices({
      {{0, 0}, {0.0, 0.0}},            // 1
      {{size.x, 0}, {1.0, 0.0}},       // 2
      {{size.x, size.y}, {1.0, 1.0}},  // 3
      {{0, 0}, {0.0, 0.0}},            // 1
      {{size.x, size.y}, {1.0, 1.0}},  // 3
      {{0, size.y}, {0.0, 1.0}},       // 4
  });
  auto vertex_buffer =
      vertex_builder.CreateVertexBuffer(*context->GetResourceAllocator());
  ASSERT_TRUE(vertex_buffer);

  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  Playground::RenderCallback callback = [&](RenderTarget& render_target) {
    auto buffer = context->CreateCommandBuffer();
    if (!buffer) {
      return false;
    }
    buffer->SetLabel("Playground Command Buffer");

    {
      auto pass = buffer->CreateBlitPass();
      if (!pass) {
        return false;
      }
      pass->SetLabel("Playground Blit Pass");

      // Blit `bridge` to the top left corner of the texture.
      pass->AddCopy(bridge, texture);

      if (!pass->EncodeCommands()) {
        return false;
      }
    }

    {
      auto pass = buffer->CreateRenderPass(render_target);
      if (!pass) {
        return false;
      }
      pass->SetLabel("Playground Render Pass");
      {
        pass->SetCommandLabel("Image");
        pass->SetPipeline(mipmaps_pipeline);
        pass->SetVertexBuffer(vertex_buffer);

        VS::FrameInfo frame_info;
        EXPECT_EQ(pass->GetOrthographicTransform(),
                  Matrix::MakeOrthographic(pass->GetRenderTargetSize()));
        frame_info.mvp = pass->GetOrthographicTransform() *
                         Matrix::MakeScale(GetContentScale());
        VS::BindFrameInfo(*pass, host_buffer->EmplaceUniform(frame_info));

        FS::FragInfo frag_info;
        frag_info.lod = 0;
        FS::BindFragInfo(*pass, host_buffer->EmplaceUniform(frag_info));

        auto sampler = context->GetSamplerLibrary()->GetSampler({});
        FS::BindTex(*pass, texture, sampler);

        pass->Draw();
      }
      pass->EncodeCommands();
    }

    if (!context->GetCommandQueue()->Submit({buffer}).ok()) {
      return false;
    }
    host_buffer->Reset();
    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, CanBlitTextureToBuffer) {
  if (GetBackend() == PlaygroundBackend::kOpenGLES) {
    GTEST_SKIP() << "Mipmap test shader not supported on GLES.";
  }
  auto context = GetContext();
  ASSERT_TRUE(context);

  using VS = MipmapsVertexShader;
  using FS = MipmapsFragmentShader;
  auto desc = PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(desc.has_value());
  desc->SetSampleCount(SampleCount::kCount4);
  desc->SetStencilAttachmentDescriptors(std::nullopt);
  auto mipmaps_pipeline =
      context->GetPipelineLibrary()->GetPipeline(std::move(desc)).Get();
  ASSERT_TRUE(mipmaps_pipeline);

  auto bridge = CreateTextureForFixture("bay_bridge.jpg");
  auto boston = CreateTextureForFixture("boston.jpg");
  ASSERT_TRUE(bridge && boston);
  raw_ptr<const Sampler> sampler = context->GetSamplerLibrary()->GetSampler({});
  ASSERT_TRUE(sampler);

  TextureDescriptor texture_desc;
  texture_desc.storage_mode = StorageMode::kHostVisible;
  texture_desc.format = PixelFormat::kR8G8B8A8UNormInt;
  texture_desc.size = bridge->GetTextureDescriptor().size;
  texture_desc.mip_count = 1u;
  texture_desc.usage = TextureUsage::kRenderTarget |
                       TextureUsage::kShaderWrite | TextureUsage::kShaderRead;
  DeviceBufferDescriptor device_buffer_desc;
  device_buffer_desc.storage_mode = StorageMode::kHostVisible;
  device_buffer_desc.size =
      bridge->GetTextureDescriptor().GetByteSizeOfBaseMipLevel();
  auto device_buffer =
      context->GetResourceAllocator()->CreateBuffer(device_buffer_desc);

  // Vertex buffer.
  VertexBufferBuilder<VS::PerVertexData> vertex_builder;
  vertex_builder.SetLabel("Box");
  auto size = Point(boston->GetSize());
  vertex_builder.AddVertices({
      {{0, 0}, {0.0, 0.0}},            // 1
      {{size.x, 0}, {1.0, 0.0}},       // 2
      {{size.x, size.y}, {1.0, 1.0}},  // 3
      {{0, 0}, {0.0, 0.0}},            // 1
      {{size.x, size.y}, {1.0, 1.0}},  // 3
      {{0, size.y}, {0.0, 1.0}},       // 4
  });
  auto vertex_buffer =
      vertex_builder.CreateVertexBuffer(*context->GetResourceAllocator());
  ASSERT_TRUE(vertex_buffer);

  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  Playground::RenderCallback callback = [&](RenderTarget& render_target) {
    {
      auto buffer = context->CreateCommandBuffer();
      if (!buffer) {
        return false;
      }
      buffer->SetLabel("Playground Command Buffer");
      auto pass = buffer->CreateBlitPass();
      if (!pass) {
        return false;
      }
      pass->SetLabel("Playground Blit Pass");

      // Blit `bridge` to the top left corner of the texture.
      pass->AddCopy(bridge, device_buffer);
      pass->EncodeCommands();

      if (!context->GetCommandQueue()->Submit({buffer}).ok()) {
        return false;
      }
    }

    {
      auto buffer = context->CreateCommandBuffer();
      if (!buffer) {
        return false;
      }
      buffer->SetLabel("Playground Command Buffer");

      auto pass = buffer->CreateRenderPass(render_target);
      if (!pass) {
        return false;
      }
      pass->SetLabel("Playground Render Pass");
      {
        pass->SetCommandLabel("Image");
        pass->SetPipeline(mipmaps_pipeline);
        pass->SetVertexBuffer(vertex_buffer);

        VS::FrameInfo frame_info;
        EXPECT_EQ(pass->GetOrthographicTransform(),
                  Matrix::MakeOrthographic(pass->GetRenderTargetSize()));
        frame_info.mvp = pass->GetOrthographicTransform() *
                         Matrix::MakeScale(GetContentScale());
        VS::BindFrameInfo(*pass, host_buffer->EmplaceUniform(frame_info));

        FS::FragInfo frag_info;
        frag_info.lod = 0;
        FS::BindFragInfo(*pass, host_buffer->EmplaceUniform(frag_info));

        raw_ptr<const Sampler> sampler =
            context->GetSamplerLibrary()->GetSampler({});
        auto buffer_view = DeviceBuffer::AsBufferView(device_buffer);
        auto texture =
            context->GetResourceAllocator()->CreateTexture(texture_desc);
        if (!texture->SetContents(device_buffer->OnGetContents(),
                                  buffer_view.GetRange().length)) {
          VALIDATION_LOG << "Could not upload texture to device memory";
          return false;
        }
        FS::BindTex(*pass, texture, sampler);

        pass->Draw().ok();
      }
      pass->EncodeCommands();
      if (!context->GetCommandQueue()->Submit({buffer}).ok()) {
        return false;
      }
    }
    host_buffer->Reset();
    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, CanGenerateMipmaps) {
  if (GetBackend() == PlaygroundBackend::kOpenGLES) {
    GTEST_SKIP() << "Mipmap test shader not supported on GLES.";
  }
  auto context = GetContext();
  ASSERT_TRUE(context);

  using VS = MipmapsVertexShader;
  using FS = MipmapsFragmentShader;
  auto desc = PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(desc.has_value());
  desc->SetSampleCount(SampleCount::kCount4);
  desc->SetStencilAttachmentDescriptors(std::nullopt);
  auto mipmaps_pipeline =
      context->GetPipelineLibrary()->GetPipeline(std::move(desc)).Get();
  ASSERT_TRUE(mipmaps_pipeline);

  auto boston = CreateTextureForFixture("boston.jpg", true);
  ASSERT_TRUE(boston);

  // Vertex buffer.
  VertexBufferBuilder<VS::PerVertexData> vertex_builder;
  vertex_builder.SetLabel("Box");
  auto size = Point(boston->GetSize());
  vertex_builder.AddVertices({
      {{0, 0}, {0.0, 0.0}},            // 1
      {{size.x, 0}, {1.0, 0.0}},       // 2
      {{size.x, size.y}, {1.0, 1.0}},  // 3
      {{0, 0}, {0.0, 0.0}},            // 1
      {{size.x, size.y}, {1.0, 1.0}},  // 3
      {{0, size.y}, {0.0, 1.0}},       // 4
  });
  auto vertex_buffer =
      vertex_builder.CreateVertexBuffer(*context->GetResourceAllocator());
  ASSERT_TRUE(vertex_buffer);

  bool first_frame = true;
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  Playground::RenderCallback callback = [&](RenderTarget& render_target) {
    const char* mip_filter_names[] = {"Base", "Nearest", "Linear"};
    const MipFilter mip_filters[] = {MipFilter::kBase, MipFilter::kNearest,
                                     MipFilter::kLinear};
    const char* min_filter_names[] = {"Nearest", "Linear"};
    const MinMagFilter min_filters[] = {MinMagFilter::kNearest,
                                        MinMagFilter::kLinear};

    // UI state.
    static int selected_mip_filter = 1;
    static int selected_min_filter = 0;
    static float lod = 4.5;

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Combo("Mip filter", &selected_mip_filter, mip_filter_names,
                 sizeof(mip_filter_names) / sizeof(char*));
    ImGui::Combo("Min filter", &selected_min_filter, min_filter_names,
                 sizeof(min_filter_names) / sizeof(char*));
    ImGui::SliderFloat("LOD", &lod, 0, boston->GetMipCount() - 1);
    ImGui::End();

    auto buffer = context->CreateCommandBuffer();
    if (!buffer) {
      return false;
    }
    buffer->SetLabel("Playground Command Buffer");

    if (first_frame) {
      auto pass = buffer->CreateBlitPass();
      if (!pass) {
        return false;
      }
      pass->SetLabel("Playground Blit Pass");

      pass->GenerateMipmap(boston, "Boston Mipmap");

      pass->EncodeCommands();
    }

    first_frame = false;

    {
      auto pass = buffer->CreateRenderPass(render_target);
      if (!pass) {
        return false;
      }
      pass->SetLabel("Playground Render Pass");
      {
        pass->SetCommandLabel("Image LOD");
        pass->SetPipeline(mipmaps_pipeline);
        pass->SetVertexBuffer(vertex_buffer);

        VS::FrameInfo frame_info;
        EXPECT_EQ(pass->GetOrthographicTransform(),
                  Matrix::MakeOrthographic(pass->GetRenderTargetSize()));
        frame_info.mvp = pass->GetOrthographicTransform() *
                         Matrix::MakeScale(GetContentScale());
        VS::BindFrameInfo(*pass, host_buffer->EmplaceUniform(frame_info));

        FS::FragInfo frag_info;
        frag_info.lod = lod;
        FS::BindFragInfo(*pass, host_buffer->EmplaceUniform(frag_info));

        SamplerDescriptor sampler_desc;
        sampler_desc.mip_filter = mip_filters[selected_mip_filter];
        sampler_desc.min_filter = min_filters[selected_min_filter];
        raw_ptr<const Sampler> sampler =
            context->GetSamplerLibrary()->GetSampler(sampler_desc);
        FS::BindTex(*pass, boston, sampler);

        pass->Draw();
      }
      pass->EncodeCommands();
    }

    if (!context->GetCommandQueue()->Submit({buffer}).ok()) {
      return false;
    }
    host_buffer->Reset();
    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, TheImpeller) {
  using VS = ImpellerVertexShader;
  using FS = ImpellerFragmentShader;

  auto context = GetContext();
  auto pipeline_descriptor =
      PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_descriptor.has_value());
  pipeline_descriptor->SetSampleCount(SampleCount::kCount4);
  pipeline_descriptor->SetStencilAttachmentDescriptors(std::nullopt);
  auto pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_descriptor).Get();
  ASSERT_TRUE(pipeline && pipeline->IsValid());

  auto blue_noise = CreateTextureForFixture("blue_noise.png");
  SamplerDescriptor noise_sampler_desc;
  noise_sampler_desc.width_address_mode = SamplerAddressMode::kRepeat;
  noise_sampler_desc.height_address_mode = SamplerAddressMode::kRepeat;
  raw_ptr<const Sampler> noise_sampler =
      context->GetSamplerLibrary()->GetSampler(noise_sampler_desc);

  auto cube_map = CreateTextureCubeForFixture(
      {"table_mountain_px.png", "table_mountain_nx.png",
       "table_mountain_py.png", "table_mountain_ny.png",
       "table_mountain_pz.png", "table_mountain_nz.png"});
  raw_ptr<const Sampler> cube_map_sampler =
      context->GetSamplerLibrary()->GetSampler({});
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());

  SinglePassCallback callback = [&](RenderPass& pass) {
    auto size = pass.GetRenderTargetSize();

    pass.SetPipeline(pipeline);
    pass.SetCommandLabel("Impeller SDF scene");
    VertexBufferBuilder<VS::PerVertexData> builder;
    builder.AddVertices({{Point()},
                         {Point(0, size.height)},
                         {Point(size.width, 0)},
                         {Point(size.width, 0)},
                         {Point(0, size.height)},
                         {Point(size.width, size.height)}});
    pass.SetVertexBuffer(builder.CreateVertexBuffer(*host_buffer));

    VS::FrameInfo frame_info;
    EXPECT_EQ(pass.GetOrthographicTransform(), Matrix::MakeOrthographic(size));
    frame_info.mvp = pass.GetOrthographicTransform();
    VS::BindFrameInfo(pass, host_buffer->EmplaceUniform(frame_info));

    FS::FragInfo fs_uniform;
    fs_uniform.texture_size = Point(size);
    fs_uniform.time = GetSecondsElapsed();
    FS::BindFragInfo(pass, host_buffer->EmplaceUniform(fs_uniform));
    FS::BindBlueNoise(pass, blue_noise, noise_sampler);
    FS::BindCubeMap(pass, cube_map, cube_map_sampler);

    pass.Draw().ok();
    host_buffer->Reset();
    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, Planet) {
  using VS = PlanetVertexShader;
  using FS = PlanetFragmentShader;

  auto context = GetContext();
  auto pipeline_descriptor =
      PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_descriptor.has_value());
  pipeline_descriptor->SetSampleCount(SampleCount::kCount4);
  pipeline_descriptor->SetStencilAttachmentDescriptors(std::nullopt);
  auto pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_descriptor).Get();
  ASSERT_TRUE(pipeline && pipeline->IsValid());

  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());

  SinglePassCallback callback = [&](RenderPass& pass) {
    static Scalar speed = 0.1;
    static Scalar planet_size = 550.0;
    static bool show_normals = false;
    static bool show_noise = false;
    static Scalar seed_value = 42.0;

    auto size = pass.GetRenderTargetSize();

    ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::SliderFloat("Speed", &speed, 0.0, 10.0);
    ImGui::SliderFloat("Planet Size", &planet_size, 0.1, 1000);
    ImGui::Checkbox("Show Normals", &show_normals);
    ImGui::Checkbox("Show Noise", &show_noise);
    ImGui::InputFloat("Seed Value", &seed_value);
    ImGui::End();

    pass.SetPipeline(pipeline);
    pass.SetCommandLabel("Planet scene");
    VertexBufferBuilder<VS::PerVertexData> builder;
    builder.AddVertices({{Point()},
                         {Point(0, size.height)},
                         {Point(size.width, 0)},
                         {Point(size.width, 0)},
                         {Point(0, size.height)},
                         {Point(size.width, size.height)}});
    pass.SetVertexBuffer(builder.CreateVertexBuffer(*host_buffer));

    VS::FrameInfo frame_info;
    EXPECT_EQ(pass.GetOrthographicTransform(), Matrix::MakeOrthographic(size));
    frame_info.mvp = pass.GetOrthographicTransform();
    VS::BindFrameInfo(pass, host_buffer->EmplaceUniform(frame_info));

    FS::FragInfo fs_uniform;
    fs_uniform.resolution = Point(size);
    fs_uniform.time = GetSecondsElapsed();
    fs_uniform.speed = speed;
    fs_uniform.planet_size = planet_size;
    fs_uniform.show_normals = show_normals ? 1.0 : 0.0;
    fs_uniform.show_noise = show_noise ? 1.0 : 0.0;
    fs_uniform.seed_value = seed_value;
    FS::BindFragInfo(pass, host_buffer->EmplaceUniform(fs_uniform));

    pass.Draw().ok();
    host_buffer->Reset();
    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, ArrayUniforms) {
  using VS = ArrayVertexShader;
  using FS = ArrayFragmentShader;

  auto context = GetContext();
  auto pipeline_descriptor =
      PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_descriptor.has_value());
  pipeline_descriptor->SetSampleCount(SampleCount::kCount4);
  pipeline_descriptor->SetStencilAttachmentDescriptors(std::nullopt);
  auto pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_descriptor).Get();
  ASSERT_TRUE(pipeline && pipeline->IsValid());

  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  SinglePassCallback callback = [&](RenderPass& pass) {
    auto size = pass.GetRenderTargetSize();

    pass.SetPipeline(pipeline);
    pass.SetCommandLabel("Google Dots");
    VertexBufferBuilder<VS::PerVertexData> builder;
    builder.AddVertices({{Point()},
                         {Point(0, size.height)},
                         {Point(size.width, 0)},
                         {Point(size.width, 0)},
                         {Point(0, size.height)},
                         {Point(size.width, size.height)}});
    pass.SetVertexBuffer(builder.CreateVertexBuffer(*host_buffer));

    VS::FrameInfo frame_info;
    EXPECT_EQ(pass.GetOrthographicTransform(), Matrix::MakeOrthographic(size));
    frame_info.mvp =
        pass.GetOrthographicTransform() * Matrix::MakeScale(GetContentScale());
    VS::BindFrameInfo(pass, host_buffer->EmplaceUniform(frame_info));

    auto time = GetSecondsElapsed();
    auto y_pos = [&time](float x) {
      return 400 + 10 * std::cos(time * 5 + x / 6);
    };

    FS::FragInfo fs_uniform = {
        .circle_positions = {Point(430, y_pos(0)), Point(480, y_pos(1)),
                             Point(530, y_pos(2)), Point(580, y_pos(3))},
        .colors = {Color::MakeRGBA8(66, 133, 244, 255),
                   Color::MakeRGBA8(219, 68, 55, 255),
                   Color::MakeRGBA8(244, 180, 0, 255),
                   Color::MakeRGBA8(15, 157, 88, 255)},
    };
    FS::BindFragInfo(pass, host_buffer->EmplaceUniform(fs_uniform));

    pass.Draw();
    host_buffer->Reset();
    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, InactiveUniforms) {
  using VS = InactiveUniformsVertexShader;
  using FS = InactiveUniformsFragmentShader;

  auto context = GetContext();
  auto pipeline_descriptor =
      PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_descriptor.has_value());
  pipeline_descriptor->SetSampleCount(SampleCount::kCount4);
  pipeline_descriptor->SetStencilAttachmentDescriptors(std::nullopt);
  auto pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_descriptor).Get();
  ASSERT_TRUE(pipeline && pipeline->IsValid());

  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  SinglePassCallback callback = [&](RenderPass& pass) {
    auto size = pass.GetRenderTargetSize();

    pass.SetPipeline(pipeline);
    pass.SetCommandLabel("Inactive Uniform");

    VertexBufferBuilder<VS::PerVertexData> builder;
    builder.AddVertices({{Point()},
                         {Point(0, size.height)},
                         {Point(size.width, 0)},
                         {Point(size.width, 0)},
                         {Point(0, size.height)},
                         {Point(size.width, size.height)}});
    pass.SetVertexBuffer(builder.CreateVertexBuffer(*host_buffer));

    VS::FrameInfo frame_info;
    EXPECT_EQ(pass.GetOrthographicTransform(), Matrix::MakeOrthographic(size));
    frame_info.mvp =
        pass.GetOrthographicTransform() * Matrix::MakeScale(GetContentScale());
    VS::BindFrameInfo(pass, host_buffer->EmplaceUniform(frame_info));

    FS::FragInfo fs_uniform = {.unused_color = Color::Red(),
                               .color = Color::Green()};
    FS::BindFragInfo(pass, host_buffer->EmplaceUniform(fs_uniform));

    pass.Draw().ok();
    host_buffer->Reset();
    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, DefaultIndexSize) {
  using VS = BoxFadeVertexShader;

  // Default to 16bit index buffer size, as this is a reasonable default and
  // supported on all backends without extensions.
  VertexBufferBuilder<VS::PerVertexData> vertex_builder;
  vertex_builder.AppendIndex(0u);
  ASSERT_EQ(vertex_builder.GetIndexType(), IndexType::k16bit);
}

TEST_P(RendererTest, DefaultIndexBehavior) {
  using VS = BoxFadeVertexShader;

  // Do not create any index buffer if no indices were provided.
  VertexBufferBuilder<VS::PerVertexData> vertex_builder;
  ASSERT_EQ(vertex_builder.GetIndexType(), IndexType::kNone);
}

TEST_P(RendererTest, VertexBufferBuilder) {
  // Does not create index buffer if one is provided.
  using VS = BoxFadeVertexShader;
  VertexBufferBuilder<VS::PerVertexData> vertex_builder;
  vertex_builder.SetLabel("Box");
  vertex_builder.AddVertices({
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 100, 0.0}, {1.0, 0.0}},  // 2
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 800, 0.0}, {0.0, 1.0}},  // 4
  });
  vertex_builder.AppendIndex(0);
  vertex_builder.AppendIndex(1);
  vertex_builder.AppendIndex(2);
  vertex_builder.AppendIndex(1);
  vertex_builder.AppendIndex(2);
  vertex_builder.AppendIndex(3);

  ASSERT_EQ(vertex_builder.GetIndexCount(), 6u);
  ASSERT_EQ(vertex_builder.GetVertexCount(), 4u);
}

class CompareFunctionUIData {
 public:
  CompareFunctionUIData() {
    labels_.push_back("Never");
    functions_.push_back(CompareFunction::kNever);
    labels_.push_back("Always");
    functions_.push_back(CompareFunction::kAlways);
    labels_.push_back("Less");
    functions_.push_back(CompareFunction::kLess);
    labels_.push_back("Equal");
    functions_.push_back(CompareFunction::kEqual);
    labels_.push_back("LessEqual");
    functions_.push_back(CompareFunction::kLessEqual);
    labels_.push_back("Greater");
    functions_.push_back(CompareFunction::kGreater);
    labels_.push_back("NotEqual");
    functions_.push_back(CompareFunction::kNotEqual);
    labels_.push_back("GreaterEqual");
    functions_.push_back(CompareFunction::kGreaterEqual);
    assert(labels_.size() == functions_.size());
  }

  const char* const* labels() const { return &labels_[0]; }

  int size() const { return labels_.size(); }

  int IndexOf(CompareFunction func) const {
    for (size_t i = 0; i < functions_.size(); i++) {
      if (functions_[i] == func) {
        return i;
      }
    }
    FML_UNREACHABLE();
    return -1;
  }

  CompareFunction FunctionOf(int index) const { return functions_[index]; }

 private:
  std::vector<const char*> labels_;
  std::vector<CompareFunction> functions_;
};

static const CompareFunctionUIData& CompareFunctionUI() {
  static CompareFunctionUIData data;
  return data;
}

TEST_P(RendererTest, StencilMask) {
  using VS = BoxFadeVertexShader;
  using FS = BoxFadeFragmentShader;
  auto context = GetContext();
  ASSERT_TRUE(context);
  using BoxFadePipelineBuilder = PipelineBuilder<VS, FS>;
  auto desc = BoxFadePipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(desc.has_value());

  // Vertex buffer.
  VertexBufferBuilder<VS::PerVertexData> vertex_builder;
  vertex_builder.SetLabel("Box");
  vertex_builder.AddVertices({
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 100, 0.0}, {1.0, 0.0}},  // 2
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 800, 0.0}, {0.0, 1.0}},  // 4
  });
  auto vertex_buffer =
      vertex_builder.CreateVertexBuffer(*context->GetResourceAllocator());
  ASSERT_TRUE(vertex_buffer);

  desc->SetSampleCount(SampleCount::kCount4);
  desc->SetStencilAttachmentDescriptors(std::nullopt);

  auto bridge = CreateTextureForFixture("bay_bridge.jpg");
  auto boston = CreateTextureForFixture("boston.jpg");
  ASSERT_TRUE(bridge && boston);
  raw_ptr<const Sampler> sampler = context->GetSamplerLibrary()->GetSampler({});
  ASSERT_TRUE(sampler);

  static bool mirror = false;
  static int stencil_reference_write = 0xFF;
  static int stencil_reference_read = 0x1;
  std::vector<uint8_t> stencil_contents;
  static int last_stencil_contents_reference_value = 0;
  static int current_front_compare =
      CompareFunctionUI().IndexOf(CompareFunction::kLessEqual);
  static int current_back_compare =
      CompareFunctionUI().IndexOf(CompareFunction::kLessEqual);

  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  Playground::RenderCallback callback = [&](RenderTarget& render_target) {
    auto buffer = context->CreateCommandBuffer();
    if (!buffer) {
      return false;
    }
    buffer->SetLabel("Playground Command Buffer");

    {
      // Configure the stencil attachment for the test.
      RenderTarget::AttachmentConfig stencil_config;
      stencil_config.load_action = LoadAction::kLoad;
      stencil_config.store_action = StoreAction::kDontCare;
      stencil_config.storage_mode = StorageMode::kHostVisible;
      render_target.SetupDepthStencilAttachments(
          *context, *context->GetResourceAllocator(),
          render_target.GetRenderTargetSize(), true, "stencil", stencil_config);
      // Fill the stencil buffer with an checkerboard pattern.
      const auto target_width = render_target.GetRenderTargetSize().width;
      const auto target_height = render_target.GetRenderTargetSize().height;
      const size_t target_size = target_width * target_height;
      if (stencil_contents.size() != target_size ||
          last_stencil_contents_reference_value != stencil_reference_write) {
        stencil_contents.resize(target_size);
        last_stencil_contents_reference_value = stencil_reference_write;
        for (int y = 0; y < target_height; y++) {
          for (int x = 0; x < target_width; x++) {
            const auto index = y * target_width + x;
            const auto kCheckSize = 64;
            const auto value =
                (((y / kCheckSize) + (x / kCheckSize)) % 2 == 0) *
                stencil_reference_write;
            stencil_contents[index] = value;
          }
        }
      }
      if (!render_target.GetStencilAttachment()->texture->SetContents(
              stencil_contents.data(), stencil_contents.size(), 0, false)) {
        VALIDATION_LOG << "Could not upload stencil contents to device memory";
        return false;
      }
      auto pass = buffer->CreateRenderPass(render_target);
      if (!pass) {
        return false;
      }
      pass->SetLabel("Stencil Buffer");
      ImGui::Begin("Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::SliderInt("Stencil Write Value", &stencil_reference_write, 0,
                       0xFF);
      ImGui::SliderInt("Stencil Compare Value", &stencil_reference_read, 0,
                       0xFF);
      ImGui::Checkbox("Back face mode", &mirror);
      ImGui::ListBox("Front face compare function", &current_front_compare,
                     CompareFunctionUI().labels(), CompareFunctionUI().size());
      ImGui::ListBox("Back face compare function", &current_back_compare,
                     CompareFunctionUI().labels(), CompareFunctionUI().size());
      ImGui::End();

      StencilAttachmentDescriptor front;
      front.stencil_compare =
          CompareFunctionUI().FunctionOf(current_front_compare);
      StencilAttachmentDescriptor back;
      back.stencil_compare =
          CompareFunctionUI().FunctionOf(current_back_compare);
      desc->SetStencilAttachmentDescriptors(front, back);
      auto pipeline = context->GetPipelineLibrary()->GetPipeline(desc).Get();

      assert(pipeline && pipeline->IsValid());

      pass->SetCommandLabel("Box");
      pass->SetPipeline(pipeline);
      pass->SetStencilReference(stencil_reference_read);
      pass->SetVertexBuffer(vertex_buffer);

      VS::UniformBuffer uniforms;
      EXPECT_EQ(pass->GetOrthographicTransform(),
                Matrix::MakeOrthographic(pass->GetRenderTargetSize()));
      uniforms.mvp = pass->GetOrthographicTransform() *
                     Matrix::MakeScale(GetContentScale());
      if (mirror) {
        uniforms.mvp = Matrix::MakeScale(Vector2(-1, 1)) * uniforms.mvp;
      }
      VS::BindUniformBuffer(*pass, host_buffer->EmplaceUniform(uniforms));

      FS::FrameInfo frame_info;
      frame_info.current_time = GetSecondsElapsed();
      frame_info.cursor_position = GetCursorPosition();
      frame_info.window_size.x = GetWindowSize().width;
      frame_info.window_size.y = GetWindowSize().height;

      FS::BindFrameInfo(*pass, host_buffer->EmplaceUniform(frame_info));
      FS::BindContents1(*pass, boston, sampler);
      FS::BindContents2(*pass, bridge, sampler);
      if (!pass->Draw().ok()) {
        return false;
      }
      pass->EncodeCommands();
    }

    if (!context->GetCommandQueue()->Submit({buffer}).ok()) {
      return false;
    }
    host_buffer->Reset();
    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, CanLookupRenderTargetProperties) {
  auto context = GetContext();
  auto cmd_buffer = context->CreateCommandBuffer();
  auto render_target_cache = std::make_shared<RenderTargetAllocator>(
      GetContext()->GetResourceAllocator());

  auto render_target = render_target_cache->CreateOffscreen(
      *context, {100, 100}, /*mip_count=*/1);
  auto render_pass = cmd_buffer->CreateRenderPass(render_target);

  EXPECT_EQ(render_pass->GetSampleCount(), render_target.GetSampleCount());
  EXPECT_EQ(render_pass->GetRenderTargetPixelFormat(),
            render_target.GetRenderTargetPixelFormat());
  EXPECT_EQ(render_pass->HasStencilAttachment(),
            render_target.GetStencilAttachment().has_value());
  EXPECT_EQ(render_pass->GetRenderTargetSize(),
            render_target.GetRenderTargetSize());
  render_pass->EncodeCommands();
}

TEST_P(RendererTest,
       RenderTargetCreateOffscreenMSAASetsDefaultDepthStencilFormat) {
  auto context = GetContext();
  auto render_target_cache = std::make_shared<RenderTargetAllocator>(
      GetContext()->GetResourceAllocator());

  RenderTarget render_target = render_target_cache->CreateOffscreenMSAA(
      *context, {100, 100}, /*mip_count=*/1);
  EXPECT_EQ(render_target.GetDepthAttachment()
                ->texture->GetTextureDescriptor()
                .format,
            GetContext()->GetCapabilities()->GetDefaultDepthStencilFormat());
}

template <class VertexShader, class FragmentShader>
std::shared_ptr<Pipeline<PipelineDescriptor>> CreateDefaultPipeline(
    const std::shared_ptr<Context>& context) {
  using TexturePipelineBuilder = PipelineBuilder<VertexShader, FragmentShader>;
  auto pipeline_desc =
      TexturePipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  if (!pipeline_desc.has_value()) {
    return nullptr;
  }
  pipeline_desc->SetSampleCount(SampleCount::kCount4);
  pipeline_desc->SetStencilAttachmentDescriptors(std::nullopt);
  auto pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_desc).Get();
  if (!pipeline || !pipeline->IsValid()) {
    return nullptr;
  }
  return pipeline;
}

TEST_P(RendererTest, CanSepiaToneWithSubpasses) {
  // Define shader types
  using TextureVS = TextureVertexShader;
  using TextureFS = TextureFragmentShader;

  using SepiaVS = SepiaVertexShader;
  using SepiaFS = SepiaFragmentShader;

  auto context = GetContext();
  ASSERT_TRUE(context);

  if (!context->GetCapabilities()->SupportsFramebufferFetch()) {
    GTEST_SKIP() << "This test uses framebuffer fetch and the backend doesn't "
                    "support it.";
    return;
  }

  // Create pipelines.
  auto texture_pipeline = CreateDefaultPipeline<TextureVS, TextureFS>(context);
  auto sepia_pipeline = CreateDefaultPipeline<SepiaVS, SepiaFS>(context);

  ASSERT_TRUE(texture_pipeline);
  ASSERT_TRUE(sepia_pipeline);

  // Vertex buffer builders.
  VertexBufferBuilder<TextureVS::PerVertexData> texture_vtx_builder;
  texture_vtx_builder.AddVertices({
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 100, 0.0}, {1.0, 0.0}},  // 2
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 800, 0.0}, {0.0, 1.0}},  // 4
  });

  VertexBufferBuilder<SepiaVS::PerVertexData> sepia_vtx_builder;
  sepia_vtx_builder.AddVertices({
      {{100, 100, 0.0}},  // 1
      {{800, 100, 0.0}},  // 2
      {{800, 800, 0.0}},  // 3
      {{100, 100, 0.0}},  // 1
      {{800, 800, 0.0}},  // 3
      {{100, 800, 0.0}},  // 4
  });

  auto boston = CreateTextureForFixture("boston.jpg");
  ASSERT_TRUE(boston);

  const auto& sampler = context->GetSamplerLibrary()->GetSampler({});
  ASSERT_TRUE(sampler);

  SinglePassCallback callback = [&](RenderPass& pass) {
    auto buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                     context->GetIdleWaiter());

    // Draw the texture.
    {
      pass.SetPipeline(texture_pipeline);
      pass.SetVertexBuffer(texture_vtx_builder.CreateVertexBuffer(
          *context->GetResourceAllocator()));
      TextureVS::UniformBuffer uniforms;
      uniforms.mvp = Matrix::MakeOrthographic(pass.GetRenderTargetSize()) *
                     Matrix::MakeScale(GetContentScale());
      TextureVS::BindUniformBuffer(pass, buffer->EmplaceUniform(uniforms));
      TextureFS::BindTextureContents(pass, boston, sampler);
      if (!pass.Draw().ok()) {
        return false;
      }
    }

    // Draw the sepia toner.
    {
      pass.SetPipeline(sepia_pipeline);
      pass.SetVertexBuffer(sepia_vtx_builder.CreateVertexBuffer(
          *context->GetResourceAllocator()));
      SepiaVS::UniformBuffer uniforms;
      uniforms.mvp = Matrix::MakeOrthographic(pass.GetRenderTargetSize()) *
                     Matrix::MakeScale(GetContentScale());
      SepiaVS::BindUniformBuffer(pass, buffer->EmplaceUniform(uniforms));
      if (!pass.Draw().ok()) {
        return false;
      }
    }

    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, CanSepiaToneThenSwizzleWithSubpasses) {
  // Define shader types
  using TextureVS = TextureVertexShader;
  using TextureFS = TextureFragmentShader;

  using SwizzleVS = SepiaVertexShader;
  using SwizzleFS = SwizzleFragmentShader;

  using SepiaVS = SepiaVertexShader;
  using SepiaFS = SepiaFragmentShader;

  auto context = GetContext();
  ASSERT_TRUE(context);

  if (!context->GetCapabilities()->SupportsFramebufferFetch()) {
    GTEST_SKIP() << "This test uses framebuffer fetch and the backend doesn't "
                    "support it.";
    return;
  }

  // Create pipelines.
  auto texture_pipeline = CreateDefaultPipeline<TextureVS, TextureFS>(context);
  auto swizzle_pipeline = CreateDefaultPipeline<SwizzleVS, SwizzleFS>(context);
  auto sepia_pipeline = CreateDefaultPipeline<SepiaVS, SepiaFS>(context);

  ASSERT_TRUE(texture_pipeline);
  ASSERT_TRUE(swizzle_pipeline);
  ASSERT_TRUE(sepia_pipeline);

  // Vertex buffer builders.
  VertexBufferBuilder<TextureVS::PerVertexData> texture_vtx_builder;
  texture_vtx_builder.AddVertices({
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 100, 0.0}, {1.0, 0.0}},  // 2
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 100, 0.0}, {0.0, 0.0}},  // 1
      {{800, 800, 0.0}, {1.0, 1.0}},  // 3
      {{100, 800, 0.0}, {0.0, 1.0}},  // 4
  });

  VertexBufferBuilder<SepiaVS::PerVertexData> sepia_vtx_builder;
  sepia_vtx_builder.AddVertices({
      {{100, 100, 0.0}},  // 1
      {{800, 100, 0.0}},  // 2
      {{800, 800, 0.0}},  // 3
      {{100, 100, 0.0}},  // 1
      {{800, 800, 0.0}},  // 3
      {{100, 800, 0.0}},  // 4
  });

  auto boston = CreateTextureForFixture("boston.jpg");
  ASSERT_TRUE(boston);

  const auto& sampler = context->GetSamplerLibrary()->GetSampler({});
  ASSERT_TRUE(sampler);

  SinglePassCallback callback = [&](RenderPass& pass) {
    auto buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                     context->GetIdleWaiter());

    // Draw the texture.
    {
      pass.SetPipeline(texture_pipeline);
      pass.SetVertexBuffer(texture_vtx_builder.CreateVertexBuffer(
          *context->GetResourceAllocator()));
      TextureVS::UniformBuffer uniforms;
      uniforms.mvp = Matrix::MakeOrthographic(pass.GetRenderTargetSize()) *
                     Matrix::MakeScale(GetContentScale());
      TextureVS::BindUniformBuffer(pass, buffer->EmplaceUniform(uniforms));
      TextureFS::BindTextureContents(pass, boston, sampler);
      if (!pass.Draw().ok()) {
        return false;
      }
    }

    // Draw the sepia toner.
    {
      pass.SetPipeline(sepia_pipeline);
      pass.SetVertexBuffer(sepia_vtx_builder.CreateVertexBuffer(
          *context->GetResourceAllocator()));
      SepiaVS::UniformBuffer uniforms;
      uniforms.mvp = Matrix::MakeOrthographic(pass.GetRenderTargetSize()) *
                     Matrix::MakeScale(GetContentScale());
      SepiaVS::BindUniformBuffer(pass, buffer->EmplaceUniform(uniforms));
      if (!pass.Draw().ok()) {
        return false;
      }
    }

    // Draw the swizzle.
    {
      pass.SetPipeline(swizzle_pipeline);
      pass.SetVertexBuffer(sepia_vtx_builder.CreateVertexBuffer(
          *context->GetResourceAllocator()));
      SwizzleVS::UniformBuffer uniforms;
      uniforms.mvp = Matrix::MakeOrthographic(pass.GetRenderTargetSize()) *
                     Matrix::MakeScale(GetContentScale());
      SwizzleVS::BindUniformBuffer(pass, buffer->EmplaceUniform(uniforms));
      if (!pass.Draw().ok()) {
        return false;
      }
    }

    return true;
  };
  OpenPlaygroundHere(callback);
}

TEST_P(RendererTest, BindingNullTexturesDoesNotCrash) {
  using FS = BoxFadeFragmentShader;

  auto context = GetContext();
  raw_ptr<const Sampler> sampler = context->GetSamplerLibrary()->GetSampler({});
  auto command_buffer = context->CreateCommandBuffer();

  RenderTargetAllocator allocator(context->GetResourceAllocator());
  RenderTarget target = allocator.CreateOffscreen(*context, {1, 1}, 1);

  auto pass = command_buffer->CreateRenderPass(target);
  EXPECT_FALSE(FS::BindContents2(*pass, nullptr, sampler));
}

}  // namespace testing
}  // namespace impeller

// NOLINTEND(bugprone-unchecked-optional-access)
