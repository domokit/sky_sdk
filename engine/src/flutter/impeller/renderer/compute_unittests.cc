// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/testing/testing.h"
#include "gmock/gmock.h"
#include "impeller/core/host_buffer.h"
#include "impeller/fixtures/sample.comp.h"
#include "impeller/fixtures/stage1.comp.h"
#include "impeller/fixtures/stage2.comp.h"
#include "impeller/playground/compute_playground_test.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/compute_pipeline_builder.h"
#include "impeller/renderer/pipeline_library.h"
#include "impeller/renderer/prefix_sum_test.comp.h"
#include "impeller/renderer/threadgroup_sizing_test.comp.h"

namespace impeller {
namespace testing {
using ComputeTest = ComputePlaygroundTest;
INSTANTIATE_COMPUTE_SUITE(ComputeTest);

TEST_P(ComputeTest, CapabilitiesReportSupport) {
  auto context = GetContext();
  ASSERT_TRUE(context);
  ASSERT_TRUE(context->GetCapabilities()->SupportsCompute());
}

TEST_P(ComputeTest, CanCreateComputePass) {
  using CS = SampleComputeShader;
  auto context = GetContext();
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  ASSERT_TRUE(context);
  ASSERT_TRUE(context->GetCapabilities()->SupportsCompute());

  using SamplePipelineBuilder = ComputePipelineBuilder<CS>;
  auto pipeline_desc =
      SamplePipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_desc.has_value());
  auto compute_pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_desc).Get();
  ASSERT_TRUE(compute_pipeline);

  auto cmd_buffer = context->CreateCommandBuffer();
  auto pass = cmd_buffer->CreateComputePass();
  ASSERT_TRUE(pass && pass->IsValid());

  static constexpr size_t kCount = 5;

  pass->SetPipeline(compute_pipeline);

  CS::Info info{.count = kCount};
  CS::Input0<kCount> input_0;
  CS::Input1<kCount> input_1;
  for (size_t i = 0; i < kCount; i++) {
    input_0.elements[i] = Vector4(2.0 + i, 3.0 + i, 4.0 + i, 5.0 * i);
    input_1.elements[i] = Vector4(6.0, 7.0, 8.0, 9.0);
  }

  input_0.fixed_array[1] = IPoint32(2, 2);
  input_1.fixed_array[0] = UintPoint32(3, 3);
  input_0.some_int = 5;
  input_1.some_struct = CS::SomeStruct{.vf = Point(3, 4), .i = 42};

  auto output_buffer = CreateHostVisibleDeviceBuffer<CS::Output<kCount>>(
      context, "Output Buffer");

  CS::BindInfo(*pass, host_buffer->EmplaceUniform(info));
  CS::BindInput0(*pass, host_buffer->EmplaceStorageBuffer(input_0));
  CS::BindInput1(*pass, host_buffer->EmplaceStorageBuffer(input_1));
  CS::BindOutput(*pass, DeviceBuffer::AsBufferView(output_buffer));

  ASSERT_TRUE(pass->Compute(ISize(kCount, 1)).ok());
  ASSERT_TRUE(pass->EncodeCommands());

  fml::AutoResetWaitableEvent latch;
  ASSERT_TRUE(
      context->GetCommandQueue()
          ->Submit(
              {cmd_buffer},
              [&latch, output_buffer, &input_0,
               &input_1](CommandBuffer::Status status) {
                EXPECT_EQ(status, CommandBuffer::Status::kCompleted);

                auto view = DeviceBuffer::AsBufferView(output_buffer);
                EXPECT_EQ(view.GetRange().length, sizeof(CS::Output<kCount>));

                CS::Output<kCount>* output =
                    reinterpret_cast<CS::Output<kCount>*>(
                        output_buffer->OnGetContents());
                EXPECT_TRUE(output);
                for (size_t i = 0; i < kCount; i++) {
                  Vector4 vector = output->elements[i];
                  Vector4 computed = input_0.elements[i] * input_1.elements[i];
                  EXPECT_EQ(vector,
                            Vector4(computed.x + 2 + input_1.some_struct.i,
                                    computed.y + 3 + input_1.some_struct.vf.x,
                                    computed.z + 5 + input_1.some_struct.vf.y,
                                    computed.w));
                }
                latch.Signal();
              })
          .ok());

  latch.Wait();
}

TEST_P(ComputeTest, CanComputePrefixSum) {
  using CS = PrefixSumTestComputeShader;
  auto context = GetContext();
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  ASSERT_TRUE(context);
  ASSERT_TRUE(context->GetCapabilities()->SupportsCompute());

  using SamplePipelineBuilder = ComputePipelineBuilder<CS>;
  auto pipeline_desc =
      SamplePipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_desc.has_value());
  auto compute_pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_desc).Get();
  ASSERT_TRUE(compute_pipeline);

  auto cmd_buffer = context->CreateCommandBuffer();
  auto pass = cmd_buffer->CreateComputePass();
  ASSERT_TRUE(pass && pass->IsValid());

  static constexpr size_t kCount = 5;

  pass->SetPipeline(compute_pipeline);

  CS::InputData<kCount> input_data;
  input_data.count = kCount;
  for (size_t i = 0; i < kCount; i++) {
    input_data.data[i] = 1 + i;
  }

  auto output_buffer = CreateHostVisibleDeviceBuffer<CS::OutputData<kCount>>(
      context, "Output Buffer");

  CS::BindInputData(*pass, host_buffer->EmplaceStorageBuffer(input_data));
  CS::BindOutputData(*pass, DeviceBuffer::AsBufferView(output_buffer));

  ASSERT_TRUE(pass->Compute(ISize(kCount, 1)).ok());
  ASSERT_TRUE(pass->EncodeCommands());

  fml::AutoResetWaitableEvent latch;
  ASSERT_TRUE(
      context->GetCommandQueue()
          ->Submit({cmd_buffer},
                   [&latch, output_buffer](CommandBuffer::Status status) {
                     EXPECT_EQ(status, CommandBuffer::Status::kCompleted);

                     auto view = DeviceBuffer::AsBufferView(output_buffer);
                     EXPECT_EQ(view.GetRange().length,
                               sizeof(CS::OutputData<kCount>));

                     CS::OutputData<kCount>* output =
                         reinterpret_cast<CS::OutputData<kCount>*>(
                             output_buffer->OnGetContents());
                     EXPECT_TRUE(output);

                     constexpr uint32_t expected[kCount] = {1, 3, 6, 10, 15};
                     for (size_t i = 0; i < kCount; i++) {
                       auto computed_sum = output->data[i];
                       EXPECT_EQ(computed_sum, expected[i]);
                     }
                     latch.Signal();
                   })
          .ok());

  latch.Wait();
}

TEST_P(ComputeTest, 1DThreadgroupSizingIsCorrect) {
  using CS = ThreadgroupSizingTestComputeShader;
  auto context = GetContext();
  ASSERT_TRUE(context);
  ASSERT_TRUE(context->GetCapabilities()->SupportsCompute());

  using SamplePipelineBuilder = ComputePipelineBuilder<CS>;
  auto pipeline_desc =
      SamplePipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_desc.has_value());
  auto compute_pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_desc).Get();
  ASSERT_TRUE(compute_pipeline);

  auto cmd_buffer = context->CreateCommandBuffer();
  auto pass = cmd_buffer->CreateComputePass();
  ASSERT_TRUE(pass && pass->IsValid());

  static constexpr size_t kCount = 2048;

  pass->SetPipeline(compute_pipeline);

  auto output_buffer = CreateHostVisibleDeviceBuffer<CS::OutputData<kCount>>(
      context, "Output Buffer");

  CS::BindOutputData(*pass, DeviceBuffer::AsBufferView(output_buffer));

  ASSERT_TRUE(pass->Compute(ISize(kCount, 1)).ok());
  ASSERT_TRUE(pass->EncodeCommands());

  fml::AutoResetWaitableEvent latch;
  ASSERT_TRUE(
      context->GetCommandQueue()
          ->Submit({cmd_buffer},
                   [&latch, output_buffer](CommandBuffer::Status status) {
                     EXPECT_EQ(status, CommandBuffer::Status::kCompleted);

                     auto view = DeviceBuffer::AsBufferView(output_buffer);
                     EXPECT_EQ(view.GetRange().length,
                               sizeof(CS::OutputData<kCount>));

                     CS::OutputData<kCount>* output =
                         reinterpret_cast<CS::OutputData<kCount>*>(
                             output_buffer->OnGetContents());
                     EXPECT_TRUE(output);
                     EXPECT_EQ(output->data[kCount - 1], kCount - 1);
                     latch.Signal();
                   })
          .ok());

  latch.Wait();
}

TEST_P(ComputeTest, CanComputePrefixSumLargeInteractive) {
  using CS = PrefixSumTestComputeShader;

  auto context = GetContext();
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());

  ASSERT_TRUE(context);
  ASSERT_TRUE(context->GetCapabilities()->SupportsCompute());

  auto callback = [&](RenderPass& render_pass) -> bool {
    using SamplePipelineBuilder = ComputePipelineBuilder<CS>;
    auto pipeline_desc =
        SamplePipelineBuilder::MakeDefaultPipelineDescriptor(*context);
    auto compute_pipeline =
        context->GetPipelineLibrary()->GetPipeline(pipeline_desc).Get();

    auto cmd_buffer = context->CreateCommandBuffer();
    auto pass = cmd_buffer->CreateComputePass();

    static constexpr size_t kCount = 1023;

    pass->SetPipeline(compute_pipeline);

    CS::InputData<kCount> input_data;
    input_data.count = kCount;
    for (size_t i = 0; i < kCount; i++) {
      input_data.data[i] = 1 + i;
    }

    auto output_buffer = CreateHostVisibleDeviceBuffer<CS::OutputData<kCount>>(
        context, "Output Buffer");

    CS::BindInputData(*pass, host_buffer->EmplaceStorageBuffer(input_data));
    CS::BindOutputData(*pass, DeviceBuffer::AsBufferView(output_buffer));

    pass->Compute(ISize(kCount, 1));
    pass->EncodeCommands();
    host_buffer->Reset();
    return context->GetCommandQueue()->Submit({cmd_buffer}).ok();
  };
  ASSERT_TRUE(OpenPlaygroundHere(callback));
}

TEST_P(ComputeTest, MultiStageInputAndOutput) {
  using CS1 = Stage1ComputeShader;
  using Stage1PipelineBuilder = ComputePipelineBuilder<CS1>;
  using CS2 = Stage2ComputeShader;
  using Stage2PipelineBuilder = ComputePipelineBuilder<CS2>;

  auto context = GetContext();
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  ASSERT_TRUE(context);
  ASSERT_TRUE(context->GetCapabilities()->SupportsCompute());

  auto pipeline_desc_1 =
      Stage1PipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_desc_1.has_value());
  auto compute_pipeline_1 =
      context->GetPipelineLibrary()->GetPipeline(pipeline_desc_1).Get();
  ASSERT_TRUE(compute_pipeline_1);

  auto pipeline_desc_2 =
      Stage2PipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_desc_2.has_value());
  auto compute_pipeline_2 =
      context->GetPipelineLibrary()->GetPipeline(pipeline_desc_2).Get();
  ASSERT_TRUE(compute_pipeline_2);

  auto cmd_buffer = context->CreateCommandBuffer();
  auto pass = cmd_buffer->CreateComputePass();
  ASSERT_TRUE(pass && pass->IsValid());

  static constexpr size_t kCount1 = 5;
  static constexpr size_t kCount2 = kCount1 * 2;

  CS1::Input<kCount1> input_1;
  input_1.count = kCount1;
  for (size_t i = 0; i < kCount1; i++) {
    input_1.elements[i] = i;
  }

  CS2::Input<kCount2> input_2;
  input_2.count = kCount2;
  for (size_t i = 0; i < kCount2; i++) {
    input_2.elements[i] = i;
  }

  auto output_buffer_1 = CreateHostVisibleDeviceBuffer<CS1::Output<kCount2>>(
      context, "Output Buffer Stage 1");
  auto output_buffer_2 = CreateHostVisibleDeviceBuffer<CS2::Output<kCount2>>(
      context, "Output Buffer Stage 2");

  {
    pass->SetPipeline(compute_pipeline_1);

    CS1::BindInput(*pass, host_buffer->EmplaceStorageBuffer(input_1));
    CS1::BindOutput(*pass, DeviceBuffer::AsBufferView(output_buffer_1));

    ASSERT_TRUE(pass->Compute(ISize(512, 1)).ok());
    pass->AddBufferMemoryBarrier();
  }

  {
    pass->SetPipeline(compute_pipeline_2);

    CS1::BindInput(*pass, DeviceBuffer::AsBufferView(output_buffer_1));
    CS2::BindOutput(*pass, DeviceBuffer::AsBufferView(output_buffer_2));
    ASSERT_TRUE(pass->Compute(ISize(512, 1)).ok());
  }

  ASSERT_TRUE(pass->EncodeCommands());

  fml::AutoResetWaitableEvent latch;
  ASSERT_TRUE(
      context->GetCommandQueue()
          ->Submit({cmd_buffer},
                   [&latch, &output_buffer_1,
                    &output_buffer_2](CommandBuffer::Status status) {
                     EXPECT_EQ(status, CommandBuffer::Status::kCompleted);

                     CS1::Output<kCount2>* output_1 =
                         reinterpret_cast<CS1::Output<kCount2>*>(
                             output_buffer_1->OnGetContents());
                     EXPECT_TRUE(output_1);
                     EXPECT_EQ(output_1->count, 10u);
                     EXPECT_THAT(
                         output_1->elements,
                         ::testing::ElementsAre(0, 0, 2, 3, 4, 6, 6, 9, 8, 12));

                     CS2::Output<kCount2>* output_2 =
                         reinterpret_cast<CS2::Output<kCount2>*>(
                             output_buffer_2->OnGetContents());
                     EXPECT_TRUE(output_2);
                     EXPECT_EQ(output_2->count, 10u);
                     EXPECT_THAT(output_2->elements,
                                 ::testing::ElementsAre(0, 0, 4, 6, 8, 12, 12,
                                                        18, 16, 24));

                     latch.Signal();
                   })
          .ok());

  latch.Wait();
}

TEST_P(ComputeTest, CanCompute1DimensionalData) {
  using CS = SampleComputeShader;
  auto context = GetContext();
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  ASSERT_TRUE(context);
  ASSERT_TRUE(context->GetCapabilities()->SupportsCompute());

  using SamplePipelineBuilder = ComputePipelineBuilder<CS>;
  auto pipeline_desc =
      SamplePipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_desc.has_value());
  auto compute_pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_desc).Get();
  ASSERT_TRUE(compute_pipeline);

  auto cmd_buffer = context->CreateCommandBuffer();
  auto pass = cmd_buffer->CreateComputePass();
  ASSERT_TRUE(pass && pass->IsValid());

  static constexpr size_t kCount = 5;

  pass->SetPipeline(compute_pipeline);

  CS::Info info{.count = kCount};
  CS::Input0<kCount> input_0;
  CS::Input1<kCount> input_1;
  for (size_t i = 0; i < kCount; i++) {
    input_0.elements[i] = Vector4(2.0 + i, 3.0 + i, 4.0 + i, 5.0 * i);
    input_1.elements[i] = Vector4(6.0, 7.0, 8.0, 9.0);
  }

  input_0.fixed_array[1] = IPoint32(2, 2);
  input_1.fixed_array[0] = UintPoint32(3, 3);
  input_0.some_int = 5;
  input_1.some_struct = CS::SomeStruct{.vf = Point(3, 4), .i = 42};

  auto output_buffer = CreateHostVisibleDeviceBuffer<CS::Output<kCount>>(
      context, "Output Buffer");

  CS::BindInfo(*pass, host_buffer->EmplaceUniform(info));
  CS::BindInput0(*pass, host_buffer->EmplaceStorageBuffer(input_0));
  CS::BindInput1(*pass, host_buffer->EmplaceStorageBuffer(input_1));
  CS::BindOutput(*pass, DeviceBuffer::AsBufferView(output_buffer));

  ASSERT_TRUE(pass->Compute(ISize(kCount, 1)).ok());
  ASSERT_TRUE(pass->EncodeCommands());

  fml::AutoResetWaitableEvent latch;
  ASSERT_TRUE(
      context->GetCommandQueue()
          ->Submit(
              {cmd_buffer},
              [&latch, output_buffer, &input_0,
               &input_1](CommandBuffer::Status status) {
                EXPECT_EQ(status, CommandBuffer::Status::kCompleted);

                auto view = DeviceBuffer::AsBufferView(output_buffer);
                EXPECT_EQ(view.GetRange().length, sizeof(CS::Output<kCount>));

                CS::Output<kCount>* output =
                    reinterpret_cast<CS::Output<kCount>*>(
                        output_buffer->OnGetContents());
                EXPECT_TRUE(output);
                for (size_t i = 0; i < kCount; i++) {
                  Vector4 vector = output->elements[i];
                  Vector4 computed = input_0.elements[i] * input_1.elements[i];
                  EXPECT_EQ(vector,
                            Vector4(computed.x + 2 + input_1.some_struct.i,
                                    computed.y + 3 + input_1.some_struct.vf.x,
                                    computed.z + 5 + input_1.some_struct.vf.y,
                                    computed.w));
                }
                latch.Signal();
              })
          .ok());

  latch.Wait();
}

TEST_P(ComputeTest, ReturnsEarlyWhenAnyGridDimensionIsZero) {
  using CS = SampleComputeShader;
  auto context = GetContext();
  auto host_buffer = HostBuffer::Create(context->GetResourceAllocator(),
                                        context->GetIdleWaiter());
  ASSERT_TRUE(context);
  ASSERT_TRUE(context->GetCapabilities()->SupportsCompute());

  using SamplePipelineBuilder = ComputePipelineBuilder<CS>;
  auto pipeline_desc =
      SamplePipelineBuilder::MakeDefaultPipelineDescriptor(*context);
  ASSERT_TRUE(pipeline_desc.has_value());
  auto compute_pipeline =
      context->GetPipelineLibrary()->GetPipeline(pipeline_desc).Get();
  ASSERT_TRUE(compute_pipeline);

  auto cmd_buffer = context->CreateCommandBuffer();
  auto pass = cmd_buffer->CreateComputePass();
  ASSERT_TRUE(pass && pass->IsValid());

  static constexpr size_t kCount = 5;

  pass->SetPipeline(compute_pipeline);

  CS::Info info{.count = kCount};
  CS::Input0<kCount> input_0;
  CS::Input1<kCount> input_1;
  for (size_t i = 0; i < kCount; i++) {
    input_0.elements[i] = Vector4(2.0 + i, 3.0 + i, 4.0 + i, 5.0 * i);
    input_1.elements[i] = Vector4(6.0, 7.0, 8.0, 9.0);
  }

  input_0.fixed_array[1] = IPoint32(2, 2);
  input_1.fixed_array[0] = UintPoint32(3, 3);
  input_0.some_int = 5;
  input_1.some_struct = CS::SomeStruct{.vf = Point(3, 4), .i = 42};

  auto output_buffer = CreateHostVisibleDeviceBuffer<CS::Output<kCount>>(
      context, "Output Buffer");

  CS::BindInfo(*pass, host_buffer->EmplaceUniform(info));
  CS::BindInput0(*pass, host_buffer->EmplaceStorageBuffer(input_0));
  CS::BindInput1(*pass, host_buffer->EmplaceStorageBuffer(input_1));
  CS::BindOutput(*pass, DeviceBuffer::AsBufferView(output_buffer));

  // Intentionally making the grid size zero in one dimension. No GPU will
  // tolerate this.
  EXPECT_FALSE(pass->Compute(ISize(0, 1)).ok());
  pass->EncodeCommands();
}

}  // namespace testing
}  // namespace impeller
