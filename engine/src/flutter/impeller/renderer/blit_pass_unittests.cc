// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cstdint>
#include "flutter/display_list/dl_builder.h"
#include "flutter/impeller/display_list/aiks_unittests.h"
#include "flutter/impeller/display_list/dl_image_impeller.h"
#include "fml/mapping.h"
#include "gtest/gtest.h"
#include "impeller/base/validation.h"
#include "impeller/core/device_buffer.h"
#include "impeller/core/device_buffer_descriptor.h"
#include "impeller/core/formats.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/playground/playground_test.h"
#include "impeller/renderer/command_buffer.h"

namespace impeller {
namespace testing {

using flutter::DisplayListBuilder;
using flutter::DlColor;
using flutter::DlImageSampling;
using flutter::DlPaint;

using BlitPassTest = AiksTest;
INSTANTIATE_PLAYGROUND_SUITE(BlitPassTest);

TEST_P(BlitPassTest, BlitAcrossDifferentPixelFormatsFails) {
  ScopedValidationDisable scope;  // avoid noise in output.
  auto context = GetContext();
  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();

  TextureDescriptor src_desc;
  src_desc.format = PixelFormat::kA8UNormInt;
  src_desc.size = {100, 100};
  src_desc.storage_mode = StorageMode::kHostVisible;
  auto src = context->GetResourceAllocator()->CreateTexture(src_desc);

  TextureDescriptor dst_format;
  dst_format.format = PixelFormat::kR8G8B8A8UNormInt;
  dst_format.size = {100, 100};
  dst_format.storage_mode = StorageMode::kHostVisible;
  auto dst = context->GetResourceAllocator()->CreateTexture(dst_format);

  EXPECT_FALSE(blit_pass->AddCopy(src, dst));
}

TEST_P(BlitPassTest, BlitAcrossDifferentSampleCountsFails) {
  ScopedValidationDisable scope;  // avoid noise in output.
  auto context = GetContext();
  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();

  TextureDescriptor src_desc;
  src_desc.format = PixelFormat::kR8G8B8A8UNormInt;
  src_desc.sample_count = SampleCount::kCount4;
  src_desc.size = {100, 100};
  auto src = context->GetResourceAllocator()->CreateTexture(src_desc);

  TextureDescriptor dst_format;
  dst_format.format = PixelFormat::kR8G8B8A8UNormInt;
  dst_format.size = {100, 100};
  auto dst = context->GetResourceAllocator()->CreateTexture(dst_format);

  EXPECT_FALSE(blit_pass->AddCopy(src, dst));
}

TEST_P(BlitPassTest, BlitPassesForMatchingFormats) {
  ScopedValidationDisable scope;  // avoid noise in output.
  auto context = GetContext();
  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();

  TextureDescriptor src_desc;
  src_desc.format = PixelFormat::kR8G8B8A8UNormInt;
  src_desc.size = {100, 100};
  src_desc.storage_mode = StorageMode::kHostVisible;
  auto src = context->GetResourceAllocator()->CreateTexture(src_desc);

  TextureDescriptor dst_format;
  dst_format.format = PixelFormat::kR8G8B8A8UNormInt;
  dst_format.size = {100, 100};
  dst_format.storage_mode = StorageMode::kHostVisible;
  auto dst = context->GetResourceAllocator()->CreateTexture(dst_format);

  EXPECT_TRUE(blit_pass->AddCopy(src, dst));
}

TEST_P(BlitPassTest, ChecksInvalidSliceParameters) {
  ScopedValidationDisable scope;  // avoid noise in output.
  auto context = GetContext();
  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();

  TextureDescriptor dst_format;
  dst_format.storage_mode = StorageMode::kDevicePrivate;
  dst_format.format = PixelFormat::kR8G8B8A8UNormInt;
  dst_format.size = {100, 100};
  auto dst = context->GetResourceAllocator()->CreateTexture(dst_format);

  DeviceBufferDescriptor src_format;
  src_format.size = 40000;
  src_format.storage_mode = StorageMode::kHostVisible;
  auto src = context->GetResourceAllocator()->CreateBuffer(src_format);

  ASSERT_TRUE(dst);
  ASSERT_TRUE(src);

  EXPECT_FALSE(blit_pass->AddCopy(DeviceBuffer::AsBufferView(src), dst,
                                  std::nullopt, "", /*mip_level=*/0,
                                  /*slice=*/25));
  EXPECT_FALSE(blit_pass->AddCopy(DeviceBuffer::AsBufferView(src), dst,
                                  std::nullopt, "", /*mip_level=*/0,
                                  /*slice=*/6));
  EXPECT_TRUE(blit_pass->AddCopy(DeviceBuffer::AsBufferView(src), dst,
                                 std::nullopt, "", /*mip_level=*/0,
                                 /*slice=*/0));
}

TEST_P(BlitPassTest, CanBlitSmallRegionToUninitializedTexture) {
  auto context = GetContext();
  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();

  TextureDescriptor dst_format;
  dst_format.storage_mode = StorageMode::kDevicePrivate;
  dst_format.format = PixelFormat::kR8G8B8A8UNormInt;
  dst_format.size = {1000, 1000};
  auto dst = context->GetResourceAllocator()->CreateTexture(dst_format);

  DeviceBufferDescriptor src_format;
  src_format.size = 4;
  src_format.storage_mode = StorageMode::kHostVisible;
  auto src = context->GetResourceAllocator()->CreateBuffer(src_format);

  ASSERT_TRUE(dst);

  EXPECT_TRUE(blit_pass->AddCopy(DeviceBuffer::AsBufferView(src), dst,
                                 IRect::MakeLTRB(0, 0, 1, 1), "",
                                 /*mip_level=*/0, /*slice=*/0));
  EXPECT_TRUE(blit_pass->EncodeCommands());
  EXPECT_TRUE(context->GetCommandQueue()->Submit({std::move(cmd_buffer)}).ok());
}

TEST_P(BlitPassTest, ChecksInvalidMipLevelParameter) {
  ScopedValidationDisable scope;
  auto context = GetContext();
  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();

  TextureDescriptor dst_format;
  dst_format.storage_mode = StorageMode::kDevicePrivate;
  dst_format.format = PixelFormat::kR8G8B8A8UNormInt;
  dst_format.size = {1000, 1000};
  auto dst = context->GetResourceAllocator()->CreateTexture(dst_format);

  DeviceBufferDescriptor src_format;
  src_format.size = 4;
  src_format.storage_mode = StorageMode::kHostVisible;
  auto src = context->GetResourceAllocator()->CreateBuffer(src_format);

  ASSERT_TRUE(dst);

  EXPECT_FALSE(blit_pass->AddCopy(DeviceBuffer::AsBufferView(src), dst,
                                  IRect::MakeLTRB(0, 0, 1, 1), "",
                                  /*mip_level=*/1, /*slice=*/0));

  EXPECT_TRUE(blit_pass->AddCopy(DeviceBuffer::AsBufferView(src), dst,
                                 IRect::MakeLTRB(0, 0, 1, 1), "",
                                 /*mip_level=*/0, /*slice=*/0));
}

TEST_P(BlitPassTest, CanBlitToHigherTextureMipLevels) {
  auto context = GetContext();
  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();

  TextureDescriptor dst_format;
  dst_format.storage_mode = StorageMode::kDevicePrivate;
  dst_format.format = PixelFormat::kR8G8B8A8UNormInt;
  dst_format.size = {1000, 1000};
  dst_format.mip_count = 4;
  auto dst = context->GetResourceAllocator()->CreateTexture(dst_format);

  DeviceBufferDescriptor src_format;
  src_format.size = 4;
  src_format.storage_mode = StorageMode::kHostVisible;
  auto src = context->GetResourceAllocator()->CreateBuffer(src_format);

  ASSERT_TRUE(dst);

  EXPECT_TRUE(blit_pass->AddCopy(DeviceBuffer::AsBufferView(src), dst,
                                 IRect::MakeLTRB(0, 0, 1, 1), "",
                                 /*mip_level=*/1, /*slice=*/0));
  EXPECT_TRUE(blit_pass->EncodeCommands());
  EXPECT_TRUE(context->GetCommandQueue()->Submit({std::move(cmd_buffer)}).ok());
}

TEST_P(BlitPassTest, CanResizeTextures) {
  auto context = GetContext();
  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();

  TextureDescriptor dst_format;
  dst_format.storage_mode = StorageMode::kDevicePrivate;
  dst_format.format = PixelFormat::kR8G8B8A8UNormInt;
  dst_format.size = {10, 10};
  dst_format.usage = TextureUsage::kShaderRead | TextureUsage::kShaderWrite;
  auto dst = context->GetResourceAllocator()->CreateTexture(dst_format);

  TextureDescriptor src_format;
  src_format.storage_mode = StorageMode::kDevicePrivate;
  src_format.format = PixelFormat::kR8G8B8A8UNormInt;
  src_format.size = {100, 100};
  auto src = context->GetResourceAllocator()->CreateTexture(src_format);

  std::vector<uint8_t> bytes(src_format.GetByteSizeOfBaseMipLevel());
  for (auto i = 0u; i < src_format.GetByteSizeOfBaseMipLevel(); i += 4) {
    // RGBA
    bytes[i + 0] = 255;
    bytes[i + 1] = 0;
    bytes[i + 2] = 0;
    bytes[i + 3] = 255;
  }
  auto mapping = fml::DataMapping(bytes);
  auto staging = context->GetResourceAllocator()->CreateBufferWithCopy(mapping);

  ASSERT_TRUE(dst);
  ASSERT_TRUE(src);
  ASSERT_TRUE(staging);

  EXPECT_TRUE(blit_pass->AddCopy(DeviceBuffer::AsBufferView(staging), src));
  EXPECT_TRUE(blit_pass->ResizeTexture(src, dst));
  EXPECT_TRUE(blit_pass->EncodeCommands());
  EXPECT_TRUE(context->GetCommandQueue()->Submit({std::move(cmd_buffer)}).ok());
}

TEST_P(BlitPassTest, CanResizeTexturesPlayground) {
  auto context = GetContext();
  auto cmd_buffer = context->CreateCommandBuffer();
  auto blit_pass = cmd_buffer->CreateBlitPass();

  std::shared_ptr<Texture> src = CreateTextureForFixture("kalimba.jpg");

  TextureDescriptor dst_format;
  dst_format.storage_mode = StorageMode::kDevicePrivate;
  dst_format.format = PixelFormat::kR8G8B8A8UNormInt;
  dst_format.size = {src->GetSize().width / 2, src->GetSize().height};
  dst_format.usage = TextureUsage::kShaderRead | TextureUsage::kShaderWrite;
  auto dst = context->GetResourceAllocator()->CreateTexture(dst_format);

  ASSERT_TRUE(dst);
  ASSERT_TRUE(src);

  EXPECT_TRUE(blit_pass->ResizeTexture(src, dst));
  EXPECT_TRUE(blit_pass->EncodeCommands());
  EXPECT_TRUE(context->GetCommandQueue()->Submit({std::move(cmd_buffer)}).ok());

  DisplayListBuilder builder;
  builder.Scale(GetContentScale().x, GetContentScale().y);
  DlPaint paint;
  paint.setColor(DlColor::kRed());
  auto image = DlImageImpeller::Make(dst);
  builder.DrawImage(image, flutter::DlPoint(100.0, 100.0),
                    DlImageSampling::kNearestNeighbor, &paint);
  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller
