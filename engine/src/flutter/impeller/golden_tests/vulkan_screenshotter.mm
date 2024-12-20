// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/golden_tests/vulkan_screenshotter.h"

#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/impeller/golden_tests/metal_screenshot.h"
#define GLFW_INCLUDE_NONE
#include "third_party/glfw/include/GLFW/glfw3.h"

namespace impeller {
namespace testing {

namespace {

using CGContextPtr = std::unique_ptr<std::remove_pointer<CGContextRef>::type,
                                     decltype(&CGContextRelease)>;
using CGImagePtr = std::unique_ptr<std::remove_pointer<CGImageRef>::type,
                                   decltype(&CGImageRelease)>;
using CGColorSpacePtr =
    std::unique_ptr<std::remove_pointer<CGColorSpaceRef>::type,
                    decltype(&CGColorSpaceRelease)>;

std::unique_ptr<Screenshot> ReadTexture(
    const std::shared_ptr<Context>& surface_context,
    const std::shared_ptr<Texture>& texture) {
  DeviceBufferDescriptor buffer_desc;
  buffer_desc.storage_mode = StorageMode::kHostVisible;
  buffer_desc.size =
      texture->GetTextureDescriptor().GetByteSizeOfBaseMipLevel();
  buffer_desc.readback = true;
  std::shared_ptr<DeviceBuffer> device_buffer =
      surface_context->GetResourceAllocator()->CreateBuffer(buffer_desc);
  FML_CHECK(device_buffer);

  auto command_buffer = surface_context->CreateCommandBuffer();
  auto blit_pass = command_buffer->CreateBlitPass();
  bool success = blit_pass->AddCopy(texture, device_buffer);
  FML_CHECK(success);

  success = blit_pass->EncodeCommands(surface_context->GetResourceAllocator());
  FML_CHECK(success);

  fml::AutoResetWaitableEvent latch;
  success =
      surface_context->GetCommandQueue()
          ->Submit({command_buffer},
                   [&latch](CommandBuffer::Status status) {
                     FML_CHECK(status == CommandBuffer::Status::kCompleted);
                     latch.Signal();
                   })
          .ok();
  FML_CHECK(success);
  latch.Wait();
  device_buffer->Invalidate();

  // TODO(gaaclarke): Replace CoreImage requirement with something
  // crossplatform.

  CGColorSpacePtr color_space(CGColorSpaceCreateDeviceRGB(),
                              &CGColorSpaceRelease);
  CGBitmapInfo bitmap_info =
      texture->GetTextureDescriptor().format == PixelFormat::kB8G8R8A8UNormInt
          ? kCGImageAlphaPremultipliedFirst | kCGBitmapByteOrder32Little
          : kCGImageAlphaPremultipliedLast;
  CGContextPtr context(
      CGBitmapContextCreate(
          device_buffer->OnGetContents(), texture->GetSize().width,
          texture->GetSize().height,
          /*bitsPerComponent=*/8,
          /*bytesPerRow=*/texture->GetTextureDescriptor().GetBytesPerRow(),
          color_space.get(), bitmap_info),
      &CGContextRelease);
  FML_CHECK(context);
  CGImagePtr image(CGBitmapContextCreateImage(context.get()), &CGImageRelease);
  FML_CHECK(image);

  // TODO(142641): Perform the flip at the blit stage to avoid this slow copy.
  if (texture->GetYCoordScale() == -1) {
    CGContextPtr flipped_context(
        CGBitmapContextCreate(
            nullptr, texture->GetSize().width, texture->GetSize().height,
            /*bitsPerComponent=*/8,
            /*bytesPerRow=*/0, color_space.get(), bitmap_info),
        &CGContextRelease);
    CGContextTranslateCTM(flipped_context.get(), 0, texture->GetSize().height);
    CGContextScaleCTM(flipped_context.get(), 1.0, -1.0);
    CGContextDrawImage(
        flipped_context.get(),
        CGRectMake(0, 0, texture->GetSize().width, texture->GetSize().height),
        image.get());
    CGImagePtr flipped_image(CGBitmapContextCreateImage(flipped_context.get()),
                             &CGImageRelease);
    image.swap(flipped_image);
  }

  return std::make_unique<MetalScreenshot>(image.release());
}
}  // namespace

VulkanScreenshotter::VulkanScreenshotter(
    const std::unique_ptr<PlaygroundImpl>& playground)
    : playground_(playground) {
  FML_CHECK(playground_);
}

std::unique_ptr<Screenshot> VulkanScreenshotter::MakeScreenshot(
    AiksContext& aiks_context,
    const std::shared_ptr<Texture> texture) {
  return ReadTexture(aiks_context.GetContext(), texture);
}

}  // namespace testing
}  // namespace impeller
