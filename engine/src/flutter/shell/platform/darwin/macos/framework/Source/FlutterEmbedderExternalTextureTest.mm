// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <memory>
#include <vector>

#include "flutter/display_list/display_list.h"
#include "flutter/display_list/dl_builder.h"
#import "flutter/display_list/skia/dl_sk_canvas.h"
#include "flutter/fml/synchronization/sync_switch.h"
#include "flutter/shell/platform/darwin/graphics/FlutterDarwinContextMetalImpeller.h"
#import "flutter/shell/platform/darwin/graphics/FlutterDarwinContextMetalSkia.h"
#import "flutter/shell/platform/darwin/graphics/FlutterDarwinExternalTextureMetal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterExternalTexture.h"
#include "flutter/shell/platform/embedder/embedder.h"
#include "flutter/shell/platform/embedder/embedder_external_texture_metal.h"
#include "flutter/testing/autoreleasepool_test.h"
#include "flutter/testing/testing.h"
#include "impeller/display_list/aiks_context.h"             // nogncheck
#include "impeller/entity/mtl/entity_shaders.h"             // nogncheck
#include "impeller/entity/mtl/framebuffer_blend_shaders.h"  // nogncheck
#include "impeller/entity/mtl/modern_shaders.h"             // nogncheck
#include "impeller/renderer/backend/metal/context_mtl.h"    // nogncheck
#include "third_party/googletest/googletest/include/gtest/gtest.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkSamplingOptions.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"

static std::shared_ptr<impeller::ContextMTL> CreateImpellerContext() {
  std::vector<std::shared_ptr<fml::Mapping>> shader_mappings = {
      std::make_shared<fml::NonOwnedMapping>(impeller_entity_shaders_data,
                                             impeller_entity_shaders_length),
      std::make_shared<fml::NonOwnedMapping>(impeller_modern_shaders_data,
                                             impeller_modern_shaders_length),
      std::make_shared<fml::NonOwnedMapping>(impeller_framebuffer_blend_shaders_data,
                                             impeller_framebuffer_blend_shaders_length),
  };
  auto sync_switch = std::make_shared<fml::SyncSwitch>(false);
  return impeller::ContextMTL::Create(shader_mappings, sync_switch, "Impeller Library");
}

@interface TestExternalTexture : NSObject <FlutterTexture>

- (nonnull instancetype)initWidth:(size_t)width
                           height:(size_t)height
                  pixelFormatType:(OSType)pixelFormatType;

@end

@implementation TestExternalTexture {
  size_t _width;
  size_t _height;
  OSType _pixelFormatType;
}

- (nonnull instancetype)initWidth:(size_t)width
                           height:(size_t)height
                  pixelFormatType:(OSType)pixelFormatType {
  if (self = [super init]) {
    _width = width;
    _height = height;
    _pixelFormatType = pixelFormatType;
  }
  return self;
}

- (CVPixelBufferRef)copyPixelBuffer {
  return [self pixelBuffer];
}

- (CVPixelBufferRef)pixelBuffer {
  NSDictionary* options = @{
    // This key is required to generate SKPicture with CVPixelBufferRef in metal.
    (NSString*)kCVPixelBufferMetalCompatibilityKey : @YES
  };
  CVPixelBufferRef pxbuffer = NULL;
  CVReturn status = CVPixelBufferCreate(kCFAllocatorDefault, _width, _width, _pixelFormatType,
                                        (__bridge CFDictionaryRef)options, &pxbuffer);
  FML_CHECK(status == kCVReturnSuccess && pxbuffer != nullptr) << "Failed to create pixel buffer";
  return pxbuffer;
}

@end

namespace flutter::testing {

// Test-specific name for AutoreleasePoolTest fixture.
using FlutterEmbedderExternalTextureTest = AutoreleasePoolTest;

TEST_F(FlutterEmbedderExternalTextureTest, TestTextureResolution) {
  // Constants.
  const size_t width = 100;
  const size_t height = 100;
  const int64_t texture_id = 1;

  // Set up the surface.
  FlutterDarwinContextMetalSkia* darwinContextMetal =
      [[FlutterDarwinContextMetalSkia alloc] initWithDefaultMTLDevice];
  SkImageInfo info = SkImageInfo::MakeN32Premul(width, height);
  GrDirectContext* grContext = darwinContextMetal.mainContext.get();
  sk_sp<SkSurface> gpuSurface(SkSurfaces::RenderTarget(grContext, skgpu::Budgeted::kNo, info));

  // Create a texture.
  MTLTextureDescriptor* textureDescriptor = [[MTLTextureDescriptor alloc] init];
  textureDescriptor.pixelFormat = MTLPixelFormatBGRA8Unorm;
  textureDescriptor.width = width;
  textureDescriptor.height = height;
  textureDescriptor.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
  id<MTLTexture> mtlTexture =
      [darwinContextMetal.device newTextureWithDescriptor:textureDescriptor];
  std::vector<FlutterMetalTextureHandle> textures = {
      (__bridge FlutterMetalTextureHandle)mtlTexture,
  };

  // Callback to resolve the texture.
  EmbedderExternalTextureMetal::ExternalTextureCallback callback = [&](int64_t texture_id, size_t w,
                                                                       size_t h) {
    EXPECT_TRUE(w == width);
    EXPECT_TRUE(h == height);

    auto texture = std::make_unique<FlutterMetalExternalTexture>();
    texture->struct_size = sizeof(FlutterMetalExternalTexture);
    texture->num_textures = 1;
    texture->height = h;
    texture->width = w;
    texture->pixel_format = FlutterMetalExternalTexturePixelFormat::kRGBA;
    texture->textures = textures.data();
    return texture;
  };

  // Render the texture.
  std::unique_ptr<flutter::Texture> texture =
      std::make_unique<EmbedderExternalTextureMetal>(texture_id, callback);
  SkRect bounds = SkRect::MakeWH(info.width(), info.height());
  DlImageSampling sampling = DlImageSampling::kNearestNeighbor;
  DlSkCanvasAdapter canvas(gpuSurface->getCanvas());
  flutter::Texture::PaintContext context{
      .canvas = &canvas,
      .gr_context = grContext,
  };
  texture->Paint(context, bounds, /*freeze=*/false, sampling);

  ASSERT_TRUE(mtlTexture != nil);

  gpuSurface->makeImageSnapshot();
}

TEST_F(FlutterEmbedderExternalTextureTest, TestPopulateExternalTexture) {
  // Constants.
  const size_t width = 100;
  const size_t height = 100;
  const int64_t texture_id = 1;

  // Set up the surface.
  FlutterDarwinContextMetalSkia* darwinContextMetal =
      [[FlutterDarwinContextMetalSkia alloc] initWithDefaultMTLDevice];
  SkImageInfo info = SkImageInfo::MakeN32Premul(width, height);
  GrDirectContext* grContext = darwinContextMetal.mainContext.get();
  sk_sp<SkSurface> gpuSurface(SkSurfaces::RenderTarget(grContext, skgpu::Budgeted::kNo, info));

  // Create a texture.
  TestExternalTexture* testExternalTexture =
      [[TestExternalTexture alloc] initWidth:width
                                      height:height
                             pixelFormatType:kCVPixelFormatType_32BGRA];
  FlutterExternalTexture* textureHolder =
      [[FlutterExternalTexture alloc] initWithFlutterTexture:testExternalTexture
                                          darwinMetalContext:darwinContextMetal];

  // Callback to resolve the texture.
  EmbedderExternalTextureMetal::ExternalTextureCallback callback = [&](int64_t texture_id, size_t w,
                                                                       size_t h) {
    EXPECT_TRUE(w == width);
    EXPECT_TRUE(h == height);

    auto texture = std::make_unique<FlutterMetalExternalTexture>();
    [textureHolder populateTexture:texture.get()];

    EXPECT_TRUE(texture->num_textures == 1);
    EXPECT_TRUE(texture->textures != nullptr);
    EXPECT_TRUE(texture->pixel_format == FlutterMetalExternalTexturePixelFormat::kRGBA);
    return texture;
  };

  // Render the texture.
  std::unique_ptr<flutter::Texture> texture =
      std::make_unique<EmbedderExternalTextureMetal>(texture_id, callback);
  SkRect bounds = SkRect::MakeWH(info.width(), info.height());
  DlImageSampling sampling = DlImageSampling::kNearestNeighbor;
  DlSkCanvasAdapter canvas(gpuSurface->getCanvas());
  flutter::Texture::PaintContext context{
      .canvas = &canvas,
      .gr_context = grContext,
  };
  texture->Paint(context, bounds, /*freeze=*/false, sampling);

  gpuSurface->makeImageSnapshot();
}

TEST_F(FlutterEmbedderExternalTextureTest, TestPopulateExternalTextureYUVA) {
  // Constants.
  const size_t width = 100;
  const size_t height = 100;
  const int64_t texture_id = 1;

  // Set up the surface.
  FlutterDarwinContextMetalSkia* darwinContextMetal =
      [[FlutterDarwinContextMetalSkia alloc] initWithDefaultMTLDevice];
  SkImageInfo info = SkImageInfo::MakeN32Premul(width, height);
  GrDirectContext* grContext = darwinContextMetal.mainContext.get();
  sk_sp<SkSurface> gpuSurface(SkSurfaces::RenderTarget(grContext, skgpu::Budgeted::kNo, info));

  // Create a texture.
  TestExternalTexture* testExternalTexture =
      [[TestExternalTexture alloc] initWidth:width
                                      height:height
                             pixelFormatType:kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange];
  FlutterExternalTexture* textureHolder =
      [[FlutterExternalTexture alloc] initWithFlutterTexture:testExternalTexture
                                          darwinMetalContext:darwinContextMetal];

  // Callback to resolve the texture.
  EmbedderExternalTextureMetal::ExternalTextureCallback callback = [&](int64_t texture_id, size_t w,
                                                                       size_t h) {
    EXPECT_TRUE(w == width);
    EXPECT_TRUE(h == height);

    auto texture = std::make_unique<FlutterMetalExternalTexture>();
    [textureHolder populateTexture:texture.get()];

    EXPECT_TRUE(texture->num_textures == 2);
    EXPECT_TRUE(texture->textures != nullptr);
    EXPECT_TRUE(texture->pixel_format == FlutterMetalExternalTexturePixelFormat::kYUVA);
    EXPECT_TRUE(texture->yuv_color_space ==
                FlutterMetalExternalTextureYUVColorSpace::kBT601LimitedRange);
    return texture;
  };

  // Render the texture.
  std::unique_ptr<flutter::Texture> texture =
      std::make_unique<EmbedderExternalTextureMetal>(texture_id, callback);
  SkRect bounds = SkRect::MakeWH(info.width(), info.height());
  DlImageSampling sampling = DlImageSampling::kNearestNeighbor;
  DlSkCanvasAdapter canvas(gpuSurface->getCanvas());
  flutter::Texture::PaintContext context{
      .canvas = &canvas,
      .gr_context = grContext,
  };
  texture->Paint(context, bounds, /*freeze=*/false, sampling);

  gpuSurface->makeImageSnapshot();
}

TEST_F(FlutterEmbedderExternalTextureTest, TestPopulateExternalTextureYUVA2) {
  // Constants.
  const size_t width = 100;
  const size_t height = 100;
  const int64_t texture_id = 1;

  // Set up the surface.
  FlutterDarwinContextMetalSkia* darwinContextMetal =
      [[FlutterDarwinContextMetalSkia alloc] initWithDefaultMTLDevice];
  SkImageInfo info = SkImageInfo::MakeN32Premul(width, height);
  GrDirectContext* grContext = darwinContextMetal.mainContext.get();
  sk_sp<SkSurface> gpuSurface(SkSurfaces::RenderTarget(grContext, skgpu::Budgeted::kNo, info));

  // Create a texture.
  TestExternalTexture* testExternalTexture =
      [[TestExternalTexture alloc] initWidth:width
                                      height:height
                             pixelFormatType:kCVPixelFormatType_420YpCbCr8BiPlanarFullRange];
  FlutterExternalTexture* textureHolder =
      [[FlutterExternalTexture alloc] initWithFlutterTexture:testExternalTexture
                                          darwinMetalContext:darwinContextMetal];

  // Callback to resolve the texture.
  EmbedderExternalTextureMetal::ExternalTextureCallback callback = [&](int64_t texture_id, size_t w,
                                                                       size_t h) {
    EXPECT_TRUE(w == width);
    EXPECT_TRUE(h == height);

    auto texture = std::make_unique<FlutterMetalExternalTexture>();
    [textureHolder populateTexture:texture.get()];

    EXPECT_TRUE(texture->num_textures == 2);
    EXPECT_TRUE(texture->textures != nullptr);
    EXPECT_TRUE(texture->pixel_format == FlutterMetalExternalTexturePixelFormat::kYUVA);
    EXPECT_TRUE(texture->yuv_color_space ==
                FlutterMetalExternalTextureYUVColorSpace::kBT601FullRange);
    return texture;
  };

  // Render the texture.
  std::unique_ptr<flutter::Texture> texture =
      std::make_unique<EmbedderExternalTextureMetal>(texture_id, callback);
  SkRect bounds = SkRect::MakeWH(info.width(), info.height());
  DlImageSampling sampling = DlImageSampling::kNearestNeighbor;
  DlSkCanvasAdapter canvas(gpuSurface->getCanvas());
  flutter::Texture::PaintContext context{
      .canvas = &canvas,
      .gr_context = grContext,
  };
  texture->Paint(context, bounds, /*freeze=*/false, sampling);

  gpuSurface->makeImageSnapshot();
}

TEST_F(FlutterEmbedderExternalTextureTest, TestPopulateUnsupportedExternalTexture) {
  // Constants.
  const size_t width = 100;
  const size_t height = 100;
  const int64_t texture_id = 1;

  // Set up the surface.
  FlutterDarwinContextMetalSkia* darwinContextMetal =
      [[FlutterDarwinContextMetalSkia alloc] initWithDefaultMTLDevice];
  SkImageInfo info = SkImageInfo::MakeN32Premul(width, height);
  GrDirectContext* grContext = darwinContextMetal.mainContext.get();
  sk_sp<SkSurface> gpuSurface(SkSurfaces::RenderTarget(grContext, skgpu::Budgeted::kNo, info));

  // Create a texture.
  TestExternalTexture* testExternalTexture =
      [[TestExternalTexture alloc] initWidth:width
                                      height:height
                             pixelFormatType:kCVPixelFormatType_420YpCbCr8PlanarFullRange];
  FlutterExternalTexture* textureHolder =
      [[FlutterExternalTexture alloc] initWithFlutterTexture:testExternalTexture
                                          darwinMetalContext:darwinContextMetal];

  // Callback to resolve the texture.
  EmbedderExternalTextureMetal::ExternalTextureCallback callback = [&](int64_t texture_id, size_t w,
                                                                       size_t h) {
    EXPECT_TRUE(w == width);
    EXPECT_TRUE(h == height);

    auto texture = std::make_unique<FlutterMetalExternalTexture>();
    EXPECT_FALSE([textureHolder populateTexture:texture.get()]);
    return nullptr;
  };

  // Render the texture.
  std::unique_ptr<flutter::Texture> texture =
      std::make_unique<EmbedderExternalTextureMetal>(texture_id, callback);
  SkRect bounds = SkRect::MakeWH(info.width(), info.height());
  DlImageSampling sampling = DlImageSampling::kNearestNeighbor;
  DlSkCanvasAdapter canvas(gpuSurface->getCanvas());
  flutter::Texture::PaintContext context{
      .canvas = &canvas,
      .gr_context = grContext,
  };
  texture->Paint(context, bounds, /*freeze=*/false, sampling);
}

TEST_F(FlutterEmbedderExternalTextureTest, TestTextureResolutionImpeller) {
  // Constants.
  const size_t width = 100;
  const size_t height = 100;
  const int64_t texture_id = 1;

  // Set up the surface.
  auto device = ::MTLCreateSystemDefaultDevice();
  impeller::AiksContext aiks_context(CreateImpellerContext(), nullptr);

  SkImageInfo info = SkImageInfo::MakeN32Premul(width, height);

  // Create a texture.
  MTLTextureDescriptor* textureDescriptor = [[MTLTextureDescriptor alloc] init];
  textureDescriptor.pixelFormat = MTLPixelFormatBGRA8Unorm;
  textureDescriptor.width = width;
  textureDescriptor.height = height;
  textureDescriptor.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
  id<MTLTexture> mtlTexture = [device newTextureWithDescriptor:textureDescriptor];
  std::vector<FlutterMetalTextureHandle> textures = {
      (__bridge FlutterMetalTextureHandle)mtlTexture,
  };

  // Callback to resolve the texture.
  EmbedderExternalTextureMetal::ExternalTextureCallback callback = [&](int64_t texture_id, size_t w,
                                                                       size_t h) {
    EXPECT_TRUE(w == width);
    EXPECT_TRUE(h == height);

    auto texture = std::make_unique<FlutterMetalExternalTexture>();
    texture->struct_size = sizeof(FlutterMetalExternalTexture);
    texture->num_textures = 1;
    texture->height = h;
    texture->width = w;
    texture->pixel_format = FlutterMetalExternalTexturePixelFormat::kRGBA;
    texture->textures = textures.data();
    return texture;
  };

  // Render the texture.
  std::unique_ptr<flutter::Texture> texture =
      std::make_unique<EmbedderExternalTextureMetal>(texture_id, callback);
  SkRect bounds = SkRect::MakeWH(info.width(), info.height());
  DlImageSampling sampling = DlImageSampling::kNearestNeighbor;

  DisplayListBuilder builder;
  flutter::Texture::PaintContext context{
      .canvas = &builder, .gr_context = nullptr, .aiks_context = &aiks_context};
  texture->Paint(context, bounds, /*freeze=*/false, sampling);

  ASSERT_TRUE(mtlTexture != nil);
}

TEST_F(FlutterEmbedderExternalTextureTest, TestPopulateExternalTextureImpeller) {
  // Constants.
  const size_t width = 100;
  const size_t height = 100;
  const int64_t texture_id = 1;

  // Set up the surface.
  FlutterDarwinContextMetalSkia* darwinContextMetal =
      [[FlutterDarwinContextMetalSkia alloc] initWithDefaultMTLDevice];
  impeller::AiksContext aiks_context(CreateImpellerContext(), nullptr);
  SkImageInfo info = SkImageInfo::MakeN32Premul(width, height);

  // Create a texture.
  TestExternalTexture* testExternalTexture =
      [[TestExternalTexture alloc] initWidth:width
                                      height:height
                             pixelFormatType:kCVPixelFormatType_32BGRA];
  FlutterExternalTexture* textureHolder =
      [[FlutterExternalTexture alloc] initWithFlutterTexture:testExternalTexture
                                          darwinMetalContext:darwinContextMetal];

  // Callback to resolve the texture.
  EmbedderExternalTextureMetal::ExternalTextureCallback callback = [&](int64_t texture_id, size_t w,
                                                                       size_t h) {
    EXPECT_TRUE(w == width);
    EXPECT_TRUE(h == height);

    auto texture = std::make_unique<FlutterMetalExternalTexture>();
    [textureHolder populateTexture:texture.get()];

    EXPECT_TRUE(texture->num_textures == 1);
    EXPECT_TRUE(texture->textures != nullptr);
    EXPECT_TRUE(texture->pixel_format == FlutterMetalExternalTexturePixelFormat::kRGBA);
    return texture;
  };

  // Render the texture.
  std::unique_ptr<flutter::Texture> texture =
      std::make_unique<EmbedderExternalTextureMetal>(texture_id, callback);
  SkRect bounds = SkRect::MakeWH(info.width(), info.height());
  DlImageSampling sampling = DlImageSampling::kNearestNeighbor;

  DisplayListBuilder builder;
  flutter::Texture::PaintContext context{
      .canvas = &builder,
      .gr_context = nullptr,
      .aiks_context = &aiks_context,
  };
  texture->Paint(context, bounds, /*freeze=*/false, sampling);
}

TEST_F(FlutterEmbedderExternalTextureTest, TestPopulateExternalTextureYUVAImpeller) {
  // Constants.
  const size_t width = 100;
  const size_t height = 100;
  const int64_t texture_id = 1;

  // Set up the surface.
  FlutterDarwinContextMetalSkia* darwinContextMetal =
      [[FlutterDarwinContextMetalSkia alloc] initWithDefaultMTLDevice];
  impeller::AiksContext aiks_context(CreateImpellerContext(), nullptr);
  SkImageInfo info = SkImageInfo::MakeN32Premul(width, height);

  // Create a texture.
  TestExternalTexture* testExternalTexture =
      [[TestExternalTexture alloc] initWidth:width
                                      height:height
                             pixelFormatType:kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange];
  FlutterExternalTexture* textureHolder =
      [[FlutterExternalTexture alloc] initWithFlutterTexture:testExternalTexture
                                          darwinMetalContext:darwinContextMetal];

  // Callback to resolve the texture.
  EmbedderExternalTextureMetal::ExternalTextureCallback callback = [&](int64_t texture_id, size_t w,
                                                                       size_t h) {
    EXPECT_TRUE(w == width);
    EXPECT_TRUE(h == height);

    auto texture = std::make_unique<FlutterMetalExternalTexture>();
    [textureHolder populateTexture:texture.get()];

    EXPECT_TRUE(texture->num_textures == 2);
    EXPECT_TRUE(texture->textures != nullptr);
    EXPECT_TRUE(texture->pixel_format == FlutterMetalExternalTexturePixelFormat::kYUVA);
    EXPECT_TRUE(texture->yuv_color_space ==
                FlutterMetalExternalTextureYUVColorSpace::kBT601LimitedRange);
    return texture;
  };

  // Render the texture.
  std::unique_ptr<flutter::Texture> texture =
      std::make_unique<EmbedderExternalTextureMetal>(texture_id, callback);
  SkRect bounds = SkRect::MakeWH(info.width(), info.height());
  DlImageSampling sampling = DlImageSampling::kNearestNeighbor;

  DisplayListBuilder builder;
  flutter::Texture::PaintContext context{
      .canvas = &builder, .gr_context = nullptr, .aiks_context = &aiks_context};
  texture->Paint(context, bounds, /*freeze=*/false, sampling);
}

TEST_F(FlutterEmbedderExternalTextureTest, TestPopulateExternalTextureYUVA2Impeller) {
  // Constants.
  const size_t width = 100;
  const size_t height = 100;
  const int64_t texture_id = 1;

  // Set up the surface.
  FlutterDarwinContextMetalSkia* darwinContextMetal =
      [[FlutterDarwinContextMetalSkia alloc] initWithDefaultMTLDevice];
  impeller::AiksContext aiks_context(CreateImpellerContext(), nullptr);
  SkImageInfo info = SkImageInfo::MakeN32Premul(width, height);

  // Create a texture.
  TestExternalTexture* testExternalTexture =
      [[TestExternalTexture alloc] initWidth:width
                                      height:height
                             pixelFormatType:kCVPixelFormatType_420YpCbCr8BiPlanarFullRange];
  FlutterExternalTexture* textureHolder =
      [[FlutterExternalTexture alloc] initWithFlutterTexture:testExternalTexture
                                          darwinMetalContext:darwinContextMetal];

  // Callback to resolve the texture.
  EmbedderExternalTextureMetal::ExternalTextureCallback callback = [&](int64_t texture_id, size_t w,
                                                                       size_t h) {
    EXPECT_TRUE(w == width);
    EXPECT_TRUE(h == height);

    auto texture = std::make_unique<FlutterMetalExternalTexture>();
    [textureHolder populateTexture:texture.get()];

    EXPECT_TRUE(texture->num_textures == 2);
    EXPECT_TRUE(texture->textures != nullptr);
    EXPECT_TRUE(texture->pixel_format == FlutterMetalExternalTexturePixelFormat::kYUVA);
    EXPECT_TRUE(texture->yuv_color_space ==
                FlutterMetalExternalTextureYUVColorSpace::kBT601FullRange);
    return texture;
  };

  // Render the texture.
  std::unique_ptr<flutter::Texture> texture =
      std::make_unique<EmbedderExternalTextureMetal>(texture_id, callback);
  SkRect bounds = SkRect::MakeWH(info.width(), info.height());
  DlImageSampling sampling = DlImageSampling::kNearestNeighbor;

  DisplayListBuilder builder;
  flutter::Texture::PaintContext context{
      .canvas = &builder, .gr_context = nullptr, .aiks_context = &aiks_context};
  texture->Paint(context, bounds, /*freeze=*/false, sampling);
}

TEST_F(FlutterEmbedderExternalTextureTest, TestPopulateUnsupportedExternalTextureImpeller) {
  // Constants.
  const size_t width = 100;
  const size_t height = 100;
  const int64_t texture_id = 1;

  // Set up the surface.
  FlutterDarwinContextMetalSkia* darwinContextMetal =
      [[FlutterDarwinContextMetalSkia alloc] initWithDefaultMTLDevice];
  impeller::AiksContext aiks_context(CreateImpellerContext(), nullptr);
  SkImageInfo info = SkImageInfo::MakeN32Premul(width, height);

  // Create a texture.
  TestExternalTexture* testExternalTexture =
      [[TestExternalTexture alloc] initWidth:width
                                      height:height
                             pixelFormatType:kCVPixelFormatType_420YpCbCr8PlanarFullRange];
  FlutterExternalTexture* textureHolder =
      [[FlutterExternalTexture alloc] initWithFlutterTexture:testExternalTexture
                                          darwinMetalContext:darwinContextMetal];

  // Callback to resolve the texture.
  EmbedderExternalTextureMetal::ExternalTextureCallback callback = [&](int64_t texture_id, size_t w,
                                                                       size_t h) {
    EXPECT_TRUE(w == width);
    EXPECT_TRUE(h == height);

    auto texture = std::make_unique<FlutterMetalExternalTexture>();
    EXPECT_FALSE([textureHolder populateTexture:texture.get()]);
    return nullptr;
  };

  // Render the texture.
  std::unique_ptr<flutter::Texture> texture =
      std::make_unique<EmbedderExternalTextureMetal>(texture_id, callback);
  SkRect bounds = SkRect::MakeWH(info.width(), info.height());
  DlImageSampling sampling = DlImageSampling::kNearestNeighbor;

  DisplayListBuilder builder;
  flutter::Texture::PaintContext context{
      .canvas = &builder, .gr_context = nullptr, .aiks_context = &aiks_context};
  texture->Paint(context, bounds, /*freeze=*/false, sampling);
}

}  // namespace flutter::testing
