// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#define FML_USED_ON_EMBEDDER

#include <string>
#include <vector>

#import <Metal/Metal.h>

#include "embedder.h"
#include "flutter/display_list/skia/dl_sk_canvas.h"
#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/shell/platform/embedder/tests/embedder_assertions.h"
#include "flutter/shell/platform/embedder/tests/embedder_config_builder.h"
#include "flutter/shell/platform/embedder/tests/embedder_test.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_context_metal.h"
#include "flutter/shell/platform/embedder/tests/embedder_unittests_util.h"
#include "flutter/testing/assertions_skia.h"
#include "flutter/testing/testing.h"

#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/gpu/GpuTypes.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"
#include "third_party/skia/include/gpu/ganesh/mtl/GrMtlBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/mtl/GrMtlTypes.h"

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace flutter {
namespace testing {

using EmbedderTest = testing::EmbedderTest;

TEST_F(EmbedderTest, CanRenderGradientWithMetal) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();

  EmbedderConfigBuilder builder(context);
  builder.SetDartEntrypoint("render_gradient");
  builder.SetSurface(SkISize::Make(800, 600));

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event), kSuccess);

  // TODO (https://github.com/flutter/flutter/issues/73590): re-enable once
  // we are able to figure out why this fails on the bots.
  // ASSERT_TRUE(ImageMatchesFixture("gradient_metal.png", rendered_scene));
}

static sk_sp<SkSurface> GetSurfaceFromTexture(const sk_sp<GrDirectContext>& skia_context,
                                              SkISize texture_size,
                                              void* texture) {
  GrMtlTextureInfo info;
  info.fTexture.retain(texture);
  GrBackendTexture backend_texture = GrBackendTextures::MakeMtl(
      texture_size.width(), texture_size.height(), skgpu::Mipmapped::kNo, info);

  return SkSurfaces::WrapBackendTexture(skia_context.get(), backend_texture,
                                        kTopLeft_GrSurfaceOrigin, 1, kBGRA_8888_SkColorType,
                                        nullptr, nullptr);
}

TEST_F(EmbedderTest, ExternalTextureMetal) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();

  const auto texture_size = SkISize::Make(800, 600);
  const int64_t texture_id = 1;

  TestMetalContext* metal_context = context.GetTestMetalContext();
  TestMetalContext::TextureInfo texture_info = metal_context->CreateMetalTexture(texture_size);

  sk_sp<SkSurface> surface =
      GetSurfaceFromTexture(metal_context->GetSkiaContext(), texture_size, texture_info.texture);
  auto canvas = surface->getCanvas();
  canvas->clear(SK_ColorRED);
  metal_context->GetSkiaContext()->flushAndSubmit();

  std::vector<FlutterMetalTextureHandle> textures{texture_info.texture};

  context.SetExternalTextureCallback(
      [&](int64_t id, size_t w, size_t h, FlutterMetalExternalTexture* output) {
        EXPECT_TRUE(w == texture_size.width());
        EXPECT_TRUE(h == texture_size.height());
        EXPECT_TRUE(texture_id == id);
        output->num_textures = 1;
        output->height = h;
        output->width = w;
        output->pixel_format = FlutterMetalExternalTexturePixelFormat::kRGBA;
        output->textures = textures.data();
        return true;
      });

  EmbedderConfigBuilder builder(context);

  builder.SetDartEntrypoint("render_texture");
  builder.SetSurface(texture_size);

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  ASSERT_EQ(FlutterEngineRegisterExternalTexture(engine.get(), texture_id), kSuccess);

  auto rendered_scene = context.GetNextSceneImage();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = texture_size.width();
  event.height = texture_size.height();
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event), kSuccess);

  ASSERT_TRUE(ImageMatchesFixture("external_texture_metal.png", rendered_scene));
}

TEST_F(EmbedderTest, MetalCompositorMustBeAbleToRenderPlatformViews) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views");

  builder.SetRenderTargetType(EmbedderTestBackingStoreProducer::RenderTargetType::kMetalTexture);

  fml::CountDownLatch latch(3);
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers, size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.struct_size = sizeof(backing_store);
          backing_store.type = kFlutterBackingStoreTypeMetal;
          backing_store.did_update = true;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 800, 600),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0, 0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[0], layer);
        }

        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 42;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(123.0, 456.0);
          layer.offset = FlutterPointMake(1.0, 2.0);

          ASSERT_EQ(*layers[1], layer);
        }

        {
          FlutterBackingStore backing_store = *layers[2]->backing_store;
          backing_store.struct_size = sizeof(backing_store);
          backing_store.type = kFlutterBackingStoreTypeMetal;
          backing_store.did_update = true;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(2, 3, 800, 600),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[2], layer);
        }

        latch.CountDown();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event), kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
}

TEST_F(EmbedderTest, CanRenderSceneWithoutCustomCompositorMetal) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();

  EmbedderConfigBuilder builder(context);

  builder.SetDartEntrypoint("can_render_scene_without_custom_compositor");
  builder.SetSurface(SkISize::Make(800, 600));

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event), kSuccess);

  ASSERT_TRUE(ImageMatchesFixture("scene_without_custom_compositor.png", rendered_scene));
}

TEST_F(EmbedderTest, TextureDestructionCallbackCalledWithoutCustomCompositorMetal) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetDartEntrypoint("texture_destruction_callback_called_without_custom_compositor");

  struct CollectContext {
    int present_count = 0;
    int collect_count = 0;
    fml::AutoResetWaitableEvent latch;
  };

  auto collect_context = std::make_unique<CollectContext>();
  context.SetNextDrawableCallback([&context, &collect_context](const FlutterFrameInfo* frame_info) {
    auto texture_info = context.GetTestMetalSurface()->GetTextureInfo();

    FlutterMetalTexture texture;
    texture.struct_size = sizeof(FlutterMetalTexture);
    texture.texture_id = texture_info.texture_id;
    texture.texture = reinterpret_cast<FlutterMetalTextureHandle>(texture_info.texture);
    texture.user_data = collect_context.get();
    texture.destruction_callback = [](void* user_data) {
      CollectContext* callback_collect_context = reinterpret_cast<CollectContext*>(user_data);
      ASSERT_TRUE(callback_collect_context->present_count > 0);
      callback_collect_context->collect_count++;
      callback_collect_context->latch.Signal();
    };
    return texture;
  });
  context.SetPresentCallback([&context, &collect_context](int64_t texture_id) {
    collect_context->present_count++;
    return context.GetTestMetalContext()->Present(texture_id);
  });

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event), kSuccess);
  ASSERT_TRUE(engine.is_valid());

  collect_context->latch.Wait();
  EXPECT_EQ(collect_context->collect_count, 1);
}

TEST_F(EmbedderTest, CompositorMustBeAbleToRenderKnownSceneMetal) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views_with_known_scene");

  builder.SetRenderTargetType(EmbedderTestBackingStoreProducer::RenderTargetType::kMetalTexture);

  fml::CountDownLatch latch(5);

  auto scene_image = context.GetNextSceneImage();

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers, size_t layers_count) {
        ASSERT_EQ(layers_count, 5u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeMetal;
          backing_store.did_update = true;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 800, 600),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[0], layer);
        }

        // Layer 1
        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 1;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(50.0, 150.0);
          layer.offset = FlutterPointMake(20.0, 20.0);

          ASSERT_EQ(*layers[1], layer);
        }

        // Layer 2
        {
          FlutterBackingStore backing_store = *layers[2]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeMetal;
          backing_store.did_update = true;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(30, 30, 80, 180),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[2], layer);
        }

        // Layer 3
        {
          FlutterPlatformView platform_view = *layers[3]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 2;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(50.0, 150.0);
          layer.offset = FlutterPointMake(40.0, 40.0);

          ASSERT_EQ(*layers[3], layer);
        }

        // Layer 4
        {
          FlutterBackingStore backing_store = *layers[4]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeMetal;
          backing_store.did_update = true;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(50, 50, 100, 200),
          };
          FlutterRegion paint_region = {
              .struct_size = sizeof(FlutterRegion),
              .rects_count = 1,
              .rects = paint_region_rects,
          };
          FlutterBackingStorePresentInfo present_info = {
              .struct_size = sizeof(FlutterBackingStorePresentInfo),
              .paint_region = &paint_region,
          };

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypeBackingStore;
          layer.backing_store = &backing_store;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[4], layer);
        }

        latch.CountDown();
      });

  context.GetCompositor().SetPlatformViewRendererCallback(
      [&](const FlutterLayer& layer, GrDirectContext* context) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(layer, context);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 1: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorGREEN);
            paint.setAlpha(127);
            const auto& rect = SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          case 2: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorMAGENTA);
            paint.setAlpha(127);
            const auto& rect = SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false) << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  // Flutter still thinks it is 800 x 600. Only the root surface is rotated.
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event), kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  ASSERT_TRUE(ImageMatchesFixture("compositor.png", scene_image));
}

TEST_F(EmbedderTest, CreateInvalidBackingstoreMetalTexture) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetRenderTargetType(EmbedderTestBackingStoreProducer::RenderTargetType::kMetalTexture);
  builder.SetDartEntrypoint("invalid_backingstore");

  class TestCollectOnce {
   public:
    // Collect() should only be called once
    void Collect() {
      ASSERT_FALSE(collected_);
      collected_ = true;
    }

   private:
    bool collected_ = false;
  };
  fml::AutoResetWaitableEvent latch;

  builder.GetCompositor().create_backing_store_callback =
      [](const FlutterBackingStoreConfig* config,  //
         FlutterBackingStore* backing_store_out,   //
         void* user_data                           //
      ) {
        backing_store_out->type = kFlutterBackingStoreTypeMetal;
        // Deliberately set this to be invalid
        backing_store_out->user_data = nullptr;
        backing_store_out->metal.texture.texture = 0;
        backing_store_out->metal.struct_size = sizeof(FlutterMetalBackingStore);
        backing_store_out->metal.texture.user_data = new TestCollectOnce();
        backing_store_out->metal.texture.destruction_callback = [](void* user_data) {
          reinterpret_cast<TestCollectOnce*>(user_data)->Collect();
        };
        return true;
      };

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&latch](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event), kSuccess);
  ASSERT_TRUE(engine.is_valid());
  latch.Wait();
}

TEST_F(EmbedderTest, ExternalTextureMetalRefreshedTooOften) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();

  TestMetalContext* metal_context = context.GetTestMetalContext();
  auto metal_texture = metal_context->CreateMetalTexture(SkISize::Make(100, 100));

  std::vector<FlutterMetalTextureHandle> textures{metal_texture.texture};

  bool resolve_called = false;

  EmbedderExternalTextureMetal::ExternalTextureCallback callback([&](int64_t id, size_t, size_t) {
    resolve_called = true;
    auto res = std::make_unique<FlutterMetalExternalTexture>();
    res->struct_size = sizeof(FlutterMetalExternalTexture);
    res->width = res->height = 100;
    res->pixel_format = FlutterMetalExternalTexturePixelFormat::kRGBA;
    res->textures = textures.data();
    res->num_textures = 1;
    return res;
  });
  EmbedderExternalTextureMetal texture(1, callback);

  auto surface = TestMetalSurface::Create(*metal_context, SkISize::Make(100, 100));
  auto skia_surface = surface->GetSurface();
  DlSkCanvasAdapter canvas(skia_surface->getCanvas());

  Texture* texture_ = &texture;
  DlImageSampling sampling = DlImageSampling::kLinear;
  Texture::PaintContext ctx{
      .canvas = &canvas,
      .gr_context = surface->GetGrContext().get(),
  };
  texture_->Paint(ctx, SkRect::MakeXYWH(0, 0, 100, 100), false, sampling);

  EXPECT_TRUE(resolve_called);
  resolve_called = false;

  texture_->Paint(ctx, SkRect::MakeXYWH(0, 0, 100, 100), false, sampling);

  EXPECT_FALSE(resolve_called);

  texture_->MarkNewFrameAvailable();

  texture_->Paint(ctx, SkRect::MakeXYWH(0, 0, 100, 100), false, sampling);

  EXPECT_TRUE(resolve_called);
}

TEST_F(EmbedderTest, CanRenderWithImpellerMetal) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();

  EmbedderConfigBuilder builder(context);

  builder.AddCommandLineArgument("--enable-impeller");
  builder.SetDartEntrypoint("render_impeller_test");
  builder.SetSurface(SkISize::Make(800, 600));

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event), kSuccess);

  ASSERT_TRUE(ImageMatchesFixture("impeller_test.png", rendered_scene));
}

TEST_F(EmbedderTest, CanRenderTextWithImpellerMetal) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();

  EmbedderConfigBuilder builder(context);

  builder.AddCommandLineArgument("--enable-impeller");
  builder.SetDartEntrypoint("render_impeller_text_test");
  builder.SetSurface(SkISize::Make(800, 600));

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event), kSuccess);

  ASSERT_TRUE(ImageMatchesFixture("impeller_text_test.png", rendered_scene));
}

TEST_F(EmbedderTest, CanRenderTextWithImpellerAndCompositorMetal) {
  auto& context = GetEmbedderContext<EmbedderTestContextMetal>();

  EmbedderConfigBuilder builder(context);

  builder.AddCommandLineArgument("--enable-impeller");
  builder.SetDartEntrypoint("render_impeller_text_test");
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();

  builder.SetRenderTargetType(EmbedderTestBackingStoreProducer::RenderTargetType::kMetalTexture);

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event), kSuccess);

  ASSERT_TRUE(ImageMatchesFixture("impeller_text_test.png", rendered_scene));
}

}  // namespace testing
}  // namespace flutter

// NOLINTEND(clang-analyzer-core.StackAddressEscape)
