// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tests/embedder_test_context.h"
#define FML_USED_ON_EMBEDDER

#include <atomic>
#include <string>
#include <vector>

#include "vulkan/vulkan.h"

#include "GLES3/gl3.h"
#include "flutter/flow/raster_cache.h"
#include "flutter/fml/file.h"
#include "flutter/fml/make_copyable.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/message_loop.h"
#include "flutter/fml/message_loop_task_queues.h"
#include "flutter/fml/native_library.h"
#include "flutter/fml/paths.h"
#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/fml/task_source.h"
#include "flutter/fml/thread.h"
#include "flutter/lib/ui/painting/image.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/platform/embedder/embedder_surface_gl_impeller.h"
#include "flutter/shell/platform/embedder/tests/embedder_assertions.h"
#include "flutter/shell/platform/embedder/tests/embedder_config_builder.h"
#include "flutter/shell/platform/embedder/tests/embedder_test.h"
#include "flutter/shell/platform/embedder/tests/embedder_test_context_gl.h"
#include "flutter/shell/platform/embedder/tests/embedder_unittests_util.h"
#include "flutter/testing/assertions_skia.h"
#include "flutter/testing/test_gl_surface.h"
#include "flutter/testing/testing.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/tonic/converter/dart_converter.h"

// CREATE_NATIVE_ENTRY is leaky by design
// NOLINTBEGIN(clang-analyzer-core.StackAddressEscape)

namespace flutter::testing {

using EmbedderTest = testing::EmbedderTest;

TEST_F(EmbedderTest, CanCreateOpenGLRenderingEngine) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
}

//------------------------------------------------------------------------------
/// If an incorrectly configured compositor is set on the engine, the engine
/// must fail to launch instead of failing to render a frame at a later point in
/// time.
///
TEST_F(EmbedderTest,
       MustPreventEngineLaunchWhenRequiredCompositorArgsAreAbsent) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetCompositor();
  builder.GetCompositor().create_backing_store_callback = nullptr;
  builder.GetCompositor().collect_backing_store_callback = nullptr;
  builder.GetCompositor().present_layers_callback = nullptr;
  builder.GetCompositor().present_view_callback = nullptr;
  auto engine = builder.LaunchEngine();
  ASSERT_FALSE(engine.is_valid());
}

//------------------------------------------------------------------------------
/// Either present_layers_callback or present_view_callback must be provided,
/// but not both, otherwise the engine must fail to launch instead of failing to
/// render a frame at a later point in time.
///
TEST_F(EmbedderTest, LaunchFailsWhenMultiplePresentCallbacks) {
  auto& context = GetEmbedderContext<EmbedderTestContextSoftware>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  builder.SetCompositor();
  builder.GetCompositor().present_layers_callback =
      [](const FlutterLayer** layers, size_t layers_count, void* user_data) {
        return true;
      };
  builder.GetCompositor().present_view_callback =
      [](const FlutterPresentViewInfo* info) { return true; };
  auto engine = builder.LaunchEngine();
  ASSERT_FALSE(engine.is_valid());
}

//------------------------------------------------------------------------------
/// Must be able to render to a custom compositor whose render targets are fully
/// complete OpenGL textures.
///
TEST_F(EmbedderTest, CompositorMustBeAbleToRenderToOpenGLFramebuffer) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  fml::CountDownLatch latch(3);
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.struct_size = sizeof(backing_store);
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeFramebuffer;

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
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeFramebuffer;

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
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
}

//------------------------------------------------------------------------------
/// Layers in a hierarchy containing a platform view should not be cached. The
/// other layers in the hierarchy should be, however.
TEST_F(EmbedderTest, RasterCacheDisabledWithPlatformViews) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views_with_opacity");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  fml::CountDownLatch setup(3);
  fml::CountDownLatch verify(1);

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.struct_size = sizeof(backing_store);
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeFramebuffer;

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
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeFramebuffer;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(3, 3, 800, 600),
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

        setup.CountDown();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&setup](Dart_NativeArguments args) { setup.CountDown(); }));

  UniqueEngine engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  setup.Wait();
  const flutter::Shell& shell = ToEmbedderEngine(engine.get())->GetShell();
  shell.GetTaskRunners().GetRasterTaskRunner()->PostTask([&] {
    const flutter::RasterCache& raster_cache =
        shell.GetRasterizer()->compositor_context()->raster_cache();
    // 3 layers total, but one of them had the platform view. So the cache
    // should only have 2 entries.
    ASSERT_EQ(raster_cache.GetCachedEntriesCount(), 2u);
    verify.CountDown();
  });

  verify.Wait();
}

//------------------------------------------------------------------------------
/// The RasterCache should normally be enabled.
///
TEST_F(EmbedderTest, RasterCacheEnabled) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_with_opacity");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  fml::CountDownLatch setup(3);
  fml::CountDownLatch verify(1);

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 1u);

        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.struct_size = sizeof(backing_store);
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeFramebuffer;

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

        setup.CountDown();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&setup](Dart_NativeArguments args) { setup.CountDown(); }));

  UniqueEngine engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  setup.Wait();
  const flutter::Shell& shell = ToEmbedderEngine(engine.get())->GetShell();
  shell.GetTaskRunners().GetRasterTaskRunner()->PostTask([&] {
    const flutter::RasterCache& raster_cache =
        shell.GetRasterizer()->compositor_context()->raster_cache();
    ASSERT_EQ(raster_cache.GetCachedEntriesCount(), 1u);
    verify.CountDown();
  });

  verify.Wait();
}

//------------------------------------------------------------------------------
/// Must be able to render using a custom compositor whose render targets for
/// the individual layers are OpenGL textures.
///
TEST_F(EmbedderTest, CompositorMustBeAbleToRenderToOpenGLTexture) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  fml::CountDownLatch latch(3);
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.struct_size = sizeof(backing_store);
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

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
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

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
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
}

//------------------------------------------------------------------------------
/// Must be able to render using a custom compositor whose render target for the
/// individual layers are software buffers.
///
TEST_F(EmbedderTest, CompositorMustBeAbleToRenderToSoftwareBuffer) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kSoftwareBuffer);

  fml::CountDownLatch latch(3);
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.struct_size = sizeof(backing_store);
          backing_store.type = kFlutterBackingStoreTypeSoftware;
          backing_store.did_update = true;
          ASSERT_FLOAT_EQ(
              backing_store.software.row_bytes * backing_store.software.height,
              800 * 4 * 600.0);

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
          backing_store.type = kFlutterBackingStoreTypeSoftware;
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
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
}

//------------------------------------------------------------------------------
/// Test the layer structure and pixels rendered when using a custom compositor.
///
TEST_F(EmbedderTest, CompositorMustBeAbleToRenderKnownScene) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views_with_known_scene");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  fml::CountDownLatch latch(5);

  auto scene_image = context.GetNextSceneImage();

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 5u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

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
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

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
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

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
      [&](const FlutterLayer& layer,
          GrDirectContext* context) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(layer, context);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 1: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorGREEN);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          case 2: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorMAGENTA);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false)
                << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  ASSERT_TRUE(ImageMatchesFixture("compositor.png", scene_image));

  // There should no present calls on the root surface.
  ASSERT_EQ(context.GetSurfacePresentCount(), 0u);
}

//------------------------------------------------------------------------------
/// Custom compositor must play nicely with a custom task runner. The raster
/// thread merging mechanism must not interfere with the custom compositor.
///
TEST_F(EmbedderTest, CustomCompositorMustWorkWithCustomTaskRunner) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);

  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views");

  auto platform_task_runner = CreateNewThread("test_platform_thread");
  static std::mutex engine_mutex;
  UniqueEngine engine;
  fml::AutoResetWaitableEvent sync_latch;

  EmbedderTestTaskRunner test_task_runner(
      platform_task_runner, [&](FlutterTask task) {
        std::scoped_lock lock(engine_mutex);
        if (!engine.is_valid()) {
          return;
        }
        ASSERT_EQ(FlutterEngineRunTask(engine.get(), &task), kSuccess);
      });

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  fml::CountDownLatch latch(3);
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.struct_size = sizeof(backing_store);
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

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
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(123.0, 456.0);
          layer.offset = FlutterPointMake(1.0, 2.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[1], layer);
        }

        {
          FlutterBackingStore backing_store = *layers[2]->backing_store;
          backing_store.struct_size = sizeof(backing_store);
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

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

  const auto task_runner_description =
      test_task_runner.GetFlutterTaskRunnerDescription();

  builder.SetPlatformTaskRunner(&task_runner_description);

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  platform_task_runner->PostTask([&]() {
    std::scoped_lock lock(engine_mutex);
    engine = builder.LaunchEngine();
    ASSERT_TRUE(engine.is_valid());

    // Send a window metrics events so frames may be scheduled.
    FlutterWindowMetricsEvent event = {};
    event.struct_size = sizeof(event);
    event.width = 800;
    event.height = 600;
    event.pixel_ratio = 1.0;
    ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
              kSuccess);
    ASSERT_TRUE(engine.is_valid());
    sync_latch.Signal();
  });
  sync_latch.Wait();

  latch.Wait();

  platform_task_runner->PostTask([&]() {
    std::scoped_lock lock(engine_mutex);
    engine.reset();
    sync_latch.Signal();
  });
  sync_latch.Wait();
}

//------------------------------------------------------------------------------
/// Test the layer structure and pixels rendered when using a custom compositor
/// and a single layer.
///
TEST_F(EmbedderTest, CompositorMustBeAbleToRenderWithRootLayerOnly) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint(
      "can_composite_platform_views_with_root_layer_only");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  fml::CountDownLatch latch(3);

  auto scene_image = context.GetNextSceneImage();

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 1u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

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

        latch.CountDown();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  ASSERT_TRUE(
      ImageMatchesFixture("compositor_with_root_layer_only.png", scene_image));
}

//------------------------------------------------------------------------------
/// Test the layer structure and pixels rendered when using a custom compositor
/// and ensure that a redundant layer is not added.
///
TEST_F(EmbedderTest, CompositorMustBeAbleToRenderWithPlatformLayerOnBottom) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint(
      "can_composite_platform_views_with_platform_layer_on_bottom");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  fml::CountDownLatch latch(3);

  auto scene_image = context.GetNextSceneImage();

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 2u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

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

        latch.CountDown();
      });

  context.GetCompositor().SetPlatformViewRendererCallback(
      [&](const FlutterLayer& layer,
          GrDirectContext* context) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(layer, context);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 1: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorGREEN);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false)
                << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  ASSERT_TRUE(ImageMatchesFixture(
      "compositor_with_platform_layer_on_bottom.png", scene_image));

  ASSERT_EQ(context.GetCompositor().GetPendingBackingStoresCount(), 1u);
}

//------------------------------------------------------------------------------
/// Test the layer structure and pixels rendered when using a custom compositor
/// with a root surface transformation.
///
TEST_F(EmbedderTest,
       CompositorMustBeAbleToRenderKnownSceneWithRootSurfaceTransformation) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(600, 800));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views_with_known_scene");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  // This must match the transformation provided in the
  // |CanRenderGradientWithoutCompositorWithXform| test to ensure that
  // transforms are consistent respected.
  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 800).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  fml::CountDownLatch latch(5);

  auto scene_image = context.GetNextSceneImage();

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 5u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 600, 800),
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
          layer.size = FlutterSizeMake(600.0, 800.0);
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
          layer.size = FlutterSizeMake(150.0, 50.0);
          layer.offset = FlutterPointMake(20.0, 730.0);

          ASSERT_EQ(*layers[1], layer);
        }

        // Layer 2
        {
          FlutterBackingStore backing_store = *layers[2]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(30, 720, 180, 770),
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
          layer.size = FlutterSizeMake(600.0, 800.0);
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
          layer.size = FlutterSizeMake(150.0, 50.0);
          layer.offset = FlutterPointMake(40.0, 710.0);

          ASSERT_EQ(*layers[3], layer);
        }

        // Layer 4
        {
          FlutterBackingStore backing_store = *layers[4]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(50, 700, 200, 750),
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
          layer.size = FlutterSizeMake(600.0, 800.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[4], layer);
        }

        latch.CountDown();
      });

  context.GetCompositor().SetPlatformViewRendererCallback(
      [&](const FlutterLayer& layer,
          GrDirectContext* context) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(layer, context);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 1: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorGREEN);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          case 2: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorMAGENTA);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false)
                << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  // Flutter still thinks it is 800 x 600. Only the root surface is rotated.
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  ASSERT_TRUE(ImageMatchesFixture("compositor_root_surface_xformation.png",
                                  scene_image));
}

TEST_F(EmbedderTest, CanRenderSceneWithoutCustomCompositor) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

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
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  ASSERT_TRUE(ImageMatchesFixture("scene_without_custom_compositor.png",
                                  rendered_scene));
}

TEST_F(EmbedderTest, CanRenderSceneWithoutCustomCompositorWithTransformation) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 800).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  EmbedderConfigBuilder builder(context);

  builder.SetDartEntrypoint("can_render_scene_without_custom_compositor");
  builder.SetSurface(SkISize::Make(600, 800));

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);

  // Flutter still thinks it is 800 x 600.
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  ASSERT_TRUE(ImageMatchesFixture(
      "scene_without_custom_compositor_with_xform.png", rendered_scene));
}

TEST_P(EmbedderTestMultiBackend, CanRenderGradientWithoutCompositor) {
  EmbedderTestContextType backend = GetParam();
  auto& context = GetEmbedderContext(backend);
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
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  ASSERT_TRUE(ImageMatchesFixture(
      FixtureNameForBackend(backend, "gradient.png"), rendered_scene));
}

TEST_F(EmbedderTest, CanRenderGradientWithoutCompositorWithXform) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 800).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  EmbedderConfigBuilder builder(context);

  const auto surface_size = SkISize::Make(600, 800);

  builder.SetDartEntrypoint("render_gradient");
  builder.SetSurface(surface_size);

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  // Flutter still thinks it is 800 x 600.
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  ASSERT_TRUE(ImageMatchesFixture("gradient_xform.png", rendered_scene));
}

TEST_P(EmbedderTestMultiBackend, CanRenderGradientWithCompositor) {
  EmbedderTestContextType backend = GetParam();
  auto& context = GetEmbedderContext(backend);

  EmbedderConfigBuilder builder(context);
  builder.SetDartEntrypoint("render_gradient");
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetRenderTargetType(GetRenderTargetFromBackend(backend, true));

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  ASSERT_TRUE(ImageMatchesFixture(
      FixtureNameForBackend(backend, "gradient.png"), rendered_scene));
}

TEST_F(EmbedderTest, CanRenderGradientWithCompositorWithXform) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  // This must match the transformation provided in the
  // |CanRenderGradientWithoutCompositorWithXform| test to ensure that
  // transforms are consistent respected.
  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 800).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  EmbedderConfigBuilder builder(context);

  builder.SetDartEntrypoint("render_gradient");
  builder.SetSurface(SkISize::Make(600, 800));
  builder.SetCompositor();
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  // Flutter still thinks it is 800 x 600.
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  ASSERT_TRUE(ImageMatchesFixture("gradient_xform.png", rendered_scene));
}

TEST_P(EmbedderTestMultiBackend,
       CanRenderGradientWithCompositorOnNonRootLayer) {
  EmbedderTestContextType backend = GetParam();
  auto& context = GetEmbedderContext(backend);

  EmbedderConfigBuilder builder(context);
  builder.SetDartEntrypoint("render_gradient_on_non_root_backing_store");
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetRenderTargetType(GetRenderTargetFromBackend(backend, true));

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.did_update = true;
          ConfigureBackingStore(backing_store, backend, true);

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
          layer.size = FlutterSizeMake(100.0, 200.0);
          layer.offset = FlutterPointMake(0.0, 0.0);

          ASSERT_EQ(*layers[1], layer);
        }

        // Layer 2
        {
          FlutterBackingStore backing_store = *layers[2]->backing_store;
          backing_store.did_update = true;
          ConfigureBackingStore(backing_store, backend, true);

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

          ASSERT_EQ(*layers[2], layer);
        }
      });

  context.GetCompositor().SetPlatformViewRendererCallback(
      [&](const FlutterLayer& layer,
          GrDirectContext* context) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(layer, context);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 1: {
            FML_CHECK(layer.size.width == 100);
            FML_CHECK(layer.size.height == 200);
            // This is occluded anyway. We just want to make sure we see this.
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false)
                << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  ASSERT_TRUE(ImageMatchesFixture(
      FixtureNameForBackend(backend, "gradient.png"), rendered_scene));
}

TEST_F(EmbedderTest, CanRenderGradientWithCompositorOnNonRootLayerWithXform) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  // This must match the transformation provided in the
  // |CanRenderGradientWithoutCompositorWithXform| test to ensure that
  // transforms are consistent respected.
  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 800).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  EmbedderConfigBuilder builder(context);

  builder.SetDartEntrypoint("render_gradient_on_non_root_backing_store");
  builder.SetSurface(SkISize::Make(600, 800));
  builder.SetCompositor();
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeFramebuffer;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 600, 800),
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
          layer.size = FlutterSizeMake(600.0, 800.0);
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
          layer.size = FlutterSizeMake(200.0, 100.0);
          layer.offset = FlutterPointMake(0.0, 700.0);

          ASSERT_EQ(*layers[1], layer);
        }

        // Layer 2
        {
          FlutterBackingStore backing_store = *layers[2]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeFramebuffer;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 600, 800),
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
          layer.size = FlutterSizeMake(600.0, 800.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[2], layer);
        }
      });

  context.GetCompositor().SetPlatformViewRendererCallback(
      [&](const FlutterLayer& layer,
          GrDirectContext* context) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(layer, context);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 1: {
            FML_CHECK(layer.size.width == 200);
            FML_CHECK(layer.size.height == 100);
            // This is occluded anyway. We just want to make sure we see this.
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false)
                << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  // Flutter still thinks it is 800 x 600.
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  ASSERT_TRUE(ImageMatchesFixture("gradient_xform.png", rendered_scene));
}

TEST_F(EmbedderTest, VerifyB141980393) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);

  // The Flutter application is 800 x 600 but rendering on a surface that is 600
  // x 800 achieved using a root surface transformation.
  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 800).preRotate(-90, 0, 0);
  const auto flutter_application_rect = SkRect::MakeWH(800, 600);
  const auto root_surface_rect =
      root_surface_transformation.mapRect(flutter_application_rect);

  ASSERT_DOUBLE_EQ(root_surface_rect.width(), 600.0);
  ASSERT_DOUBLE_EQ(root_surface_rect.height(), 800.0);

  // Configure the fixture for the surface transformation.
  context.SetRootSurfaceTransformation(root_surface_transformation);

  // Configure the Flutter project args for the root surface transformation.
  builder.SetSurface(
      SkISize::Make(root_surface_rect.width(), root_surface_rect.height()));

  // Use a compositor instead of rendering directly to the surface.
  builder.SetCompositor();
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  builder.SetDartEntrypoint("verify_b141980393");

  fml::AutoResetWaitableEvent latch;

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 1u);

        // Layer Root
        {
          FlutterPlatformView platform_view = *layers[0]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 1337;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;

          // From the Dart side. These dimensions match those specified in Dart
          // code and are free of root surface transformations.
          const double unxformed_top_margin = 31.0;
          const double unxformed_bottom_margin = 37.0;
          const auto unxformed_platform_view_rect = SkRect::MakeXYWH(
              0.0,                   // x
              unxformed_top_margin,  // y (top margin)
              800,                   // width
              600 - unxformed_top_margin - unxformed_bottom_margin  // height
          );

          // The platform views are in the coordinate space of the root surface
          // with top-left origin. The embedder has specified a transformation
          // to this surface which it must account for in the coordinates it
          // receives here.
          const auto xformed_platform_view_rect =
              root_surface_transformation.mapRect(unxformed_platform_view_rect);

          // Spell out the value that we are going to be checking below for
          // clarity.
          ASSERT_EQ(xformed_platform_view_rect,
                    SkRect::MakeXYWH(31.0,   // x
                                     0.0,    // y
                                     532.0,  // width
                                     800.0   // height
                                     ));

          // Verify that the engine is giving us the right size and offset.
          layer.offset = FlutterPointMake(xformed_platform_view_rect.x(),
                                          xformed_platform_view_rect.y());
          layer.size = FlutterSizeMake(xformed_platform_view_rect.width(),
                                       xformed_platform_view_rect.height());

          ASSERT_EQ(*layers[0], layer);
        }

        latch.Signal();
      });

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);

  // The Flutter application is 800 x 600 rendering on a surface 600 x 800
  // achieved via a root surface transformation.
  event.width = flutter_application_rect.width();
  event.height = flutter_application_rect.height();
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
}

//------------------------------------------------------------------------------
/// Asserts that embedders can provide a task runner for the render thread.
///
TEST_F(EmbedderTest, CanCreateEmbedderWithCustomRenderTaskRunner) {
  std::mutex engine_mutex;
  UniqueEngine engine;
  fml::AutoResetWaitableEvent task_latch;
  bool task_executed = false;
  EmbedderTestTaskRunner render_task_runner(
      CreateNewThread("custom_render_thread"), [&](FlutterTask task) {
        std::scoped_lock engine_lock(engine_mutex);
        if (engine.is_valid()) {
          ASSERT_EQ(FlutterEngineRunTask(engine.get(), &task), kSuccess);
          task_executed = true;
          task_latch.Signal();
        }
      });
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  EmbedderConfigBuilder builder(context);
  builder.SetDartEntrypoint("can_render_scene_without_custom_compositor");
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetRenderTaskRunner(
      &render_task_runner.GetFlutterTaskRunnerDescription());

  {
    std::scoped_lock lock(engine_mutex);
    engine = builder.InitializeEngine();
  }

  ASSERT_EQ(FlutterEngineRunInitialized(engine.get()), kSuccess);

  ASSERT_TRUE(engine.is_valid());

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  task_latch.Wait();
  ASSERT_TRUE(task_executed);
  ASSERT_EQ(FlutterEngineDeinitialize(engine.get()), kSuccess);

  {
    std::scoped_lock engine_lock(engine_mutex);
    engine.reset();
  }
}

//------------------------------------------------------------------------------
/// Asserts that the render task runner can be the same as the platform task
/// runner.
///
TEST_P(EmbedderTestMultiBackend,
       CanCreateEmbedderWithCustomRenderTaskRunnerTheSameAsPlatformTaskRunner) {
  // A new thread needs to be created for the platform thread because the test
  // can't wait for assertions to be completed on the same thread that services
  // platform task runner tasks.
  auto platform_task_runner = CreateNewThread("platform_thread");

  static std::mutex engine_mutex;
  static UniqueEngine engine;
  fml::AutoResetWaitableEvent task_latch;
  bool task_executed = false;
  EmbedderTestTaskRunner common_task_runner(
      platform_task_runner, [&](FlutterTask task) {
        std::scoped_lock engine_lock(engine_mutex);
        if (engine.is_valid()) {
          ASSERT_EQ(FlutterEngineRunTask(engine.get(), &task), kSuccess);
          task_executed = true;
          task_latch.Signal();
        }
      });

  platform_task_runner->PostTask([&]() {
    EmbedderTestContextType backend = GetParam();
    auto& context = GetEmbedderContext(backend);
    EmbedderConfigBuilder builder(context);
    builder.SetDartEntrypoint("can_render_scene_without_custom_compositor");
    builder.SetSurface(SkISize::Make(800, 600));
    builder.SetRenderTaskRunner(
        &common_task_runner.GetFlutterTaskRunnerDescription());
    builder.SetPlatformTaskRunner(
        &common_task_runner.GetFlutterTaskRunnerDescription());

    {
      std::scoped_lock lock(engine_mutex);
      engine = builder.InitializeEngine();
    }

    ASSERT_EQ(FlutterEngineRunInitialized(engine.get()), kSuccess);

    ASSERT_TRUE(engine.is_valid());

    FlutterWindowMetricsEvent event = {};
    event.struct_size = sizeof(event);
    event.width = 800;
    event.height = 600;
    event.pixel_ratio = 1.0;
    ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
              kSuccess);
  });

  task_latch.Wait();

  // Don't use the task latch because that may be called multiple time
  // (including during the shutdown process).
  fml::AutoResetWaitableEvent shutdown_latch;

  platform_task_runner->PostTask([&]() {
    ASSERT_TRUE(task_executed);
    ASSERT_EQ(FlutterEngineDeinitialize(engine.get()), kSuccess);

    {
      std::scoped_lock engine_lock(engine_mutex);
      engine.reset();
    }
    shutdown_latch.Signal();
  });

  shutdown_latch.Wait();

  {
    std::scoped_lock engine_lock(engine_mutex);
    // Engine should have been killed by this point.
    ASSERT_FALSE(engine.is_valid());
  }
}

TEST_P(EmbedderTestMultiBackend,
       CompositorMustBeAbleToRenderKnownScenePixelRatioOnSurface) {
  EmbedderTestContextType backend = GetParam();
  auto& context = GetEmbedderContext(backend);

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_display_platform_view_with_pixel_ratio");

  builder.SetRenderTargetType(GetRenderTargetFromBackend(backend, false));

  fml::CountDownLatch latch(1);

  auto rendered_scene = context.GetNextSceneImage();

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        // Layer 0 (Root)
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.did_update = true;
          ConfigureBackingStore(backing_store, backend, false);

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
          platform_view.identifier = 42;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(800.0, 560.0);
          layer.offset = FlutterPointMake(0.0, 40.0);

          ASSERT_EQ(*layers[1], layer);
        }

        // Layer 2
        {
          FlutterBackingStore backing_store = *layers[2]->backing_store;
          backing_store.did_update = true;
          ConfigureBackingStore(backing_store, backend, false);

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

          ASSERT_EQ(*layers[2], layer);
        }

        latch.CountDown();
      });

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 400 * 2.0;
  event.height = 300 * 2.0;
  event.pixel_ratio = 2.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  ASSERT_TRUE(ImageMatchesFixture(
      FixtureNameForBackend(backend, "dpr_noxform.png"), rendered_scene));
}

TEST_F(
    EmbedderTest,
    CompositorMustBeAbleToRenderKnownScenePixelRatioOnSurfaceWithRootSurfaceXformation) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(600, 800));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_display_platform_view_with_pixel_ratio");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 800).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  auto rendered_scene = context.GetNextSceneImage();
  fml::CountDownLatch latch(1);

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        // Layer 0 (Root)
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 600, 800),
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
          layer.size = FlutterSizeMake(600.0, 800.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[0], layer);
        }

        // Layer 1
        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 42;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(560.0, 800.0);
          layer.offset = FlutterPointMake(40.0, 0.0);

          ASSERT_EQ(*layers[1], layer);
        }

        // Layer 2
        {
          FlutterBackingStore backing_store = *layers[2]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 600, 800),
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
          layer.size = FlutterSizeMake(600.0, 800.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[2], layer);
        }

        latch.CountDown();
      });

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 400 * 2.0;
  event.height = 300 * 2.0;
  event.pixel_ratio = 2.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  ASSERT_TRUE(ImageMatchesFixture("dpr_xform.png", rendered_scene));
}

TEST_F(EmbedderTest,
       PushingMutlipleFramesSetsUpNewRecordingCanvasWithCustomCompositor) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(600, 1024));
  builder.SetCompositor();
  builder.SetDartEntrypoint("push_frames_over_and_over");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 1024).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 1024;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  constexpr size_t frames_expected = 10;
  fml::CountDownLatch frame_latch(frames_expected);
  std::atomic_size_t frames_seen = 0;
  context.AddNativeCallback("SignalNativeTest",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              frames_seen++;
                              frame_latch.CountDown();
                            }));
  frame_latch.Wait();

  ASSERT_GE(frames_seen, frames_expected);

  FlutterEngineShutdown(engine.release());
}

TEST_F(EmbedderTest,
       PushingMutlipleFramesSetsUpNewRecordingCanvasWithoutCustomCompositor) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(600, 1024));
  builder.SetDartEntrypoint("push_frames_over_and_over");

  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 1024).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 1024;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  constexpr size_t frames_expected = 10;
  fml::CountDownLatch frame_latch(frames_expected);
  std::atomic_size_t frames_seen = 0;
  context.AddNativeCallback("SignalNativeTest",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              frames_seen++;
                              frame_latch.CountDown();
                            }));
  frame_latch.Wait();

  ASSERT_GE(frames_seen, frames_expected);

  FlutterEngineShutdown(engine.release());
}

TEST_P(EmbedderTestMultiBackend, PlatformViewMutatorsAreValid) {
  EmbedderTestContextType backend = GetParam();
  auto& context = GetEmbedderContext(backend);

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("platform_view_mutators");

  builder.SetRenderTargetType(GetRenderTargetFromBackend(backend, false));

  fml::CountDownLatch latch(1);
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 2u);

        // Layer 0 (Root)
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.did_update = true;
          ConfigureBackingStore(backing_store, backend, false);

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

        // Layer 2
        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 42;
          platform_view.mutations_count = 3;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);

          ASSERT_EQ(*layers[1], layer);

          // There are no ordering guarantees.
          for (size_t i = 0; i < platform_view.mutations_count; i++) {
            FlutterPlatformViewMutation mutation = *platform_view.mutations[i];
            switch (mutation.type) {
              case kFlutterPlatformViewMutationTypeClipRoundedRect:
                mutation.clip_rounded_rect =
                    FlutterRoundedRectMake(SkRRect::MakeRectXY(
                        SkRect::MakeLTRB(10.0, 10.0, 800.0 - 10.0,
                                         600.0 - 10.0),
                        14.0, 14.0));
                break;
              case kFlutterPlatformViewMutationTypeClipRect:
                mutation.type = kFlutterPlatformViewMutationTypeClipRect;
                mutation.clip_rect = FlutterRectMake(
                    SkRect::MakeXYWH(10.0, 10.0, 800.0 - 20.0, 600.0 - 20.0));
                break;
              case kFlutterPlatformViewMutationTypeOpacity:
                mutation.type = kFlutterPlatformViewMutationTypeOpacity;
                mutation.opacity = 128.0 / 255.0;
                break;
              case kFlutterPlatformViewMutationTypeTransformation:
                FML_CHECK(false)
                    << "There should be no transformation in the test.";
                break;
            }

            ASSERT_EQ(*platform_view.mutations[i], mutation);
          }
        }
        latch.CountDown();
      });

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
}

TEST_F(EmbedderTest, PlatformViewMutatorsAreValidWithPixelRatio) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("platform_view_mutators_with_pixel_ratio");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  fml::CountDownLatch latch(1);
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 2u);

        // Layer 0 (Root)
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

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

        // Layer 2
        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 42;
          platform_view.mutations_count = 3;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(800.0, 600.0);
          layer.offset = FlutterPointMake(0.0, 0.0);

          ASSERT_EQ(*layers[1], layer);

          // There are no ordering guarantees.
          for (size_t i = 0; i < platform_view.mutations_count; i++) {
            FlutterPlatformViewMutation mutation = *platform_view.mutations[i];
            switch (mutation.type) {
              case kFlutterPlatformViewMutationTypeClipRoundedRect:
                mutation.clip_rounded_rect =
                    FlutterRoundedRectMake(SkRRect::MakeRectXY(
                        SkRect::MakeLTRB(5.0, 5.0, 400.0 - 5.0, 300.0 - 5.0),
                        7.0, 7.0));
                break;
              case kFlutterPlatformViewMutationTypeClipRect:
                mutation.type = kFlutterPlatformViewMutationTypeClipRect;
                mutation.clip_rect = FlutterRectMake(
                    SkRect::MakeXYWH(5.0, 5.0, 400.0 - 10.0, 300.0 - 10.0));
                break;
              case kFlutterPlatformViewMutationTypeOpacity:
                mutation.type = kFlutterPlatformViewMutationTypeOpacity;
                mutation.opacity = 128.0 / 255.0;
                break;
              case kFlutterPlatformViewMutationTypeTransformation:
                mutation.type = kFlutterPlatformViewMutationTypeTransformation;
                mutation.transformation =
                    FlutterTransformationMake(SkMatrix::Scale(2.0, 2.0));
                break;
            }

            ASSERT_EQ(*platform_view.mutations[i], mutation);
          }
        }
        latch.CountDown();
      });

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 2.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
}

TEST_F(EmbedderTest,
       PlatformViewMutatorsAreValidWithPixelRatioAndRootSurfaceTransformation) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("platform_view_mutators_with_pixel_ratio");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  static const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 800).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  fml::CountDownLatch latch(1);
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 2u);

        // Layer 0 (Root)
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeTexture;

          FlutterRect paint_region_rects[] = {
              FlutterRectMakeLTRB(0, 0, 600, 800),
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
          layer.size = FlutterSizeMake(600.0, 800.0);
          layer.offset = FlutterPointMake(0.0, 0.0);
          layer.backing_store_present_info = &present_info;

          ASSERT_EQ(*layers[0], layer);
        }

        // Layer 2
        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 42;
          platform_view.mutations_count = 4;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(600.0, 800.0);
          layer.offset = FlutterPointMake(0.0, 0.0);

          ASSERT_EQ(*layers[1], layer);

          // There are no ordering guarantees.
          for (size_t i = 0; i < platform_view.mutations_count; i++) {
            FlutterPlatformViewMutation mutation = *platform_view.mutations[i];
            switch (mutation.type) {
              case kFlutterPlatformViewMutationTypeClipRoundedRect:
                mutation.clip_rounded_rect =
                    FlutterRoundedRectMake(SkRRect::MakeRectXY(
                        SkRect::MakeLTRB(5.0, 5.0, 400.0 - 5.0, 300.0 - 5.0),
                        7.0, 7.0));
                break;
              case kFlutterPlatformViewMutationTypeClipRect:
                mutation.type = kFlutterPlatformViewMutationTypeClipRect;
                mutation.clip_rect = FlutterRectMake(
                    SkRect::MakeXYWH(5.0, 5.0, 400.0 - 10.0, 300.0 - 10.0));
                break;
              case kFlutterPlatformViewMutationTypeOpacity:
                mutation.type = kFlutterPlatformViewMutationTypeOpacity;
                mutation.opacity = 128.0 / 255.0;
                break;
              case kFlutterPlatformViewMutationTypeTransformation:
                mutation.type = kFlutterPlatformViewMutationTypeTransformation;
                mutation.transformation =
                    FlutterTransformationMake(root_surface_transformation);

                break;
            }

            ASSERT_EQ(*platform_view.mutations[i], mutation);
          }
        }
        latch.CountDown();
      });

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 2.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
}

TEST_F(EmbedderTest, EmptySceneIsAcceptable) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("empty_scene");
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();
}

TEST_F(EmbedderTest, SceneWithNoRootContainerIsAcceptable) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);
  builder.SetDartEntrypoint("scene_with_no_container");
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();
}

// Verifies that https://skia-review.googlesource.com/c/skia/+/259174 is pulled
// into the engine.
TEST_F(EmbedderTest, ArcEndCapsAreDrawnCorrectly) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 1024));
  builder.SetCompositor();
  builder.SetDartEntrypoint("arc_end_caps_correct");
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  const auto root_surface_transformation = SkMatrix()
                                               .preScale(1.0, -1.0)
                                               .preTranslate(1024.0, -800.0)
                                               .preRotate(90.0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  auto engine = builder.LaunchEngine();

  auto scene_image = context.GetNextSceneImage();

  ASSERT_TRUE(engine.is_valid());

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 1024;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  ASSERT_TRUE(ImageMatchesFixture("arc_end_caps.png", scene_image));
}

TEST_F(EmbedderTest, ClipsAreCorrectlyCalculated) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(400, 300));
  builder.SetCompositor();
  builder.SetDartEntrypoint("scene_builder_with_clips");
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 400).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  fml::AutoResetWaitableEvent latch;
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 2u);

        {
          FlutterPlatformView platform_view = *layers[0]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 42;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(300.0, 400.0);
          layer.offset = FlutterPointMake(0.0, 0.0);

          ASSERT_EQ(*layers[0], layer);

          bool clip_assertions_checked = false;

          // The total transformation on the stack upto the platform view.
          const auto total_xformation =
              GetTotalMutationTransformationMatrix(layers[0]->platform_view);

          FilterMutationsByType(
              layers[0]->platform_view,
              kFlutterPlatformViewMutationTypeClipRect,
              [&](const auto& mutation) {
                FlutterRect clip = mutation.clip_rect;

                // The test is only set up to supply one clip. Make sure it is
                // the one we expect.
                const auto rect_to_compare =
                    SkRect::MakeLTRB(10.0, 10.0, 390, 290);
                ASSERT_EQ(clip, FlutterRectMake(rect_to_compare));

                // This maps the clip from device space into surface space.
                SkRect mapped;
                ASSERT_TRUE(total_xformation.mapRect(&mapped, rect_to_compare));
                ASSERT_EQ(mapped, SkRect::MakeLTRB(10, 10, 290, 390));
                clip_assertions_checked = true;
              });

          ASSERT_TRUE(clip_assertions_checked);
        }

        latch.Signal();
      });

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 400;
  event.height = 300;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();
}

TEST_F(EmbedderTest, ComplexClipsAreCorrectlyCalculated) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1024, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("scene_builder_with_complex_clips");
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 1024).preRotate(-90, 0, 0);

  context.SetRootSurfaceTransformation(root_surface_transformation);

  fml::AutoResetWaitableEvent latch;
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 2u);

        {
          FlutterPlatformView platform_view = *layers[0]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 42;

          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(600.0, 1024.0);
          layer.offset = FlutterPointMake(0.0, -256.0);

          ASSERT_EQ(*layers[0], layer);

          const auto** mutations = platform_view.mutations;

          ASSERT_EQ(mutations[0]->type,
                    kFlutterPlatformViewMutationTypeTransformation);
          ASSERT_EQ(SkMatrixMake(mutations[0]->transformation),
                    root_surface_transformation);

          ASSERT_EQ(mutations[1]->type,
                    kFlutterPlatformViewMutationTypeClipRect);
          ASSERT_EQ(SkRectMake(mutations[1]->clip_rect),
                    SkRect::MakeLTRB(0.0, 0.0, 1024.0, 600.0));

          ASSERT_EQ(mutations[2]->type,
                    kFlutterPlatformViewMutationTypeTransformation);
          ASSERT_EQ(SkMatrixMake(mutations[2]->transformation),
                    SkMatrix::Translate(512.0, 0.0));

          ASSERT_EQ(mutations[3]->type,
                    kFlutterPlatformViewMutationTypeClipRect);
          ASSERT_EQ(SkRectMake(mutations[3]->clip_rect),
                    SkRect::MakeLTRB(0.0, 0.0, 512.0, 600.0));

          ASSERT_EQ(mutations[4]->type,
                    kFlutterPlatformViewMutationTypeTransformation);
          ASSERT_EQ(SkMatrixMake(mutations[4]->transformation),
                    SkMatrix::Translate(-256.0, 0.0));

          ASSERT_EQ(mutations[5]->type,
                    kFlutterPlatformViewMutationTypeClipRect);
          ASSERT_EQ(SkRectMake(mutations[5]->clip_rect),
                    SkRect::MakeLTRB(0.0, 0.0, 1024.0, 600.0));
        }

        latch.Signal();
      });

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 1024;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();
}

TEST_F(EmbedderTest, ObjectsCanBePostedViaPorts) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 1024));
  builder.SetDartEntrypoint("objects_can_be_posted");

  // Synchronously acquire the send port from the Dart end. We will be using
  // this to send message. The Dart end will just echo those messages back to us
  // for inspection.
  FlutterEngineDartPort port = 0;
  fml::AutoResetWaitableEvent event;
  context.AddNativeCallback("SignalNativeCount",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              port = tonic::DartConverter<int64_t>::FromDart(
                                  Dart_GetNativeArgument(args, 0));
                              event.Signal();
                            }));
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
  event.Wait();
  ASSERT_NE(port, 0);

  using Trampoline = std::function<void(Dart_Handle message)>;
  Trampoline trampoline;

  context.AddNativeCallback("SendObjectToNativeCode",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              FML_CHECK(trampoline);
                              auto trampoline_copy = trampoline;
                              trampoline = nullptr;
                              trampoline_copy(Dart_GetNativeArgument(args, 0));
                            }));

  // Check null.
  {
    FlutterEngineDartObject object = {};
    object.type = kFlutterEngineDartObjectTypeNull;
    trampoline = [&](Dart_Handle handle) {
      ASSERT_TRUE(Dart_IsNull(handle));
      event.Signal();
    };
    ASSERT_EQ(FlutterEnginePostDartObject(engine.get(), port, &object),
              kSuccess);
    event.Wait();
  }

  // Check bool.
  {
    FlutterEngineDartObject object = {};
    object.type = kFlutterEngineDartObjectTypeBool;
    object.bool_value = true;
    trampoline = [&](Dart_Handle handle) {
      ASSERT_TRUE(tonic::DartConverter<bool>::FromDart(handle));
      event.Signal();
    };
    ASSERT_EQ(FlutterEnginePostDartObject(engine.get(), port, &object),
              kSuccess);
    event.Wait();
  }

  // Check int32.
  {
    FlutterEngineDartObject object = {};
    object.type = kFlutterEngineDartObjectTypeInt32;
    object.int32_value = 1988;
    trampoline = [&](Dart_Handle handle) {
      ASSERT_EQ(tonic::DartConverter<int32_t>::FromDart(handle), 1988);
      event.Signal();
    };
    ASSERT_EQ(FlutterEnginePostDartObject(engine.get(), port, &object),
              kSuccess);
    event.Wait();
  }

  // Check int64.
  {
    FlutterEngineDartObject object = {};
    object.type = kFlutterEngineDartObjectTypeInt64;
    object.int64_value = 1988;
    trampoline = [&](Dart_Handle handle) {
      ASSERT_EQ(tonic::DartConverter<int64_t>::FromDart(handle), 1988);
      event.Signal();
    };
    ASSERT_EQ(FlutterEnginePostDartObject(engine.get(), port, &object),
              kSuccess);
    event.Wait();
  }

  // Check double.
  {
    FlutterEngineDartObject object = {};
    object.type = kFlutterEngineDartObjectTypeDouble;
    object.double_value = 1988.0;
    trampoline = [&](Dart_Handle handle) {
      ASSERT_DOUBLE_EQ(tonic::DartConverter<double>::FromDart(handle), 1988.0);
      event.Signal();
    };
    ASSERT_EQ(FlutterEnginePostDartObject(engine.get(), port, &object),
              kSuccess);
    event.Wait();
  }

  // Check string.
  {
    const char* message = "Hello. My name is Inigo Montoya.";
    FlutterEngineDartObject object = {};
    object.type = kFlutterEngineDartObjectTypeString;
    object.string_value = message;
    trampoline = [&](Dart_Handle handle) {
      ASSERT_EQ(tonic::DartConverter<std::string>::FromDart(handle),
                std::string{message});
      event.Signal();
    };
    ASSERT_EQ(FlutterEnginePostDartObject(engine.get(), port, &object),
              kSuccess);
    event.Wait();
  }

  // Check buffer (copied out).
  {
    std::vector<uint8_t> message;
    message.resize(1988);

    ASSERT_TRUE(MemsetPatternSetOrCheck(
        message, MemsetPatternOp::kMemsetPatternOpSetBuffer));

    FlutterEngineDartBuffer buffer = {};

    buffer.struct_size = sizeof(buffer);
    buffer.user_data = nullptr;
    buffer.buffer_collect_callback = nullptr;
    buffer.buffer = message.data();
    buffer.buffer_size = message.size();

    FlutterEngineDartObject object = {};
    object.type = kFlutterEngineDartObjectTypeBuffer;
    object.buffer_value = &buffer;
    trampoline = [&](Dart_Handle handle) {
      intptr_t length = 0;
      Dart_ListLength(handle, &length);
      ASSERT_EQ(length, 1988);
      // TODO(chinmaygarde); The std::vector<uint8_t> specialization for
      // DartConvertor in tonic is broken which is preventing the buffer
      // being checked here. Fix tonic and strengthen this check. For now, just
      // the buffer length is checked.
      event.Signal();
    };
    ASSERT_EQ(FlutterEnginePostDartObject(engine.get(), port, &object),
              kSuccess);
    event.Wait();
  }

  std::vector<uint8_t> message;
  fml::AutoResetWaitableEvent buffer_released_latch;

  // Check buffer (caller own buffer with zero copy transfer).
  {
    message.resize(1988);

    ASSERT_TRUE(MemsetPatternSetOrCheck(
        message, MemsetPatternOp::kMemsetPatternOpSetBuffer));

    FlutterEngineDartBuffer buffer = {};

    buffer.struct_size = sizeof(buffer);
    buffer.user_data = &buffer_released_latch;
    buffer.buffer_collect_callback = +[](void* user_data) {
      reinterpret_cast<fml::AutoResetWaitableEvent*>(user_data)->Signal();
    };
    buffer.buffer = message.data();
    buffer.buffer_size = message.size();

    FlutterEngineDartObject object = {};
    object.type = kFlutterEngineDartObjectTypeBuffer;
    object.buffer_value = &buffer;
    trampoline = [&](Dart_Handle handle) {
      intptr_t length = 0;
      Dart_ListLength(handle, &length);
      ASSERT_EQ(length, 1988);
      // TODO(chinmaygarde); The std::vector<uint8_t> specialization for
      // DartConvertor in tonic is broken which is preventing the buffer
      // being checked here. Fix tonic and strengthen this check. For now, just
      // the buffer length is checked.
      event.Signal();
    };
    ASSERT_EQ(FlutterEnginePostDartObject(engine.get(), port, &object),
              kSuccess);
    event.Wait();
  }

  engine.reset();

  // We cannot determine when the VM will GC objects that might have external
  // typed data finalizers. Since we need to ensure that we correctly wired up
  // finalizers from the embedders, we force the VM to collect all objects but
  // just shutting it down.
  buffer_released_latch.Wait();
}

TEST_F(EmbedderTest, CompositorCanPostZeroLayersForPresentation) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(300, 200));
  builder.SetCompositor();
  builder.SetDartEntrypoint("empty_scene_posts_zero_layers_to_compositor");
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  fml::AutoResetWaitableEvent latch;

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 0u);
        latch.Signal();
      });

  auto engine = builder.LaunchEngine();

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 300;
  event.height = 200;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());
  latch.Wait();

  ASSERT_EQ(context.GetCompositor().GetPendingBackingStoresCount(), 0u);
}

TEST_F(EmbedderTest, CompositorCanPostOnlyPlatformViews) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(300, 200));
  builder.SetCompositor();
  builder.SetDartEntrypoint("compositor_can_post_only_platform_views");
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  fml::AutoResetWaitableEvent latch;

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 2u);

        // Layer 0
        {
          FlutterPlatformView platform_view = *layers[0]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 42;
          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(300.0, 200.0);
          layer.offset = FlutterPointMake(0.0, 0.0);

          ASSERT_EQ(*layers[0], layer);
        }

        // Layer 1
        {
          FlutterPlatformView platform_view = *layers[1]->platform_view;
          platform_view.struct_size = sizeof(platform_view);
          platform_view.identifier = 24;
          FlutterLayer layer = {};
          layer.struct_size = sizeof(layer);
          layer.type = kFlutterLayerContentTypePlatformView;
          layer.platform_view = &platform_view;
          layer.size = FlutterSizeMake(300.0, 200.0);
          layer.offset = FlutterPointMake(0.0, 0.0);

          ASSERT_EQ(*layers[1], layer);
        }
        latch.Signal();
      });

  auto engine = builder.LaunchEngine();

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 300;
  event.height = 200;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());
  latch.Wait();

  ASSERT_EQ(context.GetCompositor().GetPendingBackingStoresCount(), 0u);
}

TEST_F(EmbedderTest, CompositorRenderTargetsAreRecycled) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(300, 200));
  builder.SetCompositor();
  builder.SetDartEntrypoint("render_targets_are_recycled");
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  fml::CountDownLatch latch(2);

  context.AddNativeCallback("SignalNativeTest",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              latch.CountDown();
                            }));

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 20u);
        latch.CountDown();
      });

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 300;
  event.height = 200;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();
  ASSERT_EQ(context.GetCompositor().GetPendingBackingStoresCount(), 10u);
  ASSERT_EQ(context.GetCompositor().GetBackingStoresCreatedCount(), 10u);
  ASSERT_EQ(context.GetCompositor().GetBackingStoresCollectedCount(), 0u);
  // Killing the engine should immediately collect all pending render targets.
  engine.reset();
  ASSERT_EQ(context.GetCompositor().GetPendingBackingStoresCount(), 0u);
  ASSERT_EQ(context.GetCompositor().GetBackingStoresCreatedCount(), 10u);
  ASSERT_EQ(context.GetCompositor().GetBackingStoresCollectedCount(), 10u);
}

TEST_F(EmbedderTest, CompositorRenderTargetsAreInStableOrder) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(300, 200));
  builder.SetCompositor();
  builder.SetDartEntrypoint("render_targets_are_in_stable_order");
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  fml::CountDownLatch latch(2);

  context.AddNativeCallback("SignalNativeTest",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              latch.CountDown();
                            }));

  size_t frame_count = 0;
  std::vector<void*> first_frame_backing_store_user_data;
  context.GetCompositor().SetPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 20u);

        if (first_frame_backing_store_user_data.empty()) {
          for (size_t i = 0; i < layers_count; ++i) {
            if (layers[i]->type == kFlutterLayerContentTypeBackingStore) {
              first_frame_backing_store_user_data.push_back(
                  layers[i]->backing_store->user_data);
            }
          }
          return;
        }

        ASSERT_EQ(first_frame_backing_store_user_data.size(), 10u);

        frame_count++;
        std::vector<void*> backing_store_user_data;
        for (size_t i = 0; i < layers_count; ++i) {
          if (layers[i]->type == kFlutterLayerContentTypeBackingStore) {
            backing_store_user_data.push_back(
                layers[i]->backing_store->user_data);
          }
        }

        ASSERT_EQ(backing_store_user_data.size(), 10u);

        ASSERT_EQ(first_frame_backing_store_user_data, backing_store_user_data);

        if (frame_count == 20) {
          latch.CountDown();
        }
      },
      false  // one shot
  );

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 300;
  event.height = 200;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();
}

TEST_F(EmbedderTest, FrameInfoContainsValidWidthAndHeight) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 1024).preRotate(-90, 0, 0);
  context.SetRootSurfaceTransformation(root_surface_transformation);

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(600, 1024));
  builder.SetDartEntrypoint("push_frames_over_and_over");
  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  static FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 1024;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  static fml::CountDownLatch frame_latch(10);

  context.AddNativeCallback("SignalNativeTest",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              /* Nothing to do. */
                            }));

  context.SetGLGetFBOCallback([](FlutterFrameInfo frame_info) {
    // width and height are rotated by 90 deg
    ASSERT_EQ(frame_info.size.width, event.height);
    ASSERT_EQ(frame_info.size.height, event.width);

    frame_latch.CountDown();
  });

  frame_latch.Wait();
}

TEST_F(EmbedderTest, MustNotRunWithBothFBOCallbacksSet) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.SetOpenGLFBOCallBack();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(600, 1024));

  auto engine = builder.LaunchEngine();
  ASSERT_FALSE(engine.is_valid());
}

TEST_F(EmbedderTest, MustNotRunWithBothPresentCallbacksSet) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.SetOpenGLPresentCallBack();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(600, 1024));

  auto engine = builder.LaunchEngine();
  ASSERT_FALSE(engine.is_valid());
}

TEST_F(EmbedderTest, MustStillRunWhenPopulateExistingDamageIsNotProvided) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.populate_existing_damage = nullptr;

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
}

TEST_F(EmbedderTest, MustRunWhenPopulateExistingDamageIsProvided) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.populate_existing_damage =
      [](void* context, const intptr_t id,
         FlutterDamage* existing_damage) -> void {
    return reinterpret_cast<EmbedderTestContextGL*>(context)
        ->GLPopulateExistingDamage(id, existing_damage);
  };

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
}

TEST_F(EmbedderTest, MustRunWithPopulateExistingDamageAndFBOCallback) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.fbo_callback =
      [](void* context) -> uint32_t { return 0; };
  context.GetRendererConfig().open_gl.fbo_with_frame_info_callback = nullptr;
  context.GetRendererConfig().open_gl.populate_existing_damage =
      [](void* context, const intptr_t id,
         FlutterDamage* existing_damage) -> void {
    return reinterpret_cast<EmbedderTestContextGL*>(context)
        ->GLPopulateExistingDamage(id, existing_damage);
  };

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
}

TEST_F(EmbedderTest,
       MustNotRunWhenPopulateExistingDamageButNoOtherFBOCallback) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.fbo_callback = nullptr;
  context.GetRendererConfig().open_gl.fbo_with_frame_info_callback = nullptr;
  context.GetRendererConfig().open_gl.populate_existing_damage =
      [](void* context, const intptr_t id,
         FlutterDamage* existing_damage) -> void {
    return reinterpret_cast<EmbedderTestContextGL*>(context)
        ->GLPopulateExistingDamage(id, existing_damage);
  };

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(1, 1));
  auto engine = builder.LaunchEngine();
  ASSERT_FALSE(engine.is_valid());
}

TEST_F(EmbedderTest, PresentInfoContainsValidFBOId) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  const auto root_surface_transformation =
      SkMatrix().preTranslate(0, 1024).preRotate(-90, 0, 0);
  context.SetRootSurfaceTransformation(root_surface_transformation);

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(600, 1024));
  builder.SetDartEntrypoint("push_frames_over_and_over");

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 1024;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  static fml::CountDownLatch frame_latch(10);

  context.AddNativeCallback("SignalNativeTest",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              /* Nothing to do. */
                            }));

  const uint32_t window_fbo_id = context.GetWindowFBOId();
  context.SetGLPresentCallback(
      [window_fbo_id = window_fbo_id](FlutterPresentInfo present_info) {
        ASSERT_EQ(present_info.fbo_id, window_fbo_id);

        frame_latch.CountDown();
      });

  frame_latch.Wait();
}

TEST_F(EmbedderTest,
       PresentInfoReceivesFullDamageWhenExistingDamageIsWholeScreen) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.populate_existing_damage =
      [](void* context, const intptr_t id,
         FlutterDamage* existing_damage) -> void {
    return reinterpret_cast<EmbedderTestContextGL*>(context)
        ->GLPopulateExistingDamage(id, existing_damage);
  };
  // Return existing damage as the entire screen on purpose.
  context.SetGLPopulateExistingDamageCallback(
      [](const intptr_t id, FlutterDamage* existing_damage_ptr) {
        const size_t num_rects = 1;
        // The array must be valid after the callback returns.
        static FlutterRect existing_damage_rects[num_rects] = {
            FlutterRect{0, 0, 800, 600}};
        existing_damage_ptr->num_rects = num_rects;
        existing_damage_ptr->damage = existing_damage_rects;
      });

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetDartEntrypoint("render_gradient_retained");
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  fml::AutoResetWaitableEvent latch;

  // First frame should be entirely rerendered.
  context.SetGLPresentCallback([&](FlutterPresentInfo present_info) {
    const size_t num_rects = 1;
    ASSERT_EQ(present_info.frame_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.frame_damage.damage->left, 0);
    ASSERT_EQ(present_info.frame_damage.damage->top, 0);
    ASSERT_EQ(present_info.frame_damage.damage->right, 800);
    ASSERT_EQ(present_info.frame_damage.damage->bottom, 600);

    ASSERT_EQ(present_info.buffer_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.buffer_damage.damage->left, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->top, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->right, 800);
    ASSERT_EQ(present_info.buffer_damage.damage->bottom, 600);

    latch.Signal();
  });

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();

  // Because it's the same as the first frame, the second frame damage should
  // be empty but, because there was a full existing buffer damage, the buffer
  // damage should be the entire screen.
  context.SetGLPresentCallback([&](FlutterPresentInfo present_info) {
    const size_t num_rects = 1;
    ASSERT_EQ(present_info.frame_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.frame_damage.damage->left, 0);
    ASSERT_EQ(present_info.frame_damage.damage->top, 0);
    ASSERT_EQ(present_info.frame_damage.damage->right, 0);
    ASSERT_EQ(present_info.frame_damage.damage->bottom, 0);

    ASSERT_EQ(present_info.buffer_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.buffer_damage.damage->left, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->top, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->right, 800);
    ASSERT_EQ(present_info.buffer_damage.damage->bottom, 600);

    latch.Signal();
  });

  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();
}

TEST_F(EmbedderTest, PresentInfoReceivesEmptyDamage) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.populate_existing_damage =
      [](void* context, const intptr_t id,
         FlutterDamage* existing_damage) -> void {
    return reinterpret_cast<EmbedderTestContextGL*>(context)
        ->GLPopulateExistingDamage(id, existing_damage);
  };
  // Return no existing damage on purpose.
  context.SetGLPopulateExistingDamageCallback(
      [](const intptr_t id, FlutterDamage* existing_damage_ptr) {
        const size_t num_rects = 1;
        // The array must be valid after the callback returns.
        static FlutterRect existing_damage_rects[num_rects] = {
            FlutterRect{0, 0, 0, 0}};
        existing_damage_ptr->num_rects = num_rects;
        existing_damage_ptr->damage = existing_damage_rects;
      });

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetDartEntrypoint("render_gradient_retained");
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  fml::AutoResetWaitableEvent latch;

  // First frame should be entirely rerendered.
  context.SetGLPresentCallback([&](FlutterPresentInfo present_info) {
    const size_t num_rects = 1;
    ASSERT_EQ(present_info.frame_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.frame_damage.damage->left, 0);
    ASSERT_EQ(present_info.frame_damage.damage->top, 0);
    ASSERT_EQ(present_info.frame_damage.damage->right, 800);
    ASSERT_EQ(present_info.frame_damage.damage->bottom, 600);

    ASSERT_EQ(present_info.buffer_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.buffer_damage.damage->left, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->top, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->right, 800);
    ASSERT_EQ(present_info.buffer_damage.damage->bottom, 600);

    latch.Signal();
  });

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();

  // Because it's the same as the first frame, the second frame should not be
  // rerendered assuming there is no existing damage.
  context.SetGLPresentCallback([&](FlutterPresentInfo present_info) {
    const size_t num_rects = 1;
    ASSERT_EQ(present_info.frame_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.frame_damage.damage->left, 0);
    ASSERT_EQ(present_info.frame_damage.damage->top, 0);
    ASSERT_EQ(present_info.frame_damage.damage->right, 0);
    ASSERT_EQ(present_info.frame_damage.damage->bottom, 0);

    ASSERT_EQ(present_info.buffer_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.buffer_damage.damage->left, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->top, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->right, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->bottom, 0);

    latch.Signal();
  });

  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();
}

TEST_F(EmbedderTest, PresentInfoReceivesPartialDamage) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.populate_existing_damage =
      [](void* context, const intptr_t id,
         FlutterDamage* existing_damage) -> void {
    return reinterpret_cast<EmbedderTestContextGL*>(context)
        ->GLPopulateExistingDamage(id, existing_damage);
  };
  // Return existing damage as only part of the screen on purpose.
  context.SetGLPopulateExistingDamageCallback(
      [&](const intptr_t id, FlutterDamage* existing_damage_ptr) {
        const size_t num_rects = 1;
        // The array must be valid after the callback returns.
        static FlutterRect existing_damage_rects[num_rects] = {
            FlutterRect{200, 150, 400, 300}};
        existing_damage_ptr->num_rects = num_rects;
        existing_damage_ptr->damage = existing_damage_rects;
      });

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetDartEntrypoint("render_gradient_retained");
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  fml::AutoResetWaitableEvent latch;

  // First frame should be entirely rerendered.
  context.SetGLPresentCallback([&](FlutterPresentInfo present_info) {
    const size_t num_rects = 1;
    ASSERT_EQ(present_info.frame_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.frame_damage.damage->left, 0);
    ASSERT_EQ(present_info.frame_damage.damage->top, 0);
    ASSERT_EQ(present_info.frame_damage.damage->right, 800);
    ASSERT_EQ(present_info.frame_damage.damage->bottom, 600);

    ASSERT_EQ(present_info.buffer_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.buffer_damage.damage->left, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->top, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->right, 800);
    ASSERT_EQ(present_info.buffer_damage.damage->bottom, 600);

    latch.Signal();
  });

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();

  // Because it's the same as the first frame, the second frame damage should be
  // empty but, because there was a partial existing damage, the buffer damage
  // should represent that partial damage area.
  context.SetGLPresentCallback([&](FlutterPresentInfo present_info) {
    const size_t num_rects = 1;
    ASSERT_EQ(present_info.frame_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.frame_damage.damage->left, 0);
    ASSERT_EQ(present_info.frame_damage.damage->top, 0);
    ASSERT_EQ(present_info.frame_damage.damage->right, 0);
    ASSERT_EQ(present_info.frame_damage.damage->bottom, 0);

    ASSERT_EQ(present_info.buffer_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.buffer_damage.damage->left, 200);
    ASSERT_EQ(present_info.buffer_damage.damage->top, 150);
    ASSERT_EQ(present_info.buffer_damage.damage->right, 400);
    ASSERT_EQ(present_info.buffer_damage.damage->bottom, 300);

    latch.Signal();
  });

  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();
}

TEST_F(EmbedderTest, PopulateExistingDamageReceivesValidID) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.populate_existing_damage =
      [](void* context, const intptr_t id,
         FlutterDamage* existing_damage) -> void {
    return reinterpret_cast<EmbedderTestContextGL*>(context)
        ->GLPopulateExistingDamage(id, existing_damage);
  };

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetDartEntrypoint("render_gradient_retained");
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  const uint32_t window_fbo_id = context.GetWindowFBOId();
  context.SetGLPopulateExistingDamageCallback(
      [window_fbo_id = window_fbo_id](intptr_t id,
                                      FlutterDamage* existing_damage) {
        ASSERT_EQ(id, window_fbo_id);
        existing_damage->num_rects = 0;
        existing_damage->damage = nullptr;
      });

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
}

TEST_F(EmbedderTest, PopulateExistingDamageReceivesInvalidID) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.populate_existing_damage =
      [](void* context, const intptr_t id,
         FlutterDamage* existing_damage) -> void {
    return reinterpret_cast<EmbedderTestContextGL*>(context)
        ->GLPopulateExistingDamage(id, existing_damage);
  };
  // Return a bad FBO ID on purpose.
  context.GetRendererConfig().open_gl.fbo_with_frame_info_callback =
      [](void* context, const FlutterFrameInfo* frame_info) -> uint32_t {
    return 123;
  };

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetDartEntrypoint("render_gradient_retained");
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  context.AddNativeCallback("SignalNativeTest",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              /* Nothing to do. */
                            }));

  const uint32_t window_fbo_id = context.GetWindowFBOId();
  context.SetGLPopulateExistingDamageCallback(
      [window_fbo_id = window_fbo_id](intptr_t id,
                                      FlutterDamage* existing_damage) {
        ASSERT_NE(id, window_fbo_id);
        existing_damage->num_rects = 0;
        existing_damage->damage = nullptr;
      });

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
}

TEST_F(EmbedderTest, SetSingleDisplayConfigurationWithDisplayId) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("empty_scene");
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());

  FlutterEngineDisplay display;
  display.struct_size = sizeof(FlutterEngineDisplay);
  display.display_id = 1;
  display.refresh_rate = 20;

  std::vector<FlutterEngineDisplay> displays = {display};

  const FlutterEngineResult result = FlutterEngineNotifyDisplayUpdate(
      engine.get(), kFlutterEngineDisplaysUpdateTypeStartup, displays.data(),
      displays.size());
  ASSERT_EQ(result, kSuccess);

  flutter::Shell& shell = ToEmbedderEngine(engine.get())->GetShell();
  ASSERT_EQ(shell.GetMainDisplayRefreshRate(), display.refresh_rate);

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();
}

TEST_F(EmbedderTest, SetSingleDisplayConfigurationWithoutDisplayId) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("empty_scene");
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());

  FlutterEngineDisplay display;
  display.struct_size = sizeof(FlutterEngineDisplay);
  display.single_display = true;
  display.refresh_rate = 20;

  std::vector<FlutterEngineDisplay> displays = {display};

  const FlutterEngineResult result = FlutterEngineNotifyDisplayUpdate(
      engine.get(), kFlutterEngineDisplaysUpdateTypeStartup, displays.data(),
      displays.size());
  ASSERT_EQ(result, kSuccess);

  flutter::Shell& shell = ToEmbedderEngine(engine.get())->GetShell();
  ASSERT_EQ(shell.GetMainDisplayRefreshRate(), display.refresh_rate);

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();
}

TEST_F(EmbedderTest, SetValidMultiDisplayConfiguration) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("empty_scene");
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());

  FlutterEngineDisplay display_1;
  display_1.struct_size = sizeof(FlutterEngineDisplay);
  display_1.display_id = 1;
  display_1.single_display = false;
  display_1.refresh_rate = 20;

  FlutterEngineDisplay display_2;
  display_2.struct_size = sizeof(FlutterEngineDisplay);
  display_2.display_id = 2;
  display_2.single_display = false;
  display_2.refresh_rate = 60;

  std::vector<FlutterEngineDisplay> displays = {display_1, display_2};

  const FlutterEngineResult result = FlutterEngineNotifyDisplayUpdate(
      engine.get(), kFlutterEngineDisplaysUpdateTypeStartup, displays.data(),
      displays.size());
  ASSERT_EQ(result, kSuccess);

  flutter::Shell& shell = ToEmbedderEngine(engine.get())->GetShell();
  ASSERT_EQ(shell.GetMainDisplayRefreshRate(), display_1.refresh_rate);

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();
}

TEST_F(EmbedderTest, MultipleDisplaysWithSingleDisplayTrueIsInvalid) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("empty_scene");
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());

  FlutterEngineDisplay display_1;
  display_1.struct_size = sizeof(FlutterEngineDisplay);
  display_1.display_id = 1;
  display_1.single_display = true;
  display_1.refresh_rate = 20;

  FlutterEngineDisplay display_2;
  display_2.struct_size = sizeof(FlutterEngineDisplay);
  display_2.display_id = 2;
  display_2.single_display = true;
  display_2.refresh_rate = 60;

  std::vector<FlutterEngineDisplay> displays = {display_1, display_2};

  const FlutterEngineResult result = FlutterEngineNotifyDisplayUpdate(
      engine.get(), kFlutterEngineDisplaysUpdateTypeStartup, displays.data(),
      displays.size());
  ASSERT_NE(result, kSuccess);

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();
}

TEST_F(EmbedderTest, MultipleDisplaysWithSameDisplayIdIsInvalid) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("empty_scene");
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  ASSERT_TRUE(engine.is_valid());

  FlutterEngineDisplay display_1;
  display_1.struct_size = sizeof(FlutterEngineDisplay);
  display_1.display_id = 1;
  display_1.single_display = false;
  display_1.refresh_rate = 20;

  FlutterEngineDisplay display_2;
  display_2.struct_size = sizeof(FlutterEngineDisplay);
  display_2.display_id = 1;
  display_2.single_display = false;
  display_2.refresh_rate = 60;

  std::vector<FlutterEngineDisplay> displays = {display_1, display_2};

  const FlutterEngineResult result = FlutterEngineNotifyDisplayUpdate(
      engine.get(), kFlutterEngineDisplaysUpdateTypeStartup, displays.data(),
      displays.size());
  ASSERT_NE(result, kSuccess);

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();
}

TEST_F(EmbedderTest, CompositorRenderTargetsNotRecycledWhenAvoidsCacheSet) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(300, 200));
  builder.SetCompositor(/*avoid_backing_store_cache=*/true);
  builder.SetDartEntrypoint("render_targets_are_recycled");
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);

  const unsigned num_frames = 8;
  const unsigned num_engine_layers = 10;
  const unsigned num_backing_stores = num_frames * num_engine_layers;
  fml::CountDownLatch latch(1 + num_frames);  // 1 for native test signal.

  context.AddNativeCallback("SignalNativeTest",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              latch.CountDown();
                            }));

  context.GetCompositor().SetPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 20u);
        latch.CountDown();
      },
      /*one_shot=*/false);

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 300;
  event.height = 200;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  latch.Wait();

  ASSERT_EQ(context.GetCompositor().GetBackingStoresCreatedCount(),
            num_backing_stores);
  // Killing the engine should collect all the frames.
  engine.reset();
  ASSERT_EQ(context.GetCompositor().GetPendingBackingStoresCount(), 0u);
}

TEST_F(EmbedderTest, SnapshotRenderTargetScalesDownToDriverMax) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();

  auto max_size = context.GetCompositor().GetGrContext()->maxRenderTargetSize();

  context.AddIsolateCreateCallback([&]() {
    Dart_Handle snapshot_large_scene = Dart_GetField(
        Dart_RootLibrary(), tonic::ToDart("snapshot_large_scene"));
    tonic::DartInvoke(snapshot_large_scene, {tonic::ToDart<int64_t>(max_size)});
  });

  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback(
      "SnapshotsCallback", CREATE_NATIVE_ENTRY(([&](Dart_NativeArguments args) {
        auto get_arg = [&args](int index) {
          Dart_Handle dart_image = Dart_GetNativeArgument(args, index);
          Dart_Handle internal_image =
              Dart_GetField(dart_image, tonic::ToDart("_image"));
          return tonic::DartConverter<flutter::CanvasImage*>::FromDart(
              internal_image);
        };

        CanvasImage* big_image = get_arg(0);
        ASSERT_EQ(big_image->width(), max_size);
        ASSERT_EQ(big_image->height(), max_size / 2);

        CanvasImage* small_image = get_arg(1);
        ASSERT_TRUE(ImageMatchesFixture("snapshot_large_scene.png",
                                        small_image->image()->skia_image()));

        latch.Signal();
      })));

  UniqueEngine engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
}

TEST_F(EmbedderTest, ObjectsPostedViaPortsServicedOnSecondaryTaskHeap) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 1024));
  builder.SetDartEntrypoint("objects_can_be_posted");

  // Synchronously acquire the send port from the Dart end. We will be using
  // this to send message. The Dart end will just echo those messages back to us
  // for inspection.
  FlutterEngineDartPort port = 0;
  fml::AutoResetWaitableEvent event;
  context.AddNativeCallback("SignalNativeCount",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              port = tonic::DartConverter<int64_t>::FromDart(
                                  Dart_GetNativeArgument(args, 0));
                              event.Signal();
                            }));
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
  event.Wait();
  ASSERT_NE(port, 0);

  using Trampoline = std::function<void(Dart_Handle message)>;
  Trampoline trampoline;

  context.AddNativeCallback("SendObjectToNativeCode",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              FML_CHECK(trampoline);
                              auto trampoline_copy = trampoline;
                              trampoline = nullptr;
                              trampoline_copy(Dart_GetNativeArgument(args, 0));
                            }));

  // Send a boolean value and assert that it's received by the right heap.
  {
    FlutterEngineDartObject object = {};
    object.type = kFlutterEngineDartObjectTypeBool;
    object.bool_value = true;
    trampoline = [&](Dart_Handle handle) {
      ASSERT_TRUE(tonic::DartConverter<bool>::FromDart(handle));
      auto task_grade = fml::MessageLoopTaskQueues::GetCurrentTaskSourceGrade();
      EXPECT_EQ(task_grade, fml::TaskSourceGrade::kDartEventLoop);
      event.Signal();
    };
    ASSERT_EQ(FlutterEnginePostDartObject(engine.get(), port, &object),
              kSuccess);
    event.Wait();
  }
}

TEST_F(EmbedderTest, CreateInvalidBackingstoreOpenGLTexture) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLTexture);
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
        backing_store_out->type = kFlutterBackingStoreTypeOpenGL;
        // Deliberately set this to be invalid
        backing_store_out->user_data = nullptr;
        backing_store_out->open_gl.type = kFlutterOpenGLTargetTypeTexture;
        backing_store_out->open_gl.texture.target = 0;
        backing_store_out->open_gl.texture.name = 0;
        backing_store_out->open_gl.texture.format = 0;
        static TestCollectOnce collect_once_user_data;
        collect_once_user_data = {};
        backing_store_out->open_gl.texture.user_data = &collect_once_user_data;
        backing_store_out->open_gl.texture.destruction_callback =
            [](void* user_data) {
              reinterpret_cast<TestCollectOnce*>(user_data)->Collect();
            };
        return true;
      };

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());
  latch.Wait();
}

TEST_F(EmbedderTest, CreateInvalidBackingstoreOpenGLFramebuffer) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);
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
        backing_store_out->type = kFlutterBackingStoreTypeOpenGL;
        // Deliberately set this to be invalid
        backing_store_out->user_data = nullptr;
        backing_store_out->open_gl.type = kFlutterOpenGLTargetTypeFramebuffer;
        backing_store_out->open_gl.framebuffer.target = 0;
        backing_store_out->open_gl.framebuffer.name = 0;
        static TestCollectOnce collect_once_user_data;
        collect_once_user_data = {};
        backing_store_out->open_gl.framebuffer.user_data =
            &collect_once_user_data;
        backing_store_out->open_gl.framebuffer.destruction_callback =
            [](void* user_data) {
              reinterpret_cast<TestCollectOnce*>(user_data)->Collect();
            };
        return true;
      };

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());
  latch.Wait();
}

TEST_F(EmbedderTest, CreateInvalidBackingstoreOpenGLSurface) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLSurface);
  builder.SetDartEntrypoint("invalid_backingstore");

  fml::AutoResetWaitableEvent latch;

  builder.GetCompositor().create_backing_store_callback =
      [](const FlutterBackingStoreConfig* config,  //
         FlutterBackingStore* backing_store_out,   //
         void* user_data                           //
      ) {
        backing_store_out->type = kFlutterBackingStoreTypeOpenGL;
        backing_store_out->user_data = user_data;
        backing_store_out->open_gl.type = kFlutterOpenGLTargetTypeSurface;
        backing_store_out->open_gl.surface.user_data = user_data;
        // Deliberately set this to an invalid format
        backing_store_out->open_gl.surface.format = 0;
        backing_store_out->open_gl.surface.make_current_callback = [](void*,
                                                                      bool*) {
          ADD_FAILURE() << "make_current_callback method should not be called";
          return true;
        };
        backing_store_out->open_gl.surface.clear_current_callback = [](void*,
                                                                       bool*) {
          ADD_FAILURE() << "clear_current_callback method should not be called";
          return true;
        };
        backing_store_out->open_gl.surface.destruction_callback =
            [](void* user_data) {
              FAIL() << "destruction_callback method should not be called";
            };

        return true;
      };

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.Signal(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());
  latch.Wait();
}

TEST_F(EmbedderTest, ExternalTextureGLRefreshedTooOften) {
  TestGLSurface surface(SkISize::Make(100, 100));
  auto context = surface.GetGrContext();

  typedef void (*glGenTexturesProc)(uint32_t n, uint32_t* textures);
  typedef void (*glFinishProc)();

  glGenTexturesProc glGenTextures;
  glFinishProc glFinish;

  glGenTextures = reinterpret_cast<glGenTexturesProc>(
      surface.GetProcAddress("glGenTextures"));
  glFinish = reinterpret_cast<glFinishProc>(surface.GetProcAddress("glFinish"));

  uint32_t name;
  glGenTextures(1, &name);

  bool resolve_called = false;

  EmbedderExternalTextureGL::ExternalTextureCallback callback(
      [&](int64_t, size_t, size_t) {
        resolve_called = true;
        auto res = std::make_unique<FlutterOpenGLTexture>();
        res->target = GL_TEXTURE_2D;
        res->name = name;
        res->format = GL_RGBA8;
        res->user_data = nullptr;
        res->destruction_callback = [](void*) {};
        res->width = res->height = 100;
        return res;
      });
  EmbedderExternalTextureGL texture(1, callback);

  auto skia_surface = surface.GetOnscreenSurface();
  DlSkCanvasAdapter canvas(skia_surface->getCanvas());

  Texture* texture_ = &texture;
  Texture::PaintContext ctx{
      .canvas = &canvas,
      .gr_context = context.get(),
  };
  texture_->Paint(ctx, SkRect::MakeXYWH(0, 0, 100, 100), false,
                  DlImageSampling::kLinear);

  EXPECT_TRUE(resolve_called);
  resolve_called = false;

  texture_->Paint(ctx, SkRect::MakeXYWH(0, 0, 100, 100), false,
                  DlImageSampling::kLinear);

  EXPECT_FALSE(resolve_called);

  texture_->MarkNewFrameAvailable();
  texture_->Paint(ctx, SkRect::MakeXYWH(0, 0, 100, 100), false,
                  DlImageSampling::kLinear);

  EXPECT_TRUE(resolve_called);

  glFinish();
}

TEST_F(
    EmbedderTest,
    PresentInfoReceivesFullScreenDamageWhenPopulateExistingDamageIsNotProvided) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.populate_existing_damage = nullptr;

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetDartEntrypoint("render_gradient_retained");
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  fml::AutoResetWaitableEvent latch;

  // First frame should be entirely rerendered.
  context.SetGLPresentCallback([&](FlutterPresentInfo present_info) {
    const size_t num_rects = 1;
    ASSERT_EQ(present_info.frame_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.frame_damage.damage->left, 0);
    ASSERT_EQ(present_info.frame_damage.damage->top, 0);
    ASSERT_EQ(present_info.frame_damage.damage->right, 800);
    ASSERT_EQ(present_info.frame_damage.damage->bottom, 600);

    ASSERT_EQ(present_info.buffer_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.buffer_damage.damage->left, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->top, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->right, 800);
    ASSERT_EQ(present_info.buffer_damage.damage->bottom, 600);

    latch.Signal();
  });

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();

  // Since populate_existing_damage is not provided, the partial repaint
  // functionality is actually disabled. So, the next frame should be entirely
  // new frame.
  context.SetGLPresentCallback([&](FlutterPresentInfo present_info) {
    const size_t num_rects = 1;
    ASSERT_EQ(present_info.frame_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.frame_damage.damage->left, 0);
    ASSERT_EQ(present_info.frame_damage.damage->top, 0);
    ASSERT_EQ(present_info.frame_damage.damage->right, 800);
    ASSERT_EQ(present_info.frame_damage.damage->bottom, 600);

    ASSERT_EQ(present_info.buffer_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.buffer_damage.damage->left, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->top, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->right, 800);
    ASSERT_EQ(present_info.buffer_damage.damage->bottom, 600);

    latch.Signal();
  });

  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();
}

TEST_F(EmbedderTest,
       PresentInfoReceivesJoinedDamageWhenExistingDamageContainsMultipleRects) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  context.GetRendererConfig().open_gl.populate_existing_damage =
      [](void* context, const intptr_t id,
         FlutterDamage* existing_damage) -> void {
    return reinterpret_cast<EmbedderTestContextGL*>(context)
        ->GLPopulateExistingDamage(id, existing_damage);
  };
  // Return existing damage as the entire screen on purpose.
  context.SetGLPopulateExistingDamageCallback(
      [](const intptr_t id, FlutterDamage* existing_damage_ptr) {
        const size_t num_rects = 2;
        // The array must be valid after the callback returns.
        static FlutterRect existing_damage_rects[num_rects] = {
            FlutterRect{100, 150, 200, 250},
            FlutterRect{200, 250, 300, 350},
        };
        existing_damage_ptr->num_rects = num_rects;
        existing_damage_ptr->damage = existing_damage_rects;
      });

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetDartEntrypoint("render_gradient_retained");
  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  fml::AutoResetWaitableEvent latch;

  // First frame should be entirely rerendered.
  context.SetGLPresentCallback([&](FlutterPresentInfo present_info) {
    const size_t num_rects = 1;
    ASSERT_EQ(present_info.frame_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.frame_damage.damage->left, 0);
    ASSERT_EQ(present_info.frame_damage.damage->top, 0);
    ASSERT_EQ(present_info.frame_damage.damage->right, 800);
    ASSERT_EQ(present_info.frame_damage.damage->bottom, 600);

    ASSERT_EQ(present_info.buffer_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.buffer_damage.damage->left, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->top, 0);
    ASSERT_EQ(present_info.buffer_damage.damage->right, 800);
    ASSERT_EQ(present_info.buffer_damage.damage->bottom, 600);

    latch.Signal();
  });

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();

  // Because it's the same as the first frame, the second frame damage should
  // be empty but, because there was a full existing buffer damage, the buffer
  // damage should be the entire screen.
  context.SetGLPresentCallback([&](FlutterPresentInfo present_info) {
    const size_t num_rects = 1;
    ASSERT_EQ(present_info.frame_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.frame_damage.damage->left, 0);
    ASSERT_EQ(present_info.frame_damage.damage->top, 0);
    ASSERT_EQ(present_info.frame_damage.damage->right, 0);
    ASSERT_EQ(present_info.frame_damage.damage->bottom, 0);

    ASSERT_EQ(present_info.buffer_damage.num_rects, num_rects);
    ASSERT_EQ(present_info.buffer_damage.damage->left, 100);
    ASSERT_EQ(present_info.buffer_damage.damage->top, 150);
    ASSERT_EQ(present_info.buffer_damage.damage->right, 300);
    ASSERT_EQ(present_info.buffer_damage.damage->bottom, 350);

    latch.Signal();
  });

  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  latch.Wait();
}

TEST_F(EmbedderTest, CanRenderWithImpellerOpenGL) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();
  EmbedderConfigBuilder builder(context);

  bool present_called = false;
  context.SetGLPresentCallback(
      [&present_called](FlutterPresentInfo present_info) {
        present_called = true;
      });

  builder.AddCommandLineArgument("--enable-impeller");
  builder.SetDartEntrypoint("render_impeller_test");
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  auto rendered_scene = context.GetNextSceneImage();

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());

  // Bind to an arbitrary FBO in order to verify that Impeller binds to the
  // provided FBO during rendering.
  typedef void (*glGenFramebuffersProc)(GLsizei n, GLuint* ids);
  typedef void (*glBindFramebufferProc)(GLenum target, GLuint framebuffer);
  auto glGenFramebuffers = reinterpret_cast<glGenFramebuffersProc>(
      context.GLGetProcAddress("glGenFramebuffers"));
  auto glBindFramebuffer = reinterpret_cast<glBindFramebufferProc>(
      context.GLGetProcAddress("glBindFramebuffer"));
  const flutter::Shell& shell = ToEmbedderEngine(engine.get())->GetShell();
  fml::AutoResetWaitableEvent raster_event;
  shell.GetTaskRunners().GetRasterTaskRunner()->PostTask([&] {
    GLuint fbo;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    raster_event.Signal();
  });
  raster_event.Wait();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);

  ASSERT_TRUE(ImageMatchesFixture(
      FixtureNameForBackend(EmbedderTestContextType::kOpenGLContext,
                            "impeller_test.png"),
      rendered_scene));

  // The scene will be rendered by the compositor, and the surface present
  // callback should not be invoked.
  ASSERT_FALSE(present_called);
}

TEST_F(EmbedderTest, ImpellerOpenGLImageSnapshot) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  bool result = false;
  fml::AutoResetWaitableEvent latch;
  context.AddNativeCallback("NotifyBoolValue",
                            CREATE_NATIVE_ENTRY([&](Dart_NativeArguments args) {
                              result = tonic::DartConverter<bool>::FromDart(
                                  Dart_GetNativeArgument(args, 0));
                              latch.Signal();
                            }));

  EmbedderConfigBuilder builder(context);
  builder.AddCommandLineArgument("--enable-impeller");
  builder.SetDartEntrypoint("render_impeller_image_snapshot_test");
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLFramebuffer);

  auto engine = builder.LaunchEngine();
  ASSERT_TRUE(engine.is_valid());
  latch.Wait();

  ASSERT_TRUE(result);
}

TEST_F(EmbedderTest, CompositorMustBeAbleToRenderToOpenGLSurface) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLSurface);

  fml::CountDownLatch latch(3);
  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 3u);

        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.struct_size = sizeof(backing_store);
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeSurface;

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
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeSurface;

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
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();
}

TEST_F(EmbedderTest, CompositorMustBeAbleToRenderKnownSceneToOpenGLSurfaces) {
  auto& context = GetEmbedderContext<EmbedderTestContextGL>();

  EmbedderConfigBuilder builder(context);
  builder.SetSurface(SkISize::Make(800, 600));
  builder.SetCompositor();
  builder.SetDartEntrypoint("can_composite_platform_views_with_known_scene");

  builder.SetRenderTargetType(
      EmbedderTestBackingStoreProducer::RenderTargetType::kOpenGLSurface);

  fml::CountDownLatch latch(5);

  auto scene_image = context.GetNextSceneImage();

  context.GetCompositor().SetNextPresentCallback(
      [&](FlutterViewId view_id, const FlutterLayer** layers,
          size_t layers_count) {
        ASSERT_EQ(layers_count, 5u);

        // Layer Root
        {
          FlutterBackingStore backing_store = *layers[0]->backing_store;
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeSurface;

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
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeSurface;

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
          backing_store.type = kFlutterBackingStoreTypeOpenGL;
          backing_store.did_update = true;
          backing_store.open_gl.type = kFlutterOpenGLTargetTypeSurface;

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
      [&](const FlutterLayer& layer,
          GrDirectContext* context) -> sk_sp<SkImage> {
        auto surface = CreateRenderSurface(layer, context);
        auto canvas = surface->getCanvas();
        FML_CHECK(canvas != nullptr);

        switch (layer.platform_view->identifier) {
          case 1: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorGREEN);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          case 2: {
            SkPaint paint;
            // See dart test for total order.
            paint.setColor(SK_ColorMAGENTA);
            paint.setAlpha(127);
            const auto& rect =
                SkRect::MakeWH(layer.size.width, layer.size.height);
            canvas->drawRect(rect, paint);
            latch.CountDown();
          } break;
          default:
            // Asked to render an unknown platform view.
            FML_CHECK(false)
                << "Test was asked to composite an unknown platform view.";
        }

        return surface->makeImageSnapshot();
      });

  context.AddNativeCallback(
      "SignalNativeTest",
      CREATE_NATIVE_ENTRY(
          [&latch](Dart_NativeArguments args) { latch.CountDown(); }));

  auto engine = builder.LaunchEngine();

  // Send a window metrics events so frames may be scheduled.
  FlutterWindowMetricsEvent event = {};
  event.struct_size = sizeof(event);
  event.width = 800;
  event.height = 600;
  event.pixel_ratio = 1.0;
  ASSERT_EQ(FlutterEngineSendWindowMetricsEvent(engine.get(), &event),
            kSuccess);
  ASSERT_TRUE(engine.is_valid());

  latch.Wait();

  ASSERT_TRUE(ImageMatchesFixture("compositor.png", scene_image));

  // There should no present calls on the root surface.
  ASSERT_EQ(context.GetSurfacePresentCount(), 0u);
}

INSTANTIATE_TEST_SUITE_P(
    EmbedderTestGlVk,
    EmbedderTestMultiBackend,
    ::testing::Values(EmbedderTestContextType::kOpenGLContext,
                      EmbedderTestContextType::kVulkanContext));

}  // namespace flutter::testing

// NOLINTEND(clang-analyzer-core.StackAddressEscape)
