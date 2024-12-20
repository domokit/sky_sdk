// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"

#include "flutter/common/constants.h"
#include "flutter/fml/logging.h"
#include "flutter/shell/platform/linux/fl_framebuffer.h"
#include "flutter/shell/platform/linux/testing/mock_epoxy.h"
#include "flutter/shell/platform/linux/testing/mock_renderer.h"

#include <epoxy/egl.h>

TEST(FlRendererTest, BackgroundColor) {
  ::testing::NiceMock<flutter::testing::MockEpoxy> epoxy;

  ON_CALL(epoxy, epoxy_is_desktop_gl).WillByDefault(::testing::Return(true));
  EXPECT_CALL(epoxy, epoxy_gl_version).WillRepeatedly(::testing::Return(30));
  ON_CALL(epoxy, glGetString(GL_VENDOR))
      .WillByDefault(
          ::testing::Return(reinterpret_cast<const GLubyte*>("Intel")));
  EXPECT_CALL(epoxy, glClearColor(0.2, 0.3, 0.4, 0.5));

  g_autoptr(FlMockRenderable) renderable = fl_mock_renderable_new();
  g_autoptr(FlMockRenderer) renderer = fl_mock_renderer_new();
  fl_renderer_setup(FL_RENDERER(renderer));
  fl_renderer_add_renderable(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId,
                             FL_RENDERABLE(renderable));
  fl_renderer_wait_for_frame(FL_RENDERER(renderer), 1024, 1024);
  FlutterBackingStoreConfig config = {
      .struct_size = sizeof(FlutterBackingStoreConfig),
      .size = {.width = 1024, .height = 1024}};
  FlutterBackingStore backing_store;
  fl_renderer_create_backing_store(FL_RENDERER(renderer), &config,
                                   &backing_store);
  const FlutterLayer layer0 = {.struct_size = sizeof(FlutterLayer),
                               .type = kFlutterLayerContentTypeBackingStore,
                               .backing_store = &backing_store,
                               .size = {.width = 1024, .height = 1024}};
  const FlutterLayer* layers[] = {&layer0};
  fl_renderer_present_layers(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId, layers, 1);
  GdkRGBA background_color = {
      .red = 0.2, .green = 0.3, .blue = 0.4, .alpha = 0.5};
  fl_renderer_render(FL_RENDERER(renderer), flutter::kFlutterImplicitViewId,
                     1024, 1024, &background_color);
}

TEST(FlRendererTest, RestoresGLState) {
  ::testing::NiceMock<flutter::testing::MockEpoxy> epoxy;

  constexpr int kWidth = 100;
  constexpr int kHeight = 100;

  g_autoptr(FlMockRenderable) renderable = fl_mock_renderable_new();
  g_autoptr(FlMockRenderer) renderer = fl_mock_renderer_new();
  g_autoptr(FlFramebuffer) framebuffer =
      fl_framebuffer_new(GL_RGB, kWidth, kHeight);

  fl_renderer_add_renderable(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId,
                             FL_RENDERABLE(renderable));
  fl_renderer_wait_for_frame(FL_RENDERER(renderer), kWidth, kHeight);

  FlutterBackingStore backing_store;
  backing_store.type = kFlutterBackingStoreTypeOpenGL;
  backing_store.open_gl.framebuffer.user_data = framebuffer;

  FlutterLayer layer;
  layer.type = kFlutterLayerContentTypeBackingStore;
  layer.backing_store = &backing_store;
  layer.offset = {0, 0};
  layer.size = {kWidth, kHeight};

  std::array<const FlutterLayer*, 1> layers = {&layer};

  constexpr GLuint kFakeTextureName = 123;
  glBindTexture(GL_TEXTURE_2D, kFakeTextureName);

  fl_renderer_present_layers(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId, layers.data(),
                             layers.size());
  GdkRGBA background_color = {
      .red = 0.0, .green = 0.0, .blue = 0.0, .alpha = 1.0};
  fl_renderer_render(FL_RENDERER(renderer), flutter::kFlutterImplicitViewId,
                     kWidth, kHeight, &background_color);

  GLuint texture_2d_binding;
  glGetIntegerv(GL_TEXTURE_BINDING_2D,
                reinterpret_cast<GLint*>(&texture_2d_binding));
  EXPECT_EQ(texture_2d_binding, kFakeTextureName);
}

static constexpr double kExpectedRefreshRate = 120.0;
static gdouble renderer_get_refresh_rate(FlRenderer* renderer) {
  return kExpectedRefreshRate;
}

TEST(FlRendererTest, RefreshRate) {
  g_autoptr(FlMockRenderer) renderer =
      fl_mock_renderer_new(&renderer_get_refresh_rate);

  gdouble result_refresh_rate =
      fl_renderer_get_refresh_rate(FL_RENDERER(renderer));
  EXPECT_DOUBLE_EQ(result_refresh_rate, kExpectedRefreshRate);
}

TEST(FlRendererTest, BlitFramebuffer) {
  ::testing::NiceMock<flutter::testing::MockEpoxy> epoxy;

  // OpenGL 3.0
  ON_CALL(epoxy, glGetString(GL_VENDOR))
      .WillByDefault(
          ::testing::Return(reinterpret_cast<const GLubyte*>("Intel")));
  ON_CALL(epoxy, epoxy_is_desktop_gl).WillByDefault(::testing::Return(true));
  EXPECT_CALL(epoxy, epoxy_gl_version).WillRepeatedly(::testing::Return(30));

  EXPECT_CALL(epoxy, glBlitFramebuffer);

  g_autoptr(FlMockRenderable) renderable = fl_mock_renderable_new();
  g_autoptr(FlMockRenderer) renderer = fl_mock_renderer_new();
  fl_renderer_setup(FL_RENDERER(renderer));
  fl_renderer_add_renderable(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId,
                             FL_RENDERABLE(renderable));
  fl_renderer_wait_for_frame(FL_RENDERER(renderer), 1024, 1024);
  FlutterBackingStoreConfig config = {
      .struct_size = sizeof(FlutterBackingStoreConfig),
      .size = {.width = 1024, .height = 1024}};
  FlutterBackingStore backing_store;
  fl_renderer_create_backing_store(FL_RENDERER(renderer), &config,
                                   &backing_store);
  const FlutterLayer layer0 = {.struct_size = sizeof(FlutterLayer),
                               .type = kFlutterLayerContentTypeBackingStore,
                               .backing_store = &backing_store,
                               .size = {.width = 1024, .height = 1024}};
  const FlutterLayer* layers[] = {&layer0};
  fl_renderer_present_layers(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId, layers, 1);
  GdkRGBA background_color = {
      .red = 0.0, .green = 0.0, .blue = 0.0, .alpha = 1.0};
  fl_renderer_render(FL_RENDERER(renderer), flutter::kFlutterImplicitViewId,
                     1024, 1024, &background_color);
}

TEST(FlRendererTest, BlitFramebufferExtension) {
  ::testing::NiceMock<flutter::testing::MockEpoxy> epoxy;

  // OpenGL 2.0 with GL_EXT_framebuffer_blit extension
  ON_CALL(epoxy, glGetString(GL_VENDOR))
      .WillByDefault(
          ::testing::Return(reinterpret_cast<const GLubyte*>("Intel")));
  ON_CALL(epoxy, epoxy_is_desktop_gl).WillByDefault(::testing::Return(true));
  EXPECT_CALL(epoxy, epoxy_gl_version).WillRepeatedly(::testing::Return(20));
  EXPECT_CALL(epoxy, epoxy_has_gl_extension(::testing::_))
      .WillRepeatedly(::testing::Return(false));
  EXPECT_CALL(epoxy, epoxy_has_gl_extension(
                         ::testing::StrEq("GL_EXT_framebuffer_blit")))
      .WillRepeatedly(::testing::Return(true));

  EXPECT_CALL(epoxy, glBlitFramebuffer);

  g_autoptr(FlMockRenderable) renderable = fl_mock_renderable_new();
  g_autoptr(FlMockRenderer) renderer = fl_mock_renderer_new();
  fl_renderer_setup(FL_RENDERER(renderer));
  fl_renderer_add_renderable(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId,
                             FL_RENDERABLE(renderable));
  fl_renderer_wait_for_frame(FL_RENDERER(renderer), 1024, 1024);
  FlutterBackingStoreConfig config = {
      .struct_size = sizeof(FlutterBackingStoreConfig),
      .size = {.width = 1024, .height = 1024}};
  FlutterBackingStore backing_store;
  fl_renderer_create_backing_store(FL_RENDERER(renderer), &config,
                                   &backing_store);
  const FlutterLayer layer0 = {.struct_size = sizeof(FlutterLayer),
                               .type = kFlutterLayerContentTypeBackingStore,
                               .backing_store = &backing_store,
                               .size = {.width = 1024, .height = 1024}};
  const FlutterLayer* layers[] = {&layer0};
  fl_renderer_present_layers(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId, layers, 1);
  GdkRGBA background_color = {
      .red = 0.0, .green = 0.0, .blue = 0.0, .alpha = 1.0};
  fl_renderer_render(FL_RENDERER(renderer), flutter::kFlutterImplicitViewId,
                     1024, 1024, &background_color);
}

TEST(FlRendererTest, NoBlitFramebuffer) {
  ::testing::NiceMock<flutter::testing::MockEpoxy> epoxy;

  // OpenGL 2.0
  ON_CALL(epoxy, glGetString(GL_VENDOR))
      .WillByDefault(
          ::testing::Return(reinterpret_cast<const GLubyte*>("Intel")));
  ON_CALL(epoxy, epoxy_is_desktop_gl).WillByDefault(::testing::Return(true));
  EXPECT_CALL(epoxy, epoxy_gl_version).WillRepeatedly(::testing::Return(20));

  g_autoptr(FlMockRenderable) renderable = fl_mock_renderable_new();
  g_autoptr(FlMockRenderer) renderer = fl_mock_renderer_new();
  fl_renderer_setup(FL_RENDERER(renderer));
  fl_renderer_add_renderable(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId,
                             FL_RENDERABLE(renderable));
  fl_renderer_wait_for_frame(FL_RENDERER(renderer), 1024, 1024);
  FlutterBackingStoreConfig config = {
      .struct_size = sizeof(FlutterBackingStoreConfig),
      .size = {.width = 1024, .height = 1024}};
  FlutterBackingStore backing_store;
  fl_renderer_create_backing_store(FL_RENDERER(renderer), &config,
                                   &backing_store);
  const FlutterLayer layer0 = {.struct_size = sizeof(FlutterLayer),
                               .type = kFlutterLayerContentTypeBackingStore,
                               .backing_store = &backing_store,
                               .size = {.width = 1024, .height = 1024}};
  const FlutterLayer* layers[] = {&layer0};
  fl_renderer_present_layers(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId, layers, 1);
  GdkRGBA background_color = {
      .red = 0.0, .green = 0.0, .blue = 0.0, .alpha = 1.0};
  fl_renderer_render(FL_RENDERER(renderer), flutter::kFlutterImplicitViewId,
                     1024, 1024, &background_color);
}

TEST(FlRendererTest, BlitFramebufferNvidia) {
  ::testing::NiceMock<flutter::testing::MockEpoxy> epoxy;

  // OpenGL 3.0, but on NVIDIA driver so temporarily disabled due to
  // https://github.com/flutter/flutter/issues/152099
  ON_CALL(epoxy, glGetString(GL_VENDOR))
      .WillByDefault(
          ::testing::Return(reinterpret_cast<const GLubyte*>("NVIDIA")));
  ON_CALL(epoxy, epoxy_is_desktop_gl).WillByDefault(::testing::Return(true));
  EXPECT_CALL(epoxy, epoxy_gl_version).WillRepeatedly(::testing::Return(30));

  g_autoptr(FlMockRenderable) renderable = fl_mock_renderable_new();
  g_autoptr(FlMockRenderer) renderer = fl_mock_renderer_new();
  fl_renderer_setup(FL_RENDERER(renderer));
  fl_renderer_add_renderable(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId,
                             FL_RENDERABLE(renderable));
  fl_renderer_wait_for_frame(FL_RENDERER(renderer), 1024, 1024);
  FlutterBackingStoreConfig config = {
      .struct_size = sizeof(FlutterBackingStoreConfig),
      .size = {.width = 1024, .height = 1024}};
  FlutterBackingStore backing_store;
  fl_renderer_create_backing_store(FL_RENDERER(renderer), &config,
                                   &backing_store);
  const FlutterLayer layer0 = {.struct_size = sizeof(FlutterLayer),
                               .type = kFlutterLayerContentTypeBackingStore,
                               .backing_store = &backing_store,
                               .size = {.width = 1024, .height = 1024}};
  const FlutterLayer* layers[] = {&layer0};
  fl_renderer_present_layers(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId, layers, 1);
  GdkRGBA background_color = {
      .red = 0.0, .green = 0.0, .blue = 0.0, .alpha = 1.0};
  fl_renderer_render(FL_RENDERER(renderer), flutter::kFlutterImplicitViewId,
                     1024, 1024, &background_color);
}

TEST(FlRendererTest, MultiView) {
  ::testing::NiceMock<flutter::testing::MockEpoxy> epoxy;

  // OpenGL 3.0
  ON_CALL(epoxy, glGetString(GL_VENDOR))
      .WillByDefault(
          ::testing::Return(reinterpret_cast<const GLubyte*>("Intel")));
  ON_CALL(epoxy, epoxy_is_desktop_gl).WillByDefault(::testing::Return(true));
  EXPECT_CALL(epoxy, epoxy_gl_version).WillRepeatedly(::testing::Return(30));

  g_autoptr(FlMockRenderable) renderable = fl_mock_renderable_new();
  g_autoptr(FlMockRenderable) secondary_renderable = fl_mock_renderable_new();

  g_autoptr(FlMockRenderer) renderer = fl_mock_renderer_new();
  fl_renderer_setup(FL_RENDERER(renderer));
  fl_renderer_add_renderable(FL_RENDERER(renderer),
                             flutter::kFlutterImplicitViewId,
                             FL_RENDERABLE(renderable));
  fl_renderer_add_renderable(FL_RENDERER(renderer), 1,
                             FL_RENDERABLE(secondary_renderable));
  fl_renderer_wait_for_frame(FL_RENDERER(renderer), 1024, 1024);

  EXPECT_EQ(fl_mock_renderable_get_redraw_count(renderable),
            static_cast<size_t>(0));
  EXPECT_EQ(fl_mock_renderable_get_redraw_count(secondary_renderable),
            static_cast<size_t>(0));

  FlutterBackingStoreConfig config = {
      .struct_size = sizeof(FlutterBackingStoreConfig),
      .size = {.width = 1024, .height = 1024}};
  FlutterBackingStore backing_store;
  fl_renderer_create_backing_store(FL_RENDERER(renderer), &config,
                                   &backing_store);
  const FlutterLayer layer0 = {.struct_size = sizeof(FlutterLayer),
                               .type = kFlutterLayerContentTypeBackingStore,
                               .backing_store = &backing_store,
                               .size = {.width = 1024, .height = 1024}};
  const FlutterLayer* layers[] = {&layer0};
  fl_renderer_present_layers(FL_RENDERER(renderer), 1, layers, 1);

  EXPECT_EQ(fl_mock_renderable_get_redraw_count(renderable),
            static_cast<size_t>(0));
  EXPECT_EQ(fl_mock_renderable_get_redraw_count(secondary_renderable),
            static_cast<size_t>(1));
}
