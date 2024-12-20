// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/testing/mock_renderer.h"

struct _FlMockRenderer {
  FlRenderer parent_instance;
  FlMockRendererGetRefreshRate get_refresh_rate;
};

struct _FlMockRenderable {
  GObject parent_instance;
  size_t redraw_count;
};

G_DEFINE_TYPE(FlMockRenderer, fl_mock_renderer, fl_renderer_get_type())

static void mock_renderable_iface_init(FlRenderableInterface* iface);

G_DEFINE_TYPE_WITH_CODE(FlMockRenderable,
                        fl_mock_renderable,
                        g_object_get_type(),
                        G_IMPLEMENT_INTERFACE(fl_renderable_get_type(),
                                              mock_renderable_iface_init))

// Implements FlRenderer::make_current.
static void fl_mock_renderer_make_current(FlRenderer* renderer) {}

// Implements FlRenderer::make_resource_current.
static void fl_mock_renderer_make_resource_current(FlRenderer* renderer) {}

// Implements FlRenderer::clear_current.
static void fl_mock_renderer_clear_current(FlRenderer* renderer) {}

// Implements FlRenderer::get_refresh_rate.
static gdouble fl_mock_renderer_default_get_refresh_rate(FlRenderer* renderer) {
  FlMockRenderer* self = FL_MOCK_RENDERER(renderer);
  if (self->get_refresh_rate != nullptr) {
    return self->get_refresh_rate(renderer);
  }
  return -1.0;
}

static void fl_mock_renderer_class_init(FlMockRendererClass* klass) {
  FL_RENDERER_CLASS(klass)->make_current = fl_mock_renderer_make_current;
  FL_RENDERER_CLASS(klass)->make_resource_current =
      fl_mock_renderer_make_resource_current;
  FL_RENDERER_CLASS(klass)->clear_current = fl_mock_renderer_clear_current;
  FL_RENDERER_CLASS(klass)->get_refresh_rate =
      fl_mock_renderer_default_get_refresh_rate;
}

static void fl_mock_renderer_init(FlMockRenderer* self) {}

static void mock_renderable_redraw(FlRenderable* renderable) {
  FlMockRenderable* self = FL_MOCK_RENDERABLE(renderable);
  self->redraw_count++;
}

static void mock_renderable_make_current(FlRenderable* renderable) {}

static void mock_renderable_iface_init(FlRenderableInterface* iface) {
  iface->redraw = mock_renderable_redraw;
  iface->make_current = mock_renderable_make_current;
}

static void fl_mock_renderable_class_init(FlMockRenderableClass* klass) {}

static void fl_mock_renderable_init(FlMockRenderable* self) {}

// Creates a stub renderer
FlMockRenderer* fl_mock_renderer_new(
    FlMockRendererGetRefreshRate get_refresh_rate) {
  FlMockRenderer* self =
      FL_MOCK_RENDERER(g_object_new(fl_mock_renderer_get_type(), nullptr));
  self->get_refresh_rate = get_refresh_rate;
  return self;
}

// Creates a sub renderable.
FlMockRenderable* fl_mock_renderable_new() {
  return FL_MOCK_RENDERABLE(
      g_object_new(fl_mock_renderable_get_type(), nullptr));
}

size_t fl_mock_renderable_get_redraw_count(FlMockRenderable* self) {
  g_return_val_if_fail(FL_IS_MOCK_RENDERABLE(self), FALSE);
  return self->redraw_count;
}
