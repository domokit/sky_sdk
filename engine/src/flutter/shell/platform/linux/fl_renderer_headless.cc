// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "fl_renderer_headless.h"

struct _FlRendererHeadless {
  FlRenderer parent_instance;
};

G_DEFINE_TYPE(FlRendererHeadless, fl_renderer_headless, fl_renderer_get_type())

// Implements FlRenderer::make_current.
static void fl_renderer_headless_make_current(FlRenderer* renderer) {}

// Implements FlRenderer::make_resource_current.
static void fl_renderer_headless_make_resource_current(FlRenderer* renderer) {}

// Implements FlRenderer::clear_current.
static void fl_renderer_headless_clear_current(FlRenderer* renderer) {}

// Implements FlRenderer::get_refresh_rate.
static gdouble fl_renderer_headless_get_refresh_rate(FlRenderer* renderer) {
  return -1.0;
}

static void fl_renderer_headless_class_init(FlRendererHeadlessClass* klass) {
  FL_RENDERER_CLASS(klass)->make_current = fl_renderer_headless_make_current;
  FL_RENDERER_CLASS(klass)->make_resource_current =
      fl_renderer_headless_make_resource_current;
  FL_RENDERER_CLASS(klass)->clear_current = fl_renderer_headless_clear_current;
  FL_RENDERER_CLASS(klass)->get_refresh_rate =
      fl_renderer_headless_get_refresh_rate;
}

static void fl_renderer_headless_init(FlRendererHeadless* self) {}

FlRendererHeadless* fl_renderer_headless_new() {
  return FL_RENDERER_HEADLESS(
      g_object_new(fl_renderer_headless_get_type(), nullptr));
}
