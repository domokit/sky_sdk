// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/fl_renderable.h"

G_DEFINE_INTERFACE(FlRenderable, fl_renderable, G_TYPE_OBJECT)

static void fl_renderable_default_init(FlRenderableInterface* iface) {}

void fl_renderable_redraw(FlRenderable* self) {
  g_return_if_fail(FL_IS_RENDERABLE(self));

  FL_RENDERABLE_GET_IFACE(self)->redraw(self);
}

void fl_renderable_make_current(FlRenderable* self) {
  g_return_if_fail(FL_IS_RENDERABLE(self));

  FL_RENDERABLE_GET_IFACE(self)->make_current(self);
}
