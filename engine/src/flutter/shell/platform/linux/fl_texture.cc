// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/linux/public/flutter_linux/fl_texture.h"
#include "flutter/shell/platform/linux/fl_texture_private.h"

#include <gmodule.h>
#include <cstdio>

G_DEFINE_INTERFACE(FlTexture, fl_texture, G_TYPE_OBJECT)

static void fl_texture_default_init(FlTextureInterface* self) {}

void fl_texture_set_id(FlTexture* self, int64_t id) {
  g_return_if_fail(FL_IS_TEXTURE(self));
  FL_TEXTURE_GET_IFACE(self)->set_id(self, id);
}

G_MODULE_EXPORT int64_t fl_texture_get_id(FlTexture* self) {
  g_return_val_if_fail(FL_IS_TEXTURE(self), -1);
  return FL_TEXTURE_GET_IFACE(self)->get_id(self);
}
