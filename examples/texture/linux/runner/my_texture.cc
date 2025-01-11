// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "my_texture.h"

// An object that generates a texture for Flutter.
struct _MyTexture {
  FlPixelBufferTexture parent_instance;

  // Dimensions of texture.
  uint32_t width;
  uint32_t height;

  // Buffer used to store texture.
  uint8_t* buffer;
};

G_DEFINE_TYPE(MyTexture, my_texture, fl_pixel_buffer_texture_get_type())

// Implements GObject::dispose.
static void my_texture_dispose(GObject* object) {
  MyTexture* self = MY_TEXTURE(object);

  free(self->buffer);

  G_OBJECT_CLASS(my_texture_parent_class)->dispose(object);
}

// Implements FlPixelBufferTexture::copy_pixels.
static gboolean my_texture_copy_pixels(FlPixelBufferTexture* texture,
                                       const uint8_t** out_buffer,
                                       uint32_t* width, uint32_t* height,
                                       GError** error) {
  MyTexture* self = MY_TEXTURE(texture);
  *out_buffer = self->buffer;
  *width = self->width;
  *height = self->height;
  return TRUE;
}

static void my_texture_class_init(MyTextureClass* klass) {
  G_OBJECT_CLASS(klass)->dispose = my_texture_dispose;
  FL_PIXEL_BUFFER_TEXTURE_CLASS(klass)->copy_pixels = my_texture_copy_pixels;
}

static void my_texture_init(MyTexture* self) {}

MyTexture* my_texture_new(uint32_t width, uint32_t height, uint8_t r, uint8_t g,
                          uint8_t b) {
  MyTexture* self = MY_TEXTURE(g_object_new(my_texture_get_type(), nullptr));
  self->width = width;
  self->height = height;
  self->buffer = static_cast<uint8_t*>(malloc(self->width * self->height * 4));
  my_texture_set_color(self, r, g, b);
  return self;
}

// Draws the texture with the requested color.
void my_texture_set_color(MyTexture* self, uint8_t r, uint8_t g, uint8_t b) {
  g_return_if_fail(MY_IS_TEXTURE(self));

  for (size_t y = 0; y < self->height; y++) {
    for (size_t x = 0; x < self->width; x++) {
      uint8_t* pixel = self->buffer + (y * self->width * 4) + (x * 4);
      pixel[0] = r;
      pixel[1] = g;
      pixel[2] = b;
      pixel[3] = 255;
    }
  }
}
