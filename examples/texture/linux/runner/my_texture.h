// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_MY_TEXTURE_H_
#define FLUTTER_MY_TEXTURE_H_

#include <flutter_linux/flutter_linux.h>

G_DECLARE_FINAL_TYPE(MyTexture, my_texture, MY, TEXTURE, FlPixelBufferTexture)

/**
 * my_texture_new:
 * @self: a #MyTexture.
 * @width: width of texture in pixels.
 * @height: height of texture in pixels.
 * @r: red value for texture color.
 * @g: green value for texture color.
 * @b: blue value for texture color.
 *
 * Creates a new texture containing a single color for Flutter to render with.
 *
 * Returns: a new #MyTexture.
 */
MyTexture* my_texture_new(uint32_t width, uint32_t height, uint8_t r, uint8_t g,
                          uint8_t b);

/**
 * my_texture_new:
 * @self: a #MyTexture.
 * @r: red value for texture color.
 * @g: green value for texture color.
 * @b: blue value for texture color.
 *
 * Sets the color the texture contains.
 */
void my_texture_set_color(MyTexture* self, uint8_t r, uint8_t g, uint8_t b);

#endif  // FLUTTER_MY_TEXTURE_H_
