// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef TEXTURE_GLSL_
#define TEXTURE_GLSL_

#include <impeller/branching.glsl>
#include <impeller/conversions.glsl>
#include <impeller/tile_mode.glsl>
#include <impeller/types.glsl>

/// Sample from a texture.
///
/// If `y_coord_scale` < 0.0, the Y coordinate is flipped. This is useful
/// for Impeller graphics backends that use a flipped framebuffer coordinate
/// space.
///
/// A negative mip bias is applied to improve the sharpness of scaled down
/// images when mip sampling is enabled. See `kDefaultMipBias` for more detail.
vec4 IPSample(sampler2D texture_sampler, vec2 coords, float y_coord_scale) {
  return texture(texture_sampler, IPRemapCoords(coords, y_coord_scale),
                 kDefaultMipBias);
}

/// Sample from a texture.
///
/// If `y_coord_scale` < 0.0, the Y coordinate is flipped. This is useful
/// for Impeller graphics backends that use a flipped framebuffer coordinate
/// space.
/// The range of `coords` will be mapped from [0, 1] to [half_texel, 1 -
/// half_texel]
vec4 IPSampleLinear(sampler2D texture_sampler,
                    vec2 coords,
                    float y_coord_scale,
                    vec2 half_texel) {
  coords.x = mix(half_texel.x, 1 - half_texel.x, coords.x);
  coords.y = mix(half_texel.y, 1 - half_texel.y, coords.y);
  return IPSample(texture_sampler, coords, y_coord_scale);
}

/// Remap a float using a tiling mode.
///
/// When `tile_mode` is `kTileModeDecal`, no tiling is applied and `t` is
/// returned. In all other cases, a value between 0 and 1 is returned by tiling
/// `t`.
/// When `t` is between [0 to 1), the original unchanged `t` is always returned.
float IPFloatTile(float t, float tile_mode) {
  if (tile_mode == kTileModeClamp) {
    t = clamp(t, 0.0, 1.0);
  } else if (tile_mode == kTileModeRepeat) {
    t = fract(t);
  } else if (tile_mode == kTileModeMirror) {
    float t1 = t - 1;
    float t2 = t1 - 2 * floor(t1 * 0.5) - 1;
    t = abs(t2);
  }
  return t;
}

/// Remap a vec2 using a tiling mode.
///
/// Runs each component of the vec2 through `IPFloatTile`.
vec2 IPVec2Tile(vec2 coords, float x_tile_mode, float y_tile_mode) {
  return vec2(IPFloatTile(coords.x, x_tile_mode),
              IPFloatTile(coords.y, y_tile_mode));
}

/// Sample a texture, emulating a specific tile mode.
///
/// This is useful for Impeller graphics backend that don't have native support
/// for Decal.
vec4 IPSampleWithTileMode(sampler2D tex,
                          vec2 coords,
                          float x_tile_mode,
                          float y_tile_mode) {
  if (x_tile_mode == kTileModeDecal && (coords.x < 0 || coords.x >= 1) ||
      y_tile_mode == kTileModeDecal && (coords.y < 0 || coords.y >= 1)) {
    return vec4(0);
  }

  return texture(tex, coords, kDefaultMipBias);
}

/// Sample a texture, emulating a specific tile mode.
///
/// This is useful for Impeller graphics backend that don't have native support
/// for Decal.
f16vec4 IPHalfSampleWithTileMode(f16sampler2D tex,
                                 vec2 coords,
                                 float x_tile_mode,
                                 float y_tile_mode) {
  if (x_tile_mode == kTileModeDecal && (coords.x < 0.0 || coords.x >= 1.0) ||
      y_tile_mode == kTileModeDecal && (coords.y < 0.0 || coords.y >= 1.0)) {
    return f16vec4(0.0hf);
  }

  return texture(tex, coords, float16_t(kDefaultMipBias));
}

/// Sample a texture, emulating a specific tile mode.
///
/// This is useful for Impeller graphics backend that don't have native support
/// for Decal.
/// The range of `coords` will be mapped from [0, 1] to [half_texel, 1 -
/// half_texel]
vec4 IPSampleLinearWithTileMode(sampler2D tex,
                                vec2 coords,
                                float y_coord_scale,
                                vec2 half_texel,
                                float x_tile_mode,
                                float y_tile_mode,
                                vec4 decal_border_color) {
  if (x_tile_mode == kTileModeDecal && (coords.x < 0 || coords.x >= 1) ||
      y_tile_mode == kTileModeDecal && (coords.y < 0 || coords.y >= 1)) {
    return decal_border_color;
  }

  return IPSampleLinear(tex, IPVec2Tile(coords, x_tile_mode, y_tile_mode),
                        y_coord_scale, half_texel);
}

/// Sample a texture with decal tile mode.
vec4 IPSampleDecal(sampler2D texture_sampler, vec2 coords) {
  if (any(lessThan(coords, vec2(0))) ||
      any(greaterThanEqual(coords, vec2(1)))) {
    return vec4(0);
  }
  return texture(texture_sampler, coords, kDefaultMipBias);
}

/// Sample a texture with decal tile mode.
f16vec4 IPHalfSampleDecal(f16sampler2D texture_sampler, vec2 coords) {
  if (any(lessThan(coords, vec2(0))) ||
      any(greaterThanEqual(coords, vec2(1)))) {
    return f16vec4(0.0);
  }
  return texture(texture_sampler, coords, float16_t(kDefaultMipBias));
}

/// Sample a texture, emulating a specific tile mode.
///
/// This is useful for Impeller graphics backend that don't have native support
/// for Decal.
/// The range of `coords` will be mapped from [0, 1] to [half_texel, 1 -
/// half_texel]
vec4 IPSampleLinearWithTileMode(sampler2D tex,
                                vec2 coords,
                                float y_coord_scale,
                                vec2 half_texel,
                                float tile_mode,
                                vec4 decal_border_color) {
  return IPSampleLinearWithTileMode(tex, coords, y_coord_scale, half_texel,
                                    tile_mode, tile_mode, decal_border_color);
}

#endif
