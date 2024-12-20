#version 320 es

// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This shader is a fixture for a test that ensures that the order of the
// unifroms is retained. The values of the uniforms in the test are strictly
// increasing. If two uniforms are out of order, the output color is red. If
// the uniforms are in the right order, the output color is green.

precision highp float;

#include <flutter/runtime_effect.glsl>

layout(location = 0) uniform vec4 u_color;
layout(location = 1) uniform float u_alpha;
layout(location = 2) uniform vec4 u_sparkle_color;
layout(location = 3) uniform float u_sparkle_alpha;
layout(location = 4) uniform float u_blur;
layout(location = 5) uniform vec2 u_center;
layout(location = 6) uniform float u_radius_scale;
layout(location = 7) uniform float u_max_radius;
layout(location = 8) uniform vec2 u_resolution_scale;
layout(location = 9) uniform vec2 u_noise_scale;
layout(location = 10) uniform float u_noise_phase;
layout(location = 11) uniform vec2 u_circle1;
layout(location = 12) uniform vec2 u_circle2;
layout(location = 13) uniform vec2 u_circle3;
layout(location = 14) uniform vec2 u_rotation1;
layout(location = 15) uniform vec2 u_rotation2;
layout(location = 16) uniform vec2 u_rotation3;

layout(location = 0) out vec4 fragColor;

const float PI = 3.1415926535897932384626;
const float PI_ROTATE_RIGHT = PI * 0.0078125;
const float PI_ROTATE_LEFT = PI * -0.0078125;
const float ONE_THIRD = 1. / 3.;
const vec2 TURBULENCE_SCALE = vec2(0.8);

float triangle_noise(highp vec2 n) {
  n = fract(n * vec2(5.3987, 5.4421));
  n += dot(n.yx, n.xy + vec2(21.5351, 14.3137));
  float xy = n.x * n.y;
  return fract(xy * 95.4307) + fract(xy * 75.04961) - 1.0;
}

float threshold(float v, float l, float h) {
  return step(l, v) * (1.0 - step(h, v));
}

mat2 rotate2d(vec2 rad) {
  return mat2(rad.x, -rad.y, rad.y, rad.x);
}

float soft_circle(vec2 uv, vec2 xy, float radius, float blur) {
  float blur_half = blur * 0.5;
  float d = distance(uv, xy);
  return 1.0 - smoothstep(1.0 - blur_half, 1.0 + blur_half, d / radius);
}

float soft_ring(vec2 uv, vec2 xy, float radius, float thickness, float blur) {
  float circle_outer = soft_circle(uv, xy, radius + thickness, blur);
  float circle_inner = soft_circle(uv, xy, max(radius - thickness, 0.0), blur);
  return clamp(circle_outer - circle_inner, 0.0, 1.0);
}

float circle_grid(vec2 resolution,
                  vec2 p,
                  vec2 xy,
                  vec2 rotation,
                  float cell_diameter) {
  p = rotate2d(rotation) * (xy - p) + xy;
  p = mod(p, cell_diameter) / resolution;
  float cell_uv = cell_diameter / resolution.y * 0.5;
  float r = 0.65 * cell_uv;
  return soft_circle(p, vec2(cell_uv), r, r * 50.0);
}

float sparkle(vec2 uv, float t) {
  float n = triangle_noise(uv);
  float s = threshold(n, 0.0, 0.05);
  s += threshold(n + sin(PI * (t + 0.35)), 0.1, 0.15);
  s += threshold(n + sin(PI * (t + 0.7)), 0.2, 0.25);
  s += threshold(n + sin(PI * (t + 1.05)), 0.3, 0.35);
  return clamp(s, 0.0, 1.0) * 0.55;
}

float turbulence(vec2 uv) {
  vec2 uv_scale = uv * TURBULENCE_SCALE;
  float g1 =
      circle_grid(TURBULENCE_SCALE, uv_scale, u_circle1, u_rotation1, 0.17);
  float g2 =
      circle_grid(TURBULENCE_SCALE, uv_scale, u_circle2, u_rotation2, 0.2);
  float g3 =
      circle_grid(TURBULENCE_SCALE, uv_scale, u_circle3, u_rotation3, 0.275);
  float v = (g1 * g1 + g2 - g3) * 0.5;
  return clamp(0.45 + 0.8 * v, 0.0, 1.0);
}

void main() {
  // This block of code triggers the compiler to emit the uniforms out of order
  // if they are not explicitly sorted.
  vec2 p = FlutterFragCoord();
  vec2 uv = p * u_resolution_scale;
  vec2 density_uv = uv - mod(p, u_noise_scale);
  float radius = u_max_radius * u_radius_scale;
  float turbulence = turbulence(uv);
  float ring = soft_ring(p, u_center, radius, 0.05 * u_max_radius, u_blur);
  float sparkle =
      sparkle(density_uv, u_noise_phase) * ring * turbulence * u_sparkle_alpha;
  float wave_alpha =
      soft_circle(p, u_center, radius, u_blur) * u_alpha * u_color.a;
  vec4 wave_color = vec4(u_color.rgb * wave_alpha, wave_alpha);
  vec4 sparkle_color =
      vec4(u_sparkle_color.rgb * u_sparkle_color.a, u_sparkle_color.a);
  fragColor = mix(wave_color, sparkle_color, sparkle);

  vec4 badColor = vec4(1.0, 0, 0, 1.0);
  vec4 goodColor = vec4(0, 1.0, 0, 1.0);
  if (u_color.x > u_alpha || u_alpha > u_sparkle_color.x ||
      u_sparkle_color.x > u_sparkle_alpha || u_sparkle_alpha > u_blur ||
      u_blur > u_center.x || u_center.x > u_radius_scale ||
      u_radius_scale > u_max_radius || u_max_radius > u_resolution_scale.x ||
      u_resolution_scale.x > u_noise_scale.x ||
      u_noise_scale.x > u_noise_phase || u_noise_phase > u_circle1.x ||
      u_circle1.x > u_circle2.x || u_circle2.x > u_circle3.x ||
      u_circle3.x > u_rotation1.x || u_rotation1.x > u_rotation2.x ||
      u_rotation2.x > u_rotation3.x) {
    fragColor = badColor;
  } else {
    fragColor = goodColor;
  }
}
