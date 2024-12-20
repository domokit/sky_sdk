// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef GRADIENT_GLSL_
#define GRADIENT_GLSL_

#include <impeller/texture.glsl>
#include <impeller/transform.glsl>

mat3 IPMapToUnitX(vec2 p0, vec2 p1) {
  // Returns a matrix that maps [p0, p1] to [(0, 0), (1, 0)]. Results are
  // undefined if p0 = p1.
  return mat3(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0) *
         IPMat3Inverse(mat3(p1.y - p0.y, p0.x - p1.x, 0.0, p1.x - p0.x,
                            p1.y - p0.y, 0.0, p0.x, p0.y, 1.0));
}

/// Compute the t value for a conical gradient at point `p` between the 2
/// circles defined by (c0, r0) and (c1, r1). The returned vec2 encapsulates 't'
/// as its x component and validity status as its y component, with positive y
/// indicating a valid result.
///
/// The code is migrated from Skia Graphite. See
/// https://github.com/google/skia/blob/ddf987d2ab3314ee0e80ac1ae7dbffb44a87d394/src/sksl/sksl_graphite_frag.sksl#L541-L666.
vec2 IPComputeConicalT(vec2 c0, float r0, vec2 c1, float r1, vec2 pos) {
  const float scalar_nearly_zero = 1.0 / float(1 << 12);
  float d_center = distance(c0, c1);
  float d_radius = r1 - r0;

  // Degenerate case: a radial gradient (p0 = p1).
  bool radial = d_center < scalar_nearly_zero;

  // Degenerate case: a strip with bandwidth 2r (r0 = r1).
  bool strip = abs(d_radius) < scalar_nearly_zero;

  if (radial) {
    if (strip) {
      // The start and end inputs are the same in both position and radius.
      // We don't expect to see this input, but just in case we avoid dividing
      // by zero.
      return vec2(0.0, -1.0);
    }

    float scale = 1.0 / d_radius;
    float scale_sign = sign(d_radius);
    float bias = r0 / d_radius;

    vec2 pt = (pos - c0) * scale;
    float t = length(pt) * scale_sign - bias;
    return vec2(t, 1.0);

  } else if (strip) {
    mat3 transform = IPMapToUnitX(c0, c1);
    float r = r0 / d_center;
    float r_2 = r * r;

    vec2 pt = (transform * vec3(pos.xy, 1.0)).xy;
    float t = r_2 - pt.y * pt.y;
    if (t < 0.0) {
      return vec2(0.0, -1.0);
    }
    t = pt.x + sqrt(t);
    return vec2(t, 1.0);

  } else {
    // See https://skia.org/docs/dev/design/conical/ for details on how this
    // algorithm works. Calculate f and swap inputs if necessary (steps 1 and
    // 2).
    float f = r0 / (r0 - r1);

    bool is_swapped = abs(f - 1.0) < scalar_nearly_zero;
    if (is_swapped) {
      vec2 tmp_pt = c0;
      c0 = c1;
      c1 = tmp_pt;
      f = 0.0;
    }

    // Apply mapping from [Cf, C1] to unit x, and apply the precalculations from
    // steps 3 and 4, all in the same transform.
    vec2 cf = c0 * (1.0 - f) + c1 * f;
    mat3 transform = IPMapToUnitX(cf, c1);

    float scale_x = abs(1.0 - f);
    float scale_y = scale_x;
    float r1 = abs(r1 - r0) / d_center;
    bool is_focal_on_circle = abs(r1 - 1.0) < scalar_nearly_zero;
    if (is_focal_on_circle) {
      scale_x *= 0.5;
      scale_y *= 0.5;
    } else {
      scale_x *= r1 / (r1 * r1 - 1.0);
      scale_y /= sqrt(abs(r1 * r1 - 1.0));
    }
    transform =
        mat3(scale_x, 0.0, 0.0, 0.0, scale_y, 0.0, 0.0, 0.0, 1.0) * transform;

    vec2 pt = (transform * vec3(pos.xy, 1.0)).xy;

    // Continue with step 5 onward.
    float inv_r1 = 1.0 / r1;
    float d_radius_sign = sign(1.0 - f);
    bool is_well_behaved = !is_focal_on_circle && r1 > 1.0;

    float x_t = -1.0;
    if (is_focal_on_circle) {
      x_t = dot(pt, pt) / pt.x;
    } else if (is_well_behaved) {
      x_t = length(pt) - pt.x * inv_r1;
    } else {
      float temp = pt.x * pt.x - pt.y * pt.y;
      if (temp >= 0.0) {
        if (is_swapped || d_radius_sign < 0.0) {
          x_t = -sqrt(temp) - pt.x * inv_r1;
        } else {
          x_t = sqrt(temp) - pt.x * inv_r1;
        }
      }
    }

    if (!is_well_behaved && x_t < 0.0) {
      return vec2(0.0, -1.0);
    }

    float t = f + d_radius_sign * x_t;
    if (is_swapped) {
      t = 1.0 - t;
    }
    return vec2(t, 1.0);
  }
}

/// Compute the indexes and mix coefficient used to mix colors for an
/// arbitrarily sized color gradient.
///
/// The returned values are the lower index, upper index, and mix
/// coefficient.
vec3 IPComputeFixedGradientValues(float t, float colors_length) {
  float rough_index = (colors_length - 1) * t;
  float lower_index = floor(rough_index);
  float upper_index = ceil(rough_index);
  float scale = rough_index - lower_index;

  return vec3(lower_index, upper_index, scale);
}

#endif
