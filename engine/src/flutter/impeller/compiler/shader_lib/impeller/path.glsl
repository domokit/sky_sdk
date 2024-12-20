// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef PATH_GLSL_
#define PATH_GLSL_

struct LineData {
  vec2 p1;
  vec2 p2;
};

struct QuadData {
  vec2 p1;
  vec2 cp;
  vec2 p2;
};

struct CubicData {
  vec2 p1;
  vec2 cp1;
  vec2 cp2;
  vec2 p2;
};

struct QuadDecomposition {
  float a0;
  float a2;
  float u0;
  float u_scale;
  uint line_count;
  float steps;
};

struct PathComponent {
  uint index;  // Location in buffer
  uint count;  // Number of points. 4 = cubic, 3 = quad, 2 = line.
};

/// Solve for point on a quadratic Bezier curve defined by starting point `p1`,
/// control point `cp`, and end point `p2` at time `t`.
vec2 QuadraticSolve(QuadData quad, float t) {
  return (1.0 - t) * (1.0 - t) * quad.p1 +  //
         2.0 * (1.0 - t) * t * quad.cp +    //
         t * t * quad.p2;
}

vec2 CubicSolve(CubicData cubic, float t) {
  return (1. - t) * (1. - t) * (1. - t) * cubic.p1 +  //
         3 * (1. - t) * (1. - t) * t * cubic.cp1 +    //
         3 * (1. - t) * t * t * cubic.cp2 +           //
         t * t * t * cubic.p2;
}

/// Used to approximate quadratic curves using parabola.
///
/// See
/// https://raphlinus.github.io/graphics/curves/2019/12/23/flatten-quadbez.html
float ApproximateParabolaIntegral(float x) {
  float d = 0.67;
  return x / (1.0 - d + sqrt(sqrt(pow(d, 4) + 0.25 * x * x)));
}

bool isfinite(float f) {
  return !isnan(f) && !isinf(f);
}

float Cross(vec2 p1, vec2 p2) {
  return p1.x * p2.y - p1.y * p2.x;
}

QuadData GenerateQuadraticFromCubic(CubicData cubic,
                                    uint index,
                                    float quad_count) {
  float t0 = index / quad_count;
  float t1 = (index + 1) / quad_count;

  // calculate the subsegment
  vec2 sub_p1 = CubicSolve(cubic, t0);
  vec2 sub_p2 = CubicSolve(cubic, t1);
  QuadData quad = QuadData(3.0 * (cubic.cp1 - cubic.p1),   //
                           3.0 * (cubic.cp2 - cubic.cp1),  //
                           3.0 * (cubic.p2 - cubic.cp2));
  float sub_scale = (t1 - t0) * (1.0 / 3.0);
  vec2 sub_cp1 = sub_p1 + sub_scale * QuadraticSolve(quad, t0);
  vec2 sub_cp2 = sub_p2 - sub_scale * QuadraticSolve(quad, t1);

  vec2 quad_p1x2 = 3.0 * sub_cp1 - sub_p1;
  vec2 quad_p2x2 = 3.0 * sub_cp2 - sub_p2;

  return QuadData(sub_p1,                           //
                  ((quad_p1x2 + quad_p2x2) / 4.0),  //
                  sub_p2);
}

uint EstimateQuadraticCount(CubicData cubic, float accuracy) {
  // The maximum error, as a vector from the cubic to the best approximating
  // quadratic, is proportional to the third derivative, which is constant
  // across the segment. Thus, the error scales down as the third power of
  // the number of subdivisions. Our strategy then is to subdivide `t` evenly.
  //
  // This is an overestimate of the error because only the component
  // perpendicular to the first derivative is important. But the simplicity is
  // appealing.

  // This magic number is the square of 36 / sqrt(3).
  // See: http://caffeineowl.com/graphics/2d/vectorial/cubic2quad01.html
  float max_hypot2 = 432.0 * accuracy * accuracy;

  vec2 err_v = (3.0 * cubic.cp2 - cubic.p2) - (3.0 * cubic.cp1 - cubic.p1);
  float err = dot(err_v, err_v);
  return uint(max(1., ceil(pow(err * (1.0 / max_hypot2), 1. / 6.0))));
}

QuadDecomposition DecomposeQuad(QuadData quad, float tolerance) {
  float sqrt_tolerance = sqrt(tolerance);

  vec2 d01 = quad.cp - quad.p1;
  vec2 d12 = quad.p2 - quad.cp;
  vec2 dd = d01 - d12;
  // This should never happen, but if it does happen be more defensive -
  // otherwise we'll get NaNs down the line.
  if (dd == vec2(0.)) {
    return QuadDecomposition(0., 0., 0., 0., 0, 0.);
  }
  float c = Cross(quad.p2 - quad.p1, dd);
  float x0 = dot(d01, dd) * 1. / c;
  float x2 = dot(d12, dd) * 1. / c;
  float scale = abs(c / (sqrt(dd.x * dd.x + dd.y * dd.y) * (x2 - x0)));

  float a0 = ApproximateParabolaIntegral(x0);
  float a2 = ApproximateParabolaIntegral(x2);
  float val = 0.f;
  if (isfinite(scale)) {
    float da = abs(a2 - a0);
    float sqrt_scale = sqrt(scale);
    if ((x0 < 0 && x2 < 0) || (x0 >= 0 && x2 >= 0)) {
      val = da * sqrt_scale;
    } else {
      // cusp case
      float xmin = sqrt_tolerance / sqrt_scale;
      val = sqrt_tolerance * da / ApproximateParabolaIntegral(xmin);
    }
  }
  float u0 = ApproximateParabolaIntegral(a0);
  float u2 = ApproximateParabolaIntegral(a2);
  float u_scale = 1. / (u2 - u0);

  uint line_count = uint(max(1., ceil(0.5 * val / sqrt_tolerance)) + 1.);
  float steps = 1. / line_count;

  return QuadDecomposition(a0, a2, u0, u_scale, line_count, steps);
}

vec2 GenerateLineFromQuad(QuadData quad,
                          uint index,
                          QuadDecomposition decomposition) {
  float u = index * decomposition.steps;
  float a = decomposition.a0 + (decomposition.a2 - decomposition.a0) * u;
  float t = (ApproximateParabolaIntegral(a) - decomposition.u0) *
            decomposition.u_scale;
  return QuadraticSolve(quad, t);
}

#endif
