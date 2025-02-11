// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/contents/solid_rrect_blur_contents.h"
#include <optional>

#include "impeller/entity/contents/content_context.h"
#include "impeller/entity/entity.h"
#include "impeller/geometry/color.h"
#include "impeller/geometry/constants.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/vertex_buffer_builder.h"

namespace impeller {

namespace {
// Generous padding to make sure blurs with large sigmas are fully visible. Used
// to expand the geometry around the rrect.  Larger sigmas have more subtle
// gradients so they need larger padding to avoid hard cutoffs.  Sigma is
// maximized to 3.5 since that should cover 99.95% of all samples.  3.0 should
// cover 99.7% but that was seen to be not enough for large sigmas.
Scalar PadForSigma(Scalar sigma) {
  Scalar scalar = std::min((1.0f / 47.6f) * sigma + 2.5f, 3.5f);
  return sigma * scalar;
}
}  // namespace

SolidRRectBlurContents::SolidRRectBlurContents() = default;

SolidRRectBlurContents::~SolidRRectBlurContents() = default;

void SolidRRectBlurContents::SetRRect(std::optional<Rect> rect,
                                      Size corner_radii) {
  rect_ = rect;
  corner_radii_ = corner_radii;
}

void SolidRRectBlurContents::SetSigma(Sigma sigma) {
  sigma_ = sigma;
}

void SolidRRectBlurContents::SetColor(Color color) {
  color_ = color.Premultiply();
}

Color SolidRRectBlurContents::GetColor() const {
  return color_;
}

static Point eccentricity(Point v, double sInverse) {
  Point vOverS = v * sInverse * 0.5;
  Point vOverS_squared = -(vOverS * vOverS);
  return {std::exp(vOverS_squared.x), std::exp(vOverS_squared.y)};
}

static Scalar kTwoOverSqrtPi = 2.0 / std::sqrt(kPi);

// use crate::math::compute_erf7;
static Scalar computeErf7(Scalar x) {
  x *= kTwoOverSqrtPi;
  float xx = x * x;
  x = x + (0.24295 + (0.03395 + 0.0104 * xx) * xx) * (x * xx);
  return x / sqrt(1.0 + x * x);
}

static Point NegPos(Scalar v) {
  return {std::min(v, 0.0f), std::max(v, 0.0f)};
}

static bool SetupFragInfo(
    RRectBlurPipeline::FragmentShader::FragInfo& frag_info,
    Scalar blurSigma,
    Point center,
    Point rSize,
    Scalar radius) {
  Scalar sigma = std::max(blurSigma * kSqrt2, 1.f);

  frag_info.center = rSize * 0.5f;
  frag_info.minEdge = std::min(rSize.x, rSize.y);
  double rMax = 0.5 * frag_info.minEdge;
  double r0 = std::min(std::hypot(radius, sigma * 1.15), rMax);
  frag_info.r1 = std::min(std::hypot(radius, sigma * 2.0), rMax);

  frag_info.exponent = 2.0 * frag_info.r1 / r0;

  frag_info.sInv = 1.0 / sigma;

  // Pull in long end (make less eccentric).
  Point eccentricV = eccentricity(rSize, frag_info.sInv);
  double delta = 1.25 * sigma * (eccentricV.x - eccentricV.y);
  rSize += NegPos(delta);

  frag_info.adjust = rSize * 0.5 - frag_info.r1;
  frag_info.exponentInv = 1.0 / frag_info.exponent;
  frag_info.scale =
      0.5 * computeErf7(frag_info.sInv * 0.5 *
                        (std::max(rSize.x, rSize.y) - 0.5 * radius));

  return frag_info.center.IsFinite() &&           //
         frag_info.adjust.IsFinite() &&           //
         std::isfinite(frag_info.minEdge) &&      //
         std::isfinite(frag_info.r1) &&           //
         std::isfinite(frag_info.exponent) &&     //
         std::isfinite(frag_info.sInv) &&         //
         std::isfinite(frag_info.exponentInv) &&  //
         std::isfinite(frag_info.scale);
}

std::optional<Rect> SolidRRectBlurContents::GetCoverage(
    const Entity& entity) const {
  if (!rect_.has_value()) {
    return std::nullopt;
  }

  Scalar radius = PadForSigma(sigma_.sigma);

  return rect_->Expand(radius).TransformBounds(entity.GetTransform());
}

bool SolidRRectBlurContents::Render(const ContentContext& renderer,
                                    const Entity& entity,
                                    RenderPass& pass) const {
  if (!rect_.has_value()) {
    return true;
  }

  using VS = RRectBlurPipeline::VertexShader;
  using FS = RRectBlurPipeline::FragmentShader;

  Matrix basis_invert = entity.GetTransform().Basis().Invert();
  Vector2 max_sigmas =
      Vector2((basis_invert * Vector2(500.f, 0.f)).GetLength(),
              (basis_invert * Vector2(0.f, 500.f)).GetLength());
  Scalar max_sigma = std::min(max_sigmas.x, max_sigmas.y);
  // Clamp the max kernel width/height to 1000 (@ 2x) to limit the extent
  // of the blur and to kEhCloseEnough to prevent NaN calculations
  // trying to evaluate a Gaussian distribution with a sigma of 0.
  auto blur_sigma = std::clamp(sigma_.sigma, kEhCloseEnough, max_sigma);
  // Increase quality by making the radius a bit bigger than the typical
  // sigma->radius conversion we use for slower blurs.
  Scalar blur_radius = PadForSigma(blur_sigma);
  Rect positive_rect = rect_->GetPositive();
  Scalar left = -blur_radius;
  Scalar top = -blur_radius;
  Scalar right = positive_rect.GetWidth() + blur_radius;
  Scalar bottom = positive_rect.GetHeight() + blur_radius;

  std::array<VS::PerVertexData, 4> vertices = {
      VS::PerVertexData{Point(left, top)},
      VS::PerVertexData{Point(right, top)},
      VS::PerVertexData{Point(left, bottom)},
      VS::PerVertexData{Point(right, bottom)},
  };

  ContentContextOptions opts = OptionsFromPassAndEntity(pass, entity);
  opts.primitive_type = PrimitiveType::kTriangleStrip;
  Color color = color_;
  if (entity.GetBlendMode() == BlendMode::kClear) {
    opts.is_for_rrect_blur_clear = true;
    color = Color::White();
  }

  VS::FrameInfo frame_info;
  frame_info.mvp = Entity::GetShaderTransform(
      entity.GetShaderClipDepth(), pass,
      entity.GetTransform() *
          Matrix::MakeTranslation(positive_rect.GetOrigin()));

  FS::FragInfo frag_info;
  frag_info.color = color;
  Scalar radius = std::min(std::clamp(corner_radii_.width, kEhCloseEnough,
                                      positive_rect.GetWidth() * 0.5f),
                           std::clamp(corner_radii_.height, kEhCloseEnough,
                                      positive_rect.GetHeight() * 0.5f));
  if (!SetupFragInfo(frag_info, blur_sigma, positive_rect.GetCenter(),
                     Point(positive_rect.GetSize()), radius)) {
    return true;
  }

  auto& host_buffer = renderer.GetTransientsBuffer();
  pass.SetCommandLabel("RRect Shadow");
  pass.SetPipeline(renderer.GetRRectBlurPipeline(opts));
  pass.SetVertexBuffer(CreateVertexBuffer(vertices, host_buffer));

  VS::BindFrameInfo(pass, host_buffer.EmplaceUniform(frame_info));
  FS::BindFragInfo(pass, host_buffer.EmplaceUniform(frag_info));

  if (!pass.Draw().ok()) {
    return false;
  }

  return true;
}

bool SolidRRectBlurContents::ApplyColorFilter(
    const ColorFilterProc& color_filter_proc) {
  color_ = color_filter_proc(color_);
  return true;
}

}  // namespace impeller
