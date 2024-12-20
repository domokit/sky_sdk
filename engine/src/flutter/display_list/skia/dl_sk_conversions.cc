// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/skia/dl_sk_conversions.h"

#include "flutter/display_list/effects/dl_color_filters.h"
#include "flutter/display_list/effects/dl_color_sources.h"
#include "flutter/display_list/effects/dl_image_filters.h"
#include "third_party/skia/include/core/SkColorFilter.h"
#include "third_party/skia/include/effects/SkGradientShader.h"
#include "third_party/skia/include/effects/SkImageFilters.h"

namespace flutter {

// clang-format off
constexpr float kInvertColorMatrix[20] = {
  -1.0,    0,    0, 1.0, 0,
     0, -1.0,    0, 1.0, 0,
     0,    0, -1.0, 1.0, 0,
   1.0,  1.0,  1.0, 1.0, 0
};
// clang-format on

SkPaint ToSk(const DlPaint& paint) {
  SkPaint sk_paint;

  sk_paint.setAntiAlias(paint.isAntiAlias());
  sk_paint.setColor(ToSk(paint.getColor()));
  sk_paint.setBlendMode(ToSk(paint.getBlendMode()));
  sk_paint.setStyle(ToSk(paint.getDrawStyle()));
  sk_paint.setStrokeWidth(paint.getStrokeWidth());
  sk_paint.setStrokeMiter(paint.getStrokeMiter());
  sk_paint.setStrokeCap(ToSk(paint.getStrokeCap()));
  sk_paint.setStrokeJoin(ToSk(paint.getStrokeJoin()));
  sk_paint.setImageFilter(ToSk(paint.getImageFilterPtr()));
  auto color_filter = ToSk(paint.getColorFilterPtr());
  if (paint.isInvertColors()) {
    auto invert_filter = SkColorFilters::Matrix(kInvertColorMatrix);
    if (color_filter) {
      invert_filter = invert_filter->makeComposed(color_filter);
    }
    color_filter = invert_filter;
  }
  sk_paint.setColorFilter(color_filter);

  auto color_source = paint.getColorSourcePtr();
  if (color_source) {
    // Unconditionally set dither to true for gradient shaders.
    sk_paint.setDither(color_source->isGradient());
    sk_paint.setShader(ToSk(color_source));
  }

  sk_paint.setMaskFilter(ToSk(paint.getMaskFilterPtr()));

  return sk_paint;
}

SkPaint ToStrokedSk(const DlPaint& paint) {
  DlPaint stroked_paint = paint;
  stroked_paint.setDrawStyle(DlDrawStyle::kStroke);
  return ToSk(stroked_paint);
}

SkPaint ToNonShaderSk(const DlPaint& paint) {
  DlPaint non_shader_paint = paint;
  non_shader_paint.setColorSource(nullptr);
  return ToSk(non_shader_paint);
}

sk_sp<SkShader> ToSk(const DlColorSource* source) {
  if (!source) {
    return nullptr;
  }
  SkMatrix scratch;
  static auto ToSkColors =
      [](const DlGradientColorSourceBase* gradient) -> std::vector<SkColor> {
    std::vector<SkColor> sk_colors;
    sk_colors.reserve(gradient->stop_count());
    for (int i = 0; i < gradient->stop_count(); ++i) {
      sk_colors.push_back(gradient->colors()[i].argb());
    }
    return sk_colors;
  };
  switch (source->type()) {
    case DlColorSourceType::kImage: {
      const DlImageColorSource* image_source = source->asImage();
      FML_DCHECK(image_source != nullptr);
      auto image = image_source->image();
      if (!image || !image->skia_image()) {
        return nullptr;
      }
      return image->skia_image()->makeShader(
          ToSk(image_source->horizontal_tile_mode()),
          ToSk(image_source->vertical_tile_mode()),
          ToSk(image_source->sampling()),
          ToSk(image_source->matrix_ptr(), scratch));
    }
    case DlColorSourceType::kLinearGradient: {
      const DlLinearGradientColorSource* linear_source =
          source->asLinearGradient();
      FML_DCHECK(linear_source != nullptr);
      SkPoint pts[] = {ToSkPoint(linear_source->start_point()),
                       ToSkPoint(linear_source->end_point())};
      std::vector<SkColor> skcolors = ToSkColors(linear_source);
      return SkGradientShader::MakeLinear(
          pts, skcolors.data(), linear_source->stops(),
          linear_source->stop_count(), ToSk(linear_source->tile_mode()), 0,
          ToSk(linear_source->matrix_ptr(), scratch));
    }
    case DlColorSourceType::kRadialGradient: {
      const DlRadialGradientColorSource* radial_source =
          source->asRadialGradient();
      FML_DCHECK(radial_source != nullptr);
      return SkGradientShader::MakeRadial(
          ToSkPoint(radial_source->center()), radial_source->radius(),
          ToSkColors(radial_source).data(), radial_source->stops(),
          radial_source->stop_count(), ToSk(radial_source->tile_mode()), 0,
          ToSk(radial_source->matrix_ptr(), scratch));
    }
    case DlColorSourceType::kConicalGradient: {
      const DlConicalGradientColorSource* conical_source =
          source->asConicalGradient();
      FML_DCHECK(conical_source != nullptr);
      return SkGradientShader::MakeTwoPointConical(
          ToSkPoint(conical_source->start_center()),
          conical_source->start_radius(),
          ToSkPoint(conical_source->end_center()), conical_source->end_radius(),
          ToSkColors(conical_source).data(), conical_source->stops(),
          conical_source->stop_count(), ToSk(conical_source->tile_mode()), 0,
          ToSk(conical_source->matrix_ptr(), scratch));
    }
    case DlColorSourceType::kSweepGradient: {
      const DlSweepGradientColorSource* sweep_source =
          source->asSweepGradient();
      FML_DCHECK(sweep_source != nullptr);
      return SkGradientShader::MakeSweep(
          sweep_source->center().x, sweep_source->center().y,
          ToSkColors(sweep_source).data(), sweep_source->stops(),
          sweep_source->stop_count(), ToSk(sweep_source->tile_mode()),
          sweep_source->start(), sweep_source->end(), 0,
          ToSk(sweep_source->matrix_ptr(), scratch));
    }
    case DlColorSourceType::kRuntimeEffect: {
      const DlRuntimeEffectColorSource* runtime_source =
          source->asRuntimeEffect();
      FML_DCHECK(runtime_source != nullptr);
      auto runtime_effect = runtime_source->runtime_effect();
      if (!runtime_effect || !runtime_effect->skia_runtime_effect()) {
        return nullptr;
      }

      auto samplers = runtime_source->samplers();
      std::vector<sk_sp<SkShader>> sk_samplers(samplers.size());
      for (size_t i = 0; i < samplers.size(); i++) {
        auto sampler = samplers[i];
        if (sampler == nullptr) {
          return nullptr;
        }
        sk_samplers[i] = ToSk(sampler);
      }

      auto uniform_data = runtime_source->uniform_data();
      auto ref = new std::shared_ptr<std::vector<uint8_t>>(uniform_data);
      auto sk_uniform_data = SkData::MakeWithProc(
          uniform_data->data(), uniform_data->size(),
          [](const void* ptr, void* context) {
            delete reinterpret_cast<std::shared_ptr<std::vector<uint8_t>>*>(
                context);
          },
          ref);

      return runtime_effect->skia_runtime_effect()->makeShader(
          sk_uniform_data, sk_samplers.data(), sk_samplers.size());
    }
  }
}

sk_sp<SkImageFilter> ToSk(const DlImageFilter* filter) {
  if (!filter) {
    return nullptr;
  }
  switch (filter->type()) {
    case DlImageFilterType::kBlur: {
      const DlBlurImageFilter* blur_filter = filter->asBlur();
      FML_DCHECK(blur_filter != nullptr);
      return SkImageFilters::Blur(blur_filter->sigma_x(),
                                  blur_filter->sigma_y(),
                                  ToSk(blur_filter->tile_mode()), nullptr);
    }
    case DlImageFilterType::kDilate: {
      const DlDilateImageFilter* dilate_filter = filter->asDilate();
      FML_DCHECK(dilate_filter != nullptr);
      return SkImageFilters::Dilate(dilate_filter->radius_x(),
                                    dilate_filter->radius_y(), nullptr);
    }
    case DlImageFilterType::kErode: {
      const DlErodeImageFilter* erode_filter = filter->asErode();
      FML_DCHECK(erode_filter != nullptr);
      return SkImageFilters::Erode(erode_filter->radius_x(),
                                   erode_filter->radius_y(), nullptr);
    }
    case DlImageFilterType::kMatrix: {
      const DlMatrixImageFilter* matrix_filter = filter->asMatrix();
      FML_DCHECK(matrix_filter != nullptr);
      return SkImageFilters::MatrixTransform(
          ToSkMatrix(matrix_filter->matrix()), ToSk(matrix_filter->sampling()),
          nullptr);
    }
    case DlImageFilterType::kCompose: {
      const DlComposeImageFilter* compose_filter = filter->asCompose();
      FML_DCHECK(compose_filter != nullptr);
      return SkImageFilters::Compose(ToSk(compose_filter->outer()),
                                     ToSk(compose_filter->inner()));
    }
    case DlImageFilterType::kColorFilter: {
      const DlColorFilterImageFilter* cf_filter = filter->asColorFilter();
      FML_DCHECK(cf_filter != nullptr);
      return SkImageFilters::ColorFilter(ToSk(cf_filter->color_filter()),
                                         nullptr);
    }
    case DlImageFilterType::kLocalMatrix: {
      const DlLocalMatrixImageFilter* lm_filter = filter->asLocalMatrix();
      FML_DCHECK(lm_filter != nullptr);
      sk_sp<SkImageFilter> skia_filter = ToSk(lm_filter->image_filter());
      // The image_filter property itself might have been null, or the
      // construction of the SkImageFilter might be optimized to null
      // for any number of reasons. In any case, if the filter is null
      // or optimizaed away, let's then optimize away this local matrix
      // case by returning null.
      if (!skia_filter) {
        return nullptr;
      }
      return skia_filter->makeWithLocalMatrix(ToSkMatrix(lm_filter->matrix()));
    }
    case DlImageFilterType::kRuntimeEffect:
      // UNSUPPORTED.
      return nullptr;
  }
}

sk_sp<SkColorFilter> ToSk(const DlColorFilter* filter) {
  if (!filter) {
    return nullptr;
  }
  switch (filter->type()) {
    case DlColorFilterType::kBlend: {
      const DlBlendColorFilter* blend_filter = filter->asBlend();
      FML_DCHECK(blend_filter != nullptr);
      return SkColorFilters::Blend(ToSk(blend_filter->color()),
                                   ToSk(blend_filter->mode()));
    }
    case DlColorFilterType::kMatrix: {
      const DlMatrixColorFilter* matrix_filter = filter->asMatrix();
      FML_DCHECK(matrix_filter != nullptr);
      float matrix[20];
      matrix_filter->get_matrix(matrix);
      return SkColorFilters::Matrix(matrix);
    }
    case DlColorFilterType::kSrgbToLinearGamma: {
      return SkColorFilters::SRGBToLinearGamma();
    }
    case DlColorFilterType::kLinearToSrgbGamma: {
      return SkColorFilters::LinearToSRGBGamma();
    }
  }
}

sk_sp<SkMaskFilter> ToSk(const DlMaskFilter* filter) {
  if (!filter) {
    return nullptr;
  }
  switch (filter->type()) {
    case DlMaskFilterType::kBlur: {
      const DlBlurMaskFilter* blur_filter = filter->asBlur();
      FML_DCHECK(blur_filter != nullptr);
      return SkMaskFilter::MakeBlur(ToSk(blur_filter->style()),
                                    blur_filter->sigma(),
                                    blur_filter->respectCTM());
    }
  }
}

sk_sp<SkVertices> ToSk(const std::shared_ptr<DlVertices>& vertices) {
  std::vector<SkColor> sk_colors;
  const SkColor* sk_colors_ptr = nullptr;
  if (vertices->colors()) {
    sk_colors.reserve(vertices->vertex_count());
    for (int i = 0; i < vertices->vertex_count(); ++i) {
      sk_colors.push_back(vertices->colors()[i].argb());
    }
    sk_colors_ptr = sk_colors.data();
  }
  return SkVertices::MakeCopy(ToSk(vertices->mode()), vertices->vertex_count(),
                              ToSkPoints(vertices->vertex_data()),
                              ToSkPoints(vertices->texture_coordinate_data()),
                              sk_colors_ptr, vertices->index_count(),
                              vertices->indices());
}

}  // namespace flutter
