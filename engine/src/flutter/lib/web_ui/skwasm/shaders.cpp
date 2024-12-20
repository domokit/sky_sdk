// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "export.h"
#include "helpers.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/effects/SkGradientShader.h"
#include "third_party/skia/include/effects/SkRuntimeEffect.h"
#include "wrappers.h"

using namespace Skwasm;

SKWASM_EXPORT SkShader* shader_createLinearGradient(
    SkPoint* endPoints,  // Two points
    SkColor* colors,
    SkScalar* stops,
    int count,  // Number of stops/colors
    SkTileMode tileMode,
    SkScalar* matrix33  // Can be nullptr
) {
  if (matrix33) {
    SkMatrix localMatrix = createMatrix(matrix33);
    return SkGradientShader::MakeLinear(endPoints, colors, stops, count,
                                        tileMode, 0, &localMatrix)
        .release();
  } else {
    return SkGradientShader::MakeLinear(endPoints, colors, stops, count,
                                        tileMode)
        .release();
  }
}

SKWASM_EXPORT SkShader* shader_createRadialGradient(SkScalar centerX,
                                                    SkScalar centerY,
                                                    SkScalar radius,
                                                    SkColor* colors,
                                                    SkScalar* stops,
                                                    int count,
                                                    SkTileMode tileMode,
                                                    SkScalar* matrix33) {
  if (matrix33) {
    SkMatrix localMatrix = createMatrix(matrix33);
    return SkGradientShader::MakeRadial({centerX, centerY}, radius, colors,
                                        stops, count, tileMode, 0, &localMatrix)
        .release();
  } else {
    return SkGradientShader::MakeRadial({centerX, centerY}, radius, colors,
                                        stops, count, tileMode)
        .release();
  }
}

SKWASM_EXPORT SkShader* shader_createConicalGradient(
    SkPoint* endPoints,  // Two points
    SkScalar startRadius,
    SkScalar endRadius,
    SkColor* colors,
    SkScalar* stops,
    int count,
    SkTileMode tileMode,
    SkScalar* matrix33) {
  if (matrix33) {
    SkMatrix localMatrix = createMatrix(matrix33);
    return SkGradientShader::MakeTwoPointConical(
               endPoints[0], startRadius, endPoints[1], endRadius, colors,
               stops, count, tileMode, 0, &localMatrix)
        .release();

  } else {
    return SkGradientShader::MakeTwoPointConical(endPoints[0], startRadius,
                                                 endPoints[1], endRadius,
                                                 colors, stops, count, tileMode)
        .release();
  }
}

SKWASM_EXPORT SkShader* shader_createSweepGradient(SkScalar centerX,
                                                   SkScalar centerY,
                                                   SkColor* colors,
                                                   SkScalar* stops,
                                                   int count,
                                                   SkTileMode tileMode,
                                                   SkScalar startAngle,
                                                   SkScalar endAngle,
                                                   SkScalar* matrix33) {
  if (matrix33) {
    SkMatrix localMatrix = createMatrix(matrix33);
    return SkGradientShader::MakeSweep(centerX, centerY, colors, stops, count,
                                       tileMode, startAngle, endAngle, 0,
                                       &localMatrix)
        .release();
  } else {
    return SkGradientShader::MakeSweep(centerX, centerY, colors, stops, count,
                                       tileMode, startAngle, endAngle, 0,
                                       nullptr)
        .release();
  }
}

SKWASM_EXPORT void shader_dispose(SkShader* shader) {
  shader->unref();
}

SKWASM_EXPORT SkRuntimeEffect* runtimeEffect_create(SkString* source) {
  auto result = SkRuntimeEffect::MakeForShader(*source);
  if (result.effect == nullptr) {
    printf("Failed to compile shader. Error text:\n%s",
           result.errorText.data());
    return nullptr;
  } else {
    return result.effect.release();
  }
}

SKWASM_EXPORT void runtimeEffect_dispose(SkRuntimeEffect* effect) {
  effect->unref();
}

SKWASM_EXPORT size_t runtimeEffect_getUniformSize(SkRuntimeEffect* effect) {
  return effect->uniformSize();
}

SKWASM_EXPORT SkShader* shader_createRuntimeEffectShader(
    SkRuntimeEffect* runtimeEffect,
    SkData* uniforms,
    SkShader** children,
    size_t childCount) {
  std::vector<sk_sp<SkShader>> childPointers;
  for (size_t i = 0; i < childCount; i++) {
    childPointers.emplace_back(sk_ref_sp<SkShader>(children[i]));
  }

  return runtimeEffect
      ->makeShader(sk_ref_sp<SkData>(uniforms), childPointers.data(),
                   childCount, nullptr)
      .release();
}

SKWASM_EXPORT SkShader* shader_createFromImage(SkImage* image,
                                               SkTileMode tileModeX,
                                               SkTileMode tileModeY,
                                               FilterQuality quality,
                                               SkScalar* matrix33) {
  if (matrix33) {
    SkMatrix localMatrix = createMatrix(matrix33);
    return image
        ->makeShader(tileModeX, tileModeY, samplingOptionsForQuality(quality),
                     &localMatrix)
        .release();
  } else {
    return image
        ->makeShader(tileModeX, tileModeY, samplingOptionsForQuality(quality))
        .release();
  }
}
