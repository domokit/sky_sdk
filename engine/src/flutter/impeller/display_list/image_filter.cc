// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/display_list/image_filter.h"

#include "flutter/display_list/effects/dl_image_filters.h"
#include "fml/logging.h"
#include "impeller/display_list/color_filter.h"
#include "impeller/display_list/skia_conversions.h"
#include "impeller/entity/contents/filters/color_filter_contents.h"
#include "impeller/entity/contents/filters/filter_contents.h"
#include "impeller/entity/contents/filters/inputs/filter_input.h"

namespace impeller {

std::shared_ptr<FilterContents> WrapInput(const flutter::DlImageFilter* filter,
                                          const FilterInput::Ref& input) {
  FML_DCHECK(filter);

  switch (filter->type()) {
    case flutter::DlImageFilterType::kBlur: {
      auto blur_filter = filter->asBlur();
      FML_DCHECK(blur_filter);

      return FilterContents::MakeGaussianBlur(
          input,                                                    //
          Sigma(blur_filter->sigma_x()),                            //
          Sigma(blur_filter->sigma_y()),                            //
          static_cast<Entity::TileMode>(blur_filter->tile_mode()),  //
          FilterContents::BlurStyle::kNormal                        //
      );
    }
    case flutter::DlImageFilterType::kDilate: {
      auto dilate_filter = filter->asDilate();
      FML_DCHECK(dilate_filter);

      return FilterContents::MakeMorphology(
          input,                              //
          Radius(dilate_filter->radius_x()),  //
          Radius(dilate_filter->radius_y()),  //
          FilterContents::MorphType::kDilate  //
      );
    }
    case flutter::DlImageFilterType::kErode: {
      auto erode_filter = filter->asErode();
      FML_DCHECK(erode_filter);

      return FilterContents::MakeMorphology(
          input,                             //
          Radius(erode_filter->radius_x()),  //
          Radius(erode_filter->radius_y()),  //
          FilterContents::MorphType::kErode  //
      );
    }
    case flutter::DlImageFilterType::kMatrix: {
      auto matrix_filter = filter->asMatrix();
      FML_DCHECK(matrix_filter);

      auto matrix = matrix_filter->matrix();
      auto desc =
          skia_conversions::ToSamplerDescriptor(matrix_filter->sampling());
      return FilterContents::MakeMatrixFilter(input, matrix, desc);
    }
    case flutter::DlImageFilterType::kLocalMatrix: {
      auto matrix_filter = filter->asLocalMatrix();
      FML_DCHECK(matrix_filter);
      FML_DCHECK(matrix_filter->image_filter());

      auto matrix = matrix_filter->matrix();
      return FilterContents::MakeLocalMatrixFilter(
          FilterInput::Make(
              WrapInput(matrix_filter->image_filter().get(), input)),
          matrix);
    }
    case flutter::DlImageFilterType::kColorFilter: {
      auto image_color_filter = filter->asColorFilter();
      FML_DCHECK(image_color_filter);
      auto color_filter = image_color_filter->color_filter();
      FML_DCHECK(color_filter);

      // When color filters are used as image filters, set the color filter's
      // "absorb opacity" flag to false. For image filters, the snapshot
      // opacity needs to be deferred until the result of the filter chain is
      // being blended with the layer.
      return WrapWithGPUColorFilter(color_filter.get(), input,
                                    ColorFilterContents::AbsorbOpacity::kNo);
    }
    case flutter::DlImageFilterType::kCompose: {
      auto compose = filter->asCompose();
      FML_DCHECK(compose);

      auto outer_dl_filter = compose->outer();
      auto inner_dl_filter = compose->inner();
      if (!outer_dl_filter) {
        return WrapInput(inner_dl_filter.get(), input);
      }
      if (!inner_dl_filter) {
        return WrapInput(outer_dl_filter.get(), input);
      }
      FML_DCHECK(outer_dl_filter && inner_dl_filter);

      return WrapInput(
          outer_dl_filter.get(),
          FilterInput::Make(WrapInput(inner_dl_filter.get(), input)));
    }
    case flutter::DlImageFilterType::kRuntimeEffect: {
      const flutter::DlRuntimeEffectImageFilter* runtime_filter =
          filter->asRuntimeEffectFilter();
      FML_DCHECK(runtime_filter);
      std::shared_ptr<impeller::RuntimeStage> runtime_stage =
          runtime_filter->runtime_effect()->runtime_stage();

      std::vector<RuntimeEffectContents::TextureInput> texture_inputs;
      size_t index = 0;
      for (const std::shared_ptr<flutter::DlColorSource>& sampler :
           runtime_filter->samplers()) {
        if (index == 0 && sampler == nullptr) {
          // Insert placeholder for filter.
          texture_inputs.push_back(
              {.sampler_descriptor = skia_conversions::ToSamplerDescriptor({}),
               .texture = nullptr});
          continue;
        }
        if (sampler == nullptr) {
          return nullptr;
        }
        auto* image = sampler->asImage();
        if (!image) {
          return nullptr;
        }
        FML_DCHECK(image->image()->impeller_texture());
        index++;
        texture_inputs.push_back({
            .sampler_descriptor =
                skia_conversions::ToSamplerDescriptor(image->sampling()),
            .texture = image->image()->impeller_texture(),
        });
      }
      return FilterContents::MakeRuntimeEffect(input, std::move(runtime_stage),
                                               runtime_filter->uniform_data(),
                                               std::move(texture_inputs));
    }
  }
  FML_UNREACHABLE();
}

}  // namespace impeller
