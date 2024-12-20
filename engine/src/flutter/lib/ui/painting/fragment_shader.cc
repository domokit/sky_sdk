// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <memory>
#include <utility>

#include "flutter/lib/ui/painting/fragment_shader.h"

#include "flutter/display_list/dl_tile_mode.h"
#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/lib/ui/painting/fragment_program.h"
#include "third_party/tonic/converter/dart_converter.h"

namespace flutter {

IMPLEMENT_WRAPPERTYPEINFO(ui, ReusableFragmentShader);

ReusableFragmentShader::ReusableFragmentShader(
    fml::RefPtr<FragmentProgram> program,
    uint64_t float_count,
    uint64_t sampler_count)
    : program_(std::move(program)),
      uniform_data_(SkData::MakeUninitialized(
          (float_count + 2 * sampler_count) * sizeof(float))),
      samplers_(sampler_count),
      float_count_(float_count) {}

Dart_Handle ReusableFragmentShader::Create(Dart_Handle wrapper,
                                           Dart_Handle program,
                                           Dart_Handle float_count_handle,
                                           Dart_Handle sampler_count_handle) {
  auto* fragment_program =
      tonic::DartConverter<FragmentProgram*>::FromDart(program);
  uint64_t float_count =
      tonic::DartConverter<uint64_t>::FromDart(float_count_handle);
  uint64_t sampler_count =
      tonic::DartConverter<uint64_t>::FromDart(sampler_count_handle);

  auto res = fml::MakeRefCounted<ReusableFragmentShader>(
      fml::Ref(fragment_program), float_count, sampler_count);
  res->AssociateWithDartWrapper(wrapper);

  void* raw_uniform_data =
      reinterpret_cast<void*>(res->uniform_data_->writable_data());
  return Dart_NewExternalTypedData(Dart_TypedData_kFloat32, raw_uniform_data,
                                   float_count);
}

bool ReusableFragmentShader::ValidateSamplers() {
  for (auto i = 0u; i < samplers_.size(); i++) {
    if (samplers_[i] == nullptr) {
      return false;
    }
    // The samplers should have been checked as they were added, this
    // is a double-sanity-check.
    FML_DCHECK(samplers_[i]->isUIThreadSafe());
  }
  return true;
}

void ReusableFragmentShader::SetImageSampler(Dart_Handle index_handle,
                                             Dart_Handle image_handle) {
  uint64_t index = tonic::DartConverter<uint64_t>::FromDart(index_handle);
  CanvasImage* image =
      tonic::DartConverter<CanvasImage*>::FromDart(image_handle);
  if (index >= samplers_.size()) {
    Dart_ThrowException(tonic::ToDart("Sampler index out of bounds"));
  }
  if (!image->image()->isUIThreadSafe()) {
    Dart_ThrowException(tonic::ToDart("Image is not thread-safe"));
  }

  // TODO(115794): Once the DlImageSampling enum is replaced, expose the
  //               sampling options as a new default parameter for users.
  samplers_[index] = DlColorSource::MakeImage(
      image->image(), DlTileMode::kClamp, DlTileMode::kClamp,
      DlImageSampling::kNearestNeighbor, nullptr);
  // This should be true since we already checked the image above, but
  // we check again for sanity.
  FML_DCHECK(samplers_[index]->isUIThreadSafe());

  auto* uniform_floats =
      reinterpret_cast<float*>(uniform_data_->writable_data());
  uniform_floats[float_count_ + 2 * index] = image->width();
  uniform_floats[float_count_ + 2 * index + 1] = image->height();
}

std::shared_ptr<DlImageFilter> ReusableFragmentShader::as_image_filter() const {
  FML_CHECK(program_);

  // The lifetime of this object is longer than a frame, and the uniforms can be
  // continually changed on the UI thread. So we take a copy of the uniforms
  // before handing it to the DisplayList for consumption on the render thread.
  auto uniform_data = std::make_shared<std::vector<uint8_t>>();
  uniform_data->resize(uniform_data_->size());
  memcpy(uniform_data->data(), uniform_data_->bytes(), uniform_data->size());

  return program_->MakeDlImageFilter(std::move(uniform_data), samplers_);
}

std::shared_ptr<DlColorSource> ReusableFragmentShader::shader(
    DlImageSampling sampling) {
  FML_CHECK(program_);

  // The lifetime of this object is longer than a frame, and the uniforms can be
  // continually changed on the UI thread. So we take a copy of the uniforms
  // before handing it to the DisplayList for consumption on the render thread.
  auto uniform_data = std::make_shared<std::vector<uint8_t>>();
  uniform_data->resize(uniform_data_->size());
  memcpy(uniform_data->data(), uniform_data_->bytes(), uniform_data->size());

  auto source = program_->MakeDlColorSource(std::move(uniform_data), samplers_);
  // The samplers should have been checked as they were added, this
  // is a double-sanity-check.
  FML_DCHECK(source->isUIThreadSafe());
  return source;
}

// Image filters require at least one uniform sampler input to bind
// the input texture.
bool ReusableFragmentShader::ValidateImageFilter() {
  if (samplers_.size() < 1) {
    return false;
  }
  // The first sampler does not need to be set.
  for (auto i = 1u; i < samplers_.size(); i++) {
    if (samplers_[i] == nullptr) {
      return false;
    }
    // The samplers should have been checked as they were added, this
    // is a double-sanity-check.
    FML_DCHECK(samplers_[i]->isUIThreadSafe());
  }
  return true;
}

void ReusableFragmentShader::Dispose() {
  uniform_data_.reset();
  program_ = nullptr;
  samplers_.clear();
  ClearDartWrapper();
}

ReusableFragmentShader::~ReusableFragmentShader() = default;

}  // namespace flutter
