// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_FRAGMENT_SHADER_H_
#define FLUTTER_LIB_UI_PAINTING_FRAGMENT_SHADER_H_

#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/fragment_program.h"
#include "flutter/lib/ui/painting/image.h"
#include "flutter/lib/ui/painting/image_shader.h"
#include "flutter/lib/ui/painting/shader.h"
#include "third_party/skia/include/core/SkShader.h"
#include "third_party/skia/include/effects/SkRuntimeEffect.h"
#include "third_party/tonic/dart_library_natives.h"
#include "third_party/tonic/typed_data/typed_list.h"

#include <string>
#include <vector>

namespace flutter {

class FragmentProgram;

class ReusableFragmentShader : public Shader {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(ReusableFragmentShader);

 public:
  ~ReusableFragmentShader() override;

  static Dart_Handle Create(Dart_Handle wrapper,
                            Dart_Handle program,
                            Dart_Handle float_count,
                            Dart_Handle sampler_count);

  void SetImageSampler(Dart_Handle index, Dart_Handle image);

  bool ValidateSamplers();

  bool ValidateImageFilter();

  void Dispose();

  // |Shader|
  std::shared_ptr<DlColorSource> shader(DlImageSampling) override;

  std::shared_ptr<DlImageFilter> as_image_filter() const;

 private:
  ReusableFragmentShader(fml::RefPtr<FragmentProgram> program,
                         uint64_t float_count,
                         uint64_t sampler_count);

  fml::RefPtr<FragmentProgram> program_;
  sk_sp<SkData> uniform_data_;
  std::vector<std::shared_ptr<DlColorSource>> samplers_;
  size_t float_count_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_FRAGMENT_SHADER_H_
