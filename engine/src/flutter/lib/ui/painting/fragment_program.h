// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_FRAGMENT_PROGRAM_H_
#define FLUTTER_LIB_UI_PAINTING_FRAGMENT_PROGRAM_H_

#include "display_list/effects/dl_image_filter.h"
#include "flutter/display_list/effects/dl_runtime_effect.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/painting/shader.h"

#include "third_party/tonic/dart_library_natives.h"
#include "third_party/tonic/typed_data/typed_list.h"

#include <memory>
#include <string>
#include <vector>

namespace flutter {

class FragmentShader;

class FragmentProgram : public RefCountedDartWrappable<FragmentProgram> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(FragmentProgram);

 public:
  ~FragmentProgram() override;
  static void Create(Dart_Handle wrapper);

  std::string initFromAsset(const std::string& asset_name);

  fml::RefPtr<FragmentShader> shader(Dart_Handle shader,
                                     Dart_Handle uniforms_handle,
                                     Dart_Handle samplers);

  std::shared_ptr<DlColorSource> MakeDlColorSource(
      std::shared_ptr<std::vector<uint8_t>> float_uniforms,
      const std::vector<std::shared_ptr<DlColorSource>>& children);

  std::shared_ptr<DlImageFilter> MakeDlImageFilter(
      std::shared_ptr<std::vector<uint8_t>> float_uniforms,
      const std::vector<std::shared_ptr<DlColorSource>>& children);

 private:
  FragmentProgram();
  sk_sp<DlRuntimeEffect> runtime_effect_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_FRAGMENT_PROGRAM_H_
