// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_SHADER_H_
#define FLUTTER_LIB_UI_PAINTING_SHADER_H_

#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/lib/ui/ui_dart_state.h"

namespace flutter {

class Shader : public RefCountedDartWrappable<Shader> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(Shader);

 public:
  ~Shader() override;

  virtual std::shared_ptr<DlColorSource> shader(DlImageSampling) = 0;

 protected:
  Shader() {}
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_SHADER_H_
