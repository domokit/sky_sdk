// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_PICTURE_RECORDER_H_
#define FLUTTER_LIB_UI_PAINTING_PICTURE_RECORDER_H_

#include "flutter/display_list/dl_builder.h"
#include "flutter/lib/ui/dart_wrapper.h"

namespace flutter {
class Canvas;
class Picture;

class PictureRecorder : public RefCountedDartWrappable<PictureRecorder> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(PictureRecorder);

 public:
  static void Create(Dart_Handle wrapper);

  ~PictureRecorder() override;

  sk_sp<DisplayListBuilder> BeginRecording(DlRect bounds);
  void endRecording(Dart_Handle dart_picture);

  void set_canvas(fml::RefPtr<Canvas> canvas) { canvas_ = std::move(canvas); }

 private:
  PictureRecorder();

  sk_sp<DisplayListBuilder> display_list_builder_;

  fml::RefPtr<Canvas> canvas_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_PICTURE_RECORDER_H_
