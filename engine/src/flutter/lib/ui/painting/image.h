// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_H_

#include "flutter/display_list/image/dl_image.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/skia/include/core/SkImage.h"

namespace flutter {

// Must be kept in sync with painting.dart.
enum ColorSpace {
  kSRGB,
  kExtendedSRGB,
};

class CanvasImage final : public RefCountedDartWrappable<CanvasImage> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(CanvasImage);

 public:
  ~CanvasImage() override;

  static fml::RefPtr<CanvasImage> Create() {
    return fml::MakeRefCounted<CanvasImage>();
  }

  Dart_Handle CreateOuterWrapping();

  int width() { return image_ ? image_->width() : 0; }

  int height() { return image_ ? image_->height() : 0; }

  Dart_Handle toByteData(int format, Dart_Handle callback);

  void dispose();

  sk_sp<DlImage> image() const { return image_; }

  void set_image(const sk_sp<DlImage>& image) {
    FML_DCHECK(image->isUIThreadSafe());
    image_ = image;
  }

  int colorSpace();

 private:
  CanvasImage();

  sk_sp<DlImage> image_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_H_
