// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "export.h"
#include "helpers.h"
#include "third_party/skia/include/core/SkBBHFactory.h"
#include "third_party/skia/include/core/SkPicture.h"
#include "third_party/skia/include/core/SkPictureRecorder.h"
#include "wrappers.h"

using namespace Skwasm;

SkRTreeFactory bbhFactory;

SKWASM_EXPORT SkPictureRecorder* pictureRecorder_create() {
  return new SkPictureRecorder();
}

SKWASM_EXPORT void pictureRecorder_dispose(SkPictureRecorder* recorder) {
  delete recorder;
}

SKWASM_EXPORT SkCanvas* pictureRecorder_beginRecording(
    SkPictureRecorder* recorder,
    const SkRect* cullRect) {
  return recorder->beginRecording(*cullRect, &bbhFactory);
}

SKWASM_EXPORT SkPicture* pictureRecorder_endRecording(
    SkPictureRecorder* recorder) {
  return recorder->finishRecordingAsPicture().release();
}

SKWASM_EXPORT void picture_getCullRect(SkPicture* picture, SkRect* outRect) {
  *outRect = picture->cullRect();
}

SKWASM_EXPORT void picture_dispose(SkPicture* picture) {
  picture->unref();
}

SKWASM_EXPORT uint32_t picture_approximateBytesUsed(SkPicture* picture) {
  return static_cast<uint32_t>(picture->approximateBytesUsed());
}
