// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/single_frame_codec.h"

#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/tonic/logging/dart_invoke.h"

namespace flutter {

SingleFrameCodec::SingleFrameCodec(
    const fml::RefPtr<ImageDescriptor>& descriptor,
    uint32_t target_width,
    uint32_t target_height)
    : descriptor_(descriptor),
      target_width_(target_width),
      target_height_(target_height) {}

SingleFrameCodec::~SingleFrameCodec() = default;

int SingleFrameCodec::frameCount() const {
  return 1;
}

int SingleFrameCodec::repetitionCount() const {
  return 0;
}

Dart_Handle SingleFrameCodec::getNextFrame(Dart_Handle callback_handle) {
  if (!Dart_IsClosure(callback_handle)) {
    return tonic::ToDart("Callback must be a function");
  }

  if (status_ == Status::kComplete) {
    if (!cached_image_->image()) {
      return tonic::ToDart("Decoded image has been disposed");
    }
    tonic::DartInvoke(callback_handle, {tonic::ToDart(cached_image_),
                                        tonic::ToDart(0), tonic::ToDart("")});
    return Dart_Null();
  }

  // This has to be valid because this method is called from Dart.
  auto dart_state = UIDartState::Current();

  pending_callbacks_.emplace_back(dart_state, callback_handle);

  if (status_ == Status::kInProgress) {
    // Another call to getNextFrame is in progress and will invoke the
    // pending callbacks when decoding completes.
    return Dart_Null();
  }

  auto decoder = dart_state->GetImageDecoder();

  if (!decoder) {
    return tonic::ToDart(
        "Failed to access the internal image decoder "
        "registry on this isolate. Please file a bug on "
        "https://github.com/flutter/flutter/issues.");
  }

  // The SingleFrameCodec must be deleted on the UI thread.  Allocate a RefPtr
  // on the heap to ensure that the SingleFrameCodec remains alive until the
  // decoder callback is invoked on the UI thread.  The callback can then
  // drop the reference.
  fml::RefPtr<SingleFrameCodec>* raw_codec_ref =
      new fml::RefPtr<SingleFrameCodec>(this);

  decoder->Decode(
      descriptor_, target_width_, target_height_,
      [raw_codec_ref](auto image, auto decode_error) {
        std::unique_ptr<fml::RefPtr<SingleFrameCodec>> codec_ref(raw_codec_ref);
        fml::RefPtr<SingleFrameCodec> codec(std::move(*codec_ref));

        auto state = codec->pending_callbacks_.front().dart_state().lock();

        if (!state) {
          // This is probably because the isolate has been terminated before the
          // image could be decoded.

          return;
        }

        tonic::DartState::Scope scope(state.get());

        if (image) {
          auto canvas_image = fml::MakeRefCounted<CanvasImage>();
          canvas_image->set_image(std::move(image));

          codec->cached_image_ = std::move(canvas_image);
        }

        // The cached frame is now available and should be returned to any
        // future callers.
        codec->status_ = Status::kComplete;

        // Invoke any callbacks that were provided before the frame was decoded.
        for (const tonic::DartPersistentValue& callback :
             codec->pending_callbacks_) {
          tonic::DartInvoke(callback.value(),
                            {tonic::ToDart(codec->cached_image_),
                             tonic::ToDart(0), tonic::ToDart(decode_error)});
        }
        codec->pending_callbacks_.clear();
      });

  // The encoded data is no longer needed now that it has been handed off
  // to the decoder.
  descriptor_ = nullptr;

  status_ = Status::kInProgress;

  return Dart_Null();
}

}  // namespace flutter
