// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_MULTI_FRAME_CODEC_H_
#define FLUTTER_LIB_UI_PAINTING_MULTI_FRAME_CODEC_H_

#include "flutter/fml/macros.h"
#include "flutter/lib/ui/painting/codec.h"
#include "flutter/lib/ui/painting/image_generator.h"

#include <utility>

namespace flutter {

class MultiFrameCodec : public Codec {
 public:
  explicit MultiFrameCodec(std::shared_ptr<ImageGenerator> generator);

  ~MultiFrameCodec() override;

  // |Codec|
  int frameCount() const override;

  // |Codec|
  int repetitionCount() const override;

  // |Codec|
  Dart_Handle getNextFrame(Dart_Handle args) override;

 private:
  // Captures the state shared between the IO and UI task runners.
  //
  // The state is initialized on the UI task runner when the Dart object is
  // created. Decoding occurs on the IO task runner. Since it is possible for
  // the UI object to be collected independently of the IO task runner work,
  // it is not safe for this state to live directly on the MultiFrameCodec.
  // Instead, the MultiFrameCodec creates this object when it is constructed,
  // shares it with the IO task runner's decoding work, and sets the live_
  // member to false when it is destructed.
  struct State {
    explicit State(std::shared_ptr<ImageGenerator> generator);

    const std::shared_ptr<ImageGenerator> generator_;
    const int frameCount_;
    const int repetitionCount_;
    bool is_impeller_enabled_ = false;

    // The non-const members and functions below here are only read or written
    // to on the IO thread. They are not safe to access or write on the UI
    // thread.
    int nextFrameIndex_ = 0;
    // The last decoded frame that's required to decode any subsequent frames.
    std::optional<SkBitmap> lastRequiredFrame_;
    // The index of the last decoded required frame.
    int lastRequiredFrameIndex_ = -1;

    // The rectangle that should be cleared if the previous frame's disposal
    // method was kRestoreBGColor.
    std::optional<SkIRect> restoreBGColorRect_;

    std::pair<sk_sp<DlImage>, std::string> GetNextFrameImage(
        const fml::WeakPtr<GrDirectContext>& resourceContext,
        const std::shared_ptr<const fml::SyncSwitch>& gpu_disable_sync_switch,
        const std::shared_ptr<impeller::Context>& impeller_context,
        const fml::RefPtr<flutter::SkiaUnrefQueue>& unref_queue);

    void GetNextFrameAndInvokeCallback(
        std::unique_ptr<tonic::DartPersistentValue> callback,
        const fml::RefPtr<fml::TaskRunner>& ui_task_runner,
        const fml::WeakPtr<GrDirectContext>& resourceContext,
        const fml::RefPtr<flutter::SkiaUnrefQueue>& unref_queue,
        const std::shared_ptr<const fml::SyncSwitch>& gpu_disable_sync_switch,
        size_t trace_id,
        const std::shared_ptr<impeller::Context>& impeller_context);
  };

  // Shared across the UI and IO task runners.
  std::shared_ptr<State> state_;

  FML_FRIEND_MAKE_REF_COUNTED(MultiFrameCodec);
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(MultiFrameCodec);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_MULTI_FRAME_CODEC_H_
