// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/multi_frame_codec.h"

#include <utility>

#include "flutter/fml/make_copyable.h"
#include "flutter/lib/ui/painting/display_list_image_gpu.h"
#include "flutter/lib/ui/painting/image.h"
#if IMPELLER_SUPPORTS_RENDERING
#include "flutter/lib/ui/painting/image_decoder_impeller.h"
#endif  // IMPELLER_SUPPORTS_RENDERING
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/skia/include/codec/SkCodecAnimation.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkPixelRef.h"
#include "third_party/skia/include/gpu/ganesh/SkImageGanesh.h"
#include "third_party/tonic/logging/dart_invoke.h"

namespace flutter {

MultiFrameCodec::MultiFrameCodec(std::shared_ptr<ImageGenerator> generator)
    : state_(new State(std::move(generator))) {}

MultiFrameCodec::~MultiFrameCodec() = default;

MultiFrameCodec::State::State(std::shared_ptr<ImageGenerator> generator)
    : generator_(std::move(generator)),
      frameCount_(generator_->GetFrameCount()),
      repetitionCount_(generator_->GetPlayCount() ==
                               ImageGenerator::kInfinitePlayCount
                           ? -1
                           : generator_->GetPlayCount() - 1),
      is_impeller_enabled_(UIDartState::Current()->IsImpellerEnabled()) {}

static void InvokeNextFrameCallback(
    const fml::RefPtr<CanvasImage>& image,
    int duration,
    const std::string& decode_error,
    std::unique_ptr<tonic::DartPersistentValue> callback,
    size_t trace_id) {
  std::shared_ptr<tonic::DartState> dart_state = callback->dart_state().lock();
  if (!dart_state) {
    FML_DLOG(ERROR) << "Could not acquire Dart state while attempting to fire "
                       "next frame callback.";
    return;
  }
  tonic::DartState::Scope scope(dart_state);
  tonic::DartInvoke(callback->value(),
                    {tonic::ToDart(image), tonic::ToDart(duration),
                     tonic::ToDart(decode_error)});
}

std::pair<sk_sp<DlImage>, std::string>
MultiFrameCodec::State::GetNextFrameImage(
    fml::WeakPtr<GrDirectContext> resourceContext,
    const std::shared_ptr<const fml::SyncSwitch>& gpu_disable_sync_switch,
    const std::shared_ptr<impeller::Context>& impeller_context,
    fml::RefPtr<flutter::SkiaUnrefQueue> unref_queue) {
  SkBitmap bitmap = SkBitmap();
  SkImageInfo info = generator_->GetInfo().makeColorType(kN32_SkColorType);
  if (info.alphaType() == kUnpremul_SkAlphaType) {
    SkImageInfo updated = info.makeAlphaType(kPremul_SkAlphaType);
    info = updated;
  }
  if (!bitmap.tryAllocPixels(info)) {
    std::ostringstream ostr;
    ostr << "Failed to allocate memory for bitmap of size "
         << info.computeMinByteSize() << "B";
    std::string decode_error = ostr.str();
    FML_LOG(ERROR) << decode_error;
    return std::make_pair(nullptr, decode_error);
  }

  ImageGenerator::FrameInfo frameInfo =
      generator_->GetFrameInfo(nextFrameIndex_);

  const int requiredFrameIndex =
      frameInfo.required_frame.value_or(SkCodec::kNoFrame);

  if (requiredFrameIndex != SkCodec::kNoFrame) {
    // We are here when the frame said |disposal_method| is
    // `DisposalMethod::kKeep` or `DisposalMethod::kRestorePrevious` and
    // |requiredFrameIndex| is set to ex-frame or ex-ex-frame.
    if (!lastRequiredFrame_.has_value()) {
      FML_DLOG(INFO)
          << "Frame " << nextFrameIndex_ << " depends on frame "
          << requiredFrameIndex
          << " and no required frames are cached. Using blank slate instead.";
    } else {
      // Copy the previous frame's output buffer into the current frame as the
      // starting point.
      bitmap.writePixels(lastRequiredFrame_->pixmap());
      if (restoreBGColorRect_.has_value()) {
        bitmap.erase(SK_ColorTRANSPARENT, restoreBGColorRect_.value());
      }
    }
  }

  // Write the new frame to the output buffer. The bitmap pixels as supplied
  // are already set in accordance with the previous frame's disposal policy.
  if (!generator_->GetPixels(info, bitmap.getPixels(), bitmap.rowBytes(),
                             nextFrameIndex_, requiredFrameIndex)) {
    std::ostringstream ostr;
    ostr << "Could not getPixels for frame " << nextFrameIndex_;
    std::string decode_error = ostr.str();
    FML_LOG(ERROR) << decode_error;
    return std::make_pair(nullptr, decode_error);
  }

  const bool keep_current_frame =
      frameInfo.disposal_method == SkCodecAnimation::DisposalMethod::kKeep;
  const bool restore_previous_frame =
      frameInfo.disposal_method ==
      SkCodecAnimation::DisposalMethod::kRestorePrevious;
  const bool previous_frame_available = lastRequiredFrame_.has_value();

  // Store the current frame in `lastRequiredFrame_` if the frame's disposal
  // method indicates we should do so.
  // * When the disposal method is "Keep", the stored frame should always be
  //   overwritten with the new frame we just crafted.
  // * When the disposal method is "RestorePrevious", the previously stored
  //   frame should be retained and used as the backdrop for the next frame
  //   again. If there isn't already a stored frame, that means we haven't
  //   rendered any frames yet! When this happens, we just fall back to "Keep"
  //   behavior and store the current frame as the backdrop of the next frame.

  if (keep_current_frame ||
      (previous_frame_available && !restore_previous_frame)) {
    // Replace the stored frame. The `lastRequiredFrame_` will get used as the
    // starting backdrop for the next frame.
    lastRequiredFrame_ = bitmap;
    lastRequiredFrameIndex_ = nextFrameIndex_;
  }

  if (frameInfo.disposal_method ==
      SkCodecAnimation::DisposalMethod::kRestoreBGColor) {
    restoreBGColorRect_ = frameInfo.disposal_rect;
  } else {
    restoreBGColorRect_.reset();
  }

#if IMPELLER_SUPPORTS_RENDERING
  if (is_impeller_enabled_) {
    // This is safe regardless of whether the GPU is available or not because
    // without mipmap creation there is no command buffer encoding done.
    return ImageDecoderImpeller::UploadTextureToStorage(
        impeller_context, std::make_shared<SkBitmap>(bitmap));
  }
#endif  // IMPELLER_SUPPORTS_RENDERING

#if !SLIMPELLER
  sk_sp<SkImage> skImage;
  gpu_disable_sync_switch->Execute(
      fml::SyncSwitch::Handlers()
          .SetIfTrue([&skImage, &bitmap] {
            // Defer decoding until time of draw later on the raster thread.
            // Can happen when GL operations are currently forbidden such as
            // in the background on iOS.
            skImage = SkImages::RasterFromBitmap(bitmap);
          })
          .SetIfFalse([&skImage, &resourceContext, &bitmap] {
            if (resourceContext) {
              SkPixmap pixmap(bitmap.info(), bitmap.pixelRef()->pixels(),
                              bitmap.pixelRef()->rowBytes());
              skImage = SkImages::CrossContextTextureFromPixmap(
                  resourceContext.get(), pixmap, true);
            } else {
              // Defer decoding until time of draw later on the raster thread.
              // Can happen when GL operations are currently forbidden such as
              // in the background on iOS.
              skImage = SkImages::RasterFromBitmap(bitmap);
            }
          }));

  return std::make_pair(DlImageGPU::Make({skImage, std::move(unref_queue)}),
                        std::string());
#else   //  !SLIMPELLER
  return std::make_pair(nullptr, "Unsupported backend.");
#endif  //  !SLIMPELLER
}

void MultiFrameCodec::State::GetNextFrameAndInvokeCallback(
    std::unique_ptr<tonic::DartPersistentValue> callback,
    const fml::RefPtr<fml::TaskRunner>& ui_task_runner,
    fml::WeakPtr<GrDirectContext> resourceContext,
    fml::RefPtr<flutter::SkiaUnrefQueue> unref_queue,
    const std::shared_ptr<const fml::SyncSwitch>& gpu_disable_sync_switch,
    size_t trace_id,
    const std::shared_ptr<impeller::Context>& impeller_context) {
#if FML_OS_IOS_SIMULATOR
  // Noop backend.
  if (!resourceContext && !impeller_context) {
    ui_task_runner->PostTask(
        fml::MakeCopyable([callback = std::move(callback)]() {
          // must be destroyed on UI thread.
        }));
    return;
  }
#endif  // FML_OS_IOS_SIMULATOR

  fml::RefPtr<CanvasImage> image = nullptr;
  int duration = 0;
  sk_sp<DlImage> dlImage;
  std::string decode_error;
  std::tie(dlImage, decode_error) =
      GetNextFrameImage(std::move(resourceContext), gpu_disable_sync_switch,
                        impeller_context, std::move(unref_queue));
  if (dlImage) {
    image = CanvasImage::Create();
    image->set_image(dlImage);
    ImageGenerator::FrameInfo frameInfo =
        generator_->GetFrameInfo(nextFrameIndex_);
    duration = frameInfo.duration;
  }
  nextFrameIndex_ = (nextFrameIndex_ + 1) % frameCount_;

  // The static leak checker gets confused by the use of fml::MakeCopyable.
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
  ui_task_runner->PostTask(fml::MakeCopyable(
      [callback = std::move(callback), image = std::move(image),
       decode_error = std::move(decode_error), duration, trace_id]() mutable {
        InvokeNextFrameCallback(image, duration, decode_error,
                                std::move(callback), trace_id);
      }));
}

Dart_Handle MultiFrameCodec::getNextFrame(Dart_Handle callback_handle) {
  static size_t trace_counter = 1;
  const size_t trace_id = trace_counter++;

  if (!Dart_IsClosure(callback_handle)) {
    return tonic::ToDart("Callback must be a function");
  }

  auto* dart_state = UIDartState::Current();

  const auto& task_runners = dart_state->GetTaskRunners();

  if (state_->frameCount_ == 0) {
    std::string decode_error("Could not provide any frame.");
    FML_LOG(ERROR) << decode_error;
    task_runners.GetUITaskRunner()->PostTask(fml::MakeCopyable(
        [trace_id, decode_error = std::move(decode_error),
         callback = std::make_unique<tonic::DartPersistentValue>(
             tonic::DartState::Current(), callback_handle)]() mutable {
          InvokeNextFrameCallback(nullptr, 0, decode_error, std::move(callback),
                                  trace_id);
        }));
    return Dart_Null();
  }

  task_runners.GetIOTaskRunner()->PostTask(fml::MakeCopyable(
      [callback = std::make_unique<tonic::DartPersistentValue>(
           tonic::DartState::Current(), callback_handle),
       weak_state = std::weak_ptr<MultiFrameCodec::State>(state_), trace_id,
       ui_task_runner = task_runners.GetUITaskRunner(),
       io_manager = dart_state->GetIOManager()]() mutable {
        auto state = weak_state.lock();
        if (!state) {
          ui_task_runner->PostTask(fml::MakeCopyable(
              [callback = std::move(callback)]() { callback->Clear(); }));
          return;
        }
        state->GetNextFrameAndInvokeCallback(
            std::move(callback), ui_task_runner,
            io_manager->GetResourceContext(), io_manager->GetSkiaUnrefQueue(),
            io_manager->GetIsGpuDisabledSyncSwitch(), trace_id,
            io_manager->GetImpellerContext());
      }));

  return Dart_Null();
  // The static leak checker gets confused by the control flow, unique
  // pointers and closures in this function.
  // NOLINTNEXTLINE(clang-analyzer-cplusplus.NewDeleteLeaks)
}

int MultiFrameCodec::frameCount() const {
  return state_->frameCount_;
}

int MultiFrameCodec::repetitionCount() const {
  return state_->repetitionCount_;
}

}  // namespace flutter
