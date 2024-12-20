// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_GENERATOR_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_GENERATOR_H_

#include <optional>
#include "flutter/fml/macros.h"
#include "third_party/skia/include/codec/SkCodec.h"
#include "third_party/skia/include/codec/SkCodecAnimation.h"
#include "third_party/skia/include/core/SkData.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkImageGenerator.h"
#include "third_party/skia/include/core/SkImageInfo.h"
#include "third_party/skia/include/core/SkSize.h"

namespace flutter {

/// @brief  The minimal interface necessary for defining a decoder that can be
///         used for both single and multi-frame image decoding. Image
///         generators can also optionally support decoding into a subscaled
///         buffer. Implementers of `ImageGenerator` regularly keep internal
///         state which is not thread safe, and so aliasing and parallel access
///         should never be done with `ImageGenerator`s.
/// @see    `ImageGenerator::GetScaledDimensions`
class ImageGenerator {
 public:
  /// Frame count value to denote infinite looping.
  const static unsigned int kInfinitePlayCount =
      std::numeric_limits<unsigned int>::max();

  /// @brief  Info about a single frame in the context of a multi-frame image,
  ///         useful for animation and blending.
  struct FrameInfo {
    /// The frame index of the frame that, if any, this frame needs to be
    /// blended with.
    std::optional<unsigned int> required_frame;

    /// Number of milliseconds to show this frame. 0 means only show it for one
    /// frame.
    unsigned int duration;

    /// How this frame should be modified before decoding the next one.
    SkCodecAnimation::DisposalMethod disposal_method;

    /// The region of the frame that is affected by the disposal method.
    std::optional<SkIRect> disposal_rect;

    /// How this frame should be blended with the previous frame.
    SkCodecAnimation::Blend blend_mode;
  };

  virtual ~ImageGenerator();

  /// @brief   Returns basic information about the contents of the encoded
  ///          image. This information can almost always be collected by just
  ///          interpreting the header of a decoded image.
  /// @return  Size and color information describing the image.
  /// @note    This method is executed on the UI thread and used for layout
  ///          purposes by the framework, and so this method should not perform
  ///          long synchronous tasks.
  virtual const SkImageInfo& GetInfo() = 0;

  /// @brief   Get the number of frames that the encoded image stores. This
  ///          method is always expected to be called before `GetFrameInfo`, as
  ///          the underlying image decoder may interpret frame information that
  ///          is then used when calling `GetFrameInfo`.
  /// @return  The number of frames that the encoded image stores. This will
  ///          always be 1 for single-frame images.
  virtual unsigned int GetFrameCount() const = 0;

  /// @brief  The number of times an animated image should play through before
  ///         playback stops.
  /// @return If this image is animated, the number of times the animation
  ///         should play through is returned, otherwise it'll just return 1.
  ///         If the animation should loop forever, `kInfinitePlayCount` is
  ///         returned.
  virtual unsigned int GetPlayCount() const = 0;

  /// @brief      Get information about a single frame in the context of a
  ///             multi-frame image, useful for animation and frame blending.
  ///             This method should only ever be called after `GetFrameCount`
  ///             has been called. This information is nonsensical for
  ///             single-frame images.
  /// @param[in]  frame_index  The index of the frame to get information about.
  /// @return     Information about the given frame. If the image is
  ///             single-frame, a default result is returned.
  /// @see        `GetFrameCount`
  virtual const FrameInfo GetFrameInfo(unsigned int frame_index) = 0;

  /// @brief      Given a scale value, find the closest image size that can be
  ///             used for efficiently decoding the image. If subpixel image
  ///             decoding is not supported by the decoder, this method should
  ///             just return the original image size.
  /// @param[in]  scale  The desired scale factor of the image for decoding.
  /// @return     The closest image size that can be used for efficiently
  ///             decoding the image.
  /// @note       This method is called prior to `GetPixels` in order to query
  ///             for supported sizes.
  /// @see        `GetPixels`
  virtual SkISize GetScaledDimensions(float scale) = 0;

  /// @brief      Decode the image into a given buffer. This method is currently
  ///             always used for sub-pixel image decoding. For full-sized still
  ///             images, `GetImage` is always attempted first.
  /// @param[in]  info         The desired size and color info of the decoded
  ///                          image to be returned. The implementation of
  ///                          `GetScaledDimensions` determines which sizes are
  ///                          supported by the image decoder.
  /// @param[in]  pixels       The location where the raw decoded image data
  ///                          should be written.
  /// @param[in]  row_bytes    The total number of bytes that should make up a
  ///                          single row of decoded image data
  ///                          (i.e. width * bytes_per_pixel).
  /// @param[in]  frame_index  Which frame to decode. This is only useful for
  ///                          multi-frame images.
  /// @param[in]  prior_frame  Optional frame index parameter for multi-frame
  ///                          images which specifies the previous frame that
  ///                          should be use for blending. This hints to the
  ///                          decoder that it should use a previously cached
  ///                          frame instead of decoding dependency frame(s).
  ///                          If an empty value is supplied, the decoder should
  ///                          decode any necessary frames first.
  /// @return     True if the image was successfully decoded.
  /// @note       This method performs potentially long synchronous work, and so
  ///             it should never be executed on the UI thread. Image decoders
  ///             do not require GPU acceleration, and so threads without a GPU
  ///             context may also be used.
  /// @see        `GetScaledDimensions`
  virtual bool GetPixels(
      const SkImageInfo& info,
      void* pixels,
      size_t row_bytes,
      unsigned int frame_index = 0,
      std::optional<unsigned int> prior_frame = std::nullopt) = 0;

  /// @brief   Creates an `SkImage` based on the current `ImageInfo` of this
  ///          `ImageGenerator`.
  /// @return  A new `SkImage` containing the decoded image data.
  sk_sp<SkImage> GetImage();
};

class BuiltinSkiaImageGenerator : public ImageGenerator {
 public:
  ~BuiltinSkiaImageGenerator();

  explicit BuiltinSkiaImageGenerator(
      std::unique_ptr<SkImageGenerator> generator);

  // |ImageGenerator|
  const SkImageInfo& GetInfo() override;

  // |ImageGenerator|
  unsigned int GetFrameCount() const override;

  // |ImageGenerator|
  unsigned int GetPlayCount() const override;

  // |ImageGenerator|
  const ImageGenerator::FrameInfo GetFrameInfo(
      unsigned int frame_index) override;

  // |ImageGenerator|
  SkISize GetScaledDimensions(float desired_scale) override;

  // |ImageGenerator|
  bool GetPixels(
      const SkImageInfo& info,
      void* pixels,
      size_t row_bytes,
      unsigned int frame_index = 0,
      std::optional<unsigned int> prior_frame = std::nullopt) override;

  static std::unique_ptr<ImageGenerator> MakeFromGenerator(
      std::unique_ptr<SkImageGenerator> generator);

 private:
  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(BuiltinSkiaImageGenerator);
  std::unique_ptr<SkImageGenerator> generator_;
};

class BuiltinSkiaCodecImageGenerator : public ImageGenerator {
 public:
  ~BuiltinSkiaCodecImageGenerator();

  explicit BuiltinSkiaCodecImageGenerator(std::unique_ptr<SkCodec> codec);

  explicit BuiltinSkiaCodecImageGenerator(sk_sp<SkData> buffer);

  // |ImageGenerator|
  const SkImageInfo& GetInfo() override;

  // |ImageGenerator|
  unsigned int GetFrameCount() const override;

  // |ImageGenerator|
  unsigned int GetPlayCount() const override;

  // |ImageGenerator|
  const ImageGenerator::FrameInfo GetFrameInfo(
      unsigned int frame_index) override;

  // |ImageGenerator|
  SkISize GetScaledDimensions(float desired_scale) override;

  // |ImageGenerator|
  bool GetPixels(
      const SkImageInfo& info,
      void* pixels,
      size_t row_bytes,
      unsigned int frame_index = 0,
      std::optional<unsigned int> prior_frame = std::nullopt) override;

  static std::unique_ptr<ImageGenerator> MakeFromData(sk_sp<SkData> data);

 private:
  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(BuiltinSkiaCodecImageGenerator);
  std::unique_ptr<SkCodec> codec_;
  SkImageInfo image_info_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_GENERATOR_H_
