// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_IMAGE_GENERATOR_APNG_H_
#define FLUTTER_LIB_UI_PAINTING_IMAGE_GENERATOR_APNG_H_

#include "image_generator.h"

#include "flutter/fml/endianness.h"
#include "flutter/fml/logging.h"

#define PNG_FIELD(T, name)                \
 private:                                 \
  T name;                                 \
                                          \
 public:                                  \
  T get_##name() const {                  \
    return fml::BigEndianToArch<T>(name); \
  }                                       \
  void set_##name(T n) {                  \
    name = fml::BigEndianToArch<T>(n);    \
  }

namespace flutter {

class APNGImageGenerator : public ImageGenerator {
 public:
  ~APNGImageGenerator();

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
  bool GetPixels(const SkImageInfo& info,
                 void* pixels,
                 size_t row_bytes,
                 unsigned int frame_index,
                 std::optional<unsigned int> prior_frame) override;

  static std::unique_ptr<ImageGenerator> MakeFromData(sk_sp<SkData> data);

 private:
  static constexpr uint8_t kPngSignature[8] = {137, 80, 78, 71, 13, 10, 26, 10};
  static constexpr size_t kChunkCrcSize = 4;

  enum ChunkType {
    kImageHeaderChunkType = 'IHDR',
    kAnimationControlChunkType = 'acTL',
    kImageDataChunkType = 'IDAT',
    kFrameControlChunkType = 'fcTL',
    kFrameDataChunkType = 'fdAT',
    kImageTrailerChunkType = 'IEND',
  };

  class __attribute__((packed, aligned(1))) ChunkHeader {
    PNG_FIELD(uint32_t, data_length)
    PNG_FIELD(ChunkType, type)

   public:
    void UpdateChunkCrc32();

   private:
    uint32_t ComputeChunkCrc32();
  };

  class __attribute__((packed, aligned(1))) ImageHeaderChunkData {
    PNG_FIELD(uint32_t, width)
    PNG_FIELD(uint32_t, height)
    PNG_FIELD(uint8_t, bit_depth)
    PNG_FIELD(uint8_t, color_type)
    PNG_FIELD(uint8_t, compression_method)
    PNG_FIELD(uint8_t, filter_method)
    PNG_FIELD(uint8_t, interlace_method)
  };

  class __attribute__((packed, aligned(1))) AnimationControlChunkData {
    PNG_FIELD(uint32_t, num_frames)
    PNG_FIELD(uint32_t, num_plays)
  };

  class __attribute__((packed, aligned(1))) FrameControlChunkData {
    PNG_FIELD(uint32_t, sequence_number)
    PNG_FIELD(uint32_t, width)
    PNG_FIELD(uint32_t, height)
    PNG_FIELD(uint32_t, x_offset)
    PNG_FIELD(uint32_t, y_offset)
    PNG_FIELD(uint16_t, delay_num)
    PNG_FIELD(uint16_t, delay_den)
    PNG_FIELD(uint8_t, dispose_op)
    PNG_FIELD(uint8_t, blend_op)
  };

  /// @brief  The first PNG frame is always the "default" PNG frame. Absence of
  ///         `frame_info` is only possible on the "default" PNG frame.
  ///         Each frame goes through two decoding stages:
  ///         1. Demuxing stage: An individual PNG codec is created for a frame
  ///            while walking through the APNG chunk stream -- this is placed
  ///            in the `codec` field.
  ///         2. Decoding stage: When a frame is requested for the first time,
  ///            the decoded image is requested from the `SkCodec` and then
  ///            (depending on the `frame_info`) composited with a previous
  ///            frame. The final "canvas" frame is placed in the
  ///            `composited_image` field. At this point, the `codec` is freed
  ///            and the `composited_image` is handed to the caller for drawing.
  struct APNGImage {
    std::unique_ptr<SkCodec> codec;

    // The rendered frame pixels.
    std::vector<uint8_t> pixels;

    // Absence of frame info is possible on the "default" image.
    std::optional<ImageGenerator::FrameInfo> frame_info;

    // X offset of this image when composited. Only applicable to frames.
    unsigned int x_offset;

    // Y offset of this image when composited. Only applicable to frames.
    unsigned int y_offset;
  };

  APNGImageGenerator(sk_sp<SkData>& data,
                     SkImageInfo& image_info,
                     APNGImage&& default_image,
                     unsigned int frame_count,
                     unsigned int play_count,
                     const void* next_chunk_p,
                     const std::vector<uint8_t>& header);

  static bool IsValidChunkHeader(const void* buffer,
                                 size_t size,
                                 const ChunkHeader* chunk);

  static const ChunkHeader* GetNextChunk(const void* buffer,
                                         size_t size,
                                         const ChunkHeader* current_chunk);

  /// @brief  This is a utility template for casting a png buffer pointer to a
  ///         chunk header. Its primary purpose is to statically insert runtime
  ///         debug checks that detect invalid decoding behavior.
  template <typename T>
  static constexpr const T* CastChunkData(const ChunkHeader* chunk) {
    if constexpr (std::is_same_v<T, ImageHeaderChunkData>) {
      FML_DCHECK(chunk->get_type() == kImageHeaderChunkType);
    } else if constexpr (std::is_same_v<T, AnimationControlChunkData>) {
      FML_DCHECK(chunk->get_type() == kAnimationControlChunkType);
    } else if constexpr (std::is_same_v<T, FrameControlChunkData>) {
      FML_DCHECK(chunk->get_type() == kFrameControlChunkType);
    } else {
      static_assert(!sizeof(T), "Invalid chunk struct");
    }

    return reinterpret_cast<const T*>(reinterpret_cast<const uint8_t*>(chunk) +
                                      sizeof(ChunkHeader));
  }

  static constexpr size_t GetChunkSize(const ChunkHeader* chunk) {
    return sizeof(ChunkHeader) + chunk->get_data_length() + kChunkCrcSize;
  }

  static constexpr bool IsChunkCopySafe(const ChunkHeader* chunk) {
    // The safe-to-copy bit is the 5th bit of the chunk name's 4th byte. This is
    // the same as checking that the 4th byte is lowercase.
    return (chunk->get_type() & 0x20) != 0;
  }

  /// @brief  Extract a header that's safe to use for both the "default" image
  ///         and individual PNG frames. Strip the animation control chunk.
  static std::pair<std::optional<std::vector<uint8_t>>, const void*>
  ExtractHeader(const void* buffer_p, size_t buffer_size);

  /// @brief  Takes a chunk pointer to a chunk and demuxes/interprets the next
  ///         image in the APNG sequence. It also provides the next `chunk_p`
  ///         to use.
  /// @see    `APNGImage`
  static std::pair<std::optional<APNGImage>, const void*> DemuxNextImage(
      const void* buffer_p,
      size_t buffer_size,
      const std::vector<uint8_t>& header,
      const void* chunk_p);

  bool DemuxNextImageInternal();

  bool DemuxToImageIndex(unsigned int image_index);

  bool RenderDefaultImage(const SkImageInfo& info,
                          void* pixels,
                          size_t row_bytes);

  FML_DISALLOW_COPY_ASSIGN_AND_MOVE(APNGImageGenerator);
  sk_sp<SkData> data_;
  SkImageInfo image_info_;
  unsigned int frame_count_;
  unsigned int play_count_;

  // The first image is always the default image, which may or may not be a
  // frame. All subsequent images are guaranteed to have frame data.
  std::vector<APNGImage> images_;

  unsigned int first_frame_index_;

  const void* next_chunk_p_;
  std::vector<uint8_t> header_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_IMAGE_GENERATOR_APNG_H_
