// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "image_generator_apng.h"
#include <cstddef>
#include <cstring>

#include "flutter/fml/logging.h"
#include "third_party/skia/include/codec/SkCodec.h"
#include "third_party/skia/include/codec/SkCodecAnimation.h"
#include "third_party/skia/include/core/SkAlphaType.h"
#include "third_party/skia/include/core/SkColorType.h"
#include "third_party/skia/include/core/SkImageInfo.h"
#include "third_party/skia/include/core/SkStream.h"
#include "third_party/zlib/zlib.h"  // For crc32

namespace flutter {

APNGImageGenerator::~APNGImageGenerator() = default;

APNGImageGenerator::APNGImageGenerator(sk_sp<SkData>& data,
                                       SkImageInfo& image_info,
                                       APNGImage&& default_image,
                                       unsigned int frame_count,
                                       unsigned int play_count,
                                       const void* next_chunk_p,
                                       const std::vector<uint8_t>& header)
    : data_(data),
      image_info_(image_info),
      frame_count_(frame_count),
      play_count_(play_count),
      first_frame_index_(default_image.frame_info.has_value() ? 0 : 1),
      next_chunk_p_(next_chunk_p),
      header_(header) {
  images_.push_back(std::move(default_image));
}

const SkImageInfo& APNGImageGenerator::GetInfo() {
  return image_info_;
}

unsigned int APNGImageGenerator::GetFrameCount() const {
  return frame_count_;
}

unsigned int APNGImageGenerator::GetPlayCount() const {
  return frame_count_ > 1 ? play_count_ : 1;
}

const ImageGenerator::FrameInfo APNGImageGenerator::GetFrameInfo(
    unsigned int frame_index) {
  unsigned int image_index = first_frame_index_ + frame_index;
  if (!DemuxToImageIndex(image_index)) {
    return {};
  }

  auto frame_info = images_[image_index].frame_info;
  if (frame_info.has_value()) {
    return frame_info.value();
  }
  return {};
}

SkISize APNGImageGenerator::GetScaledDimensions(float desired_scale) {
  return image_info_.dimensions();
}

bool APNGImageGenerator::GetPixels(const SkImageInfo& info,
                                   void* pixels,
                                   size_t row_bytes,
                                   unsigned int frame_index,
                                   std::optional<unsigned int> prior_frame) {
  FML_DCHECK(images_.size() > 0);
  unsigned int image_index = first_frame_index_ + frame_index;

  //----------------------------------------------------------------------------
  /// 1. Demux the frame from the APNG stream.
  ///

  if (!DemuxToImageIndex(image_index)) {
    FML_DLOG(ERROR) << "Couldn't demux image at index " << image_index
                    << " (frame index: " << frame_index
                    << ") from APNG stream.";
    return RenderDefaultImage(info, pixels, row_bytes);
  }

  //----------------------------------------------------------------------------
  /// 2. Decode the frame.
  ///

  APNGImage& frame = images_[image_index];
  SkImageInfo frame_info = frame.codec->getInfo();
  auto frame_row_bytes = frame_info.bytesPerPixel() * frame_info.width();

  if (frame.pixels.empty()) {
    frame.pixels.resize(frame_row_bytes * frame_info.height());
    SkCodec::Result result = frame.codec->getPixels(
        frame.codec->getInfo(), frame.pixels.data(), frame_row_bytes);
    if (result != SkCodec::kSuccess) {
      FML_DLOG(ERROR) << "Failed to decode image at index " << image_index
                      << " (frame index: " << frame_index
                      << ") of APNG. SkCodec::Result: " << result;
      return RenderDefaultImage(info, pixels, row_bytes);
    }
  }
  if (!frame.frame_info.has_value()) {
    FML_DLOG(ERROR) << "Failed to decode image at index " << image_index
                    << " (frame index: " << frame_index
                    << ") of APNG due to the frame missing data (frame_info).";
    return false;
  }
  if (
      // Check for unsigned integer wrapping for
      // frame.{x|y}_offset + frame_info.{width|height}().
      frame.x_offset >
          std::numeric_limits<uint32_t>::max() - frame_info.width() ||
      frame.y_offset >
          std::numeric_limits<uint32_t>::max() - frame_info.height() ||

      frame.x_offset + frame_info.width() >
          static_cast<unsigned int>(info.width()) ||
      frame.y_offset + frame_info.height() >
          static_cast<unsigned int>(info.height())) {
    FML_DLOG(ERROR)
        << "Decoded image at index " << image_index
        << " (frame index: " << frame_index
        << ") rejected because the destination region (x: " << frame.x_offset
        << ", y: " << frame.y_offset << ", width: " << frame_info.width()
        << ", height: " << frame_info.height()
        << ") is not entirely within the destination surface (width: "
        << info.width() << ", height: " << info.height() << ").";
    return false;
  }

  //----------------------------------------------------------------------------
  /// 3. Composite the frame onto the canvas.
  ///

  if (info.colorType() != kN32_SkColorType) {
    FML_DLOG(ERROR) << "Failed to composite image at index " << image_index
                    << " (frame index: " << frame_index
                    << ") of APNG due to the destination surface having an "
                       "unsupported color type.";
    return false;
  }
  if (frame_info.colorType() != kN32_SkColorType) {
    FML_DLOG(ERROR)
        << "Failed to composite image at index " << image_index
        << " (frame index: " << frame_index
        << ") of APNG due to the frame having an unsupported color type.";
    return false;
  }

  // Regardless of the byte order (RGBA vs BGRA), the blending operations are
  // the same.
  struct Pixel {
    uint8_t channel[4];

    uint8_t GetAlpha() { return channel[3]; }

    void Premultiply() {
      for (int i = 0; i < 3; i++) {
        channel[i] = channel[i] * GetAlpha() / 0xFF;
      }
    }

    void Unpremultiply() {
      if (GetAlpha() == 0) {
        channel[0] = channel[1] = channel[2] = 0;
        return;
      }
      for (int i = 0; i < 3; i++) {
        channel[i] = channel[i] * 0xFF / GetAlpha();
      }
    }
  };

  FML_DCHECK(frame_info.bytesPerPixel() == sizeof(Pixel));

  bool result = true;

  if (frame.frame_info->blend_mode == SkCodecAnimation::Blend::kSrc) {
    SkPixmap src_pixmap(frame_info, frame.pixels.data(), frame_row_bytes);
    uint8_t* dst_pixels = static_cast<uint8_t*>(pixels) +
                          frame.y_offset * row_bytes +
                          frame.x_offset * frame_info.bytesPerPixel();
    result = src_pixmap.readPixels(info, dst_pixels, row_bytes);
    if (!result) {
      FML_DLOG(ERROR) << "Failed to copy pixels at index " << image_index
                      << " (frame index: " << frame_index << ") of APNG.";
    }
  } else if (frame.frame_info->blend_mode ==
             SkCodecAnimation::Blend::kSrcOver) {
    for (int y = 0; y < frame_info.height(); y++) {
      auto src_row = frame.pixels.data() + y * frame_row_bytes;
      auto dst_row = static_cast<uint8_t*>(pixels) +
                     (y + frame.y_offset) * row_bytes +
                     frame.x_offset * frame_info.bytesPerPixel();

      for (int x = 0; x < frame_info.width(); x++) {
        auto x_offset_bytes = x * frame_info.bytesPerPixel();

        Pixel src = *reinterpret_cast<Pixel*>(src_row + x_offset_bytes);
        Pixel* dst_p = reinterpret_cast<Pixel*>(dst_row + x_offset_bytes);
        Pixel dst = *dst_p;

        // Ensure both colors are premultiplied for the blending operation.
        if (info.alphaType() == kUnpremul_SkAlphaType) {
          dst.Premultiply();
        }
        if (frame_info.alphaType() == kUnpremul_SkAlphaType) {
          src.Premultiply();
        }

        for (int i = 0; i < 4; i++) {
          dst.channel[i] =
              src.channel[i] + dst.channel[i] * (0xFF - src.GetAlpha()) / 0xFF;
        }

        // The final color is premultiplied. Unpremultiply to match the
        // backdrop surface if necessary.
        if (info.alphaType() == kUnpremul_SkAlphaType) {
          dst.Unpremultiply();
        }

        *dst_p = dst;
      }
    }
  }

  return result;
}

std::unique_ptr<ImageGenerator> APNGImageGenerator::MakeFromData(
    sk_sp<SkData> data) {
  // Ensure the buffer is large enough to at least contain the PNG signature
  // and a chunk header.
  if (data->size() < sizeof(kPngSignature) + sizeof(ChunkHeader)) {
    return nullptr;
  }
  // Validate the full PNG signature.
  const uint8_t* data_p = static_cast<const uint8_t*>(data.get()->data());
  if (memcmp(data_p, kPngSignature, sizeof(kPngSignature))) {
    return nullptr;
  }

  // Validate the header chunk.
  const ChunkHeader* chunk = reinterpret_cast<const ChunkHeader*>(data_p + 8);
  if (!IsValidChunkHeader(data_p, data->size(), chunk) ||
      chunk->get_data_length() != sizeof(ImageHeaderChunkData) ||
      chunk->get_type() != kImageHeaderChunkType) {
    return nullptr;
  }

  // Walk the chunks to find the "animation control" chunk. If an "image data"
  // chunk is found first, this PNG is not animated.
  while (true) {
    chunk = GetNextChunk(data_p, data->size(), chunk);

    if (chunk == nullptr) {
      return nullptr;
    }
    if (chunk->get_type() == kImageDataChunkType) {
      return nullptr;
    }
    if (chunk->get_type() == kAnimationControlChunkType) {
      break;
    }
  }

  const AnimationControlChunkData* animation_data =
      CastChunkData<AnimationControlChunkData>(chunk);

  // Extract the header signature and chunks to prepend when demuxing images.
  std::optional<std::vector<uint8_t>> header;
  const void* first_chunk_p;
  std::tie(header, first_chunk_p) = ExtractHeader(data_p, data->size());
  if (!header.has_value()) {
    return nullptr;
  }

  // Demux the first image in the APNG chunk stream in order to interpret
  // extent and blending info immediately.
  std::optional<APNGImage> default_image;
  const void* next_chunk_p;
  std::tie(default_image, next_chunk_p) =
      DemuxNextImage(data_p, data->size(), header.value(), first_chunk_p);
  if (!default_image.has_value()) {
    return nullptr;
  }

  unsigned int play_count = animation_data->get_num_plays();
  if (play_count == 0) {
    play_count = kInfinitePlayCount;
  }

  SkImageInfo image_info = default_image.value().codec->getInfo();
  return std::unique_ptr<APNGImageGenerator>(
      new APNGImageGenerator(data, image_info, std::move(default_image.value()),
                             animation_data->get_num_frames(), play_count,
                             next_chunk_p, header.value()));
}

bool APNGImageGenerator::IsValidChunkHeader(const void* buffer,
                                            size_t size,
                                            const ChunkHeader* chunk) {
  // Ensure the chunk doesn't start before the beginning of the buffer.
  if (reinterpret_cast<const uint8_t*>(chunk) <
      static_cast<const uint8_t*>(buffer)) {
    return false;
  }

  // Ensure the buffer is large enough to contain at least the chunk header.
  if (reinterpret_cast<const uint8_t*>(chunk) + sizeof(ChunkHeader) >
      static_cast<const uint8_t*>(buffer) + size) {
    return false;
  }

  // Ensure the buffer is large enough to contain the chunk's given data size
  // and CRC.
  const uint8_t* chunk_end =
      reinterpret_cast<const uint8_t*>(chunk) + GetChunkSize(chunk);
  if (chunk_end > static_cast<const uint8_t*>(buffer) + size) {
    return false;
  }

  // Ensure the 4-byte type only contains ISO 646 letters.
  uint32_t type = chunk->get_type();
  for (int i = 0; i < 4; i++) {
    uint8_t c = type >> i * 8 & 0xFF;
    if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z'))) {
      return false;
    }
  }

  return true;
}

const APNGImageGenerator::ChunkHeader* APNGImageGenerator::GetNextChunk(
    const void* buffer,
    size_t size,
    const ChunkHeader* current_chunk) {
  FML_DCHECK((uint8_t*)current_chunk + sizeof(ChunkHeader) <=
             (uint8_t*)buffer + size);

  const ChunkHeader* next_chunk = reinterpret_cast<const ChunkHeader*>(
      reinterpret_cast<const uint8_t*>(current_chunk) +
      GetChunkSize(current_chunk));
  if (!IsValidChunkHeader(buffer, size, next_chunk)) {
    return nullptr;
  }

  return next_chunk;
}

std::pair<std::optional<std::vector<uint8_t>>, const void*>
APNGImageGenerator::ExtractHeader(const void* buffer_p, size_t buffer_size) {
  std::vector<uint8_t> result(sizeof(kPngSignature));
  memcpy(result.data(), kPngSignature, sizeof(kPngSignature));

  const ChunkHeader* chunk = reinterpret_cast<const ChunkHeader*>(
      static_cast<const uint8_t*>(buffer_p) + sizeof(kPngSignature));
  // Validate the first chunk to ensure it's safe to read.
  if (!IsValidChunkHeader(buffer_p, buffer_size, chunk)) {
    return std::make_pair(std::nullopt, nullptr);
  }

  // Walk the chunks and copy in the non-APNG chunks until we come across a
  // frame or image chunk.
  do {
    if (chunk->get_type() != kAnimationControlChunkType) {
      size_t chunk_size = GetChunkSize(chunk);
      result.resize(result.size() + chunk_size);
      memcpy(result.data() + result.size() - chunk_size, chunk, chunk_size);
    }

    chunk = GetNextChunk(buffer_p, buffer_size, chunk);
  } while (chunk != nullptr && chunk->get_type() != kFrameControlChunkType &&
           chunk->get_type() != kImageDataChunkType &&
           chunk->get_type() != kFrameDataChunkType);

  // nullptr means the end of the buffer was reached, which means there's no
  // frame or image data, so just return nothing because the PNG isn't even
  // valid.
  if (chunk == nullptr) {
    return std::make_pair(std::nullopt, nullptr);
  }

  return std::make_pair(result, chunk);
}

std::pair<std::optional<APNGImageGenerator::APNGImage>, const void*>
APNGImageGenerator::DemuxNextImage(const void* buffer_p,
                                   size_t buffer_size,
                                   const std::vector<uint8_t>& header,
                                   const void* chunk_p) {
  const ChunkHeader* chunk = reinterpret_cast<const ChunkHeader*>(chunk_p);
  // Validate the given chunk to ensure it's safe to read.
  if (!IsValidChunkHeader(buffer_p, buffer_size, chunk)) {
    return std::make_pair(std::nullopt, nullptr);
  }

  // Expect frame data to begin at fdAT or IDAT
  if (chunk->get_type() != kFrameControlChunkType &&
      chunk->get_type() != kImageDataChunkType) {
    return std::make_pair(std::nullopt, nullptr);
  }

  APNGImage result;
  const FrameControlChunkData* control_data = nullptr;

  // The presence of an fcTL chunk is optional for the first (default) image
  // of a PNG. Both cases are handled in APNGImage.
  if (chunk->get_type() == kFrameControlChunkType) {
    control_data = CastChunkData<FrameControlChunkData>(chunk);

    ImageGenerator::FrameInfo frame_info;
    switch (control_data->get_blend_op()) {
      case 0:  // APNG_BLEND_OP_SOURCE
        frame_info.blend_mode = SkCodecAnimation::Blend::kSrc;
        break;
      case 1:  // APNG_BLEND_OP_OVER
        frame_info.blend_mode = SkCodecAnimation::Blend::kSrcOver;
        break;
      default:
        return std::make_pair(std::nullopt, nullptr);
    }

    SkIRect frame_rect = SkIRect::MakeXYWH(
        control_data->get_x_offset(), control_data->get_y_offset(),
        control_data->get_width(), control_data->get_height());
    switch (control_data->get_dispose_op()) {
      case 0:  // APNG_DISPOSE_OP_NONE
        frame_info.disposal_method = SkCodecAnimation::DisposalMethod::kKeep;
        break;
      case 1:  // APNG_DISPOSE_OP_BACKGROUND
        frame_info.disposal_method =
            SkCodecAnimation::DisposalMethod::kRestoreBGColor;
        frame_info.disposal_rect = frame_rect;
        break;
      case 2:  // APNG_DISPOSE_OP_PREVIOUS
        frame_info.disposal_method =
            SkCodecAnimation::DisposalMethod::kRestorePrevious;
        break;
      default:
        return std::make_pair(std::nullopt, nullptr);
    }
    uint16_t denominator = control_data->get_delay_den() == 0
                               ? 100
                               : control_data->get_delay_den();
    frame_info.duration =
        static_cast<int>(control_data->get_delay_num() * 1000.f / denominator);

    result.frame_info = frame_info;
    result.x_offset = control_data->get_x_offset();
    result.y_offset = control_data->get_y_offset();
  }

  std::vector<const ChunkHeader*> image_chunks;
  size_t chunk_space = 0;

  // Walk the chunks until the next frame, end chunk, or an invalid chunk is
  // reached, recording the chunks to copy along with their required space.
  // TODO(bdero): Validate that IDAT/fdAT chunks are contiguous.
  // TODO(bdero): Validate the acTL/fcTL/fdAT sequence number ordering.
  do {
    if (chunk->get_type() != kFrameControlChunkType) {
      image_chunks.push_back(chunk);
      chunk_space += GetChunkSize(chunk);

      // fdAT chunks are converted into IDAT chunks when demuxed. The only
      // difference between these chunk types is that fdAT has a 4 byte
      // sequence number prepended to its data, so subtract that space from
      // the buffer.
      if (chunk->get_type() == kFrameDataChunkType) {
        chunk_space -= 4;
      }
    }

    chunk = GetNextChunk(buffer_p, buffer_size, chunk);
  } while (chunk != nullptr && chunk->get_type() != kFrameControlChunkType &&
           chunk->get_type() != kImageTrailerChunkType);

  const uint8_t end_chunk[] = {0,   0,   0,    0,    'I',  'E',
                               'N', 'D', 0xAE, 0x42, 0x60, 0x82};

  // Form a buffer for the new encoded PNG and copy the chunks in.
  sk_sp<SkData> new_png_buffer = SkData::MakeUninitialized(
      header.size() + chunk_space + sizeof(end_chunk));

  {
    uint8_t* write_cursor =
        static_cast<uint8_t*>(new_png_buffer->writable_data());

    // Copy the signature/header chunks
    memcpy(write_cursor, header.data(), header.size());
    // If this is a frame, override the width/height in the IHDR chunk.
    if (control_data) {
      ChunkHeader* ihdr_header =
          reinterpret_cast<ChunkHeader*>(write_cursor + sizeof(kPngSignature));
      ImageHeaderChunkData* ihdr_data = const_cast<ImageHeaderChunkData*>(
          CastChunkData<ImageHeaderChunkData>(ihdr_header));
      ihdr_data->set_width(control_data->get_width());
      ihdr_data->set_height(control_data->get_height());
      ihdr_header->UpdateChunkCrc32();
    }
    write_cursor += header.size();

    // Copy the image data/ancillary chunks.
    for (const ChunkHeader* c : image_chunks) {
      if (c->get_type() == kFrameDataChunkType) {
        // Write a new IDAT chunk header.
        ChunkHeader* write_header =
            reinterpret_cast<ChunkHeader*>(write_cursor);
        write_header->set_data_length(c->get_data_length() - 4);
        write_header->set_type(kImageDataChunkType);
        write_cursor += sizeof(ChunkHeader);

        // Copy all of the data except for the 4 byte sequence number at the
        // beginning of the fdAT data.
        memcpy(write_cursor,
               reinterpret_cast<const uint8_t*>(c) + sizeof(ChunkHeader) + 4,
               write_header->get_data_length());
        write_cursor += write_header->get_data_length();

        // Recompute the chunk CRC.
        write_header->UpdateChunkCrc32();
        write_cursor += 4;
      } else {
        size_t chunk_size = GetChunkSize(c);
        memcpy(write_cursor, c, chunk_size);
        write_cursor += chunk_size;
      }
    }

    // Copy the trailer chunk.
    memcpy(write_cursor, &end_chunk, sizeof(end_chunk));
  }

  SkCodec::Result header_parse_result;
  result.codec = SkCodec::MakeFromStream(SkMemoryStream::Make(new_png_buffer),
                                         &header_parse_result);
  if (header_parse_result != SkCodec::Result::kSuccess) {
    FML_DLOG(ERROR)
        << "Failed to parse image header during APNG demux. SkCodec::Result: "
        << header_parse_result;
    return std::make_pair(std::nullopt, nullptr);
  }

  if (chunk->get_type() == kImageTrailerChunkType) {
    chunk = nullptr;
  }

  return std::make_pair(std::optional<APNGImage>{std::move(result)}, chunk);
}

bool APNGImageGenerator::DemuxNextImageInternal() {
  if (next_chunk_p_ == nullptr) {
    return false;
  }

  std::optional<APNGImage> image;
  const void* data_p = const_cast<void*>(data_.get()->data());
  std::tie(image, next_chunk_p_) =
      DemuxNextImage(data_p, data_->size(), header_, next_chunk_p_);
  if (!image.has_value() || !image->frame_info.has_value()) {
    return false;
  }

  auto last_frame_info = images_.back().frame_info;
  if (!last_frame_info.has_value()) {
    return false;
  }

  if (images_.size() > first_frame_index_ &&
      (last_frame_info->disposal_method ==
           SkCodecAnimation::DisposalMethod::kKeep ||
       last_frame_info->disposal_method ==
           SkCodecAnimation::DisposalMethod::kRestoreBGColor)) {
    // Mark the required frame as the previous frame in all cases.
    image->frame_info->required_frame = images_.size() - 1;
  } else if (images_.size() > (first_frame_index_ + 1) &&
             last_frame_info->disposal_method ==
                 SkCodecAnimation::DisposalMethod::kRestorePrevious) {
    // Mark the required frame as the last previous frame
    // It is not valid if there are 2 or above frames set |disposal_method| to
    // |kRestorePrevious|. But it also works in MultiFrameCodec.
    image->frame_info->required_frame = images_.size() - 2;
  }

  // Calling SkCodec::getInfo at least once prior to decoding is mandatory.
  SkImageInfo info = image.value().codec->getInfo();
  FML_DCHECK(info.colorInfo() == image_info_.colorInfo());

  images_.push_back(std::move(image.value()));

  auto default_info = images_[0].codec->getInfo();
  if (info.colorType() != default_info.colorType()) {
    return false;
  }
  return true;
}

bool APNGImageGenerator::DemuxToImageIndex(unsigned int image_index) {
  // If the requested image doesn't exist yet, demux more frames from the APNG
  // stream.
  if (image_index >= images_.size()) {
    while (DemuxNextImageInternal() && image_index >= images_.size()) {
    }

    if (image_index >= images_.size()) {
      // The chunk stream was exhausted before the image was found.
      return false;
    }
  }

  return true;
}

void APNGImageGenerator::ChunkHeader::UpdateChunkCrc32() {
  uint32_t* crc_p =
      reinterpret_cast<uint32_t*>(reinterpret_cast<uint8_t*>(this) +
                                  sizeof(ChunkHeader) + get_data_length());
  *crc_p = fml::BigEndianToArch(ComputeChunkCrc32());
}

uint32_t APNGImageGenerator::ChunkHeader::ComputeChunkCrc32() {
  // Exclude the length field at the beginning of the chunk header.
  size_t length = sizeof(ChunkHeader) - 4 + get_data_length();
  uint8_t* chunk_data_p = reinterpret_cast<uint8_t*>(this) + 4;
  uint32_t crc = 0;

  // zlib's crc32 can only take 16 bits at a time for the length, but PNG
  // supports a 32 bit chunk length, so looping is necessary here.
  // Note that crc32 is always called at least once, even if the chunk has an
  // empty data section.
  do {
    uint16_t length16 = length;
    if (length16 == 0 && length > 0) {
      length16 = std::numeric_limits<uint16_t>::max();
    }

    crc = crc32(crc, chunk_data_p, length16);
    length -= length16;
    chunk_data_p += length16;
  } while (length > 0);

  return crc;
}

bool APNGImageGenerator::RenderDefaultImage(const SkImageInfo& info,
                                            void* pixels,
                                            size_t row_bytes) {
  APNGImage& frame = images_[0];
  SkImageInfo frame_info = frame.codec->getInfo();
  if (frame_info.width() > info.width() ||
      frame_info.height() > info.height()) {
    FML_DLOG(ERROR)
        << "Default image rejected because the destination region (width: "
        << frame_info.width() << ", height: " << frame_info.height()
        << ") is not entirely within the destination surface (width: "
        << info.width() << ", height: " << info.height() << ").";
    return false;
  }

  SkCodec::Result result = frame.codec->getPixels(info, pixels, row_bytes);
  if (result != SkCodec::kSuccess) {
    FML_DLOG(ERROR) << "Failed to decode the APNG's default/fallback image. "
                       "SkCodec::Result: "
                    << result;
    return false;
  }
  return true;
}

}  // namespace flutter
