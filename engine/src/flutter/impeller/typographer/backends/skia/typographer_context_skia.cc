// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/typographer/backends/skia/typographer_context_skia.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include "flutter/fml/logging.h"
#include "flutter/fml/trace_event.h"
#include "fml/closure.h"

#include "impeller/base/validation.h"
#include "impeller/core/allocator.h"
#include "impeller/core/buffer_view.h"
#include "impeller/core/formats.h"
#include "impeller/core/host_buffer.h"
#include "impeller/core/platform.h"
#include "impeller/core/texture_descriptor.h"
#include "impeller/geometry/rect.h"
#include "impeller/geometry/size.h"
#include "impeller/renderer/command_buffer.h"
#include "impeller/renderer/render_pass.h"
#include "impeller/renderer/render_target.h"
#include "impeller/typographer/backends/skia/typeface_skia.h"
#include "impeller/typographer/font_glyph_pair.h"
#include "impeller/typographer/glyph.h"
#include "impeller/typographer/glyph_atlas.h"
#include "impeller/typographer/rectangle_packer.h"
#include "impeller/typographer/typographer_context.h"
#include "include/core/SkColor.h"
#include "include/core/SkImageInfo.h"
#include "include/core/SkPaint.h"
#include "include/core/SkSize.h"

#include "third_party/skia/include/core/SkBitmap.h"
#include "third_party/skia/include/core/SkBlendMode.h"
#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkFont.h"
#include "third_party/skia/include/core/SkSurface.h"

namespace impeller {

constexpr auto kPadding = 2;

namespace {
SkPaint::Cap ToSkiaCap(Cap cap) {
  switch (cap) {
    case Cap::kButt:
      return SkPaint::Cap::kButt_Cap;
    case Cap::kRound:
      return SkPaint::Cap::kRound_Cap;
    case Cap::kSquare:
      return SkPaint::Cap::kSquare_Cap;
  }
  FML_UNREACHABLE();
}

SkPaint::Join ToSkiaJoin(Join join) {
  switch (join) {
    case Join::kMiter:
      return SkPaint::Join::kMiter_Join;
    case Join::kRound:
      return SkPaint::Join::kRound_Join;
    case Join::kBevel:
      return SkPaint::Join::kBevel_Join;
  }
  FML_UNREACHABLE();
}
}  // namespace

std::shared_ptr<TypographerContext> TypographerContextSkia::Make() {
  return std::make_shared<TypographerContextSkia>();
}

TypographerContextSkia::TypographerContextSkia() = default;

TypographerContextSkia::~TypographerContextSkia() = default;

std::shared_ptr<GlyphAtlasContext>
TypographerContextSkia::CreateGlyphAtlasContext(GlyphAtlas::Type type) const {
  return std::make_shared<GlyphAtlasContext>(type);
}

static SkImageInfo GetImageInfo(const GlyphAtlas& atlas, Size size) {
  switch (atlas.GetType()) {
    case GlyphAtlas::Type::kAlphaBitmap:
      return SkImageInfo::MakeA8(SkISize{static_cast<int32_t>(size.width),
                                         static_cast<int32_t>(size.height)});
    case GlyphAtlas::Type::kColorBitmap:
      return SkImageInfo::MakeN32Premul(size.width, size.height);
  }
  FML_UNREACHABLE();
}

/// Append as many glyphs to the texture as will fit, and return the first index
/// of [extra_pairs] that did not fit.
static size_t AppendToExistingAtlas(
    const std::shared_ptr<GlyphAtlas>& atlas,
    const std::vector<FontGlyphPair>& extra_pairs,
    std::vector<Rect>& glyph_positions,
    const std::vector<Rect>& glyph_sizes,
    ISize atlas_size,
    int64_t height_adjustment,
    const std::shared_ptr<RectanglePacker>& rect_packer) {
  TRACE_EVENT0("impeller", __FUNCTION__);
  if (!rect_packer || atlas_size.IsEmpty()) {
    return 0;
  }

  for (size_t i = 0; i < extra_pairs.size(); i++) {
    ISize glyph_size = ISize::Ceil(glyph_sizes[i].GetSize());
    IPoint16 location_in_atlas;
    if (!rect_packer->AddRect(glyph_size.width + kPadding,   //
                              glyph_size.height + kPadding,  //
                              &location_in_atlas             //
                              )) {
      return i;
    }
    // Position the glyph in the center of the 1px padding.
    glyph_positions.push_back(Rect::MakeXYWH(
        location_in_atlas.x() + 1,                      //
        location_in_atlas.y() + height_adjustment + 1,  //
        glyph_size.width,                               //
        glyph_size.height                               //
        ));
  }

  return extra_pairs.size();
}

static size_t PairsFitInAtlasOfSize(
    const std::vector<FontGlyphPair>& pairs,
    const ISize& atlas_size,
    std::vector<Rect>& glyph_positions,
    const std::vector<Rect>& glyph_sizes,
    int64_t height_adjustment,
    const std::shared_ptr<RectanglePacker>& rect_packer,
    size_t start_index) {
  FML_DCHECK(!atlas_size.IsEmpty());

  for (size_t i = start_index; i < pairs.size(); i++) {
    ISize glyph_size = ISize::Ceil(glyph_sizes[i].GetSize());
    IPoint16 location_in_atlas;
    if (!rect_packer->AddRect(glyph_size.width + kPadding,   //
                              glyph_size.height + kPadding,  //
                              &location_in_atlas             //
                              )) {
      return i;
    }
    glyph_positions.push_back(Rect::MakeXYWH(
        location_in_atlas.x() + 1,                      //
        location_in_atlas.y() + height_adjustment + 1,  //
        glyph_size.width,                               //
        glyph_size.height                               //
        ));
  }

  return pairs.size();
}

static ISize ComputeNextAtlasSize(
    const std::shared_ptr<GlyphAtlasContext>& atlas_context,
    const std::vector<FontGlyphPair>& extra_pairs,
    std::vector<Rect>& glyph_positions,
    const std::vector<Rect>& glyph_sizes,
    size_t glyph_index_start,
    int64_t max_texture_height) {
  // Because we can't grow the skyline packer horizontally, pick a reasonable
  // large width for all atlases.
  static constexpr int64_t kAtlasWidth = 4096;
  static constexpr int64_t kMinAtlasHeight = 1024;

  ISize current_size = ISize(kAtlasWidth, kMinAtlasHeight);
  if (atlas_context->GetAtlasSize().height > current_size.height) {
    current_size.height = atlas_context->GetAtlasSize().height * 2;
  }

  auto height_adjustment = atlas_context->GetAtlasSize().height;
  while (current_size.height <= max_texture_height) {
    std::shared_ptr<RectanglePacker> rect_packer;
    if (atlas_context->GetRectPacker() || glyph_index_start) {
      rect_packer = RectanglePacker::Factory(
          kAtlasWidth,
          current_size.height - atlas_context->GetAtlasSize().height);
    } else {
      rect_packer = RectanglePacker::Factory(kAtlasWidth, current_size.height);
    }
    glyph_positions.erase(glyph_positions.begin() + glyph_index_start,
                          glyph_positions.end());
    atlas_context->UpdateRectPacker(rect_packer);
    auto next_index = PairsFitInAtlasOfSize(
        extra_pairs, current_size, glyph_positions, glyph_sizes,
        height_adjustment, rect_packer, glyph_index_start);
    if (next_index == extra_pairs.size()) {
      return current_size;
    }
    current_size = ISize(current_size.width, current_size.height * 2);
  }
  return {};
}

static void DrawGlyph(SkCanvas* canvas,
                      const SkPoint position,
                      const ScaledFont& scaled_font,
                      const SubpixelGlyph& glyph,
                      const Rect& scaled_bounds,
                      const std::optional<GlyphProperties>& prop,
                      bool has_color) {
  const auto& metrics = scaled_font.font.GetMetrics();
  SkGlyphID glyph_id = glyph.glyph.index;

  SkFont sk_font(
      TypefaceSkia::Cast(*scaled_font.font.GetTypeface()).GetSkiaTypeface(),
      metrics.point_size, metrics.scaleX, metrics.skewX);
  sk_font.setEdging(SkFont::Edging::kAntiAlias);
  sk_font.setHinting(SkFontHinting::kSlight);
  sk_font.setEmbolden(metrics.embolden);
  sk_font.setSubpixel(true);
  sk_font.setSize(sk_font.getSize() * scaled_font.scale);

  auto glyph_color = prop.has_value() ? prop->color.ToARGB() : SK_ColorBLACK;

  SkPaint glyph_paint;
  glyph_paint.setColor(glyph_color);
  glyph_paint.setBlendMode(SkBlendMode::kSrc);
  if (prop.has_value() && prop->stroke) {
    glyph_paint.setStroke(true);
    glyph_paint.setStrokeWidth(prop->stroke_width * scaled_font.scale);
    glyph_paint.setStrokeCap(ToSkiaCap(prop->stroke_cap));
    glyph_paint.setStrokeJoin(ToSkiaJoin(prop->stroke_join));
    glyph_paint.setStrokeMiter(prop->stroke_miter);
  }
  canvas->save();
  canvas->translate(glyph.subpixel_offset.x, glyph.subpixel_offset.y);
  canvas->drawGlyphs(1u,         // count
                     &glyph_id,  // glyphs
                     &position,  // positions
                     SkPoint::Make(-scaled_bounds.GetLeft(),
                                   -scaled_bounds.GetTop()),  // origin
                     sk_font,                                 // font
                     glyph_paint                              // paint
  );
  canvas->restore();
}

/// @brief Batch render to a single surface.
///
/// This is only safe for use when updating a fresh texture.
static bool BulkUpdateAtlasBitmap(const GlyphAtlas& atlas,
                                  std::shared_ptr<BlitPass>& blit_pass,
                                  HostBuffer& host_buffer,
                                  const std::shared_ptr<Texture>& texture,
                                  const std::vector<FontGlyphPair>& new_pairs,
                                  size_t start_index,
                                  size_t end_index) {
  TRACE_EVENT0("impeller", __FUNCTION__);

  bool has_color = atlas.GetType() == GlyphAtlas::Type::kColorBitmap;

  SkBitmap bitmap;
  bitmap.setInfo(GetImageInfo(atlas, Size(texture->GetSize())));
  if (!bitmap.tryAllocPixels()) {
    return false;
  }

  auto surface = SkSurfaces::WrapPixels(bitmap.pixmap());
  if (!surface) {
    return false;
  }
  auto canvas = surface->getCanvas();
  if (!canvas) {
    return false;
  }

  for (size_t i = start_index; i < end_index; i++) {
    const FontGlyphPair& pair = new_pairs[i];
    auto data = atlas.FindFontGlyphBounds(pair);
    if (!data.has_value()) {
      continue;
    }
    auto [pos, bounds, placeholder] = data.value();
    FML_DCHECK(!placeholder);
    Size size = pos.GetSize();
    if (size.IsEmpty()) {
      continue;
    }

    DrawGlyph(canvas, SkPoint::Make(pos.GetLeft(), pos.GetTop()),
              pair.scaled_font, pair.glyph, bounds, pair.glyph.properties,
              has_color);
  }

  // Writing to a malloc'd buffer and then copying to the staging buffers
  // benchmarks as substantially faster on a number of Android devices.
  BufferView buffer_view = host_buffer.Emplace(
      bitmap.getAddr(0, 0),
      texture->GetSize().Area() *
          BytesPerPixelForPixelFormat(
              atlas.GetTexture()->GetTextureDescriptor().format),
      DefaultUniformAlignment());

  return blit_pass->AddCopy(std::move(buffer_view),  //
                            texture,                 //
                            IRect::MakeXYWH(0, 0, texture->GetSize().width,
                                            texture->GetSize().height));
}

static bool UpdateAtlasBitmap(const GlyphAtlas& atlas,
                              std::shared_ptr<BlitPass>& blit_pass,
                              HostBuffer& host_buffer,
                              const std::shared_ptr<Texture>& texture,
                              const std::vector<FontGlyphPair>& new_pairs,
                              size_t start_index,
                              size_t end_index) {
  TRACE_EVENT0("impeller", __FUNCTION__);

  bool has_color = atlas.GetType() == GlyphAtlas::Type::kColorBitmap;

  for (size_t i = start_index; i < end_index; i++) {
    const FontGlyphPair& pair = new_pairs[i];
    auto data = atlas.FindFontGlyphBounds(pair);
    if (!data.has_value()) {
      continue;
    }
    auto [pos, bounds, placeholder] = data.value();
    FML_DCHECK(!placeholder);

    Size size = pos.GetSize();
    if (size.IsEmpty()) {
      continue;
    }
    // The uploaded bitmap is expanded by 1px of padding
    // on each side.
    size.width += 2;
    size.height += 2;

    SkBitmap bitmap;
    bitmap.setInfo(GetImageInfo(atlas, size));
    if (!bitmap.tryAllocPixels()) {
      return false;
    }

    auto surface = SkSurfaces::WrapPixels(bitmap.pixmap());
    if (!surface) {
      return false;
    }
    auto canvas = surface->getCanvas();
    if (!canvas) {
      return false;
    }

    DrawGlyph(canvas, SkPoint::Make(1, 1), pair.scaled_font, pair.glyph, bounds,
              pair.glyph.properties, has_color);

    // Writing to a malloc'd buffer and then copying to the staging buffers
    // benchmarks as substantially faster on a number of Android devices.
    BufferView buffer_view = host_buffer.Emplace(
        bitmap.getAddr(0, 0),
        size.Area() * BytesPerPixelForPixelFormat(
                          atlas.GetTexture()->GetTextureDescriptor().format),
        DefaultUniformAlignment());

    // convert_to_read is set to false so that the texture remains in a transfer
    // dst layout until we finish writing to it below. This only has an impact
    // on Vulkan where we are responsible for managing image layouts.
    if (!blit_pass->AddCopy(std::move(buffer_view),  //
                            texture,                 //
                            IRect::MakeXYWH(pos.GetLeft() - 1, pos.GetTop() - 1,
                                            size.width, size.height),  //
                            /*label=*/"",                              //
                            /*mip_level=*/0,                           //
                            /*slice=*/0,                               //
                            /*convert_to_read=*/false                  //
                            )) {
      return false;
    }
  }
  return blit_pass->ConvertTextureToShaderRead(texture);
}

static Rect ComputeGlyphSize(const SkFont& font,
                             const SubpixelGlyph& glyph,
                             Scalar scale) {
  SkRect scaled_bounds;
  SkPaint glyph_paint;
  if (glyph.properties.has_value() && glyph.properties->stroke) {
    glyph_paint.setStroke(true);
    glyph_paint.setStrokeWidth(glyph.properties->stroke_width * scale);
    glyph_paint.setStrokeCap(ToSkiaCap(glyph.properties->stroke_cap));
    glyph_paint.setStrokeJoin(ToSkiaJoin(glyph.properties->stroke_join));
    glyph_paint.setStrokeMiter(glyph.properties->stroke_miter);
  }
  font.getBounds(&glyph.glyph.index, 1, &scaled_bounds, &glyph_paint);

  // Expand the bounds of glyphs at subpixel offsets by 2 in the x direction.
  Scalar adjustment = 0.0;
  if (glyph.subpixel_offset != Point(0, 0)) {
    adjustment = 1.0;
  }
  return Rect::MakeLTRB(scaled_bounds.fLeft - adjustment, scaled_bounds.fTop,
                        scaled_bounds.fRight + adjustment,
                        scaled_bounds.fBottom);
};

std::pair<std::vector<FontGlyphPair>, std::vector<Rect>>
TypographerContextSkia::CollectNewGlyphs(
    const std::shared_ptr<GlyphAtlas>& atlas,
    const std::vector<std::shared_ptr<TextFrame>>& text_frames) {
  std::vector<FontGlyphPair> new_glyphs;
  std::vector<Rect> glyph_sizes;
  size_t generation_id = atlas->GetAtlasGeneration();
  intptr_t atlas_id = reinterpret_cast<intptr_t>(atlas.get());
  for (const auto& frame : text_frames) {
    auto [frame_generation_id, frame_atlas_id] =
        frame->GetAtlasGenerationAndID();
    if (atlas->IsValid() && frame->IsFrameComplete() &&
        frame_generation_id == generation_id && frame_atlas_id == atlas_id &&
        !frame->GetFrameBounds(0).is_placeholder) {
      continue;
    }
    frame->ClearFrameBounds();
    frame->SetAtlasGeneration(generation_id, atlas_id);

    for (const auto& run : frame->GetRuns()) {
      auto metrics = run.GetFont().GetMetrics();

      auto rounded_scale = TextFrame::RoundScaledFontSize(frame->GetScale());
      ScaledFont scaled_font{.font = run.GetFont(), .scale = rounded_scale};

      FontGlyphAtlas* font_glyph_atlas =
          atlas->GetOrCreateFontGlyphAtlas(scaled_font);
      FML_DCHECK(!!font_glyph_atlas);

      SkFont sk_font(
          TypefaceSkia::Cast(*scaled_font.font.GetTypeface()).GetSkiaTypeface(),
          metrics.point_size, metrics.scaleX, metrics.skewX);
      sk_font.setEdging(SkFont::Edging::kAntiAlias);
      sk_font.setHinting(SkFontHinting::kSlight);
      sk_font.setEmbolden(metrics.embolden);
      // Rather than computing the bounds at the requested point size and
      // scaling up the bounds, we scale up the font size and request the
      // bounds. This seems to give more accurate bounds information.
      sk_font.setSize(sk_font.getSize() * scaled_font.scale);
      sk_font.setSubpixel(true);

      for (const auto& glyph_position : run.GetGlyphPositions()) {
        Point subpixel = TextFrame::ComputeSubpixelPosition(
            glyph_position, scaled_font.font.GetAxisAlignment(),
            frame->GetOffset(), frame->GetScale());
        SubpixelGlyph subpixel_glyph(glyph_position.glyph, subpixel,
                                     frame->GetProperties());
        const auto& font_glyph_bounds =
            font_glyph_atlas->FindGlyphBounds(subpixel_glyph);

        if (!font_glyph_bounds.has_value()) {
          new_glyphs.push_back(FontGlyphPair{scaled_font, subpixel_glyph});
          auto glyph_bounds =
              ComputeGlyphSize(sk_font, subpixel_glyph, scaled_font.scale);
          glyph_sizes.push_back(glyph_bounds);

          auto frame_bounds = FrameBounds{
              Rect::MakeLTRB(0, 0, 0, 0),  //
              glyph_bounds,                //
              /*placeholder=*/true         //
          };

          frame->AppendFrameBounds(frame_bounds);
          font_glyph_atlas->AppendGlyph(subpixel_glyph, frame_bounds);
        } else {
          frame->AppendFrameBounds(font_glyph_bounds.value());
        }
      }
    }
  }
  return {std::move(new_glyphs), std::move(glyph_sizes)};
}

std::shared_ptr<GlyphAtlas> TypographerContextSkia::CreateGlyphAtlas(
    Context& context,
    GlyphAtlas::Type type,
    HostBuffer& host_buffer,
    const std::shared_ptr<GlyphAtlasContext>& atlas_context,
    const std::vector<std::shared_ptr<TextFrame>>& text_frames) const {
  TRACE_EVENT0("impeller", __FUNCTION__);
  if (!IsValid()) {
    return nullptr;
  }
  std::shared_ptr<GlyphAtlas> last_atlas = atlas_context->GetGlyphAtlas();
  FML_DCHECK(last_atlas->GetType() == type);

  if (text_frames.empty()) {
    return last_atlas;
  }

  // ---------------------------------------------------------------------------
  // Step 1: Determine if the atlas type and font glyph pairs are compatible
  //         with the current atlas and reuse if possible. For each new font and
  //         glyph pair, compute the glyph size at scale.
  // ---------------------------------------------------------------------------
  auto [new_glyphs, glyph_sizes] = CollectNewGlyphs(last_atlas, text_frames);
  if (new_glyphs.size() == 0) {
    return last_atlas;
  }

  // ---------------------------------------------------------------------------
  // Step 2: Determine if the additional missing glyphs can be appended to the
  //         existing bitmap without recreating the atlas.
  // ---------------------------------------------------------------------------
  std::vector<Rect> glyph_positions;
  glyph_positions.reserve(new_glyphs.size());
  size_t first_missing_index = 0;

  if (last_atlas->GetTexture()) {
    // Append all glyphs that fit into the current atlas.
    first_missing_index = AppendToExistingAtlas(
        last_atlas, new_glyphs, glyph_positions, glyph_sizes,
        atlas_context->GetAtlasSize(), atlas_context->GetHeightAdjustment(),
        atlas_context->GetRectPacker());

    // ---------------------------------------------------------------------------
    // Step 3a: Record the positions in the glyph atlas of the newly added
    //          glyphs.
    // ---------------------------------------------------------------------------
    for (size_t i = 0; i < first_missing_index; i++) {
      last_atlas->AddTypefaceGlyphPositionAndBounds(
          new_glyphs[i], glyph_positions[i], glyph_sizes[i]);
    }

    std::shared_ptr<CommandBuffer> cmd_buffer = context.CreateCommandBuffer();
    std::shared_ptr<BlitPass> blit_pass = cmd_buffer->CreateBlitPass();

    fml::ScopedCleanupClosure closure([&]() {
      blit_pass->EncodeCommands(context.GetResourceAllocator());
      if (!context.EnqueueCommandBuffer(std::move(cmd_buffer))) {
        VALIDATION_LOG << "Failed to submit glyph atlas command buffer";
      }
    });

    // ---------------------------------------------------------------------------
    // Step 4a: Draw new font-glyph pairs into the a host buffer and encode
    // the uploads into the blit pass.
    // ---------------------------------------------------------------------------
    if (!UpdateAtlasBitmap(*last_atlas, blit_pass, host_buffer,
                           last_atlas->GetTexture(), new_glyphs, 0,
                           first_missing_index)) {
      return nullptr;
    }

    // If all glyphs fit, just return the old atlas.
    if (first_missing_index == new_glyphs.size()) {
      return last_atlas;
    }
  }

  int64_t height_adjustment = atlas_context->GetAtlasSize().height;
  const int64_t max_texture_height =
      context.GetResourceAllocator()->GetMaxTextureSizeSupported().height;

  // IF the current atlas size is as big as it can get, then "GC" and create an
  // atlas with only the required glyphs. OpenGLES cannot reliably perform the
  // blit required here, as 1) it requires attaching textures as read and write
  // framebuffers which has substantially smaller size limits that max textures
  // and 2) is missing a GLES 2.0 implementation and cap check.
  bool blit_old_atlas = true;
  std::shared_ptr<GlyphAtlas> new_atlas = last_atlas;
  if (atlas_context->GetAtlasSize().height >= max_texture_height ||
      context.GetBackendType() == Context::BackendType::kOpenGLES) {
    blit_old_atlas = false;
    new_atlas = std::make_shared<GlyphAtlas>(
        type, /*initial_generation=*/last_atlas->GetAtlasGeneration() + 1);

    auto [update_glyphs, update_sizes] =
        CollectNewGlyphs(new_atlas, text_frames);
    new_glyphs = std::move(update_glyphs);
    glyph_sizes = std::move(update_sizes);

    glyph_positions.clear();
    glyph_positions.reserve(new_glyphs.size());
    first_missing_index = 0;

    height_adjustment = 0;
    atlas_context->UpdateRectPacker(nullptr);
    atlas_context->UpdateGlyphAtlas(new_atlas, {0, 0}, 0);
  }

  // A new glyph atlas must be created.
  ISize atlas_size = ComputeNextAtlasSize(atlas_context,        //
                                          new_glyphs,           //
                                          glyph_positions,      //
                                          glyph_sizes,          //
                                          first_missing_index,  //
                                          max_texture_height    //
  );

  atlas_context->UpdateGlyphAtlas(new_atlas, atlas_size, height_adjustment);
  if (atlas_size.IsEmpty()) {
    return nullptr;
  }
  FML_DCHECK(new_glyphs.size() == glyph_positions.size());

  TextureDescriptor descriptor;
  switch (type) {
    case GlyphAtlas::Type::kAlphaBitmap:
      descriptor.format =
          context.GetCapabilities()->GetDefaultGlyphAtlasFormat();
      break;
    case GlyphAtlas::Type::kColorBitmap:
      descriptor.format = PixelFormat::kR8G8B8A8UNormInt;
      break;
  }
  descriptor.size = atlas_size;
  descriptor.storage_mode = StorageMode::kDevicePrivate;
  descriptor.usage = TextureUsage::kShaderRead;
  std::shared_ptr<Texture> new_texture =
      context.GetResourceAllocator()->CreateTexture(descriptor);
  if (!new_texture) {
    return nullptr;
  }

  new_texture->SetLabel("GlyphAtlas");

  std::shared_ptr<CommandBuffer> cmd_buffer = context.CreateCommandBuffer();
  std::shared_ptr<BlitPass> blit_pass = cmd_buffer->CreateBlitPass();

  fml::ScopedCleanupClosure closure([&]() {
    blit_pass->EncodeCommands(context.GetResourceAllocator());
    if (!context.EnqueueCommandBuffer(std::move(cmd_buffer))) {
      VALIDATION_LOG << "Failed to submit glyph atlas command buffer";
    }
  });

  // Now append all remaining glyphs. This should never have any missing data...
  auto old_texture = new_atlas->GetTexture();
  new_atlas->SetTexture(std::move(new_texture));

  // ---------------------------------------------------------------------------
  // Step 3a: Record the positions in the glyph atlas of the newly added
  //          glyphs.
  // ---------------------------------------------------------------------------
  for (size_t i = first_missing_index; i < glyph_positions.size(); i++) {
    new_atlas->AddTypefaceGlyphPositionAndBounds(
        new_glyphs[i], glyph_positions[i], glyph_sizes[i]);
  }

  // ---------------------------------------------------------------------------
  // Step 4a: Draw new font-glyph pairs into the a host buffer and encode
  // the uploads into the blit pass.
  // ---------------------------------------------------------------------------
  if (!BulkUpdateAtlasBitmap(*new_atlas, blit_pass, host_buffer,
                             new_atlas->GetTexture(), new_glyphs,
                             first_missing_index, new_glyphs.size())) {
    return nullptr;
  }

  // Blit the old texture to the top left of the new atlas.
  if (blit_old_atlas && old_texture) {
    blit_pass->AddCopy(old_texture, new_atlas->GetTexture(),
                       IRect::MakeSize(new_atlas->GetTexture()->GetSize()),
                       {0, 0});
  }

  // ---------------------------------------------------------------------------
  // Step 8b: Record the texture in the glyph atlas.
  // ---------------------------------------------------------------------------

  return new_atlas;
}

}  // namespace impeller
