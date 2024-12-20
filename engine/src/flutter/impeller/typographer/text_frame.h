// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_TEXT_FRAME_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_TEXT_FRAME_H_

#include <cstdint>
#include "impeller/typographer/glyph_atlas.h"
#include "impeller/typographer/text_run.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      Represents a collection of shaped text runs.
///
///             This object is typically the entrypoint in the Impeller type
///             rendering subsystem.
///
/// A text frame should not be reused in multiple places within a single frame,
/// as internally it is used as a cache for various glyph properties.
class TextFrame {
 public:
  TextFrame();

  TextFrame(std::vector<TextRun>& runs, Rect bounds, bool has_color);

  ~TextFrame();

  static Point ComputeSubpixelPosition(
      const TextRun::GlyphPosition& glyph_position,
      AxisAlignment alignment,
      Point offset,
      Scalar scale);

  static Scalar RoundScaledFontSize(Scalar scale);

  //----------------------------------------------------------------------------
  /// @brief      The conservative bounding box for this text frame.
  ///
  /// @return     The bounds rectangle. If there are no glyphs in this text
  ///             frame and empty Rectangle is returned instead.
  ///
  Rect GetBounds() const;

  //----------------------------------------------------------------------------
  /// @brief      The number of runs in this text frame.
  ///
  /// @return     The run count.
  ///
  size_t GetRunCount() const;

  //----------------------------------------------------------------------------
  /// @brief      Returns a reference to all the text runs in this frame.
  ///
  /// @return     The runs in this frame.
  ///
  const std::vector<TextRun>& GetRuns() const;

  //----------------------------------------------------------------------------
  /// @brief      Returns the paint color this text frame was recorded with.
  ///
  ///             Non-bitmap/COLR fonts always use a black text color here, but
  ///             COLR fonts can potentially use the paint color in the glyph
  ///             atlas, so this color must be considered as part of the cache
  ///             key.
  bool HasColor() const;

  //----------------------------------------------------------------------------
  /// @brief      The type of atlas this run should be place in.
  GlyphAtlas::Type GetAtlasType() const;

  /// @brief Verifies that all glyphs in this text frame have computed bounds
  ///        information.
  bool IsFrameComplete() const;

  /// @brief Retrieve the frame bounds for the glyph at [index].
  ///
  /// This method is only valid if [IsFrameComplete] returns true.
  const FrameBounds& GetFrameBounds(size_t index) const;

  /// @brief Store text frame scale, offset, and properties for hashing in th
  /// glyph atlas.
  void SetPerFrameData(Scalar scale,
                       Point offset,
                       std::optional<GlyphProperties> properties);

  // A generation id for the glyph atlas this text run was associated
  // with. As long as the frame generation matches the atlas generation,
  // the contents are guaranteed to be populated and do not need to be
  // processed.
  std::pair<size_t, intptr_t> GetAtlasGenerationAndID() const;

  TextFrame& operator=(TextFrame&& other) = default;

  TextFrame(const TextFrame& other) = default;

 private:
  friend class TypographerContextSkia;
  friend class LazyGlyphAtlas;

  Scalar GetScale() const;

  Point GetOffset() const;

  std::optional<GlyphProperties> GetProperties() const;

  void AppendFrameBounds(const FrameBounds& frame_bounds);

  void ClearFrameBounds();

  void SetAtlasGeneration(size_t value, intptr_t atlas_id);

  std::vector<TextRun> runs_;
  Rect bounds_;
  bool has_color_;

  // Data that is cached when rendering the text frame and is only
  // valid for the current atlas generation.
  std::vector<FrameBounds> bound_values_;
  Scalar scale_ = 0;
  size_t generation_ = 0;
  intptr_t atlas_id_ = 0;
  Point offset_;
  std::optional<GlyphProperties> properties_;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_TEXT_FRAME_H_
