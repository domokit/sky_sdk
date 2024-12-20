// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TYPOGRAPHER_GLYPH_ATLAS_H_
#define FLUTTER_IMPELLER_TYPOGRAPHER_GLYPH_ATLAS_H_

#include <functional>
#include <memory>
#include <optional>

#include "flutter/fml/build_config.h"

#if defined(OS_FUCHSIA)
// TODO(gaaclarke): Migrate to use absl. I couldn't get it working since absl
// has special logic in its GN files for Fuchsia that I couldn't sort out.
#define IMPELLER_TYPOGRAPHER_USE_STD_HASH
#else
#include "flutter/third_party/abseil-cpp/absl/container/flat_hash_map.h"
#endif

#include "impeller/core/texture.h"
#include "impeller/geometry/rect.h"
#include "impeller/typographer/font_glyph_pair.h"
#include "impeller/typographer/rectangle_packer.h"

namespace impeller {

class FontGlyphAtlas;

/// Helper for AbslHashAdapter. Tallies a hash value with fml::HashCombine.
template <typename T>
struct AbslHashAdapterCombiner {
  std::size_t value = 0;

  template <typename... Args>
  static AbslHashAdapterCombiner combine(AbslHashAdapterCombiner combiner,
                                         const Args&... args) {
    combiner.value = fml::HashCombine(combiner.value, args...);
    return combiner;
  }
};

/// Adapts AbslHashValue functions to be used with std::unordered_map and the
/// fml hash functions.
template <typename T>
struct AbslHashAdapter {
  constexpr std::size_t operator()(const T& element) const {
    AbslHashAdapterCombiner<T> combiner;
    combiner = AbslHashValue(std::move(combiner), element);
    return combiner.value;
  }
};

struct FrameBounds {
  /// The bounds of the glyph within the glyph atlas.
  Rect atlas_bounds;
  /// The local glyph bounds.
  Rect glyph_bounds;
  /// Whether [atlas_bounds] are still a placeholder and have
  /// not yet been computed.
  bool is_placeholder = true;
};

//------------------------------------------------------------------------------
/// @brief      A texture containing the bitmap representation of glyphs in
///             different fonts along with the ability to query the location of
///             specific font glyphs within the texture.
///
class GlyphAtlas {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Describes how the glyphs are represented in the texture.
  enum class Type {
    //--------------------------------------------------------------------------
    /// The glyphs are reprsented at their requested size using only an 8-bit
    /// color channel.
    ///
    /// This might be backed by a grey or red single channel texture, depending
    /// on the backend capabilities.
    kAlphaBitmap,

    //--------------------------------------------------------------------------
    /// The glyphs are reprsented at their requested size using N32 premul
    /// colors.
    ///
    kColorBitmap,
  };

  //----------------------------------------------------------------------------
  /// @brief      Create an empty glyph atlas.
  ///
  /// @param[in]  type  How the glyphs are represented in the texture.
  /// @param[in]  initial_generation the atlas generation.
  ///
  GlyphAtlas(Type type, size_t initial_generation);

  ~GlyphAtlas();

  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Describes how the glyphs are represented in the texture.
  ///
  Type GetType() const;

  //----------------------------------------------------------------------------
  /// @brief      Set the texture for the glyph atlas.
  ///
  /// @param[in]  texture  The texture
  ///
  void SetTexture(std::shared_ptr<Texture> texture);

  //----------------------------------------------------------------------------
  /// @brief      Get the texture for the glyph atlas.
  ///
  /// @return     The texture.
  ///
  const std::shared_ptr<Texture>& GetTexture() const;

  //----------------------------------------------------------------------------
  /// @brief      Record the location of a specific font-glyph pair within the
  ///             atlas.
  ///
  /// @param[in]  pair  The font-glyph pair
  /// @param[in]  rect  The position in the atlas
  /// @param[in]  bounds The bounds of the glyph at scale
  ///
  void AddTypefaceGlyphPositionAndBounds(const FontGlyphPair& pair,
                                         Rect position,
                                         Rect bounds);

  //----------------------------------------------------------------------------
  /// @brief      Get the number of unique font-glyph pairs in this atlas.
  ///
  /// @return     The glyph count.
  ///
  size_t GetGlyphCount() const;

  //----------------------------------------------------------------------------
  /// @brief      Iterate of all the glyphs along with their locations in the
  ///             atlas.
  ///
  /// @param[in]  iterator  The iterator. Return `false` from the iterator to
  ///                       stop iterating.
  ///
  /// @return     The number of glyphs iterated over.
  ///
  size_t IterateGlyphs(
      const std::function<bool(const ScaledFont& scaled_font,
                               const SubpixelGlyph& glyph,
                               const Rect& rect)>& iterator) const;

  //----------------------------------------------------------------------------
  /// @brief      Find the location of a specific font-glyph pair in the atlas.
  ///
  /// @param[in]  pair  The font-glyph pair
  ///
  /// @return     The location of the font-glyph pair in the atlas.
  ///             `std::nullopt` if the pair is not in the atlas.
  ///
  std::optional<FrameBounds> FindFontGlyphBounds(
      const FontGlyphPair& pair) const;

  //----------------------------------------------------------------------------
  /// @brief      Obtain an interface for querying the location of glyphs in the
  ///             atlas for the given font and scale.  This provides a more
  ///             efficient way to look up a run of glyphs in the same font.
  ///
  /// @param[in]  font  The font
  /// @param[in]  scale The scale
  ///
  /// @return     A pointer to a FontGlyphAtlas, or nullptr if the font and
  ///             scale are not available in the atlas.  The pointer is only
  ///             valid for the lifetime of the GlyphAtlas.
  ///
  FontGlyphAtlas* GetOrCreateFontGlyphAtlas(const ScaledFont& scaled_font);

  //----------------------------------------------------------------------------
  /// @brief      Retrieve the generation id for this glyph atlas.
  ///
  ///             The generation id is used to match with a TextFrame to
  ///             determine if the frame is guaranteed to already be populated
  ///             in the atlas.
  size_t GetAtlasGeneration() const;

  //----------------------------------------------------------------------------
  /// @brief      Update the atlas generation.
  void SetAtlasGeneration(size_t value);

 private:
  const Type type_;
  std::shared_ptr<Texture> texture_;
  size_t generation_ = 0;

#if defined(IMPELLER_TYPOGRAPHER_USE_STD_HASH)
  using FontAtlasMap = std::unordered_map<ScaledFont,
                                          FontGlyphAtlas,
                                          AbslHashAdapter<ScaledFont>,
                                          ScaledFont::Equal>;
#else
  using FontAtlasMap = absl::flat_hash_map<ScaledFont,
                                           FontGlyphAtlas,
                                           absl::Hash<ScaledFont>,
                                           ScaledFont::Equal>;
#endif

  FontAtlasMap font_atlas_map_;

  GlyphAtlas(const GlyphAtlas&) = delete;

  GlyphAtlas& operator=(const GlyphAtlas&) = delete;
};

//------------------------------------------------------------------------------
/// @brief      A container for caching a glyph atlas across frames.
///
class GlyphAtlasContext {
 public:
  explicit GlyphAtlasContext(GlyphAtlas::Type type);

  virtual ~GlyphAtlasContext();

  //----------------------------------------------------------------------------
  /// @brief      Retrieve the current glyph atlas.
  std::shared_ptr<GlyphAtlas> GetGlyphAtlas() const;

  //----------------------------------------------------------------------------
  /// @brief      Retrieve the size of the current glyph atlas.
  const ISize& GetAtlasSize() const;

  //----------------------------------------------------------------------------
  /// @brief      Retrieve the previous (if any) rect packer.
  std::shared_ptr<RectanglePacker> GetRectPacker() const;

  //----------------------------------------------------------------------------
  /// @brief      A y-coordinate shift that must be applied to glyphs appended
  /// to
  ///             the atlas.
  ///
  ///             The rectangle packer is only initialized for unfilled regions
  ///             of the atlas. The area the rectangle packer covers is offset
  ///             from the origin by this height adjustment.
  int64_t GetHeightAdjustment() const;

  //----------------------------------------------------------------------------
  /// @brief      Update the context with a newly constructed glyph atlas.
  void UpdateGlyphAtlas(std::shared_ptr<GlyphAtlas> atlas,
                        ISize size,
                        int64_t height_adjustment_);

  void UpdateRectPacker(std::shared_ptr<RectanglePacker> rect_packer);

 private:
  std::shared_ptr<GlyphAtlas> atlas_;
  ISize atlas_size_;
  std::shared_ptr<RectanglePacker> rect_packer_;
  int64_t height_adjustment_;

  GlyphAtlasContext(const GlyphAtlasContext&) = delete;

  GlyphAtlasContext& operator=(const GlyphAtlasContext&) = delete;
};

//------------------------------------------------------------------------------
/// @brief      An object that can look up glyph locations within the GlyphAtlas
///             for a particular typeface.
///
class FontGlyphAtlas {
 public:
  FontGlyphAtlas() = default;
  FontGlyphAtlas(FontGlyphAtlas&&) = default;

  //----------------------------------------------------------------------------
  /// @brief      Find the location of a glyph in the atlas.
  ///
  /// @param[in]  glyph The glyph
  ///
  /// @return     The location of the glyph in the atlas.
  ///             `std::nullopt` if the glyph is not in the atlas.
  ///
  std::optional<FrameBounds> FindGlyphBounds(const SubpixelGlyph& glyph) const;

  //----------------------------------------------------------------------------
  /// @brief      Append the frame bounds of a glyph to this atlas.
  ///
  ///             This may indicate a placeholder glyph location to be replaced
  ///             at a later time, as indicated by FrameBounds.placeholder.
  void AppendGlyph(const SubpixelGlyph& glyph, const FrameBounds& frame_bounds);

 private:
  friend class GlyphAtlas;

#if defined(IMPELLER_TYPOGRAPHER_USE_STD_HASH)
  using PositionsMap = std::unordered_map<SubpixelGlyph,
                                          FrameBounds,
                                          AbslHashAdapter<SubpixelGlyph>,
                                          SubpixelGlyph::Equal>;
#else
  using PositionsMap = absl::flat_hash_map<SubpixelGlyph,
                                           FrameBounds,
                                           absl::Hash<SubpixelGlyph>,
                                           SubpixelGlyph::Equal>;
#endif

  PositionsMap positions_;
  FontGlyphAtlas(const FontGlyphAtlas&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TYPOGRAPHER_GLYPH_ATLAS_H_
