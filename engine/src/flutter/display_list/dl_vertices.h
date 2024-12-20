// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_DL_VERTICES_H_
#define FLUTTER_DISPLAY_LIST_DL_VERTICES_H_

#include <memory>

#include "flutter/display_list/dl_color.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Defines the way in which the vertices of a DlVertices object
///             are separated into triangles into which to render.
///
enum class DlVertexMode {
  /// The vertices are taken 3 at a time to form a triangle.
  kTriangles,

  /// The vertices are taken in overlapping triplets to form triangles, with
  /// each triplet sharing 2 of its vertices with the preceding and following
  /// triplets.
  /// vertices [ABCDE] yield 3 triangles ABC,BCD,CDE
  kTriangleStrip,

  /// The vertices are taken in overlapping pairs and combined with the first
  /// vertex to form triangles that radiate outward from the initial point.
  /// vertices [ABCDE] yield 3 triangles ABC,ACD,ADE
  kTriangleFan,
};

//------------------------------------------------------------------------------
/// @brief      Holds all of the data (both required and optional) for a
///             DisplayList drawVertices call.
///
/// There are 4 main pices of data:
///   - vertices():
///       the points on the rendering surface that define the pixels
///       being rendered to in a series of triangles. These points
///       can map to triangles in various ways depending on the
///       supplied |DlVertexMode|.
///   - texture_coordinates():
///       the points in the DlColorSource to map to the coordinates
///       of the triangles in the vertices(). If missing, the
///       vertex coordinates themselves will be used to map the
///       source colors to the vertices.
///   - colors():
///       colors to map to each triangle vertex. Note that each
///       vertex is mapped to exactly 1 color even if the DlVertex
///   - indices():
///       An indirection based on indices into the array of vertices
///       (and by extension, their associated texture_coordinates and
///       colors). Note that the DLVertexMode will still apply to the
///       indices in the same way (and instead of the way) that it
///       would normally be applied to the vertices themselves. The
///       indices are useful, for example, to fill the vertices with
///       a grid of points and then use the indices to define a
///       triangular mesh that covers that grid without having to
///       repeat the vertex (and texture coordinate and color)
///       information for the times when a given grid coordinate
///       gets reused in up to 4 mesh triangles.
///
/// Note that each vertex is mapped to exactly 1 texture_coordinate and
/// color even if the DlVertexMode or indices specify that it contributes
/// to more than one output triangle.
///
class DlVertices {
 public:
  /// @brief     A utility class to build up a |DlVertices| object
  ///            one set of data at a time.
  class Builder {
   public:
    /// @brief     flags to indicate/promise which of the optional
    ///            texture coordinates or colors will be supplied
    ///            during the build phase.
    union Flags {
      struct {
        unsigned has_texture_coordinates : 1;
        unsigned has_colors : 1;
      };
      uint32_t mask = 0;

      inline Flags operator|(const Flags& rhs) const {
        return {.mask = (mask | rhs.mask)};
      }

      inline Flags& operator|=(const Flags& rhs) {
        mask = mask | rhs.mask;
        return *this;
      }
    };
    static constexpr Flags kNone = {{false, false}};
    static constexpr Flags kHasTextureCoordinates = {{true, false}};
    static constexpr Flags kHasColors = {{false, true}};

    //--------------------------------------------------------------------------
    /// @brief     Constructs a Builder and prepares room for the
    ///            required and optional data.
    ///
    /// Vertices are always required. Optional texture coordinates
    /// and optional colors are reserved depending on the |Flags|.
    /// Optional indices will be reserved if the index_count is
    /// positive (>0).
    ///
    /// The caller must provide all data that is promised by the
    /// supplied |flags| and |index_count| parameters before
    /// calling the |build()| method.
    Builder(DlVertexMode mode, int vertex_count, Flags flags, int index_count);

    /// Returns true iff the underlying object was successfully allocated.
    bool is_valid() const { return vertices_ != nullptr; }

    /// @brief Copies the indicated list of points as vertices.
    ///
    /// fails if vertices have already been supplied.
    void store_vertices(const DlPoint vertices[]);

    /// @brief Copies the indicated list of float pairs as vertices.
    ///
    /// fails if vertices have already been supplied.
    void store_vertices(const float coordinates[]);

    /// @brief Copies the indicated list of points as texture coordinates.
    ///
    /// fails if texture coordinates have already been supplied or if they
    /// were not promised by the flags.has_texture_coordinates.
    void store_texture_coordinates(const DlPoint points[]);

    /// @brief Copies the indicated list of float pairs as texture coordinates.
    ///
    /// fails if texture coordinates have already been supplied or if they
    /// were not promised by the flags.has_texture_coordinates.
    void store_texture_coordinates(const float coordinates[]);

    /// @brief Copies the indicated list of colors as vertex colors.
    ///
    /// fails if colors have already been supplied or if they were not
    /// promised by the flags.has_colors.
    void store_colors(const DlColor colors[]);

    /// @brief Copies the indicated list of unsigned ints as vertex colors
    ///        in the 32-bit RGBA format.
    ///
    /// fails if colors have already been supplied or if they were not
    /// promised by the flags.has_colors.
    void store_colors(const uint32_t colors[]);

    /// @brief Copies the indicated list of 16-bit indices as vertex indices.
    ///
    /// fails if indices have already been supplied or if they were not
    /// promised by (index_count > 0).
    void store_indices(const uint16_t indices[]);

    /// @brief Overwrite the internal bounds with a precomputed bounding rect.
    void store_bounds(DlRect bounds);

    /// @brief Finalizes and the constructed DlVertices object.
    ///
    /// fails if any of the optional data promised in the constructor is
    /// missing.
    std::shared_ptr<DlVertices> build();

   private:
    std::shared_ptr<DlVertices> vertices_;
    bool needs_vertices_ = true;
    bool needs_texture_coords_;
    bool needs_colors_;
    bool needs_indices_;
    bool needs_bounds_ = true;
  };

  //--------------------------------------------------------------------------
  /// @brief     Constructs a DlVector with compact inline storage for all
  ///            of its required and optional lists of data.
  ///
  /// Vertices are always required. Optional texture coordinates
  /// and optional colors are stored if the arguments are non-null.
  /// Optional indices will be stored iff the array argument is
  /// non-null and the index_count is positive (>0).
  static std::shared_ptr<DlVertices> Make(DlVertexMode mode,
                                          int vertex_count,
                                          const DlPoint vertices[],
                                          const DlPoint texture_coordinates[],
                                          const DlColor colors[],
                                          int index_count = 0,
                                          const uint16_t indices[] = nullptr,
                                          const DlRect* bounds = nullptr);

  /// Returns the size of the object including all of the inlined data.
  size_t size() const;

  /// Returns the bounds of the vertices.
  DlRect GetBounds() const { return bounds_; }

  /// Returns the vertex mode that defines how the vertices (or the indices)
  /// are turned into triangles.
  DlVertexMode mode() const { return mode_; }

  /// Returns the number of vertices, which also applies to the number of
  /// texture coordinate and colors if they are provided.
  int vertex_count() const { return vertex_count_; }

  /// Returns a pointer to the vertex information. Should be non-null.
  const DlPoint* vertex_data() const {
    return static_cast<const DlPoint*>(pod(vertices_offset_));
  }

  /// Returns a pointer to the vertex texture coordinate
  /// or null if none were provided.
  const DlPoint* texture_coordinate_data() const {
    return static_cast<const DlPoint*>(pod(texture_coordinates_offset_));
  }

  /// Returns a pointer to the vertex colors
  /// or null if none were provided.
  const DlColor* colors() const {
    return static_cast<const DlColor*>(pod(colors_offset_));
  }

  /// Returns a pointer to the count of vertex indices
  /// or 0 if none were provided.
  int index_count() const { return index_count_; }

  /// Returns a pointer to the vertex indices
  /// or null if none were provided.
  const uint16_t* indices() const {
    return static_cast<const uint16_t*>(pod(indices_offset_));
  }

  bool operator==(DlVertices const& other) const;

  bool operator!=(DlVertices const& other) const { return !(*this == other); }

 private:
  // Constructors are designed to encapsulate arrays sequentially in memory
  // which means they can only be called by intantiations that use the
  // new (ptr) paradigm which precomputes and preallocates the memory for
  // the class body and all of its arrays, such as in Builder.
  DlVertices(DlVertexMode mode,
             int vertex_count,
             const DlPoint vertices[],
             const DlPoint texture_coordinates[],
             const DlColor colors[],
             int index_count,
             const uint16_t indices[],
             const DlRect* bounds = nullptr);

  // This constructor is specifically used by the DlVertices::Builder to
  // establish the object before the copying of data is requested.
  DlVertices(DlVertexMode mode,
             int vertex_count,
             Builder::Flags flags,
             int index_count);

  // The copy constructor has the same memory pre-allocation requirements
  // as this other constructors. This particular version is used by the
  // DisplaylistBuilder to copy the instance into pre-allocated pod memory
  // in the display list buffer.
  explicit DlVertices(const DlVertices* other);

  DlVertexMode mode_;

  int vertex_count_;
  size_t vertices_offset_;
  size_t texture_coordinates_offset_;
  size_t colors_offset_;

  int index_count_;
  size_t indices_offset_;

  DlRect bounds_;

  const void* pod(int offset) const {
    if (offset <= 0) {
      return nullptr;
    }
    const void* base = static_cast<const void*>(this);
    return static_cast<const char*>(base) + offset;
  }

  friend class DisplayListBuilder;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_DL_VERTICES_H_
