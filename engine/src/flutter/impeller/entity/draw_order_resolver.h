// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_ENTITY_DRAW_ORDER_RESOLVER_H_
#define FLUTTER_IMPELLER_ENTITY_DRAW_ORDER_RESOLVER_H_

#include <optional>
#include <vector>

namespace impeller {

/// Helper that records draw indices in painter's order and sorts the draws into
/// an optimized order based on translucency and clips.
class DrawOrderResolver {
 public:
  using ElementRefs = std::vector<size_t>;

  DrawOrderResolver();

  void AddElement(size_t element_index, bool is_opaque);

  void PushClip(size_t element_index);

  void PopClip();

  void Flush();

  //-------------------------------------------------------------------------
  /// @brief      Returns the sorted draws for the current draw order layer.
  ///             This should only be called after all recording has finished.
  ///
  /// @param[in]  opaque_skip_count       The number of opaque elements to skip
  ///                                     when appending the combined elements.
  ///                                     This is used for the "clear color"
  ///                                     optimization.
  /// @param[in]  translucent_skip_count  The number of translucent elements to
  ///                                     skip when appending the combined
  ///                                     elements. This is used for the
  ///                                     "clear color" optimization.
  ///
  ElementRefs GetSortedDraws(size_t opaque_skip_count,
                             size_t translucent_skip_count) const;

 private:
  /// A data structure for collecting sorted draws for a given "draw order
  /// layer". Currently these layers just correspond to the local clip stack.
  struct DrawOrderLayer {
    /// The list of backdrop-independent elements (always just opaque). These
    /// are order independent, and so we render these elements in reverse
    /// painter's order so that they cull one another.
    ElementRefs opaque_elements;

    /// The list of backdrop-dependent elements with respect to this draw
    /// order layer. These elements are drawn after all of the independent
    /// elements.
    ElementRefs dependent_elements;

    //-----------------------------------------------------------------------
    /// @brief      Appends the combined opaque and transparent elements into
    ///             a final destination buffer.
    ///
    /// @param[in]  destination             The buffer to append the combined
    ///                                     elements to.
    /// @param[in]  opaque_skip_count       The number of opaque elements to
    ///                                     skip when appending the combined
    ///                                     elements. This is used for the
    ///                                     "clear color" optimization.
    /// @param[in]  translucent_skip_count  The number of translucent elements
    ///                                     to skip when appending the combined
    ///                                     elements. This is used for the
    ///                                     "clear color" optimization.
    ///
    void WriteCombinedDraws(ElementRefs& destination,
                            size_t opaque_skip_count,
                            size_t translucent_skip_count) const;
  };
  std::vector<DrawOrderLayer> draw_order_layers_;

  // The first time the root layer is flushed, the layer contents are stored
  // here. This is done to enable element skipping for the clear color
  // optimization.
  std::optional<DrawOrderLayer> first_root_flush_;
  // All subsequent root flushes are stored here.
  ElementRefs sorted_elements_;

  DrawOrderResolver(const DrawOrderResolver&) = delete;

  DrawOrderResolver& operator=(const DrawOrderResolver&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_ENTITY_DRAW_ORDER_RESOLVER_H_
