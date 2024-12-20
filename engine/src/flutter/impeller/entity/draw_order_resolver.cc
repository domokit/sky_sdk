// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/entity/draw_order_resolver.h"

#include "flutter/fml/logging.h"
#include "impeller/base/validation.h"

namespace impeller {

DrawOrderResolver::DrawOrderResolver() : draw_order_layers_({{}}){};

void DrawOrderResolver::AddElement(size_t element_index, bool is_opaque) {
  DrawOrderLayer& layer = draw_order_layers_.back();
  if (is_opaque) {
    layer.opaque_elements.push_back(element_index);
  } else {
    layer.dependent_elements.push_back(element_index);
  }
}
void DrawOrderResolver::PushClip(size_t element_index) {
  draw_order_layers_.back().dependent_elements.push_back(element_index);
  draw_order_layers_.push_back({});
};

void DrawOrderResolver::PopClip() {
  if (draw_order_layers_.size() == 1u) {
    // This is likely recoverable, so don't assert.
    VALIDATION_LOG << "Attemped to pop the first draw order clip layer. This "
                      "may be a bug in `EntityPass`.";
    return;
  }

  DrawOrderLayer& layer = draw_order_layers_.back();
  DrawOrderLayer& parent_layer =
      draw_order_layers_[draw_order_layers_.size() - 2];

  layer.WriteCombinedDraws(parent_layer.dependent_elements, 0, 0);

  draw_order_layers_.pop_back();
}

void DrawOrderResolver::Flush() {
  FML_DCHECK(draw_order_layers_.size() >= 1u);

  size_t layer_count = draw_order_layers_.size();

  // Pop all clip layers.
  while (draw_order_layers_.size() > 1u) {
    PopClip();
  }

  // Move the root layer items into the sorted list.
  DrawOrderLayer& layer = draw_order_layers_.back();
  if (!first_root_flush_.has_value()) {
    // Record the first flush.
    first_root_flush_ = std::move(layer);
    layer = {};
  } else {
    // Write subsequent flushes into the sorted root list.
    layer.WriteCombinedDraws(sorted_elements_, 0, 0);
    layer.opaque_elements.clear();
    layer.dependent_elements.clear();
  }

  // Refill with empty layers.
  draw_order_layers_.resize(layer_count);
}

DrawOrderResolver::ElementRefs DrawOrderResolver::GetSortedDraws(
    size_t opaque_skip_count,
    size_t translucent_skip_count) const {
  FML_DCHECK(draw_order_layers_.size() == 1u)
      << "Attempted to get sorted draws before all clips were popped.";

  ElementRefs sorted_elements;
  sorted_elements.reserve(
      (first_root_flush_.has_value()
           ? first_root_flush_->opaque_elements.size() +
                 first_root_flush_->dependent_elements.size()
           : 0u) +
      sorted_elements_.size() +
      draw_order_layers_.back().opaque_elements.size() +
      draw_order_layers_.back().dependent_elements.size());

  // Write all flushed items.
  if (first_root_flush_.has_value()) {
    first_root_flush_->WriteCombinedDraws(sorted_elements, opaque_skip_count,
                                          translucent_skip_count);
  }
  sorted_elements.insert(sorted_elements.end(), sorted_elements_.begin(),
                         sorted_elements_.end());

  // Write any remaining non-flushed items.
  draw_order_layers_.back().WriteCombinedDraws(
      sorted_elements, first_root_flush_.has_value() ? 0 : opaque_skip_count,
      first_root_flush_.has_value() ? 0 : translucent_skip_count);

  return sorted_elements;
}

void DrawOrderResolver::DrawOrderLayer::WriteCombinedDraws(
    ElementRefs& destination,
    size_t opaque_skip_count,
    size_t translucent_skip_count) const {
  FML_DCHECK(opaque_skip_count <= opaque_elements.size());
  FML_DCHECK(translucent_skip_count <= dependent_elements.size());

  destination.reserve(destination.size() +                          //
                      opaque_elements.size() - opaque_skip_count +  //
                      dependent_elements.size() - translucent_skip_count);

  // Draw backdrop-independent elements in reverse order first.
  destination.insert(destination.end(), opaque_elements.rbegin(),
                     opaque_elements.rend() - opaque_skip_count);
  // Then, draw backdrop-dependent elements in their original order.
  destination.insert(destination.end(),
                     dependent_elements.begin() + translucent_skip_count,
                     dependent_elements.end());
}

}  // namespace impeller
