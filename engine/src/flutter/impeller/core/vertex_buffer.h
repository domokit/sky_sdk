// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_CORE_VERTEX_BUFFER_H_
#define FLUTTER_IMPELLER_CORE_VERTEX_BUFFER_H_

#include "impeller/core/buffer_view.h"
#include "impeller/core/formats.h"

namespace impeller {

constexpr size_t kMaxVertexBuffers = 16;

struct VertexBuffer {
  BufferView vertex_buffer;

  //----------------------------------------------------------------------------
  /// The index buffer binding used by the vertex shader stage.
  BufferView index_buffer;

  //----------------------------------------------------------------------------
  /// The total count of vertices, either in the vertex_buffer if the
  /// index_type is IndexType::kNone or in the index_buffer otherwise.
  size_t vertex_count = 0u;

  //----------------------------------------------------------------------------
  /// The type of indices in the index buffer. The indices must be tightly
  /// packed in the index buffer.
  ///
  IndexType index_type = IndexType::kUnknown;

  constexpr explicit operator bool() const {
    return static_cast<bool>(vertex_buffer) &&
           (index_type == IndexType::kNone || static_cast<bool>(index_buffer));
  }
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_CORE_VERTEX_BUFFER_H_
