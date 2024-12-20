// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "export.h"

#include "third_party/skia/include/core/SkVertices.h"

SKWASM_EXPORT SkVertices* vertices_create(SkVertices::VertexMode vertexMode,
                                          int vertexCount,
                                          SkPoint* positions,
                                          SkPoint* textureCoordinates,
                                          SkColor* colors,
                                          int indexCount,
                                          uint16_t* indices) {
  return SkVertices::MakeCopy(vertexMode, vertexCount, positions,
                              textureCoordinates, colors, indexCount, indices)
      .release();
}

SKWASM_EXPORT void vertices_dispose(SkVertices* vertices) {
  vertices->unref();
}
