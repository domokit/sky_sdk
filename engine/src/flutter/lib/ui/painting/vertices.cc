// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/vertices.h"

#include <algorithm>

#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/tonic/dart_binding_macros.h"
#include "third_party/tonic/dart_library_natives.h"

namespace flutter {

IMPLEMENT_WRAPPERTYPEINFO(ui, Vertices);

Vertices::Vertices() {}

Vertices::~Vertices() {}

bool Vertices::init(Dart_Handle vertices_handle,
                    DlVertexMode vertex_mode,
                    Dart_Handle positions_handle,
                    Dart_Handle texture_coordinates_handle,
                    Dart_Handle colors_handle,
                    Dart_Handle indices_handle) {
  UIDartState::ThrowIfUIOperationsProhibited();

  tonic::Float32List positions(positions_handle);
  tonic::Float32List texture_coordinates(texture_coordinates_handle);
  tonic::Int32List colors(colors_handle);
  tonic::Uint16List indices(indices_handle);

  if (!positions.data()) {
    return false;
  }

  DlVertices::Builder::Flags flags;
  if (texture_coordinates.data()) {
    flags = flags | DlVertices::Builder::kHasTextureCoordinates;
  }
  if (colors.data()) {
    flags = flags | DlVertices::Builder::kHasColors;
  }
  DlVertices::Builder builder(vertex_mode, positions.num_elements() / 2, flags,
                              indices.num_elements());

  if (!builder.is_valid()) {
    return false;
  }

  // positions are required for SkVertices::Builder
  builder.store_vertices(positions.data());

  if (texture_coordinates.data()) {
    // SkVertices::Builder assumes equal numbers of elements
    FML_DCHECK(positions.num_elements() == texture_coordinates.num_elements());
    builder.store_texture_coordinates(texture_coordinates.data());
  }

  if (colors.data()) {
    // SkVertices::Builder assumes equal numbers of elements
    FML_DCHECK(positions.num_elements() / 2 == colors.num_elements());
    builder.store_colors(reinterpret_cast<const SkColor*>(colors.data()));
  }

  if (indices.data() && indices.num_elements() > 0) {
    builder.store_indices(indices.data());
  }

  positions.Release();
  texture_coordinates.Release();
  colors.Release();
  indices.Release();

  auto vertices = fml::MakeRefCounted<Vertices>();
  vertices->vertices_ = builder.build();
  vertices->AssociateWithDartWrapper(vertices_handle);

  return true;
}

void Vertices::dispose() {
  vertices_.reset();
  ClearDartWrapper();
}

}  // namespace flutter
