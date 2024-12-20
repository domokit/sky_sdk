// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_PAINTING_VERTICES_H_
#define FLUTTER_LIB_UI_PAINTING_VERTICES_H_

#include "flutter/display_list/dl_vertices.h"
#include "flutter/lib/ui/dart_wrapper.h"
#include "third_party/skia/include/core/SkVertices.h"
#include "third_party/tonic/typed_data/typed_list.h"

namespace flutter {

class Vertices : public RefCountedDartWrappable<Vertices> {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_MAKE_REF_COUNTED(Vertices);

 public:
  ~Vertices() override;

  static bool init(Dart_Handle vertices_handle,
                   DlVertexMode vertex_mode,
                   Dart_Handle positions_handle,
                   Dart_Handle texture_coordinates_handle,
                   Dart_Handle colors_handle,
                   Dart_Handle indices_handle);

  const std::shared_ptr<DlVertices>& vertices() const { return vertices_; }

  void dispose();

 private:
  Vertices();

  std::shared_ptr<DlVertices> vertices_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_PAINTING_VERTICES_H_
