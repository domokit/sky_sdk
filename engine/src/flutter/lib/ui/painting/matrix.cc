// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/matrix.h"

#include "flutter/fml/logging.h"
#include "flutter/lib/ui/floating_point.h"

namespace flutter {

// Mappings from SkMatrix-index to input-index.
static const int kSkMatrixIndexToMatrix4Index[] = {
    // clang-format off
    0, 4, 12,
    1, 5, 13,
    3, 7, 15,
    // clang-format on
};

SkM44 ToSkM44(const tonic::Float64List& matrix4) {
  // clang-format off
  return SkM44(
    SafeNarrow(matrix4[ 0]), SafeNarrow(matrix4[ 4]), SafeNarrow(matrix4[ 8]), SafeNarrow(matrix4[12]),
    SafeNarrow(matrix4[ 1]), SafeNarrow(matrix4[ 5]), SafeNarrow(matrix4[ 9]), SafeNarrow(matrix4[13]),
    SafeNarrow(matrix4[ 2]), SafeNarrow(matrix4[ 6]), SafeNarrow(matrix4[10]), SafeNarrow(matrix4[14]),
    SafeNarrow(matrix4[ 3]), SafeNarrow(matrix4[ 7]), SafeNarrow(matrix4[11]), SafeNarrow(matrix4[15])
  );
  // clang-format on
}

SkMatrix ToSkMatrix(const tonic::Float64List& matrix4) {
  FML_DCHECK(matrix4.data());
  SkMatrix sk_matrix;
  for (int i = 0; i < 9; ++i) {
    int matrix4_index = kSkMatrixIndexToMatrix4Index[i];
    if (matrix4_index < matrix4.num_elements()) {
      sk_matrix[i] = SafeNarrow(matrix4[matrix4_index]);
    } else {
      sk_matrix[i] = 0.0f;
    }
  }
  return sk_matrix;
}

DlMatrix ToDlMatrix(const tonic::Float64List& matrix4) {
  FML_DCHECK(matrix4.data());
  // clang-format off
  return DlMatrix::MakeColumn(
    SafeNarrow(matrix4[ 0]), SafeNarrow(matrix4[ 1]), SafeNarrow(matrix4[ 2]), SafeNarrow(matrix4[ 3]),
    SafeNarrow(matrix4[ 4]), SafeNarrow(matrix4[ 5]), SafeNarrow(matrix4[ 6]), SafeNarrow(matrix4[ 7]),
    SafeNarrow(matrix4[ 8]), SafeNarrow(matrix4[ 9]), SafeNarrow(matrix4[10]), SafeNarrow(matrix4[11]),
    SafeNarrow(matrix4[12]), SafeNarrow(matrix4[13]), SafeNarrow(matrix4[14]), SafeNarrow(matrix4[15])
  );
  // clang-format on
}

tonic::Float64List ToMatrix4(const SkMatrix& sk_matrix) {
  tonic::Float64List matrix4(Dart_NewTypedData(Dart_TypedData_kFloat64, 16));
  for (int i = 0; i < 9; ++i) {
    matrix4[kSkMatrixIndexToMatrix4Index[i]] = sk_matrix[i];
  }
  matrix4[10] = 1.0;  // Identity along the z axis.
  return matrix4;
}

}  // namespace flutter
