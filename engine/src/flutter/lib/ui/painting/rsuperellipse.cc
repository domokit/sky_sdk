// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/painting/rsuperellipse.h"

#include "flutter/fml/logging.h"
#include "third_party/tonic/logging/dart_error.h"
#include "third_party/tonic/typed_data/typed_list.h"

using flutter::RSuperellipse;

namespace tonic {

// Construct an DlRoundSuperellipse from a Dart RSuperellipse object.
// The Dart RSuperellipse is a Float32List containing
//   [left, top, right, bottom, radius]
RSuperellipse DartConverter<flutter::RSuperellipse>::FromDart(
    Dart_Handle value) {
  Float32List buffer(value);

  if (buffer.data() == nullptr) {
    return RSuperellipse{
        .is_null = true,
    };
  }

  // The Flutter rect may be inverted (upside down, backward, or both)
  // Historically, Skia would normalize such rects but we will do that
  // manually below when we construct the Impeller RoundSuperellipse
  flutter::DlRect raw_rect =
      flutter::DlRect::MakeLTRB(buffer[0], buffer[1], buffer[2], buffer[3]);

  return RSuperellipse{
      .rsuperellipse = flutter::DlRoundSuperellipse::MakeRectRadius(
          raw_rect.GetPositive(), buffer[4]),
      .is_null = false,
  };
}

RSuperellipse DartConverter<flutter::RSuperellipse>::FromArguments(
    Dart_NativeArguments args,
    int index,
    Dart_Handle& exception) {
  Dart_Handle value = Dart_GetNativeArgument(args, index);
  FML_DCHECK(!CheckAndHandleError(value));
  return FromDart(value);
}

}  // namespace tonic
