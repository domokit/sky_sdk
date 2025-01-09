// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:macrobenchmarks/common.dart';

import 'util.dart';

void main() {
  macroPerfTestMultiPageE2E(
    'opacity_peephole_col_of_alpha_save_layer_rows',
    <ScrollableButtonRoute>[
      ScrollableButtonRoute(kScrollableName, kOpacityPeepholeRouteName),
      ScrollableButtonRoute(
        kOpacityScrollableName,
        kOpacityPeepholeColumnOfAlphaSaveLayerRowsOfRectsRouteName,
      ),
    ],
    pageDelay: const Duration(seconds: 1),
    duration: const Duration(seconds: 10),
  );
}
