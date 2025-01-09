// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:macrobenchmarks/common.dart';

import 'util.dart';

void main() {
  macroPerfTestMultiPageE2E(
    'opacity_peephole_grid_of_alpha_savelayers',
    <ScrollableButtonRoute>[
      ScrollableButtonRoute(kScrollableName, kOpacityPeepholeRouteName),
      ScrollableButtonRoute(
        kOpacityScrollableName,
        kOpacityPeepholeGridOfAlphaSaveLayerRectsRouteName,
      ),
    ],
    pageDelay: const Duration(seconds: 1),
    duration: const Duration(seconds: 10),
  );
}
