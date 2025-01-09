// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';

void main() {
  // Changes made in https://github.com/flutter/flutter/pull/122446
  final clipboardData1 = ClipboardData();
  final clipboardData2 = ClipboardData(text: null);

  // Changes made in https://github.com/flutter/flutter/pull/60320
  final SurfaceAndroidViewController surfaceController =
      SurfaceAndroidViewController(
        viewId: 10,
        viewType: 'FixTester',
        layoutDirection: TextDirection.ltr,
      );
  int viewId = surfaceController.id;
  final SurfaceAndroidViewController surfaceController =
      SurfaceAndroidViewController(error: '');
  final TextureAndroidViewController textureController =
      TextureAndroidViewController(error: '');
  final TextureAndroidViewController textureController =
      TextureAndroidViewController(
        viewId: 10,
        viewType: 'FixTester',
        layoutDirection: TextDirection.ltr,
      );
  viewId = textureController.id;

  // Changes made in https://github.com/flutter/flutter/pull/81303
  await SystemChrome.setEnabledSystemUIOverlays(<SystemUiOverlay>[
    SystemUiOverlay.top,
  ]);
  await SystemChrome.setEnabledSystemUIOverlays(<SystemUiOverlay>[
    SystemUiOverlay.bottom,
  ]);
  await SystemChrome.setEnabledSystemUIOverlays(<SystemUiOverlay>[
    SystemUiOverlay.top,
    SystemUiOverlay.bottom,
  ]);
  await SystemChrome.setEnabledSystemUIOverlays(<SystemUiOverlay>[]);
  await SystemChrome.setEnabledSystemUIOverlays(error: '');
}
