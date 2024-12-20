// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:macrobenchmarks/common.dart';

import 'util.dart';

void main() {
  macroPerfTestE2E(
    'picture_cache_perf',
    kPictureCacheRouteName,
    pageDelay: const Duration(seconds: 1),
    body: (WidgetController controller) async {
      final Finder nestedScroll = find.byKey(const ValueKey<String>('tabbar_view'));
      expect(nestedScroll, findsOneWidget);
      Future<void> scrollOnce(double offset) async {
        await controller.timedDrag(
          nestedScroll,
          Offset(offset, 0.0),
          const Duration(milliseconds: 300),
        );
        await Future<void>.delayed(const Duration(milliseconds: 500));
      }

      for (int i = 0; i < 3; i += 1) {
        await scrollOnce(-300.0);
        await scrollOnce(-300.0);
        await scrollOnce(300.0);
        await scrollOnce(300.0);
      }
    },
  );
}
