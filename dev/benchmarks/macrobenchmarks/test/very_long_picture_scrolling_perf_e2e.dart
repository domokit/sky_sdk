// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:macrobenchmarks/common.dart';

import 'util.dart';

void main() {
  macroPerfTestE2E(
    'very_long_picture_scrolling_perf',
    kVeryLongPictureScrollingRouteName,
    pageDelay: const Duration(seconds: 1),
    duration: const Duration(seconds: 30),
    body: (WidgetController controller) async {
      final Finder nestedScroll = find.byKey(const ValueKey<String>('vlp_single_child_scrollable'));
      expect(nestedScroll, findsOneWidget);
      Future<void> scrollOnce(double offset) async {
        await controller.timedDrag(
          nestedScroll,
          Offset(offset, 0.0),
          const Duration(milliseconds: 3500),
        );
        await Future<void>.delayed(const Duration(milliseconds: 500));
      }

      for (int i = 0; i < 2; i += 1) {
        await scrollOnce(-3000.0);
        await scrollOnce(-3000.0);
        await scrollOnce(3000.0);
        await scrollOnce(3000.0);
      }
    },
  );
}
