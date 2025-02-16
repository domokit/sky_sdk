// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/web/compile.dart';
import 'package:test/test.dart';

void main() {
  group('dart-defines and web-renderer options', () {
    late List<String> dartDefines;

    setUp(() {
      dartDefines = <String>[];
    });

    test('canvaskit web-renderer with no dart-defines', () {
      dartDefines = WebRendererMode.canvaskit.updateDartDefines(dartDefines);
      expect(dartDefines, <String>['FLUTTER_WEB_USE_SKIA=true', 'FLUTTER_WEB_USE_SKWASM=false']);
    });

    test('canvaskit web-renderer with existing dart-defines', () {
      dartDefines = <String>['FLUTTER_WEB_USE_SKIA=false'];
      dartDefines = WebRendererMode.canvaskit.updateDartDefines(dartDefines);
      expect(dartDefines, <String>['FLUTTER_WEB_USE_SKIA=true', 'FLUTTER_WEB_USE_SKWASM=false']);
    });

    test('skwasm web-renderer with existing dart-defines', () {
      dartDefines = <String>['FLUTTER_WEB_USE_SKWASM=false'];
      dartDefines = WebRendererMode.skwasm.updateDartDefines(dartDefines);
      expect(dartDefines, <String>['FLUTTER_WEB_USE_SKIA=false', 'FLUTTER_WEB_USE_SKWASM=true']);
    });
  });
}
