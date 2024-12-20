// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final LiveTestWidgetsFlutterBinding binding = LiveTestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('localToGlobal and globalToLocal calculate correct results', (
    WidgetTester tester,
  ) async {
    tester.view.physicalSize = const Size(2400, 1800);
    tester.view.devicePixelRatio = 3.0;
    final RenderView renderView = RenderView(
      view: tester.view,
      configuration: TestViewConfiguration.fromView(view: tester.view, size: const Size(400, 200)),
    );

    // The configuration above defines a view with a resolution of 2400x1800
    // physical pixels. With a device pixel ratio of 3x, this yields a
    // resolution of 800x600 logical pixels. In this view, a RenderView sized
    // 400x200 (in logical pixels) is fitted using the BoxFit.contain
    // algorithm (as documented on TestViewConfiguration. To fit 400x200 into
    // 800x600 the RenderView is scaled up by 2 to fill the full width and then
    // vertically positioned in the middle. The origin of the RenderView is
    // located at (0, 100) in the logical coordinate space of the view:
    //
    //           View: 800 logical pixels wide (or 2400 physical pixels)
    // +---------------------------------------+
    // |                                       |
    // | 100px                                 |
    // |                                       |
    // +---------------------------------------+
    // |                                       |
    // |     RenderView (400x200px)            |
    // | 400px          scaled to 800x400px    | View: 600 logical pixels high (or 1800 physical pixels)
    // |                                       |
    // |                                       |
    // +---------------------------------------+
    // |                                       |
    // | 200px                                 |
    // |                                       |
    // +---------------------------------------+
    //
    // All values in logical pixels until otherwise noted.
    //
    // A point  can be translated from the local coordinate space of the
    // RenderView (in logical pixels) to the global coordinate space of the View
    // (in logical pixels) by multiplying each coordinate by 2 and adding 100 to
    // the y coordinate. This is what the localToGlobal/globalToLocal methods
    // do:

    expect(binding.localToGlobal(const Offset(0, -50), renderView), Offset.zero);
    expect(binding.localToGlobal(Offset.zero, renderView), const Offset(0, 100));
    expect(binding.localToGlobal(const Offset(200, 100), renderView), const Offset(400, 300));
    expect(binding.localToGlobal(const Offset(150, 75), renderView), const Offset(300, 250));
    expect(binding.localToGlobal(const Offset(400, 200), renderView), const Offset(800, 500));
    expect(binding.localToGlobal(const Offset(400, 400), renderView), const Offset(800, 900));

    expect(binding.globalToLocal(Offset.zero, renderView), const Offset(0, -50));
    expect(binding.globalToLocal(const Offset(0, 100), renderView), Offset.zero);
    expect(binding.globalToLocal(const Offset(400, 300), renderView), const Offset(200, 100));
    expect(binding.globalToLocal(const Offset(300, 250), renderView), const Offset(150, 75));
    expect(binding.globalToLocal(const Offset(800, 500), renderView), const Offset(400, 200));
    expect(binding.globalToLocal(const Offset(800, 900), renderView), const Offset(400, 400));
  });
}
