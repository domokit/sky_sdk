// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('toString control test', (WidgetTester tester) async {
    await tester.pumpWidget(const Center(child: Text('Hello', textDirection: TextDirection.ltr)));
    final HitTestResult result = tester.hitTestOnBinding(Offset.zero);
    expect(result, hasOneLineDescription);
    expect(result.path.first, hasOneLineDescription);
  });

  testWidgets('A mouse click should only cause one hit test', (WidgetTester tester) async {
    int hitCount = 0;
    await tester.pumpWidget(
      _HitTestCounter(
        onHitTestCallback: () {
          hitCount += 1;
        },
        child: Container(),
      ),
    );

    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byType(_HitTestCounter)),
      kind: PointerDeviceKind.mouse,
    );
    await gesture.up();

    expect(hitCount, 1);
  });

  testWidgets('Non-mouse events should not cause movement hit tests', (WidgetTester tester) async {
    int hitCount = 0;
    await tester.pumpWidget(
      _HitTestCounter(
        onHitTestCallback: () {
          hitCount += 1;
        },
        child: Container(),
      ),
    );

    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.byType(_HitTestCounter)),
    );
    await gesture.moveBy(const Offset(1, 1));
    await gesture.up();

    expect(hitCount, 1);
  });
}

// The [_HitTestCounter] invokes [onHitTestCallback] every time
// [hitTestChildren] is called.
class _HitTestCounter extends SingleChildRenderObjectWidget {
  const _HitTestCounter({required Widget super.child, required this.onHitTestCallback});

  final VoidCallback? onHitTestCallback;

  @override
  _RenderHitTestCounter createRenderObject(BuildContext context) {
    return _RenderHitTestCounter().._onHitTestCallback = onHitTestCallback;
  }

  @override
  void updateRenderObject(BuildContext context, _RenderHitTestCounter renderObject) {
    renderObject._onHitTestCallback = onHitTestCallback;
  }
}

class _RenderHitTestCounter extends RenderProxyBox {
  VoidCallback? _onHitTestCallback;

  @override
  bool hitTestChildren(BoxHitTestResult result, {required Offset position}) {
    _onHitTestCallback?.call();
    return super.hitTestChildren(result, position: position);
  }
}
