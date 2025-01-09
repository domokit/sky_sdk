// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final FlutterMemoryAllocations ma = FlutterMemoryAllocations.instance;

  setUp(() {
    assert(!ma.hasListeners);
  });

  testWidgets('$FlutterMemoryAllocations is noop when kFlutterMemoryAllocationsEnabled is false.', (
    WidgetTester tester,
  ) async {
    ObjectEvent? receivedEvent;
    ObjectEvent listener(ObjectEvent event) => receivedEvent = event;

    ma.addListener(listener);
    expect(ma.hasListeners, isFalse);

    await _activateFlutterObjects(tester);
    expect(receivedEvent, isNull);
    expect(ma.hasListeners, isFalse);

    ma.removeListener(listener);
  });
}

class _TestLeafRenderObjectWidget extends LeafRenderObjectWidget {
  @override
  RenderObject createRenderObject(BuildContext context) {
    return _TestRenderObject();
  }
}

class _TestRenderObject extends RenderObject {
  @override
  void debugAssertDoesMeetConstraints() {}

  @override
  Rect get paintBounds => throw UnimplementedError();

  @override
  void performLayout() {}

  @override
  void performResize() {}

  @override
  Rect get semanticBounds => throw UnimplementedError();
}

class _TestElement extends RenderTreeRootElement with RootElementMixin {
  _TestElement() : super(_TestLeafRenderObjectWidget());

  void makeInactive() {
    assignOwner(BuildOwner(focusManager: FocusManager()));
    mount(null, null);
    deactivate();
  }

  @override
  void insertRenderObjectChild(covariant RenderObject child, covariant Object? slot) {}

  @override
  void moveRenderObjectChild(
    covariant RenderObject child,
    covariant Object? oldSlot,
    covariant Object? newSlot,
  ) {}

  @override
  void removeRenderObjectChild(covariant RenderObject child, covariant Object? slot) {}
}

class _MyStatefulWidget extends StatefulWidget {
  const _MyStatefulWidget();

  @override
  State<_MyStatefulWidget> createState() => _MyStatefulWidgetState();
}

class _MyStatefulWidgetState extends State<_MyStatefulWidget> {
  @override
  Widget build(BuildContext context) {
    return Container();
  }
}

/// Create and dispose Flutter objects to fire memory allocation events.
Future<void> _activateFlutterObjects(WidgetTester tester) async {
  final _TestElement element = _TestElement();
  element.makeInactive();
  element.unmount();

  // Create and dispose State:
  await tester.pumpWidget(const _MyStatefulWidget());
  await tester.pumpWidget(const SizedBox.shrink());
}
