// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

int _creations = 0;
int _disposals = 0;

void main() {
  // LeakTesting is turned off because it adds subscriptions to
  // [FlutterMemoryAllocations], that may interfere with the tests.
  LeakTesting.settings = LeakTesting.settings.withIgnoredAll();

  final FlutterMemoryAllocations ma = FlutterMemoryAllocations.instance;

  test('Publishers dispatch events in debug mode', () async {
    void listener(ObjectEvent event) {
      if (event is ObjectDisposed) {
        _disposals++;
      }
      if (event is ObjectCreated) {
        _creations++;
      }
    }

    ma.addListener(listener);

    final _EventStats actual = await _activateFlutterObjectsAndReturnCountOfEvents();
    expect(actual.creations, _creations);
    expect(actual.disposals, _disposals);

    ma.removeListener(listener);
    expect(ma.hasListeners, isFalse);
  });

  testWidgets('State dispatches events in debug mode', (WidgetTester tester) async {
    bool stateCreated = false;
    bool stateDisposed = false;

    expect(ma.hasListeners, false);

    void listener(ObjectEvent event) {
      if (event is ObjectCreated && event.object is State) {
        stateCreated = true;
      }
      if (event is ObjectDisposed && event.object is State) {
        stateDisposed = true;
      }
    }

    ma.addListener(listener);

    await tester.pumpWidget(const _TestStatefulWidget());
    expect(stateCreated, isTrue);
    expect(stateDisposed, isFalse);
    await tester.pumpWidget(const SizedBox.shrink());

    expect(stateCreated, isTrue);
    expect(stateDisposed, isTrue);
    ma.removeListener(listener);
    expect(ma.hasListeners, false);
  });
}

class _TestLeafRenderObjectWidget extends LeafRenderObjectWidget {
  @override
  RenderObject createRenderObject(BuildContext context) {
    return _TestRenderObject();
  }
}

class _TestElement extends RenderObjectElement with RootElementMixin {
  _TestElement() : super(_TestLeafRenderObjectWidget());

  void makeInactive() {
    final FocusManager newFocusManager = FocusManager();
    assignOwner(BuildOwner(focusManager: newFocusManager));
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

class _TestStatefulWidget extends StatefulWidget {
  const _TestStatefulWidget();

  @override
  State<_TestStatefulWidget> createState() => _TestStatefulWidgetState();
}

class _TestStatefulWidgetState extends State<_TestStatefulWidget> {
  @override
  Widget build(BuildContext context) {
    return Container();
  }
}

class _EventStats {
  int creations = 0;
  int disposals = 0;
}

/// Create and dispose Flutter objects to fire memory allocation events.
Future<_EventStats> _activateFlutterObjectsAndReturnCountOfEvents() async {
  final _EventStats result = _EventStats();

  final _TestElement element = _TestElement();
  result.creations++;
  final RenderObject renderObject = _TestRenderObject();
  result.creations++;

  element.makeInactive();
  result.creations +=
      4; // 1 for the new BuildOwner, 1 for the new FocusManager, 1 for the new FocusScopeNode, 1 for the new _HighlightModeManager
  element.unmount();
  result.disposals += 2; // 1 for the old BuildOwner, 1 for the element
  renderObject.dispose();
  result.disposals += 1;

  return result;
}
