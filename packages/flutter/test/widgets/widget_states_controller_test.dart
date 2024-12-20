// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  test('WidgetStatesController constructor', () {
    expect(WidgetStatesController().value, <WidgetState>{});
    expect(WidgetStatesController(<WidgetState>{}).value, <WidgetState>{});
    expect(WidgetStatesController(<WidgetState>{WidgetState.selected}).value, <WidgetState>{
      WidgetState.selected,
    });
  });

  test('WidgetStatesController dispatches memory events', () async {
    await expectLater(
      await memoryEvents(() => WidgetStatesController().dispose(), WidgetStatesController),
      areCreateAndDispose,
    );
  });

  test('WidgetStatesController update, listener', () {
    int count = 0;
    void valueChanged() {
      count += 1;
    }

    final WidgetStatesController controller = WidgetStatesController();
    controller.addListener(valueChanged);

    controller.update(WidgetState.selected, true);
    expect(controller.value, <WidgetState>{WidgetState.selected});
    expect(count, 1);
    controller.update(WidgetState.selected, true);
    expect(controller.value, <WidgetState>{WidgetState.selected});
    expect(count, 1);

    controller.update(WidgetState.hovered, false);
    expect(count, 1);
    expect(controller.value, <WidgetState>{WidgetState.selected});
    controller.update(WidgetState.selected, false);
    expect(count, 2);
    expect(controller.value, <WidgetState>{});

    controller.update(WidgetState.hovered, true);
    expect(controller.value, <WidgetState>{WidgetState.hovered});
    expect(count, 3);
    controller.update(WidgetState.hovered, true);
    expect(controller.value, <WidgetState>{WidgetState.hovered});
    expect(count, 3);
    controller.update(WidgetState.pressed, true);
    expect(controller.value, <WidgetState>{WidgetState.hovered, WidgetState.pressed});
    expect(count, 4);
    controller.update(WidgetState.selected, true);
    expect(controller.value, <WidgetState>{
      WidgetState.hovered,
      WidgetState.pressed,
      WidgetState.selected,
    });
    expect(count, 5);
    controller.update(WidgetState.selected, false);
    expect(controller.value, <WidgetState>{WidgetState.hovered, WidgetState.pressed});
    expect(count, 6);
    controller.update(WidgetState.selected, false);
    expect(controller.value, <WidgetState>{WidgetState.hovered, WidgetState.pressed});
    expect(count, 6);
    controller.update(WidgetState.pressed, false);
    expect(controller.value, <WidgetState>{WidgetState.hovered});
    expect(count, 7);
    controller.update(WidgetState.hovered, false);
    expect(controller.value, <WidgetState>{});
    expect(count, 8);

    controller.removeListener(valueChanged);
    controller.update(WidgetState.selected, true);
    expect(controller.value, <WidgetState>{WidgetState.selected});
    expect(count, 8);
  });

  test('WidgetStatesController const initial value', () {
    int count = 0;
    void valueChanged() {
      count += 1;
    }

    final WidgetStatesController controller = WidgetStatesController(const <WidgetState>{
      WidgetState.selected,
    });
    controller.addListener(valueChanged);

    controller.update(WidgetState.selected, true);
    expect(controller.value, <WidgetState>{WidgetState.selected});
    expect(count, 0);

    controller.update(WidgetState.selected, false);
    expect(controller.value, <WidgetState>{});
    expect(count, 1);
  });
}
