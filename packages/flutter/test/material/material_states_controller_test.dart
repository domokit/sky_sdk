// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  test('MaterialStatesController constructor', () {
    expect(MaterialStatesController().value, <MaterialState>{});
    expect(MaterialStatesController(<MaterialState>{}).value, <MaterialState>{});
    expect(MaterialStatesController(<MaterialState>{MaterialState.selected}).value, <MaterialState>{
      MaterialState.selected,
    });
  });

  test('MaterialStatesController dispatches memory events', () async {
    await expectLater(
      await memoryEvents(() => MaterialStatesController().dispose(), MaterialStatesController),
      areCreateAndDispose,
    );
  });

  test('MaterialStatesController update, listener', () {
    int count = 0;
    void valueChanged() {
      count += 1;
    }

    final MaterialStatesController controller = MaterialStatesController();
    controller.addListener(valueChanged);

    controller.update(MaterialState.selected, true);
    expect(controller.value, <MaterialState>{MaterialState.selected});
    expect(count, 1);
    controller.update(MaterialState.selected, true);
    expect(controller.value, <MaterialState>{MaterialState.selected});
    expect(count, 1);

    controller.update(MaterialState.hovered, false);
    expect(count, 1);
    expect(controller.value, <MaterialState>{MaterialState.selected});
    controller.update(MaterialState.selected, false);
    expect(count, 2);
    expect(controller.value, <MaterialState>{});

    controller.update(MaterialState.hovered, true);
    expect(controller.value, <MaterialState>{MaterialState.hovered});
    expect(count, 3);
    controller.update(MaterialState.hovered, true);
    expect(controller.value, <MaterialState>{MaterialState.hovered});
    expect(count, 3);
    controller.update(MaterialState.pressed, true);
    expect(controller.value, <MaterialState>{MaterialState.hovered, MaterialState.pressed});
    expect(count, 4);
    controller.update(MaterialState.selected, true);
    expect(controller.value, <MaterialState>{
      MaterialState.hovered,
      MaterialState.pressed,
      MaterialState.selected,
    });
    expect(count, 5);
    controller.update(MaterialState.selected, false);
    expect(controller.value, <MaterialState>{MaterialState.hovered, MaterialState.pressed});
    expect(count, 6);
    controller.update(MaterialState.selected, false);
    expect(controller.value, <MaterialState>{MaterialState.hovered, MaterialState.pressed});
    expect(count, 6);
    controller.update(MaterialState.pressed, false);
    expect(controller.value, <MaterialState>{MaterialState.hovered});
    expect(count, 7);
    controller.update(MaterialState.hovered, false);
    expect(controller.value, <MaterialState>{});
    expect(count, 8);

    controller.removeListener(valueChanged);
    controller.update(MaterialState.selected, true);
    expect(controller.value, <MaterialState>{MaterialState.selected});
    expect(count, 8);
  });

  test('MaterialStatesController const initial value', () {
    int count = 0;
    void valueChanged() {
      count += 1;
    }

    final MaterialStatesController controller = MaterialStatesController(const <MaterialState>{
      MaterialState.selected,
    });
    controller.addListener(valueChanged);

    controller.update(MaterialState.selected, true);
    expect(controller.value, <MaterialState>{MaterialState.selected});
    expect(count, 0);

    controller.update(MaterialState.selected, false);
    expect(controller.value, <MaterialState>{});
    expect(count, 1);
  });
}
