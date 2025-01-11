// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter_test/flutter_test.dart';

typedef _Create<T extends GestureDetailsWithPositions> =
    T Function(Offset globalPosition, {Offset? localPosition});

void main() {
  test('GestureDetailsWithPositions is correctly extended by other gestures', () {
    final Set<_Create> creates = <_Create>{
      (Offset globalPosition, {Offset? localPosition}) =>
          DragDownDetails(globalPosition: globalPosition, localPosition: localPosition),
      (Offset globalPosition, {Offset? localPosition}) =>
          DragStartDetails(globalPosition: globalPosition, localPosition: localPosition),
      (Offset globalPosition, {Offset? localPosition}) =>
          DragUpdateDetails(globalPosition: globalPosition, localPosition: localPosition),
      (Offset globalPosition, {Offset? localPosition}) =>
          DragEndDetails(globalPosition: globalPosition, localPosition: localPosition),
      (Offset globalPosition, {Offset? localPosition}) => ForcePressDetails(
        globalPosition: globalPosition,
        localPosition: localPosition,
        pressure: 0,
      ),
      (Offset globalPosition, {Offset? localPosition}) =>
          LongPressDownDetails(globalPosition: globalPosition, localPosition: localPosition),
      (Offset globalPosition, {Offset? localPosition}) =>
          LongPressStartDetails(globalPosition: globalPosition, localPosition: localPosition),
      (Offset globalPosition, {Offset? localPosition}) =>
          LongPressMoveUpdateDetails(globalPosition: globalPosition, localPosition: localPosition),
      (Offset globalPosition, {Offset? localPosition}) =>
          LongPressEndDetails(globalPosition: globalPosition, localPosition: localPosition),
      (Offset globalPosition, {Offset? localPosition}) => SerialTapDownDetails(
        globalPosition: globalPosition,
        localPosition: localPosition,
        kind: PointerDeviceKind.unknown,
      ),
      (Offset globalPosition, {Offset? localPosition}) =>
          SerialTapUpDetails(globalPosition: globalPosition, localPosition: localPosition),
      (Offset globalPosition, {Offset? localPosition}) =>
          TapDownDetails(globalPosition: globalPosition, localPosition: localPosition),
      (Offset globalPosition, {Offset? localPosition}) => TapUpDetails(
        globalPosition: globalPosition,
        localPosition: localPosition,
        kind: PointerDeviceKind.unknown,
      ),
      (Offset globalPosition, {Offset? localPosition}) => TapDragDownDetails(
        globalPosition: globalPosition,
        localPosition: localPosition ?? globalPosition,
        consecutiveTapCount: 0,
      ),
      (Offset globalPosition, {Offset? localPosition}) => TapDragUpDetails(
        globalPosition: globalPosition,
        localPosition: localPosition ?? globalPosition,
        kind: PointerDeviceKind.unknown,
        consecutiveTapCount: 0,
      ),
      (Offset globalPosition, {Offset? localPosition}) => TapDragStartDetails(
        globalPosition: globalPosition,
        localPosition: localPosition ?? globalPosition,
        consecutiveTapCount: 0,
      ),
      (Offset globalPosition, {Offset? localPosition}) => TapDragUpdateDetails(
        globalPosition: globalPosition,
        localPosition: localPosition ?? globalPosition,
        consecutiveTapCount: 0,
        offsetFromOrigin: Offset.zero,
        localOffsetFromOrigin: Offset.zero,
      ),
      (Offset globalPosition, {Offset? localPosition}) => TapDragEndDetails(
        globalPosition: globalPosition,
        localPosition: localPosition ?? globalPosition,
        consecutiveTapCount: 0,
      ),
    };

    for (final _Create create in creates) {
      final GestureDetailsWithPositions gesture1 = create(const Offset(0, 100));
      expect(gesture1.globalPosition, const Offset(0, 100));
      expect(gesture1.localPosition, const Offset(0, 100));
      final GestureDetailsWithPositions gesture2 = create(
        const Offset(0, 100),
        localPosition: const Offset(0, 200),
      );
      expect(gesture2.globalPosition, const Offset(0, 100));
      expect(gesture2.localPosition, const Offset(0, 200));
    }
  });
}
