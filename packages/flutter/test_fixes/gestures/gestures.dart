// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';

void main() {
  // Change made in https://github.com/flutter/flutter/pull/28602
  final PointerEnterEvent enterEvent = PointerEnterEvent.fromHoverEvent(
    PointerHoverEvent(),
  );

  // Change made in https://github.com/flutter/flutter/pull/28602
  final PointerExitEvent exitEvent = PointerExitEvent.fromHoverEvent(
    PointerHoverEvent(),
  );

  // Changes made in https://github.com/flutter/flutter/pull/66043
  VelocityTracker tracker = VelocityTracker();
  tracker = VelocityTracker(PointerDeviceKind.mouse);
  tracker = VelocityTracker(error: '');

  // Changes made in https://github.com/flutter/flutter/pull/81858
  DragGestureRecognizer();
  DragGestureRecognizer(kind: PointerDeviceKind.touch);
  DragGestureRecognizer(error: '');
  VerticalDragGestureRecognizer();
  VerticalDragGestureRecognizer(kind: PointerDeviceKind.touch);
  VerticalDragGestureRecognizer(error: '');
  HorizontalDragGestureRecognizer();
  HorizontalDragGestureRecognizer(kind: PointerDeviceKind.touch);
  HorizontalDragGestureRecognizer(error: '');
  GestureRecognizer();
  GestureRecognizer(kind: PointerDeviceKind.touch);
  GestureRecognizer(error: '');
  OneSequenceGestureRecognizer();
  OneSequenceGestureRecognizer(kind: PointerDeviceKind.touch);
  OneSequenceGestureRecognizer(error: '');
  PrimaryPointerGestureRecognizer();
  PrimaryPointerGestureRecognizer(kind: PointerDeviceKind.touch);
  PrimaryPointerGestureRecognizer(error: '');
  EagerGestureRecognizer();
  EagerGestureRecognizer(kind: PointerDeviceKind.touch);
  EagerGestureRecognizer(error: '');
  ForcePressGestureRecognizer();
  ForcePressGestureRecognizer(kind: PointerDeviceKind.touch);
  ForcePressGestureRecognizer(error: '');
  LongPressGestureRecognizer();
  LongPressGestureRecognizer(kind: PointerDeviceKind.touch);
  LongPressGestureRecognizer(error: '');
  MultiDragGestureRecognizer();
  MultiDragGestureRecognizer(kind: PointerDeviceKind.touch);
  MultiDragGestureRecognizer(error: '');
  ImmediateMultiDragGestureRecognizer();
  ImmediateMultiDragGestureRecognizer(kind: PointerDeviceKind.touch);
  ImmediateMultiDragGestureRecognizer(error: '');
  HorizontalMultiDragGestureRecognizer();
  HorizontalMultiDragGestureRecognizer(kind: PointerDeviceKind.touch);
  HorizontalMultiDragGestureRecognizer(error: '');
  VerticalMultiDragGestureRecognizer();
  VerticalMultiDragGestureRecognizer(kind: PointerDeviceKind.touch);
  VerticalMultiDragGestureRecognizer(error: '');
  DelayedMultiDragGestureRecognizer();
  DelayedMultiDragGestureRecognizer(kind: PointerDeviceKind.touch);
  DelayedMultiDragGestureRecognizer(error: '');
  DoubleTapGestureRecognizer();
  DoubleTapGestureRecognizer(kind: PointerDeviceKind.touch);
  DoubleTapGestureRecognizer(error: '');
  MultiTapGestureRecognizer();
  MultiTapGestureRecognizer(kind: PointerDeviceKind.touch);
  MultiTapGestureRecognizer(error: '');
  ScaleGestureRecognizer();
  ScaleGestureRecognizer(kind: PointerDeviceKind.touch);
  ScaleGestureRecognizer(error: '');
}
