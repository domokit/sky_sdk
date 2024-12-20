// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:collection/collection.dart';

// Android MotionEvent actions for which a pointer index is encoded in the
// unmasked action code.
const List<int> kPointerActions = <int>[
  0, // DOWN
  1, // UP
  5, // POINTER_DOWN
  6, // POINTER_UP
];

const double kDoubleErrorMargin = 1e-4;

String diffMotionEvents(Map<String, dynamic> originalEvent, Map<String, dynamic> synthesizedEvent) {
  final StringBuffer diff = StringBuffer();

  diffMaps(
    originalEvent,
    synthesizedEvent,
    diff,
    excludeKeys: const <String>[
      'pointerProperties', // Compared separately.
      'pointerCoords', // Compared separately.
      'source', // Unused by Flutter.
      'deviceId', // Android documentation says that's an arbitrary number that shouldn't be depended on.
      'action', // Compared separately.
    ],
  );

  diffActions(diff, originalEvent, synthesizedEvent);
  diffPointerProperties(diff, originalEvent, synthesizedEvent);
  diffPointerCoordsList(diff, originalEvent, synthesizedEvent);

  return diff.toString();
}

void diffActions(
  StringBuffer diffBuffer,
  Map<String, dynamic> originalEvent,
  Map<String, dynamic> synthesizedEvent,
) {
  final int synthesizedActionMasked = getActionMasked(synthesizedEvent['action'] as int);
  final int originalActionMasked = getActionMasked(originalEvent['action'] as int);
  final String synthesizedActionName = getActionName(
    synthesizedActionMasked,
    synthesizedEvent['action'] as int,
  );
  final String originalActionName = getActionName(
    originalActionMasked,
    originalEvent['action'] as int,
  );

  if (synthesizedActionMasked != originalActionMasked) {
    diffBuffer.write('action (expected: $originalActionName actual: $synthesizedActionName) ');
  }

  if (kPointerActions.contains(originalActionMasked) &&
      originalActionMasked == synthesizedActionMasked) {
    final int originalPointer = getPointerIdx(originalEvent['action'] as int);
    final int synthesizedPointer = getPointerIdx(synthesizedEvent['action'] as int);
    if (originalPointer != synthesizedPointer) {
      diffBuffer.write(
        'pointerIdx (expected: $originalPointer actual: $synthesizedPointer action: $originalActionName ',
      );
    }
  }
}

void diffPointerProperties(
  StringBuffer diffBuffer,
  Map<String, dynamic> originalEvent,
  Map<String, dynamic> synthesizedEvent,
) {
  final List<Map<dynamic, dynamic>> expectedList =
      (originalEvent['pointerProperties'] as List<dynamic>).cast<Map<dynamic, dynamic>>();
  final List<Map<dynamic, dynamic>> actualList =
      (synthesizedEvent['pointerProperties'] as List<dynamic>).cast<Map<dynamic, dynamic>>();

  if (expectedList.length != actualList.length) {
    diffBuffer.write(
      'pointerProperties (actual length: ${actualList.length}, expected length: ${expectedList.length} ',
    );
    return;
  }

  for (int i = 0; i < expectedList.length; i++) {
    final Map<String, dynamic> expected = expectedList[i].cast<String, dynamic>();
    final Map<String, dynamic> actual = actualList[i].cast<String, dynamic>();
    diffMaps(expected, actual, diffBuffer, messagePrefix: '[pointerProperty $i] ');
  }
}

void diffPointerCoordsList(
  StringBuffer diffBuffer,
  Map<String, dynamic> originalEvent,
  Map<String, dynamic> synthesizedEvent,
) {
  final List<Map<dynamic, dynamic>> expectedList =
      (originalEvent['pointerCoords'] as List<dynamic>).cast<Map<dynamic, dynamic>>();
  final List<Map<dynamic, dynamic>> actualList =
      (synthesizedEvent['pointerCoords'] as List<dynamic>).cast<Map<dynamic, dynamic>>();

  if (expectedList.length != actualList.length) {
    diffBuffer.write(
      'pointerCoords (actual length: ${actualList.length}, expected length: ${expectedList.length} ',
    );
    return;
  }

  for (int i = 0; i < expectedList.length; i++) {
    final Map<String, dynamic> expected = expectedList[i].cast<String, dynamic>();
    final Map<String, dynamic> actual = actualList[i].cast<String, dynamic>();
    diffPointerCoords(expected, actual, i, diffBuffer);
  }
}

void diffPointerCoords(
  Map<String, dynamic> expected,
  Map<String, dynamic> actual,
  int pointerIdx,
  StringBuffer diffBuffer,
) {
  diffMaps(expected, actual, diffBuffer, messagePrefix: '[pointerCoord $pointerIdx] ');
}

void diffMaps(
  Map<String, dynamic> expected,
  Map<String, dynamic> actual,
  StringBuffer diffBuffer, {
  List<String> excludeKeys = const <String>[],
  String messagePrefix = '',
}) {
  const IterableEquality<String> eq = IterableEquality<String>();
  if (!eq.equals(expected.keys, actual.keys)) {
    diffBuffer.write('${messagePrefix}keys (expected: ${expected.keys} actual: ${actual.keys} ');
    return;
  }
  for (final String key in expected.keys) {
    if (excludeKeys.contains(key)) {
      continue;
    }
    if (doublesApproximatelyMatch(expected[key], actual[key])) {
      continue;
    }

    if (expected[key] != actual[key]) {
      diffBuffer.write('$messagePrefix$key (expected: ${expected[key]} actual: ${actual[key]}) ');
    }
  }
}

int getActionMasked(int action) => action & 0xff;

int getPointerIdx(int action) => (action >> 8) & 0xff;

String getActionName(int actionMasked, int action) {
  const List<String> actionNames = <String>[
    'DOWN',
    'UP',
    'MOVE',
    'CANCEL',
    'OUTSIDE',
    'POINTER_DOWN',
    'POINTER_UP',
    'HOVER_MOVE',
    'SCROLL',
    'HOVER_ENTER',
    'HOVER_EXIT',
    'BUTTON_PRESS',
    'BUTTON_RELEASE',
  ];
  if (actionMasked < actionNames.length) {
    return '${actionNames[actionMasked]}($action)';
  } else {
    return 'ACTION_$actionMasked';
  }
}

bool doublesApproximatelyMatch(dynamic a, dynamic b) =>
    a is double && b is double && (a - b).abs() < kDoubleErrorMargin;
