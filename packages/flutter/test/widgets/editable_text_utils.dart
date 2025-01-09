// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

/// On web, the context menu (aka toolbar) is provided by the browser.
const bool isContextMenuProvidedByPlatform = isBrowser;

// Returns the RenderEditable at the given index, or the first if not given.
RenderEditable findRenderEditable(WidgetTester tester, {int index = 0}) {
  final RenderObject root = tester.renderObject(find.byType(EditableText).at(index));
  expect(root, isNotNull);

  late RenderEditable renderEditable;
  void recursiveFinder(RenderObject child) {
    if (child is RenderEditable) {
      renderEditable = child;
      return;
    }
    child.visitChildren(recursiveFinder);
  }

  root.visitChildren(recursiveFinder);
  expect(renderEditable, isNotNull);
  return renderEditable;
}

List<TextSelectionPoint> globalize(Iterable<TextSelectionPoint> points, RenderBox box) {
  return points.map<TextSelectionPoint>((TextSelectionPoint point) {
    return TextSelectionPoint(box.localToGlobal(point.point), point.direction);
  }).toList();
}

Offset textOffsetToPosition(WidgetTester tester, int offset, {int index = 0}) {
  final RenderEditable renderEditable = findRenderEditable(tester, index: index);
  final List<TextSelectionPoint> endpoints = globalize(
    renderEditable.getEndpointsForSelection(TextSelection.collapsed(offset: offset)),
    renderEditable,
  );
  expect(endpoints.length, 1);
  return endpoints[0].point + const Offset(kIsWeb ? 1.0 : 0.0, -2.0);
}

/// Mimic key press events by sending key down and key up events via the [tester].
Future<void> sendKeys(
  WidgetTester tester,
  List<LogicalKeyboardKey> keys, {
  bool shift = false,
  bool wordModifier = false,
  bool lineModifier = false,
  bool shortcutModifier = false,
  required TargetPlatform targetPlatform,
}) async {
  final String targetPlatformString = targetPlatform.toString();
  final String platform =
      targetPlatformString.substring(targetPlatformString.indexOf('.') + 1).toLowerCase();
  if (shift) {
    await tester.sendKeyDownEvent(LogicalKeyboardKey.shiftLeft, platform: platform);
  }
  if (shortcutModifier) {
    await tester.sendKeyDownEvent(
      platform == 'macos' || platform == 'ios'
          ? LogicalKeyboardKey.metaLeft
          : LogicalKeyboardKey.controlLeft,
      platform: platform,
    );
  }
  if (wordModifier) {
    await tester.sendKeyDownEvent(
      platform == 'macos' || platform == 'ios'
          ? LogicalKeyboardKey.altLeft
          : LogicalKeyboardKey.controlLeft,
      platform: platform,
    );
  }
  if (lineModifier) {
    await tester.sendKeyDownEvent(
      platform == 'macos' || platform == 'ios'
          ? LogicalKeyboardKey.metaLeft
          : LogicalKeyboardKey.altLeft,
      platform: platform,
    );
  }
  for (final LogicalKeyboardKey key in keys) {
    await tester.sendKeyEvent(key, platform: platform);
    await tester.pump();
  }
  if (lineModifier) {
    await tester.sendKeyUpEvent(
      platform == 'macos' || platform == 'ios'
          ? LogicalKeyboardKey.metaLeft
          : LogicalKeyboardKey.altLeft,
      platform: platform,
    );
  }
  if (wordModifier) {
    await tester.sendKeyUpEvent(
      platform == 'macos' || platform == 'ios'
          ? LogicalKeyboardKey.altLeft
          : LogicalKeyboardKey.controlLeft,
      platform: platform,
    );
  }
  if (shortcutModifier) {
    await tester.sendKeyUpEvent(
      platform == 'macos' || platform == 'ios'
          ? LogicalKeyboardKey.metaLeft
          : LogicalKeyboardKey.controlLeft,
      platform: platform,
    );
  }
  if (shift) {
    await tester.sendKeyUpEvent(LogicalKeyboardKey.shiftLeft, platform: platform);
  }
  if (shift || wordModifier || lineModifier) {
    await tester.pump();
  }
}

// Simple controller that builds a WidgetSpan with 100 height.
class OverflowWidgetTextEditingController extends TextEditingController {
  @override
  TextSpan buildTextSpan({
    required BuildContext context,
    TextStyle? style,
    required bool withComposing,
  }) {
    return TextSpan(
      style: style,
      children: <InlineSpan>[
        const TextSpan(text: 'Hi'),
        WidgetSpan(child: Container(color: Colors.redAccent, height: 100.0)),
      ],
    );
  }
}
