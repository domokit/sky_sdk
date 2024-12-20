// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';
import 'package:flutter/src/foundation/platform.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/editable_text_utils.dart';

Finder findCupertinoOverflowNextButton() {
  return find.byWidgetPredicate((Widget widget) {
    return widget is CustomPaint &&
        '${widget.painter?.runtimeType}' == '_RightCupertinoChevronPainter';
  });
}

Finder findCupertinoOverflowBackButton() {
  return find.byWidgetPredicate((Widget widget) {
    return widget is CustomPaint &&
        '${widget.painter?.runtimeType}' == '_LeftCupertinoChevronPainter';
  });
}

Future<void> tapCupertinoOverflowNextButton(WidgetTester tester) async {
  await tester.tapAt(tester.getCenter(findCupertinoOverflowNextButton()));
  await tester.pumpAndSettle();
}

void expectNoCupertinoToolbar() {
  expect(find.byType(CupertinoButton), findsNothing);
}

// Check that the Cupertino text selection toolbars show the expected buttons
// when the content is partially selected.
void expectCupertinoToolbarForPartialSelection() {
  if (isContextMenuProvidedByPlatform) {
    expectNoCupertinoToolbar();
    return;
  }

  switch (defaultTargetPlatform) {
    case TargetPlatform.android:
      expect(find.byType(CupertinoButton), findsNWidgets(5));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Share...'), findsOneWidget);
      expect(find.text('Select All'), findsOneWidget);
    case TargetPlatform.iOS:
      expect(find.byType(CupertinoButton), findsNWidgets(6));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Share...'), findsOneWidget);
      expect(find.text('Look Up'), findsOneWidget);
      expect(find.text('Search Web'), findsOneWidget);
    case TargetPlatform.macOS:
      expect(find.byType(CupertinoButton), findsNWidgets(3));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
    case TargetPlatform.fuchsia:
    case TargetPlatform.linux:
    case TargetPlatform.windows:
      expect(find.byType(CupertinoButton), findsNWidgets(4));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Select All'), findsOneWidget);
  }
}

// Check that the Cupertino text selection toolbar shows the expected buttons
// when the content is fully selected.
void expectCupertinoToolbarForFullSelection() {
  if (isContextMenuProvidedByPlatform) {
    expectNoCupertinoToolbar();
    return;
  }

  switch (defaultTargetPlatform) {
    case TargetPlatform.android:
      expect(find.byType(CupertinoButton), findsNWidgets(4));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Share...'), findsOneWidget);
    case TargetPlatform.iOS:
      expect(find.byType(CupertinoButton), findsNWidgets(6));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Share...'), findsOneWidget);
      expect(find.text('Look Up'), findsOneWidget);
      expect(find.text('Search Web'), findsOneWidget);
    case TargetPlatform.fuchsia:
    case TargetPlatform.linux:
    case TargetPlatform.macOS:
    case TargetPlatform.windows:
      expect(find.byType(CupertinoButton), findsNWidgets(3));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
  }
}

// Check that the Cupertino text selection toolbar is correct for a collapsed selection.
void expectCupertinoToolbarForCollapsedSelection() {
  if (isContextMenuProvidedByPlatform) {
    expectNoCupertinoToolbar();
    return;
  }

  switch (defaultTargetPlatform) {
    case TargetPlatform.android:
      expect(find.byType(CupertinoButton), findsNWidgets(4));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Share...'), findsOneWidget);
    case TargetPlatform.iOS:
      expect(find.byType(CupertinoButton), findsNWidgets(2));
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Select All'), findsOneWidget);
    case TargetPlatform.fuchsia:
    case TargetPlatform.linux:
    case TargetPlatform.windows:
    case TargetPlatform.macOS:
      expect(find.byType(CupertinoButton), findsNWidgets(1));
      expect(find.text('Paste'), findsOneWidget);
  }
}

void expectNoMaterialToolbar() {
  expect(find.byType(TextButton), findsNothing);
}

// Check that the Material text selection toolbars show the expected buttons
// when the content is partially selected.
void expectMaterialToolbarForPartialSelection() {
  if (isContextMenuProvidedByPlatform) {
    expectNoMaterialToolbar();
    return;
  }

  switch (defaultTargetPlatform) {
    case TargetPlatform.android:
      expect(find.byType(TextButton), findsNWidgets(5));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Share'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);
    case TargetPlatform.iOS:
    case TargetPlatform.macOS:
    case TargetPlatform.fuchsia:
    case TargetPlatform.linux:
    case TargetPlatform.windows:
      expect(find.byType(TextButton), findsNWidgets(4));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Select all'), findsOneWidget);
  }
}

// Check that the Material text selection toolbar shows the expected buttons
// when the content is fully selected.
void expectMaterialToolbarForFullSelection() {
  if (isContextMenuProvidedByPlatform) {
    expectNoMaterialToolbar();
    return;
  }

  switch (defaultTargetPlatform) {
    case TargetPlatform.android:
      expect(find.byType(TextButton), findsNWidgets(4));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
      expect(find.text('Share'), findsOneWidget);
    case TargetPlatform.iOS:
    case TargetPlatform.fuchsia:
    case TargetPlatform.linux:
    case TargetPlatform.macOS:
    case TargetPlatform.windows:
      expect(find.byType(TextButton), findsNWidgets(3));
      expect(find.text('Cut'), findsOneWidget);
      expect(find.text('Copy'), findsOneWidget);
      expect(find.text('Paste'), findsOneWidget);
  }
}

Finder findMaterialOverflowNextButton() {
  return find.byKey(StandardComponentType.moreButton.key);
}

Finder findMaterialOverflowBackButton() {
  return find.byKey(StandardComponentType.backButton.key);
}

Future<void> tapMaterialOverflowNextButton(WidgetTester tester) async {
  await tester.tapAt(tester.getCenter(findMaterialOverflowNextButton()));
  await tester.pumpAndSettle();
}
