// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

Finder findAppBarMaterial() {
  return find.descendant(of: find.byType(AppBar), matching: find.byType(Material)).first;
}

Color? getAppBarBackgroundColor(WidgetTester tester) {
  return tester.widget<Material>(findAppBarMaterial()).color;
}

double appBarHeight(WidgetTester tester) {
  return tester.getSize(find.byType(AppBar, skipOffstage: false)).height;
}

double appBarTop(WidgetTester tester) {
  return tester.getTopLeft(find.byType(AppBar, skipOffstage: false)).dy;
}

double appBarBottom(WidgetTester tester) {
  return tester.getBottomLeft(find.byType(AppBar, skipOffstage: false)).dy;
}

double tabBarHeight(WidgetTester tester) {
  return tester.getSize(find.byType(TabBar, skipOffstage: false)).height;
}

ScrollController primaryScrollController(WidgetTester tester) {
  return PrimaryScrollController.of(tester.element(find.byType(CustomScrollView)));
}

void verifyTextNotClipped(Finder textFinder, WidgetTester tester) {
  final Rect clipRect = tester.getRect(
    find.ancestor(of: textFinder, matching: find.byType(ClipRect)).first,
  );
  final Rect textRect = tester.getRect(textFinder);
  expect(textRect.top, inInclusiveRange(clipRect.top, clipRect.bottom));
  expect(textRect.bottom, inInclusiveRange(clipRect.top, clipRect.bottom));
  expect(textRect.left, inInclusiveRange(clipRect.left, clipRect.right));
  expect(textRect.right, inInclusiveRange(clipRect.left, clipRect.right));
}
