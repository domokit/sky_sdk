// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:ui';

import 'package:flutter_test/flutter_test.dart';

class FakeView extends TestFlutterView {
  FakeView(FlutterView view, {this.viewId = 100})
    : super(
        view: view,
        platformDispatcher: view.platformDispatcher as TestPlatformDispatcher,
        display: view.display as TestDisplay,
      );

  @override
  final int viewId;

  @override
  void render(Scene scene, {Size? size}) {
    // Do not render the scene in the engine. The engine only observes one
    // instance of FlutterView (the _view), and it is generally expected that
    // the framework will render no more than one `Scene` per frame.
  }

  @override
  void updateSemantics(SemanticsUpdate update) {
    // Do not send the update to the engine. The engine only observes one
    // instance of FlutterView (the _view). Sending semantic updates meant for
    // different views to the same engine view does not work as the updates do
    // not produce consistent semantics trees.
  }
}

/// A test platform dispatcher that can show/hide its underlying `implicitView`,
/// depending on the value of the [implicitViewHidden] flag.
class NoImplicitViewPlatformDispatcher extends TestPlatformDispatcher {
  NoImplicitViewPlatformDispatcher({required super.platformDispatcher})
    : superPlatformDispatcher = platformDispatcher;

  final PlatformDispatcher superPlatformDispatcher;

  bool implicitViewHidden = false;

  @override
  TestFlutterView? get implicitView {
    return implicitViewHidden ? null : superPlatformDispatcher.implicitView as TestFlutterView?;
  }
}

/// Test Flutter Bindings that allow tests to hide/show the `implicitView`
/// of their [NoImplicitViewPlatformDispatcher] `platformDispatcher`.
///
/// This is used to test that [runApp] throws an assertion error with an
/// explanation when used when the `implicitView` is disabled (like in Flutter
/// web when multi-view is enabled).
///
/// Because of how [testWidgets] uses `runApp` internally to manage the lifecycle
/// of a test, the implicitView must be disabled/reenabled inside of the body of
/// the [WidgetTesterCallback] under test. In practice: the implicitView is disabled
/// in the first line of the test, and reenabled in the last.
///
/// See: multi_view_no_implicitView_binding_test.dart
class NoImplicitViewWidgetsBinding extends AutomatedTestWidgetsFlutterBinding {
  late final NoImplicitViewPlatformDispatcher _platformDispatcher =
      NoImplicitViewPlatformDispatcher(platformDispatcher: super.platformDispatcher);

  @override
  NoImplicitViewPlatformDispatcher get platformDispatcher => _platformDispatcher;

  void hideImplicitView() {
    platformDispatcher.implicitViewHidden = true;
  }

  void showImplicitView() {
    platformDispatcher.implicitViewHidden = false;
  }
}
