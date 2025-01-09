// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

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
