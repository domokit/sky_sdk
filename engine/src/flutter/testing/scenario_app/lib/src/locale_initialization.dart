// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';
import 'dart:ui';

import 'package:vector_math/vector_math_64.dart';

import 'scenario.dart';

/// Sends the received locale data back as semantics information.
class LocaleInitialization extends Scenario {
  /// Constructor
  LocaleInitialization(super.view);

  int _tapCount = 0;

  /// Start off by sending the supported locales list via semantics.
  @override
  void onBeginFrame(Duration duration) {
    // Doesn't matter what we draw. Just paint white.
    final SceneBuilder builder = SceneBuilder();
    final PictureRecorder recorder = PictureRecorder();
    final Canvas canvas = Canvas(recorder);

    canvas.drawRect(
      Rect.fromLTWH(0, 0, view.physicalSize.width, view.physicalSize.height),
      Paint()..color = const Color.fromARGB(255, 255, 255, 255),
    );
    final Picture picture = recorder.endRecording();

    builder.addPicture(Offset.zero, picture);
    final Scene scene = builder.build();
    view.render(scene);
    scene.dispose();

    // On the first frame, pretend that it drew a text field. Send the
    // corresponding semantics tree comprised of 1 node with the locale data
    // as the label.
    final SemanticsUpdateBuilder semanticsUpdateBuilder =
        SemanticsUpdateBuilder()..updateNode(
          id: 0,
          // SemanticsFlag.isTextField.
          flags: 16,
          // SemanticsAction.tap.
          actions: 1,
          rect: const Rect.fromLTRB(0.0, 0.0, 414.0, 48.0),
          identifier: '',
          label: view.platformDispatcher.locales.toString(),
          labelAttributes: <StringAttribute>[],
          textDirection: TextDirection.ltr,
          textSelectionBase: -1,
          textSelectionExtent: -1,
          platformViewId: -1,
          maxValueLength: -1,
          currentValueLength: 0,
          scrollChildren: 0,
          scrollIndex: 0,
          scrollPosition: 0.0,
          scrollExtentMax: 0.0,
          scrollExtentMin: 0.0,
          transform: Matrix4.identity().storage,
          elevation: 0.0,
          thickness: 0.0,
          hint: '',
          hintAttributes: <StringAttribute>[],
          value: '',
          valueAttributes: <StringAttribute>[],
          increasedValue: '',
          increasedValueAttributes: <StringAttribute>[],
          decreasedValue: '',
          decreasedValueAttributes: <StringAttribute>[],
          tooltip: '',
          childrenInTraversalOrder: Int32List(0),
          childrenInHitTestOrder: Int32List(0),
          additionalActions: Int32List(0),
        );

    final SemanticsUpdate semanticsUpdate = semanticsUpdateBuilder.build();

    view.updateSemantics(semanticsUpdate);
  }

  /// Handle taps.
  ///
  /// Send changing information via semantics on each successive tap.
  @override
  void onPointerDataPacket(PointerDataPacket packet) {
    String label = '';
    switch (_tapCount) {
      case 1:
        {
          // Set label to string data we wish to pass on first frame.
          label = '1';
          break;
        }
      // Expand for other test cases.
    }

    final SemanticsUpdateBuilder semanticsUpdateBuilder =
        SemanticsUpdateBuilder()..updateNode(
          id: 0,
          // SemanticsFlag.isTextField.
          flags: 16,
          // SemanticsAction.tap.
          actions: 1,
          rect: const Rect.fromLTRB(0.0, 0.0, 414.0, 48.0),
          identifier: '',
          label: label,
          labelAttributes: <StringAttribute>[],
          textDirection: TextDirection.ltr,
          textSelectionBase: 0,
          textSelectionExtent: 0,
          platformViewId: -1,
          maxValueLength: -1,
          currentValueLength: 0,
          scrollChildren: 0,
          scrollIndex: 0,
          scrollPosition: 0.0,
          scrollExtentMax: 0.0,
          scrollExtentMin: 0.0,
          transform: Matrix4.identity().storage,
          elevation: 0.0,
          thickness: 0.0,
          hint: '',
          hintAttributes: <StringAttribute>[],
          value: '',
          valueAttributes: <StringAttribute>[],
          increasedValue: '',
          increasedValueAttributes: <StringAttribute>[],
          decreasedValue: '',
          decreasedValueAttributes: <StringAttribute>[],
          tooltip: '',
          childrenInTraversalOrder: Int32List(0),
          childrenInHitTestOrder: Int32List(0),
          additionalActions: Int32List(0),
        );

    final SemanticsUpdate semanticsUpdate = semanticsUpdateBuilder.build();

    view.updateSemantics(semanticsUpdate);

    _tapCount++;
  }
}
