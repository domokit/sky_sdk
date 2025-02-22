// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';

/// Flutter code sample for [EditableTextTapUpOutsideIntent].

void main() {
  runApp(const SampleApp());
}

class SampleApp extends StatelessWidget {
  const SampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: EditableTextTapUpOutsideIntentExample());
  }
}

class EditableTextTapUpOutsideIntentExample extends StatefulWidget {
  const EditableTextTapUpOutsideIntentExample({super.key});

  @override
  State<EditableTextTapUpOutsideIntentExample> createState() =>
      _EditableTextTapUpOutsideIntentExampleState();
}

class _EditableTextTapUpOutsideIntentExampleState
    extends State<EditableTextTapUpOutsideIntentExample> {
  PointerDownEvent? latestPointerDownEvent;

  void _handlePointerDown(EditableTextTapOutsideIntent intent) {
    // Store the latest pointer down event to calculate the distance between
    // the pointer down and pointer up events later.
    latestPointerDownEvent = intent.pointerDownEvent;

    // Match the default behavior of unfocusing on tap down on desktop platforms
    // and on mobile web. Additionally, save the latest pointer down event to
    // on non-web mobile platforms to calculate the distance between the pointer
    // down and pointer up events later.
    switch (defaultTargetPlatform) {
      case TargetPlatform.android:
      case TargetPlatform.iOS:
      case TargetPlatform.fuchsia:
        // On mobile platforms, we don't unfocus on touch events unless they're
        // in the web browser, but we do unfocus for all other kinds of events.
        switch (intent.pointerDownEvent.kind) {
          case ui.PointerDeviceKind.touch:
            if (kIsWeb) {
              intent.focusNode.unfocus();
            } else {
              // Store the latest pointer down event to calculate the distance
              // between the pointer down and pointer up events later.
              latestPointerDownEvent = intent.pointerDownEvent;
            }
          case ui.PointerDeviceKind.mouse:
          case ui.PointerDeviceKind.stylus:
          case ui.PointerDeviceKind.invertedStylus:
          case ui.PointerDeviceKind.unknown:
            intent.focusNode.unfocus();
          case ui.PointerDeviceKind.trackpad:
            throw UnimplementedError('Unexpected pointer down event for trackpad');
        }
      case TargetPlatform.linux:
      case TargetPlatform.macOS:
      case TargetPlatform.windows:
        intent.focusNode.unfocus();
    }
  }

  void _handlePointerUp(EditableTextTapUpOutsideIntent intent) {
    if (latestPointerDownEvent == null) {
      return;
    }

    final double distance =
        (latestPointerDownEvent!.position - intent.pointerUpEvent.position).distance;

    // Unfocus on taps but not scrolls.
    // kTouchSlop is a framework constant that is used to determine if a
    // pointer event is a tap or a scroll.
    if (distance < kTouchSlop) {
      intent.focusNode.unfocus();
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Padding(
        padding: const EdgeInsets.all(20),
        child: Actions(
          actions: <Type, Action<Intent>>{
            EditableTextTapOutsideIntent: CallbackAction<EditableTextTapOutsideIntent>(
              onInvoke: _handlePointerDown,
            ),
            EditableTextTapUpOutsideIntent: CallbackAction<EditableTextTapUpOutsideIntent>(
              onInvoke: _handlePointerUp,
            ),
          },
          child: ListView(
            children: <Widget>[
              TextField(focusNode: FocusNode()),
              ...List<Widget>.generate(50, (int index) => Text('Item $index')),
            ],
          ),
        ),
      ),
    );
  }
}
