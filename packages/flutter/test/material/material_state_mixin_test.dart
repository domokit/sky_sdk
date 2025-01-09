// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

const Key key = Key('testContainer');
const Color trueColor = Colors.red;
const Color falseColor = Colors.green;

/// Mock widget which plays the role of a button -- it can emit notifications
/// that [MaterialState] values are now in or out of play.
class _InnerWidget extends StatefulWidget {
  const _InnerWidget({required this.onValueChanged, required this.controller});
  final ValueChanged<bool> onValueChanged;
  final StreamController<bool> controller;

  @override
  _InnerWidgetState createState() => _InnerWidgetState();
}

class _InnerWidgetState extends State<_InnerWidget> {
  @override
  void initState() {
    super.initState();
    widget.controller.stream.listen((bool val) => widget.onValueChanged(val));
  }

  @override
  Widget build(BuildContext context) => Container();
}

class _MyWidget extends StatefulWidget {
  const _MyWidget({required this.controller, required this.evaluator, required this.materialState});

  /// Wrapper around `MaterialStateMixin.isPressed/isHovered/isFocused/etc`.
  final bool Function(_MyWidgetState state) evaluator;

  /// Stream passed down to the child [_InnerWidget] to begin the process.
  /// This plays the role of an actual user interaction in the wild, but allows
  /// us to engage the system without mocking pointers/hovers etc.
  final StreamController<bool> controller;

  /// The value we're watching in the given test.
  final MaterialState materialState;

  @override
  State createState() => _MyWidgetState();
}

class _MyWidgetState extends State<_MyWidget> with MaterialStateMixin {
  @override
  Widget build(BuildContext context) {
    return ColoredBox(
      key: key,
      color: widget.evaluator(this) ? trueColor : falseColor,
      child: _InnerWidget(
        onValueChanged: updateMaterialState(widget.materialState),
        controller: widget.controller,
      ),
    );
  }
}

void main() {
  Future<void> verify(WidgetTester tester, Widget widget, StreamController<bool> controller) async {
    await tester.pumpWidget(MaterialApp(home: Scaffold(body: widget)));
    // Set the value to True
    controller.sink.add(true);
    await tester.pumpAndSettle();
    expect(tester.widget<ColoredBox>(find.byKey(key)).color, trueColor);

    // Set the value to False
    controller.sink.add(false);
    await tester.pumpAndSettle();
    expect(tester.widget<ColoredBox>(find.byKey(key)).color, falseColor);
  }

  testWidgets('MaterialState.pressed is tracked', (WidgetTester tester) async {
    final StreamController<bool> controller = StreamController<bool>();
    final _MyWidget widget = _MyWidget(
      controller: controller,
      evaluator: (_MyWidgetState state) => state.isPressed,
      materialState: MaterialState.pressed,
    );
    await verify(tester, widget, controller);
  });

  testWidgets('MaterialState.focused is tracked', (WidgetTester tester) async {
    final StreamController<bool> controller = StreamController<bool>();
    final _MyWidget widget = _MyWidget(
      controller: controller,
      evaluator: (_MyWidgetState state) => state.isFocused,
      materialState: MaterialState.focused,
    );
    await verify(tester, widget, controller);
  });

  testWidgets('MaterialState.hovered is tracked', (WidgetTester tester) async {
    final StreamController<bool> controller = StreamController<bool>();
    final _MyWidget widget = _MyWidget(
      controller: controller,
      evaluator: (_MyWidgetState state) => state.isHovered,
      materialState: MaterialState.hovered,
    );
    await verify(tester, widget, controller);
  });

  testWidgets('MaterialState.disabled is tracked', (WidgetTester tester) async {
    final StreamController<bool> controller = StreamController<bool>();
    final _MyWidget widget = _MyWidget(
      controller: controller,
      evaluator: (_MyWidgetState state) => state.isDisabled,
      materialState: MaterialState.disabled,
    );
    await verify(tester, widget, controller);
  });

  testWidgets('MaterialState.selected is tracked', (WidgetTester tester) async {
    final StreamController<bool> controller = StreamController<bool>();
    final _MyWidget widget = _MyWidget(
      controller: controller,
      evaluator: (_MyWidgetState state) => state.isSelected,
      materialState: MaterialState.selected,
    );
    await verify(tester, widget, controller);
  });

  testWidgets('MaterialState.scrolledUnder is tracked', (WidgetTester tester) async {
    final StreamController<bool> controller = StreamController<bool>();
    final _MyWidget widget = _MyWidget(
      controller: controller,
      evaluator: (_MyWidgetState state) => state.isScrolledUnder,
      materialState: MaterialState.scrolledUnder,
    );
    await verify(tester, widget, controller);
  });

  testWidgets('MaterialState.dragged is tracked', (WidgetTester tester) async {
    final StreamController<bool> controller = StreamController<bool>();
    final _MyWidget widget = _MyWidget(
      controller: controller,
      evaluator: (_MyWidgetState state) => state.isDragged,
      materialState: MaterialState.dragged,
    );
    await verify(tester, widget, controller);
  });

  testWidgets('MaterialState.error is tracked', (WidgetTester tester) async {
    final StreamController<bool> controller = StreamController<bool>();
    final _MyWidget widget = _MyWidget(
      controller: controller,
      evaluator: (_MyWidgetState state) => state.isErrored,
      materialState: MaterialState.error,
    );
    await verify(tester, widget, controller);
  });
}
