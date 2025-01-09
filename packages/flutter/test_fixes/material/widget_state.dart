// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  // Changes made in https://github.com/flutter/flutter/pull/142151
  MaterialState selected = MaterialState.selected;
  MaterialState hovered = MaterialState.hovered;
  MaterialState focused = MaterialState.focused;
  MaterialState pressed = MaterialState.pressed;
  MaterialState dragged = MaterialState.dragged;
  MaterialState scrolledUnder = MaterialState.scrolledUnder;
  MaterialState disabled = MaterialState.disabled;
  MaterialState error = MaterialState.error;

  final MaterialPropertyResolver<MouseCursor?> resolveCallback;

  Color getColor(Set<MaterialState> states) {
    if (states.contains(MaterialState.disabled)) {
      if (states.contains(MaterialState.selected)) {
        return Color(0xFF000002);
      }
      return Color(0xFF000004);
    }
    if (states.contains(MaterialState.selected)) {
      return Color(0xFF000001);
    }
    return Color(0xFF000003);
  }

  final MaterialStateProperty<Color> backgroundColor =
      MaterialStateColor.resolveWith(getColor);

  TextStyle floatingLabelStyle = MaterialStateTextStyle.resolveWith((
    Set<MaterialState> states,
  ) {
    final Color color =
        states.contains(MaterialState.error)
            ? Theme.of(context).colorScheme.error
            : Colors.orange;
    return TextStyle(color: color, letterSpacing: 1.3);
  });

  final MaterialStateProperty<Icon?> thumbIcon =
      MaterialStateProperty.resolveWith<Icon?>((Set<MaterialState> states) {
        if (states.contains(MaterialState.selected)) {
          return const Icon(Icons.check);
        }
        return const Icon(Icons.close);
      });

  final Color backgroundColor = MaterialStatePropertyAll<Color>(
    Colors.blue.withOpacity(0.12),
  );

  final MaterialStatesController statesController = MaterialStatesController(
    <MaterialState>{if (widget.selected) MaterialState.selected},
  );
}

class _MouseCursor extends MaterialStateMouseCursor {
  const _MouseCursor(this.resolveCallback);

  final MaterialPropertyResolver<MouseCursor?> resolveCallback;

  @override
  MouseCursor resolve(Set<MaterialState> states) =>
      resolveCallback(states) ?? MouseCursor.uncontrolled;
}

class SelectedBorder extends RoundedRectangleBorder
    implements MaterialStateOutlinedBorder {
  const SelectedBorder();

  @override
  OutlinedBorder? resolve(Set<MaterialState> states) {
    if (states.contains(MaterialState.selected)) {
      return const RoundedRectangleBorder();
    }
    return null;
  }
}

class _MyWidget extends StatefulWidget {
  const _MyWidget({
    required this.controller,
    required this.evaluator,
    required this.materialState,
  });

  final bool Function(_MyWidgetState state) evaluator;

  /// Stream passed down to the child [_InnerWidget] to begin the process.
  /// This plays the role of an actual user interaction in the wild, but allows
  /// us to engage the system without mocking pointers/hovers etc.
  final StreamController<bool> controller;

  /// The value we're watching in the given test.
  final MaterialState materialState;

  MaterialStateBorderSide? get side {
    return MaterialStateBorderSide.resolveWith((Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        if (states.contains(MaterialState.selected)) {
          return const BorderSide(width: 2.0);
        }
        return BorderSide(width: 1.0);
      }
      if (states.contains(MaterialState.selected)) {
        return const BorderSide(width: 1.5);
      }
      return BorderSide(width: 0.5);
    });
  }

  @override
  State createState() => _MyWidgetState();
}
