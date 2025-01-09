// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [TextButton].

void main() {
  runApp(const MaterialApp(home: Home()));
}

class SelectableButton extends StatefulWidget {
  const SelectableButton({
    super.key,
    required this.selected,
    this.style,
    required this.onPressed,
    required this.child,
  });

  final bool selected;
  final ButtonStyle? style;
  final VoidCallback? onPressed;
  final Widget child;

  @override
  State<SelectableButton> createState() => _SelectableButtonState();
}

class _SelectableButtonState extends State<SelectableButton> {
  late final WidgetStatesController statesController;

  @override
  void initState() {
    super.initState();
    statesController = WidgetStatesController(<WidgetState>{
      if (widget.selected) WidgetState.selected,
    });
  }

  @override
  void didUpdateWidget(SelectableButton oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (widget.selected != oldWidget.selected) {
      statesController.update(WidgetState.selected, widget.selected);
    }
  }

  @override
  Widget build(BuildContext context) {
    return TextButton(
      statesController: statesController,
      style: widget.style,
      onPressed: widget.onPressed,
      child: widget.child,
    );
  }
}

class Home extends StatefulWidget {
  const Home({super.key});

  @override
  State<Home> createState() => _HomeState();
}

class _HomeState extends State<Home> {
  bool selected = false;

  /// Sets the button's foreground and background colors.
  /// If not selected, resolves to null and defers to default values.
  static const ButtonStyle style = ButtonStyle(
    foregroundColor: WidgetStateProperty<Color?>.fromMap(<WidgetState, Color>{
      WidgetState.selected: Colors.white,
    }),
    backgroundColor: WidgetStateProperty<Color?>.fromMap(<WidgetState, Color>{
      WidgetState.selected: Colors.indigo,
    }),
  );

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: SelectableButton(
          selected: selected,
          style: style,
          onPressed: () {
            setState(() {
              selected = !selected;
            });
          },
          child: const Text('toggle selected'),
        ),
      ),
    );
  }
}
