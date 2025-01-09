// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';

import 'package:flutter/material.dart';

/// Flutter code sample for [PointerSignalResolver].

void main() => runApp(const PointerSignalResolverExampleApp());

class PointerSignalResolverExampleApp extends StatelessWidget {
  const PointerSignalResolverExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: PointerSignalResolverExample());
  }
}

class ColorChanger extends StatefulWidget {
  const ColorChanger({
    super.key,
    required this.initialColor,
    required this.useResolver,
    this.child,
  });

  final HSVColor initialColor;
  final bool useResolver;
  final Widget? child;

  @override
  State<ColorChanger> createState() => _ColorChangerState();
}

class _ColorChangerState extends State<ColorChanger> {
  late HSVColor color;

  void rotateColor() {
    setState(() {
      color = color.withHue((color.hue + 3) % 360.0);
    });
  }

  @override
  void initState() {
    super.initState();
    color = widget.initialColor;
  }

  @override
  Widget build(BuildContext context) {
    return DecoratedBox(
      decoration: BoxDecoration(
        border: const Border.fromBorderSide(BorderSide()),
        color: color.toColor(),
      ),
      child: Listener(
        onPointerSignal: (PointerSignalEvent event) {
          if (widget.useResolver) {
            GestureBinding.instance.pointerSignalResolver.register(event, (
              PointerSignalEvent event,
            ) {
              rotateColor();
            });
          } else {
            rotateColor();
          }
        },
        child: Stack(
          fit: StackFit.expand,
          children: <Widget>[const AbsorbPointer(), if (widget.child != null) widget.child!],
        ),
      ),
    );
  }
}

class PointerSignalResolverExample extends StatefulWidget {
  const PointerSignalResolverExample({super.key});

  @override
  State<PointerSignalResolverExample> createState() => _PointerSignalResolverExampleState();
}

class _PointerSignalResolverExampleState extends State<PointerSignalResolverExample> {
  bool useResolver = false;

  @override
  Widget build(BuildContext context) {
    return Material(
      child: Stack(
        fit: StackFit.expand,
        children: <Widget>[
          ColorChanger(
            initialColor: const HSVColor.fromAHSV(0.2, 120.0, 1, 1),
            useResolver: useResolver,
            child: FractionallySizedBox(
              widthFactor: 0.5,
              heightFactor: 0.5,
              child: ColorChanger(
                initialColor: const HSVColor.fromAHSV(1, 60.0, 1, 1),
                useResolver: useResolver,
              ),
            ),
          ),
          Align(
            alignment: Alignment.topLeft,
            child: Row(
              children: <Widget>[
                Switch(
                  value: useResolver,
                  onChanged: (bool value) {
                    setState(() {
                      useResolver = value;
                    });
                  },
                ),
                const Text(
                  'Use the PointerSignalResolver?',
                  style: TextStyle(fontWeight: FontWeight.bold),
                ),
              ],
            ),
          ),
        ],
      ),
    );
  }
}
