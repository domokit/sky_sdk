// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Switch.adaptive].

void main() => runApp(const SwitchApp());

class SwitchApp extends StatefulWidget {
  const SwitchApp({super.key});

  @override
  State<SwitchApp> createState() => _SwitchAppState();
}

class _SwitchAppState extends State<SwitchApp> {
  bool isMaterial = true;
  bool isCustomized = false;

  @override
  Widget build(BuildContext context) {
    final ThemeData theme = ThemeData(
      platform: isMaterial ? TargetPlatform.android : TargetPlatform.iOS,
      adaptations: <Adaptation<Object>>[if (isCustomized) const _SwitchThemeAdaptation()],
    );
    final ButtonStyle style = OutlinedButton.styleFrom(fixedSize: const Size(220, 40));

    return MaterialApp(
      theme: theme,
      home: Scaffold(
        appBar: AppBar(title: const Text('Adaptive Switches')),
        body: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            OutlinedButton(
              style: style,
              onPressed: () {
                setState(() {
                  isMaterial = !isMaterial;
                });
              },
              child:
                  isMaterial
                      ? const Text('Show cupertino style')
                      : const Text('Show material style'),
            ),
            OutlinedButton(
              style: style,
              onPressed: () {
                setState(() {
                  isCustomized = !isCustomized;
                });
              },
              child:
                  isCustomized
                      ? const Text('Remove customization')
                      : const Text('Add customization'),
            ),
            const SizedBox(height: 20),
            const SwitchWithLabel(label: 'enabled', enabled: true),
            const SwitchWithLabel(label: 'disabled', enabled: false),
          ],
        ),
      ),
    );
  }
}

class SwitchWithLabel extends StatefulWidget {
  const SwitchWithLabel({super.key, required this.enabled, required this.label});

  final bool enabled;
  final String label;

  @override
  State<SwitchWithLabel> createState() => _SwitchWithLabelState();
}

class _SwitchWithLabelState extends State<SwitchWithLabel> {
  bool active = true;

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        Container(width: 150, padding: const EdgeInsets.only(right: 20), child: Text(widget.label)),
        Switch.adaptive(
          value: active,
          onChanged:
              !widget.enabled
                  ? null
                  : (bool value) {
                    setState(() {
                      active = value;
                    });
                  },
        ),
      ],
    );
  }
}

class _SwitchThemeAdaptation extends Adaptation<SwitchThemeData> {
  const _SwitchThemeAdaptation();

  @override
  SwitchThemeData adapt(ThemeData theme, SwitchThemeData defaultValue) {
    switch (theme.platform) {
      case TargetPlatform.android:
      case TargetPlatform.fuchsia:
      case TargetPlatform.linux:
      case TargetPlatform.windows:
        return defaultValue;
      case TargetPlatform.iOS:
      case TargetPlatform.macOS:
        return const SwitchThemeData(
          thumbColor: WidgetStateProperty<Color?>.fromMap(<WidgetState, Color>{
            WidgetState.selected: Colors.yellow,
            // Resolves to null if not selected, deferring to default values.
          }),
          trackColor: WidgetStatePropertyAll<Color>(Colors.brown),
        );
    }
  }
}
