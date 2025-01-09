// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [WidgetStateInputBorder].

void main() => runApp(const WidgetStateInputBorderExampleApp());

/// This extension isn't necessary when WidgetState properties are
/// configured using [WidgetStateMapper] objects.
///
/// But sometimes it makes sense to use a resolveWith() callback,
/// and these getters make those callbacks a bit more readable!
extension WidgetStateHelpers on Set<WidgetState> {
  bool get focused => contains(WidgetState.focused);
  bool get hovered => contains(WidgetState.hovered);
  bool get disabled => contains(WidgetState.disabled);
}

class WidgetStateInputBorderExampleApp extends StatelessWidget {
  const WidgetStateInputBorderExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('WidgetStateInputBorder Example')),
        body: const Center(child: PageContent()),
      ),
    );
  }
}

class PageContent extends StatefulWidget {
  const PageContent({super.key});

  @override
  State<PageContent> createState() => _PageContentState();
}

class _PageContentState extends State<PageContent> {
  bool enabled = false;

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: <Widget>[
        const Spacer(flex: 8),
        Focus(child: WidgetStateInputBorderExample(enabled: enabled)),
        const Spacer(),
        FilterChip(
          label: const Text('enable text field'),
          selected: enabled,
          onSelected: (bool selected) {
            setState(() {
              enabled = selected;
            });
          },
        ),
        const Spacer(flex: 8),
      ],
    );
  }
}

class WidgetStateInputBorderExample extends StatelessWidget {
  const WidgetStateInputBorderExample({super.key, required this.enabled});

  final bool enabled;

  /// A global or static function can be referenced in a `const` constructor,
  /// such as [WidgetStateInputBorder.resolveWith].
  ///
  /// Constant values can be useful for promoting accurate equality checks,
  /// such as when rebuilding a [Theme] widget.
  static UnderlineInputBorder veryCoolBorder(Set<WidgetState> states) {
    if (states.disabled) {
      return const UnderlineInputBorder(borderSide: BorderSide(color: Colors.grey));
    }

    const Color dullViolet = Color(0xFF502080);

    return UnderlineInputBorder(
      borderSide: BorderSide(
        width: states.hovered ? 6 : (states.focused ? 3 : 1.5),
        color: states.focused ? Colors.deepPurpleAccent : dullViolet,
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    final InputDecoration decoration = InputDecoration(
      border: const WidgetStateInputBorder.resolveWith(veryCoolBorder),
      labelText: enabled ? 'Type something awesome…' : '(click below to enable)',
    );

    return AnimatedFractionallySizedBox(
      duration: Durations.medium1,
      curve: Curves.ease,
      widthFactor: Focus.of(context).hasFocus ? 0.9 : 0.6,
      child: TextField(decoration: decoration, enabled: enabled),
    );
  }
}
