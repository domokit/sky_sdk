// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [IconButton].

void main() {
  runApp(const IconButtonApp());
}

class IconButtonApp extends StatelessWidget {
  const IconButtonApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(colorSchemeSeed: const Color(0xff6750a4), useMaterial3: true),
      title: 'Icon Button Types',
      home: const Scaffold(body: ButtonTypesExample()),
    );
  }
}

class ButtonTypesExample extends StatelessWidget {
  const ButtonTypesExample({super.key});

  @override
  Widget build(BuildContext context) {
    return const Padding(
      padding: EdgeInsets.all(4.0),
      child: Row(
        children: <Widget>[
          Spacer(),
          ButtonTypesGroup(enabled: true),
          ButtonTypesGroup(enabled: false),
          Spacer(),
        ],
      ),
    );
  }
}

class ButtonTypesGroup extends StatelessWidget {
  const ButtonTypesGroup({super.key, required this.enabled});

  final bool enabled;

  @override
  Widget build(BuildContext context) {
    final VoidCallback? onPressed = enabled ? () {} : null;

    return Padding(
      padding: const EdgeInsets.all(4.0),
      child: Column(
        mainAxisAlignment: MainAxisAlignment.spaceEvenly,
        children: <Widget>[
          IconButton(icon: const Icon(Icons.filter_drama), onPressed: onPressed),

          // Filled icon button
          IconButton.filled(onPressed: onPressed, icon: const Icon(Icons.filter_drama)),

          // Filled tonal icon button
          IconButton.filledTonal(onPressed: onPressed, icon: const Icon(Icons.filter_drama)),

          // Outlined icon button
          IconButton.outlined(onPressed: onPressed, icon: const Icon(Icons.filter_drama)),
        ],
      ),
    );
  }
}
