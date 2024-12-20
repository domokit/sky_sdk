// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [MaterialStateProperty].

void main() => runApp(const MaterialStatePropertyExampleApp());

class MaterialStatePropertyExampleApp extends StatelessWidget {
  const MaterialStatePropertyExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('MaterialStateProperty Sample')),
        body: const Center(child: MaterialStatePropertyExample()),
      ),
    );
  }
}

class MaterialStatePropertyExample extends StatelessWidget {
  const MaterialStatePropertyExample({super.key});

  @override
  Widget build(BuildContext context) {
    Color getColor(Set<MaterialState> states) {
      const Set<MaterialState> interactiveStates = <MaterialState>{
        MaterialState.pressed,
        MaterialState.hovered,
        MaterialState.focused,
      };
      if (states.any(interactiveStates.contains)) {
        return Colors.blue;
      }
      return Colors.red;
    }

    return TextButton(
      style: ButtonStyle(foregroundColor: MaterialStateProperty.resolveWith(getColor)),
      onPressed: () {},
      child: const Text('TextButton'),
    );
  }
}
