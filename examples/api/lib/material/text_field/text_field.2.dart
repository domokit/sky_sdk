// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for Material Design 3 [TextField]s.

void main() {
  runApp(const TextFieldExamplesApp());
}

class TextFieldExamplesApp extends StatelessWidget {
  const TextFieldExamplesApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(colorSchemeSeed: const Color(0xff6750a4)),
      home: Scaffold(
        appBar: AppBar(title: const Text('TextField Examples')),
        body: const Column(
          children: <Widget>[
            Spacer(),
            FilledTextFieldExample(),
            OutlinedTextFieldExample(),
            Spacer(),
          ],
        ),
      ),
    );
  }
}

/// An example of the filled text field type.
///
/// A filled [TextField] with default settings matching the spec:
/// https://m3.material.io/components/text-fields/specs#6d654d1d-262e-4697-858c-9a75e8e7c81d
class FilledTextFieldExample extends StatelessWidget {
  const FilledTextFieldExample({super.key});

  @override
  Widget build(BuildContext context) {
    return const TextField(
      decoration: InputDecoration(
        prefixIcon: Icon(Icons.search),
        suffixIcon: Icon(Icons.clear),
        labelText: 'Filled',
        hintText: 'hint text',
        helperText: 'supporting text',
        filled: true,
      ),
    );
  }
}

/// An example of the outlined text field type.
///
/// A Outlined [TextField] with default settings matching the spec:
/// https://m3.material.io/components/text-fields/specs#68b00bd6-ab40-4b4f-93d9-ed1fbbc5d06e
class OutlinedTextFieldExample extends StatelessWidget {
  const OutlinedTextFieldExample({super.key});

  @override
  Widget build(BuildContext context) {
    return const TextField(
      decoration: InputDecoration(
        prefixIcon: Icon(Icons.search),
        suffixIcon: Icon(Icons.clear),
        labelText: 'Outlined',
        hintText: 'hint text',
        helperText: 'supporting text',
        border: OutlineInputBorder(),
      ),
    );
  }
}
