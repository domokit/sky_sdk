// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Focus].

void main() => runApp(const FocusExampleApp());

class FocusExampleApp extends StatelessWidget {
  const FocusExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: FocusExample());
  }
}

class FocusableText extends StatelessWidget {
  const FocusableText(this.data, {super.key, required this.autofocus});

  /// The string to display as the text for this widget.
  final String data;

  /// Whether or not to focus this widget initially if nothing else is focused.
  final bool autofocus;

  @override
  Widget build(BuildContext context) {
    return Focus(
      autofocus: autofocus,
      child: Builder(
        builder: (BuildContext context) {
          // The contents of this Builder are being made focusable. It is inside
          // of a Builder because the builder provides the correct context
          // variable for Focus.of() to be able to find the Focus widget that is
          // the Builder's parent. Without the builder, the context variable used
          // would be the one given the FocusableText build function, and that
          // would start looking for a Focus widget ancestor of the FocusableText
          // instead of finding the one inside of its build function.
          return Container(
            padding: const EdgeInsets.all(8.0),
            // Change the color based on whether or not this Container has focus.
            color: Focus.of(context).hasPrimaryFocus ? Colors.red : Colors.white,
            child: Text(data),
          );
        },
      ),
    );
  }
}

class FocusExample extends StatelessWidget {
  const FocusExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: ListView.builder(
        itemBuilder:
            (BuildContext context, int index) =>
                FocusableText('Item $index', autofocus: index == 0),
        itemCount: 50,
      ),
    );
  }
}
