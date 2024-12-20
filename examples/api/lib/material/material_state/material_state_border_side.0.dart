// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [MaterialStateBorderSide].

void main() => runApp(const MaterialStateBorderSideExampleApp());

class MaterialStateBorderSideExampleApp extends StatelessWidget {
  const MaterialStateBorderSideExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('MaterialStateBorderSide Sample')),
        body: const Center(child: MaterialStateBorderSideExample()),
      ),
    );
  }
}

class MaterialStateBorderSideExample extends StatefulWidget {
  const MaterialStateBorderSideExample({super.key});

  @override
  State<MaterialStateBorderSideExample> createState() => _MaterialStateBorderSideExampleState();
}

class _MaterialStateBorderSideExampleState extends State<MaterialStateBorderSideExample> {
  bool isSelected = true;

  @override
  Widget build(BuildContext context) {
    return FilterChip(
      label: const Text('Select chip'),
      selected: isSelected,
      onSelected: (bool value) {
        setState(() {
          isSelected = value;
        });
      },
      side: MaterialStateBorderSide.resolveWith((Set<MaterialState> states) {
        if (states.contains(MaterialState.selected)) {
          return const BorderSide(color: Colors.red);
        }
        return null; // Defer to default value on the theme or widget.
      }),
    );
  }
}
