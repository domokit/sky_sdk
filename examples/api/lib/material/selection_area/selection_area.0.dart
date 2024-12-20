// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SelectionArea].

void main() => runApp(const SelectionAreaExampleApp());

class SelectionAreaExampleApp extends StatelessWidget {
  const SelectionAreaExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: SelectionArea(
        child: Scaffold(
          appBar: AppBar(title: const Text('SelectionArea Sample')),
          body: const Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: <Widget>[Text('Row 1'), Text('Row 2'), Text('Row 3')],
            ),
          ),
        ),
      ),
    );
  }
}
