// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [OverflowBar].

void main() => runApp(const OverflowBarExampleApp());

class OverflowBarExampleApp extends StatelessWidget {
  const OverflowBarExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('OverflowBar Sample')),
        body: const Center(child: OverflowBarExample()),
      ),
    );
  }
}

class OverflowBarExample extends StatelessWidget {
  const OverflowBarExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Container(
      alignment: Alignment.center,
      padding: const EdgeInsets.all(16),
      color: Colors.black.withOpacity(0.15),
      child: Material(
        color: Colors.white,
        elevation: 24,
        shape: const RoundedRectangleBorder(borderRadius: BorderRadius.all(Radius.circular(4))),
        child: Padding(
          padding: const EdgeInsets.all(8),
          child: SingleChildScrollView(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: <Widget>[
                const SizedBox(height: 128, child: Placeholder()),
                Align(
                  alignment: AlignmentDirectional.centerEnd,
                  child: OverflowBar(
                    spacing: 8,
                    overflowAlignment: OverflowBarAlignment.end,
                    children: <Widget>[
                      TextButton(child: const Text('Cancel'), onPressed: () {}),
                      TextButton(child: const Text('Really Really Cancel'), onPressed: () {}),
                      OutlinedButton(child: const Text('OK'), onPressed: () {}),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
