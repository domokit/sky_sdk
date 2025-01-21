// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoTextMagnifier].

void main() => runApp(const CupertinoTextMagnifierApp());

class CupertinoTextMagnifierApp extends StatelessWidget {
  const CupertinoTextMagnifierApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: CupertinoTextMagnifierExampleApp(),
    );
  }
}

class CupertinoTextMagnifierExampleApp extends StatefulWidget {
  const CupertinoTextMagnifierExampleApp({super.key});

  @override
  State<CupertinoTextMagnifierExampleApp> createState() => _CupertinoTextMagnifierExampleAppState();
}

class _CupertinoTextMagnifierExampleAppState extends State<CupertinoTextMagnifierExampleApp> {
  final MagnifierController _controller = MagnifierController();

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('CupertinoTextMagnifier Sample')),
      child: Padding(
        padding: const EdgeInsets.symmetric(horizontal: 48.0),
        child: Center(
          child: CupertinoTextField(
            magnifierConfiguration: TextMagnifierConfiguration(
              magnifierBuilder: (_, _, ValueNotifier<MagnifierInfo> magnifierInfo) {
                return CupertinoTextMagnifier(
                  controller: _controller,
                  magnifierInfo: magnifierInfo,
                );
              },
            ),
            controller: TextEditingController(text: 'Hello world!'),
          ),
        ),
      ),
    );
  }
}
