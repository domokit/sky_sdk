// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoButton].

void main() => runApp(const CupertinoButtonApp());

class CupertinoButtonApp extends StatelessWidget {
  const CupertinoButtonApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: CupertinoButtonExample(),
    );
  }
}

class CupertinoButtonExample extends StatelessWidget {
  const CupertinoButtonExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('CupertinoButton Sample')),
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: <Widget>[
            const CupertinoButton(onPressed: null, child: Text('Disabled')),
            const SizedBox(height: 30),
            const CupertinoButton.filled(onPressed: null, child: Text('Disabled')),
            const SizedBox(height: 30),
            CupertinoButton(onPressed: () {}, child: const Text('Enabled')),
            const SizedBox(height: 30),
            CupertinoButton.filled(onPressed: () {}, child: const Text('Enabled')),
          ],
        ),
      ),
    );
  }
}
