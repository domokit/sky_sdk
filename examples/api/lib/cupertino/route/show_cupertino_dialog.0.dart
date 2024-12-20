// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [showCupertinoDialog].

void main() => runApp(const CupertinoDialogApp());

class CupertinoDialogApp extends StatelessWidget {
  const CupertinoDialogApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      restorationScopeId: 'app',
      home: CupertinoDialogExample(),
    );
  }
}

class CupertinoDialogExample extends StatelessWidget {
  const CupertinoDialogExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('Home')),
      child: Center(
        child: CupertinoButton(
          onPressed: () {
            Navigator.of(context).restorablePush(_dialogBuilder);
          },
          child: const Text('Open Dialog'),
        ),
      ),
    );
  }

  @pragma('vm:entry-point')
  static Route<Object?> _dialogBuilder(BuildContext context, Object? arguments) {
    return CupertinoDialogRoute<void>(
      context: context,
      builder: (BuildContext context) {
        return CupertinoAlertDialog(
          title: const Text('Title'),
          content: const Text('Content'),
          actions: <Widget>[
            CupertinoDialogAction(
              onPressed: () {
                Navigator.pop(context);
              },
              child: const Text('Yes'),
            ),
            CupertinoDialogAction(
              onPressed: () {
                Navigator.pop(context);
              },
              child: const Text('No'),
            ),
          ],
        );
      },
    );
  }
}
