// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';

/// Flutter code sample for [CupertinoAlertDialog].

void main() => runApp(const AlertDialogApp());

class AlertDialogApp extends StatelessWidget {
  const AlertDialogApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(
      theme: CupertinoThemeData(brightness: Brightness.light),
      home: AlertDialogExample(),
    );
  }
}

class AlertDialogExample extends StatelessWidget {
  const AlertDialogExample({super.key});

  void _showAlertDialog(BuildContext context) {
    showCupertinoDialog<void>(
      context: context,
      builder:
          (BuildContext context) => CupertinoAlertDialog(
            title: const Text('Alert'),
            content: const Text('Proceed with destructive action?'),
            actions: <CupertinoDialogAction>[
              CupertinoDialogAction(
                /// This parameter indicates this action is the default,
                /// and turns the action's text to bold text.
                isDefaultAction: true,
                onPressed: () {
                  Navigator.pop(context);
                },
                child: const Text('No'),
              ),
              CupertinoDialogAction(
                /// This parameter indicates the action would perform
                /// a destructive action such as deletion, and turns
                /// the action's text color to red.
                isDestructiveAction: true,
                onPressed: () {
                  Navigator.pop(context);
                },
                child: const Text('Yes'),
              ),
            ],
          ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('CupertinoAlertDialog Sample')),
      child: Center(
        child: CupertinoButton(
          onPressed: () => _showAlertDialog(context),
          child: const Text('CupertinoAlertDialog'),
        ),
      ),
    );
  }
}
