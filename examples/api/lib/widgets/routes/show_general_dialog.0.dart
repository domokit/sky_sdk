// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [showGeneralDialog].

void main() => runApp(const GeneralDialogApp());

class GeneralDialogApp extends StatelessWidget {
  const GeneralDialogApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(restorationScopeId: 'app', home: GeneralDialogExample());
  }
}

class GeneralDialogExample extends StatelessWidget {
  const GeneralDialogExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: OutlinedButton(
          onPressed: () {
            /// This shows an alert dialog.
            Navigator.of(context).restorablePush(_dialogBuilder);
          },
          child: const Text('Open Dialog'),
        ),
      ),
    );
  }

  @pragma('vm:entry-point')
  static Route<Object?> _dialogBuilder(BuildContext context, Object? arguments) {
    return RawDialogRoute<void>(
      pageBuilder: (
        BuildContext context,
        Animation<double> animation,
        Animation<double> secondaryAnimation,
      ) {
        return const AlertDialog(title: Text('Alert!'));
      },
    );
  }
}
