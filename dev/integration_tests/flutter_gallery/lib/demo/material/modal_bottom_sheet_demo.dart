// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

class ModalBottomSheetDemo extends StatelessWidget {
  const ModalBottomSheetDemo({super.key});

  static const String routeName = '/material/modal-bottom-sheet';

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Modal bottom sheet'),
        actions: <Widget>[MaterialDemoDocumentationButton(routeName)],
      ),
      body: Center(
        child: ElevatedButton(
          child: const Text('SHOW BOTTOM SHEET'),
          onPressed: () {
            showModalBottomSheet<void>(
              context: context,
              builder: (BuildContext context) {
                return Padding(
                  padding: const EdgeInsets.all(32.0),
                  child: Text(
                    'This is the modal bottom sheet. Slide down to dismiss.',
                    textAlign: TextAlign.center,
                    style: TextStyle(
                      color: Theme.of(context).colorScheme.secondary,
                      fontSize: 24.0,
                    ),
                  ),
                );
              },
            );
          },
        ),
      ),
    );
  }
}
