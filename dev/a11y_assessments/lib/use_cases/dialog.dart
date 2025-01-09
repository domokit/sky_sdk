// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../utils.dart';
import 'use_cases.dart';

class DialogUseCase extends UseCase {
  @override
  String get name => 'Dialog';

  @override
  String get route => '/dialog';

  @override
  Widget build(BuildContext context) => _MainWidget();
}

class _MainWidget extends StatelessWidget {
  _MainWidget();

  final String pageTitle = getUseCaseName(DialogUseCase());

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Semantics(headingLevel: 1, child: Text('$pageTitle Demo')),
      ),
      body: Center(
        child: TextButton(
          onPressed:
              () => showDialog<String>(
                context: context,
                builder:
                    (BuildContext context) => Dialog(
                      child: Padding(
                        padding: const EdgeInsets.all(8.0),
                        child: Column(
                          mainAxisSize: MainAxisSize.min,
                          mainAxisAlignment: MainAxisAlignment.center,
                          children: <Widget>[
                            const Text('This is a typical dialog.'),
                            const SizedBox(height: 15),
                            Row(
                              children: <Widget>[
                                TextButton(
                                  key: const Key('OK Button'),
                                  autofocus: true,
                                  onPressed: () {
                                    Navigator.pop(context);
                                  },
                                  child: const Text('OK'),
                                ),
                                TextButton(
                                  onPressed: () {
                                    Navigator.pop(context);
                                  },
                                  child: const Text('Cancel'),
                                ),
                              ],
                            ),
                          ],
                        ),
                      ),
                    ),
              ),
          child: const Text('Show Dialog'),
        ),
      ),
    );
  }
}
