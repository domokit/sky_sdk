// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../utils.dart';
import 'use_cases.dart';

class SnackBarUseCase extends UseCase {
  @override
  String get name => 'SnackBar';

  @override
  String get route => '/snack-bar';

  @override
  Widget build(BuildContext context) => const MainWidget();
}

class MainWidget extends StatefulWidget {
  const MainWidget({super.key});

  @override
  State<MainWidget> createState() => MainWidgetState();
}

class MainWidgetState extends State<MainWidget> {
  String pageTitle = getUseCaseName(SnackBarUseCase());

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Semantics(headingLevel: 1, child: Text('$pageTitle Demo')),
      ),
      body: Center(
        child: Column(
          children: <Widget>[
            ElevatedButton(
              child: const Text('Show Snackbar'),
              onPressed: () {
                ScaffoldMessenger.of(
                  context,
                ).showSnackBar(const SnackBar(content: Text('Awesome Snackbar!')));
              },
            ),
            ElevatedButton(
              child: const Text('Show Snackbar with action '),
              onPressed: () {
                ScaffoldMessenger.of(context).showSnackBar(
                  SnackBar(
                    content: const Text('Awesome Snackbar!'),
                    action: SnackBarAction(label: 'Action', onPressed: () {}),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }
}
