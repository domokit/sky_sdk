// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../utils.dart';
import 'use_cases.dart';

class ActionChipUseCase extends UseCase {
  @override
  String get name => 'ActionChip';

  @override
  String get route => '/action-chip';

  @override
  Widget build(BuildContext context) => const MainWidget();
}

class MainWidget extends StatefulWidget {
  const MainWidget({super.key});

  @override
  State<MainWidget> createState() => MainWidgetState();
}

class MainWidgetState extends State<MainWidget> {
  bool favorite = false;

  String pageTitle = getUseCaseName(ActionChipUseCase());

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Semantics(headingLevel: 1, child: Text('$pageTitle Demo')),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            ActionChip(
              avatar: const Icon(Icons.favorite),
              label: const Text('Action'),
              onPressed: () {},
            ),
            const ActionChip(avatar: Icon(Icons.favorite), label: Text('Action')),
          ],
        ),
      ),
    );
  }
}
