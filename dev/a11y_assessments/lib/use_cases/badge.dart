// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../utils.dart';
import 'use_cases.dart';

class BadgeUseCase extends UseCase {
  @override
  String get name => 'Badge';

  @override
  String get route => '/badge';

  @override
  Widget build(BuildContext context) => const MainWidget();
}

class MainWidget extends StatefulWidget {
  const MainWidget({super.key});

  @override
  State<MainWidget> createState() => MainWidgetState();
}

class MainWidgetState extends State<MainWidget> {
  String pageTitle = getUseCaseName(BadgeUseCase());

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Semantics(headingLevel: 1, child: Text('$pageTitle Demo')),
      ),
      body: const Center(
        child: Badge(
          label: Text('5', semanticsLabel: '5 new messages', style: TextStyle(color: Colors.white)),
          backgroundColor: Colors.green,
          child: Icon(Icons.mail, semanticLabel: 'Messages'),
        ),
      ),
    );
  }
}
