// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import '../utils.dart';
import 'use_cases.dart';

class ExpansionTileUseCase extends UseCase {
  @override
  String get name => 'ExpansionTile';

  @override
  String get route => '/expansion-tile';

  @override
  Widget build(BuildContext context) => const ExpansionTileExample();
}

class ExpansionTileExample extends StatefulWidget {
  const ExpansionTileExample({super.key});

  @override
  State<ExpansionTileExample> createState() => _ExpansionTileExampleState();
}

class _ExpansionTileExampleState extends State<ExpansionTileExample> {
  bool _customTileExpanded = false;

  String pageTitle = getUseCaseName(ExpansionTileUseCase());

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        backgroundColor: Theme.of(context).colorScheme.inversePrimary,
        title: Semantics(headingLevel: 1, child: Text('$pageTitle Demo')),
      ),
      body: Column(
        children: <Widget>[
          const ExpansionTile(
            title: Text('ExpansionTile 1'),
            subtitle: Text('Trailing expansion arrow icon'),
            children: <Widget>[ListTile(title: Text('This is tile number 1'))],
          ),
          ExpansionTile(
            title: const Text('ExpansionTile 2'),
            subtitle: const Text('Custom expansion arrow icon'),
            trailing: Icon(
              _customTileExpanded ? Icons.arrow_drop_down_circle : Icons.arrow_drop_down,
            ),
            children: const <Widget>[ListTile(title: Text('This is tile number 2'))],
            onExpansionChanged: (bool expanded) {
              setState(() {
                _customTileExpanded = expanded;
              });
            },
          ),
          const ExpansionTile(
            title: Text('ExpansionTile 3'),
            subtitle: Text('Leading expansion arrow icon'),
            controlAffinity: ListTileControlAffinity.leading,
            children: <Widget>[ListTile(title: Text('This is tile number 3'))],
          ),
        ],
      ),
    );
  }
}
