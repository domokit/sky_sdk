// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [DeletableChipAttributes.onDeleted].

void main() => runApp(const OnDeletedExampleApp());

class OnDeletedExampleApp extends StatelessWidget {
  const OnDeletedExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      home: Scaffold(
        appBar: AppBar(title: const Text('DeletableChipAttributes.onDeleted Sample')),
        body: const Center(child: OnDeletedExample()),
      ),
    );
  }
}

class Actor {
  const Actor(this.name, this.initials);
  final String name;
  final String initials;
}

class CastList extends StatefulWidget {
  const CastList({super.key});

  @override
  State createState() => CastListState();
}

class CastListState extends State<CastList> {
  final List<Actor> _cast = <Actor>[
    const Actor('Aaron Burr', 'AB'),
    const Actor('Alexander Hamilton', 'AH'),
    const Actor('Eliza Hamilton', 'EH'),
    const Actor('James Madison', 'JM'),
  ];

  Iterable<Widget> get actorWidgets {
    return _cast.map((Actor actor) {
      return Padding(
        padding: const EdgeInsets.all(4.0),
        child: Chip(
          avatar: CircleAvatar(child: Text(actor.initials)),
          label: Text(actor.name),
          onDeleted: () {
            setState(() {
              _cast.removeWhere((Actor entry) {
                return entry.name == actor.name;
              });
            });
          },
        ),
      );
    });
  }

  @override
  Widget build(BuildContext context) {
    return Wrap(children: actorWidgets.toList());
  }
}

class OnDeletedExample extends StatefulWidget {
  const OnDeletedExample({super.key});

  @override
  State<OnDeletedExample> createState() => _OnDeletedExampleState();
}

class _OnDeletedExampleState extends State<OnDeletedExample> {
  @override
  Widget build(BuildContext context) {
    return const CastList();
  }
}
