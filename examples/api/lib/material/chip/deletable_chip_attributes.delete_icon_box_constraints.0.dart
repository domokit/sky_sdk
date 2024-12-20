// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [DeletableChipAttributes.deleteIconBoxConstraints].

void main() => runApp(const DeleteIconBoxConstraintsApp());

class DeleteIconBoxConstraintsApp extends StatelessWidget {
  const DeleteIconBoxConstraintsApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      home: Scaffold(body: Center(child: DeleteIconBoxConstraintsExample())),
    );
  }
}

class DeleteIconBoxConstraintsExample extends StatelessWidget {
  const DeleteIconBoxConstraintsExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        RawChip(
          deleteIconBoxConstraints: const BoxConstraints.tightForFinite(),
          onDeleted: () {},
          label: const SizedBox(
            width: 150,
            child: Text('One line text.', maxLines: 3, overflow: TextOverflow.ellipsis),
          ),
        ),
        const SizedBox(height: 10),
        RawChip(
          deleteIconBoxConstraints: const BoxConstraints.tightForFinite(),
          onDeleted: () {},
          label: const SizedBox(
            width: 150,
            child: Text(
              'This text will wrap into two lines.',
              maxLines: 3,
              overflow: TextOverflow.ellipsis,
            ),
          ),
        ),
        const SizedBox(height: 10),
        RawChip(
          deleteIconBoxConstraints: const BoxConstraints.tightForFinite(),
          onDeleted: () {},
          label: const SizedBox(
            width: 150,
            child: Text(
              'This is a very long text that will wrap into three lines.',
              maxLines: 3,
              overflow: TextOverflow.ellipsis,
            ),
          ),
        ),
      ],
    );
  }
}
