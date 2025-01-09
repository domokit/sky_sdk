// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [ChipAttributes.avatarBoxConstraints].

void main() => runApp(const AvatarBoxConstraintsApp());

class AvatarBoxConstraintsApp extends StatelessWidget {
  const AvatarBoxConstraintsApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: Scaffold(body: Center(child: AvatarBoxConstraintsExample())));
  }
}

class AvatarBoxConstraintsExample extends StatelessWidget {
  const AvatarBoxConstraintsExample({super.key});

  @override
  Widget build(BuildContext context) {
    return const Column(
      mainAxisAlignment: MainAxisAlignment.center,
      children: <Widget>[
        RawChip(
          avatarBoxConstraints: BoxConstraints.tightForFinite(),
          avatar: Icon(Icons.star),
          label: SizedBox(
            width: 150,
            child: Text('One line text.', maxLines: 3, overflow: TextOverflow.ellipsis),
          ),
        ),
        SizedBox(height: 10),
        RawChip(
          avatarBoxConstraints: BoxConstraints.tightForFinite(),
          avatar: Icon(Icons.star),
          label: SizedBox(
            width: 150,
            child: Text(
              'This text will wrap into two lines.',
              maxLines: 3,
              overflow: TextOverflow.ellipsis,
            ),
          ),
        ),
        SizedBox(height: 10),
        RawChip(
          avatarBoxConstraints: BoxConstraints.tightForFinite(),
          avatar: Icon(Icons.star),
          label: SizedBox(
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
