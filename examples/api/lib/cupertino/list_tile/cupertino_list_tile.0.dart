// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart';

/// Flutter code sample for [CupertinoListTile].

void main() => runApp(const CupertinoListTileApp());

class CupertinoListTileApp extends StatelessWidget {
  const CupertinoListTileApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const CupertinoApp(home: CupertinoListTileExample());
  }
}

class CupertinoListTileExample extends StatelessWidget {
  const CupertinoListTileExample({super.key});

  @override
  Widget build(BuildContext context) {
    return CupertinoPageScaffold(
      navigationBar: const CupertinoNavigationBar(middle: Text('CupertinoListTile Sample')),
      child: ListView(
        children: const <Widget>[
          CupertinoListTile(title: Text('One-line CupertinoListTile')),
          CupertinoListTile(leading: FlutterLogo(), title: Text('One-line with leading widget')),
          CupertinoListTile(
            title: Text('One-line with trailing widget'),
            trailing: Icon(Icons.more_vert),
          ),
          CupertinoListTile(
            leading: FlutterLogo(),
            title: Text('One-line with both widgets'),
            trailing: Icon(Icons.more_vert),
          ),
          CupertinoListTile(
            leading: FlutterLogo(size: 56.0),
            title: Text('Two-line CupertinoListTile'),
            subtitle: Text('Here is a subtitle'),
            trailing: Icon(Icons.more_vert),
            additionalInfo: Icon(Icons.info),
          ),
          CupertinoListTile(
            key: Key('CupertinoListTile with background color'),
            leading: FlutterLogo(size: 56.0),
            title: Text('CupertinoListTile with background color'),
            backgroundColor: Colors.lightBlue,
          ),
        ],
      ),
    );
  }
}
