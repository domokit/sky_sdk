// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for custom list items.

void main() => runApp(const CustomListItemApp());

class CustomListItemApp extends StatelessWidget {
  const CustomListItemApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: CustomListItemExample());
  }
}

class CustomListItem extends StatelessWidget {
  const CustomListItem({
    super.key,
    required this.thumbnail,
    required this.title,
    required this.user,
    required this.viewCount,
  });

  final Widget thumbnail;
  final String title;
  final String user;
  final int viewCount;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 5.0),
      child: Row(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Expanded(flex: 2, child: thumbnail),
          Expanded(
            flex: 3,
            child: _VideoDescription(title: title, user: user, viewCount: viewCount),
          ),
          const Icon(Icons.more_vert, size: 16.0),
        ],
      ),
    );
  }
}

class _VideoDescription extends StatelessWidget {
  const _VideoDescription({required this.title, required this.user, required this.viewCount});

  final String title;
  final String user;
  final int viewCount;

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.fromLTRB(5.0, 0.0, 0.0, 0.0),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: <Widget>[
          Text(title, style: const TextStyle(fontWeight: FontWeight.w500, fontSize: 14.0)),
          const Padding(padding: EdgeInsets.symmetric(vertical: 2.0)),
          Text(user, style: const TextStyle(fontSize: 10.0)),
          const Padding(padding: EdgeInsets.symmetric(vertical: 1.0)),
          Text('$viewCount views', style: const TextStyle(fontSize: 10.0)),
        ],
      ),
    );
  }
}

class CustomListItemExample extends StatelessWidget {
  const CustomListItemExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('Custom List Item Sample')),
      body: ListView(
        padding: const EdgeInsets.all(8.0),
        itemExtent: 106.0,
        children: <CustomListItem>[
          CustomListItem(
            user: 'Flutter',
            viewCount: 999000,
            thumbnail: Container(decoration: const BoxDecoration(color: Colors.blue)),
            title: 'The Flutter YouTube Channel',
          ),
          CustomListItem(
            user: 'Dash',
            viewCount: 884000,
            thumbnail: Container(decoration: const BoxDecoration(color: Colors.yellow)),
            title: 'Announcing Flutter 1.0',
          ),
        ],
      ),
    );
  }
}
