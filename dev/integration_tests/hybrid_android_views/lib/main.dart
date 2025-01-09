// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_driver/driver_extension.dart';

import 'future_data_handler.dart';
import 'motion_events_page.dart';
import 'nested_view_event_page.dart';
import 'page.dart';

final List<PageWidget> _allPages = <PageWidget>[
  const MotionEventsPage(),
  const NestedViewEventPage(),
];

void main() {
  enableFlutterDriverExtension(handler: driverDataHandler.handleMessage);
  runApp(const MaterialApp(home: Home()));
}

class Home extends StatelessWidget {
  const Home({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: ListView(
        children: _allPages.map((PageWidget p) => _buildPageListTile(context, p)).toList(),
      ),
    );
  }

  Widget _buildPageListTile(BuildContext context, PageWidget page) {
    return ListTile(
      title: Text(page.title),
      key: page.tileKey,
      onTap: () {
        _pushPage(context, page);
      },
    );
  }

  void _pushPage(BuildContext context, PageWidget page) {
    Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => Scaffold(body: page)));
  }
}
