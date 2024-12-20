// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'use_cases.dart';

class TabBarViewUseCase extends UseCase {
  @override
  String get name => 'TabBarView';

  @override
  String get route => '/tab-bar-view';

  @override
  Widget build(BuildContext context) => const TabBarViewExample();
}

class TabBarViewExample extends StatelessWidget {
  const TabBarViewExample({super.key});

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      initialIndex: 1,
      length: 3,
      child: Scaffold(
        appBar: AppBar(
          title: Semantics(headingLevel: 1, child: const Text('TabBarView Sample')),
          bottom: const TabBar(
            tabs: <Widget>[
              Tab(icon: Icon(Icons.cloud_outlined), text: 'Cloudy'),
              Tab(icon: Icon(Icons.beach_access_sharp), text: 'Rainy'),
              Tab(icon: Icon(Icons.brightness_5_sharp), text: 'Sunny'),
            ],
          ),
        ),
        body: const TabBarView(
          children: <Widget>[
            Center(child: Text("It's cloudy here")),
            Center(child: Text("It's rainy here")),
            Center(child: Text("It's sunny here")),
          ],
        ),
      ),
    );
  }
}
