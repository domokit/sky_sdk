// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';

/// Flutter code sample for [RefreshIndicator].

void main() => runApp(const RefreshIndicatorExampleApp());

class RefreshIndicatorExampleApp extends StatelessWidget {
  const RefreshIndicatorExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      scrollBehavior: const MaterialScrollBehavior().copyWith(
        dragDevices: PointerDeviceKind.values.toSet(),
      ),
      home: const RefreshIndicatorExample(),
    );
  }
}

class RefreshIndicatorExample extends StatelessWidget {
  const RefreshIndicatorExample({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('RefreshIndicator Sample')),
      body: RefreshIndicator(
        color: Colors.white,
        backgroundColor: Colors.blue,
        onRefresh: () async {
          // Replace this delay with the code to be executed during refresh
          // and return asynchronous code
          return Future<void>.delayed(const Duration(seconds: 3));
        },
        // This check is used to customize listening to scroll notifications
        // from the widget's children.
        //
        // By default this is set to `notification.depth == 0`, which ensures
        // the only the scroll notifications from the first scroll view are listened to.
        //
        // Here setting `notification.depth == 1` triggers the refresh indicator
        // when overscrolling the nested scroll view.
        notificationPredicate: (ScrollNotification notification) {
          return notification.depth == 1;
        },
        child: CustomScrollView(
          slivers: <Widget>[
            SliverToBoxAdapter(
              child: Container(
                height: 100,
                alignment: Alignment.center,
                color: Colors.pink[100],
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: <Widget>[
                    Text('Pull down here', style: Theme.of(context).textTheme.headlineMedium),
                    const Text("RefreshIndicator won't trigger"),
                  ],
                ),
              ),
            ),
            SliverToBoxAdapter(
              child: Container(
                color: Colors.green[100],
                height: 300,
                child: ListView.builder(
                  itemCount: 25,
                  itemBuilder: (BuildContext context, int index) {
                    return const ListTile(
                      title: Text('Pull down here'),
                      subtitle: Text('RefreshIndicator will trigger'),
                    );
                  },
                ),
              ),
            ),
            SliverList.builder(
              itemCount: 20,
              itemBuilder: (BuildContext context, int index) {
                return const ListTile(
                  title: Text('Pull down here'),
                  subtitle: Text("Refresh indicator won't trigger"),
                );
              },
            ),
          ],
        ),
      ),
    );
  }
}
