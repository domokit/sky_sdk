// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Notification].

void main() => runApp(const NotificationExampleApp());

class NotificationExampleApp extends StatelessWidget {
  const NotificationExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: NotificationExample());
  }
}

class NotificationExample extends StatelessWidget {
  const NotificationExample({super.key});

  static const List<String> _tabs = <String>['Months', 'Days'];
  static const List<String> _months = <String>['January', 'February', 'March'];
  static const List<String> _days = <String>['Sunday', 'Monday', 'Tuesday'];

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: _tabs.length,
      child: Scaffold(
        // Listens to the scroll events and returns the current position.
        body: NotificationListener<ScrollNotification>(
          onNotification: (ScrollNotification scrollNotification) {
            if (scrollNotification is ScrollStartNotification) {
              debugPrint('Scrolling has started');
            } else if (scrollNotification is ScrollEndNotification) {
              debugPrint('Scrolling has ended');
            }
            // Return true to cancel the notification bubbling.
            return true;
          },
          child: NestedScrollView(
            headerSliverBuilder: (BuildContext context, bool innerBoxIsScrolled) {
              return <Widget>[
                SliverAppBar(
                  title: const Text('Notification Sample'),
                  pinned: true,
                  floating: true,
                  bottom: TabBar(tabs: _tabs.map((String name) => Tab(text: name)).toList()),
                ),
              ];
            },
            body: TabBarView(
              children: <Widget>[
                ListView.builder(
                  itemCount: _months.length,
                  itemBuilder: (BuildContext context, int index) {
                    return ListTile(title: Text(_months[index]));
                  },
                ),
                ListView.builder(
                  itemCount: _days.length,
                  itemBuilder: (BuildContext context, int index) {
                    return ListTile(title: Text(_days[index]));
                  },
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
