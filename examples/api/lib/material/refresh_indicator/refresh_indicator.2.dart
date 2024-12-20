// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [RefreshIndicator.noSpinner].

void main() => runApp(const RefreshIndicatorExampleApp());

class RefreshIndicatorExampleApp extends StatelessWidget {
  const RefreshIndicatorExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: RefreshIndicatorExample());
  }
}

class RefreshIndicatorExample extends StatefulWidget {
  const RefreshIndicatorExample({super.key});

  @override
  State<RefreshIndicatorExample> createState() => _RefreshIndicatorExampleState();
}

class _RefreshIndicatorExampleState extends State<RefreshIndicatorExample> {
  bool _isRefreshing = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('RefreshIndicator.noSpinner Sample')),
      body: Stack(
        children: <Widget>[
          RefreshIndicator.noSpinner(
            // Callback function used by the app to listen to the
            // status of the RefreshIndicator pull-down action.
            onStatusChange: (RefreshIndicatorStatus? status) {
              if (status == RefreshIndicatorStatus.done) {
                setState(() {
                  _isRefreshing = false;
                });
              }
            },

            // Callback that gets called whenever the user pulls down to refresh.
            onRefresh: () async {
              // This can be also done in onStatusChange when the status is RefreshIndicatorStatus.refresh.
              setState(() {
                _isRefreshing = true;
              });

              // Replace this delay with the code to be executed during refresh
              // and return asynchronous code.
              return Future<void>.delayed(const Duration(seconds: 3));
            },

            child: CustomScrollView(
              slivers: <Widget>[
                SliverList.builder(
                  itemCount: 20,
                  itemBuilder: (BuildContext context, int index) {
                    return ListTile(
                      tileColor: Colors.green[100],
                      title: const Text('Pull down here'),
                      subtitle: const Text('A custom refresh indicator will be shown'),
                    );
                  },
                ),
              ],
            ),
          ),

          // Shows an overlay with a CircularProgressIndicator when refreshing.
          if (_isRefreshing)
            ColoredBox(
              color: Colors.black45,
              child: Align(
                child: CircularProgressIndicator(
                  color: Colors.purple[500],
                  strokeWidth: 10,
                  semanticsLabel: 'Circular progress indicator',
                ),
              ),
            ),
        ],
      ),
    );
  }
}
