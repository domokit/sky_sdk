// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'use_cases.dart';

class AppBarUseCase extends UseCase {
  @override
  String get name => 'AppBar';

  @override
  String get route => '/app-bar';

  @override
  Widget build(BuildContext context) => const MainWidget();
}

class MainWidget extends StatefulWidget {
  const MainWidget({super.key});

  @override
  State<MainWidget> createState() => MainWidgetState();
}

class MainWidgetState extends State<MainWidget> {
  int currentIndex = 0;

  void _onChanged(int? value) {
    setState(() {
      currentIndex = value!;
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar:
          <PreferredSizeWidget>[
            AppBar(
              backgroundColor: Theme.of(context).colorScheme.inversePrimary,
              title: Semantics(headingLevel: 1, child: const Text('AppBar')),
            ),
            AppBar(
              backgroundColor: Theme.of(context).colorScheme.inversePrimary,
              title: Semantics(headingLevel: 1, child: const Text('AppBar')),
              actions: <Widget>[
                IconButton(
                  icon: const Icon(Icons.add_alert),
                  tooltip: 'Show Snackbar',
                  onPressed: () {
                    ScaffoldMessenger.of(
                      context,
                    ).showSnackBar(const SnackBar(content: Text('This is a snackbar')));
                  },
                ),
                IconButton(
                  icon: const Icon(Icons.navigate_next),
                  tooltip: 'Go to the next page',
                  onPressed: () {
                    Navigator.push(
                      context,
                      MaterialPageRoute<void>(
                        builder: (BuildContext context) {
                          return Scaffold(
                            appBar: AppBar(
                              backgroundColor: Theme.of(context).colorScheme.inversePrimary,
                              title: Semantics(headingLevel: 1, child: const Text('Next Page')),
                            ),
                            body: const Center(
                              child: Text('This is the next page', style: TextStyle(fontSize: 24)),
                            ),
                          );
                        },
                      ),
                    );
                  },
                ),
              ],
            ),
            AppBar(
              backgroundColor: Theme.of(context).colorScheme.inversePrimary,
              title: Semantics(headingLevel: 1, child: const Text('AppBar')),
              actions: <Widget>[
                TextButton(onPressed: () {}, child: const Text('Action 1')),
                TextButton(onPressed: () {}, child: const Text('Action 2')),
              ],
            ),
          ][currentIndex],
      body: ListView(
        children: <Widget>[
          RadioListTile<int>(
            title: const Text('1. Simple app bar'),
            value: 0,
            groupValue: currentIndex,
            onChanged: _onChanged,
          ),
          RadioListTile<int>(
            title: const Text('2. App bar with actions'),
            value: 1,
            groupValue: currentIndex,
            onChanged: _onChanged,
          ),
          RadioListTile<int>(
            title: const Text('3. App bar with text buttons'),
            value: 2,
            groupValue: currentIndex,
            onChanged: _onChanged,
          ),
        ],
      ),
    );
  }
}
