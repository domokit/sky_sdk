// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

const String _text1 =
    'Snackbars provide lightweight feedback about an operation by '
    'showing a brief message at the bottom of the screen. Snackbars '
    'can contain an action.';

const String _text2 =
    'Snackbars should contain a single line of text directly related '
    'to the operation performed. They cannot contain icons.';

const String _text3 = 'By default snackbars automatically disappear after a few seconds ';

class SnackBarDemo extends StatefulWidget {
  const SnackBarDemo({super.key});

  static const String routeName = '/material/snack-bar';

  @override
  State<SnackBarDemo> createState() => _SnackBarDemoState();
}

class _SnackBarDemoState extends State<SnackBarDemo> {
  int _snackBarIndex = 1;

  Widget buildBody(BuildContext context) {
    return SafeArea(
      top: false,
      bottom: false,
      child: ListView(
        padding: const EdgeInsets.all(24.0),
        children:
            <Widget>[
              const Text(_text1),
              const Text(_text2),
              Center(
                child: ElevatedButton(
                  child: const Text('SHOW A SNACKBAR'),
                  onPressed: () {
                    final int thisSnackBarIndex = _snackBarIndex++;
                    ScaffoldMessenger.of(context).showSnackBar(
                      SnackBar(
                        content: Text('This is snackbar #$thisSnackBarIndex.'),
                        action: SnackBarAction(
                          label: 'ACTION',
                          onPressed: () {
                            ScaffoldMessenger.of(context).showSnackBar(
                              SnackBar(
                                content: Text("You pressed snackbar $thisSnackBarIndex's action."),
                              ),
                            );
                          },
                        ),
                      ),
                    );
                  },
                ),
              ),
              const Text(_text3),
            ].map<Widget>((Widget child) {
              return Container(margin: const EdgeInsets.symmetric(vertical: 12.0), child: child);
            }).toList(),
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Snackbar'),
        actions: <Widget>[MaterialDemoDocumentationButton(SnackBarDemo.routeName)],
      ),
      body: Builder(
        // Create an inner BuildContext so that the snackBar onPressed methods
        // can refer to the Scaffold with Scaffold.of().
        builder: buildBody,
      ),
      floatingActionButton: FloatingActionButton(
        tooltip: 'Create',
        child: const Icon(Icons.add),
        onPressed: () {
          print('Floating Action Button was pressed');
        },
      ),
    );
  }
}
