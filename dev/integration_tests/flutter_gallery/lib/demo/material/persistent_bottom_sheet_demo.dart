// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import '../../gallery/demo.dart';

class PersistentBottomSheetDemo extends StatefulWidget {
  const PersistentBottomSheetDemo({super.key});

  static const String routeName = '/material/persistent-bottom-sheet';

  @override
  State<PersistentBottomSheetDemo> createState() => _PersistentBottomSheetDemoState();
}

class _PersistentBottomSheetDemoState extends State<PersistentBottomSheetDemo> {
  final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey<ScaffoldState>();

  VoidCallback? _showBottomSheetCallback;

  @override
  void initState() {
    super.initState();
    _showBottomSheetCallback = _showBottomSheet;
  }

  void _showBottomSheet() {
    setState(() {
      // disable the button
      _showBottomSheetCallback = null;
    });
    _scaffoldKey.currentState!
        .showBottomSheet((BuildContext context) {
          final ThemeData themeData = Theme.of(context);
          return Container(
            decoration: BoxDecoration(
              border: Border(top: BorderSide(color: themeData.disabledColor)),
            ),
            child: Padding(
              padding: const EdgeInsets.all(32.0),
              child: Text(
                'This is a Material persistent bottom sheet. Drag downwards to dismiss it.',
                textAlign: TextAlign.center,
                style: TextStyle(color: themeData.colorScheme.secondary, fontSize: 24.0),
              ),
            ),
          );
        })
        .closed
        .whenComplete(() {
          if (mounted) {
            setState(() {
              // re-enable the button
              _showBottomSheetCallback = _showBottomSheet;
            });
          }
        });
  }

  void _showMessage() {
    showDialog<void>(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          content: const Text('You tapped the floating action button.'),
          actions: <Widget>[
            TextButton(
              onPressed: () {
                Navigator.pop(context);
              },
              child: const Text('OK'),
            ),
          ],
        );
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      key: _scaffoldKey,
      appBar: AppBar(
        title: const Text('Persistent bottom sheet'),
        actions: <Widget>[MaterialDemoDocumentationButton(PersistentBottomSheetDemo.routeName)],
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: _showMessage,
        backgroundColor: Colors.redAccent,
        child: const Icon(Icons.add, semanticLabel: 'Add'),
      ),
      body: Center(
        child: ElevatedButton(
          onPressed: _showBottomSheetCallback,
          child: const Text('SHOW BOTTOM SHEET'),
        ),
      ),
    );
  }
}
