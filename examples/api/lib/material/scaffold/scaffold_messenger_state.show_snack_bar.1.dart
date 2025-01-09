// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [SnackBar].

void main() => runApp(const SnackBarApp());

class SnackBarApp extends StatelessWidget {
  const SnackBarApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(home: SnackBarExample());
  }
}

class SnackBarExample extends StatefulWidget {
  const SnackBarExample({super.key});

  @override
  State<SnackBarExample> createState() => _SnackBarExampleState();
}

class _SnackBarExampleState extends State<SnackBarExample> {
  bool _largeLogo = false;

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('SnackBar Sample')),
      body: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Column(
          children: <Widget>[
            ElevatedButton(
              onPressed: () {
                const SnackBar snackBar = SnackBar(
                  content: Text('A SnackBar has been shown.'),
                  behavior: SnackBarBehavior.floating,
                );
                ScaffoldMessenger.of(context).showSnackBar(snackBar);
              },
              child: const Text('Show SnackBar'),
            ),
            const SizedBox(height: 8.0),
            ElevatedButton(
              onPressed: () {
                setState(() => _largeLogo = !_largeLogo);
              },
              child: Text(_largeLogo ? 'Shrink Logo' : 'Grow Logo'),
            ),
          ],
        ),
      ),
      // A floating [SnackBar] is positioned above [Scaffold.floatingActionButton].
      // If the Widget provided to the floatingActionButton slot takes up too much space
      // for the SnackBar to be visible, an error will be thrown.
      floatingActionButton: Container(
        constraints: BoxConstraints.tightFor(
          width: 150,
          height: _largeLogo ? double.infinity : 150,
        ),
        decoration: const BoxDecoration(
          color: Colors.blueGrey,
          borderRadius: BorderRadius.all(Radius.circular(20)),
        ),
        child: const FlutterLogo(),
      ),
    );
  }
}
