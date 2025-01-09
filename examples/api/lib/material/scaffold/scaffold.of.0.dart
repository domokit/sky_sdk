// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [Scaffold.of].

void main() => runApp(const OfExampleApp());

class OfExampleApp extends StatelessWidget {
  const OfExampleApp({super.key});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      theme: ThemeData(primarySwatch: Colors.blue),
      home: Scaffold(
        body: const MyScaffoldBody(),
        appBar: AppBar(title: const Text('Scaffold.of Example')),
      ),
      color: Colors.white,
    );
  }
}

class MyScaffoldBody extends StatelessWidget {
  const MyScaffoldBody({super.key});

  @override
  Widget build(BuildContext context) {
    return Center(
      child: ElevatedButton(
        child: const Text('SHOW BOTTOM SHEET'),
        onPressed: () {
          Scaffold.of(context).showBottomSheet((BuildContext context) {
            return Container(
              alignment: Alignment.center,
              height: 200,
              color: Colors.amber,
              child: Center(
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    const Text('BottomSheet'),
                    ElevatedButton(
                      child: const Text('Close BottomSheet'),
                      onPressed: () {
                        Navigator.pop(context);
                      },
                    ),
                  ],
                ),
              ),
            );
          });
        },
      ),
    );
  }
}
