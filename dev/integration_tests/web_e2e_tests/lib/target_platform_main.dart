// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';

void main() => runApp(const MyApp());

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(key: Key('mainapp'), title: 'Platform Test', home: MyHomePage());
  }
}

class MyHomePage extends StatelessWidget {
  const MyHomePage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            if (defaultTargetPlatform == TargetPlatform.macOS)
              const Text('I am running on MacOS', key: Key('macOSKey')),
            if (defaultTargetPlatform == TargetPlatform.iOS)
              const Text('I am running on MacOS', key: Key('iOSKey')),
            if (defaultTargetPlatform == TargetPlatform.android)
              const Text('I am running on Android', key: Key('androidKey')),
          ],
        ),
      ),
    );
  }
}
