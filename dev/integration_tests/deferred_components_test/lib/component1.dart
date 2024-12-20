// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

class LogoScreen extends StatelessWidget {
  const LogoScreen({super.key});

  static const String _testSentinel = 'Running deferred code';

  @override
  Widget build(BuildContext context) {
    print(_testSentinel);
    return Container(
      padding: const EdgeInsets.all(25),
      color: Colors.blue,
      child: Column(
        children: <Widget>[
          const Text('DeferredWidget', key: Key('DeferredWidget')),
          Image.asset('customassets/flutter_logo.png', key: const Key('DeferredImage')),
        ],
      ),
    );
  }
}
