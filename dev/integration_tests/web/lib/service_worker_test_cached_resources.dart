// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

Future<void> main() async {
  runApp(
    const Directionality(
      textDirection: TextDirection.ltr,
      child: Scaffold(
        body: Center(
          child: Column(
            children: <Widget>[
              Icon(Icons.ac_unit),
              Text('Hello, World', textDirection: TextDirection.ltr),
            ],
          ),
        ),
      ),
    ),
  );
}
