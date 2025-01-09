// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_driver/driver_extension.dart';

void main() {
  enableFlutterDriverExtension();

  runApp(
    MaterialApp(
      home: Material(
        child: Builder(
          builder: (BuildContext context) {
            return TextButton(
              child: const Text('flutter drive lib/xxx.dart', textDirection: TextDirection.ltr),
              onPressed: () {
                Navigator.push<Object?>(
                  context,
                  MaterialPageRoute<Object?>(
                    builder: (BuildContext context) {
                      return const Material(
                        child: Center(
                          child: Text('navigated here', textDirection: TextDirection.ltr),
                        ),
                      );
                    },
                  ),
                );
              },
            );
          },
        ),
      ),
    ),
  );
}
