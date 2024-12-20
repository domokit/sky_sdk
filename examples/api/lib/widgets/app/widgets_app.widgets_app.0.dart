// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

/// Flutter code sample for [WidgetsApp].

void main() => runApp(const WidgetsAppExampleApp());

class WidgetsAppExampleApp extends StatelessWidget {
  const WidgetsAppExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return WidgetsApp(
      title: 'Example',
      color: const Color(0xFF000000),
      home: const Center(child: Text('Hello World')),
      pageRouteBuilder:
          <T>(RouteSettings settings, WidgetBuilder builder) => PageRouteBuilder<T>(
            settings: settings,
            pageBuilder:
                (
                  BuildContext context,
                  Animation<double> animation,
                  Animation<double> secondaryAnimation,
                ) => builder(context),
          ),
    );
  }
}
