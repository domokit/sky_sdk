// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

/// Flutter code sample for [FontFeature.contextualAlternates].

void main() => runApp(const ExampleApp());

class ExampleApp extends StatelessWidget {
  const ExampleApp({super.key});

  @override
  Widget build(BuildContext context) {
    return WidgetsApp(
      builder: (BuildContext context, Widget? navigator) => const ExampleWidget(),
      color: const Color(0xffffffff),
    );
  }
}

class ExampleWidget extends StatelessWidget {
  const ExampleWidget({super.key});

  @override
  Widget build(BuildContext context) {
    // The Barriecito font can be downloaded from Google Fonts
    // (https://www.google.com/fonts).
    return const Text(
      "Ooohh, we weren't going to tell him that.",
      style: TextStyle(
        fontFamily: 'Barriecito',
        fontFeatures: <FontFeature>[FontFeature.contextualAlternates()],
      ),
    );
  }
}
