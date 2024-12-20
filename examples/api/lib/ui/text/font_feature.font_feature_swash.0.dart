// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';

/// Flutter code sample for [FontFeature.swash].

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
    // The BioRhyme Expanded font can be downloaded from Google Fonts
    // (https://www.google.com/fonts).
    return const Text(
      'Queer & Romantic',
      style: TextStyle(
        fontFamily: 'BioRhyme Expanded',
        fontFeatures: <FontFeature>[FontFeature.swash()],
      ),
    );
  }
}
