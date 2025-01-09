// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets("builder doesn't get called if app doesn't change", (WidgetTester tester) async {
    final List<String> log = <String>[];
    final Widget app = MaterialApp(
      home: const Placeholder(),
      builder: (BuildContext context, Widget? child) {
        log.add('build');
        expect(Directionality.of(context), TextDirection.ltr);
        expect(child, isA<FocusScope>());
        return const Placeholder();
      },
    );
    await tester.pumpWidget(Directionality(textDirection: TextDirection.rtl, child: app));
    expect(log, <String>['build']);
    await tester.pumpWidget(Directionality(textDirection: TextDirection.ltr, child: app));
    expect(log, <String>['build']);
  });

  testWidgets("builder doesn't get called if app doesn't change", (WidgetTester tester) async {
    final List<String> log = <String>[];
    await tester.pumpWidget(
      MaterialApp(
        home: Builder(
          builder: (BuildContext context) {
            log.add('build');
            expect(Directionality.of(context), TextDirection.rtl);
            return const Placeholder();
          },
        ),
        builder: (BuildContext context, Widget? child) {
          return Directionality(textDirection: TextDirection.rtl, child: child!);
        },
      ),
    );
    expect(log, <String>['build']);
  });
}
