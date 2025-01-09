// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_gallery/demo_lists.dart';
import 'package:flutter_gallery/gallery/app.dart' show GalleryApp;
import 'package:flutter_gallery/gallery/demos.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

import 'run_demos.dart';

// All of the gallery demos, identified as "title@category".
//
// These names are reported by the test app, see _handleMessages()
// in transitions_perf.dart.
List<String> _allDemos =
    kAllGalleryDemos.map((GalleryDemo demo) => '${demo.title}@${demo.category.name}').toList();

void main([List<String> args = const <String>[]]) {
  final bool withSemantics = args.contains('--with_semantics');
  final IntegrationTestWidgetsFlutterBinding binding =
      IntegrationTestWidgetsFlutterBinding.ensureInitialized();
  binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.fullyLive;
  group('flutter gallery transitions on e2e', () {
    testWidgets('find.bySemanticsLabel', (WidgetTester tester) async {
      runApp(const GalleryApp(testMode: true));
      await tester.pumpAndSettle();
      final int id = tester.getSemantics(find.bySemanticsLabel('Material')).id;
      expect(id, greaterThan(-1));
    }, skip: !withSemantics);

    testWidgets('all demos', (WidgetTester tester) async {
      runApp(const GalleryApp(testMode: true));
      await tester.pumpAndSettle();
      // Collect timeline data for just a limited set of demos to avoid OOMs.
      await binding.watchPerformance(() async {
        await runDemos(kProfiledDemos, tester);
      });

      // Execute the remaining tests.
      final Set<String> unprofiledDemos = Set<String>.from(_allDemos)..removeAll(kProfiledDemos);
      await runDemos(unprofiledDemos.toList(), tester);
    }, semanticsEnabled: withSemantics);
  });
}
