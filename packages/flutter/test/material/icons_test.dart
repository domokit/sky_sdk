// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:platform/platform.dart';

void main() {
  testWidgets('IconData object test', (WidgetTester tester) async {
    expect(Icons.account_balance, isNot(equals(Icons.account_box)));
    expect(Icons.account_balance.hashCode, isNot(equals(Icons.account_box.hashCode)));
    expect(Icons.account_balance, hasOneLineDescription);
  });

  testWidgets('Icons specify the material font', (WidgetTester tester) async {
    expect(Icons.clear.fontFamily, 'MaterialIcons');
    expect(Icons.search.fontFamily, 'MaterialIcons');
  });

  testWidgets('Certain icons (and their variants) match text direction', (
    WidgetTester tester,
  ) async {
    expect(Icons.arrow_back.matchTextDirection, true);
    expect(Icons.arrow_back_rounded.matchTextDirection, true);
    expect(Icons.arrow_back_outlined.matchTextDirection, true);
    expect(Icons.arrow_back_sharp.matchTextDirection, true);

    expect(Icons.access_time.matchTextDirection, false);
    expect(Icons.access_time_rounded.matchTextDirection, false);
    expect(Icons.access_time_outlined.matchTextDirection, false);
    expect(Icons.access_time_sharp.matchTextDirection, false);
  });

  testWidgets(
    'Adaptive icons are correct on cupertino platforms',
    (WidgetTester tester) async {
      expect(Icons.adaptive.arrow_back, Icons.arrow_back_ios);
      expect(Icons.adaptive.arrow_back_outlined, Icons.arrow_back_ios_outlined);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.iOS,
      TargetPlatform.macOS,
    }),
  );

  testWidgets(
    'Adaptive icons are correct on non-cupertino platforms',
    (WidgetTester tester) async {
      expect(Icons.adaptive.arrow_back, Icons.arrow_back);
      expect(Icons.adaptive.arrow_back_outlined, Icons.arrow_back_outlined);
    },
    variant: const TargetPlatformVariant(<TargetPlatform>{
      TargetPlatform.android,
      TargetPlatform.fuchsia,
      TargetPlatform.windows,
      TargetPlatform.linux,
    }),
  );

  testWidgets('A sample of icons look as expected', (WidgetTester tester) async {
    await _loadIconFont();

    await tester.pumpWidget(
      const MaterialApp(
        home: IconTheme(
          data: IconThemeData(size: 200),
          child: Wrap(
            children: <Icon>[
              Icon(Icons.ten_k),
              Icon(Icons.ac_unit),
              Icon(Icons.local_taxi),
              Icon(Icons.local_taxi_outlined),
              Icon(Icons.local_taxi_rounded),
              Icon(Icons.local_taxi_sharp),
              Icon(Icons.zoom_out_sharp),
            ],
          ),
        ),
      ),
    );

    await expectLater(find.byType(Wrap), matchesGoldenFile('test.icons.sample.png'));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/39998

  // Regression test for https://github.com/flutter/flutter/issues/95886
  testWidgets('Another sample of icons look as expected', (WidgetTester tester) async {
    await _loadIconFont();

    await tester.pumpWidget(
      const MaterialApp(
        home: IconTheme(
          data: IconThemeData(size: 200),
          child: Wrap(
            children: <Icon>[
              Icon(Icons.water_drop),
              Icon(Icons.water_drop_outlined),
              Icon(Icons.water_drop_rounded),
              Icon(Icons.water_drop_sharp),
            ],
          ),
        ),
      ),
    );

    await expectLater(find.byType(Wrap), matchesGoldenFile('test.icons.sample2.png'));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/39998

  testWidgets('Another sample of icons look as expected', (WidgetTester tester) async {
    await _loadIconFont();

    await tester.pumpWidget(
      const MaterialApp(
        home: IconTheme(
          data: IconThemeData(size: 200),
          child: Wrap(
            children: <Icon>[
              Icon(Icons.electric_bolt),
              Icon(Icons.electric_bolt_outlined),
              Icon(Icons.electric_bolt_rounded),
              Icon(Icons.electric_bolt_sharp),
            ],
          ),
        ),
      ),
    );

    await expectLater(find.byType(Wrap), matchesGoldenFile('test.icons.sample3.png'));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/39998

  // Regression test for https://github.com/flutter/flutter/issues/103202.
  testWidgets('Another sample of icons look as expected', (WidgetTester tester) async {
    await _loadIconFont();

    await tester.pumpWidget(
      const MaterialApp(
        home: IconTheme(
          data: IconThemeData(size: 200),
          child: Wrap(
            children: <Icon>[
              Icon(Icons.repeat_on),
              Icon(Icons.repeat_on_outlined),
              Icon(Icons.repeat_on_rounded),
              Icon(Icons.repeat_on_sharp),
            ],
          ),
        ),
      ),
    );

    await expectLater(find.byType(Wrap), matchesGoldenFile('test.icons.sample4.png'));
  }, skip: isBrowser); // https://github.com/flutter/flutter/issues/39998
}

// Loads the cached material icon font.
// Only necessary for golden tests. Relies on the tool updating cached assets before
// running tests.
Future<void> _loadIconFont() async {
  const FileSystem fs = LocalFileSystem();
  const Platform platform = LocalPlatform();
  final Directory flutterRoot = fs.directory(platform.environment['FLUTTER_ROOT']);

  final File iconFont = flutterRoot.childFile(
    fs.path.join('bin', 'cache', 'artifacts', 'material_fonts', 'MaterialIcons-Regular.otf'),
  );

  final Future<ByteData> bytes = Future<ByteData>.value(
    iconFont.readAsBytesSync().buffer.asByteData(),
  );

  await (FontLoader('MaterialIcons')..addFont(bytes)).load();
}
