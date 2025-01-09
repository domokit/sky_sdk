// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_api_samples/painting/image_provider/image_provider.0.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('$CustomNetworkImage', (WidgetTester tester) async {
    const String expectedUrl =
        'https://flutter.github.io/assets-for-api-docs/assets/widgets/flamingos.jpg?dpr=3.0&locale=en-US&platform=android&width=800.0&height=600.0&bidi=ltr';
    final List<String> log = <String>[];
    final DebugPrintCallback originalDebugPrint = debugPrint;
    debugPrint = (String? message, {int? wrapWidth}) {
      log.add('$message');
    };
    await tester.pumpWidget(const ExampleApp());
    expect(tester.takeException().toString(), 'Exception: Invalid image data');
    expect(log, <String>['Fetching "$expectedUrl"...']);
    debugPrint = originalDebugPrint;
  });
}
