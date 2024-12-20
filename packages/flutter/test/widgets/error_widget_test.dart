// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  testWidgets('ErrorWidget displays actual error when throwing during build', (
    WidgetTester tester,
  ) async {
    final Key container = UniqueKey();
    const String errorText = 'Oh no, there was a crash!!1';

    await tester.pumpWidget(
      Container(
        key: container,
        color: Colors.red,
        padding: const EdgeInsets.all(10),
        child: Builder(
          builder: (BuildContext context) {
            throw UnsupportedError(errorText);
          },
        ),
      ),
    );

    expect(
      tester.takeException(),
      isA<UnsupportedError>().having(
        (UnsupportedError error) => error.message,
        'message',
        contains(errorText),
      ),
    );

    final ErrorWidget errorWidget = tester.widget(find.byType(ErrorWidget));
    expect(errorWidget.message, contains(errorText));

    // Failure in one widget shouldn't ripple through the entire tree and effect
    // ancestors. Those should still be in the tree.
    expect(find.byKey(container), findsOneWidget);
  });

  testWidgets(
    'when constructing an ErrorWidget due to a build failure throws an error, fail gracefully',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // leaking by design because of exception
    (WidgetTester tester) async {
      final Key container = UniqueKey();
      await tester.pumpWidget(
        Container(
          key: container,
          color: Colors.red,
          padding: const EdgeInsets.all(10),
          // This widget throws during build, which causes the construction of an
          // ErrorWidget with the build error. However, during construction of
          // that ErrorWidget, another error is thrown.
          child: const MyDoubleThrowingWidget(),
        ),
      );

      expect(
        tester.takeException(),
        isA<UnsupportedError>().having(
          (UnsupportedError error) => error.message,
          'message',
          contains(MyThrowingElement.debugFillPropertiesErrorMessage),
        ),
      );

      final ErrorWidget errorWidget = tester.widget(find.byType(ErrorWidget));
      expect(errorWidget.message, contains(MyThrowingElement.debugFillPropertiesErrorMessage));

      // Failure in one widget shouldn't ripple through the entire tree and effect
      // ancestors. Those should still be in the tree.
      expect(find.byKey(container), findsOneWidget);
    },
  );
}

// This widget throws during its regular build and then again when the
// ErrorWidget is constructed, which calls MyThrowingElement.debugFillProperties.
class MyDoubleThrowingWidget extends StatelessWidget {
  const MyDoubleThrowingWidget({super.key});

  @override
  StatelessElement createElement() => MyThrowingElement(this);

  @override
  Widget build(BuildContext context) {
    throw UnsupportedError('You cannot build me!');
  }
}

class MyThrowingElement extends StatelessElement {
  MyThrowingElement(super.widget);

  static const String debugFillPropertiesErrorMessage = 'Crash during debugFillProperties';

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    throw UnsupportedError(debugFillPropertiesErrorMessage);
  }
}
