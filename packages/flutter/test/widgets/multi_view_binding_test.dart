// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

void main() {
  testWidgets('runApp uses deprecated pipelineOwner and renderView', (WidgetTester tester) async {
    runApp(const SizedBox());
    final RenderObject renderObject = tester.renderObject(find.byType(SizedBox));

    RenderObject parent = renderObject;
    while (parent.parent != null) {
      parent = parent.parent!;
    }
    expect(parent, isA<RenderView>());
    expect(parent, equals(tester.binding.renderView));

    expect(renderObject.owner, equals(tester.binding.pipelineOwner));
  });

  testWidgets('can manually attach RootWidget to build owner', (WidgetTester tester) async {
    expect(find.byType(ColoredBox), findsNothing);

    final RootWidget rootWidget = RootWidget(
      child: View(
        view: FakeFlutterView(tester.view),
        child: const ColoredBox(color: Colors.orange),
      ),
    );
    tester.binding.attachToBuildOwner(rootWidget);
    await tester.pump();
    expect(find.byType(ColoredBox), findsOneWidget);
    expect(tester.binding.rootElement!.widget, equals(rootWidget));
    expect(tester.element(find.byType(ColoredBox)).owner, equals(tester.binding.buildOwner));
  });

  testWidgets(
    'runApp throws if given a View',
    experimentalLeakTesting:
        LeakTesting.settings.withIgnoredAll(), // The test leaks by design because of exception.
    (WidgetTester tester) async {
      runApp(View(view: FakeFlutterView(tester.view), child: const SizedBox.shrink()));
      expect(
        tester.takeException(),
        isFlutterError.having(
          (FlutterError e) => e.message,
          'message',
          contains('passing it to "runWidget" instead of "runApp"'),
        ),
      );
    },
  );

  testWidgets('runWidget throws if not given a View', (WidgetTester tester) async {
    runWidget(const SizedBox.shrink());
    expect(
      tester.takeException(),
      isFlutterError.having(
        (FlutterError e) => e.message,
        'message',
        contains('Try wrapping your widget in a View widget'),
      ),
    );
  });

  testWidgets('runWidget does not throw if given a View', (WidgetTester tester) async {
    runWidget(View(view: FakeFlutterView(tester.view), child: const SizedBox.shrink()));
    expect(find.byType(View), findsOne);
  });

  testWidgets('can call runWidget with an empty ViewCollection', (WidgetTester tester) async {
    runWidget(const ViewCollection(views: <Widget>[]));
    expect(find.byType(ViewCollection), findsOne);
  });
}

class FakeFlutterView extends TestFlutterView {
  FakeFlutterView(TestFlutterView view)
    : super(view: view, display: view.display, platformDispatcher: view.platformDispatcher);
}
