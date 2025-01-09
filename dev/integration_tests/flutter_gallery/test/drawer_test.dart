// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_gallery/gallery/app.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();
  if (binding is LiveTestWidgetsFlutterBinding) {
    binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.fullyLive;
  }

  testWidgets('Flutter Gallery drawer item test', (WidgetTester tester) async {
    bool hasFeedback = false;

    await tester.pumpWidget(
      GalleryApp(
        testMode: true,
        onSendFeedback: () {
          hasFeedback = true;
        },
      ),
    );
    await tester.pump(); // see https://github.com/flutter/flutter/issues/1865
    await tester.pump(); // triggers a frame

    // Show the options page
    await tester.tap(find.byTooltip('Toggle options page'));
    await tester.pumpAndSettle();

    // Verify theme settings
    MaterialApp app = find.byType(MaterialApp).evaluate().first.widget as MaterialApp;
    expect(app.theme!.brightness, equals(Brightness.light));
    expect(app.darkTheme!.brightness, equals(Brightness.dark));

    // Switch to the dark theme: first menu button, choose 'Dark'
    await tester.tap(find.byIcon(Icons.arrow_drop_down).first);
    await tester.pumpAndSettle();
    await tester.tap(find.text('Dark'));
    await tester.pumpAndSettle();
    app = find.byType(MaterialApp).evaluate().first.widget as MaterialApp;
    expect(app.themeMode, ThemeMode.dark);

    // Switch to the light theme: first menu button, choose 'Light'
    await tester.tap(find.byIcon(Icons.arrow_drop_down).first);
    await tester.pumpAndSettle();
    await tester.tap(find.text('Light'));
    await tester.pumpAndSettle();
    app = find.byType(MaterialApp).evaluate().first.widget as MaterialApp;
    expect(app.themeMode, ThemeMode.light);

    // Switch back to system theme setting: first menu button, choose 'System Default'
    await tester.tap(find.byIcon(Icons.arrow_drop_down).first);
    await tester.pumpAndSettle();
    await tester.tap(
      find.descendant(
        of: find.byWidgetPredicate(
          (Widget widget) => widget.runtimeType.toString() == 'PopupMenuItem<ThemeMode>',
        ),
        matching: find.text('System Default'),
      ),
    );
    await tester.pumpAndSettle();
    app = find.byType(MaterialApp).evaluate().first.widget as MaterialApp;
    expect(app.themeMode, ThemeMode.system);

    // Verify density settings
    expect(app.theme!.visualDensity, equals(VisualDensity.standard));

    // Popup the density menu: third menu button, choose 'Compact'
    await tester.tap(find.byIcon(Icons.arrow_drop_down).at(2));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Compact'));
    await tester.pumpAndSettle();
    app = find.byType(MaterialApp).evaluate().first.widget as MaterialApp;
    expect(app.theme!.visualDensity, equals(VisualDensity.compact));

    await tester.tap(find.byIcon(Icons.arrow_drop_down).at(2));
    await tester.pumpAndSettle();
    await tester.tap(
      find.descendant(
        of: find.byWidgetPredicate(
          (Widget widget) =>
              widget.runtimeType.toString() == 'PopupMenuItem<GalleryVisualDensityValue>',
        ),
        matching: find.text('System Default'),
      ),
    );
    await tester.pumpAndSettle();
    app = find.byType(MaterialApp).evaluate().first.widget as MaterialApp;
    expect(app.theme!.visualDensity, equals(VisualDensity.standard));

    // Verify platform settings
    expect(app.theme!.platform, equals(TargetPlatform.android));

    // Popup the platform menu: fourth menu button, choose 'Cupertino'
    await tester.tap(find.byIcon(Icons.arrow_drop_down).at(3));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Cupertino').at(1));
    await tester.pumpAndSettle();
    app = find.byType(MaterialApp).evaluate().first.widget as MaterialApp;
    expect(app.theme!.platform, equals(TargetPlatform.iOS));

    // Verify the font scale.
    final Size origTextSize = tester.getSize(find.text('Text size'));
    expect(origTextSize, equals(const Size(144.0, 16.0)));

    // Popup the text size menu: second menu button, choose 'Small'
    await tester.tap(find.byIcon(Icons.arrow_drop_down).at(1));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Small'));
    await tester.pumpAndSettle();
    Size textSize = tester.getSize(find.text('Text size'));
    expect(textSize, equals(within(distance: 0.05, from: const Size(115.2, 13.0))));

    // Set font scale back to the default.
    await tester.tap(find.byIcon(Icons.arrow_drop_down).at(1));
    await tester.pumpAndSettle();
    await tester.tap(
      find.descendant(
        of: find.byWidgetPredicate(
          (Widget widget) =>
              widget.runtimeType.toString() == 'PopupMenuItem<GalleryTextScaleValue>',
        ),
        matching: find.text('System Default'),
      ),
    );
    await tester.pumpAndSettle();
    textSize = tester.getSize(find.text('Text size'));
    expect(textSize, origTextSize);

    // Switch to slow animation: second switch control
    expect(timeDilation, 1.0);
    await tester.tap(find.byType(Switch).at(1));
    await tester.pumpAndSettle();
    expect(timeDilation, greaterThan(1.0));

    // Restore normal animation: second switch control
    await tester.tap(find.byType(Switch).at(1));
    await tester.pumpAndSettle();
    expect(timeDilation, 1.0);

    // Send feedback.
    expect(hasFeedback, false);

    // Scroll to the end
    await tester.drag(find.text('Text size'), const Offset(0.0, -1000.0));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Send feedback'));
    await tester.pumpAndSettle();
    expect(hasFeedback, true);

    // Hide the options page
    await tester.tap(find.byTooltip('Toggle options page'));
    await tester.pumpAndSettle();
  });
}
