// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'package:flutter_api_samples/painting/star_border/star_border.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  Finder getStartBorderFinder(StarBorder shape) {
    return find.byWidgetPredicate(
      (Widget widget) => widget is example.ExampleBorder && widget.border == shape,
    );
  }

  testWidgets('Initial content is visible', (WidgetTester tester) async {
    await tester.pumpWidget(const example.StarBorderApp());

    expect(find.widgetWithText(AppBar, 'StarBorder Example'), findsOne);
    expect(find.text('Star'), findsOne);
    expect(find.text('Polygon'), findsOne);
    expect(find.byType(SelectableText), findsExactly(2));
    expect(find.byType(example.ExampleBorder), findsExactly(2));

    expect(find.byType(Slider), findsExactly(6));
    expect(find.text('Point Rounding'), findsOne);
    expect(find.text('Valley Rounding'), findsOne);
    expect(find.text('Squash'), findsOne);
    expect(find.text('Rotation'), findsOne);
    expect(find.text('Points'), findsOne);
    expect(find.widgetWithText(OutlinedButton, 'Nearest'), findsOne);
    expect(find.text('Inner Radius'), findsOne);
    expect(find.widgetWithText(ElevatedButton, 'Reset'), findsOne);
  });

  testWidgets('StartBorder uses the values from the sliders', (WidgetTester tester) async {
    await tester.pumpWidget(const example.StarBorderApp());

    expect(find.text('0.00'), findsExactly(4));
    expect(find.text('5.0'), findsOne);
    expect(find.text('0.40'), findsOne);
    expect(
      getStartBorderFinder(
        const StarBorder(
          side: BorderSide(),
          // The default values of the example are the same as the default
          // values of the constructor.
        ),
      ),
      findsOne,
    );
    expect(
      find.text('''
Container(
  decoration: ShapeDecoration(
    shape: StarBorder(
      points: 5.00,
      rotation: 0.00,
      innerRadiusRatio: 0.40,
      pointRounding: 0.00,
      valleyRounding: 0.00,
      squash: 0.00,
    ),
  ),
);'''),
      findsOne,
    );

    // Put all the sliders to the middle.
    for (int i = 0; i < 6; i++) {
      await tester.tap(find.byType(Slider).at(i));
      await tester.pump();
    }

    expect(find.text('0.50'), findsExactly(4));
    expect(find.text('11.5'), findsOne);
    expect(find.text('180.00'), findsOne);
    expect(
      getStartBorderFinder(
        const StarBorder(
          side: BorderSide(),
          points: 11.5,
          innerRadiusRatio: 0.5,
          pointRounding: 0.5,
          valleyRounding: 0.5,
          rotation: 180,
          squash: 0.5,
        ),
      ),
      findsOne,
    );
    expect(
      find.text('''
Container(
  decoration: ShapeDecoration(
    shape: StarBorder(
      points: 11.50,
      rotation: 180.00,
      innerRadiusRatio: 0.50,
      pointRounding: 0.50,
      valleyRounding: 0.50,
      squash: 0.50,
    ),
  ),
);'''),
      findsOne,
    );
  });

  testWidgets('StartBorder.polygon uses the values from the sliders', (WidgetTester tester) async {
    await tester.pumpWidget(const example.StarBorderApp());

    expect(find.text('0.00'), findsExactly(4));
    expect(find.text('5.0'), findsOne);
    expect(find.text('0.40'), findsOne);
    expect(getStartBorderFinder(const StarBorder(side: BorderSide())), findsOne);
    expect(
      find.text('''
Container(
  decoration: ShapeDecoration(
    shape: StarBorder(
      points: 5.00,
      rotation: 0.00,
      innerRadiusRatio: 0.40,
      pointRounding: 0.00,
      valleyRounding: 0.00,
      squash: 0.00,
    ),
  ),
);'''),
      findsOne,
    );

    // Put all the sliders to the middle.
    for (int i = 0; i < 6; i++) {
      await tester.tap(find.byType(Slider).at(i));
      await tester.pump();
    }

    expect(find.text('0.50'), findsExactly(4));
    expect(find.text('11.5'), findsOne);
    expect(find.text('180.00'), findsOne);
    expect(
      getStartBorderFinder(
        const StarBorder(
          side: BorderSide(),
          points: 11.5,
          innerRadiusRatio: 0.5,
          pointRounding: 0.5,
          valleyRounding: 0.5,
          rotation: 180,
          squash: 0.5,
        ),
      ),
      findsOne,
    );
    expect(
      find.text('''
Container(
  decoration: ShapeDecoration(
    shape: StarBorder(
      points: 11.50,
      rotation: 180.00,
      innerRadiusRatio: 0.50,
      pointRounding: 0.50,
      valleyRounding: 0.50,
      squash: 0.50,
    ),
  ),
);'''),
      findsOne,
    );
  });

  testWidgets('The "Nearest" button rounds the number of points', (WidgetTester tester) async {
    await tester.pumpWidget(const example.StarBorderApp());

    expect(find.text('5.0'), findsOne);

    // Put the points slider to the middle.
    await tester.tap(find.byType(Slider).at(4));
    await tester.pump();

    expect(find.text('11.5'), findsOne);

    await tester.tap(find.widgetWithText(OutlinedButton, 'Nearest'));
    await tester.pump();

    expect(find.text('12.0'), findsOne);
  });

  testWidgets('The "Reset" button resets the parameters to the default values', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.StarBorderApp());

    expect(find.text('0.00'), findsExactly(4));
    expect(find.text('5.0'), findsOne);
    expect(find.text('0.40'), findsOne);

    // Put all the sliders to the middle.
    for (int i = 0; i < 6; i++) {
      await tester.tap(find.byType(Slider).at(i));
      await tester.pump();
    }

    expect(find.text('0.50'), findsExactly(4));
    expect(find.text('11.5'), findsOne);
    expect(find.text('180.00'), findsOne);

    await tester.tap(find.widgetWithText(ElevatedButton, 'Reset'));
    await tester.pump();

    expect(find.text('0.00'), findsExactly(4));
    expect(find.text('5.0'), findsOne);
    expect(find.text('0.40'), findsOne);
  });
}
