// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

enum RadiusType { Sharp, Shifting, Round }

void matches(BorderRadius? borderRadius, RadiusType top, RadiusType bottom) {
  final Radius cardRadius = kMaterialEdges[MaterialType.card]!.topLeft;

  switch (top) {
    case RadiusType.Sharp:
      expect(borderRadius?.topLeft, equals(Radius.zero));
      expect(borderRadius?.topRight, equals(Radius.zero));
    case RadiusType.Shifting:
      expect(borderRadius?.topLeft.x, greaterThan(0.0));
      expect(borderRadius?.topLeft.x, lessThan(cardRadius.x));
      expect(borderRadius?.topLeft.y, greaterThan(0.0));
      expect(borderRadius?.topLeft.y, lessThan(cardRadius.y));
      expect(borderRadius?.topRight.x, greaterThan(0.0));
      expect(borderRadius?.topRight.x, lessThan(cardRadius.x));
      expect(borderRadius?.topRight.y, greaterThan(0.0));
      expect(borderRadius?.topRight.y, lessThan(cardRadius.y));
    case RadiusType.Round:
      expect(borderRadius?.topLeft, equals(cardRadius));
      expect(borderRadius?.topRight, equals(cardRadius));
  }

  switch (bottom) {
    case RadiusType.Sharp:
      expect(borderRadius?.bottomLeft, equals(Radius.zero));
      expect(borderRadius?.bottomRight, equals(Radius.zero));
    case RadiusType.Shifting:
      expect(borderRadius?.bottomLeft.x, greaterThan(0.0));
      expect(borderRadius?.bottomLeft.x, lessThan(cardRadius.x));
      expect(borderRadius?.bottomLeft.y, greaterThan(0.0));
      expect(borderRadius?.bottomLeft.y, lessThan(cardRadius.y));
      expect(borderRadius?.bottomRight.x, greaterThan(0.0));
      expect(borderRadius?.bottomRight.x, lessThan(cardRadius.x));
      expect(borderRadius?.bottomRight.y, greaterThan(0.0));
      expect(borderRadius?.bottomRight.y, lessThan(cardRadius.y));
    case RadiusType.Round:
      expect(borderRadius?.bottomLeft, equals(cardRadius));
      expect(borderRadius?.bottomRight, equals(cardRadius));
  }
}

// Returns the border radius decoration of an item within a MergeableMaterial.
// This depends on the exact structure of objects built by the Material and
// MergeableMaterial widgets.
BorderRadius? getBorderRadius(WidgetTester tester, int index) {
  final List<Element> containers = tester.elementList(find.byType(Container)).toList();

  final Container container = containers[index].widget as Container;
  final BoxDecoration? boxDecoration = container.decoration as BoxDecoration?;

  return boxDecoration!.borderRadius as BorderRadius?;
}

void main() {
  testWidgets('MergeableMaterial empty', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: SingleChildScrollView(child: MergeableMaterial()))),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(0));
  });

  testWidgets('MergeableMaterial update slice', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(100.0));

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 200.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(200.0));
  });

  testWidgets('MergeableMaterial swap slices', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(200.0));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(200.0));

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(200.0));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Round);
  });

  testWidgets('MergeableMaterial paints shadows', (WidgetTester tester) async {
    debugDisableShadows = false;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RRect rrect = kMaterialEdges[MaterialType.card]!.toRRect(
      const Rect.fromLTRB(0.0, 0.0, 800.0, 100.0),
    );
    expect(
      find.byType(MergeableMaterial),
      paints
        ..shadow(elevation: 2.0)
        ..rrect(rrect: rrect, color: Colors.white, hasMaskFilter: false),
    );
    debugDisableShadows = true;
  });

  testWidgets('MergeableMaterial skips shadow for zero elevation', (WidgetTester tester) async {
    debugDisableShadows = false;
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              elevation: 0,
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(find.byType(MergeableMaterial), isNot(paints..shadow(elevation: 0.0)));
    debugDisableShadows = true;
  });

  testWidgets('MergeableMaterial merge gap', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('x')),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, lessThan(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Shifting);
    matches(getBorderRadius(tester, 1), RadiusType.Shifting, RadiusType.Round);

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(200));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Round);
  });

  testWidgets('MergeableMaterial separate slices', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(200));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('x')),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, lessThan(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Shifting);
    matches(getBorderRadius(tester, 1), RadiusType.Shifting, RadiusType.Round);

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);
  });

  testWidgets('MergeableMaterial separate merge separate', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(200));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('x')),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, lessThan(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Shifting);
    matches(getBorderRadius(tester, 1), RadiusType.Shifting, RadiusType.Round);

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, lessThan(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Shifting);
    matches(getBorderRadius(tester, 1), RadiusType.Shifting, RadiusType.Round);

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(200));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('x')),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, lessThan(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Shifting);
    matches(getBorderRadius(tester, 1), RadiusType.Shifting, RadiusType.Round);

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);
  });

  testWidgets('MergeableMaterial insert slice', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(200));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    expect(box.size.height, equals(300));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Sharp);
    matches(getBorderRadius(tester, 2), RadiusType.Sharp, RadiusType.Round);
  });

  testWidgets('MergeableMaterial remove slice', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(300));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Sharp);
    matches(getBorderRadius(tester, 2), RadiusType.Sharp, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump();
    expect(box.size.height, equals(200));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Round);
  });

  testWidgets('MergeableMaterial insert chunk', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(200));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('x')),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('y')),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, lessThan(332));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Shifting);
    matches(getBorderRadius(tester, 1), RadiusType.Shifting, RadiusType.Shifting);
    matches(getBorderRadius(tester, 2), RadiusType.Shifting, RadiusType.Round);

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(332));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 2), RadiusType.Round, RadiusType.Round);
  });

  testWidgets('MergeableMaterial remove chunk', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('x')),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('y')),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(332));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 2), RadiusType.Round, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, lessThan(332));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Shifting);
    matches(getBorderRadius(tester, 1), RadiusType.Shifting, RadiusType.Round);

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(200));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Sharp);
    matches(getBorderRadius(tester, 1), RadiusType.Sharp, RadiusType.Round);
  });

  testWidgets('MergeableMaterial replace gap with chunk', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('x')),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('y')),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('z')),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, lessThan(332));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Shifting);
    matches(getBorderRadius(tester, 1), RadiusType.Shifting, RadiusType.Shifting);
    matches(getBorderRadius(tester, 2), RadiusType.Shifting, RadiusType.Round);

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(332));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 2), RadiusType.Round, RadiusType.Round);
  });

  testWidgets('MergeableMaterial replace chunk with gap', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('x')),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('y')),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(332));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 2), RadiusType.Round, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('z')),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, lessThan(332));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Shifting);
    matches(getBorderRadius(tester, 1), RadiusType.Shifting, RadiusType.Round);

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);
  });

  testWidgets('MergeableMaterial insert and separate slice', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final RenderBox box = tester.renderObject(find.byType(MergeableMaterial));
    expect(box.size.height, equals(100));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);

    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('x')),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, lessThan(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Shifting);
    matches(getBorderRadius(tester, 1), RadiusType.Shifting, RadiusType.Round);

    await tester.pump(const Duration(milliseconds: 100));
    expect(box.size.height, equals(216));

    matches(getBorderRadius(tester, 0), RadiusType.Round, RadiusType.Round);
    matches(getBorderRadius(tester, 1), RadiusType.Round, RadiusType.Round);
  });

  bool isDivider(BoxDecoration decoration, bool top, bool bottom) {
    const BorderSide side = BorderSide(color: Color(0x1F000000), width: 0.5);

    return decoration ==
        BoxDecoration(
          border: Border(
            top: top ? side : BorderSide.none,
            bottom: bottom ? side : BorderSide.none,
          ),
        );
  }

  testWidgets('MergeableMaterial dividers', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              hasDividers: true,
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('D'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    List<Widget> animatedContainers = tester.widgetList(find.byType(AnimatedContainer)).toList();
    List<BoxDecoration> boxes = <BoxDecoration>[];
    for (final Widget container in animatedContainers) {
      boxes.add((container as AnimatedContainer).decoration! as BoxDecoration);
    }

    int offset = 0;

    expect(isDivider(boxes[offset], false, true), isTrue);
    expect(isDivider(boxes[offset + 1], true, true), isTrue);
    expect(isDivider(boxes[offset + 2], true, true), isTrue);
    expect(isDivider(boxes[offset + 3], true, false), isTrue);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: const Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              hasDividers: true,
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialGap(key: ValueKey<String>('x')),
                MaterialSlice(
                  key: ValueKey<String>('C'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('D'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    // Wait for dividers to shrink.
    await tester.pump(const Duration(milliseconds: 200));

    animatedContainers = tester.widgetList(find.byType(AnimatedContainer)).toList();
    boxes = <BoxDecoration>[];

    for (final Widget container in animatedContainers) {
      boxes.add((container as AnimatedContainer).decoration! as BoxDecoration);
    }

    offset = 0;

    expect(isDivider(boxes[offset], false, true), isTrue);
    expect(isDivider(boxes[offset + 1], true, false), isTrue);
    expect(isDivider(boxes[offset + 2], false, true), isTrue);
    expect(isDivider(boxes[offset + 3], true, false), isTrue);
  });

  testWidgets('MergeableMaterial respects dividerColor', (WidgetTester tester) async {
    const Color dividerColor = Colors.red;
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              hasDividers: true,
              dividerColor: dividerColor,
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
                MaterialSlice(
                  key: ValueKey<String>('B'),
                  child: SizedBox(width: 100.0, height: 100.0),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final DecoratedBox decoratedBox = tester.widget(find.byType(DecoratedBox).last);
    final BoxDecoration decoration = decoratedBox.decoration as BoxDecoration;
    // Since we are getting the last DecoratedBox, it will have a Border.top.
    expect(decoration.border!.top.color, dividerColor);
  });

  testWidgets('MergeableMaterial respects MaterialSlice.color', (WidgetTester tester) async {
    const Color themeCardColor = Colors.red;
    const Color materialSliceColor = Colors.green;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(cardColor: themeCardColor),
        home: const Scaffold(
          body: SingleChildScrollView(
            child: MergeableMaterial(
              children: <MergeableMaterialItem>[
                MaterialSlice(
                  key: ValueKey<String>('A'),
                  color: materialSliceColor,
                  child: SizedBox(height: 100, width: 100),
                ),
                MaterialGap(key: ValueKey<String>('B')),
                MaterialSlice(key: ValueKey<String>('C'), child: SizedBox(height: 100, width: 100)),
              ],
            ),
          ),
        ),
      ),
    );

    BoxDecoration boxDecoration =
        tester.widget<Container>(find.byType(Container).first).decoration! as BoxDecoration;
    expect(boxDecoration.color, materialSliceColor);

    boxDecoration =
        tester.widget<Container>(find.byType(Container).last).decoration! as BoxDecoration;
    expect(boxDecoration.color, themeCardColor);
  });
}
