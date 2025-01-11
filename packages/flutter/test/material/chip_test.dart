// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file is run as part of a reduced test set in CI on Mac and Windows
// machines.
@Tags(<String>['reduced-test-set'])
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/feedback_tester.dart';
import '../widgets/semantics_tester.dart';

Finder findRenderChipElement() {
  return find.byElementPredicate((Element e) => '${e.renderObject.runtimeType}' == '_RenderChip');
}

RenderBox getMaterialBox(WidgetTester tester) {
  return tester.firstRenderObject<RenderBox>(
    find.descendant(of: find.byType(RawChip), matching: find.byType(CustomPaint)),
  );
}

Material getMaterial(WidgetTester tester) {
  return tester.widget<Material>(
    find.descendant(of: find.byType(RawChip), matching: find.byType(Material)),
  );
}

IconThemeData getIconData(WidgetTester tester) {
  final IconTheme iconTheme = tester.firstWidget(
    find.descendant(of: find.byType(RawChip), matching: find.byType(IconTheme)),
  );
  return iconTheme.data;
}

DefaultTextStyle getLabelStyle(WidgetTester tester, String labelText) {
  return tester.widget(
    find.ancestor(of: find.text(labelText), matching: find.byType(DefaultTextStyle)).first,
  );
}

TextStyle? getIconStyle(WidgetTester tester, IconData icon) {
  final RichText iconRichText = tester.widget<RichText>(
    find.descendant(of: find.byIcon(icon).first, matching: find.byType(RichText)),
  );
  return iconRichText.text.style;
}

dynamic getRenderChip(WidgetTester tester) {
  if (!tester.any(findRenderChipElement())) {
    return null;
  }
  final Element element = tester.element(findRenderChipElement().first);
  return element.renderObject;
}

double getSelectProgress(WidgetTester tester) {
  // ignore: avoid_dynamic_calls
  return getRenderChip(tester)?.checkmarkAnimation?.value as double;
}

double getAvatarDrawerProgress(WidgetTester tester) {
  // ignore: avoid_dynamic_calls
  return getRenderChip(tester)?.avatarDrawerAnimation?.value as double;
}

double getDeleteDrawerProgress(WidgetTester tester) {
  // ignore: avoid_dynamic_calls
  return getRenderChip(tester)?.deleteDrawerAnimation?.value as double;
}

/// Adds the basic requirements for a Chip.
Widget wrapForChip({
  required Widget child,
  TextDirection textDirection = TextDirection.ltr,
  TextScaler textScaler = TextScaler.noScaling,
  ThemeData? theme,
}) {
  return MaterialApp(
    theme: theme,
    home: Directionality(
      textDirection: textDirection,
      child: MediaQuery(
        data: MediaQueryData(textScaler: textScaler),
        child: Material(child: child),
      ),
    ),
  );
}

/// Tests that a [Chip] that has its size constrained by its parent is
/// further constraining the size of its child, the label widget.
/// Optionally, adding an avatar or delete icon to the chip should not
/// cause the chip or label to exceed its constrained height.
Future<void> testConstrainedLabel(
  WidgetTester tester, {
  CircleAvatar? avatar,
  VoidCallback? onDeleted,
}) async {
  const double labelWidth = 100.0;
  const double labelHeight = 50.0;
  const double chipParentWidth = 75.0;
  const double chipParentHeight = 25.0;
  final Key labelKey = UniqueKey();

  await tester.pumpWidget(
    wrapForChip(
      child: Center(
        child: SizedBox(
          width: chipParentWidth,
          height: chipParentHeight,
          child: Chip(
            avatar: avatar,
            label: SizedBox(key: labelKey, width: labelWidth, height: labelHeight),
            onDeleted: onDeleted,
          ),
        ),
      ),
    ),
  );

  final Size labelSize = tester.getSize(find.byKey(labelKey));
  expect(labelSize.width, lessThan(chipParentWidth));
  expect(labelSize.height, lessThanOrEqualTo(chipParentHeight));

  final Size chipSize = tester.getSize(find.byType(Chip));
  expect(chipSize.width, chipParentWidth);
  expect(chipSize.height, chipParentHeight);
}

void doNothing() {}

Widget chipWithOptionalDeleteButton({
  Key? deleteButtonKey,
  Key? labelKey,
  required bool deletable,
  TextDirection textDirection = TextDirection.ltr,
  String? chipTooltip,
  String? deleteButtonTooltipMessage,
  double? size,
  VoidCallback? onPressed = doNothing,
  ThemeData? themeData,
}) {
  return wrapForChip(
    textDirection: textDirection,
    theme: themeData,
    child: Wrap(
      children: <Widget>[
        RawChip(
          tooltip: chipTooltip,
          onPressed: onPressed,
          onDeleted: deletable ? doNothing : null,
          deleteIcon: Icon(key: deleteButtonKey, size: size, Icons.close),
          deleteButtonTooltipMessage: deleteButtonTooltipMessage,
          label: Text(
            deletable ? 'Chip with Delete Button' : 'Chip without Delete Button',
            key: labelKey,
          ),
        ),
      ],
    ),
  );
}

bool offsetsAreClose(Offset a, Offset b) => (a - b).distance < 1.0;
bool radiiAreClose(double a, double b) => (a - b).abs() < 1.0;

// Ripple pattern matches if there exists at least one ripple
// with the [expectedCenter] and [expectedRadius].
// This ensures the existence of a ripple.
PaintPattern ripplePattern(Offset expectedCenter, double expectedRadius) {
  return paints..something((Symbol method, List<dynamic> arguments) {
    if (method != #drawCircle) {
      return false;
    }
    final Offset center = arguments[0] as Offset;
    final double radius = arguments[1] as double;
    return offsetsAreClose(center, expectedCenter) && radiiAreClose(radius, expectedRadius);
  });
}

// Unique ripple pattern matches if there does not exist ripples
// other than ones with the [expectedCenter] and [expectedRadius].
// This ensures the nonexistence of two different ripples.
PaintPattern uniqueRipplePattern(Offset expectedCenter, double expectedRadius) {
  return paints..everything((Symbol method, List<dynamic> arguments) {
    if (method != #drawCircle) {
      return true;
    }
    final Offset center = arguments[0] as Offset;
    final double radius = arguments[1] as double;
    if (offsetsAreClose(center, expectedCenter) && radiiAreClose(radius, expectedRadius)) {
      return true;
    }
    throw '''
              Expected: center == $expectedCenter, radius == $expectedRadius
              Found: center == $center radius == $radius''';
  });
}

// Finds any container of a tooltip.
Finder findTooltipContainer(String tooltipText) {
  return find.ancestor(of: find.text(tooltipText), matching: find.byType(Container));
}

void main() {
  testWidgets('M3 Chip defaults', (WidgetTester tester) async {
    late TextTheme textTheme;
    final ThemeData lightTheme = ThemeData.light();
    final ThemeData darkTheme = ThemeData.dark();

    Widget buildFrame(ThemeData theme) {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: Builder(
              builder: (BuildContext context) {
                textTheme = Theme.of(context).textTheme;
                return Chip(
                  avatar: const CircleAvatar(child: Text('A')),
                  label: const Text('Chip A'),
                  onDeleted: () {},
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(lightTheme));
    expect(getMaterial(tester).color, null);
    expect(getMaterial(tester).elevation, 0);
    expect(
      getMaterial(tester).shape,
      RoundedRectangleBorder(
        side: BorderSide(color: lightTheme.colorScheme.outlineVariant),
        borderRadius: BorderRadius.circular(8.0),
      ),
    );
    expect(getIconData(tester).color, lightTheme.colorScheme.primary);
    expect(getIconData(tester).opacity, null);
    expect(getIconData(tester).size, 18);

    TextStyle labelStyle = getLabelStyle(tester, 'Chip A').style;
    expect(labelStyle.color, lightTheme.colorScheme.onSurfaceVariant);
    expect(labelStyle.fontFamily, textTheme.labelLarge?.fontFamily);
    expect(labelStyle.fontFamilyFallback, textTheme.labelLarge?.fontFamilyFallback);
    expect(labelStyle.fontFeatures, textTheme.labelLarge?.fontFeatures);
    expect(labelStyle.fontSize, textTheme.labelLarge?.fontSize);
    expect(labelStyle.fontStyle, textTheme.labelLarge?.fontStyle);
    expect(labelStyle.fontWeight, textTheme.labelLarge?.fontWeight);
    expect(labelStyle.height, textTheme.labelLarge?.height);
    expect(labelStyle.inherit, textTheme.labelLarge?.inherit);
    expect(labelStyle.leadingDistribution, textTheme.labelLarge?.leadingDistribution);
    expect(labelStyle.letterSpacing, textTheme.labelLarge?.letterSpacing);
    expect(labelStyle.overflow, textTheme.labelLarge?.overflow);
    expect(labelStyle.textBaseline, textTheme.labelLarge?.textBaseline);
    expect(labelStyle.wordSpacing, textTheme.labelLarge?.wordSpacing);

    await tester.pumpWidget(buildFrame(darkTheme));
    await tester.pumpAndSettle(); // Theme transition animation
    expect(getMaterial(tester).color, null);
    expect(getMaterial(tester).elevation, 0);
    expect(
      getMaterial(tester).shape,
      RoundedRectangleBorder(
        side: BorderSide(color: darkTheme.colorScheme.outlineVariant),
        borderRadius: BorderRadius.circular(8.0),
      ),
    );
    expect(getIconData(tester).color, darkTheme.colorScheme.primary);
    expect(getIconData(tester).opacity, null);
    expect(getIconData(tester).size, 18);

    labelStyle = getLabelStyle(tester, 'Chip A').style;
    expect(labelStyle.color, darkTheme.colorScheme.onSurfaceVariant);
    expect(labelStyle.fontFamily, textTheme.labelLarge?.fontFamily);
    expect(labelStyle.fontFamilyFallback, textTheme.labelLarge?.fontFamilyFallback);
    expect(labelStyle.fontFeatures, textTheme.labelLarge?.fontFeatures);
    expect(labelStyle.fontSize, textTheme.labelLarge?.fontSize);
    expect(labelStyle.fontStyle, textTheme.labelLarge?.fontStyle);
    expect(labelStyle.fontWeight, textTheme.labelLarge?.fontWeight);
    expect(labelStyle.height, textTheme.labelLarge?.height);
    expect(labelStyle.inherit, textTheme.labelLarge?.inherit);
    expect(labelStyle.leadingDistribution, textTheme.labelLarge?.leadingDistribution);
    expect(labelStyle.letterSpacing, textTheme.labelLarge?.letterSpacing);
    expect(labelStyle.overflow, textTheme.labelLarge?.overflow);
    expect(labelStyle.textBaseline, textTheme.labelLarge?.textBaseline);
    expect(labelStyle.wordSpacing, textTheme.labelLarge?.wordSpacing);
  });

  testWidgets('Chip control test', (WidgetTester tester) async {
    final FeedbackTester feedback = FeedbackTester();
    final List<String> deletedChipLabels = <String>[];
    await tester.pumpWidget(
      wrapForChip(
        child: Column(
          children: <Widget>[
            Chip(
              avatar: const CircleAvatar(child: Text('A')),
              label: const Text('Chip A'),
              onDeleted: () {
                deletedChipLabels.add('A');
              },
              deleteButtonTooltipMessage: 'Delete chip A',
            ),
            Chip(
              avatar: const CircleAvatar(child: Text('B')),
              label: const Text('Chip B'),
              onDeleted: () {
                deletedChipLabels.add('B');
              },
              deleteButtonTooltipMessage: 'Delete chip B',
            ),
          ],
        ),
      ),
    );

    expect(tester.widget(find.byTooltip('Delete chip A')), isNotNull);
    expect(tester.widget(find.byTooltip('Delete chip B')), isNotNull);

    expect(feedback.clickSoundCount, 0);

    expect(deletedChipLabels, isEmpty);
    await tester.tap(find.byTooltip('Delete chip A'));
    expect(deletedChipLabels, equals(<String>['A']));

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(feedback.clickSoundCount, 1);

    await tester.tap(find.byTooltip('Delete chip B'));
    expect(deletedChipLabels, equals(<String>['A', 'B']));

    await tester.pumpAndSettle(const Duration(seconds: 1));
    expect(feedback.clickSoundCount, 2);

    feedback.dispose();
  });

  testWidgets('Chip does not constrain size of label widget if it does not exceed '
      'the available space', (WidgetTester tester) async {
    const double labelWidth = 50.0;
    const double labelHeight = 30.0;
    final Key labelKey = UniqueKey();

    await tester.pumpWidget(
      wrapForChip(
        child: Center(
          child: SizedBox(
            width: 500.0,
            height: 500.0,
            child: Column(
              children: <Widget>[
                Chip(label: SizedBox(key: labelKey, width: labelWidth, height: labelHeight)),
              ],
            ),
          ),
        ),
      ),
    );

    final Size labelSize = tester.getSize(find.byKey(labelKey));
    expect(labelSize.width, labelWidth);
    expect(labelSize.height, labelHeight);
  });

  testWidgets('Chip constrains the size of the label widget when it exceeds the '
      'available space', (WidgetTester tester) async {
    await testConstrainedLabel(tester);
  });

  testWidgets('Chip constrains the size of the label widget when it exceeds the '
      'available space and the avatar is present', (WidgetTester tester) async {
    await testConstrainedLabel(tester, avatar: const CircleAvatar(child: Text('A')));
  });

  testWidgets('Chip constrains the size of the label widget when it exceeds the '
      'available space and the delete icon is present', (WidgetTester tester) async {
    await testConstrainedLabel(tester, onDeleted: () {});
  });

  testWidgets('Chip constrains the size of the label widget when it exceeds the '
      'available space and both avatar and delete icons are present', (WidgetTester tester) async {
    await testConstrainedLabel(
      tester,
      avatar: const CircleAvatar(child: Text('A')),
      onDeleted: () {},
    );
  });

  testWidgets('Chip constrains the avatar, label, and delete icons to the bounds of '
      'the chip when it exceeds the available space', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/11523
    Widget chipBuilder(String text, {Widget? avatar, VoidCallback? onDeleted}) {
      return MaterialApp(
        home: Scaffold(
          body: SizedBox(
            width: 150,
            child: Column(
              children: <Widget>[Chip(avatar: avatar, label: Text(text), onDeleted: onDeleted)],
            ),
          ),
        ),
      );
    }

    void chipRectContains(Rect chipRect, Rect rect) {
      expect(chipRect.contains(rect.topLeft), true);
      expect(chipRect.contains(rect.topRight), true);
      expect(chipRect.contains(rect.bottomLeft), true);
      expect(chipRect.contains(rect.bottomRight), true);
    }

    Rect chipRect;
    Rect avatarRect;
    Rect labelRect;
    Rect deleteIconRect;
    const String text = 'Very long text that will be clipped';

    await tester.pumpWidget(chipBuilder(text));

    chipRect = tester.getRect(find.byType(Chip));
    labelRect = tester.getRect(find.text(text));
    chipRectContains(chipRect, labelRect);

    await tester.pumpWidget(chipBuilder(text, avatar: const CircleAvatar(child: Text('A'))));
    await tester.pumpAndSettle();

    chipRect = tester.getRect(find.byType(Chip));
    avatarRect = tester.getRect(find.byType(CircleAvatar));
    chipRectContains(chipRect, avatarRect);

    labelRect = tester.getRect(find.text(text));
    chipRectContains(chipRect, labelRect);

    await tester.pumpWidget(
      chipBuilder(text, avatar: const CircleAvatar(child: Text('A')), onDeleted: () {}),
    );
    await tester.pumpAndSettle();

    chipRect = tester.getRect(find.byType(Chip));
    avatarRect = tester.getRect(find.byType(CircleAvatar));
    chipRectContains(chipRect, avatarRect);

    labelRect = tester.getRect(find.text(text));
    chipRectContains(chipRect, labelRect);

    deleteIconRect = tester.getRect(find.byIcon(Icons.cancel));
    chipRectContains(chipRect, deleteIconRect);
  });

  testWidgets('Material2 - Chip in row works ok', (WidgetTester tester) async {
    const TextStyle style = TextStyle(fontSize: 10.0);
    await tester.pumpWidget(
      wrapForChip(
        theme: ThemeData(useMaterial3: false),
        child: const Row(children: <Widget>[Chip(label: Text('Test'), labelStyle: style)]),
      ),
    );
    expect(tester.getSize(find.byType(Text)), const Size(40.0, 10.0));
    expect(tester.getSize(find.byType(Chip)), const Size(64.0, 48.0));
    await tester.pumpWidget(
      wrapForChip(
        child: const Row(
          children: <Widget>[Flexible(child: Chip(label: Text('Test'), labelStyle: style))],
        ),
      ),
    );
    expect(tester.getSize(find.byType(Text)), const Size(40.0, 10.0));
    expect(tester.getSize(find.byType(Chip)), const Size(64.0, 48.0));
    await tester.pumpWidget(
      wrapForChip(
        child: const Row(
          children: <Widget>[Expanded(child: Chip(label: Text('Test'), labelStyle: style))],
        ),
      ),
    );
    expect(tester.getSize(find.byType(Text)), const Size(40.0, 10.0));
    expect(tester.getSize(find.byType(Chip)), const Size(800.0, 48.0));
  });

  testWidgets('Material3 - Chip in row works ok', (WidgetTester tester) async {
    const TextStyle style = TextStyle(fontSize: 10.0);
    await tester.pumpWidget(
      wrapForChip(
        child: const Row(children: <Widget>[Chip(label: Text('Test'), labelStyle: style)]),
      ),
    );
    expect(tester.getSize(find.byType(Text)).width, closeTo(40.4, 0.01));
    expect(tester.getSize(find.byType(Text)).height, equals(14.0));
    expect(tester.getSize(find.byType(Chip)).width, closeTo(74.4, 0.01));
    expect(tester.getSize(find.byType(Chip)).height, equals(48.0));
    await tester.pumpWidget(
      wrapForChip(
        child: const Row(
          children: <Widget>[Flexible(child: Chip(label: Text('Test'), labelStyle: style))],
        ),
      ),
    );
    expect(tester.getSize(find.byType(Text)).width, closeTo(40.4, 0.01));
    expect(tester.getSize(find.byType(Text)).height, equals(14.0));
    expect(tester.getSize(find.byType(Chip)).width, closeTo(74.4, 0.01));
    expect(tester.getSize(find.byType(Chip)).height, equals(48.0));
    await tester.pumpWidget(
      wrapForChip(
        child: const Row(
          children: <Widget>[Expanded(child: Chip(label: Text('Test'), labelStyle: style))],
        ),
      ),
    );
    expect(tester.getSize(find.byType(Text)).width, closeTo(40.4, 0.01));
    expect(tester.getSize(find.byType(Text)).height, equals(14.0));
    expect(tester.getSize(find.byType(Chip)), const Size(800.0, 48.0));
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Material2 - Chip responds to materialTapTargetSize', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrapForChip(
        theme: ThemeData(useMaterial3: false),
        child: const Column(
          children: <Widget>[
            Chip(label: Text('X'), materialTapTargetSize: MaterialTapTargetSize.padded),
            Chip(label: Text('X'), materialTapTargetSize: MaterialTapTargetSize.shrinkWrap),
          ],
        ),
      ),
    );
    expect(tester.getSize(find.byType(Chip).first), const Size(48.0, 48.0));
    expect(tester.getSize(find.byType(Chip).last), const Size(38.0, 32.0));
  });

  testWidgets('Material3 - Chip responds to materialTapTargetSize', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrapForChip(
        child: const Column(
          children: <Widget>[
            Chip(label: Text('X'), materialTapTargetSize: MaterialTapTargetSize.padded),
            Chip(label: Text('X'), materialTapTargetSize: MaterialTapTargetSize.shrinkWrap),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.byType(Chip).first).width, closeTo(48.1, 0.01));
    expect(tester.getSize(find.byType(Chip).first).height, equals(48.0));
    expect(tester.getSize(find.byType(Chip).last).width, closeTo(48.1, 0.01));
    expect(tester.getSize(find.byType(Chip).last).height, equals(38.0));
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Delete button tap target is the right proportion of the chip', (
    WidgetTester tester,
  ) async {
    final UniqueKey deleteKey = UniqueKey();
    bool calledDelete = false;
    await tester.pumpWidget(
      wrapForChip(
        child: Column(
          children: <Widget>[
            Chip(
              label: const Text('Really Long Label'),
              deleteIcon: Icon(Icons.delete, key: deleteKey),
              onDeleted: () {
                calledDelete = true;
              },
            ),
          ],
        ),
      ),
    );

    // Test correct tap target size.
    await tester.tapAt(
      tester.getCenter(find.byKey(deleteKey)) - const Offset(18.0, 0.0),
    ); // Half the width of the delete button + right label padding.
    await tester.pump();
    expect(calledDelete, isTrue);
    calledDelete = false;

    // Test incorrect tap target size.
    await tester.tapAt(tester.getCenter(find.byKey(deleteKey)) - const Offset(19.0, 0.0));
    await tester.pump();
    expect(calledDelete, isFalse);
    calledDelete = false;

    await tester.pumpWidget(
      wrapForChip(
        child: Column(
          children: <Widget>[
            Chip(
              label: const SizedBox(), // Short label
              deleteIcon: Icon(Icons.cancel, key: deleteKey),
              onDeleted: () {
                calledDelete = true;
              },
            ),
          ],
        ),
      ),
    );

    // Chip width is 48 with padding, 40 without padding, so halfway is at 20. Cancel
    // icon is 24x24, so since 24 > 20 the split location should be halfway across the
    // chip, which is at 12 + 8 = 20 from the right side. Since the split is just
    // slightly less than 50%, 8 from the center of the delete button should hit the
    // chip, not the delete button.
    await tester.tapAt(tester.getCenter(find.byKey(deleteKey)) - const Offset(7.0, 0.0));
    await tester.pump();
    expect(calledDelete, isTrue);
    calledDelete = false;

    await tester.tapAt(tester.getCenter(find.byKey(deleteKey)) - const Offset(8.0, 0.0));
    await tester.pump();
    expect(calledDelete, isFalse);
  });

  testWidgets('Chip elements are ordered horizontally for locale', (WidgetTester tester) async {
    final UniqueKey iconKey = UniqueKey();
    late final OverlayEntry entry;
    addTearDown(
      () =>
          entry
            ..remove()
            ..dispose(),
    );
    final Widget test = Overlay(
      initialEntries: <OverlayEntry>[
        entry = OverlayEntry(
          builder: (BuildContext context) {
            return Material(
              child: Chip(
                deleteIcon: Icon(Icons.delete, key: iconKey),
                onDeleted: () {},
                label: const Text('ABC'),
              ),
            );
          },
        ),
      ],
    );

    await tester.pumpWidget(wrapForChip(child: test, textDirection: TextDirection.rtl));
    await tester.pumpAndSettle(const Duration(milliseconds: 500));
    expect(
      tester.getCenter(find.text('ABC')).dx,
      greaterThan(tester.getCenter(find.byKey(iconKey)).dx),
    );
    await tester.pumpWidget(wrapForChip(child: test));
    await tester.pumpAndSettle(const Duration(milliseconds: 500));
    expect(
      tester.getCenter(find.text('ABC')).dx,
      lessThan(tester.getCenter(find.byKey(iconKey)).dx),
    );
  });

  testWidgets('Material2 - Chip responds to textScaleFactor', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrapForChip(
        theme: ThemeData(useMaterial3: false),
        child: const Column(
          children: <Widget>[
            Chip(avatar: CircleAvatar(child: Text('A')), label: Text('Chip A')),
            Chip(avatar: CircleAvatar(child: Text('B')), label: Text('Chip B')),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.text('Chip A')), const Size(84.0, 14.0));
    expect(tester.getSize(find.text('Chip B')), const Size(84.0, 14.0));
    expect(tester.getSize(find.byType(Chip).first), const Size(132.0, 48.0));
    expect(tester.getSize(find.byType(Chip).last), const Size(132.0, 48.0));

    await tester.pumpWidget(
      wrapForChip(
        textScaler: const TextScaler.linear(3.0),
        child: const Column(
          children: <Widget>[
            Chip(avatar: CircleAvatar(child: Text('A')), label: Text('Chip A')),
            Chip(avatar: CircleAvatar(child: Text('B')), label: Text('Chip B')),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.text('Chip A')), const Size(252.0, 42.0));
    expect(tester.getSize(find.text('Chip B')), const Size(252.0, 42.0));
    expect(tester.getSize(find.byType(Chip).first), const Size(310.0, 50.0));
    expect(tester.getSize(find.byType(Chip).last), const Size(310.0, 50.0));

    // Check that individual text scales are taken into account.
    await tester.pumpWidget(
      wrapForChip(
        child: const Column(
          children: <Widget>[
            Chip(
              avatar: CircleAvatar(child: Text('A')),
              label: Text('Chip A', textScaleFactor: 3.0),
            ),
            Chip(avatar: CircleAvatar(child: Text('B')), label: Text('Chip B')),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.text('Chip A')), const Size(252.0, 42.0));
    expect(tester.getSize(find.text('Chip B')), const Size(84.0, 14.0));
    expect(tester.getSize(find.byType(Chip).first), const Size(318.0, 50.0));
    expect(tester.getSize(find.byType(Chip).last), const Size(132.0, 48.0));
  });

  testWidgets('Material3 - Chip responds to textScaleFactor', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrapForChip(
        child: const Column(
          children: <Widget>[
            Chip(avatar: CircleAvatar(child: Text('A')), label: Text('Chip A')),
            Chip(avatar: CircleAvatar(child: Text('B')), label: Text('Chip B')),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.text('Chip A')).width, closeTo(84.5, 0.1));
    expect(tester.getSize(find.text('Chip A')).height, equals(20.0));
    expect(tester.getSize(find.text('Chip B')).width, closeTo(84.5, 0.1));
    expect(tester.getSize(find.text('Chip B')).height, equals(20.0));

    await tester.pumpWidget(
      wrapForChip(
        textScaler: const TextScaler.linear(3.0),
        child: const Column(
          children: <Widget>[
            Chip(avatar: CircleAvatar(child: Text('A')), label: Text('Chip A')),
            Chip(avatar: CircleAvatar(child: Text('B')), label: Text('Chip B')),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.text('Chip A')).width, closeTo(252.6, 0.1));
    expect(tester.getSize(find.text('Chip A')).height, equals(60.0));
    expect(tester.getSize(find.text('Chip B')).width, closeTo(252.6, 0.1));
    expect(tester.getSize(find.text('Chip B')).height, equals(60.0));
    expect(tester.getSize(find.byType(Chip).first).width, closeTo(338.6, 0.1));
    expect(tester.getSize(find.byType(Chip).first).height, equals(78.0));
    expect(tester.getSize(find.byType(Chip).last).width, closeTo(338.6, 0.1));
    expect(tester.getSize(find.byType(Chip).last).height, equals(78.0));

    // Check that individual text scales are taken into account.
    await tester.pumpWidget(
      wrapForChip(
        child: const Column(
          children: <Widget>[
            Chip(
              avatar: CircleAvatar(child: Text('A')),
              label: Text('Chip A', textScaleFactor: 3.0),
            ),
            Chip(avatar: CircleAvatar(child: Text('B')), label: Text('Chip B')),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.text('Chip A')).width, closeTo(252.6, 0.01));
    expect(tester.getSize(find.text('Chip A')).height, equals(60.0));
    expect(tester.getSize(find.text('Chip B')).width, closeTo(84.59, 0.01));
    expect(tester.getSize(find.text('Chip B')).height, equals(20.0));
    expect(tester.getSize(find.byType(Chip).first).width, closeTo(346.6, 0.01));
    expect(tester.getSize(find.byType(Chip).first).height, equals(78.0));
    expect(tester.getSize(find.byType(Chip).last).width, closeTo(138.59, 0.01));
    expect(tester.getSize(find.byType(Chip).last).height, equals(48.0));
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Material2 - Labels can be non-text widgets', (WidgetTester tester) async {
    final Key keyA = GlobalKey();
    final Key keyB = GlobalKey();
    await tester.pumpWidget(
      wrapForChip(
        theme: ThemeData(useMaterial3: false),
        child: Column(
          children: <Widget>[
            Chip(avatar: const CircleAvatar(child: Text('A')), label: Text('Chip A', key: keyA)),
            Chip(
              avatar: const CircleAvatar(child: Text('B')),
              label: SizedBox(key: keyB, width: 10.0, height: 10.0),
            ),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.byKey(keyA)), const Size(84.0, 14.0));
    expect(tester.getSize(find.byKey(keyB)), const Size(10.0, 10.0));
    expect(tester.getSize(find.byType(Chip).first), const Size(132.0, 48.0));
    expect(tester.getSize(find.byType(Chip).last), const Size(58.0, 48.0));
  });

  testWidgets('Material3 - Labels can be non-text widgets', (WidgetTester tester) async {
    final Key keyA = GlobalKey();
    final Key keyB = GlobalKey();
    await tester.pumpWidget(
      wrapForChip(
        child: Column(
          children: <Widget>[
            Chip(avatar: const CircleAvatar(child: Text('A')), label: Text('Chip A', key: keyA)),
            Chip(
              avatar: const CircleAvatar(child: Text('B')),
              label: SizedBox(key: keyB, width: 10.0, height: 10.0),
            ),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.byKey(keyA)).width, moreOrLessEquals(84.5, epsilon: 0.1));
    expect(tester.getSize(find.byKey(keyA)).height, equals(20.0));
    expect(tester.getSize(find.byKey(keyB)), const Size(10.0, 10.0));
    expect(tester.getSize(find.byType(Chip).first).width, moreOrLessEquals(138.5, epsilon: 0.1));
    expect(tester.getSize(find.byType(Chip).first).height, equals(48.0));
    expect(tester.getSize(find.byType(Chip).last), const Size(60.0, 48.0));
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Avatars can be non-circle avatar widgets', (WidgetTester tester) async {
    final Key keyA = GlobalKey();
    await tester.pumpWidget(
      wrapForChip(
        child: Column(
          children: <Widget>[
            Chip(
              avatar: SizedBox(key: keyA, width: 20.0, height: 20.0),
              label: const Text('Chip A'),
            ),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.byKey(keyA)), equals(const Size(20.0, 20.0)));
  });

  testWidgets('Delete icons can be non-icon widgets', (WidgetTester tester) async {
    final Key keyA = GlobalKey();
    await tester.pumpWidget(
      wrapForChip(
        child: Column(
          children: <Widget>[
            Chip(
              deleteIcon: SizedBox(key: keyA, width: 20.0, height: 20.0),
              label: const Text('Chip A'),
              onDeleted: () {},
            ),
          ],
        ),
      ),
    );

    expect(tester.getSize(find.byKey(keyA)), equals(const Size(20.0, 20.0)));
  });

  testWidgets('Chip padding - LTR', (WidgetTester tester) async {
    final GlobalKey keyA = GlobalKey();
    final GlobalKey keyB = GlobalKey();

    late final OverlayEntry entry;
    addTearDown(
      () =>
          entry
            ..remove()
            ..dispose(),
    );
    await tester.pumpWidget(
      wrapForChip(
        child: Overlay(
          initialEntries: <OverlayEntry>[
            entry = OverlayEntry(
              builder: (BuildContext context) {
                return Material(
                  child: Center(
                    child: Chip(
                      avatar: Placeholder(key: keyA),
                      label: SizedBox(key: keyB, width: 40.0, height: 40.0),
                      onDeleted: () {},
                    ),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
    expect(tester.getTopLeft(find.byKey(keyA)), const Offset(332.0, 280.0));
    expect(tester.getBottomRight(find.byKey(keyA)), const Offset(372.0, 320.0));
    expect(tester.getTopLeft(find.byKey(keyB)), const Offset(380.0, 280.0));
    expect(tester.getBottomRight(find.byKey(keyB)), const Offset(420.0, 320.0));
    expect(tester.getTopLeft(find.byType(Icon)), const Offset(439.0, 291.0));
    expect(tester.getBottomRight(find.byType(Icon)), const Offset(457.0, 309.0));
  });

  testWidgets('Chip padding - RTL', (WidgetTester tester) async {
    final GlobalKey keyA = GlobalKey();
    final GlobalKey keyB = GlobalKey();

    late final OverlayEntry entry;
    addTearDown(
      () =>
          entry
            ..remove()
            ..dispose(),
    );

    await tester.pumpWidget(
      wrapForChip(
        textDirection: TextDirection.rtl,
        child: Overlay(
          initialEntries: <OverlayEntry>[
            entry = OverlayEntry(
              builder: (BuildContext context) {
                return Material(
                  child: Center(
                    child: Chip(
                      avatar: Placeholder(key: keyA),
                      label: SizedBox(key: keyB, width: 40.0, height: 40.0),
                      onDeleted: () {},
                    ),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );

    expect(tester.getTopLeft(find.byKey(keyA)), const Offset(428.0, 280.0));
    expect(tester.getBottomRight(find.byKey(keyA)), const Offset(468.0, 320.0));
    expect(tester.getTopLeft(find.byKey(keyB)), const Offset(380.0, 280.0));
    expect(tester.getBottomRight(find.byKey(keyB)), const Offset(420.0, 320.0));
    expect(tester.getTopLeft(find.byType(Icon)), const Offset(343.0, 291.0));
    expect(tester.getBottomRight(find.byType(Icon)), const Offset(361.0, 309.0));
  });

  testWidgets('Material2 - Avatar drawer works as expected on RawChip', (
    WidgetTester tester,
  ) async {
    final GlobalKey labelKey = GlobalKey();
    Future<void> pushChip({Widget? avatar}) async {
      return tester.pumpWidget(
        wrapForChip(
          theme: ThemeData(useMaterial3: false),
          child: Wrap(
            children: <Widget>[
              RawChip(
                avatar: avatar,
                label: Text('Chip', key: labelKey),
                shape: const StadiumBorder(),
              ),
            ],
          ),
        ),
      );
    }

    // No avatar
    await pushChip();
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(80.0, 48.0)));
    final GlobalKey avatarKey = GlobalKey();

    // Add an avatar
    await pushChip(
      avatar: Container(key: avatarKey, color: const Color(0xff000000), width: 40.0, height: 40.0),
    );
    // Avatar drawer should start out closed.
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(80.0, 48.0)));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)), equals(const Offset(-20.0, 12.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(12.0, 17.0)));

    await tester.pump(const Duration(milliseconds: 20));
    // Avatar drawer should start expanding.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(81.2, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-18.8, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(13.2, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(86.7, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-13.3, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(18.6, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(94.7, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-5.3, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(26.7, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(99.5, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-0.5, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(31.5, epsilon: 0.1));

    // Wait for being done with animation, and make sure it didn't change
    // height.
    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(104.0, 48.0)));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)), equals(const Offset(4.0, 12.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(36.0, 17.0)));

    // Remove the avatar again
    await pushChip();
    // Avatar drawer should start out open.
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(104.0, 48.0)));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)), equals(const Offset(4.0, 12.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(36.0, 17.0)));

    await tester.pump(const Duration(milliseconds: 20));
    // Avatar drawer should start contracting.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(102.9, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(2.9, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(34.9, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(98.0, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-2.0, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(30.0, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(84.1, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-15.9, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(16.1, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(80.0, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-20.0, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(12.0, epsilon: 0.1));

    // Wait for being done with animation, make sure it didn't change
    // height, and make sure that the avatar is no longer drawn.
    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(80.0, 48.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(12.0, 17.0)));
    expect(find.byKey(avatarKey), findsNothing);
  });

  testWidgets('Material3 - Avatar drawer works as expected on RawChip', (
    WidgetTester tester,
  ) async {
    final GlobalKey labelKey = GlobalKey();
    Future<void> pushChip({Widget? avatar}) async {
      return tester.pumpWidget(
        wrapForChip(
          child: Wrap(
            children: <Widget>[
              RawChip(
                avatar: avatar,
                label: Text('Chip', key: labelKey),
                shape: const StadiumBorder(),
              ),
            ],
          ),
        ),
      );
    }

    // No avatar
    await pushChip();
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(90.4, epsilon: 0.1));
    final GlobalKey avatarKey = GlobalKey();

    // Add an avatar
    await pushChip(
      avatar: Container(key: avatarKey, color: const Color(0xff000000), width: 40.0, height: 40.0),
    );
    // Avatar drawer should start out closed.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(90.4, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)), equals(const Offset(-11.0, 14.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(17.0, 14.0)));

    await tester.pump(const Duration(milliseconds: 20));
    // Avatar drawer should start expanding.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(91.3, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-10, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(17.9, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(95.9, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-5.4, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(22.5, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(102.6, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(1.2, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(29.2, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(106.6, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(5.2, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(33.2, epsilon: 0.1));

    // Wait for being done with animation, and make sure it didn't change
    // height.
    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(110.4, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)), equals(const Offset(9.0, 14.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(37.0, 14.0)));

    // Remove the avatar again
    await pushChip();
    // Avatar drawer should start out open.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(110.4, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)), equals(const Offset(9.0, 14.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(37.0, 14.0)));

    await tester.pump(const Duration(milliseconds: 20));
    // Avatar drawer should start contracting.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(109.5, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(8.1, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(36.1, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(105.4, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(4.0, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(32.0, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(93.7, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-7.6, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(20.3, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(90.4, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(avatarKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(avatarKey)).dx, moreOrLessEquals(-11.0, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)).dx, moreOrLessEquals(17.0, epsilon: 0.1));

    // Wait for being done with animation, make sure it didn't change
    // height, and make sure that the avatar is no longer drawn.
    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(90.4, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(17.0, 14.0)));
    expect(find.byKey(avatarKey), findsNothing);
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Material2 - Delete button drawer works as expected on RawChip', (
    WidgetTester tester,
  ) async {
    const Key labelKey = Key('label');
    const Key deleteButtonKey = Key('delete');
    bool wasDeleted = false;
    Future<void> pushChip({bool deletable = false}) async {
      return tester.pumpWidget(
        wrapForChip(
          theme: ThemeData(useMaterial3: false),
          child: Wrap(
            children: <Widget>[
              StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return RawChip(
                    onDeleted:
                        deletable
                            ? () {
                              setState(() {
                                wasDeleted = true;
                              });
                            }
                            : null,
                    deleteIcon: Container(
                      width: 40.0,
                      height: 40.0,
                      color: Colors.blue,
                      key: deleteButtonKey,
                    ),
                    label: const Text('Chip', key: labelKey),
                    shape: const StadiumBorder(),
                  );
                },
              ),
            ],
          ),
        ),
      );
    }

    // No delete button
    await pushChip();
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(80.0, 48.0)));

    // Add a delete button
    await pushChip(deletable: true);
    // Delete button drawer should start out closed.
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(80.0, 48.0)));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)), equals(const Offset(52.0, 12.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(12.0, 17.0)));

    await tester.pump(const Duration(milliseconds: 20));
    // Delete button drawer should start expanding.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(81.2, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(53.2, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(12.0, 17.0)));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(86.7, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(58.7, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(94.7, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(66.7, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(99.5, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(71.5, epsilon: 0.1));

    // Wait for being done with animation, and make sure it didn't change
    // height.
    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(104.0, 48.0)));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)), equals(const Offset(76.0, 12.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(12.0, 17.0)));

    // Test the tap work for the delete button, but not the rest of the chip.
    expect(wasDeleted, isFalse);
    await tester.tap(find.byKey(labelKey));
    expect(wasDeleted, isFalse);
    await tester.tap(find.byKey(deleteButtonKey));
    expect(wasDeleted, isTrue);

    // Remove the delete button again
    await pushChip();
    // Delete button drawer should start out open.
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(104.0, 48.0)));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)), equals(const Offset(76.0, 12.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(12.0, 17.0)));

    await tester.pump(const Duration(milliseconds: 20));
    // Delete button drawer should start contracting.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(103.8, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(75.8, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(12.0, 17.0)));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(102.9, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(74.9, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(101.0, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(73.0, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(97.5, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(24.0, 24.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(69.5, epsilon: 0.1));

    // Wait for being done with animation, make sure it didn't change
    // height, and make sure that the delete button is no longer drawn.
    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(80.0, 48.0)));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(12.0, 17.0)));
    expect(find.byKey(deleteButtonKey), findsNothing);
  });

  testWidgets('Material3 - Delete button drawer works as expected on RawChip', (
    WidgetTester tester,
  ) async {
    const Key labelKey = Key('label');
    const Key deleteButtonKey = Key('delete');
    bool wasDeleted = false;
    Future<void> pushChip({bool deletable = false}) async {
      return tester.pumpWidget(
        wrapForChip(
          child: Wrap(
            children: <Widget>[
              StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return RawChip(
                    onDeleted:
                        deletable
                            ? () {
                              setState(() {
                                wasDeleted = true;
                              });
                            }
                            : null,
                    deleteIcon: Container(
                      width: 40.0,
                      height: 40.0,
                      color: Colors.blue,
                      key: deleteButtonKey,
                    ),
                    label: const Text('Chip', key: labelKey),
                    shape: const StadiumBorder(),
                  );
                },
              ),
            ],
          ),
        ),
      );
    }

    // No delete button
    await pushChip();
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(90.4, epsilon: 0.01));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));

    // Add a delete button
    await pushChip(deletable: true);
    // Delete button drawer should start out closed.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(90.4, epsilon: 0.01));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(
      tester.getTopLeft(find.byKey(deleteButtonKey)),
      offsetMoreOrLessEquals(const Offset(61.4, 14.0), epsilon: 0.01),
    );
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(17.0, 14.0)));

    await tester.pump(const Duration(milliseconds: 20));
    // Delete button drawer should start expanding.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(91.3, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(62.3, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(17.0, 14.0)));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(95.9, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(66.9, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(102.6, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(73.6, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(106.6, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(77.6, epsilon: 0.1));

    // Wait for being done with animation, and make sure it didn't change
    // height.
    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(110.4, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(
      tester.getTopLeft(find.byKey(deleteButtonKey)),
      offsetMoreOrLessEquals(const Offset(81.4, 14.0), epsilon: 0.01),
    );
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(17.0, 14.0)));

    // Test the tap work for the delete button, but not the rest of the chip.
    expect(wasDeleted, isFalse);
    await tester.tap(find.byKey(labelKey));
    expect(wasDeleted, isFalse);
    await tester.tap(find.byKey(deleteButtonKey));
    expect(wasDeleted, isTrue);

    // Remove the delete button again
    await pushChip();
    // Delete button drawer should start out open.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(110.4, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(
      tester.getTopLeft(find.byKey(deleteButtonKey)),
      offsetMoreOrLessEquals(const Offset(81.4, 14.0), epsilon: 0.01),
    );
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(17.0, 14.0)));

    await tester.pump(const Duration(milliseconds: 20));
    // Delete button drawer should start contracting.
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(110.1, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(81.1, epsilon: 0.1));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(17.0, 14.0)));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(109.4, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(80.4, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(107.9, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(78.9, epsilon: 0.1));

    await tester.pump(const Duration(milliseconds: 20));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(104.9, epsilon: 0.1));
    expect(tester.getSize(find.byKey(deleteButtonKey)), equals(const Size(20.0, 20.0)));
    expect(tester.getTopLeft(find.byKey(deleteButtonKey)).dx, moreOrLessEquals(75.9, epsilon: 0.1));

    // Wait for being done with animation, make sure it didn't change
    // height, and make sure that the delete button is no longer drawn.
    await tester.pumpAndSettle(const Duration(milliseconds: 200));
    expect(tester.getSize(find.byType(RawChip)).width, moreOrLessEquals(90.4, epsilon: 0.1));
    expect(tester.getSize(find.byType(RawChip)).height, equals(48.0));
    expect(tester.getTopLeft(find.byKey(labelKey)), equals(const Offset(17.0, 14.0)));
    expect(find.byKey(deleteButtonKey), findsNothing);
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Delete button takes up at most half of the chip', (WidgetTester tester) async {
    final UniqueKey chipKey = UniqueKey();
    bool chipPressed = false;
    bool deletePressed = false;

    await tester.pumpWidget(
      wrapForChip(
        child: Wrap(
          children: <Widget>[
            RawChip(
              key: chipKey,
              onPressed: () {
                chipPressed = true;
              },
              onDeleted: () {
                deletePressed = true;
              },
              label: const Text(''),
            ),
          ],
        ),
      ),
    );

    await tester.tapAt(tester.getCenter(find.byKey(chipKey)));
    await tester.pump();
    expect(chipPressed, isTrue);
    expect(deletePressed, isFalse);
    chipPressed = false;

    await tester.tapAt(tester.getCenter(find.byKey(chipKey)) + const Offset(1.0, 0.0));
    await tester.pump();
    expect(chipPressed, isFalse);
    expect(deletePressed, isTrue);
  });

  testWidgets('Material2 - Chip creates centered, unique ripple when label is tapped', (
    WidgetTester tester,
  ) async {
    final UniqueKey labelKey = UniqueKey();
    final UniqueKey deleteButtonKey = UniqueKey();

    await tester.pumpWidget(
      chipWithOptionalDeleteButton(
        themeData: ThemeData(useMaterial3: false),
        labelKey: labelKey,
        deleteButtonKey: deleteButtonKey,
        deletable: true,
      ),
    );

    final RenderBox box = getMaterialBox(tester);

    // Taps at a location close to the center of the label.
    final Offset centerOfLabel = tester.getCenter(find.byKey(labelKey));
    final Offset tapLocationOfLabel = centerOfLabel + const Offset(-10, -10);
    final TestGesture gesture = await tester.startGesture(tapLocationOfLabel);
    await tester.pump();

    // Waits for 100 ms.
    await tester.pump(const Duration(milliseconds: 100));

    // There should be one unique, centered ink ripple.
    expect(box, ripplePattern(const Offset(163.0, 6.0), 20.9));
    expect(box, uniqueRipplePattern(const Offset(163.0, 6.0), 20.9));

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for 100 ms again.
    await tester.pump(const Duration(milliseconds: 100));

    // The ripple should grow, with the same center.
    expect(box, ripplePattern(const Offset(163.0, 6.0), 41.8));
    expect(box, uniqueRipplePattern(const Offset(163.0, 6.0), 41.8));

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for a very long time.
    await tester.pumpAndSettle();

    // There should still be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    await gesture.up();
  });

  testWidgets('Material3 - Chip creates centered, unique sparkle when label is tapped', (
    WidgetTester tester,
  ) async {
    final UniqueKey labelKey = UniqueKey();
    final UniqueKey deleteButtonKey = UniqueKey();

    await tester.pumpWidget(
      chipWithOptionalDeleteButton(
        labelKey: labelKey,
        deleteButtonKey: deleteButtonKey,
        deletable: true,
      ),
    );

    // Taps at a location close to the center of the label.
    final Offset centerOfLabel = tester.getCenter(find.byKey(labelKey));
    final Offset tapLocationOfLabel = centerOfLabel + const Offset(-10, -10);
    final TestGesture gesture = await tester.startGesture(tapLocationOfLabel);
    await tester.pump();

    // Waits for 100 ms.
    await tester.pump(const Duration(milliseconds: 100));

    // There should be one unique, centered ink sparkle.
    await expectLater(
      find.byType(RawChip),
      matchesGoldenFile('chip.label_tapped.ink_sparkle.0.png'),
    );

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for 100 ms again.
    await tester.pump(const Duration(milliseconds: 100));

    // The sparkle should grow, with the same center.
    await expectLater(
      find.byType(RawChip),
      matchesGoldenFile('chip.label_tapped.ink_sparkle.1.png'),
    );

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for a very long time.
    await tester.pumpAndSettle();

    // There should still be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    await gesture.up();
  });

  testWidgets('Delete button is focusable', (WidgetTester tester) async {
    final GlobalKey labelKey = GlobalKey();
    final GlobalKey deleteButtonKey = GlobalKey();

    await tester.pumpWidget(
      chipWithOptionalDeleteButton(
        labelKey: labelKey,
        deleteButtonKey: deleteButtonKey,
        deletable: true,
      ),
    );

    Focus.of(deleteButtonKey.currentContext!).requestFocus();
    await tester.pump();

    // They shouldn't have the same focus node.
    expect(
      Focus.of(deleteButtonKey.currentContext!),
      isNot(equals(Focus.of(labelKey.currentContext!))),
    );
    expect(Focus.of(deleteButtonKey.currentContext!).hasFocus, isTrue);
    expect(Focus.of(deleteButtonKey.currentContext!).hasPrimaryFocus, isTrue);
    // Delete button is a child widget of the Chip, so the Chip should have focus if
    // the delete button does.
    expect(Focus.of(labelKey.currentContext!).hasFocus, isTrue);
    expect(Focus.of(labelKey.currentContext!).hasPrimaryFocus, isFalse);

    Focus.of(labelKey.currentContext!).requestFocus();
    await tester.pump();

    expect(Focus.of(deleteButtonKey.currentContext!).hasFocus, isFalse);
    expect(Focus.of(deleteButtonKey.currentContext!).hasPrimaryFocus, isFalse);
    expect(Focus.of(labelKey.currentContext!).hasFocus, isTrue);
    expect(Focus.of(labelKey.currentContext!).hasPrimaryFocus, isTrue);
  });

  testWidgets('Material2 - Delete button creates centered, unique ripple when tapped', (
    WidgetTester tester,
  ) async {
    final UniqueKey labelKey = UniqueKey();
    final UniqueKey deleteButtonKey = UniqueKey();

    await tester.pumpWidget(
      chipWithOptionalDeleteButton(
        themeData: ThemeData(useMaterial3: false),
        labelKey: labelKey,
        deleteButtonKey: deleteButtonKey,
        deletable: true,
      ),
    );

    final RenderBox box = getMaterialBox(tester);

    // Taps at a location close to the center of the delete icon.
    final Offset centerOfDeleteButton = tester.getCenter(find.byKey(deleteButtonKey));
    final Offset tapLocationOfDeleteButton = centerOfDeleteButton + const Offset(-10, -10);
    final TestGesture gesture = await tester.startGesture(tapLocationOfDeleteButton);
    await tester.pump();

    // Waits for 200 ms.
    await tester.pump(const Duration(milliseconds: 100));
    await tester.pump(const Duration(milliseconds: 100));

    // There should be one unique ink ripple.
    expect(box, ripplePattern(Offset.zero, 1.44));
    expect(box, uniqueRipplePattern(Offset.zero, 1.44));

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for 200 ms again.
    await tester.pump(const Duration(milliseconds: 100));
    await tester.pump(const Duration(milliseconds: 100));

    // The ripple should grow, but the center should move,
    // Towards the center of the delete icon.
    expect(box, ripplePattern(const Offset(2.0, 2.0), 4.32));
    expect(box, uniqueRipplePattern(const Offset(2.0, 2.0), 4.32));

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for a very long time.
    // This is pressing and holding the delete button.
    await tester.pumpAndSettle();

    // There should be a tooltip.
    expect(findTooltipContainer('Delete'), findsOneWidget);

    await gesture.up();
  });

  testWidgets('Material3 - Delete button creates non-centered, unique sparkle when tapped', (
    WidgetTester tester,
  ) async {
    final UniqueKey labelKey = UniqueKey();
    final UniqueKey deleteButtonKey = UniqueKey();

    await tester.pumpWidget(
      chipWithOptionalDeleteButton(
        labelKey: labelKey,
        deleteButtonKey: deleteButtonKey,
        deletable: true,
        size: 18.0,
      ),
    );

    // Taps at a location close to the center of the delete icon.
    final Offset centerOfDeleteButton = tester.getCenter(find.byKey(deleteButtonKey));
    final Offset tapLocationOfDeleteButton = centerOfDeleteButton + const Offset(-10, -10);
    final TestGesture gesture = await tester.startGesture(tapLocationOfDeleteButton);
    await tester.pump();

    // Waits for 200 ms.
    await tester.pump(const Duration(milliseconds: 100));
    await tester.pump(const Duration(milliseconds: 100));

    // There should be one unique ink sparkle.
    await expectLater(
      find.byType(RawChip),
      matchesGoldenFile('chip.delete_button_tapped.ink_sparkle.0.png'),
    );

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for 200 ms again.
    await tester.pump(const Duration(milliseconds: 100));
    await tester.pump(const Duration(milliseconds: 100));

    // The sparkle should grow, but the center should move,
    // towards the center of the delete icon.
    await expectLater(
      find.byType(RawChip),
      matchesGoldenFile('chip.delete_button_tapped.ink_sparkle.1.png'),
    );

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for a very long time.
    // This is pressing and holding the delete button.
    await tester.pumpAndSettle();

    // There should be a tooltip.
    expect(findTooltipContainer('Delete'), findsOneWidget);

    await gesture.up();
  });

  testWidgets(
    'Material2 - Delete button in a chip with null onPressed creates ripple when tapped',
    (WidgetTester tester) async {
      final UniqueKey labelKey = UniqueKey();
      final UniqueKey deleteButtonKey = UniqueKey();

      await tester.pumpWidget(
        chipWithOptionalDeleteButton(
          themeData: ThemeData(useMaterial3: false),
          labelKey: labelKey,
          onPressed: null,
          deleteButtonKey: deleteButtonKey,
          deletable: true,
        ),
      );

      final RenderBox box = getMaterialBox(tester);

      // Taps at a location close to the center of the delete icon.
      final Offset centerOfDeleteButton = tester.getCenter(find.byKey(deleteButtonKey));
      final Offset tapLocationOfDeleteButton = centerOfDeleteButton + const Offset(-10, -10);
      final TestGesture gesture = await tester.startGesture(tapLocationOfDeleteButton);
      await tester.pump();

      // Waits for 200 ms.
      await tester.pump(const Duration(milliseconds: 100));
      await tester.pump(const Duration(milliseconds: 100));

      // There should be one unique ink ripple.
      expect(box, ripplePattern(Offset.zero, 1.44));
      expect(box, uniqueRipplePattern(Offset.zero, 1.44));

      // There should be no tooltip.
      expect(findTooltipContainer('Delete'), findsNothing);

      // Waits for 200 ms again.
      await tester.pump(const Duration(milliseconds: 100));
      await tester.pump(const Duration(milliseconds: 100));

      // The ripple should grow, but the center should move,
      // Towards the center of the delete icon.
      expect(box, ripplePattern(const Offset(2.0, 2.0), 4.32));
      expect(box, uniqueRipplePattern(const Offset(2.0, 2.0), 4.32));

      // There should be no tooltip.
      expect(findTooltipContainer('Delete'), findsNothing);

      // Waits for a very long time.
      // This is pressing and holding the delete button.
      await tester.pumpAndSettle();

      // There should be a tooltip.
      expect(findTooltipContainer('Delete'), findsOneWidget);

      await gesture.up();
    },
  );

  testWidgets(
    'Material3 - Delete button in a chip with null onPressed creates sparkle when tapped',
    (WidgetTester tester) async {
      final UniqueKey labelKey = UniqueKey();
      final UniqueKey deleteButtonKey = UniqueKey();

      await tester.pumpWidget(
        chipWithOptionalDeleteButton(
          labelKey: labelKey,
          onPressed: null,
          deleteButtonKey: deleteButtonKey,
          deletable: true,
          size: 18.0,
        ),
      );

      // Taps at a location close to the center of the delete icon.
      final Offset centerOfDeleteButton = tester.getCenter(find.byKey(deleteButtonKey));
      final Offset tapLocationOfDeleteButton = centerOfDeleteButton + const Offset(-10, -10);
      final TestGesture gesture = await tester.startGesture(tapLocationOfDeleteButton);
      await tester.pump();

      // Waits for 200 ms.
      await tester.pump(const Duration(milliseconds: 100));
      await tester.pump(const Duration(milliseconds: 100));

      // There should be one unique ink sparkle.
      await expectLater(
        find.byType(RawChip),
        matchesGoldenFile('chip.delete_button_tapped.disabled.ink_sparkle.0.png'),
      );

      // There should be no tooltip.
      expect(findTooltipContainer('Delete'), findsNothing);

      // Waits for 200 ms again.
      await tester.pump(const Duration(milliseconds: 100));
      await tester.pump(const Duration(milliseconds: 100));

      // The sparkle should grow, but the center should move,
      // towards the center of the delete icon.
      await expectLater(
        find.byType(RawChip),
        matchesGoldenFile('chip.delete_button_tapped.disabled.ink_sparkle.1.png'),
      );

      // There should be no tooltip.
      expect(findTooltipContainer('Delete'), findsNothing);

      // Waits for a very long time.
      // This is pressing and holding the delete button.
      await tester.pumpAndSettle();

      // There should be a tooltip.
      expect(findTooltipContainer('Delete'), findsOneWidget);

      await gesture.up();
    },
  );

  testWidgets('RTL delete button responds to tap on the left of the chip', (
    WidgetTester tester,
  ) async {
    // Creates an RTL chip with a delete button.
    final UniqueKey labelKey = UniqueKey();
    final UniqueKey deleteButtonKey = UniqueKey();

    await tester.pumpWidget(
      chipWithOptionalDeleteButton(
        labelKey: labelKey,
        deleteButtonKey: deleteButtonKey,
        deletable: true,
        textDirection: TextDirection.rtl,
      ),
    );

    // Taps at a location close to the center of the delete icon,
    // Which is on the left side of the chip.
    final Offset topLeftOfInkWell = tester.getTopLeft(find.byType(InkWell).first);
    final Offset tapLocation = topLeftOfInkWell + const Offset(8, 8);
    final TestGesture gesture = await tester.startGesture(tapLocation);
    await tester.pump();

    await tester.pumpAndSettle();

    // The existence of a 'Delete' tooltip indicates the delete icon is tapped,
    // Instead of the label.
    expect(findTooltipContainer('Delete'), findsOneWidget);

    await gesture.up();
  });

  testWidgets('Material2 - Chip without delete button creates correct ripple', (
    WidgetTester tester,
  ) async {
    // Creates a chip with a delete button.
    final UniqueKey labelKey = UniqueKey();

    await tester.pumpWidget(
      chipWithOptionalDeleteButton(
        themeData: ThemeData(useMaterial3: false),
        labelKey: labelKey,
        deletable: false,
      ),
    );

    final RenderBox box = getMaterialBox(tester);

    // Taps at a location close to the bottom-right corner of the chip.
    final Offset bottomRightOfInkWell = tester.getBottomRight(find.byType(InkWell));
    final Offset tapLocation = bottomRightOfInkWell + const Offset(-10, -10);
    final TestGesture gesture = await tester.startGesture(tapLocation);
    await tester.pump();

    // Waits for 100 ms.
    await tester.pump(const Duration(milliseconds: 100));

    // There should be exactly one ink-creating widget.
    expect(find.byType(InkWell), findsOneWidget);
    expect(find.byType(InkResponse), findsNothing);

    // There should be one unique, centered ink ripple.
    expect(box, ripplePattern(const Offset(378.0, 22.0), 37.9));
    expect(box, uniqueRipplePattern(const Offset(378.0, 22.0), 37.9));

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for 100 ms again.
    await tester.pump(const Duration(milliseconds: 100));

    // The ripple should grow, with the same center.
    // This indicates that the tap is not on a delete icon.
    expect(box, ripplePattern(const Offset(378.0, 22.0), 75.8));
    expect(box, uniqueRipplePattern(const Offset(378.0, 22.0), 75.8));

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for a very long time.
    await tester.pumpAndSettle();

    // There should still be no tooltip.
    // This indicates that the tap is not on a delete icon.
    expect(findTooltipContainer('Delete'), findsNothing);

    await gesture.up();
  });

  testWidgets('Material3 - Chip without delete button creates correct sparkle', (
    WidgetTester tester,
  ) async {
    // Creates a chip with a delete button.
    final UniqueKey labelKey = UniqueKey();

    await tester.pumpWidget(chipWithOptionalDeleteButton(labelKey: labelKey, deletable: false));

    // Taps at a location close to the bottom-right corner of the chip.
    final Offset bottomRightOfInkWell = tester.getBottomRight(find.byType(InkWell));
    final Offset tapLocation = bottomRightOfInkWell + const Offset(-10, -10);
    final TestGesture gesture = await tester.startGesture(tapLocation);
    await tester.pump();

    // Waits for 100 ms.
    await tester.pump(const Duration(milliseconds: 100));

    // There should be exactly one ink-creating widget.
    expect(find.byType(InkWell), findsOneWidget);
    expect(find.byType(InkResponse), findsNothing);

    // There should be one unique, centered ink sparkle.
    await expectLater(
      find.byType(RawChip),
      matchesGoldenFile('chip.without_delete_button.ink_sparkle.0.png'),
    );

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for 100 ms again.
    await tester.pump(const Duration(milliseconds: 100));

    // The sparkle should grow, with the same center.
    // This indicates that the tap is not on a delete icon.
    await expectLater(
      find.byType(RawChip),
      matchesGoldenFile('chip.without_delete_button.ink_sparkle.1.png'),
    );

    // There should be no tooltip.
    expect(findTooltipContainer('Delete'), findsNothing);

    // Waits for a very long time.
    await tester.pumpAndSettle();

    // There should still be no tooltip.
    // This indicates that the tap is not on a delete icon.
    expect(findTooltipContainer('Delete'), findsNothing);

    await gesture.up();
  });

  testWidgets('Material2 - Selection with avatar works as expected on RawChip', (
    WidgetTester tester,
  ) async {
    bool selected = false;
    final UniqueKey labelKey = UniqueKey();
    Future<void> pushChip({Widget? avatar, bool selectable = false}) async {
      return tester.pumpWidget(
        wrapForChip(
          theme: ThemeData(useMaterial3: false),
          child: Wrap(
            children: <Widget>[
              StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return RawChip(
                    avatar: avatar,
                    onSelected:
                        selectable
                            ? (bool value) {
                              setState(() {
                                selected = value;
                              });
                            }
                            : null,
                    selected: selected,
                    label: Text('Long Chip Label', key: labelKey),
                    shape: const StadiumBorder(),
                  );
                },
              ),
            ],
          ),
        ),
      );
    }

    // With avatar, but not selectable.
    final UniqueKey avatarKey = UniqueKey();
    await pushChip(avatar: SizedBox(width: 40.0, height: 40.0, key: avatarKey));
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(258.0, 48.0)));

    // Turn on selection.
    await pushChip(avatar: SizedBox(width: 40.0, height: 40.0, key: avatarKey), selectable: true);
    await tester.pumpAndSettle();

    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));

    // Simulate a tap on the label to select the chip.
    await tester.tap(find.byKey(labelKey));
    expect(selected, equals(true));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(2));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.002, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.54, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(getSelectProgress(tester), equals(1.0));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pumpAndSettle();

    // Simulate another tap on the label to deselect the chip.
    await tester.tap(find.byKey(labelKey));
    expect(selected, equals(false));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(2));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 20));
    expect(getSelectProgress(tester), moreOrLessEquals(0.875, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 20));
    expect(getSelectProgress(tester), moreOrLessEquals(0.13, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(getSelectProgress(tester), equals(0.0));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
  });

  testWidgets('Material3 - Selection with avatar works as expected on RawChip', (
    WidgetTester tester,
  ) async {
    bool selected = false;
    final UniqueKey labelKey = UniqueKey();
    Future<void> pushChip({Widget? avatar, bool selectable = false}) async {
      return tester.pumpWidget(
        wrapForChip(
          child: Wrap(
            children: <Widget>[
              StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return RawChip(
                    avatar: avatar,
                    onSelected:
                        selectable
                            ? (bool value) {
                              setState(() {
                                selected = value;
                              });
                            }
                            : null,
                    selected: selected,
                    label: Text('Long Chip Label', key: labelKey),
                    shape: const StadiumBorder(),
                  );
                },
              ),
            ],
          ),
        ),
      );
    }

    // With avatar, but not selectable.
    final UniqueKey avatarKey = UniqueKey();
    await pushChip(avatar: SizedBox(width: 40.0, height: 40.0, key: avatarKey));
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(265.5, 48.0)));

    // Turn on selection.
    await pushChip(avatar: SizedBox(width: 40.0, height: 40.0, key: avatarKey), selectable: true);
    await tester.pumpAndSettle();

    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));

    // Simulate a tap on the label to select the chip.
    await tester.tap(find.byKey(labelKey));
    expect(selected, equals(true));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(kIsWeb && isSkiaWeb ? 3 : 1));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.002, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.54, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(getSelectProgress(tester), equals(1.0));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pumpAndSettle();

    // Simulate another tap on the label to deselect the chip.
    await tester.tap(find.byKey(labelKey));
    expect(selected, equals(false));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(kIsWeb && isSkiaWeb ? 3 : 1));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 20));
    expect(getSelectProgress(tester), moreOrLessEquals(0.875, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 20));
    expect(getSelectProgress(tester), moreOrLessEquals(0.13, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(getSelectProgress(tester), equals(0.0));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Material2 - Selection without avatar works as expected on RawChip', (
    WidgetTester tester,
  ) async {
    bool selected = false;
    final UniqueKey labelKey = UniqueKey();
    Future<void> pushChip({bool selectable = false}) async {
      return tester.pumpWidget(
        wrapForChip(
          theme: ThemeData(useMaterial3: false),
          child: Wrap(
            children: <Widget>[
              StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return RawChip(
                    onSelected:
                        selectable
                            ? (bool value) {
                              setState(() {
                                selected = value;
                              });
                            }
                            : null,
                    selected: selected,
                    label: Text('Long Chip Label', key: labelKey),
                    shape: const StadiumBorder(),
                  );
                },
              ),
            ],
          ),
        ),
      );
    }

    // Without avatar, but not selectable.
    await pushChip();
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(234.0, 48.0)));

    // Turn on selection.
    await pushChip(selectable: true);
    await tester.pumpAndSettle();

    // Simulate a tap on the label to select the chip.
    await tester.tap(find.byKey(labelKey));
    expect(selected, equals(true));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(2));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.002, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), moreOrLessEquals(0.459, epsilon: 0.01));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.54, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), moreOrLessEquals(0.92, epsilon: 0.01));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(getSelectProgress(tester), equals(1.0));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));

    await tester.pumpAndSettle();

    // Simulate another tap on the label to deselect the chip.
    await tester.tap(find.byKey(labelKey));
    expect(selected, equals(false));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(2));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 20));
    expect(getSelectProgress(tester), moreOrLessEquals(0.875, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), moreOrLessEquals(0.96, epsilon: 0.01));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 20));
    expect(getSelectProgress(tester), moreOrLessEquals(0.13, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), moreOrLessEquals(0.75, epsilon: 0.01));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(getSelectProgress(tester), equals(0.0));
    expect(getAvatarDrawerProgress(tester), equals(0.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
  });

  testWidgets('Material3 - Selection without avatar works as expected on RawChip', (
    WidgetTester tester,
  ) async {
    bool selected = false;
    final UniqueKey labelKey = UniqueKey();
    Future<void> pushChip({bool selectable = false}) async {
      return tester.pumpWidget(
        wrapForChip(
          child: Wrap(
            children: <Widget>[
              StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return RawChip(
                    onSelected:
                        selectable
                            ? (bool value) {
                              setState(() {
                                selected = value;
                              });
                            }
                            : null,
                    selected: selected,
                    label: Text('Long Chip Label', key: labelKey),
                    shape: const StadiumBorder(),
                  );
                },
              ),
            ],
          ),
        ),
      );
    }

    // Without avatar, but not selectable.
    await pushChip();
    expect(tester.getSize(find.byType(RawChip)), equals(const Size(245.5, 48.0)));

    // Turn on selection.
    await pushChip(selectable: true);
    await tester.pumpAndSettle();

    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));

    // Simulate a tap on the label to select the chip.
    await tester.tap(find.byKey(labelKey));
    expect(selected, equals(true));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(kIsWeb && isSkiaWeb ? 3 : 1));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.002, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), moreOrLessEquals(0.459, epsilon: 0.01));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.54, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), moreOrLessEquals(0.92, epsilon: 0.01));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(getSelectProgress(tester), equals(1.0));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));

    await tester.pumpAndSettle();

    // Simulate another tap on the label to deselect the chip.
    await tester.tap(find.byKey(labelKey));
    expect(selected, equals(false));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(kIsWeb && isSkiaWeb ? 3 : 1));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 20));
    expect(getSelectProgress(tester), moreOrLessEquals(0.875, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), moreOrLessEquals(0.96, epsilon: 0.01));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 20));
    expect(getSelectProgress(tester), moreOrLessEquals(0.13, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), moreOrLessEquals(0.75, epsilon: 0.01));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(getSelectProgress(tester), equals(0.0));
    expect(getAvatarDrawerProgress(tester), equals(0.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Material2 - Activation works as expected on RawChip', (WidgetTester tester) async {
    bool selected = false;
    final UniqueKey labelKey = UniqueKey();
    Future<void> pushChip({Widget? avatar, bool selectable = false}) async {
      return tester.pumpWidget(
        wrapForChip(
          theme: ThemeData(useMaterial3: false),
          child: Wrap(
            children: <Widget>[
              StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return RawChip(
                    avatar: avatar,
                    onSelected:
                        selectable
                            ? (bool value) {
                              setState(() {
                                selected = value;
                              });
                            }
                            : null,
                    selected: selected,
                    label: Text('Long Chip Label', key: labelKey),
                    shape: const StadiumBorder(),
                    showCheckmark: false,
                  );
                },
              ),
            ],
          ),
        ),
      );
    }

    final UniqueKey avatarKey = UniqueKey();
    await pushChip(avatar: SizedBox(width: 40.0, height: 40.0, key: avatarKey), selectable: true);
    await tester.pumpAndSettle();

    await tester.tap(find.byKey(labelKey));
    expect(selected, equals(true));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(2));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.002, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.54, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(getSelectProgress(tester), equals(1.0));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pumpAndSettle();
  });

  testWidgets('Material3 - Activation works as expected on RawChip', (WidgetTester tester) async {
    bool selected = false;
    final UniqueKey labelKey = UniqueKey();
    Future<void> pushChip({Widget? avatar, bool selectable = false}) async {
      return tester.pumpWidget(
        wrapForChip(
          child: Wrap(
            children: <Widget>[
              StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return RawChip(
                    avatar: avatar,
                    onSelected:
                        selectable
                            ? (bool value) {
                              setState(() {
                                selected = value;
                              });
                            }
                            : null,
                    selected: selected,
                    label: Text('Long Chip Label', key: labelKey),
                    shape: const StadiumBorder(),
                    showCheckmark: false,
                  );
                },
              ),
            ],
          ),
        ),
      );
    }

    final UniqueKey avatarKey = UniqueKey();
    await pushChip(avatar: SizedBox(width: 40.0, height: 40.0, key: avatarKey), selectable: true);
    await tester.pumpAndSettle();

    expect(SchedulerBinding.instance.transientCallbackCount, equals(0));

    await tester.tap(find.byKey(labelKey));
    expect(selected, equals(true));
    expect(SchedulerBinding.instance.transientCallbackCount, equals(kIsWeb && isSkiaWeb ? 3 : 1));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.002, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 50));
    expect(getSelectProgress(tester), moreOrLessEquals(0.54, epsilon: 0.01));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pump(const Duration(milliseconds: 100));
    expect(getSelectProgress(tester), equals(1.0));
    expect(getAvatarDrawerProgress(tester), equals(1.0));
    expect(getDeleteDrawerProgress(tester), equals(0.0));
    await tester.pumpAndSettle();
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Chip uses ThemeData chip theme if present', (WidgetTester tester) async {
    final ThemeData theme = ThemeData(
      chipTheme: const ChipThemeData(backgroundColor: Color(0xffff0000)),
    );

    Widget buildChip() {
      return wrapForChip(
        child: Theme(data: theme, child: InputChip(label: const Text('Label'), onPressed: () {})),
      );
    }

    await tester.pumpWidget(buildChip());

    final RenderBox materialBox = tester.firstRenderObject<RenderBox>(
      find.descendant(of: find.byType(RawChip), matching: find.byType(CustomPaint)),
    );

    expect(materialBox, paints..rrect(color: theme.chipTheme.backgroundColor));
  });

  testWidgets('Chip merges ChipThemeData label style with the provided label style', (
    WidgetTester tester,
  ) async {
    // The font family should be preserved even if the chip overrides some label style properties
    final ThemeData theme = ThemeData(fontFamily: 'MyFont');

    Widget buildChip() {
      return wrapForChip(
        child: Theme(
          data: theme,
          child: const Chip(
            label: Text('Label'),
            labelStyle: TextStyle(fontWeight: FontWeight.w200),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildChip());

    final TextStyle labelStyle = getLabelStyle(tester, 'Label').style;
    expect(labelStyle.inherit, false);
    expect(labelStyle.fontFamily, 'MyFont');
    expect(labelStyle.fontWeight, FontWeight.w200);
  });

  testWidgets('ChipTheme labelStyle with inherit:true', (WidgetTester tester) async {
    Widget buildChip() {
      return wrapForChip(
        child: Theme(
          data: ThemeData.light().copyWith(
            chipTheme: const ChipThemeData(
              labelStyle: TextStyle(height: 4), // inherit: true
            ),
          ),
          child: const Chip(label: Text('Label')), // labelStyle: null
        ),
      );
    }

    await tester.pumpWidget(buildChip());
    final TextStyle labelStyle = getLabelStyle(tester, 'Label').style;
    expect(labelStyle.inherit, true); // because chipTheme.labelStyle.merge(null)
    expect(labelStyle.height, 4);
  });

  testWidgets('Chip does not merge inherit:false label style with the theme label style', (
    WidgetTester tester,
  ) async {
    Widget buildChip() {
      return wrapForChip(
        child: Theme(
          data: ThemeData(fontFamily: 'MyFont'),
          child: const DefaultTextStyle(
            style: TextStyle(height: 8),
            child: Chip(
              label: Text('Label'),
              labelStyle: TextStyle(fontWeight: FontWeight.w200, inherit: false),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildChip());
    final TextStyle labelStyle = getLabelStyle(tester, 'Label').style;
    expect(labelStyle.inherit, false);
    expect(labelStyle.fontFamily, null);
    expect(labelStyle.height, null);
    expect(labelStyle.fontWeight, FontWeight.w200);
  });

  testWidgets('Material2 - Chip size is configurable by ThemeData.materialTapTargetSize', (
    WidgetTester tester,
  ) async {
    final Key key1 = UniqueKey();
    await tester.pumpWidget(
      wrapForChip(
        child: Theme(
          data: ThemeData(useMaterial3: false, materialTapTargetSize: MaterialTapTargetSize.padded),
          child: Center(child: RawChip(key: key1, label: const Text('test'))),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(key1)), const Size(80.0, 48.0));

    final Key key2 = UniqueKey();
    await tester.pumpWidget(
      wrapForChip(
        child: Theme(
          data: ThemeData(
            useMaterial3: false,
            materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
          ),
          child: Center(child: RawChip(key: key2, label: const Text('test'))),
        ),
      ),
    );

    expect(tester.getSize(find.byKey(key2)), const Size(80.0, 32.0));
  });

  testWidgets(
    'Material3 - Chip size is configurable by ThemeData.materialTapTargetSize',
    (WidgetTester tester) async {
      final Key key1 = UniqueKey();
      await tester.pumpWidget(
        wrapForChip(
          child: Theme(
            data: ThemeData(materialTapTargetSize: MaterialTapTargetSize.padded),
            child: Center(child: RawChip(key: key1, label: const Text('test'))),
          ),
        ),
      );

      expect(tester.getSize(find.byKey(key1)).width, moreOrLessEquals(90.4, epsilon: 0.1));
      expect(tester.getSize(find.byKey(key1)).height, equals(48.0));

      final Key key2 = UniqueKey();
      await tester.pumpWidget(
        wrapForChip(
          child: Theme(
            data: ThemeData(materialTapTargetSize: MaterialTapTargetSize.shrinkWrap),
            child: Center(child: RawChip(key: key2, label: const Text('test'))),
          ),
        ),
      );

      expect(tester.getSize(find.byKey(key2)).width, moreOrLessEquals(90.4, epsilon: 0.1));
      expect(tester.getSize(find.byKey(key2)).height, equals(38.0));
    },
    skip: kIsWeb && !isSkiaWeb, // https://github.com/flutter/flutter/issues/99933
  );

  testWidgets('Chip uses the right theme colors for the right components', (
    WidgetTester tester,
  ) async {
    final ThemeData themeData = ThemeData(
      platform: TargetPlatform.android,
      primarySwatch: Colors.blue,
    );
    final ChipThemeData defaultChipTheme = ChipThemeData.fromDefaults(
      brightness: themeData.brightness,
      secondaryColor: Colors.blue,
      labelStyle: themeData.textTheme.bodyLarge!,
    );
    bool value = false;
    Widget buildApp({
      ChipThemeData? chipTheme,
      Widget? avatar,
      Widget? deleteIcon,
      bool isSelectable = true,
      bool isPressable = false,
      bool isDeletable = true,
      bool showCheckmark = true,
    }) {
      chipTheme ??= defaultChipTheme;
      return wrapForChip(
        child: Theme(
          data: themeData,
          child: ChipTheme(
            data: chipTheme,
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return RawChip(
                  showCheckmark: showCheckmark,
                  onDeleted: isDeletable ? () {} : null,
                  avatar: avatar,
                  deleteIcon: deleteIcon,
                  isEnabled: isSelectable || isPressable,
                  shape: chipTheme?.shape,
                  selected: isSelectable && value,
                  label: Text('$value'),
                  onSelected:
                      isSelectable
                          ? (bool newValue) {
                            setState(() {
                              value = newValue;
                            });
                          }
                          : null,
                  onPressed:
                      isPressable
                          ? () {
                            setState(() {
                              value = true;
                            });
                          }
                          : null,
                );
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildApp());

    RenderBox materialBox = getMaterialBox(tester);
    IconThemeData iconData = getIconData(tester);
    DefaultTextStyle labelStyle = getLabelStyle(tester, 'false');

    // Check default theme for enabled widget.
    expect(materialBox, paints..rrect(color: defaultChipTheme.backgroundColor));
    expect(iconData.color, equals(const Color(0xde000000)));
    expect(labelStyle.style.color, equals(Colors.black.withAlpha(0xde)));
    await tester.tap(find.byType(RawChip));
    await tester.pumpAndSettle();
    materialBox = getMaterialBox(tester);
    expect(materialBox, paints..rrect(color: defaultChipTheme.selectedColor));
    await tester.tap(find.byType(RawChip));
    await tester.pumpAndSettle();

    // Check default theme with disabled widget.
    await tester.pumpWidget(buildApp(isSelectable: false));
    await tester.pumpAndSettle();
    materialBox = getMaterialBox(tester);
    labelStyle = getLabelStyle(tester, 'false');
    expect(materialBox, paints..rrect(color: defaultChipTheme.disabledColor));
    expect(labelStyle.style.color, equals(Colors.black.withAlpha(0xde)));

    // Apply a custom theme.
    const Color customColor1 = Color(0xcafefeed);
    const Color customColor2 = Color(0xdeadbeef);
    const Color customColor3 = Color(0xbeefcafe);
    const Color customColor4 = Color(0xaddedabe);
    final ChipThemeData customTheme = defaultChipTheme.copyWith(
      brightness: Brightness.dark,
      backgroundColor: customColor1,
      disabledColor: customColor2,
      selectedColor: customColor3,
      deleteIconColor: customColor4,
    );
    await tester.pumpWidget(buildApp(chipTheme: customTheme));
    await tester.pumpAndSettle();
    materialBox = getMaterialBox(tester);
    iconData = getIconData(tester);
    labelStyle = getLabelStyle(tester, 'false');

    // Check custom theme for enabled widget.
    expect(materialBox, paints..rrect(color: customTheme.backgroundColor));
    expect(iconData.color, equals(customTheme.deleteIconColor));
    expect(labelStyle.style.color, equals(Colors.black.withAlpha(0xde)));
    await tester.tap(find.byType(RawChip));
    await tester.pumpAndSettle();
    materialBox = getMaterialBox(tester);
    expect(materialBox, paints..rrect(color: customTheme.selectedColor));
    await tester.tap(find.byType(RawChip));
    await tester.pumpAndSettle();

    // Check custom theme with disabled widget.
    await tester.pumpWidget(buildApp(chipTheme: customTheme, isSelectable: false));
    await tester.pumpAndSettle();
    materialBox = getMaterialBox(tester);
    labelStyle = getLabelStyle(tester, 'false');
    expect(materialBox, paints..rrect(color: customTheme.disabledColor));
    expect(labelStyle.style.color, equals(Colors.black.withAlpha(0xde)));
  });

  group('Chip semantics', () {
    testWidgets('label only', (WidgetTester tester) async {
      final SemanticsTester semanticsTester = SemanticsTester(tester);

      await tester.pumpWidget(
        const MaterialApp(home: Material(child: RawChip(label: Text('test')))),
      );

      expect(
        semanticsTester,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    children: <TestSemantics>[
                      TestSemantics(
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            label: 'test',
                            textDirection: TextDirection.ltr,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasSelectedState,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isButton,
                            ],
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreTransform: true,
          ignoreId: true,
          ignoreRect: true,
        ),
      );
      semanticsTester.dispose();
    });

    testWidgets('delete', (WidgetTester tester) async {
      final SemanticsTester semanticsTester = SemanticsTester(tester);

      await tester.pumpWidget(
        MaterialApp(home: Material(child: RawChip(label: const Text('test'), onDeleted: () {}))),
      );

      expect(
        semanticsTester,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    children: <TestSemantics>[
                      TestSemantics(
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            label: 'test',
                            textDirection: TextDirection.ltr,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasSelectedState,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isButton,
                            ],
                            children: <TestSemantics>[
                              TestSemantics(
                                tooltip: 'Delete',
                                actions: <SemanticsAction>[
                                  SemanticsAction.tap,
                                  SemanticsAction.focus,
                                ],
                                textDirection: TextDirection.ltr,
                                flags: <SemanticsFlag>[
                                  SemanticsFlag.isButton,
                                  SemanticsFlag.isFocusable,
                                ],
                              ),
                            ],
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreTransform: true,
          ignoreId: true,
          ignoreRect: true,
        ),
      );
      semanticsTester.dispose();
    });

    testWidgets('with onPressed', (WidgetTester tester) async {
      final SemanticsTester semanticsTester = SemanticsTester(tester);

      await tester.pumpWidget(
        MaterialApp(home: Material(child: RawChip(label: const Text('test'), onPressed: () {}))),
      );

      expect(
        semanticsTester,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    children: <TestSemantics>[
                      TestSemantics(
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            label: 'test',
                            textDirection: TextDirection.ltr,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasSelectedState,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isButton,
                              SemanticsFlag.isEnabled,
                              SemanticsFlag.isFocusable,
                            ],
                            actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreTransform: true,
          ignoreId: true,
          ignoreRect: true,
        ),
      );

      semanticsTester.dispose();
    });

    testWidgets('with onSelected', (WidgetTester tester) async {
      final SemanticsTester semanticsTester = SemanticsTester(tester);
      bool selected = false;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: RawChip(
              label: const Text('test'),
              selected: selected,
              onSelected: (bool value) {
                selected = value;
              },
            ),
          ),
        ),
      );

      expect(
        semanticsTester,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    children: <TestSemantics>[
                      TestSemantics(
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            label: 'test',
                            textDirection: TextDirection.ltr,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasSelectedState,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isButton,
                              SemanticsFlag.isEnabled,
                              SemanticsFlag.isFocusable,
                            ],
                            actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreTransform: true,
          ignoreId: true,
          ignoreRect: true,
        ),
      );

      await tester.tap(find.byType(RawChip));
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: RawChip(
              label: const Text('test'),
              selected: selected,
              onSelected: (bool value) {
                selected = value;
              },
            ),
          ),
        ),
      );

      expect(selected, true);
      expect(
        semanticsTester,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    children: <TestSemantics>[
                      TestSemantics(
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            label: 'test',
                            textDirection: TextDirection.ltr,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isButton,
                              SemanticsFlag.isEnabled,
                              SemanticsFlag.isFocusable,
                              SemanticsFlag.hasSelectedState,
                              SemanticsFlag.isSelected,
                            ],
                            actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreTransform: true,
          ignoreId: true,
          ignoreRect: true,
        ),
      );

      semanticsTester.dispose();
    });

    testWidgets('disabled', (WidgetTester tester) async {
      final SemanticsTester semanticsTester = SemanticsTester(tester);

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: RawChip(isEnabled: false, onPressed: () {}, label: const Text('test')),
          ),
        ),
      );

      expect(
        semanticsTester,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    children: <TestSemantics>[
                      TestSemantics(
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            label: 'test',
                            textDirection: TextDirection.ltr,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasSelectedState,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isButton,
                            ],
                            actions: <SemanticsAction>[],
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreTransform: true,
          ignoreId: true,
          ignoreRect: true,
        ),
      );

      semanticsTester.dispose();
    });

    testWidgets('tapEnabled explicitly false', (WidgetTester tester) async {
      final SemanticsTester semanticsTester = SemanticsTester(tester);

      await tester.pumpWidget(
        const MaterialApp(home: Material(child: RawChip(tapEnabled: false, label: Text('test')))),
      );

      expect(
        semanticsTester,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    children: <TestSemantics>[
                      TestSemantics(
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            label: 'test',
                            textDirection: TextDirection.ltr,
                            // Must not be a button when tapping is disabled.
                            flags: <SemanticsFlag>[SemanticsFlag.hasSelectedState],
                            actions: <SemanticsAction>[],
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreTransform: true,
          ignoreId: true,
          ignoreRect: true,
        ),
      );

      semanticsTester.dispose();
    });

    testWidgets('enabled when tapEnabled and canTap', (WidgetTester tester) async {
      final SemanticsTester semanticsTester = SemanticsTester(tester);

      // These settings make a Chip which can be tapped, both in general and at this moment.
      await tester.pumpWidget(
        MaterialApp(home: Material(child: RawChip(onPressed: () {}, label: const Text('test')))),
      );

      expect(
        semanticsTester,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    children: <TestSemantics>[
                      TestSemantics(
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            label: 'test',
                            textDirection: TextDirection.ltr,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasSelectedState,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isButton,
                              SemanticsFlag.isEnabled,
                              SemanticsFlag.isFocusable,
                            ],
                            actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreTransform: true,
          ignoreId: true,
          ignoreRect: true,
        ),
      );

      semanticsTester.dispose();
    });

    testWidgets('disabled when tapEnabled but not canTap', (WidgetTester tester) async {
      final SemanticsTester semanticsTester = SemanticsTester(tester);
      // These settings make a Chip which _could_ be tapped, but not currently (ensures `canTap == false`).
      await tester.pumpWidget(
        const MaterialApp(home: Material(child: RawChip(label: Text('test')))),
      );

      expect(
        semanticsTester,
        hasSemantics(
          TestSemantics.root(
            children: <TestSemantics>[
              TestSemantics(
                textDirection: TextDirection.ltr,
                children: <TestSemantics>[
                  TestSemantics(
                    children: <TestSemantics>[
                      TestSemantics(
                        flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                        children: <TestSemantics>[
                          TestSemantics(
                            label: 'test',
                            textDirection: TextDirection.ltr,
                            flags: <SemanticsFlag>[
                              SemanticsFlag.hasSelectedState,
                              SemanticsFlag.hasEnabledState,
                              SemanticsFlag.isButton,
                            ],
                          ),
                        ],
                      ),
                    ],
                  ),
                ],
              ),
            ],
          ),
          ignoreTransform: true,
          ignoreId: true,
          ignoreRect: true,
        ),
      );

      semanticsTester.dispose();
    });
  });

  testWidgets('can be tapped outside of chip delete icon', (WidgetTester tester) async {
    bool deleted = false;
    await tester.pumpWidget(
      wrapForChip(
        child: Row(
          children: <Widget>[
            Chip(
              materialTapTargetSize: MaterialTapTargetSize.padded,
              shape: const RoundedRectangleBorder(),
              avatar: const CircleAvatar(child: Text('A')),
              label: const Text('Chip A'),
              onDeleted: () {
                deleted = true;
              },
              deleteIcon: const Icon(Icons.delete),
            ),
          ],
        ),
      ),
    );

    await tester.tapAt(tester.getTopRight(find.byType(Chip)) - const Offset(2.0, -2.0));
    await tester.pumpAndSettle();
    expect(deleted, true);
  });

  testWidgets('Chips can be tapped', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(home: Material(child: RawChip(label: Text('raw chip')))),
    );

    await tester.tap(find.byType(RawChip));
    expect(tester.takeException(), null);
  });

  testWidgets('Material2 - Chip elevation and shadow color work correctly', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData(
      useMaterial3: false,
      platform: TargetPlatform.android,
      primarySwatch: Colors.red,
    );

    InputChip inputChip = const InputChip(label: Text('Label'));

    Widget buildChip() {
      return wrapForChip(child: Theme(data: theme, child: inputChip));
    }

    await tester.pumpWidget(buildChip());
    Material material = getMaterial(tester);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.black);

    inputChip = const InputChip(
      label: Text('Label'),
      elevation: 4.0,
      shadowColor: Colors.green,
      selectedShadowColor: Colors.blue,
    );

    await tester.pumpWidget(buildChip());
    await tester.pumpAndSettle();
    material = getMaterial(tester);
    expect(material.elevation, 4.0);
    expect(material.shadowColor, Colors.green);

    inputChip = const InputChip(
      label: Text('Label'),
      selected: true,
      shadowColor: Colors.green,
      selectedShadowColor: Colors.blue,
    );

    await tester.pumpWidget(buildChip());
    await tester.pumpAndSettle();
    material = getMaterial(tester);
    expect(material.shadowColor, Colors.blue);
  });

  testWidgets('Material3 - Chip elevation and shadow color work correctly', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();

    InputChip inputChip = const InputChip(label: Text('Label'));

    Widget buildChip() {
      return wrapForChip(theme: theme, child: inputChip);
    }

    await tester.pumpWidget(buildChip());
    Material material = getMaterial(tester);
    expect(material.elevation, 0.0);
    expect(material.shadowColor, Colors.transparent);

    inputChip = const InputChip(
      label: Text('Label'),
      elevation: 4.0,
      shadowColor: Colors.green,
      selectedShadowColor: Colors.blue,
    );

    await tester.pumpWidget(buildChip());
    await tester.pumpAndSettle();
    material = getMaterial(tester);
    expect(material.elevation, 4.0);
    expect(material.shadowColor, Colors.green);

    inputChip = const InputChip(
      label: Text('Label'),
      selected: true,
      shadowColor: Colors.green,
      selectedShadowColor: Colors.blue,
    );

    await tester.pumpWidget(buildChip());
    await tester.pumpAndSettle();
    material = getMaterial(tester);
    expect(material.shadowColor, Colors.blue);
  });

  testWidgets('can be tapped outside of chip body', (WidgetTester tester) async {
    bool pressed = false;
    await tester.pumpWidget(
      wrapForChip(
        child: Row(
          children: <Widget>[
            InputChip(
              materialTapTargetSize: MaterialTapTargetSize.padded,
              shape: const RoundedRectangleBorder(),
              avatar: const CircleAvatar(child: Text('A')),
              label: const Text('Chip A'),
              onPressed: () {
                pressed = true;
              },
            ),
          ],
        ),
      ),
    );

    await tester.tapAt(tester.getRect(find.byType(InputChip)).topCenter);
    await tester.pumpAndSettle();
    expect(pressed, true);
  });

  testWidgets('is hitTestable', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrapForChip(
        child: InputChip(
          shape: const RoundedRectangleBorder(),
          avatar: const CircleAvatar(child: Text('A')),
          label: const Text('Chip A'),
          onPressed: () {},
        ),
      ),
    );

    expect(find.byType(InputChip).hitTestable(), findsOneWidget);
  });

  void checkChipMaterialClipBehavior(WidgetTester tester, Clip clipBehavior) {
    final Iterable<Material> materials = tester.widgetList<Material>(find.byType(Material));
    expect(materials.length, 2);
    expect(materials.last.clipBehavior, clipBehavior);
  }

  testWidgets('Chip clipBehavior properly passes through to the Material', (
    WidgetTester tester,
  ) async {
    const Text label = Text('label');
    await tester.pumpWidget(wrapForChip(child: const Chip(label: label)));
    checkChipMaterialClipBehavior(tester, Clip.none);

    await tester.pumpWidget(
      wrapForChip(child: const Chip(label: label, clipBehavior: Clip.antiAlias)),
    );
    checkChipMaterialClipBehavior(tester, Clip.antiAlias);
  });

  testWidgets('Material2 - selected chip and avatar draw darkened layer within avatar circle', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      wrapForChip(
        theme: ThemeData(useMaterial3: false),
        child: const FilterChip(
          avatar: CircleAvatar(child: Text('t')),
          label: Text('test'),
          selected: true,
          onSelected: null,
        ),
      ),
    );
    final RenderBox rawChip = tester.firstRenderObject<RenderBox>(
      find.descendant(
        of: find.byType(RawChip),
        matching: find.byWidgetPredicate((Widget widget) {
          return widget.runtimeType.toString() == '_ChipRenderWidget';
        }),
      ),
    );
    const Color selectScrimColor = Color(0x60191919);
    expect(
      rawChip,
      paints..path(
        color: selectScrimColor,
        includes: <Offset>[const Offset(10, 10)],
        excludes: <Offset>[const Offset(4, 4)],
      ),
    );
  });

  testWidgets('Material3 - selected chip and avatar draw darkened layer within avatar circle', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      wrapForChip(
        child: const FilterChip(
          avatar: CircleAvatar(child: Text('t')),
          label: Text('test'),
          selected: true,
          onSelected: null,
        ),
      ),
    );
    final RenderBox rawChip = tester.firstRenderObject<RenderBox>(
      find.descendant(
        of: find.byType(RawChip),
        matching: find.byWidgetPredicate((Widget widget) {
          return widget.runtimeType.toString() == '_ChipRenderWidget';
        }),
      ),
    );
    const Color selectScrimColor = Color(0x60191919);
    expect(
      rawChip,
      paints..path(
        color: selectScrimColor,
        includes: <Offset>[const Offset(11, 11)],
        excludes: <Offset>[const Offset(4, 4)],
      ),
    );
  });

  testWidgets('Chips should use InkWell instead of InkResponse.', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/28646
    await tester.pumpWidget(
      MaterialApp(
        home: Material(child: ActionChip(onPressed: () {}, label: const Text('action chip'))),
      ),
    );
    expect(find.byType(InkWell), findsOneWidget);
  });

  testWidgets('Chip uses stateful color for text color in different states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);
    const Color selectedColor = Color(0x00000005);
    const Color disabledColor = Color(0x00000006);

    Color getTextColor(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return disabledColor;
      }

      if (states.contains(MaterialState.pressed)) {
        return pressedColor;
      }

      if (states.contains(MaterialState.hovered)) {
        return hoverColor;
      }

      if (states.contains(MaterialState.focused)) {
        return focusedColor;
      }

      if (states.contains(MaterialState.selected)) {
        return selectedColor;
      }

      return defaultColor;
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        home: Scaffold(
          body: Focus(
            focusNode: focusNode,
            child: ChoiceChip(
              label: const Text('Chip'),
              selected: selected,
              onSelected: enabled ? (_) {} : null,
              labelStyle: TextStyle(color: MaterialStateColor.resolveWith(getTextColor)),
            ),
          ),
        ),
      );
    }

    Color textColor() {
      return tester.renderObject<RenderParagraph>(find.text('Chip')).text.style!.color!;
    }

    // Default, not disabled.
    await tester.pumpWidget(chipWidget());
    expect(textColor(), equals(defaultColor));

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(textColor(), selectedColor);

    // Focused.
    final FocusNode chipFocusNode = focusNode.children.first;
    chipFocusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(textColor(), focusedColor);

    // Hovered.
    final Offset center = tester.getCenter(find.byType(ChoiceChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(textColor(), hoverColor);

    // Pressed.
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(textColor(), pressedColor);

    // Disabled.
    await tester.pumpWidget(chipWidget(enabled: false));
    await tester.pumpAndSettle();
    expect(textColor(), disabledColor);
  });

  testWidgets('Material2 - Chip uses stateful border side color in different states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);
    const Color selectedColor = Color(0x00000005);
    const Color disabledColor = Color(0x00000006);

    BorderSide getBorderSide(Set<MaterialState> states) {
      Color sideColor = defaultColor;
      if (states.contains(MaterialState.disabled)) {
        sideColor = disabledColor;
      } else if (states.contains(MaterialState.pressed)) {
        sideColor = pressedColor;
      } else if (states.contains(MaterialState.hovered)) {
        sideColor = hoverColor;
      } else if (states.contains(MaterialState.focused)) {
        sideColor = focusedColor;
      } else if (states.contains(MaterialState.selected)) {
        sideColor = selectedColor;
      }
      return BorderSide(color: sideColor);
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Focus(
            focusNode: focusNode,
            child: ChoiceChip(
              label: const Text('Chip'),
              selected: selected,
              onSelected: enabled ? (_) {} : null,
              side: _MaterialStateBorderSide(getBorderSide),
            ),
          ),
        ),
      );
    }

    // Default, not disabled.
    await tester.pumpWidget(chipWidget());
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: defaultColor),
    );

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: selectedColor),
    );

    // Focused.
    final FocusNode chipFocusNode = focusNode.children.first;
    chipFocusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: focusedColor),
    );

    // Hovered.
    final Offset center = tester.getCenter(find.byType(ChoiceChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: hoverColor),
    );

    // Pressed.
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: pressedColor),
    );

    // Disabled.
    await tester.pumpWidget(chipWidget(enabled: false));
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: disabledColor),
    );
  });

  testWidgets('Material3 - Chip uses stateful border side color in different states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);
    const Color selectedColor = Color(0x00000005);
    const Color disabledColor = Color(0x00000006);

    BorderSide getBorderSide(Set<MaterialState> states) {
      Color sideColor = defaultColor;
      if (states.contains(MaterialState.disabled)) {
        sideColor = disabledColor;
      } else if (states.contains(MaterialState.pressed)) {
        sideColor = pressedColor;
      } else if (states.contains(MaterialState.hovered)) {
        sideColor = hoverColor;
      } else if (states.contains(MaterialState.focused)) {
        sideColor = focusedColor;
      } else if (states.contains(MaterialState.selected)) {
        sideColor = selectedColor;
      }
      return BorderSide(color: sideColor);
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        home: Scaffold(
          body: Focus(
            focusNode: focusNode,
            child: ChoiceChip(
              label: const Text('Chip'),
              selected: selected,
              onSelected: enabled ? (_) {} : null,
              side: _MaterialStateBorderSide(getBorderSide),
            ),
          ),
        ),
      );
    }

    // Default, not disabled.
    await tester.pumpWidget(chipWidget());
    expect(find.byType(RawChip), paints..drrect(color: defaultColor));

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(find.byType(RawChip), paints..drrect(color: selectedColor));

    // Focused.
    final FocusNode chipFocusNode = focusNode.children.first;
    chipFocusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: focusedColor));

    // Hovered.
    final Offset center = tester.getCenter(find.byType(ChoiceChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: hoverColor));

    // Pressed.
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: pressedColor));

    // Disabled.
    await tester.pumpWidget(chipWidget(enabled: false));
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: disabledColor));
  });

  testWidgets('Material2 - Chip uses stateful border side color from resolveWith', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);
    const Color selectedColor = Color(0x00000005);
    const Color disabledColor = Color(0x00000006);

    BorderSide getBorderSide(Set<MaterialState> states) {
      Color sideColor = defaultColor;
      if (states.contains(MaterialState.disabled)) {
        sideColor = disabledColor;
      } else if (states.contains(MaterialState.pressed)) {
        sideColor = pressedColor;
      } else if (states.contains(MaterialState.hovered)) {
        sideColor = hoverColor;
      } else if (states.contains(MaterialState.focused)) {
        sideColor = focusedColor;
      } else if (states.contains(MaterialState.selected)) {
        sideColor = selectedColor;
      }
      return BorderSide(color: sideColor);
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Focus(
            focusNode: focusNode,
            child: ChoiceChip(
              label: const Text('Chip'),
              selected: selected,
              onSelected: enabled ? (_) {} : null,
              side: MaterialStateBorderSide.resolveWith(getBorderSide),
            ),
          ),
        ),
      );
    }

    // Default, not disabled.
    await tester.pumpWidget(chipWidget());
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: defaultColor),
    );

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: selectedColor),
    );

    // Focused.
    final FocusNode chipFocusNode = focusNode.children.first;
    chipFocusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: focusedColor),
    );

    // Hovered.
    final Offset center = tester.getCenter(find.byType(ChoiceChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: hoverColor),
    );

    // Pressed.
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: pressedColor),
    );

    // Disabled.
    await tester.pumpWidget(chipWidget(enabled: false));
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: disabledColor),
    );
  });

  testWidgets('Material3 - Chip uses stateful border side color from resolveWith', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);
    const Color selectedColor = Color(0x00000005);
    const Color disabledColor = Color(0x00000006);

    BorderSide getBorderSide(Set<MaterialState> states) {
      Color sideColor = defaultColor;
      if (states.contains(MaterialState.disabled)) {
        sideColor = disabledColor;
      } else if (states.contains(MaterialState.pressed)) {
        sideColor = pressedColor;
      } else if (states.contains(MaterialState.hovered)) {
        sideColor = hoverColor;
      } else if (states.contains(MaterialState.focused)) {
        sideColor = focusedColor;
      } else if (states.contains(MaterialState.selected)) {
        sideColor = selectedColor;
      }
      return BorderSide(color: sideColor);
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        home: Scaffold(
          body: Focus(
            focusNode: focusNode,
            child: ChoiceChip(
              label: const Text('Chip'),
              selected: selected,
              onSelected: enabled ? (_) {} : null,
              side: MaterialStateBorderSide.resolveWith(getBorderSide),
            ),
          ),
        ),
      );
    }

    // Default, not disabled.
    await tester.pumpWidget(chipWidget());
    expect(find.byType(RawChip), paints..drrect(color: defaultColor));

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(find.byType(RawChip), paints..drrect(color: selectedColor));

    // Focused.
    final FocusNode chipFocusNode = focusNode.children.first;
    chipFocusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: focusedColor));

    // Hovered.
    final Offset center = tester.getCenter(find.byType(ChoiceChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: hoverColor));

    // Pressed.
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: pressedColor));

    // Disabled.
    await tester.pumpWidget(chipWidget(enabled: false));
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: disabledColor));
  });

  testWidgets('Material2 - Chip uses stateful nullable border side color from resolveWith', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);
    const Color disabledColor = Color(0x00000006);

    const Color fallbackThemeColor = Color(0x00000007);
    const BorderSide defaultBorderSide = BorderSide(color: fallbackThemeColor, width: 10.0);

    BorderSide? getBorderSide(Set<MaterialState> states) {
      Color sideColor = defaultColor;
      if (states.contains(MaterialState.disabled)) {
        sideColor = disabledColor;
      } else if (states.contains(MaterialState.pressed)) {
        sideColor = pressedColor;
      } else if (states.contains(MaterialState.hovered)) {
        sideColor = hoverColor;
      } else if (states.contains(MaterialState.focused)) {
        sideColor = focusedColor;
      } else if (states.contains(MaterialState.selected)) {
        return null;
      }
      return BorderSide(color: sideColor);
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Focus(
            focusNode: focusNode,
            child: ChipTheme(
              data: ThemeData.light().chipTheme.copyWith(side: defaultBorderSide),
              child: ChoiceChip(
                label: const Text('Chip'),
                selected: selected,
                onSelected: enabled ? (_) {} : null,
                side: MaterialStateBorderSide.resolveWith(getBorderSide),
              ),
            ),
          ),
        ),
      );
    }

    // Default, not disabled.
    await tester.pumpWidget(chipWidget());
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: defaultColor),
    );

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    // Because the resolver returns `null` for this value, we should fall back
    // to the theme.
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: fallbackThemeColor),
    );

    // Focused.
    final FocusNode chipFocusNode = focusNode.children.first;
    chipFocusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: focusedColor),
    );

    // Hovered.
    final Offset center = tester.getCenter(find.byType(ChoiceChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: hoverColor),
    );

    // Pressed.
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: pressedColor),
    );

    // Disabled.
    await tester.pumpWidget(chipWidget(enabled: false));
    await tester.pumpAndSettle();
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: disabledColor),
    );
  });

  testWidgets('Material3 - Chip uses stateful nullable border side color from resolveWith', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    const Color pressedColor = Color(0x00000001);
    const Color hoverColor = Color(0x00000002);
    const Color focusedColor = Color(0x00000003);
    const Color defaultColor = Color(0x00000004);
    const Color disabledColor = Color(0x00000006);

    const Color fallbackThemeColor = Color(0x00000007);
    const BorderSide defaultBorderSide = BorderSide(color: fallbackThemeColor, width: 10.0);

    BorderSide? getBorderSide(Set<MaterialState> states) {
      Color sideColor = defaultColor;
      if (states.contains(MaterialState.disabled)) {
        sideColor = disabledColor;
      } else if (states.contains(MaterialState.pressed)) {
        sideColor = pressedColor;
      } else if (states.contains(MaterialState.hovered)) {
        sideColor = hoverColor;
      } else if (states.contains(MaterialState.focused)) {
        sideColor = focusedColor;
      } else if (states.contains(MaterialState.selected)) {
        return null;
      }
      return BorderSide(color: sideColor);
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        home: Scaffold(
          body: Focus(
            focusNode: focusNode,
            child: ChipTheme(
              data: ThemeData.light().chipTheme.copyWith(side: defaultBorderSide),
              child: ChoiceChip(
                label: const Text('Chip'),
                selected: selected,
                onSelected: enabled ? (_) {} : null,
                side: MaterialStateBorderSide.resolveWith(getBorderSide),
              ),
            ),
          ),
        ),
      );
    }

    // Default, not disabled.
    await tester.pumpWidget(chipWidget());
    expect(find.byType(RawChip), paints..drrect(color: defaultColor));

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    // Because the resolver returns `null` for this value, we should fall back
    // to the theme
    expect(find.byType(RawChip), paints..drrect(color: fallbackThemeColor));

    // Focused.
    final FocusNode chipFocusNode = focusNode.children.first;
    chipFocusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: focusedColor));

    // Hovered.
    final Offset center = tester.getCenter(find.byType(ChoiceChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: hoverColor));

    // Pressed.
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: pressedColor));

    // Disabled.
    await tester.pumpWidget(chipWidget(enabled: false));
    await tester.pumpAndSettle();
    expect(find.byType(RawChip), paints..drrect(color: disabledColor));
  });

  testWidgets('Material2 - Chip uses stateful shape in different states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    OutlinedBorder? getShape(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return const BeveledRectangleBorder();
      } else if (states.contains(MaterialState.pressed)) {
        return const CircleBorder();
      } else if (states.contains(MaterialState.hovered)) {
        return const ContinuousRectangleBorder();
      } else if (states.contains(MaterialState.focused)) {
        return const RoundedRectangleBorder();
      } else if (states.contains(MaterialState.selected)) {
        return const BeveledRectangleBorder();
      }
      return null;
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Focus(
            focusNode: focusNode,
            child: ChoiceChip(
              selected: selected,
              label: const Text('Chip'),
              shape: _MaterialStateOutlinedBorder(getShape),
              onSelected: enabled ? (_) {} : null,
            ),
          ),
        ),
      );
    }

    // Default, not disabled. Defers to default shape.
    await tester.pumpWidget(chipWidget());
    expect(getMaterial(tester).shape, isA<StadiumBorder>());

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(getMaterial(tester).shape, isA<BeveledRectangleBorder>());

    // Focused.
    final FocusNode chipFocusNode = focusNode.children.first;
    chipFocusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(getMaterial(tester).shape, isA<RoundedRectangleBorder>());

    // Hovered.
    final Offset center = tester.getCenter(find.byType(ChoiceChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(getMaterial(tester).shape, isA<ContinuousRectangleBorder>());

    // Pressed.
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(getMaterial(tester).shape, isA<CircleBorder>());

    // Disabled.
    await tester.pumpWidget(chipWidget(enabled: false));
    await tester.pumpAndSettle();
    expect(getMaterial(tester).shape, isA<BeveledRectangleBorder>());
  });

  testWidgets('Material3 - Chip uses stateful shape in different states', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode();
    addTearDown(focusNode.dispose);

    OutlinedBorder? getShape(Set<MaterialState> states) {
      if (states.contains(MaterialState.disabled)) {
        return const BeveledRectangleBorder();
      } else if (states.contains(MaterialState.pressed)) {
        return const CircleBorder();
      } else if (states.contains(MaterialState.hovered)) {
        return const ContinuousRectangleBorder();
      } else if (states.contains(MaterialState.focused)) {
        return const RoundedRectangleBorder();
      } else if (states.contains(MaterialState.selected)) {
        return const BeveledRectangleBorder();
      }
      return null;
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        home: Scaffold(
          body: Focus(
            focusNode: focusNode,
            child: ChoiceChip(
              selected: selected,
              label: const Text('Chip'),
              shape: _MaterialStateOutlinedBorder(getShape),
              onSelected: enabled ? (_) {} : null,
            ),
          ),
        ),
      );
    }

    // Default, not disabled. Defers to default shape.
    await tester.pumpWidget(chipWidget());
    expect(getMaterial(tester).shape, isA<RoundedRectangleBorder>());

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(getMaterial(tester).shape, isA<BeveledRectangleBorder>());

    // Focused.
    final FocusNode chipFocusNode = focusNode.children.first;
    chipFocusNode.requestFocus();
    await tester.pumpAndSettle();
    expect(getMaterial(tester).shape, isA<RoundedRectangleBorder>());

    // Hovered.
    final Offset center = tester.getCenter(find.byType(ChoiceChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    await tester.pumpAndSettle();
    expect(getMaterial(tester).shape, isA<ContinuousRectangleBorder>());

    // Pressed.
    await gesture.down(center);
    await tester.pumpAndSettle();
    expect(getMaterial(tester).shape, isA<CircleBorder>());

    // Disabled.
    await tester.pumpWidget(chipWidget(enabled: false));
    await tester.pumpAndSettle();
    expect(getMaterial(tester).shape, isA<BeveledRectangleBorder>());
  });

  testWidgets('Material2 - Chip defers to theme, if shape and side resolves to null', (
    WidgetTester tester,
  ) async {
    const OutlinedBorder themeShape = StadiumBorder();
    const OutlinedBorder selectedShape = RoundedRectangleBorder();
    const BorderSide themeBorderSide = BorderSide(color: Color(0x00000001));
    const BorderSide selectedBorderSide = BorderSide(color: Color(0x00000002));

    OutlinedBorder? getShape(Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return selectedShape;
      }
      return null;
    }

    BorderSide? getBorderSide(Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return selectedBorderSide;
      }
      return null;
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(
          useMaterial3: false,
          chipTheme: ThemeData.light().chipTheme.copyWith(shape: themeShape, side: themeBorderSide),
        ),
        home: Scaffold(
          body: ChoiceChip(
            selected: selected,
            label: const Text('Chip'),
            shape: _MaterialStateOutlinedBorder(getShape),
            side: _MaterialStateBorderSide(getBorderSide),
            onSelected: enabled ? (_) {} : null,
          ),
        ),
      );
    }

    // Default, not disabled. Defer to theme.
    await tester.pumpWidget(chipWidget());
    expect(getMaterial(tester).shape, isA<StadiumBorder>());
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: themeBorderSide.color),
    );

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(getMaterial(tester).shape, isA<RoundedRectangleBorder>());
    expect(
      find.byType(RawChip),
      paints
        ..rect()
        ..drrect(color: selectedBorderSide.color),
    );
  });

  testWidgets('Chip defers to theme, if shape and side resolves to null', (
    WidgetTester tester,
  ) async {
    const OutlinedBorder themeShape = StadiumBorder();
    const OutlinedBorder selectedShape = RoundedRectangleBorder();
    const BorderSide themeBorderSide = BorderSide(color: Color(0x00000001));
    const BorderSide selectedBorderSide = BorderSide(color: Color(0x00000002));

    OutlinedBorder? getShape(Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return selectedShape;
      }
      return null;
    }

    BorderSide? getBorderSide(Set<MaterialState> states) {
      if (states.contains(MaterialState.selected)) {
        return selectedBorderSide;
      }
      return null;
    }

    Widget chipWidget({bool enabled = true, bool selected = false}) {
      return MaterialApp(
        theme: ThemeData(
          chipTheme: ThemeData.light().chipTheme.copyWith(shape: themeShape, side: themeBorderSide),
        ),
        home: Scaffold(
          body: ChoiceChip(
            selected: selected,
            label: const Text('Chip'),
            shape: _MaterialStateOutlinedBorder(getShape),
            side: _MaterialStateBorderSide(getBorderSide),
            onSelected: enabled ? (_) {} : null,
          ),
        ),
      );
    }

    // Default, not disabled. Defer to theme.
    await tester.pumpWidget(chipWidget());
    expect(getMaterial(tester).shape, isA<StadiumBorder>());
    expect(
      find.byType(RawChip),
      paints
        ..rrect()
        ..rrect(color: themeBorderSide.color),
    );

    // Selected.
    await tester.pumpWidget(chipWidget(selected: true));
    expect(getMaterial(tester).shape, isA<RoundedRectangleBorder>());
    expect(
      find.byType(RawChip),
      paints
        ..rect()
        ..drrect(color: selectedBorderSide.color),
    );
  });

  testWidgets('Material2 - Chip responds to density changes', (WidgetTester tester) async {
    const Key key = Key('test');
    const Key textKey = Key('test text');
    const Key iconKey = Key('test icon');
    const Key avatarKey = Key('test avatar');
    Future<void> buildTest(VisualDensity visualDensity) async {
      return tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Material(
            child: Center(
              child: Column(
                children: <Widget>[
                  InputChip(
                    visualDensity: visualDensity,
                    key: key,
                    onPressed: () {},
                    onDeleted: () {},
                    label: const Text('Test', key: textKey),
                    deleteIcon: const Icon(Icons.delete, key: iconKey),
                    avatar: const Icon(Icons.play_arrow, key: avatarKey),
                  ),
                ],
              ),
            ),
          ),
        ),
      );
    }

    // The Chips only change in size vertically in response to density, so
    // horizontal changes aren't expected.
    await buildTest(VisualDensity.standard);
    Rect box = tester.getRect(find.byKey(key));
    Rect textBox = tester.getRect(find.byKey(textKey));
    Rect iconBox = tester.getRect(find.byKey(iconKey));
    Rect avatarBox = tester.getRect(find.byKey(avatarKey));
    expect(box.size, equals(const Size(128, 32.0 + 16.0)));
    expect(textBox.size, equals(const Size(56, 14)));
    expect(iconBox.size, equals(const Size(18, 18)));
    expect(avatarBox.size, equals(const Size(18, 18)));
    expect(textBox.top, equals(17));
    expect(box.bottom - textBox.bottom, equals(17));
    expect(textBox.left, equals(372));
    expect(box.right - textBox.right, equals(36));

    // Try decreasing density (with higher density numbers).
    await buildTest(const VisualDensity(horizontal: 3.0, vertical: 3.0));
    box = tester.getRect(find.byKey(key));
    textBox = tester.getRect(find.byKey(textKey));
    iconBox = tester.getRect(find.byKey(iconKey));
    avatarBox = tester.getRect(find.byKey(avatarKey));
    expect(box.size, equals(const Size(128, 60)));
    expect(textBox.size, equals(const Size(56, 14)));
    expect(iconBox.size, equals(const Size(18, 18)));
    expect(avatarBox.size, equals(const Size(18, 18)));
    expect(textBox.top, equals(23));
    expect(box.bottom - textBox.bottom, equals(23));
    expect(textBox.left, equals(372));
    expect(box.right - textBox.right, equals(36));

    // Try increasing density (with lower density numbers).
    await buildTest(const VisualDensity(horizontal: -3.0, vertical: -3.0));
    box = tester.getRect(find.byKey(key));
    textBox = tester.getRect(find.byKey(textKey));
    iconBox = tester.getRect(find.byKey(iconKey));
    avatarBox = tester.getRect(find.byKey(avatarKey));
    expect(box.size, equals(const Size(128, 36)));
    expect(textBox.size, equals(const Size(56, 14)));
    expect(iconBox.size, equals(const Size(18, 18)));
    expect(avatarBox.size, equals(const Size(18, 18)));
    expect(textBox.top, equals(11));
    expect(box.bottom - textBox.bottom, equals(11));
    expect(textBox.left, equals(372));
    expect(box.right - textBox.right, equals(36));

    // Now test that horizontal and vertical are wired correctly. Negating the
    // horizontal should have no change over what's above.
    await buildTest(const VisualDensity(horizontal: 3.0, vertical: -3.0));
    await tester.pumpAndSettle();
    box = tester.getRect(find.byKey(key));
    textBox = tester.getRect(find.byKey(textKey));
    iconBox = tester.getRect(find.byKey(iconKey));
    avatarBox = tester.getRect(find.byKey(avatarKey));
    expect(box.size, equals(const Size(128, 36)));
    expect(textBox.size, equals(const Size(56, 14)));
    expect(iconBox.size, equals(const Size(18, 18)));
    expect(avatarBox.size, equals(const Size(18, 18)));
    expect(textBox.top, equals(11));
    expect(box.bottom - textBox.bottom, equals(11));
    expect(textBox.left, equals(372));
    expect(box.right - textBox.right, equals(36));

    // Make sure the "Comfortable" setting is the spec'd size
    await buildTest(VisualDensity.comfortable);
    await tester.pumpAndSettle();
    box = tester.getRect(find.byKey(key));
    expect(box.size, equals(const Size(128, 28.0 + 16.0)));

    // Make sure the "Compact" setting is the spec'd size
    await buildTest(VisualDensity.compact);
    await tester.pumpAndSettle();
    box = tester.getRect(find.byKey(key));
    expect(box.size, equals(const Size(128, 24.0 + 16.0)));
  });

  testWidgets('Material3 - Chip responds to density changes', (WidgetTester tester) async {
    const Key key = Key('test');
    const Key textKey = Key('test text');
    const Key iconKey = Key('test icon');
    const Key avatarKey = Key('test avatar');
    Future<void> buildTest(VisualDensity visualDensity) async {
      return tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: Column(
                children: <Widget>[
                  InputChip(
                    visualDensity: visualDensity,
                    key: key,
                    onPressed: () {},
                    onDeleted: () {},
                    label: const Text('Test', key: textKey),
                    deleteIcon: const Icon(Icons.delete, key: iconKey),
                    avatar: const Icon(Icons.play_arrow, key: avatarKey),
                  ),
                ],
              ),
            ),
          ),
        ),
      );
    }

    // The Chips only change in size vertically in response to density, so
    // horizontal changes aren't expected.
    await buildTest(VisualDensity.standard);
    Rect box = tester.getRect(find.byKey(key));
    Rect textBox = tester.getRect(find.byKey(textKey));
    Rect iconBox = tester.getRect(find.byKey(iconKey));
    Rect avatarBox = tester.getRect(find.byKey(avatarKey));
    expect(box.size.width, moreOrLessEquals(130.4, epsilon: 0.1));
    expect(box.size.height, equals(32.0 + 16.0));
    expect(textBox.size.width, moreOrLessEquals(56.4, epsilon: 0.1));
    expect(textBox.size.height, equals(20.0));
    expect(iconBox.size, equals(const Size(18, 18)));
    expect(avatarBox.size, equals(const Size(18, 18)));
    expect(textBox.top, equals(14));
    expect(box.bottom - textBox.bottom, equals(14));
    expect(textBox.left, moreOrLessEquals(371.79, epsilon: 0.1));
    expect(box.right - textBox.right, equals(37));

    // Try decreasing density (with higher density numbers).
    await buildTest(const VisualDensity(horizontal: 3.0, vertical: 3.0));
    box = tester.getRect(find.byKey(key));
    textBox = tester.getRect(find.byKey(textKey));
    iconBox = tester.getRect(find.byKey(iconKey));
    avatarBox = tester.getRect(find.byKey(avatarKey));
    expect(box.size.width, moreOrLessEquals(130.4, epsilon: 0.1));
    expect(box.size.height, equals(60));
    expect(textBox.size.width, moreOrLessEquals(56.4, epsilon: 0.1));
    expect(textBox.size.height, equals(20.0));
    expect(iconBox.size, equals(const Size(18, 18)));
    expect(avatarBox.size, equals(const Size(18, 18)));
    expect(textBox.top, equals(20));
    expect(box.bottom - textBox.bottom, equals(20));
    expect(textBox.left, moreOrLessEquals(371.79, epsilon: 0.1));
    expect(box.right - textBox.right, equals(37));

    // Try increasing density (with lower density numbers).
    await buildTest(const VisualDensity(horizontal: -3.0, vertical: -3.0));
    box = tester.getRect(find.byKey(key));
    textBox = tester.getRect(find.byKey(textKey));
    iconBox = tester.getRect(find.byKey(iconKey));
    avatarBox = tester.getRect(find.byKey(avatarKey));
    expect(box.size.width, moreOrLessEquals(130.4, epsilon: 0.1));
    expect(box.size.height, equals(36));
    expect(textBox.size.width, moreOrLessEquals(56.4, epsilon: 0.1));
    expect(textBox.size.height, equals(20.0));
    expect(iconBox.size, equals(const Size(18, 18)));
    expect(avatarBox.size, equals(const Size(18, 18)));
    expect(textBox.top, equals(8));
    expect(box.bottom - textBox.bottom, equals(8));
    expect(textBox.left, moreOrLessEquals(371.79, epsilon: 0.1));
    expect(box.right - textBox.right, equals(37));

    // Now test that horizontal and vertical are wired correctly. Negating the
    // horizontal should have no change over what's above.
    await buildTest(const VisualDensity(horizontal: 3.0, vertical: -3.0));
    await tester.pumpAndSettle();
    box = tester.getRect(find.byKey(key));
    textBox = tester.getRect(find.byKey(textKey));
    iconBox = tester.getRect(find.byKey(iconKey));
    avatarBox = tester.getRect(find.byKey(avatarKey));
    expect(box.size.width, moreOrLessEquals(130.4, epsilon: 0.1));
    expect(box.size.height, equals(36));
    expect(textBox.size.width, moreOrLessEquals(56.4, epsilon: 0.1));
    expect(textBox.size.height, equals(20.0));
    expect(iconBox.size, equals(const Size(18, 18)));
    expect(avatarBox.size, equals(const Size(18, 18)));
    expect(textBox.top, equals(8));
    expect(box.bottom - textBox.bottom, equals(8));
    expect(textBox.left, moreOrLessEquals(371.79, epsilon: 0.1));
    expect(box.right - textBox.right, equals(37));

    // Make sure the "Comfortable" setting is the spec'd size
    await buildTest(VisualDensity.comfortable);
    await tester.pumpAndSettle();
    box = tester.getRect(find.byKey(key));
    expect(box.size.width, moreOrLessEquals(130.4, epsilon: 0.1));
    expect(box.size.height, equals(28.0 + 16.0));

    // Make sure the "Compact" setting is the spec'd size
    await buildTest(VisualDensity.compact);
    await tester.pumpAndSettle();
    box = tester.getRect(find.byKey(key));
    expect(box.size.width, moreOrLessEquals(130.4, epsilon: 0.1));
    expect(box.size.height, equals(24.0 + 16.0));
  }, skip: kIsWeb && !isSkiaWeb); // https://github.com/flutter/flutter/issues/99933

  testWidgets('Chip delete button tooltip is disabled if deleteButtonTooltipMessage is empty', (
    WidgetTester tester,
  ) async {
    final UniqueKey deleteButtonKey = UniqueKey();
    await tester.pumpWidget(
      chipWithOptionalDeleteButton(
        deleteButtonKey: deleteButtonKey,
        deletable: true,
        deleteButtonTooltipMessage: '',
      ),
    );

    // Hover over the delete icon of the chip
    final Offset centerOfDeleteButton = tester.getCenter(find.byKey(deleteButtonKey));
    final TestGesture hoverGesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await hoverGesture.moveTo(centerOfDeleteButton);
    addTearDown(hoverGesture.removePointer);

    await tester.pump();

    // Wait for some more time while hovering over the delete button
    await tester.pumpAndSettle();

    // There should be no delete button tooltip
    expect(findTooltipContainer(''), findsNothing);
  });

  testWidgets('Disabling delete button tooltip does not disable chip tooltip', (
    WidgetTester tester,
  ) async {
    final UniqueKey deleteButtonKey = UniqueKey();
    await tester.pumpWidget(
      chipWithOptionalDeleteButton(
        deleteButtonKey: deleteButtonKey,
        deletable: true,
        deleteButtonTooltipMessage: '',
        chipTooltip: 'Chip Tooltip',
      ),
    );

    // Hover over the delete icon of the chip
    final Offset centerOfDeleteButton = tester.getCenter(find.byKey(deleteButtonKey));
    final TestGesture hoverGesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await hoverGesture.moveTo(centerOfDeleteButton);
    addTearDown(hoverGesture.removePointer);

    await tester.pump();

    // Wait for some more time while hovering over the delete button
    await tester.pumpAndSettle();

    // There should be no delete button tooltip
    expect(findTooltipContainer(''), findsNothing);
    // There should be a chip tooltip, however.
    expect(findTooltipContainer('Chip Tooltip'), findsOneWidget);
  });

  testWidgets('Triggering delete button tooltip does not trigger Chip tooltip', (
    WidgetTester tester,
  ) async {
    final UniqueKey deleteButtonKey = UniqueKey();
    await tester.pumpWidget(
      chipWithOptionalDeleteButton(
        deleteButtonKey: deleteButtonKey,
        deletable: true,
        chipTooltip: 'Chip Tooltip',
      ),
    );

    // Hover over the delete icon of the chip
    final Offset centerOfDeleteButton = tester.getCenter(find.byKey(deleteButtonKey));
    final TestGesture hoverGesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await hoverGesture.moveTo(centerOfDeleteButton);
    addTearDown(hoverGesture.removePointer);

    await tester.pump();

    // Wait for some more time while hovering over the delete button
    await tester.pumpAndSettle();

    // There should not be a chip tooltip
    expect(findTooltipContainer('Chip Tooltip'), findsNothing);
    // There should be a delete button tooltip
    expect(findTooltipContainer('Delete'), findsOneWidget);
  });

  testWidgets('intrinsicHeight implementation meets constraints', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/49478.
    await tester.pumpWidget(
      wrapForChip(
        child: const Chip(label: Text('text'), padding: EdgeInsets.symmetric(horizontal: 20)),
      ),
    );

    expect(tester.takeException(), isNull);
  });

  testWidgets('Material2 - Chip background color and shape are drawn on Ink', (
    WidgetTester tester,
  ) async {
    const Color backgroundColor = Color(0xff00ff00);
    const OutlinedBorder shape = ContinuousRectangleBorder();

    await tester.pumpWidget(
      wrapForChip(
        theme: ThemeData(useMaterial3: false),
        child: const RawChip(label: Text('text'), backgroundColor: backgroundColor, shape: shape),
      ),
    );

    final Ink ink = tester.widget(
      find.descendant(of: find.byType(RawChip), matching: find.byType(Ink)),
    );
    final ShapeDecoration decoration = ink.decoration! as ShapeDecoration;
    expect(decoration.color, backgroundColor);
    expect(decoration.shape, shape);
  });

  testWidgets('Material3 - Chip background color and shape are drawn on Ink', (
    WidgetTester tester,
  ) async {
    const Color backgroundColor = Color(0xff00ff00);
    const OutlinedBorder shape = ContinuousRectangleBorder();
    final ThemeData theme = ThemeData();

    await tester.pumpWidget(
      wrapForChip(
        theme: theme,
        child: const RawChip(label: Text('text'), backgroundColor: backgroundColor, shape: shape),
      ),
    );

    final Ink ink = tester.widget(
      find.descendant(of: find.byType(RawChip), matching: find.byType(Ink)),
    );
    final ShapeDecoration decoration = ink.decoration! as ShapeDecoration;
    expect(decoration.color, backgroundColor);
    expect(
      decoration.shape,
      shape.copyWith(side: BorderSide(color: theme.colorScheme.outlineVariant)),
    );
  });

  testWidgets('Chip highlight color is drawn on top of the backgroundColor', (
    WidgetTester tester,
  ) async {
    final FocusNode focusNode = FocusNode(debugLabel: 'RawChip');
    addTearDown(focusNode.dispose);
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
    const Color backgroundColor = Color(0xff00ff00);

    await tester.pumpWidget(
      wrapForChip(
        child: RawChip(
          label: const Text('text'),
          backgroundColor: backgroundColor,
          autofocus: true,
          focusNode: focusNode,
          onPressed: () {},
        ),
      ),
    );

    await tester.pumpAndSettle();

    expect(focusNode.hasPrimaryFocus, isTrue);
    expect(
      find.byType(Material).last,
      paints
        // Background color is drawn first.
        ..rrect(color: backgroundColor)
        // Highlight color is drawn on top of the background color.
        ..rect(color: const Color(0x1f000000)),
    );
  });

  testWidgets('RawChip.color resolves material states', (WidgetTester tester) async {
    const Color disabledSelectedColor = Color(0xffffff00);
    const Color disabledColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    const Color selectedColor = Color(0xffff0000);
    Widget buildApp({required bool enabled, required bool selected}) {
      return wrapForChip(
        child: RawChip(
          isEnabled: enabled,
          selected: selected,
          color: MaterialStateProperty.resolveWith((Set<MaterialState> states) {
            if (states.contains(MaterialState.disabled) &&
                states.contains(MaterialState.selected)) {
              return disabledSelectedColor;
            }
            if (states.contains(MaterialState.disabled)) {
              return disabledColor;
            }
            if (states.contains(MaterialState.selected)) {
              return selectedColor;
            }
            return backgroundColor;
          }),
          label: const Text('RawChip'),
        ),
      );
    }

    // Test enabled chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));

    // Enabled chip should have the provided backgroundColor.
    expect(getMaterialBox(tester), paints..rrect(color: backgroundColor));

    // Test disabled chip.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled chip should have the provided disabledColor.
    expect(getMaterialBox(tester), paints..rrect(color: disabledColor));

    // Test enabled & selected chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: true));
    await tester.pumpAndSettle();

    // Enabled & selected chip should have the provided selectedColor.
    expect(getMaterialBox(tester), paints..rrect(color: selectedColor));

    // Test disabled & selected chip.
    await tester.pumpWidget(buildApp(enabled: false, selected: true));
    await tester.pumpAndSettle();

    // Disabled & selected chip should have the provided disabledSelectedColor.
    expect(getMaterialBox(tester), paints..rrect(color: disabledSelectedColor));
  });

  testWidgets('RawChip uses provided state color properties', (WidgetTester tester) async {
    const Color disabledColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    const Color selectedColor = Color(0xffff0000);
    Widget buildApp({required bool enabled, required bool selected}) {
      return wrapForChip(
        child: RawChip(
          isEnabled: enabled,
          selected: selected,
          disabledColor: disabledColor,
          backgroundColor: backgroundColor,
          selectedColor: selectedColor,
          label: const Text('RawChip'),
        ),
      );
    }

    // Test enabled chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: false));

    // Enabled chip should have the provided backgroundColor.
    expect(getMaterialBox(tester), paints..rrect(color: backgroundColor));

    // Test disabled chip.
    await tester.pumpWidget(buildApp(enabled: false, selected: false));
    await tester.pumpAndSettle();

    // Disabled chip should have the provided disabledColor.
    expect(getMaterialBox(tester), paints..rrect(color: disabledColor));

    // Test enabled & selected chip.
    await tester.pumpWidget(buildApp(enabled: true, selected: true));
    await tester.pumpAndSettle();

    // Enabled & selected chip should have the provided selectedColor.
    expect(getMaterialBox(tester), paints..rrect(color: selectedColor));
  });

  testWidgets('Delete button tap target area does not include label', (WidgetTester tester) async {
    bool calledDelete = false;
    await tester.pumpWidget(
      wrapForChip(
        child: Column(
          children: <Widget>[
            Chip(
              label: const Text('Chip'),
              onDeleted: () {
                calledDelete = true;
              },
            ),
          ],
        ),
      ),
    );

    // Tap on the delete button.
    await tester.tapAt(tester.getCenter(find.byType(Icon)));
    await tester.pump();
    expect(calledDelete, isTrue);
    calledDelete = false;

    final Offset labelCenter = tester.getCenter(find.text('Chip'));

    // Tap on the label.
    await tester.tapAt(labelCenter);
    await tester.pump();
    expect(calledDelete, isFalse);

    // Tap before end of the label.
    final Size labelSize = tester.getSize(find.text('Chip'));
    await tester.tapAt(Offset(labelCenter.dx + (labelSize.width / 2) - 1, labelCenter.dy));
    await tester.pump();
    expect(calledDelete, isFalse);

    // Tap after end of the label.
    await tester.tapAt(Offset(labelCenter.dx + (labelSize.width / 2) + 0.01, labelCenter.dy));
    await tester.pump();
    expect(calledDelete, isTrue);
  });

  // This is a regression test for https://github.com/flutter/flutter/pull/133615.
  testWidgets('Material3 - Custom shape without provided side uses default side', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();
    await tester.pumpWidget(
      MaterialApp(
        theme: theme,
        home: const Material(
          child: Center(
            child: RawChip(
              // No side provided.
              shape: StadiumBorder(),
              label: Text('RawChip'),
            ),
          ),
        ),
      ),
    );

    // Chip should have the default side.
    expect(
      getMaterial(tester).shape,
      StadiumBorder(side: BorderSide(color: theme.colorScheme.outlineVariant)),
    );
  });

  testWidgets("Material3 - RawChip.shape's side is used when provided", (
    WidgetTester tester,
  ) async {
    Widget buildChip({OutlinedBorder? shape, BorderSide? side}) {
      return MaterialApp(
        home: Material(
          child: Center(child: RawChip(shape: shape, side: side, label: const Text('RawChip'))),
        ),
      );
    }

    // Test [RawChip.shape] with a side.
    await tester.pumpWidget(
      buildChip(
        shape: const RoundedRectangleBorder(
          side: BorderSide(color: Color(0xffff00ff)),
          borderRadius: BorderRadius.all(Radius.circular(7.0)),
        ),
      ),
    );

    // Chip should have the provided shape and the side from [RawChip.shape].
    expect(
      getMaterial(tester).shape,
      const RoundedRectangleBorder(
        side: BorderSide(color: Color(0xffff00ff)),
        borderRadius: BorderRadius.all(Radius.circular(7.0)),
      ),
    );

    // Test [RawChip.shape] with a side and [RawChip.side].
    await tester.pumpWidget(
      buildChip(
        shape: const RoundedRectangleBorder(
          side: BorderSide(color: Color(0xffff00ff)),
          borderRadius: BorderRadius.all(Radius.circular(7.0)),
        ),
        side: const BorderSide(color: Color(0xfffff000)),
      ),
    );
    await tester.pumpAndSettle();

    // Chip use shape from [RawChip.shape] and the side from [RawChip.side].
    // [RawChip.shape]'s side should be ignored.
    expect(
      getMaterial(tester).shape,
      const RoundedRectangleBorder(
        side: BorderSide(color: Color(0xfffff000)),
        borderRadius: BorderRadius.all(Radius.circular(7.0)),
      ),
    );
  });

  testWidgets('Material3 - Chip.iconTheme respects default iconTheme.size', (
    WidgetTester tester,
  ) async {
    Widget buildChip({IconThemeData? iconTheme}) {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.ltr,
          child: Material(
            child: Center(
              child: RawChip(
                iconTheme: iconTheme,
                avatar: const Icon(Icons.add),
                label: const SizedBox(width: 100, height: 100),
                onSelected: (bool newValue) {},
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildChip(iconTheme: const IconThemeData(color: Color(0xff332211))));

    // Icon should have the default chip iconSize.
    expect(getIconData(tester).size, 18.0);
    expect(getIconData(tester).color, const Color(0xff332211));

    // Icon should have the provided iconSize.
    await tester.pumpWidget(
      buildChip(iconTheme: const IconThemeData(color: Color(0xff112233), size: 23.0)),
    );
    await tester.pumpAndSettle();

    expect(getIconData(tester).size, 23.0);
    expect(getIconData(tester).color, const Color(0xff112233));
  });

  // This is a regression test for https://github.com/flutter/flutter/issues/138287.
  testWidgets("Enabling and disabling Chip with Tooltip doesn't throw an exception", (
    WidgetTester tester,
  ) async {
    bool isEnabled = true;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Column(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    RawChip(
                      tooltip: 'tooltip',
                      isEnabled: isEnabled,
                      onPressed: isEnabled ? () {} : null,
                      label: const Text('RawChip'),
                    ),
                    ElevatedButton(
                      onPressed: () {
                        setState(() {
                          isEnabled = !isEnabled;
                        });
                      },
                      child: Text('${isEnabled ? 'Disable' : 'Enable'} Chip'),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );

    // Tap the elevated button to disable the chip with a tooltip.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Disable Chip'));
    await tester.pumpAndSettle();

    // No exception should be thrown.
    expect(tester.takeException(), isNull);

    // Tap the elevated button to enable the chip with a tooltip.
    await tester.tap(find.widgetWithText(ElevatedButton, 'Enable Chip'));
    await tester.pumpAndSettle();

    // No exception should be thrown.
    expect(tester.takeException(), isNull);
  });

  testWidgets('Delete button is visible on disabled RawChip', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrapForChip(child: RawChip(isEnabled: false, label: const Text('Label'), onDeleted: () {})),
    );

    // Delete button should be visible.
    await expectLater(
      find.byType(RawChip),
      matchesGoldenFile('raw_chip.disabled.delete_button.png'),
    );
  });

  testWidgets('Delete button tooltip is not shown on disabled RawChip', (
    WidgetTester tester,
  ) async {
    Widget buildChip({bool enabled = true}) {
      return wrapForChip(
        child: RawChip(isEnabled: enabled, label: const Text('Label'), onDeleted: () {}),
      );
    }

    // Test enabled chip.
    await tester.pumpWidget(buildChip());

    final Offset deleteButtonLocation = tester.getCenter(find.byType(Icon));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(deleteButtonLocation);
    await tester.pump();

    // Delete button tooltip should be visible.
    expect(findTooltipContainer('Delete'), findsOneWidget);

    // Test disabled chip.
    await tester.pumpWidget(buildChip(enabled: false));
    await tester.pump();

    // Delete button tooltip should not be visible.
    expect(findTooltipContainer('Delete'), findsNothing);
  });

  testWidgets('Chip avatar layout constraints can be customized', (WidgetTester tester) async {
    const double border = 1.0;
    const double iconSize = 18.0;
    const double labelPadding = 8.0;
    const double padding = 8.0;
    const Size labelSize = Size(100, 100);

    Widget buildChip({BoxConstraints? avatarBoxConstraints}) {
      return wrapForChip(
        child: Center(
          child: Chip(
            avatarBoxConstraints: avatarBoxConstraints,
            avatar: const Icon(Icons.favorite),
            label: Container(
              width: labelSize.width,
              height: labelSize.width,
              color: const Color(0xFFFF0000),
            ),
          ),
        ),
      );
    }

    // Test default avatar layout constraints.
    await tester.pumpWidget(buildChip());

    expect(tester.getSize(find.byType(Chip)).width, equals(234.0));
    expect(tester.getSize(find.byType(Chip)).height, equals(118.0));

    // Calculate the distance between avatar and chip edges.
    Offset chipTopLeft = tester.getTopLeft(find.byWidget(getMaterial(tester)));
    final Offset avatarCenter = tester.getCenter(find.byIcon(Icons.favorite));
    expect(chipTopLeft.dx, avatarCenter.dx - (labelSize.width / 2) - padding - border);
    expect(chipTopLeft.dy, avatarCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between avatar and label.
    Offset labelTopLeft = tester.getTopLeft(find.byType(Container));
    expect(labelTopLeft.dx, avatarCenter.dx + (labelSize.width / 2) + labelPadding);

    // Test custom avatar layout constraints.
    await tester.pumpWidget(buildChip(avatarBoxConstraints: const BoxConstraints.tightForFinite()));
    await tester.pump();

    expect(tester.getSize(find.byType(Chip)).width, equals(152.0));
    expect(tester.getSize(find.byType(Chip)).height, equals(118.0));

    // Calculate the distance between avatar and chip edges.
    chipTopLeft = tester.getTopLeft(find.byWidget(getMaterial(tester)));
    expect(chipTopLeft.dx, avatarCenter.dx - (iconSize / 2) - padding - border);
    expect(chipTopLeft.dy, avatarCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between avatar and label.
    labelTopLeft = tester.getTopLeft(find.byType(Container));
    expect(labelTopLeft.dx, avatarCenter.dx + (iconSize / 2) + labelPadding);
  });

  testWidgets('RawChip avatar layout constraints can be customized', (WidgetTester tester) async {
    const double border = 1.0;
    const double iconSize = 18.0;
    const double labelPadding = 8.0;
    const double padding = 8.0;
    const Size labelSize = Size(100, 100);

    Widget buildChip({BoxConstraints? avatarBoxConstraints}) {
      return wrapForChip(
        child: Center(
          child: RawChip(
            avatarBoxConstraints: avatarBoxConstraints,
            avatar: const Icon(Icons.favorite),
            label: Container(
              width: labelSize.width,
              height: labelSize.width,
              color: const Color(0xFFFF0000),
            ),
          ),
        ),
      );
    }

    // Test default avatar layout constraints.
    await tester.pumpWidget(buildChip());

    expect(tester.getSize(find.byType(RawChip)).width, equals(234.0));
    expect(tester.getSize(find.byType(RawChip)).height, equals(118.0));

    // Calculate the distance between avatar and chip edges.
    Offset chipTopLeft = tester.getTopLeft(find.byWidget(getMaterial(tester)));
    final Offset avatarCenter = tester.getCenter(find.byIcon(Icons.favorite));
    expect(chipTopLeft.dx, avatarCenter.dx - (labelSize.width / 2) - padding - border);
    expect(chipTopLeft.dy, avatarCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between avatar and label.
    Offset labelTopLeft = tester.getTopLeft(find.byType(Container));
    expect(labelTopLeft.dx, avatarCenter.dx + (labelSize.width / 2) + labelPadding);

    // Test custom avatar layout constraints.
    await tester.pumpWidget(buildChip(avatarBoxConstraints: const BoxConstraints.tightForFinite()));
    await tester.pump();

    expect(tester.getSize(find.byType(RawChip)).width, equals(152.0));
    expect(tester.getSize(find.byType(RawChip)).height, equals(118.0));

    // Calculate the distance between avatar and chip edges.
    chipTopLeft = tester.getTopLeft(find.byWidget(getMaterial(tester)));
    expect(chipTopLeft.dx, avatarCenter.dx - (iconSize / 2) - padding - border);
    expect(chipTopLeft.dy, avatarCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between avatar and label.
    labelTopLeft = tester.getTopLeft(find.byType(Container));
    expect(labelTopLeft.dx, avatarCenter.dx + (iconSize / 2) + labelPadding);
  });

  testWidgets('Chip delete icon layout constraints can be customized', (WidgetTester tester) async {
    const double border = 1.0;
    const double iconSize = 18.0;
    const double labelPadding = 8.0;
    const double padding = 8.0;
    const Size labelSize = Size(100, 100);

    Widget buildChip({BoxConstraints? deleteIconBoxConstraints}) {
      return wrapForChip(
        child: Center(
          child: Chip(
            deleteIconBoxConstraints: deleteIconBoxConstraints,
            onDeleted: () {},
            label: Container(
              width: labelSize.width,
              height: labelSize.width,
              color: const Color(0xFFFF0000),
            ),
          ),
        ),
      );
    }

    // Test default delete icon layout constraints.
    await tester.pumpWidget(buildChip());

    expect(tester.getSize(find.byType(Chip)).width, equals(234.0));
    expect(tester.getSize(find.byType(Chip)).height, equals(118.0));

    // Calculate the distance between delete icon and chip edges.
    Offset chipTopRight = tester.getTopRight(find.byWidget(getMaterial(tester)));
    final Offset deleteIconCenter = tester.getCenter(find.byIcon(Icons.cancel));
    expect(chipTopRight.dx, deleteIconCenter.dx + (labelSize.width / 2) + padding + border);
    expect(chipTopRight.dy, deleteIconCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between delete icon and label.
    Offset labelTopRight = tester.getTopRight(find.byType(Container));
    expect(labelTopRight.dx, deleteIconCenter.dx - (labelSize.width / 2) - labelPadding);

    // Test custom avatar layout constraints.
    await tester.pumpWidget(
      buildChip(deleteIconBoxConstraints: const BoxConstraints.tightForFinite()),
    );
    await tester.pump();

    expect(tester.getSize(find.byType(Chip)).width, equals(152.0));
    expect(tester.getSize(find.byType(Chip)).height, equals(118.0));

    // Calculate the distance between delete icon and chip edges.
    chipTopRight = tester.getTopRight(find.byWidget(getMaterial(tester)));
    expect(chipTopRight.dx, deleteIconCenter.dx + (iconSize / 2) + padding + border);
    expect(chipTopRight.dy, deleteIconCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between delete icon and label.
    labelTopRight = tester.getTopRight(find.byType(Container));
    expect(labelTopRight.dx, deleteIconCenter.dx - (iconSize / 2) - labelPadding);
  });

  testWidgets('RawChip delete icon layout constraints can be customized', (
    WidgetTester tester,
  ) async {
    const double border = 1.0;
    const double iconSize = 18.0;
    const double labelPadding = 8.0;
    const double padding = 8.0;
    const Size labelSize = Size(100, 100);

    Widget buildChip({BoxConstraints? deleteIconBoxConstraints}) {
      return wrapForChip(
        child: Center(
          child: RawChip(
            deleteIconBoxConstraints: deleteIconBoxConstraints,
            onDeleted: () {},
            label: Container(
              width: labelSize.width,
              height: labelSize.width,
              color: const Color(0xFFFF0000),
            ),
          ),
        ),
      );
    }

    // Test default delete icon layout constraints.
    await tester.pumpWidget(buildChip());

    expect(tester.getSize(find.byType(RawChip)).width, equals(234.0));
    expect(tester.getSize(find.byType(RawChip)).height, equals(118.0));

    // Calculate the distance between delete icon and chip edges.
    Offset chipTopRight = tester.getTopRight(find.byWidget(getMaterial(tester)));
    final Offset deleteIconCenter = tester.getCenter(find.byIcon(Icons.cancel));
    expect(chipTopRight.dx, deleteIconCenter.dx + (labelSize.width / 2) + padding + border);
    expect(chipTopRight.dy, deleteIconCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between delete icon and label.
    Offset labelTopRight = tester.getTopRight(find.byType(Container));
    expect(labelTopRight.dx, deleteIconCenter.dx - (labelSize.width / 2) - labelPadding);

    // Test custom avatar layout constraints.
    await tester.pumpWidget(
      buildChip(deleteIconBoxConstraints: const BoxConstraints.tightForFinite()),
    );
    await tester.pump();

    expect(tester.getSize(find.byType(RawChip)).width, equals(152.0));
    expect(tester.getSize(find.byType(RawChip)).height, equals(118.0));

    // Calculate the distance between delete icon and chip edges.
    chipTopRight = tester.getTopRight(find.byWidget(getMaterial(tester)));
    expect(chipTopRight.dx, deleteIconCenter.dx + (iconSize / 2) + padding + border);
    expect(chipTopRight.dy, deleteIconCenter.dy - (labelSize.width / 2) - padding - border);

    // Calculate the distance between delete icon and label.
    labelTopRight = tester.getTopRight(find.byType(Container));
    expect(labelTopRight.dx, deleteIconCenter.dx - (iconSize / 2) - labelPadding);
  });

  testWidgets('Default delete button InkWell shape', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrapForChip(child: Center(child: RawChip(onDeleted: () {}, label: const Text('RawChip')))),
    );

    final InkWell deleteButtonInkWell = tester.widget<InkWell>(
      find.ancestor(of: find.byIcon(Icons.cancel), matching: find.byType(InkWell).last),
    );
    expect(deleteButtonInkWell.customBorder, const CircleBorder());
  });

  testWidgets('Default delete button overlay', (WidgetTester tester) async {
    final ThemeData theme = ThemeData();
    await tester.pumpWidget(
      wrapForChip(
        child: Center(child: RawChip(onDeleted: () {}, label: const Text('RawChip'))),
        theme: theme,
      ),
    );

    SplashController controller = Material.of(tester.element(find.byType(Ink)));
    expect(controller, isNot(paints..rect(color: theme.hoverColor)));
    expect(controller, paintsExactlyCountTimes(#clipPath, 0));

    // Hover over the delete icon.
    final Offset centerOfDeleteButton = tester.getCenter(find.byType(Icon));
    final TestGesture hoverGesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await hoverGesture.moveTo(centerOfDeleteButton);
    addTearDown(hoverGesture.removePointer);
    await tester.pumpAndSettle();

    controller = Material.of(tester.element(find.byType(Ink)));
    expect(controller, paints..rect(color: theme.hoverColor));
    expect(controller, paintsExactlyCountTimes(#clipPath, 1));

    const Rect expectedClipRect = Rect.fromLTRB(124.7, 10.0, 142.7, 28.0);
    final Path expectedClipPath = Path()..addRect(expectedClipRect);
    expect(
      controller,
      paints..clipPath(
        pathMatcher: coversSameAreaAs(
          expectedClipPath,
          areaToCompare: expectedClipRect.inflate(48.0),
          sampleSize: 100,
        ),
      ),
    );
  });

  group('Material 2', () {
    // These tests are only relevant for Material 2. Once Material 2
    // support is deprecated and the APIs are removed, these tests
    // can be deleted.

    testWidgets('M2 Chip defaults', (WidgetTester tester) async {
      late TextTheme textTheme;

      Widget buildFrame(Brightness brightness) {
        return MaterialApp(
          theme: ThemeData(brightness: brightness, useMaterial3: false),
          home: Scaffold(
            body: Center(
              child: Builder(
                builder: (BuildContext context) {
                  textTheme = Theme.of(context).textTheme;
                  return Chip(
                    avatar: const CircleAvatar(child: Text('A')),
                    label: const Text('Chip A'),
                    onDeleted: () {},
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildFrame(Brightness.light));
      expect(
        getMaterialBox(tester),
        paints
          ..rrect()
          ..circle(color: const Color(0xff1976d2)),
      );
      expect(tester.getSize(find.byType(Chip)), const Size(156.0, 48.0));
      expect(getMaterial(tester).color, null);
      expect(getMaterial(tester).elevation, 0);
      expect(getMaterial(tester).shape, const StadiumBorder());
      expect(getIconData(tester).color, const Color(0xdd000000));
      expect(getIconData(tester).opacity, null);
      expect(getIconData(tester).size, 18.0);

      TextStyle labelStyle = getLabelStyle(tester, 'Chip A').style;
      expect(labelStyle.color?.value, 0xde000000);
      expect(labelStyle.fontFamily, textTheme.bodyLarge?.fontFamily);
      expect(labelStyle.fontFamilyFallback, textTheme.bodyLarge?.fontFamilyFallback);
      expect(labelStyle.fontFeatures, textTheme.bodyLarge?.fontFeatures);
      expect(labelStyle.fontSize, textTheme.bodyLarge?.fontSize);
      expect(labelStyle.fontStyle, textTheme.bodyLarge?.fontStyle);
      expect(labelStyle.fontWeight, textTheme.bodyLarge?.fontWeight);
      expect(labelStyle.height, textTheme.bodyLarge?.height);
      expect(labelStyle.inherit, textTheme.bodyLarge?.inherit);
      expect(labelStyle.leadingDistribution, textTheme.bodyLarge?.leadingDistribution);
      expect(labelStyle.letterSpacing, textTheme.bodyLarge?.letterSpacing);
      expect(labelStyle.overflow, textTheme.bodyLarge?.overflow);
      expect(labelStyle.textBaseline, textTheme.bodyLarge?.textBaseline);
      expect(labelStyle.wordSpacing, textTheme.bodyLarge?.wordSpacing);

      await tester.pumpWidget(buildFrame(Brightness.dark));
      await tester.pumpAndSettle(); // Theme transition animation
      expect(getMaterialBox(tester), paints..rrect(color: const Color(0x1fffffff)));
      expect(tester.getSize(find.byType(Chip)), const Size(156.0, 48.0));
      expect(getMaterial(tester).color, null);
      expect(getMaterial(tester).elevation, 0);
      expect(getMaterial(tester).shape, const StadiumBorder());
      expect(getIconData(tester).color?.value, 0xffffffff);
      expect(getIconData(tester).opacity, null);
      expect(getIconData(tester).size, 18.0);

      labelStyle = getLabelStyle(tester, 'Chip A').style;
      expect(labelStyle.color?.value, 0xdeffffff);
      expect(labelStyle.fontFamily, textTheme.bodyLarge?.fontFamily);
      expect(labelStyle.fontFamilyFallback, textTheme.bodyLarge?.fontFamilyFallback);
      expect(labelStyle.fontFeatures, textTheme.bodyLarge?.fontFeatures);
      expect(labelStyle.fontSize, textTheme.bodyLarge?.fontSize);
      expect(labelStyle.fontStyle, textTheme.bodyLarge?.fontStyle);
      expect(labelStyle.fontWeight, textTheme.bodyLarge?.fontWeight);
      expect(labelStyle.height, textTheme.bodyLarge?.height);
      expect(labelStyle.inherit, textTheme.bodyLarge?.inherit);
      expect(labelStyle.leadingDistribution, textTheme.bodyLarge?.leadingDistribution);
      expect(labelStyle.letterSpacing, textTheme.bodyLarge?.letterSpacing);
      expect(labelStyle.overflow, textTheme.bodyLarge?.overflow);
      expect(labelStyle.textBaseline, textTheme.bodyLarge?.textBaseline);
      expect(labelStyle.wordSpacing, textTheme.bodyLarge?.wordSpacing);
    });

    testWidgets('Chip uses the right theme colors for the right components', (
      WidgetTester tester,
    ) async {
      final ThemeData themeData = ThemeData(
        platform: TargetPlatform.android,
        primarySwatch: Colors.blue,
        useMaterial3: false,
      );
      final ChipThemeData defaultChipTheme = ChipThemeData.fromDefaults(
        brightness: themeData.brightness,
        secondaryColor: Colors.blue,
        labelStyle: themeData.textTheme.bodyLarge!,
      );
      bool value = false;
      Widget buildApp({
        ChipThemeData? chipTheme,
        Widget? avatar,
        Widget? deleteIcon,
        bool isSelectable = true,
        bool isPressable = false,
        bool isDeletable = true,
        bool showCheckmark = true,
      }) {
        chipTheme ??= defaultChipTheme;
        return wrapForChip(
          child: Theme(
            data: themeData,
            child: ChipTheme(
              data: chipTheme,
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return RawChip(
                    showCheckmark: showCheckmark,
                    onDeleted: isDeletable ? () {} : null,
                    avatar: avatar,
                    deleteIcon: deleteIcon,
                    isEnabled: isSelectable || isPressable,
                    shape: chipTheme?.shape,
                    selected: isSelectable && value,
                    label: Text('$value'),
                    onSelected:
                        isSelectable
                            ? (bool newValue) {
                              setState(() {
                                value = newValue;
                              });
                            }
                            : null,
                    onPressed:
                        isPressable
                            ? () {
                              setState(() {
                                value = true;
                              });
                            }
                            : null,
                  );
                },
              ),
            ),
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      RenderBox materialBox = getMaterialBox(tester);
      IconThemeData iconData = getIconData(tester);
      DefaultTextStyle labelStyle = getLabelStyle(tester, 'false');

      // Check default theme for enabled chip.
      expect(materialBox, paints..rrect(color: defaultChipTheme.backgroundColor));
      expect(iconData.color, equals(const Color(0xde000000)));
      expect(labelStyle.style.color, equals(Colors.black.withAlpha(0xde)));

      // Check default theme for disabled chip.
      await tester.pumpWidget(buildApp(isSelectable: false));
      await tester.pumpAndSettle();
      materialBox = getMaterialBox(tester);
      labelStyle = getLabelStyle(tester, 'false');
      expect(materialBox, paints..rrect(color: defaultChipTheme.disabledColor));
      expect(labelStyle.style.color, equals(Colors.black.withAlpha(0xde)));

      // Check default theme for enabled and selected chip.
      await tester.pumpWidget(buildApp());
      await tester.pumpAndSettle();
      await tester.tap(find.byType(RawChip));
      await tester.pumpAndSettle();
      materialBox = getMaterialBox(tester);
      expect(materialBox, paints..rrect(color: defaultChipTheme.selectedColor));

      // Check default theme for disabled and selected chip.
      await tester.pumpWidget(buildApp(isSelectable: false));
      await tester.pumpAndSettle();
      materialBox = getMaterialBox(tester);
      labelStyle = getLabelStyle(tester, 'true');
      expect(materialBox, paints..rrect(color: defaultChipTheme.disabledColor));
      expect(labelStyle.style.color, equals(Colors.black.withAlpha(0xde)));

      // Enable the chip again.
      await tester.pumpWidget(buildApp());
      await tester.pumpAndSettle();
      // Tap to unselect the chip.
      await tester.tap(find.byType(RawChip));
      await tester.pumpAndSettle();

      // Apply a custom theme.
      const Color customColor1 = Color(0xcafefeed);
      const Color customColor2 = Color(0xdeadbeef);
      const Color customColor3 = Color(0xbeefcafe);
      const Color customColor4 = Color(0xaddedabe);
      final ChipThemeData customTheme = defaultChipTheme.copyWith(
        brightness: Brightness.dark,
        backgroundColor: customColor1,
        disabledColor: customColor2,
        selectedColor: customColor3,
        deleteIconColor: customColor4,
      );
      await tester.pumpWidget(buildApp(chipTheme: customTheme));
      await tester.pumpAndSettle();
      materialBox = getMaterialBox(tester);
      iconData = getIconData(tester);
      labelStyle = getLabelStyle(tester, 'false');

      // Check custom theme for enabled chip.
      expect(materialBox, paints..rrect(color: customTheme.backgroundColor));
      expect(iconData.color, equals(customTheme.deleteIconColor));
      expect(labelStyle.style.color, equals(Colors.black.withAlpha(0xde)));

      // Check custom theme with disabled widget.
      await tester.pumpWidget(buildApp(chipTheme: customTheme, isSelectable: false));
      await tester.pumpAndSettle();
      materialBox = getMaterialBox(tester);
      labelStyle = getLabelStyle(tester, 'false');
      expect(materialBox, paints..rrect(color: customTheme.disabledColor));
      expect(labelStyle.style.color, equals(Colors.black.withAlpha(0xde)));

      // Check custom theme for enabled and selected chip.
      await tester.pumpWidget(buildApp(chipTheme: customTheme));
      await tester.pumpAndSettle();
      await tester.tap(find.byType(RawChip));
      await tester.pumpAndSettle();
      materialBox = getMaterialBox(tester);
      expect(materialBox, paints..rrect(color: customTheme.selectedColor));

      // Check custom theme for disabled and selected chip.
      await tester.pumpWidget(buildApp(chipTheme: customTheme, isSelectable: false));
      await tester.pumpAndSettle();
      materialBox = getMaterialBox(tester);
      labelStyle = getLabelStyle(tester, 'true');
      expect(materialBox, paints..rrect(color: customTheme.disabledColor));
      expect(labelStyle.style.color, equals(Colors.black.withAlpha(0xde)));
    });
  });

  testWidgets('Chip Baseline location', (WidgetTester tester) async {
    const Text text = Text('A', style: TextStyle(fontSize: 10.0, height: 1.0));
    await tester.pumpWidget(
      wrapForChip(
        child: const Align(
          child: Row(
            crossAxisAlignment: CrossAxisAlignment.baseline,
            textBaseline: TextBaseline.alphabetic,
            children: <Widget>[text, RawChip(label: text)],
          ),
        ),
      ),
    );

    expect(find.text('A'), findsNWidgets(2));
    // Baseline aligning text.
    expect(tester.getTopLeft(find.text('A').first).dy, tester.getTopLeft(find.text('A').last).dy);
  });

  testWidgets('ChipThemeData.iconTheme updates avatar and delete icons', (
    WidgetTester tester,
  ) async {
    const Color iconColor = Color(0xffff00ff);
    const double iconSize = 28.0;
    const IconData avatarIcon = Icons.favorite;
    const IconData deleteIcon = Icons.delete;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: RawChip(
              iconTheme: const IconThemeData(color: iconColor, size: iconSize),
              avatar: const Icon(Icons.favorite),
              deleteIcon: const Icon(Icons.delete),
              onDeleted: () {},
              label: const SizedBox(height: 100),
            ),
          ),
        ),
      ),
    );

    // Test rendered icon size.
    final RenderBox avatarIconBox = tester.renderObject(find.byIcon(avatarIcon));
    final RenderBox deleteIconBox = tester.renderObject(find.byIcon(deleteIcon));
    expect(avatarIconBox.size.width, equals(iconSize));
    expect(deleteIconBox.size.width, equals(iconSize));

    // Test rendered icon color.
    expect(getIconStyle(tester, avatarIcon)?.color, iconColor);
    expect(getIconStyle(tester, deleteIcon)?.color, iconColor);
  });

  testWidgets('RawChip.deleteIconColor overrides iconTheme color', (WidgetTester tester) async {
    const Color iconColor = Color(0xffff00ff);
    const Color deleteIconColor = Color(0xffff00ff);
    const IconData deleteIcon = Icons.delete;

    Widget buildChip({Color? deleteIconColor, Color? iconColor}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: RawChip(
              deleteIconColor: deleteIconColor,
              iconTheme: IconThemeData(color: iconColor),
              deleteIcon: const Icon(Icons.delete),
              onDeleted: () {},
              label: const SizedBox(height: 100),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildChip(iconColor: iconColor));

    // Test rendered icon color.
    expect(getIconStyle(tester, deleteIcon)?.color, iconColor);

    await tester.pumpWidget(buildChip(deleteIconColor: deleteIconColor, iconColor: iconColor));

    // Test rendered icon color.
    expect(getIconStyle(tester, deleteIcon)?.color, deleteIconColor);
  });

  testWidgets('Chip label only does layout once', (WidgetTester tester) async {
    final RenderLayoutCount renderLayoutCount = RenderLayoutCount();
    final Widget layoutCounter = Center(
      key: GlobalKey(),
      child: WidgetToRenderBoxAdapter(renderBox: renderLayoutCount),
    );

    await tester.pumpWidget(wrapForChip(child: RawChip(label: layoutCounter)));

    expect(renderLayoutCount.layoutCount, 1);
  });

  testWidgets('ChipAnimationStyle.enableAnimation overrides chip enable animation duration', (
    WidgetTester tester,
  ) async {
    const Color disabledColor = Color(0xffff0000);
    const Color backgroundColor = Color(0xff00ff00);
    bool enabled = true;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Column(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    RawChip(
                      chipAnimationStyle: ChipAnimationStyle(
                        enableAnimation: AnimationStyle(
                          duration: const Duration(milliseconds: 300),
                          reverseDuration: const Duration(milliseconds: 150),
                        ),
                      ),
                      isEnabled: enabled,
                      disabledColor: disabledColor,
                      backgroundColor: backgroundColor,
                      label: const Text('RawChip'),
                    ),
                    ElevatedButton(
                      onPressed: () {
                        setState(() {
                          enabled = !enabled;
                        });
                      },
                      child: Text('${enabled ? 'Disable' : 'Enable'} Chip'),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );

    final RenderBox materialBox = tester.firstRenderObject<RenderBox>(
      find.descendant(of: find.byType(RawChip), matching: find.byType(CustomPaint)),
    );

    // Test background color when the chip is enabled.
    expect(materialBox, paints..rrect(color: backgroundColor));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Disable Chip'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 75));

    expect(materialBox, paints..rrect(color: const Color(0x80ff0000)));

    await tester.pump(const Duration(milliseconds: 75));

    // Test background color when the chip is disabled.
    expect(materialBox, paints..rrect(color: disabledColor));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Enable Chip'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 150));

    expect(materialBox, paints..rrect(color: const Color(0x8000ff00)));

    await tester.pump(const Duration(milliseconds: 150));

    // Test background color when the chip is enabled.
    expect(materialBox, paints..rrect(color: backgroundColor));
  });

  testWidgets('ChipAnimationStyle.selectAnimation overrides chip selection animation duration', (
    WidgetTester tester,
  ) async {
    const Color backgroundColor = Color(0xff00ff00);
    const Color selectedColor = Color(0xff0000ff);
    bool selected = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setState) {
                return Column(
                  mainAxisSize: MainAxisSize.min,
                  children: <Widget>[
                    RawChip(
                      chipAnimationStyle: ChipAnimationStyle(
                        selectAnimation: AnimationStyle(
                          duration: const Duration(milliseconds: 600),
                          reverseDuration: const Duration(milliseconds: 300),
                        ),
                      ),
                      backgroundColor: backgroundColor,
                      selectedColor: selectedColor,
                      selected: selected,
                      onSelected: (bool value) {},
                      label: const Text('RawChip'),
                    ),
                    ElevatedButton(
                      onPressed: () {
                        setState(() {
                          selected = !selected;
                        });
                      },
                      child: Text('${selected ? 'Unselect' : 'Select'} Chip'),
                    ),
                  ],
                );
              },
            ),
          ),
        ),
      ),
    );

    final RenderBox materialBox = tester.firstRenderObject<RenderBox>(
      find.descendant(of: find.byType(RawChip), matching: find.byType(CustomPaint)),
    );

    // Test background color when the chip is unselected.
    expect(materialBox, paints..rrect(color: backgroundColor));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Select Chip'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 300));

    expect(materialBox, paints..rrect(color: const Color(0xc60000ff)));

    await tester.pump(const Duration(milliseconds: 300));

    // Test background color when the chip is selected.
    expect(materialBox, paints..rrect(color: selectedColor));

    await tester.tap(find.widgetWithText(ElevatedButton, 'Unselect Chip'));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 150));

    expect(materialBox, paints..rrect(color: const Color(0x3900ff00)));

    await tester.pump(const Duration(milliseconds: 150));

    // Test background color when the chip is unselected.
    expect(materialBox, paints..rrect(color: backgroundColor));
  });

  testWidgets(
    'ChipAnimationStyle.avatarDrawerAnimation overrides chip avatar animation duration',
    (WidgetTester tester) async {
      const Color checkmarkColor = Color(0xffff0000);
      bool selected = false;

      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Column(
                    mainAxisSize: MainAxisSize.min,
                    children: <Widget>[
                      RawChip(
                        chipAnimationStyle: ChipAnimationStyle(
                          avatarDrawerAnimation: AnimationStyle(
                            duration: const Duration(milliseconds: 800),
                            reverseDuration: const Duration(milliseconds: 400),
                          ),
                        ),
                        checkmarkColor: checkmarkColor,
                        selected: selected,
                        onSelected: (bool value) {},
                        label: const Text('RawChip'),
                      ),
                      ElevatedButton(
                        onPressed: () {
                          setState(() {
                            selected = !selected;
                          });
                        },
                        child: Text('${selected ? 'Unselect' : 'Select'} Chip'),
                      ),
                    ],
                  );
                },
              ),
            ),
          ),
        ),
      );

      final RenderBox materialBox = tester.firstRenderObject<RenderBox>(
        find.descendant(of: find.byType(RawChip), matching: find.byType(CustomPaint)),
      );

      // Test the checkmark is not visible yet.
      expect(materialBox, isNot(paints..path(color: checkmarkColor)));
      expect(tester.getSize(find.byType(RawChip)).width, closeTo(132.6, 0.1));

      await tester.tap(find.widgetWithText(ElevatedButton, 'Select Chip'));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 400));

      expect(materialBox, paints..path(color: checkmarkColor));
      expect(tester.getSize(find.byType(RawChip)).width, closeTo(148.2, 0.1));

      await tester.pump(const Duration(milliseconds: 400));

      // Test the checkmark is fully visible.
      expect(materialBox, paints..path(color: checkmarkColor));
      expect(tester.getSize(find.byType(RawChip)).width, closeTo(152.6, 0.1));

      await tester.tap(find.widgetWithText(ElevatedButton, 'Unselect Chip'));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 200));

      expect(materialBox, isNot(paints..path(color: checkmarkColor)));
      expect(tester.getSize(find.byType(RawChip)).width, closeTo(148.2, 0.1));

      await tester.pump(const Duration(milliseconds: 200));

      // Test if checkmark is removed.
      expect(materialBox, isNot(paints..path(color: checkmarkColor)));
      expect(tester.getSize(find.byType(RawChip)).width, closeTo(132.6, 0.1));
    },
    skip: kIsWeb && !isSkiaWeb, // https://github.com/flutter/flutter/issues/99933
  );

  testWidgets(
    'ChipAnimationStyle.deleteDrawerAnimation overrides chip delete icon animation duration',
    (WidgetTester tester) async {
      bool showDeleteIcon = false;
      await tester.pumpWidget(
        MaterialApp(
          home: Material(
            child: Center(
              child: StatefulBuilder(
                builder: (BuildContext context, StateSetter setState) {
                  return Column(
                    mainAxisSize: MainAxisSize.min,
                    children: <Widget>[
                      RawChip(
                        chipAnimationStyle: ChipAnimationStyle(
                          deleteDrawerAnimation: AnimationStyle(
                            duration: const Duration(milliseconds: 500),
                            reverseDuration: const Duration(milliseconds: 250),
                          ),
                        ),
                        onDeleted: showDeleteIcon ? () {} : null,
                        label: const Text('RawChip'),
                      ),
                      ElevatedButton(
                        onPressed: () {
                          setState(() {
                            showDeleteIcon = !showDeleteIcon;
                          });
                        },
                        child: Text('${showDeleteIcon ? 'Hide' : 'Show'} delete icon'),
                      ),
                    ],
                  );
                },
              ),
            ),
          ),
        ),
      );

      // Test the delete icon is not visible yet.
      expect(find.byIcon(Icons.cancel), findsNothing);
      expect(tester.getSize(find.byType(RawChip)).width, closeTo(132.6, 0.1));

      await tester.tap(find.widgetWithText(ElevatedButton, 'Show delete icon'));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 250));

      expect(find.byIcon(Icons.cancel), findsOneWidget);
      expect(tester.getSize(find.byType(RawChip)).width, closeTo(148.2, 0.1));

      await tester.pump(const Duration(milliseconds: 250));

      // Test the delete icon is fully visible.
      expect(find.byIcon(Icons.cancel), findsOneWidget);
      expect(tester.getSize(find.byType(RawChip)).width, closeTo(152.6, 0.1));

      await tester.tap(find.widgetWithText(ElevatedButton, 'Hide delete icon'));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 125));

      expect(find.byIcon(Icons.cancel), findsOneWidget);
      expect(tester.getSize(find.byType(RawChip)).width, closeTo(148.2, 0.1));

      await tester.pump(const Duration(milliseconds: 125));

      // Test if delete icon is removed.
      expect(find.byIcon(Icons.cancel), findsNothing);
      expect(tester.getSize(find.byType(RawChip)).width, closeTo(132.6, 0.1));
    },
    skip: kIsWeb && !isSkiaWeb, // https://github.com/flutter/flutter/issues/99933
  );

  testWidgets('Chip.chipAnimationStyle is passed to RawChip', (WidgetTester tester) async {
    final ChipAnimationStyle chipAnimationStyle = ChipAnimationStyle(
      enableAnimation: AnimationStyle.noAnimation,
      selectAnimation: AnimationStyle(duration: Durations.long3),
    );

    await tester.pumpWidget(
      wrapForChip(
        child: Center(
          child: Chip(chipAnimationStyle: chipAnimationStyle, label: const Text('Chip')),
        ),
      ),
    );

    expect(tester.widget<RawChip>(find.byType(RawChip)).chipAnimationStyle, chipAnimationStyle);
  });

  // Regression test for https://github.com/flutter/flutter/issues/157622.
  testWidgets('Chip does not glitch on hover when providing ThemeData.hoverColor', (
    WidgetTester tester,
  ) async {
    const Color themeDataHoverColor = Color(0xffff0000);
    const Color hoverColor = Color(0xff00ff00);
    const Color backgroundColor = Color(0xff0000ff);
    await tester.pumpWidget(
      wrapForChip(
        theme: ThemeData(hoverColor: themeDataHoverColor),
        child: Center(
          child: RawChip(
            color: WidgetStateProperty.resolveWith((Set<WidgetState> states) {
              if (states.contains(WidgetState.hovered)) {
                return hoverColor;
              }
              return backgroundColor;
            }),
            label: const Text('Chip'),
            onPressed: () {},
          ),
        ),
      ),
    );

    expect(getMaterialBox(tester), paints..rrect(color: backgroundColor));

    // Hover over the chip.
    final Offset center = tester.getCenter(find.byType(RawChip));
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer();
    await gesture.moveTo(center);
    addTearDown(gesture.removePointer);
    await tester.pumpAndSettle();

    expect(
      getMaterialBox(tester),
      paints
        ..rrect(color: hoverColor)
        ..rect(color: Colors.transparent),
    );
    expect(
      getMaterialBox(tester),
      isNot(
        paints
          ..rrect(color: hoverColor)
          ..rect(color: themeDataHoverColor),
      ),
    );
  });

  testWidgets('Chip mouse cursor behavior', (WidgetTester tester) async {
    const SystemMouseCursor customCursor = SystemMouseCursors.grab;

    await tester.pumpWidget(
      wrapForChip(child: const Center(child: Chip(mouseCursor: customCursor, label: Text('Chip')))),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: const Offset(10, 10));
    await tester.pump();
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );

    final Offset chip = tester.getCenter(find.text('Chip'));
    await gesture.moveTo(chip);
    await tester.pump();
    expect(RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1), customCursor);
  });

  testWidgets('Mouse cursor resolves in disabled states', (WidgetTester tester) async {
    tester.binding.focusManager.highlightStrategy = FocusHighlightStrategy.alwaysTraditional;

    await tester.pumpWidget(
      wrapForChip(
        child: const Center(
          child: Chip(
            mouseCursor: WidgetStateMouseCursor.fromMap(<WidgetStatesConstraint, MouseCursor>{
              WidgetState.disabled: SystemMouseCursors.forbidden,
            }),
            label: Text('Chip'),
          ),
        ),
      ),
    );
    // Unfocused case.
    final TestGesture gesture1 = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    addTearDown(gesture1.removePointer);
    await gesture1.addPointer(location: tester.getCenter(find.text('Chip')));
    await tester.pump();
    await gesture1.moveTo(tester.getCenter(find.text('Chip')));
    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
  });
}

class _MaterialStateOutlinedBorder extends StadiumBorder implements MaterialStateOutlinedBorder {
  const _MaterialStateOutlinedBorder(this.resolver);

  final MaterialPropertyResolver<OutlinedBorder?> resolver;

  @override
  OutlinedBorder? resolve(Set<MaterialState> states) => resolver(states);
}

class _MaterialStateBorderSide extends MaterialStateBorderSide {
  const _MaterialStateBorderSide(this.resolver);

  final MaterialPropertyResolver<BorderSide?> resolver;

  @override
  BorderSide? resolve(Set<MaterialState> states) => resolver(states);
}

class RenderLayoutCount extends RenderBox {
  int layoutCount = 0;

  @override
  Size computeDryLayout(covariant BoxConstraints constraints) => constraints.biggest;

  @override
  void performLayout() {
    layoutCount += 1;
    size = constraints.biggest;
  }
}
