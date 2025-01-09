// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';
import '../widgets/semantics_tester.dart';

const Key avatarA = Key('A');
const Key avatarC = Key('C');
const Key avatarD = Key('D');

Future<void> pumpTestWidget(
  WidgetTester tester, {
  bool withName = true,
  bool withEmail = true,
  bool withOnDetailsPressedHandler = true,
  Size otherAccountsPictureSize = const Size.square(40.0),
  Size currentAccountPictureSize = const Size.square(72.0),
  Color? primaryColor,
  Color? colorSchemePrimary,
}) async {
  await tester.pumpWidget(
    MaterialApp(
      theme: ThemeData(
        primaryColor: primaryColor,
        colorScheme: const ColorScheme.light().copyWith(primary: colorSchemePrimary),
      ),
      home: MediaQuery(
        data: const MediaQueryData(
          padding: EdgeInsets.only(left: 10.0, top: 20.0, right: 30.0, bottom: 40.0),
        ),
        child: Material(
          child: Center(
            child: UserAccountsDrawerHeader(
              onDetailsPressed: withOnDetailsPressedHandler ? () {} : null,
              currentAccountPictureSize: currentAccountPictureSize,
              otherAccountsPicturesSize: otherAccountsPictureSize,
              currentAccountPicture: const ExcludeSemantics(
                child: CircleAvatar(key: avatarA, child: Text('A')),
              ),
              otherAccountsPictures: const <Widget>[
                CircleAvatar(child: Text('B')),
                CircleAvatar(key: avatarC, child: Text('C')),
                CircleAvatar(key: avatarD, child: Text('D')),
                CircleAvatar(child: Text('E')),
              ],
              accountName: withName ? const Text('name') : null,
              accountEmail: withEmail ? const Text('email') : null,
            ),
          ),
        ),
      ),
    ),
  );
}

void main() {
  // Find the exact transform which is the descendant of [UserAccountsDrawerHeader].
  final Finder findTransform = find.descendant(
    of: find.byType(UserAccountsDrawerHeader),
    matching: find.byType(Transform),
  );

  testWidgets('UserAccountsDrawerHeader inherits ColorScheme.primary', (WidgetTester tester) async {
    const Color primaryColor = Color(0xff00ff00);
    const Color colorSchemePrimary = Color(0xff0000ff);

    await pumpTestWidget(
      tester,
      primaryColor: primaryColor,
      colorSchemePrimary: colorSchemePrimary,
    );

    final BoxDecoration? boxDecoration =
        tester.widget<DrawerHeader>(find.byType(DrawerHeader)).decoration as BoxDecoration?;
    expect(boxDecoration?.color == primaryColor, false);
    expect(boxDecoration?.color == colorSchemePrimary, true);
  });

  testWidgets('UserAccountsDrawerHeader test', (WidgetTester tester) async {
    await pumpTestWidget(tester);

    expect(find.text('A'), findsOneWidget);
    expect(find.text('B'), findsOneWidget);
    expect(find.text('C'), findsOneWidget);
    expect(find.text('D'), findsOneWidget);
    expect(find.text('E'), findsNothing);

    expect(find.text('name'), findsOneWidget);
    expect(find.text('email'), findsOneWidget);

    RenderBox box = tester.renderObject(find.byKey(avatarA));
    expect(box.size.width, equals(72.0));
    expect(box.size.height, equals(72.0));

    box = tester.renderObject(find.byKey(avatarC));
    expect(box.size.width, equals(40.0));
    expect(box.size.height, equals(40.0));

    // Verify height = height + top padding + bottom margin + bottom edge)
    box = tester.renderObject(find.byType(UserAccountsDrawerHeader));
    expect(box.size.height, equals(160.0 + 20.0 + 8.0 + 1.0));

    final Offset topLeft = tester.getTopLeft(find.byType(UserAccountsDrawerHeader));
    final Offset topRight = tester.getTopRight(find.byType(UserAccountsDrawerHeader));

    final Offset avatarATopLeft = tester.getTopLeft(find.byKey(avatarA));
    final Offset avatarDTopRight = tester.getTopRight(find.byKey(avatarD));
    final Offset avatarCTopRight = tester.getTopRight(find.byKey(avatarC));

    expect(avatarATopLeft.dx - topLeft.dx, equals(16.0 + 10.0)); // left padding
    expect(avatarATopLeft.dy - topLeft.dy, equals(16.0 + 20.0)); // add top padding
    expect(topRight.dx - avatarDTopRight.dx, equals(16.0 + 30.0)); // right padding
    expect(avatarDTopRight.dy - topRight.dy, equals(16.0 + 20.0)); // add top padding
    expect(avatarDTopRight.dx - avatarCTopRight.dx, equals(40.0 + 16.0)); // size + space between
  });

  testWidgets('UserAccountsDrawerHeader change default size test', (WidgetTester tester) async {
    const Size currentAccountPictureSize = Size.square(60.0);
    const Size otherAccountsPictureSize = Size.square(30.0);

    await pumpTestWidget(
      tester,
      currentAccountPictureSize: currentAccountPictureSize,
      otherAccountsPictureSize: otherAccountsPictureSize,
    );

    final RenderBox currentAccountRenderBox = tester.renderObject(find.byKey(avatarA));
    final RenderBox otherAccountRenderBox = tester.renderObject(find.byKey(avatarC));

    expect(currentAccountRenderBox.size, currentAccountPictureSize);
    expect(otherAccountRenderBox.size, otherAccountsPictureSize);
  });

  testWidgets('UserAccountsDrawerHeader icon rotation test', (WidgetTester tester) async {
    await pumpTestWidget(tester);
    Transform transformWidget = tester.firstWidget(findTransform);

    // Icon is right side up.
    expect(transformWidget.transform.getRotation()[0], 1.0);
    expect(transformWidget.transform.getRotation()[4], 1.0);

    await tester.tap(find.byType(Icon));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));
    expect(tester.hasRunningAnimations, isTrue);

    await tester.pumpAndSettle();
    await tester.pump();
    transformWidget = tester.firstWidget(findTransform);

    // Icon has rotated 180 degrees.
    expect(transformWidget.transform.getRotation()[0], -1.0);
    expect(transformWidget.transform.getRotation()[4], -1.0);

    await tester.tap(find.byType(Icon));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 10));
    expect(tester.hasRunningAnimations, isTrue);

    await tester.pumpAndSettle();
    await tester.pump();
    transformWidget = tester.firstWidget(findTransform);

    // Icon has rotated 180 degrees back to the original position.
    expect(transformWidget.transform.getRotation()[0], 1.0);
    expect(transformWidget.transform.getRotation()[4], 1.0);
  });

  // Regression test for https://github.com/flutter/flutter/issues/25801.
  testWidgets('UserAccountsDrawerHeader icon does not rotate after setState', (
    WidgetTester tester,
  ) async {
    late StateSetter testSetState;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: StatefulBuilder(
            builder: (BuildContext context, StateSetter setState) {
              testSetState = setState;
              return UserAccountsDrawerHeader(
                onDetailsPressed: () {},
                accountName: const Text('name'),
                accountEmail: const Text('email'),
              );
            },
          ),
        ),
      ),
    );

    Transform transformWidget = tester.firstWidget(findTransform);

    // Icon is right side up.
    expect(transformWidget.transform.getRotation()[0], 1.0);
    expect(transformWidget.transform.getRotation()[4], 1.0);

    testSetState(() {});
    await tester.pump(const Duration(milliseconds: 10));
    expect(tester.hasRunningAnimations, isFalse);

    expect(await tester.pumpAndSettle(), 1);
    transformWidget = tester.firstWidget(findTransform);

    // Icon has not rotated.
    expect(transformWidget.transform.getRotation()[0], 1.0);
    expect(transformWidget.transform.getRotation()[4], 1.0);
  });

  testWidgets('UserAccountsDrawerHeader icon rotation test speeeeeedy', (
    WidgetTester tester,
  ) async {
    await pumpTestWidget(tester);
    Transform transformWidget = tester.firstWidget(findTransform);

    // Icon is right side up.
    expect(transformWidget.transform.getRotation()[0], 1.0);
    expect(transformWidget.transform.getRotation()[4], 1.0);

    // Icon starts to rotate down.
    await tester.tap(find.byType(Icon));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(tester.hasRunningAnimations, isTrue);

    // Icon starts to rotate up mid animation.
    await tester.tap(find.byType(Icon));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(tester.hasRunningAnimations, isTrue);

    // Icon starts to rotate down again still mid animation.
    await tester.tap(find.byType(Icon));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(tester.hasRunningAnimations, isTrue);

    // Icon starts to rotate up to its original position mid animation.
    await tester.tap(find.byType(Icon));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 100));
    expect(tester.hasRunningAnimations, isTrue);

    await tester.pumpAndSettle();
    await tester.pump();
    transformWidget = tester.firstWidget(findTransform);

    // Icon has rotated 180 degrees back to the original position.
    expect(transformWidget.transform.getRotation()[0], 1.0);
    expect(transformWidget.transform.getRotation()[4], 1.0);
  });

  testWidgets('UserAccountsDrawerHeader icon color changes', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: UserAccountsDrawerHeader(
            onDetailsPressed: () {},
            accountName: const Text('name'),
            accountEmail: const Text('email'),
          ),
        ),
      ),
    );

    Icon iconWidget = tester.firstWidget(find.byType(Icon));
    // Default icon color is white.
    expect(iconWidget.color, Colors.white);

    const Color arrowColor = Colors.red;
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: UserAccountsDrawerHeader(
            onDetailsPressed: () {},
            accountName: const Text('name'),
            accountEmail: const Text('email'),
            arrowColor: arrowColor,
          ),
        ),
      ),
    );

    iconWidget = tester.firstWidget(find.byType(Icon));
    expect(iconWidget.color, arrowColor);
  });

  testWidgets('UserAccountsDrawerHeader null parameters LTR', (WidgetTester tester) async {
    Widget buildFrame({
      Widget? currentAccountPicture,
      List<Widget>? otherAccountsPictures,
      Widget? accountName,
      Widget? accountEmail,
      VoidCallback? onDetailsPressed,
      EdgeInsets? margin,
    }) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: UserAccountsDrawerHeader(
              currentAccountPicture: currentAccountPicture,
              otherAccountsPictures: otherAccountsPictures,
              accountName: accountName,
              accountEmail: accountEmail,
              onDetailsPressed: onDetailsPressed,
              margin: margin,
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    final RenderBox box = tester.renderObject(find.byType(UserAccountsDrawerHeader));
    expect(box.size.height, equals(160.0 + 1.0)); // height + bottom edge)
    expect(find.byType(Icon), findsNothing);

    await tester.pumpWidget(buildFrame(onDetailsPressed: () {}));
    expect(find.byType(Icon), findsOneWidget);

    await tester.pumpWidget(
      buildFrame(accountName: const Text('accountName'), onDetailsPressed: () {}),
    );
    expect(tester.getCenter(find.text('accountName')).dy, tester.getCenter(find.byType(Icon)).dy);
    expect(
      tester.getCenter(find.text('accountName')).dx,
      lessThan(tester.getCenter(find.byType(Icon)).dx),
    );

    await tester.pumpWidget(
      buildFrame(accountEmail: const Text('accountEmail'), onDetailsPressed: () {}),
    );
    expect(tester.getCenter(find.text('accountEmail')).dy, tester.getCenter(find.byType(Icon)).dy);
    expect(
      tester.getCenter(find.text('accountEmail')).dx,
      lessThan(tester.getCenter(find.byType(Icon)).dx),
    );

    await tester.pumpWidget(
      buildFrame(
        accountName: const Text('accountName'),
        accountEmail: const Text('accountEmail'),
        onDetailsPressed: () {},
      ),
    );
    expect(tester.getCenter(find.text('accountEmail')).dy, tester.getCenter(find.byType(Icon)).dy);
    expect(
      tester.getCenter(find.text('accountEmail')).dx,
      lessThan(tester.getCenter(find.byType(Icon)).dx),
    );
    expect(
      tester.getBottomLeft(find.text('accountEmail')).dy,
      greaterThan(tester.getBottomLeft(find.text('accountName')).dy),
    );
    expect(
      tester.getBottomLeft(find.text('accountEmail')).dx,
      tester.getBottomLeft(find.text('accountName')).dx,
    );

    await tester.pumpWidget(
      buildFrame(currentAccountPicture: const CircleAvatar(child: Text('A'))),
    );
    expect(find.text('A'), findsOneWidget);

    await tester.pumpWidget(
      buildFrame(otherAccountsPictures: <Widget>[const CircleAvatar(child: Text('A'))]),
    );
    expect(find.text('A'), findsOneWidget);

    const Key avatarA = Key('A');
    await tester.pumpWidget(
      buildFrame(
        currentAccountPicture: const CircleAvatar(key: avatarA, child: Text('A')),
        accountName: const Text('accountName'),
      ),
    );
    expect(
      tester.getBottomLeft(find.byKey(avatarA)).dx,
      tester.getBottomLeft(find.text('accountName')).dx,
    );
    expect(
      tester.getBottomLeft(find.text('accountName')).dy,
      greaterThan(tester.getBottomLeft(find.byKey(avatarA)).dy),
    );
  });

  testWidgets('UserAccountsDrawerHeader null parameters RTL', (WidgetTester tester) async {
    Widget buildFrame({
      Widget? currentAccountPicture,
      List<Widget>? otherAccountsPictures,
      Widget? accountName,
      Widget? accountEmail,
      VoidCallback? onDetailsPressed,
      EdgeInsets? margin,
    }) {
      return MaterialApp(
        home: Directionality(
          textDirection: TextDirection.rtl,
          child: Material(
            child: Center(
              child: UserAccountsDrawerHeader(
                currentAccountPicture: currentAccountPicture,
                otherAccountsPictures: otherAccountsPictures,
                accountName: accountName,
                accountEmail: accountEmail,
                onDetailsPressed: onDetailsPressed,
                margin: margin,
              ),
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildFrame());
    final RenderBox box = tester.renderObject(find.byType(UserAccountsDrawerHeader));
    expect(box.size.height, equals(160.0 + 1.0)); // height + bottom edge)
    expect(find.byType(Icon), findsNothing);

    await tester.pumpWidget(buildFrame(onDetailsPressed: () {}));
    expect(find.byType(Icon), findsOneWidget);

    await tester.pumpWidget(
      buildFrame(accountName: const Text('accountName'), onDetailsPressed: () {}),
    );
    expect(tester.getCenter(find.text('accountName')).dy, tester.getCenter(find.byType(Icon)).dy);
    expect(
      tester.getCenter(find.text('accountName')).dx,
      greaterThan(tester.getCenter(find.byType(Icon)).dx),
    );

    await tester.pumpWidget(
      buildFrame(accountEmail: const Text('accountEmail'), onDetailsPressed: () {}),
    );
    expect(tester.getCenter(find.text('accountEmail')).dy, tester.getCenter(find.byType(Icon)).dy);
    expect(
      tester.getCenter(find.text('accountEmail')).dx,
      greaterThan(tester.getCenter(find.byType(Icon)).dx),
    );

    await tester.pumpWidget(
      buildFrame(
        accountName: const Text('accountName'),
        accountEmail: const Text('accountEmail'),
        onDetailsPressed: () {},
      ),
    );
    expect(tester.getCenter(find.text('accountEmail')).dy, tester.getCenter(find.byType(Icon)).dy);
    expect(
      tester.getCenter(find.text('accountEmail')).dx,
      greaterThan(tester.getCenter(find.byType(Icon)).dx),
    );
    expect(
      tester.getBottomLeft(find.text('accountEmail')).dy,
      greaterThan(tester.getBottomLeft(find.text('accountName')).dy),
    );
    expect(
      tester.getBottomRight(find.text('accountEmail')).dx,
      tester.getBottomRight(find.text('accountName')).dx,
    );

    await tester.pumpWidget(
      buildFrame(currentAccountPicture: const CircleAvatar(child: Text('A'))),
    );
    expect(find.text('A'), findsOneWidget);

    await tester.pumpWidget(
      buildFrame(otherAccountsPictures: <Widget>[const CircleAvatar(child: Text('A'))]),
    );
    expect(find.text('A'), findsOneWidget);

    const Key avatarA = Key('A');
    await tester.pumpWidget(
      buildFrame(
        currentAccountPicture: const CircleAvatar(key: avatarA, child: Text('A')),
        accountName: const Text('accountName'),
      ),
    );
    expect(
      tester.getBottomRight(find.byKey(avatarA)).dx,
      tester.getBottomRight(find.text('accountName')).dx,
    );
    expect(
      tester.getBottomLeft(find.text('accountName')).dy,
      greaterThan(tester.getBottomLeft(find.byKey(avatarA)).dy),
    );
  });

  testWidgets('UserAccountsDrawerHeader provides semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await pumpTestWidget(tester);

    expect(
      semantics,
      hasSemantics(
        TestSemantics(
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.isFocusable],
                          label: 'Signed in\nname\nemail',
                          textDirection: TextDirection.ltr,
                          actions: <SemanticsAction>[SemanticsAction.focus],
                          children: <TestSemantics>[
                            TestSemantics(label: r'B', textDirection: TextDirection.ltr),
                            TestSemantics(label: r'C', textDirection: TextDirection.ltr),
                            TestSemantics(label: r'D', textDirection: TextDirection.ltr),
                            TestSemantics(
                              flags: <SemanticsFlag>[SemanticsFlag.isButton],
                              actions: <SemanticsAction>[SemanticsAction.tap],
                              label: r'Show accounts',
                              textDirection: TextDirection.ltr,
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
        ignoreId: true,
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });

  testWidgets('alternative account selectors have sufficient tap targets', (
    WidgetTester tester,
  ) async {
    final SemanticsHandle handle = tester.ensureSemantics();
    await pumpTestWidget(tester);

    expect(
      tester.getSemantics(find.text('B')),
      matchesSemantics(label: 'B', size: const Size(48.0, 48.0)),
    );

    expect(
      tester.getSemantics(find.text('C')),
      matchesSemantics(label: 'C', size: const Size(48.0, 48.0)),
    );

    expect(
      tester.getSemantics(find.text('D')),
      matchesSemantics(label: 'D', size: const Size(48.0, 48.0)),
    );
    handle.dispose();
  });

  testWidgets('UserAccountsDrawerHeader provides semantics with missing properties', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await pumpTestWidget(
      tester,
      withEmail: false,
      withName: false,
      withOnDetailsPressedHandler: false,
    );

    expect(
      semantics,
      hasSemantics(
        TestSemantics(
          children: <TestSemantics>[
            TestSemantics(
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute],
                      children: <TestSemantics>[
                        TestSemantics(
                          label: 'Signed in',
                          textDirection: TextDirection.ltr,
                          children: <TestSemantics>[
                            TestSemantics(label: r'B', textDirection: TextDirection.ltr),
                            TestSemantics(label: r'C', textDirection: TextDirection.ltr),
                            TestSemantics(label: r'D', textDirection: TextDirection.ltr),
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
        ignoreId: true,
        ignoreTransform: true,
        ignoreRect: true,
      ),
    );

    semantics.dispose();
  });
}
