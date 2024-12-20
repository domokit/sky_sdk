// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' show DisplayFeature, DisplayFeatureState, DisplayFeatureType, SemanticsFlag;

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

import '../widgets/feedback_tester.dart';
import '../widgets/semantics_tester.dart';

void main() {
  testWidgets('Navigator.push works within a PopupMenuButton', (WidgetTester tester) async {
    final Key targetKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        routes: <String, WidgetBuilder>{
          '/next': (BuildContext context) {
            return const Text('Next');
          },
        },
        home: Material(
          child: Center(
            child: Builder(
              key: targetKey,
              builder: (BuildContext context) {
                return PopupMenuButton<int>(
                  onSelected: (int value) {
                    Navigator.pushNamed(context, '/next');
                  },
                  itemBuilder: (BuildContext context) {
                    return <PopupMenuItem<int>>[
                      const PopupMenuItem<int>(value: 1, child: Text('One')),
                    ];
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(targetKey));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1)); // finish the menu animation

    expect(find.text('One'), findsOneWidget);
    expect(find.text('Next'), findsNothing);

    await tester.tap(find.text('One'));
    await tester.pump(); // return the future
    await tester.pump(); // start the navigation
    await tester.pump(const Duration(seconds: 1)); // end the navigation

    expect(find.text('One'), findsNothing);
    expect(find.text('Next'), findsOneWidget);
  });

  testWidgets('PopupMenuButton calls onOpened callback when the menu is opened', (
    WidgetTester tester,
  ) async {
    int opens = 0;
    late BuildContext popupContext;
    final Key noItemsKey = UniqueKey();
    final Key noCallbackKey = UniqueKey();
    final Key withCallbackKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              PopupMenuButton<int>(
                key: noItemsKey,
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[];
                },
                onOpened: () => opens++,
              ),
              PopupMenuButton<int>(
                key: noCallbackKey,
                itemBuilder: (BuildContext context) {
                  popupContext = context;
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                  ];
                },
              ),
              PopupMenuButton<int>(
                key: withCallbackKey,
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me, too!')),
                  ];
                },
                onOpened: () => opens++,
              ),
            ],
          ),
        ),
      ),
    );

    // Make sure callback is not called when the menu is not shown
    await tester.tap(find.byKey(noItemsKey));
    await tester.pump();
    expect(opens, equals(0));

    // Make sure everything works if no callback is provided
    await tester.tap(find.byKey(noCallbackKey));
    await tester.pump();
    expect(opens, equals(0));

    // Close the opened menu
    Navigator.of(popupContext).pop();
    await tester.pump();

    // Make sure callback is called when the button is tapped
    await tester.tap(find.byKey(withCallbackKey));
    await tester.pump();
    expect(opens, equals(1));
  });

  testWidgets('PopupMenuButton calls onCanceled callback when an item is not selected', (
    WidgetTester tester,
  ) async {
    int cancels = 0;
    late BuildContext popupContext;
    final Key noCallbackKey = UniqueKey();
    final Key withCallbackKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              PopupMenuButton<int>(
                key: noCallbackKey,
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                  ];
                },
              ),
              PopupMenuButton<int>(
                key: withCallbackKey,
                onCanceled: () => cancels++,
                itemBuilder: (BuildContext context) {
                  popupContext = context;
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me, too!')),
                  ];
                },
              ),
            ],
          ),
        ),
      ),
    );

    // Make sure everything works if no callback is provided
    await tester.tap(find.byKey(noCallbackKey));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.tapAt(Offset.zero);
    await tester.pump();
    expect(cancels, equals(0));

    // Make sure callback is called when a non-selection tap occurs
    await tester.tap(find.byKey(withCallbackKey));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    await tester.tapAt(Offset.zero);
    await tester.pump();
    expect(cancels, equals(1));

    // Make sure callback is called when back navigation occurs
    await tester.tap(find.byKey(withCallbackKey));
    await tester.pump();
    await tester.pump(const Duration(seconds: 1));
    Navigator.of(popupContext).pop();
    await tester.pump();
    expect(cancels, equals(2));
  });

  testWidgets(
    'Disabled PopupMenuButton will not call itemBuilder, onOpened, onSelected or onCanceled',
    (WidgetTester tester) async {
      final GlobalKey popupButtonKey = GlobalKey();
      bool itemBuilderCalled = false;
      bool onOpenedCalled = false;
      bool onSelectedCalled = false;
      bool onCanceledCalled = false;

      Widget buildApp({bool directional = false}) {
        return MaterialApp(
          home: Builder(
            builder: (BuildContext context) {
              return MediaQuery(
                data: MediaQuery.of(context).copyWith(navigationMode: NavigationMode.directional),
                child: Material(
                  child: Column(
                    children: <Widget>[
                      PopupMenuButton<int>(
                        enabled: false,
                        child: Text('Tap Me', key: popupButtonKey),
                        itemBuilder: (BuildContext context) {
                          itemBuilderCalled = true;
                          return <PopupMenuEntry<int>>[
                            const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                          ];
                        },
                        onOpened: () => onOpenedCalled = true,
                        onSelected: (int selected) => onSelectedCalled = true,
                        onCanceled: () => onCanceledCalled = true,
                      ),
                    ],
                  ),
                ),
              );
            },
          ),
        );
      }

      await tester.pumpWidget(buildApp());

      // Try to bring up the popup menu and select the first item from it
      await tester.tap(find.byKey(popupButtonKey));
      await tester.pumpAndSettle();
      await tester.tap(find.byKey(popupButtonKey));
      await tester.pumpAndSettle();
      expect(itemBuilderCalled, isFalse);
      expect(onOpenedCalled, isFalse);
      expect(onSelectedCalled, isFalse);

      // Try to bring up the popup menu and tap outside it to cancel the menu
      await tester.tap(find.byKey(popupButtonKey));
      await tester.pumpAndSettle();
      await tester.tapAt(Offset.zero);
      await tester.pumpAndSettle();
      expect(itemBuilderCalled, isFalse);
      expect(onOpenedCalled, isFalse);
      expect(onCanceledCalled, isFalse);

      // Test again, with directional navigation mode and after focusing the button.
      await tester.pumpWidget(buildApp(directional: true));

      // Try to bring up the popup menu and select the first item from it
      Focus.of(popupButtonKey.currentContext!).requestFocus();
      await tester.pumpAndSettle();
      await tester.tap(find.byKey(popupButtonKey));
      await tester.pumpAndSettle();
      await tester.tap(find.byKey(popupButtonKey));
      await tester.pumpAndSettle();
      expect(itemBuilderCalled, isFalse);
      expect(onOpenedCalled, isFalse);
      expect(onSelectedCalled, isFalse);

      // Try to bring up the popup menu and tap outside it to cancel the menu
      await tester.tap(find.byKey(popupButtonKey));
      await tester.pumpAndSettle();
      await tester.tapAt(Offset.zero);
      await tester.pumpAndSettle();
      expect(itemBuilderCalled, isFalse);
      expect(onOpenedCalled, isFalse);
      expect(onCanceledCalled, isFalse);
    },
  );

  testWidgets('disabled PopupMenuButton is not focusable', (WidgetTester tester) async {
    final Key popupButtonKey = UniqueKey();
    final GlobalKey childKey = GlobalKey();
    bool itemBuilderCalled = false;
    bool onOpenedCalled = false;
    bool onSelectedCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              PopupMenuButton<int>(
                key: popupButtonKey,
                enabled: false,
                child: Container(key: childKey),
                itemBuilder: (BuildContext context) {
                  itemBuilderCalled = true;
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                  ];
                },
                onOpened: () => onOpenedCalled = true,
                onSelected: (int selected) => onSelectedCalled = true,
              ),
            ],
          ),
        ),
      ),
    );
    Focus.of(childKey.currentContext!).requestFocus();
    await tester.pump();

    expect(Focus.of(childKey.currentContext!).hasPrimaryFocus, isFalse);
    expect(itemBuilderCalled, isFalse);
    expect(onOpenedCalled, isFalse);
    expect(onSelectedCalled, isFalse);
  });

  testWidgets('Disabled PopupMenuButton is focusable with directional navigation', (
    WidgetTester tester,
  ) async {
    final Key popupButtonKey = UniqueKey();
    final GlobalKey childKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Builder(
          builder: (BuildContext context) {
            return MediaQuery(
              data: MediaQuery.of(context).copyWith(navigationMode: NavigationMode.directional),
              child: Material(
                child: Column(
                  children: <Widget>[
                    PopupMenuButton<int>(
                      key: popupButtonKey,
                      enabled: false,
                      child: Container(key: childKey),
                      itemBuilder: (BuildContext context) {
                        return <PopupMenuEntry<int>>[
                          const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                        ];
                      },
                      onSelected: (int selected) {},
                    ),
                  ],
                ),
              ),
            );
          },
        ),
      ),
    );
    Focus.of(childKey.currentContext!).requestFocus();
    await tester.pump();

    expect(Focus.of(childKey.currentContext!).hasPrimaryFocus, isTrue);
  });

  testWidgets('PopupMenuItem onTap callback is called when defined', (WidgetTester tester) async {
    final List<int> menuItemTapCounters = <int>[0, 0];

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: RepaintBoundary(
            child: PopupMenuButton<void>(
              child: const Text('Actions'),
              itemBuilder:
                  (BuildContext context) => <PopupMenuItem<void>>[
                    PopupMenuItem<void>(
                      child: const Text('First option'),
                      onTap: () {
                        menuItemTapCounters[0] += 1;
                      },
                    ),
                    PopupMenuItem<void>(
                      child: const Text('Second option'),
                      onTap: () {
                        menuItemTapCounters[1] += 1;
                      },
                    ),
                    const PopupMenuItem<void>(child: Text('Option without onTap')),
                  ],
            ),
          ),
        ),
      ),
    );

    // Tap the first time
    await tester.tap(find.text('Actions'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('First option'));
    await tester.pumpAndSettle();
    expect(menuItemTapCounters, <int>[1, 0]);

    // Tap the item again
    await tester.tap(find.text('Actions'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('First option'));
    await tester.pumpAndSettle();
    expect(menuItemTapCounters, <int>[2, 0]);

    // Tap a different item
    await tester.tap(find.text('Actions'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Second option'));
    await tester.pumpAndSettle();
    expect(menuItemTapCounters, <int>[2, 1]);

    // Tap an item without onTap
    await tester.tap(find.text('Actions'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Option without onTap'));
    await tester.pumpAndSettle();
    expect(menuItemTapCounters, <int>[2, 1]);
  });

  testWidgets('PopupMenuItem can have both onTap and value', (WidgetTester tester) async {
    final List<int> menuItemTapCounters = <int>[0, 0];
    String? selected;

    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: RepaintBoundary(
            child: PopupMenuButton<String>(
              child: const Text('Actions'),
              onSelected: (String value) {
                selected = value;
              },
              itemBuilder:
                  (BuildContext context) => <PopupMenuItem<String>>[
                    PopupMenuItem<String>(
                      value: 'first',
                      child: const Text('First option'),
                      onTap: () {
                        menuItemTapCounters[0] += 1;
                      },
                    ),
                    PopupMenuItem<String>(
                      value: 'second',
                      child: const Text('Second option'),
                      onTap: () {
                        menuItemTapCounters[1] += 1;
                      },
                    ),
                    const PopupMenuItem<String>(
                      value: 'third',
                      child: Text('Option without onTap'),
                    ),
                  ],
            ),
          ),
        ),
      ),
    );

    // Tap the first item
    await tester.tap(find.text('Actions'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('First option'));
    await tester.pumpAndSettle();
    expect(menuItemTapCounters, <int>[1, 0]);
    expect(selected, 'first');

    // Tap the item again
    await tester.tap(find.text('Actions'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('First option'));
    await tester.pumpAndSettle();
    expect(menuItemTapCounters, <int>[2, 0]);
    expect(selected, 'first');

    // Tap a different item
    await tester.tap(find.text('Actions'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Second option'));
    await tester.pumpAndSettle();
    expect(menuItemTapCounters, <int>[2, 1]);
    expect(selected, 'second');

    // Tap an item without onTap
    await tester.tap(find.text('Actions'));
    await tester.pumpAndSettle();
    await tester.tap(find.text('Option without onTap'));
    await tester.pumpAndSettle();
    expect(menuItemTapCounters, <int>[2, 1]);
    expect(selected, 'third');
  });

  testWidgets('PopupMenuItem is only focusable when enabled', (WidgetTester tester) async {
    final Key popupButtonKey = UniqueKey();
    final GlobalKey childKey = GlobalKey();
    bool itemBuilderCalled = false;

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              PopupMenuButton<int>(
                key: popupButtonKey,
                itemBuilder: (BuildContext context) {
                  itemBuilderCalled = true;
                  return <PopupMenuEntry<int>>[
                    PopupMenuItem<int>(value: 1, child: Text('Tap me please!', key: childKey)),
                  ];
                },
              ),
            ],
          ),
        ),
      ),
    );

    // Open the popup to build and show the menu contents.
    await tester.tap(find.byKey(popupButtonKey));
    await tester.pumpAndSettle();
    final FocusNode childNode = Focus.of(childKey.currentContext!);
    // Now that the contents are shown, request focus on the child text.
    childNode.requestFocus();
    await tester.pumpAndSettle();
    expect(itemBuilderCalled, isTrue);

    // Make sure that the focus went where we expected it to.
    expect(childNode.hasPrimaryFocus, isTrue);
    itemBuilderCalled = false;

    // Close the popup.
    await tester.tap(find.byKey(popupButtonKey), warnIfMissed: false);
    await tester.pumpAndSettle();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              PopupMenuButton<int>(
                key: popupButtonKey,
                itemBuilder: (BuildContext context) {
                  itemBuilderCalled = true;
                  return <PopupMenuEntry<int>>[
                    PopupMenuItem<int>(
                      enabled: false,
                      value: 1,
                      child: Text('Tap me please!', key: childKey),
                    ),
                  ];
                },
              ),
            ],
          ),
        ),
      ),
    );
    await tester.pumpAndSettle();
    // Open the popup again to rebuild the contents with enabled == false.
    await tester.tap(find.byKey(popupButtonKey));
    await tester.pumpAndSettle();

    expect(itemBuilderCalled, isTrue);
    expect(Focus.of(childKey.currentContext!).hasPrimaryFocus, isFalse);
  });

  testWidgets('PopupMenuButton is horizontal on iOS', (WidgetTester tester) async {
    Widget build(TargetPlatform platform) {
      debugDefaultTargetPlatformOverride = platform;
      return MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            actions: <Widget>[
              PopupMenuButton<int>(
                itemBuilder: (BuildContext context) {
                  return <PopupMenuItem<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('One')),
                  ];
                },
              ),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(build(TargetPlatform.android));

    expect(find.byIcon(Icons.more_vert), findsOneWidget);
    expect(find.byIcon(Icons.more_horiz), findsNothing);

    await tester.pumpWidget(build(TargetPlatform.iOS));
    await tester.pumpAndSettle(); // Run theme change animation.

    expect(find.byIcon(Icons.more_vert), findsNothing);
    expect(find.byIcon(Icons.more_horiz), findsOneWidget);

    await tester.pumpWidget(build(TargetPlatform.macOS));
    await tester.pumpAndSettle(); // Run theme change animation.

    expect(find.byIcon(Icons.more_vert), findsNothing);
    expect(find.byIcon(Icons.more_horiz), findsOneWidget);

    debugDefaultTargetPlatformOverride = null;
  });

  group('PopupMenuButton with Icon', () {
    // Helper function to create simple and valid popup menus.
    List<PopupMenuItem<int>> simplePopupMenuItemBuilder(BuildContext context) {
      return <PopupMenuItem<int>>[const PopupMenuItem<int>(value: 1, child: Text('1'))];
    }

    testWidgets('PopupMenuButton fails when given both child and icon', (
      WidgetTester tester,
    ) async {
      expect(() {
        PopupMenuButton<int>(
          icon: const Icon(Icons.view_carousel),
          itemBuilder: simplePopupMenuItemBuilder,
          child: const Text('heyo'),
        );
      }, throwsAssertionError);
    });

    testWidgets('PopupMenuButton creates IconButton when given an icon', (
      WidgetTester tester,
    ) async {
      final PopupMenuButton<int> button = PopupMenuButton<int>(
        icon: const Icon(Icons.view_carousel),
        itemBuilder: simplePopupMenuItemBuilder,
      );

      await tester.pumpWidget(
        MaterialApp(home: Scaffold(appBar: AppBar(actions: <Widget>[button]))),
      );

      expect(find.byType(IconButton), findsOneWidget);
      expect(find.byIcon(Icons.view_carousel), findsOneWidget);
    });
  });

  testWidgets('PopupMenu positioning', (WidgetTester tester) async {
    final Widget testButton = PopupMenuButton<int>(
      itemBuilder: (BuildContext context) {
        return <PopupMenuItem<int>>[
          const PopupMenuItem<int>(value: 1, child: Text('AAA')),
          const PopupMenuItem<int>(value: 2, child: Text('BBB')),
          const PopupMenuItem<int>(value: 3, child: Text('CCC')),
        ];
      },
      child: const SizedBox(height: 100.0, width: 100.0, child: Text('XXX')),
    );

    bool popupMenu(Widget widget) => widget.runtimeType.toString() == '_PopupMenu<int?>';

    Future<void> openMenu(TextDirection textDirection, Alignment alignment) async {
      return TestAsyncUtils.guard<void>(() async {
        await tester.pumpWidget(Container()); // reset in case we had a menu up already
        await tester.pumpWidget(
          TestApp(
            textDirection: textDirection,
            child: Align(alignment: alignment, child: testButton),
          ),
        );
        await tester.tap(find.text('XXX'));
        await tester.pump();
      });
    }

    Future<void> testPositioningDown(
      WidgetTester tester,
      TextDirection textDirection,
      Alignment alignment,
      TextDirection growthDirection,
      Rect startRect,
    ) {
      return TestAsyncUtils.guard<void>(() async {
        await openMenu(textDirection, alignment);
        Rect rect = tester.getRect(find.byWidgetPredicate(popupMenu));
        expect(rect, startRect);
        bool doneVertically = false;
        bool doneHorizontally = false;
        do {
          await tester.pump(const Duration(milliseconds: 20));
          final Rect newRect = tester.getRect(find.byWidgetPredicate(popupMenu));
          expect(newRect.top, rect.top);
          if (doneVertically) {
            expect(newRect.bottom, rect.bottom);
          } else {
            if (newRect.bottom == rect.bottom) {
              doneVertically = true;
            } else {
              expect(newRect.bottom, greaterThan(rect.bottom));
            }
          }
          switch (growthDirection) {
            case TextDirection.rtl:
              expect(newRect.right, rect.right);
              if (doneHorizontally) {
                expect(newRect.left, rect.left);
              } else {
                if (newRect.left == rect.left) {
                  doneHorizontally = true;
                } else {
                  expect(newRect.left, lessThan(rect.left));
                }
              }
            case TextDirection.ltr:
              expect(newRect.left, rect.left);
              if (doneHorizontally) {
                expect(newRect.right, rect.right);
              } else {
                if (newRect.right == rect.right) {
                  doneHorizontally = true;
                } else {
                  expect(newRect.right, greaterThan(rect.right));
                }
              }
          }
          rect = newRect;
        } while (tester.binding.hasScheduledFrame);
      });
    }

    Future<void> testPositioningDownThenUp(
      WidgetTester tester,
      TextDirection textDirection,
      Alignment alignment,
      TextDirection growthDirection,
      Rect startRect,
    ) {
      return TestAsyncUtils.guard<void>(() async {
        await openMenu(textDirection, alignment);
        Rect rect = tester.getRect(find.byWidgetPredicate(popupMenu));
        expect(rect, startRect);
        int verticalStage = 0; // 0=down, 1=up, 2=done
        bool doneHorizontally = false;
        do {
          await tester.pump(const Duration(milliseconds: 20));
          final Rect newRect = tester.getRect(find.byWidgetPredicate(popupMenu));
          switch (verticalStage) {
            case 0:
              if (newRect.top < rect.top) {
                verticalStage = 1;
                expect(newRect.bottom, greaterThanOrEqualTo(rect.bottom));
                break;
              }
              expect(newRect.top, rect.top);
              expect(newRect.bottom, greaterThan(rect.bottom));
            case 1:
              if (newRect.top == rect.top) {
                verticalStage = 2;
                expect(newRect.bottom, rect.bottom);
                break;
              }
              expect(newRect.top, lessThan(rect.top));
              expect(newRect.bottom, rect.bottom);
            case 2:
              expect(newRect.bottom, rect.bottom);
              expect(newRect.top, rect.top);
            default:
              assert(false);
          }
          switch (growthDirection) {
            case TextDirection.rtl:
              expect(newRect.right, rect.right);
              if (doneHorizontally) {
                expect(newRect.left, rect.left);
              } else {
                if (newRect.left == rect.left) {
                  doneHorizontally = true;
                } else {
                  expect(newRect.left, lessThan(rect.left));
                }
              }
            case TextDirection.ltr:
              expect(newRect.left, rect.left);
              if (doneHorizontally) {
                expect(newRect.right, rect.right);
              } else {
                if (newRect.right == rect.right) {
                  doneHorizontally = true;
                } else {
                  expect(newRect.right, greaterThan(rect.right));
                }
              }
          }
          rect = newRect;
        } while (tester.binding.hasScheduledFrame);
      });
    }

    await testPositioningDown(
      tester,
      TextDirection.ltr,
      Alignment.topRight,
      TextDirection.rtl,
      const Rect.fromLTWH(792.0, 8.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.rtl,
      Alignment.topRight,
      TextDirection.rtl,
      const Rect.fromLTWH(792.0, 8.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.ltr,
      Alignment.topLeft,
      TextDirection.ltr,
      const Rect.fromLTWH(8.0, 8.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.rtl,
      Alignment.topLeft,
      TextDirection.ltr,
      const Rect.fromLTWH(8.0, 8.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.ltr,
      Alignment.topCenter,
      TextDirection.ltr,
      const Rect.fromLTWH(350.0, 8.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.rtl,
      Alignment.topCenter,
      TextDirection.rtl,
      const Rect.fromLTWH(450.0, 8.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.ltr,
      Alignment.centerRight,
      TextDirection.rtl,
      const Rect.fromLTWH(792.0, 250.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.rtl,
      Alignment.centerRight,
      TextDirection.rtl,
      const Rect.fromLTWH(792.0, 250.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.ltr,
      Alignment.centerLeft,
      TextDirection.ltr,
      const Rect.fromLTWH(8.0, 250.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.rtl,
      Alignment.centerLeft,
      TextDirection.ltr,
      const Rect.fromLTWH(8.0, 250.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.ltr,
      Alignment.center,
      TextDirection.ltr,
      const Rect.fromLTWH(350.0, 250.0, 0.0, 0.0),
    );
    await testPositioningDown(
      tester,
      TextDirection.rtl,
      Alignment.center,
      TextDirection.rtl,
      const Rect.fromLTWH(450.0, 250.0, 0.0, 0.0),
    );
    await testPositioningDownThenUp(
      tester,
      TextDirection.ltr,
      Alignment.bottomRight,
      TextDirection.rtl,
      const Rect.fromLTWH(792.0, 500.0, 0.0, 0.0),
    );
    await testPositioningDownThenUp(
      tester,
      TextDirection.rtl,
      Alignment.bottomRight,
      TextDirection.rtl,
      const Rect.fromLTWH(792.0, 500.0, 0.0, 0.0),
    );
    await testPositioningDownThenUp(
      tester,
      TextDirection.ltr,
      Alignment.bottomLeft,
      TextDirection.ltr,
      const Rect.fromLTWH(8.0, 500.0, 0.0, 0.0),
    );
    await testPositioningDownThenUp(
      tester,
      TextDirection.rtl,
      Alignment.bottomLeft,
      TextDirection.ltr,
      const Rect.fromLTWH(8.0, 500.0, 0.0, 0.0),
    );
    await testPositioningDownThenUp(
      tester,
      TextDirection.ltr,
      Alignment.bottomCenter,
      TextDirection.ltr,
      const Rect.fromLTWH(350.0, 500.0, 0.0, 0.0),
    );
    await testPositioningDownThenUp(
      tester,
      TextDirection.rtl,
      Alignment.bottomCenter,
      TextDirection.rtl,
      const Rect.fromLTWH(450.0, 500.0, 0.0, 0.0),
    );
  });

  testWidgets('PopupMenu positioning inside nested Overlay', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    late final OverlayEntry entry;
    addTearDown(
      () =>
          entry
            ..remove()
            ..dispose(),
    );

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Example')),
          body: Padding(
            padding: const EdgeInsets.all(8.0),
            child: Overlay(
              initialEntries: <OverlayEntry>[
                entry = OverlayEntry(
                  builder:
                      (_) => Center(
                        child: PopupMenuButton<int>(
                          key: buttonKey,
                          itemBuilder:
                              (_) => <PopupMenuItem<int>>[
                                const PopupMenuItem<int>(value: 1, child: Text('Item 1')),
                                const PopupMenuItem<int>(value: 2, child: Text('Item 2')),
                              ],
                          child: const Text('Show Menu'),
                        ),
                      ),
                ),
              ],
            ),
          ),
        ),
      ),
    );

    final Finder buttonFinder = find.byKey(buttonKey);
    final Finder popupFinder = find.bySemanticsLabel('Popup menu');
    await tester.tap(buttonFinder);
    await tester.pumpAndSettle();

    final Offset buttonTopLeft = tester.getTopLeft(buttonFinder);
    expect(tester.getTopLeft(popupFinder), buttonTopLeft);
  });

  testWidgets('PopupMenu positioning inside nested Navigator', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Example')),
          body: Padding(
            padding: const EdgeInsets.all(8.0),
            child: Navigator(
              onGenerateRoute: (RouteSettings settings) {
                return MaterialPageRoute<dynamic>(
                  builder: (BuildContext context) {
                    return Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: Center(
                        child: PopupMenuButton<int>(
                          key: buttonKey,
                          itemBuilder:
                              (_) => <PopupMenuItem<int>>[
                                const PopupMenuItem<int>(value: 1, child: Text('Item 1')),
                                const PopupMenuItem<int>(value: 2, child: Text('Item 2')),
                              ],
                          child: const Text('Show Menu'),
                        ),
                      ),
                    );
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    final Finder buttonFinder = find.byKey(buttonKey);
    final Finder popupFinder = find.bySemanticsLabel('Popup menu');
    await tester.tap(buttonFinder);
    await tester.pumpAndSettle();

    final Offset buttonTopLeft = tester.getTopLeft(buttonFinder);
    expect(tester.getTopLeft(popupFinder), buttonTopLeft);
  });

  testWidgets('PopupMenu positioning inside nested Navigator when useRootNavigator', (
    WidgetTester tester,
  ) async {
    final Key buttonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(title: const Text('Example')),
          body: Padding(
            padding: const EdgeInsets.all(8.0),
            child: Navigator(
              onGenerateRoute: (RouteSettings settings) {
                return MaterialPageRoute<dynamic>(
                  builder: (BuildContext context) {
                    return Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: Center(
                        child: PopupMenuButton<int>(
                          key: buttonKey,
                          useRootNavigator: true,
                          itemBuilder:
                              (_) => <PopupMenuItem<int>>[
                                const PopupMenuItem<int>(value: 1, child: Text('Item 1')),
                                const PopupMenuItem<int>(value: 2, child: Text('Item 2')),
                              ],
                          child: const Text('Show Menu'),
                        ),
                      ),
                    );
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    final Finder buttonFinder = find.byKey(buttonKey);
    final Finder popupFinder = find.bySemanticsLabel('Popup menu');
    await tester.tap(buttonFinder);
    await tester.pumpAndSettle();

    final Offset buttonTopLeft = tester.getTopLeft(buttonFinder);
    expect(tester.getTopLeft(popupFinder), buttonTopLeft);
  });

  testWidgets('Popup menu with RouteSettings', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();
    const RouteSettings popupRoute = RouteSettings(name: '/popup');
    late RouteSettings currentRouteSetting;

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[
          _ClosureNavigatorObserver(
            onDidChange: (Route<dynamic> newRoute) {
              currentRouteSetting = newRoute.settings;
            },
          ),
        ],
        home: Scaffold(
          body: PopupMenuButton<int>(
            key: buttonKey,
            routeSettings: popupRoute,
            itemBuilder:
                (_) => <PopupMenuItem<int>>[
                  const PopupMenuItem<int>(value: 1, child: Text('Item 1')),
                  const PopupMenuItem<int>(value: 2, child: Text('Item 2')),
                ],
            child: const Text('Show Menu'),
          ),
        ),
      ),
    );

    final Finder buttonFinder = find.byKey(buttonKey);
    await tester.tap(buttonFinder);
    await tester.pumpAndSettle();

    expect(currentRouteSetting, popupRoute);
  });

  testWidgets('PopupMenu positioning around display features', (WidgetTester tester) async {
    final Key buttonKey = UniqueKey();

    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(
            size: Size(800, 600),
            displayFeatures: <DisplayFeature>[
              // A 20-pixel wide vertical display feature, similar to a foldable
              // with a visible hinge. Splits the display into two "virtual screens"
              // and the popup menu should never overlap the display feature.
              DisplayFeature(
                bounds: Rect.fromLTRB(390, 0, 410, 600),
                type: DisplayFeatureType.cutout,
                state: DisplayFeatureState.unknown,
              ),
            ],
          ),
          child: Scaffold(
            body: Navigator(
              onGenerateRoute: (RouteSettings settings) {
                return MaterialPageRoute<dynamic>(
                  builder: (BuildContext context) {
                    return Padding(
                      // Position the button in the top-right of the first "virtual screen"
                      padding: const EdgeInsets.only(right: 390.0),
                      child: Align(
                        alignment: Alignment.topRight,
                        child: PopupMenuButton<int>(
                          key: buttonKey,
                          itemBuilder:
                              (_) => <PopupMenuItem<int>>[
                                const PopupMenuItem<int>(value: 1, child: Text('Item 1')),
                                const PopupMenuItem<int>(value: 2, child: Text('Item 2')),
                              ],
                          child: const Text('Show Menu'),
                        ),
                      ),
                    );
                  },
                );
              },
            ),
          ),
        ),
      ),
    );

    final Finder buttonFinder = find.byKey(buttonKey);
    final Finder popupFinder = find.bySemanticsLabel('Popup menu');
    await tester.tap(buttonFinder);
    await tester.pumpAndSettle();

    // Since the display feature splits the display into 2 sub-screens, popup
    // menu should be positioned to fit in the first virtual screen, where the
    // originating button is.
    // The 8 pixels is [_kMenuScreenPadding].
    expect(tester.getTopRight(popupFinder), const Offset(390 - 8, 8));
  });

  testWidgets('PopupMenu removes MediaQuery padding', (WidgetTester tester) async {
    late BuildContext popupContext;

    await tester.pumpWidget(
      MaterialApp(
        home: MediaQuery(
          data: const MediaQueryData(padding: EdgeInsets.all(50.0)),
          child: Material(
            child: PopupMenuButton<int>(
              itemBuilder: (BuildContext context) {
                popupContext = context;
                return <PopupMenuItem<int>>[
                  PopupMenuItem<int>(
                    value: 1,
                    child: Builder(
                      builder: (BuildContext context) {
                        popupContext = context;
                        return const Text('AAA');
                      },
                    ),
                  ),
                ];
              },
              child: const SizedBox(height: 100.0, width: 100.0, child: Text('XXX')),
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('XXX'));

    await tester.pump();

    expect(MediaQuery.of(popupContext).padding, EdgeInsets.zero);
  });

  testWidgets('Popup Menu Offset Test', (WidgetTester tester) async {
    PopupMenuButton<int> buildMenuButton({Offset offset = Offset.zero}) {
      return PopupMenuButton<int>(
        offset: offset,
        itemBuilder: (BuildContext context) {
          return <PopupMenuItem<int>>[
            PopupMenuItem<int>(
              value: 1,
              child: Builder(
                builder: (BuildContext context) {
                  return const Text('AAA');
                },
              ),
            ),
          ];
        },
      );
    }

    // Popup a menu without any offset.
    await tester.pumpWidget(MaterialApp(home: Scaffold(body: Material(child: buildMenuButton()))));

    // Popup the menu.
    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    // Initial state, the menu start at Offset(8.0, 8.0), the 8 pixels is edge padding when offset.dx < 8.0.
    expect(
      tester.getTopLeft(
        find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_PopupMenu<int?>'),
      ),
      const Offset(8.0, 8.0),
    );

    // Collapse the menu.
    await tester.tap(find.byType(IconButton), warnIfMissed: false);
    await tester.pumpAndSettle();

    // Popup a new menu with Offset(50.0, 50.0).
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: Material(child: buildMenuButton(offset: const Offset(50.0, 50.0)))),
      ),
    );

    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    // This time the menu should start at Offset(50.0, 50.0), the padding only added when offset.dx < 8.0.
    expect(
      tester.getTopLeft(
        find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_PopupMenu<int?>'),
      ),
      const Offset(50.0, 50.0),
    );
  });

  testWidgets('Opened PopupMenu has correct semantics', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: PopupMenuButton<int>(
            itemBuilder: (BuildContext context) {
              return <PopupMenuItem<int>>[
                const PopupMenuItem<int>(value: 1, child: Text('1')),
                const PopupMenuItem<int>(value: 2, child: Text('2')),
                const PopupMenuItem<int>(value: 3, child: Text('3')),
                const PopupMenuItem<int>(value: 4, child: Text('4')),
                const PopupMenuItem<int>(value: 5, child: Text('5')),
              ];
            },
            child: const SizedBox(height: 100.0, width: 100.0, child: Text('XXX')),
          ),
        ),
      ),
    );
    await tester.tap(find.text('XXX'));
    await tester.pumpAndSettle();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute, SemanticsFlag.namesRoute],
                      label: 'Popup menu',
                      textDirection: TextDirection.ltr,
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                          children: <TestSemantics>[
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: '1',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: '2',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: '3',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: '4',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: '5',
                              textDirection: TextDirection.ltr,
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
                TestSemantics(
                  actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.dismiss],
                  label: 'Dismiss menu',
                  textDirection: TextDirection.ltr,
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

  testWidgets('PopupMenuItem merges the semantics of its descendants', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: PopupMenuButton<int>(
            itemBuilder: (BuildContext context) {
              return <PopupMenuItem<int>>[
                PopupMenuItem<int>(
                  value: 1,
                  child: Row(
                    children: <Widget>[
                      Semantics(child: const Text('test1')),
                      Semantics(child: const Text('test2')),
                    ],
                  ),
                ),
              ];
            },
            child: const SizedBox(height: 100.0, width: 100.0, child: Text('XXX')),
          ),
        ),
      ),
    );
    await tester.tap(find.text('XXX'));
    await tester.pumpAndSettle();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute, SemanticsFlag.namesRoute],
                      label: 'Popup menu',
                      textDirection: TextDirection.ltr,
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                          children: <TestSemantics>[
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: 'test1\ntest2',
                              textDirection: TextDirection.ltr,
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
                TestSemantics(
                  actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.dismiss],
                  label: 'Dismiss menu',
                  textDirection: TextDirection.ltr,
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

  testWidgets('Disabled PopupMenuItem has correct semantics', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/45044.
    final SemanticsTester semantics = SemanticsTester(tester);
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: PopupMenuButton<int>(
            itemBuilder: (BuildContext context) {
              return <PopupMenuItem<int>>[
                const PopupMenuItem<int>(value: 1, child: Text('1')),
                const PopupMenuItem<int>(value: 2, enabled: false, child: Text('2')),
                const PopupMenuItem<int>(value: 3, child: Text('3')),
                const PopupMenuItem<int>(value: 4, child: Text('4')),
                const PopupMenuItem<int>(value: 5, child: Text('5')),
              ];
            },
            child: const SizedBox(height: 100.0, width: 100.0, child: Text('XXX')),
          ),
        ),
      ),
    );
    await tester.tap(find.text('XXX'));
    await tester.pumpAndSettle();

    expect(
      semantics,
      hasSemantics(
        TestSemantics.root(
          children: <TestSemantics>[
            TestSemantics(
              textDirection: TextDirection.ltr,
              children: <TestSemantics>[
                TestSemantics(
                  children: <TestSemantics>[
                    TestSemantics(
                      flags: <SemanticsFlag>[SemanticsFlag.scopesRoute, SemanticsFlag.namesRoute],
                      label: 'Popup menu',
                      textDirection: TextDirection.ltr,
                      children: <TestSemantics>[
                        TestSemantics(
                          flags: <SemanticsFlag>[SemanticsFlag.hasImplicitScrolling],
                          children: <TestSemantics>[
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: '1',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                              ],
                              actions: <SemanticsAction>[],
                              label: '2',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: '3',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: '4',
                              textDirection: TextDirection.ltr,
                            ),
                            TestSemantics(
                              flags: <SemanticsFlag>[
                                SemanticsFlag.isButton,
                                SemanticsFlag.hasEnabledState,
                                SemanticsFlag.isEnabled,
                                SemanticsFlag.isFocusable,
                              ],
                              actions: <SemanticsAction>[
                                SemanticsAction.tap,
                                SemanticsAction.focus,
                              ],
                              label: '5',
                              textDirection: TextDirection.ltr,
                            ),
                          ],
                        ),
                      ],
                    ),
                  ],
                ),
                TestSemantics(
                  actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.dismiss],
                  label: 'Dismiss menu',
                  textDirection: TextDirection.ltr,
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

  testWidgets('PopupMenuButton PopupMenuDivider', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/27072

    late String selectedValue;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              onSelected: (String result) {
                selectedValue = result;
              },
              initialValue: '1',
              child: const Text('Menu Button'),
              itemBuilder:
                  (BuildContext context) => <PopupMenuEntry<String>>[
                    const PopupMenuItem<String>(value: '1', child: Text('1')),
                    const PopupMenuDivider(),
                    const PopupMenuItem<String>(value: '2', child: Text('2')),
                  ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.text('Menu Button'));
    await tester.pumpAndSettle();
    expect(find.text('1'), findsOneWidget);
    expect(find.byType(PopupMenuDivider), findsOneWidget);
    expect(find.text('2'), findsOneWidget);

    await tester.tap(find.text('1'));
    await tester.pumpAndSettle();
    expect(selectedValue, '1');

    await tester.tap(find.text('Menu Button'));
    await tester.pumpAndSettle();
    expect(find.text('1'), findsOneWidget);
    expect(find.byType(PopupMenuDivider), findsOneWidget);
    expect(find.text('2'), findsOneWidget);

    await tester.tap(find.text('2'));
    await tester.pumpAndSettle();
    expect(selectedValue, '2');
  });

  testWidgets('PopupMenuItem child height is a minimum, child is vertically centered', (
    WidgetTester tester,
  ) async {
    final Key popupMenuButtonKey = UniqueKey();
    final Type menuItemType = const PopupMenuItem<String>(child: Text('item')).runtimeType;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  // This menu item's height will be 48 because the default minimum height
                  // is 48 and the height of the text is less than 48.
                  const PopupMenuItem<String>(value: '0', child: Text('Item 0')),
                  // This menu item's height parameter specifies its minimum height. The
                  // overall height of the menu item will be 50 because the child's
                  // height 40, is less than 50.
                  const PopupMenuItem<String>(
                    height: 50,
                    value: '1',
                    child: SizedBox(height: 40, child: Text('Item 1')),
                  ),
                  // This menu item's height parameter specifies its minimum height, so the
                  // overall height of the menu item will be 75.
                  const PopupMenuItem<String>(
                    height: 75,
                    value: '2',
                    child: SizedBox(child: Text('Item 2')),
                  ),
                  // This menu item's height will be 100.
                  const PopupMenuItem<String>(
                    value: '3',
                    child: SizedBox(height: 100, child: Text('Item 3')),
                  ),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Show the menu
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // The menu items and their InkWells should have the expected vertical size
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 0')).height, 48);
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 1')).height, 50);
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 2')).height, 75);
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 3')).height, 100);
    expect(tester.getSize(find.widgetWithText(InkWell, 'Item 0')).height, 48);
    expect(tester.getSize(find.widgetWithText(InkWell, 'Item 1')).height, 50);
    expect(tester.getSize(find.widgetWithText(InkWell, 'Item 2')).height, 75);
    expect(tester.getSize(find.widgetWithText(InkWell, 'Item 3')).height, 100);

    // Menu item children which whose height is less than the PopupMenuItem
    // are vertically centered.
    expect(
      tester.getRect(find.widgetWithText(menuItemType, 'Item 0')).center.dy,
      tester.getRect(find.text('Item 0')).center.dy,
    );
    expect(
      tester.getRect(find.widgetWithText(menuItemType, 'Item 2')).center.dy,
      tester.getRect(find.text('Item 2')).center.dy,
    );
  });

  testWidgets('Material3 - PopupMenuItem default padding', (WidgetTester tester) async {
    final Key popupMenuButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: true),
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  const PopupMenuItem<String>(value: '0', enabled: false, child: Text('Item 0')),
                  const PopupMenuItem<String>(value: '1', child: Text('Item 1')),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    EdgeInsetsGeometry paddingFor(String text) {
      return tester
          .widget<Padding>(
            find.ancestor(
              of: find.widgetWithText(Align, 'Item 0'),
              matching: find.byKey(const Key('menu item padding')),
            ),
          )
          .padding;
    }

    expect(paddingFor('Item 0'), const EdgeInsets.symmetric(horizontal: 12.0));
    expect(paddingFor('Item 1'), const EdgeInsets.symmetric(horizontal: 12.0));
  });

  testWidgets('PopupMenu default padding', (WidgetTester tester) async {
    final Key popupMenuButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  const PopupMenuItem<String>(value: '0', enabled: false, child: Text('Item 0')),
                  const PopupMenuItem<String>(value: '1', child: Text('Item 1')),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pump(const Duration(milliseconds: 300));

    // Check popup menu padding.
    final SingleChildScrollView popupMenu = tester.widget<SingleChildScrollView>(
      find.byType(SingleChildScrollView),
    );
    expect(popupMenu.padding, const EdgeInsets.symmetric(vertical: 8.0));
  });

  testWidgets('Material2 - PopupMenuItem default padding', (WidgetTester tester) async {
    final Key popupMenuButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  const PopupMenuItem<String>(value: '0', enabled: false, child: Text('Item 0')),
                  const PopupMenuItem<String>(value: '1', child: Text('Item 1')),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    EdgeInsetsGeometry paddingFor(String text) {
      return tester
          .widget<Padding>(
            find.ancestor(
              of: find.widgetWithText(Align, 'Item 0'),
              matching: find.byKey(const Key('menu item padding')),
            ),
          )
          .padding;
    }

    expect(paddingFor('Item 0'), const EdgeInsets.symmetric(horizontal: 16.0));
    expect(paddingFor('Item 1'), const EdgeInsets.symmetric(horizontal: 16.0));
  });

  testWidgets('Material2 - PopupMenuItem default padding', (WidgetTester tester) async {
    final Key popupMenuButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  const PopupMenuItem<String>(value: '0', enabled: false, child: Text('Item 0')),
                  const PopupMenuItem<String>(value: '1', child: Text('Item 1')),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pump(const Duration(milliseconds: 300));

    // Check popup menu padding.
    final SingleChildScrollView popupMenu = tester.widget<SingleChildScrollView>(
      find.byType(SingleChildScrollView),
    );
    expect(popupMenu.padding, const EdgeInsets.symmetric(vertical: 8.0));
  });

  testWidgets('PopupMenuItem custom padding', (WidgetTester tester) async {
    final Key popupMenuButtonKey = UniqueKey();
    final Type menuItemType = const PopupMenuItem<String>(child: Text('item')).runtimeType;

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  const PopupMenuItem<String>(
                    padding: EdgeInsets.zero,
                    value: '0',
                    child: Text('Item 0'),
                  ),
                  const PopupMenuItem<String>(
                    padding: EdgeInsets.zero,
                    height: 0,
                    value: '0',
                    child: Text('Item 1'),
                  ),
                  const PopupMenuItem<String>(
                    padding: EdgeInsets.all(20),
                    value: '0',
                    child: Text('Item 2'),
                  ),
                  const PopupMenuItem<String>(
                    padding: EdgeInsets.all(20),
                    height: 100,
                    value: '0',
                    child: Text('Item 3'),
                  ),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Show the menu
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // The menu items and their InkWells should have the expected vertical size
    // given the interactions between heights and padding.
    expect(
      tester.getSize(find.widgetWithText(menuItemType, 'Item 0')).height,
      48,
    ); // Minimum interactive height (48)
    expect(
      tester.getSize(find.widgetWithText(menuItemType, 'Item 1')).height,
      16,
    ); // Height of text (16)
    expect(
      tester.getSize(find.widgetWithText(menuItemType, 'Item 2')).height,
      56,
    ); // Padding (20.0 + 20.0) + Height of text (16) = 56
    expect(
      tester.getSize(find.widgetWithText(menuItemType, 'Item 3')).height,
      100,
    ); // Height value of 100, since child (16) + padding (40) < 100

    EdgeInsetsGeometry paddingFor(String text) {
      final ConstrainedBox widget = tester.widget<ConstrainedBox>(
        find.ancestor(
          of: find.text(text),
          matching: find.byWidgetPredicate(
            (Widget widget) => widget is ConstrainedBox && widget.child is Padding,
          ),
        ),
      );
      return (widget.child! as Padding).padding;
    }

    expect(paddingFor('Item 0'), EdgeInsets.zero);
    expect(paddingFor('Item 1'), EdgeInsets.zero);
    expect(paddingFor('Item 2'), const EdgeInsets.all(20));
    expect(paddingFor('Item 3'), const EdgeInsets.all(20));
  });

  testWidgets('CheckedPopupMenuItem child height is a minimum, child is vertically centered', (
    WidgetTester tester,
  ) async {
    final Key popupMenuButtonKey = UniqueKey();
    final Type menuItemType = const CheckedPopupMenuItem<String>(child: Text('item')).runtimeType;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  // This menu item's height will be 56.0 because the default minimum height
                  // is 48, but the contents of PopupMenuItem are 56.0 tall.
                  const CheckedPopupMenuItem<String>(
                    checked: true,
                    value: '0',
                    child: Text('Item 0'),
                  ),
                  // This menu item's height parameter specifies its minimum height. The
                  // overall height of the menu item will be 60 because the child's
                  // height 56, is less than 60.
                  const CheckedPopupMenuItem<String>(
                    checked: true,
                    height: 60,
                    value: '1',
                    child: SizedBox(height: 40, child: Text('Item 1')),
                  ),
                  // This menu item's height parameter specifies its minimum height, so the
                  // overall height of the menu item will be 75.
                  const CheckedPopupMenuItem<String>(
                    checked: true,
                    height: 75,
                    value: '2',
                    child: SizedBox(child: Text('Item 2')),
                  ),
                  // This menu item's height will be 100.
                  const CheckedPopupMenuItem<String>(
                    checked: true,
                    height: 100,
                    value: '3',
                    child: SizedBox(child: Text('Item 3')),
                  ),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Show the menu
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // The menu items and their InkWells should have the expected vertical size
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 0')).height, 56);
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 1')).height, 60);
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 2')).height, 75);
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 3')).height, 100);
    // We evaluate the InkWell at the first index because that is the ListTile's
    // InkWell, which wins in the gesture arena over the child's InkWell and
    // is the one of interest.
    expect(tester.getSize(find.widgetWithText(InkWell, 'Item 0').at(1)).height, 56);
    expect(tester.getSize(find.widgetWithText(InkWell, 'Item 1').at(1)).height, 60);
    expect(tester.getSize(find.widgetWithText(InkWell, 'Item 2').at(1)).height, 75);
    expect(tester.getSize(find.widgetWithText(InkWell, 'Item 3').at(1)).height, 100);

    // Menu item children which whose height is less than the PopupMenuItem
    // are vertically centered.
    expect(
      tester.getRect(find.widgetWithText(menuItemType, 'Item 0')).center.dy,
      tester.getRect(find.text('Item 0')).center.dy,
    );
    expect(
      tester.getRect(find.widgetWithText(menuItemType, 'Item 2')).center.dy,
      tester.getRect(find.text('Item 2')).center.dy,
    );
  });

  testWidgets('CheckedPopupMenuItem custom padding', (WidgetTester tester) async {
    final Key popupMenuButtonKey = UniqueKey();
    final Type menuItemType = const CheckedPopupMenuItem<String>(child: Text('item')).runtimeType;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  const CheckedPopupMenuItem<String>(
                    padding: EdgeInsets.zero,
                    value: '0',
                    child: Text('Item 0'),
                  ),
                  const CheckedPopupMenuItem<String>(
                    padding: EdgeInsets.zero,
                    height: 0,
                    value: '0',
                    child: Text('Item 1'),
                  ),
                  const CheckedPopupMenuItem<String>(
                    padding: EdgeInsets.all(20),
                    value: '0',
                    child: Text('Item 2'),
                  ),
                  const CheckedPopupMenuItem<String>(
                    padding: EdgeInsets.all(20),
                    height: 100,
                    value: '0',
                    child: Text('Item 3'),
                  ),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Show the menu
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // The menu items and their InkWells should have the expected vertical size
    // given the interactions between heights and padding.
    expect(
      tester.getSize(find.widgetWithText(menuItemType, 'Item 0')).height,
      56,
    ); // Minimum ListTile height (56)
    expect(
      tester.getSize(find.widgetWithText(menuItemType, 'Item 1')).height,
      56,
    ); // Minimum ListTile height (56)
    expect(
      tester.getSize(find.widgetWithText(menuItemType, 'Item 2')).height,
      96,
    ); // Padding (20.0 + 20.0) + Height of ListTile (56) = 96
    expect(
      tester.getSize(find.widgetWithText(menuItemType, 'Item 3')).height,
      100,
    ); // Height value of 100, since child (56) + padding (40) < 100

    EdgeInsetsGeometry paddingFor(String text) {
      final ConstrainedBox widget = tester.widget<ConstrainedBox>(
        find.ancestor(
          of: find.text(text),
          matching: find.byWidgetPredicate(
            (Widget widget) => widget is ConstrainedBox && widget.child is Padding,
          ),
        ),
      );
      return (widget.child! as Padding).padding;
    }

    expect(paddingFor('Item 0'), EdgeInsets.zero);
    expect(paddingFor('Item 1'), EdgeInsets.zero);
    expect(paddingFor('Item 2'), const EdgeInsets.all(20));
    expect(paddingFor('Item 3'), const EdgeInsets.all(20));
  });

  testWidgets('Update PopupMenuItem layout while the menu is visible', (WidgetTester tester) async {
    final Key popupMenuButtonKey = UniqueKey();
    final Type menuItemType = const PopupMenuItem<String>(child: Text('item')).runtimeType;

    Widget buildFrame({TextDirection textDirection = TextDirection.ltr, double fontSize = 24}) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        builder: (BuildContext context, Widget? child) {
          return Directionality(
            textDirection: textDirection,
            child: PopupMenuTheme(
              data: PopupMenuTheme.of(context).copyWith(
                textStyle: Theme.of(context).textTheme.titleMedium!.copyWith(fontSize: fontSize),
              ),
              child: child!,
            ),
          );
        },
        home: Scaffold(
          body: PopupMenuButton<String>(
            key: popupMenuButtonKey,
            child: const Text('button'),
            onSelected: (String result) {},
            itemBuilder: (BuildContext context) {
              return <PopupMenuEntry<String>>[
                const PopupMenuItem<String>(value: '0', child: Text('Item 0')),
                const PopupMenuItem<String>(value: '1', child: Text('Item 1')),
              ];
            },
          ),
        ),
      );
    }

    // Show the menu
    await tester.pumpWidget(buildFrame());
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // The menu items should have their default heights and horizontal alignment.
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 0')).height, 48);
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 1')).height, 48);
    expect(tester.getTopLeft(find.text('Item 0')).dx, 24);
    expect(tester.getTopLeft(find.text('Item 1')).dx, 24);

    // While the menu is up, change its font size to 64 (default is 16).
    await tester.pumpWidget(buildFrame(fontSize: 64));
    await tester.pumpAndSettle(); // Theme changes are animated.
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 0')).height, 128);
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 1')).height, 128);
    expect(tester.getSize(find.text('Item 0')).height, 128);
    expect(tester.getSize(find.text('Item 1')).height, 128);
    expect(tester.getTopLeft(find.text('Item 0')).dx, 24);
    expect(tester.getTopLeft(find.text('Item 1')).dx, 24);

    // While the menu is up, change the textDirection to rtl. Now menu items
    // will be aligned right.
    await tester.pumpWidget(buildFrame(textDirection: TextDirection.rtl));
    await tester.pumpAndSettle(); // Theme changes are animated.
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 0')).height, 48);
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 1')).height, 48);
    expect(tester.getTopLeft(find.text('Item 0')).dx, 72);
    expect(tester.getTopLeft(find.text('Item 1')).dx, 72);
  });

  test("PopupMenuButton's child and icon properties cannot be simultaneously defined", () {
    expect(() {
      PopupMenuButton<int>(
        itemBuilder: (BuildContext context) => <PopupMenuItem<int>>[],
        icon: const Icon(Icons.error),
        child: Container(),
      );
    }, throwsAssertionError);
  });

  testWidgets('PopupMenuButton default tooltip', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              // Default Tooltip should be present when [PopupMenuButton.icon]
              // and [PopupMenuButton.child] are undefined.
              PopupMenuButton<int>(
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                  ];
                },
              ),
              // Default Tooltip should be present when
              // [PopupMenuButton.child] is defined.
              PopupMenuButton<int>(
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                  ];
                },
                child: const Text('Test text'),
              ),
              // Default Tooltip should be present when
              // [PopupMenuButton.icon] is defined.
              PopupMenuButton<int>(
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                  ];
                },
                icon: const Icon(Icons.check),
              ),
            ],
          ),
        ),
      ),
    );

    // The default tooltip is defined as [MaterialLocalizations.showMenuTooltip]
    // and it is used when no tooltip is provided.
    expect(find.byType(Tooltip), findsNWidgets(3));
    expect(find.byTooltip(const DefaultMaterialLocalizations().showMenuTooltip), findsNWidgets(3));
  });

  testWidgets('PopupMenuButton custom tooltip', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              // Tooltip should work when [PopupMenuButton.icon]
              // and [PopupMenuButton.child] are undefined.
              PopupMenuButton<int>(
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                  ];
                },
                tooltip: 'Test tooltip',
              ),
              // Tooltip should work when
              // [PopupMenuButton.child] is defined.
              PopupMenuButton<int>(
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                  ];
                },
                tooltip: 'Test tooltip',
                child: const Text('Test text'),
              ),
              // Tooltip should work when
              // [PopupMenuButton.icon] is defined.
              PopupMenuButton<int>(
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                  ];
                },
                tooltip: 'Test tooltip',
                icon: const Icon(Icons.check),
              ),
            ],
          ),
        ),
      ),
    );

    expect(find.byType(Tooltip), findsNWidgets(3));
    expect(find.byTooltip('Test tooltip'), findsNWidgets(3));
  });

  testWidgets('Allow Widget for PopupMenuButton.icon', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: PopupMenuButton<int>(
            itemBuilder: (BuildContext context) {
              return <PopupMenuEntry<int>>[
                const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
              ];
            },
            tooltip: 'Test tooltip',
            icon: const Text('PopupMenuButton icon'),
          ),
        ),
      ),
    );

    expect(find.text('PopupMenuButton icon'), findsOneWidget);
  });

  testWidgets('showMenu uses nested navigator by default', (WidgetTester tester) async {
    final MenuObserver rootObserver = MenuObserver();
    final MenuObserver nestedObserver = MenuObserver();

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[nestedObserver],
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<dynamic>(
              builder: (BuildContext context) {
                return ElevatedButton(
                  onPressed: () {
                    showMenu<int>(
                      context: context,
                      position: RelativeRect.fill,
                      items: <PopupMenuItem<int>>[
                        const PopupMenuItem<int>(value: 1, child: Text('1')),
                      ],
                    );
                  },
                  child: const Text('Show Menu'),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.byType(ElevatedButton));

    expect(rootObserver.menuCount, 0);
    expect(nestedObserver.menuCount, 1);
  });

  testWidgets('showMenu uses root navigator if useRootNavigator is true', (
    WidgetTester tester,
  ) async {
    final MenuObserver rootObserver = MenuObserver();
    final MenuObserver nestedObserver = MenuObserver();

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[nestedObserver],
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<dynamic>(
              builder: (BuildContext context) {
                return ElevatedButton(
                  onPressed: () {
                    showMenu<int>(
                      context: context,
                      useRootNavigator: true,
                      position: RelativeRect.fill,
                      items: <PopupMenuItem<int>>[
                        const PopupMenuItem<int>(value: 1, child: Text('1')),
                      ],
                    );
                  },
                  child: const Text('Show Menu'),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.byType(ElevatedButton));

    expect(rootObserver.menuCount, 1);
    expect(nestedObserver.menuCount, 0);
  });

  testWidgets('PopupMenuButton calling showButtonMenu manually', (WidgetTester tester) async {
    final GlobalKey<PopupMenuButtonState<int>> globalKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              PopupMenuButton<int>(
                key: globalKey,
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Tap me please!')),
                  ];
                },
              ),
            ],
          ),
        ),
      ),
    );

    expect(find.text('Tap me please!'), findsNothing);

    globalKey.currentState!.showButtonMenu();
    // The PopupMenuItem will appear after an animation, hence,
    // we have to first wait for the tester to settle.
    await tester.pumpAndSettle();

    expect(find.text('Tap me please!'), findsOneWidget);
  });

  testWidgets('PopupMenuItem changes mouse cursor when hovered', (WidgetTester tester) async {
    const Key key = ValueKey<int>(1);
    // Test PopupMenuItem() constructor
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: PopupMenuItem<int>(
                  key: key,
                  mouseCursor: SystemMouseCursors.text,
                  value: 1,
                  child: Container(),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byKey(key)));

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test default cursor
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: PopupMenuItem<int>(key: key, value: 1, child: Container()),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test default cursor when disabled
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: PopupMenuItem<int>(key: key, value: 1, enabled: false, child: Container()),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });

  testWidgets('CheckedPopupMenuItem changes mouse cursor when hovered', (
    WidgetTester tester,
  ) async {
    const Key key = ValueKey<int>(1);
    // Test CheckedPopupMenuItem() constructor
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: CheckedPopupMenuItem<int>(
                  key: key,
                  mouseCursor: SystemMouseCursors.text,
                  value: 1,
                  child: Container(),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    final TestGesture gesture = await tester.createGesture(
      kind: PointerDeviceKind.mouse,
      pointer: 1,
    );
    await gesture.addPointer(location: tester.getCenter(find.byKey(key)));
    addTearDown(gesture.removePointer);

    await tester.pump();

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.text,
    );

    // Test default cursor
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: CheckedPopupMenuItem<int>(key: key, value: 1, child: Container()),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );

    // Test default cursor when disabled
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Align(
            alignment: Alignment.topLeft,
            child: Material(
              child: MouseRegion(
                cursor: SystemMouseCursors.forbidden,
                child: CheckedPopupMenuItem<int>(
                  key: key,
                  value: 1,
                  enabled: false,
                  child: Container(),
                ),
              ),
            ),
          ),
        ),
      ),
    );

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.basic,
    );
  });

  testWidgets('PopupMenu in AppBar does not overlap with the status bar', (
    WidgetTester tester,
  ) async {
    const List<PopupMenuItem<int>> choices = <PopupMenuItem<int>>[
      PopupMenuItem<int>(value: 1, child: Text('Item 1')),
      PopupMenuItem<int>(value: 2, child: Text('Item 2')),
      PopupMenuItem<int>(value: 3, child: Text('Item 3')),
    ];

    const double statusBarHeight = 24.0;
    final PopupMenuItem<int> firstItem = choices[0];
    int selectedValue = choices[0].value!;

    await tester.pumpWidget(
      MaterialApp(
        builder: (BuildContext context, Widget? child) {
          return MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.only(top: statusBarHeight),
            ), // status bar
            child: child!,
          );
        },
        home: StatefulBuilder(
          builder: (BuildContext context, StateSetter setState) {
            return Scaffold(
              appBar: AppBar(
                title: const Text('PopupMenu Test'),
                actions: <Widget>[
                  PopupMenuButton<int>(
                    onSelected: (int result) {
                      setState(() {
                        selectedValue = result;
                      });
                    },
                    initialValue: selectedValue,
                    itemBuilder: (BuildContext context) {
                      return choices;
                    },
                  ),
                ],
              ),
            );
          },
        ),
      ),
    );

    await tester.tap(find.byIcon(Icons.more_vert));
    await tester.pumpAndSettle();

    // Tap third item.
    await tester.tap(find.text('Item 3'));
    await tester.pumpAndSettle();

    // Open popupMenu again.
    await tester.tap(find.byIcon(Icons.more_vert));
    await tester.pumpAndSettle();

    // Check whether the first item is not overlapping with status bar.
    expect(tester.getTopLeft(find.byWidget(firstItem)).dy, greaterThan(statusBarHeight));
  });

  testWidgets('Vertically long PopupMenu does not overlap with the status bar and bottom notch', (
    WidgetTester tester,
  ) async {
    const double windowPaddingTop = 44;
    const double windowPaddingBottom = 34;

    await tester.pumpWidget(
      MaterialApp(
        builder: (BuildContext context, Widget? child) {
          return MediaQuery(
            data: const MediaQueryData(
              padding: EdgeInsets.only(top: windowPaddingTop, bottom: windowPaddingBottom),
            ),
            child: child!,
          );
        },
        home: Scaffold(
          appBar: AppBar(title: const Text('PopupMenu Test')),
          body: PopupMenuButton<int>(
            child: const Text('Show Menu'),
            itemBuilder:
                (BuildContext context) =>
                    Iterable<PopupMenuItem<int>>.generate(
                      20,
                      (int i) => PopupMenuItem<int>(value: i, child: Text('Item $i')),
                    ).toList(),
          ),
        ),
      ),
    );

    await tester.tap(find.text('Show Menu'));
    await tester.pumpAndSettle();

    final Offset topRightOfMenu = tester.getTopRight(find.byType(SingleChildScrollView));
    final Offset bottomRightOfMenu = tester.getBottomRight(find.byType(SingleChildScrollView));

    expect(topRightOfMenu.dy, windowPaddingTop + 8.0);
    expect(bottomRightOfMenu.dy, 600.0 - windowPaddingBottom - 8.0); // Screen height is 600.
  });

  testWidgets('PopupMenu position test when have unsafe area', (WidgetTester tester) async {
    final GlobalKey buttonKey = GlobalKey();

    Widget buildFrame(double width, double height) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        builder: (BuildContext context, Widget? child) {
          return MediaQuery(
            data: const MediaQueryData(padding: EdgeInsets.only(top: 32.0, bottom: 32.0)),
            child: child!,
          );
        },
        home: Scaffold(
          appBar: AppBar(
            title: const Text('PopupMenu Test'),
            actions: <Widget>[
              PopupMenuButton<int>(
                child: SizedBox(
                  key: buttonKey,
                  height: height,
                  width: width,
                  child: const ColoredBox(color: Colors.pink),
                ),
                itemBuilder:
                    (BuildContext context) => <PopupMenuEntry<int>>[
                      const PopupMenuItem<int>(value: 1, child: Text('-1-')),
                      const PopupMenuItem<int>(value: 2, child: Text('-2-')),
                    ],
              ),
            ],
          ),
          body: Container(),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(20.0, 20.0));

    await tester.tap(find.byKey(buttonKey));
    await tester.pumpAndSettle();

    final Offset button = tester.getTopRight(find.byKey(buttonKey));
    expect(button, const Offset(800.0, 32.0)); // The topPadding is 32.0.

    final Offset popupMenu = tester.getTopRight(find.byType(SingleChildScrollView));

    // The menu should be positioned directly next to the top of the button.
    // The 8.0 pixels is [_kMenuScreenPadding].
    expect(popupMenu, Offset(button.dx - 8.0, button.dy + 8.0));
  });

  // Regression test for https://github.com/flutter/flutter/issues/82874
  testWidgets('PopupMenu position test when have unsafe area - left/right padding', (
    WidgetTester tester,
  ) async {
    final GlobalKey buttonKey = GlobalKey();
    const EdgeInsets padding = EdgeInsets.only(left: 300.0, top: 32.0, right: 310.0, bottom: 64.0);
    EdgeInsets? mediaQueryPadding;

    Widget buildFrame(double width, double height) {
      return MaterialApp(
        theme: ThemeData(useMaterial3: false),
        builder: (BuildContext context, Widget? child) {
          return MediaQuery(data: const MediaQueryData(padding: padding), child: child!);
        },
        home: Scaffold(
          appBar: AppBar(
            title: const Text('PopupMenu Test'),
            actions: <Widget>[
              PopupMenuButton<int>(
                child: SizedBox(
                  key: buttonKey,
                  height: height,
                  width: width,
                  child: const ColoredBox(color: Colors.pink),
                ),
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<int>>[
                    PopupMenuItem<int>(
                      value: 1,
                      child: Builder(
                        builder: (BuildContext context) {
                          mediaQueryPadding = MediaQuery.paddingOf(context);
                          return Text('-1-' * 500); // A long long text string.
                        },
                      ),
                    ),
                    const PopupMenuItem<int>(value: 2, child: Text('-2-')),
                  ];
                },
              ),
            ],
          ),
          body: const SizedBox.shrink(),
        ),
      );
    }

    await tester.pumpWidget(buildFrame(20.0, 20.0));

    await tester.tap(find.byKey(buttonKey));
    await tester.pumpAndSettle();

    final Offset button = tester.getTopRight(find.byKey(buttonKey));
    expect(button, Offset(800.0 - padding.right, padding.top)); // The topPadding is 32.0.

    final Offset popupMenuTopRight = tester.getTopRight(find.byType(SingleChildScrollView));

    // The menu should be positioned directly next to the top of the button.
    // The 8.0 pixels is [_kMenuScreenPadding].
    expect(popupMenuTopRight, Offset(800.0 - padding.right - 8.0, padding.top + 8.0));

    final Offset popupMenuTopLeft = tester.getTopLeft(find.byType(SingleChildScrollView));
    expect(popupMenuTopLeft, Offset(padding.left + 8.0, padding.top + 8.0));

    final Offset popupMenuBottomLeft = tester.getBottomLeft(find.byType(SingleChildScrollView));
    expect(popupMenuBottomLeft, Offset(padding.left + 8.0, 600.0 - padding.bottom - 8.0));

    // The `MediaQueryData.padding` should be removed.
    expect(mediaQueryPadding, EdgeInsets.zero);
  });

  group('feedback', () {
    late FeedbackTester feedback;

    setUp(() {
      feedback = FeedbackTester();
    });

    tearDown(() {
      feedback.dispose();
    });

    Widget buildFrame({bool? widgetEnableFeedback, bool? themeEnableFeedback}) {
      return MaterialApp(
        home: Scaffold(
          body: PopupMenuTheme(
            data: PopupMenuThemeData(enableFeedback: themeEnableFeedback),
            child: PopupMenuButton<int>(
              enableFeedback: widgetEnableFeedback,
              child: const Text('Show Menu'),
              itemBuilder: (BuildContext context) {
                return <PopupMenuItem<int>>[const PopupMenuItem<int>(value: 1, child: Text('One'))];
              },
            ),
          ),
        ),
      );
    }

    testWidgets('PopupMenuButton enableFeedback works properly', (WidgetTester tester) async {
      expect(feedback.clickSoundCount, 0);
      expect(feedback.hapticCount, 0);

      // PopupMenuButton with enabled feedback.
      await tester.pumpWidget(buildFrame(widgetEnableFeedback: true));
      await tester.tap(find.text('Show Menu'));
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);

      await tester.pumpWidget(Container());

      // PopupMenuButton with disabled feedback.
      await tester.pumpWidget(buildFrame(widgetEnableFeedback: false));
      await tester.tap(find.text('Show Menu'));
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 1);
      expect(feedback.hapticCount, 0);

      await tester.pumpWidget(Container());

      // PopupMenuButton with enabled feedback by default.
      await tester.pumpWidget(buildFrame());
      await tester.tap(find.text('Show Menu'));
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 2);
      expect(feedback.hapticCount, 0);

      await tester.pumpWidget(Container());

      // PopupMenu with disabled feedback using PopupMenuButtonTheme.
      await tester.pumpWidget(buildFrame(themeEnableFeedback: false));
      await tester.tap(find.text('Show Menu'));
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 2);
      expect(feedback.hapticCount, 0);

      await tester.pumpWidget(Container());

      // PopupMenu enableFeedback property overrides PopupMenuButtonTheme.
      await tester.pumpWidget(buildFrame(widgetEnableFeedback: false, themeEnableFeedback: true));
      await tester.tap(find.text('Show Menu'));
      await tester.pumpAndSettle();
      expect(feedback.clickSoundCount, 2);
      expect(feedback.hapticCount, 0);
    });
  });

  testWidgets('Can customize PopupMenuButton icon', (WidgetTester tester) async {
    const Color iconColor = Color(0xffffff00);
    const double iconSize = 29.5;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              iconColor: iconColor,
              iconSize: iconSize,
              itemBuilder:
                  (_) => <PopupMenuEntry<String>>[
                    const PopupMenuItem<String>(value: 'value', child: Text('child')),
                  ],
            ),
          ),
        ),
      ),
    );

    expect(_iconStyle(tester, Icons.adaptive.more)?.color, iconColor);
    expect(tester.getSize(find.byIcon(Icons.adaptive.more)), const Size(iconSize, iconSize));
  });

  testWidgets('does not crash in small overlay', (WidgetTester tester) async {
    final GlobalKey navigator = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Column(
            children: <Widget>[
              OutlinedButton(
                onPressed: () {
                  showMenu<void>(
                    context: navigator.currentContext!,
                    position: RelativeRect.fill,
                    items: const <PopupMenuItem<void>>[PopupMenuItem<void>(child: Text('foo'))],
                  );
                },
                child: const Text('press'),
              ),
              SizedBox(
                height: 10,
                width: 10,
                child: Navigator(
                  key: navigator,
                  onGenerateRoute:
                      (RouteSettings settings) => MaterialPageRoute<void>(
                        builder: (BuildContext context) => Container(color: Colors.red),
                      ),
                ),
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.text('press'));
    await tester.pumpAndSettle();
    expect(find.text('foo'), findsOneWidget);
  });

  // Regression test for https://github.com/flutter/flutter/issues/80869
  testWidgets('The menu position test in the scrollable widget', (WidgetTester tester) async {
    final GlobalKey buttonKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: SingleChildScrollView(
            child: Column(
              children: <Widget>[
                const SizedBox(height: 100),
                PopupMenuButton<int>(
                  child: SizedBox(
                    key: buttonKey,
                    height: 10.0,
                    width: 10.0,
                    child: const ColoredBox(color: Colors.pink),
                  ),
                  itemBuilder:
                      (BuildContext context) => <PopupMenuEntry<int>>[
                        const PopupMenuItem<int>(value: 1, child: Text('-1-')),
                        const PopupMenuItem<int>(value: 2, child: Text('-2-')),
                      ],
                ),
                const SizedBox(height: 600),
              ],
            ),
          ),
        ),
      ),
    );

    // Open the menu.
    await tester.tap(find.byKey(buttonKey));
    await tester.pumpAndSettle();

    Offset button = tester.getTopLeft(find.byKey(buttonKey));
    expect(button, const Offset(0.0, 100.0));

    Offset popupMenu = tester.getTopLeft(find.byType(SingleChildScrollView).last);
    // The menu should be positioned directly next to the top of the button.
    // The 8.0 pixels is [_kMenuScreenPadding].
    expect(popupMenu, const Offset(8.0, 100.0));

    // Close the menu.
    await tester.tap(find.byKey(buttonKey), warnIfMissed: false);
    await tester.pumpAndSettle();

    // Scroll a little bit.
    await tester.drag(find.byType(SingleChildScrollView), const Offset(0.0, -50.0));

    button = tester.getTopLeft(find.byKey(buttonKey));
    expect(button, const Offset(0.0, 50.0));

    // Open the menu again.
    await tester.tap(find.byKey(buttonKey));
    await tester.pumpAndSettle();

    popupMenu = tester.getTopLeft(find.byType(SingleChildScrollView).last);
    // The menu should be positioned directly next to the top of the button.
    // The 8.0 pixels is [_kMenuScreenPadding].
    expect(popupMenu, const Offset(8.0, 50.0));
  });

  testWidgets('PopupMenuButton custom splash radius', (WidgetTester tester) async {
    Future<void> buildFrameWithoutChild({double? splashRadius}) {
      return tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(useMaterial3: false),
          home: Scaffold(
            body: Center(
              child: PopupMenuButton<String>(
                splashRadius: splashRadius,
                itemBuilder:
                    (_) => <PopupMenuEntry<String>>[
                      const PopupMenuItem<String>(value: 'value', child: Text('child')),
                    ],
              ),
            ),
          ),
        ),
      );
    }

    Future<void> buildFrameWithChild({double? splashRadius}) {
      return tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Center(
              child: PopupMenuButton<String>(
                splashRadius: splashRadius,
                child: const Text('An item'),
                itemBuilder: (_) => <PopupMenuEntry<String>>[const PopupMenuDivider()],
              ),
            ),
          ),
        ),
      );
    }

    await buildFrameWithoutChild();
    expect(
      tester.widget<InkResponse>(find.byType(InkResponse)).radius,
      Material.defaultSplashRadius,
    );
    await buildFrameWithChild();
    expect(tester.widget<InkWell>(find.byType(InkWell)).radius, null);

    const double testSplashRadius = 50;

    await buildFrameWithoutChild(splashRadius: testSplashRadius);
    expect(tester.widget<InkResponse>(find.byType(InkResponse)).radius, testSplashRadius);
    await buildFrameWithChild(splashRadius: testSplashRadius);
    expect(tester.widget<InkWell>(find.byType(InkWell)).radius, testSplashRadius);
  });

  testWidgets('Can override menu size constraints', (WidgetTester tester) async {
    final Key popupMenuButtonKey = UniqueKey();
    final Type menuItemType = const PopupMenuItem<String>(child: Text('item')).runtimeType;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              constraints: const BoxConstraints(minWidth: 500),
              itemBuilder:
                  (_) => <PopupMenuEntry<String>>[
                    const PopupMenuItem<String>(value: 'value', child: Text('Item 0')),
                  ],
            ),
          ),
        ),
      ),
    );

    // Show the menu
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 0')).height, 48);
    expect(tester.getSize(find.widgetWithText(menuItemType, 'Item 0')).width, 500);
  });

  testWidgets('Can change menu position and offset', (WidgetTester tester) async {
    PopupMenuButton<int> buildMenuButton({required PopupMenuPosition position}) {
      return PopupMenuButton<int>(
        position: position,
        itemBuilder: (BuildContext context) {
          return <PopupMenuItem<int>>[
            PopupMenuItem<int>(
              value: 1,
              child: Builder(
                builder: (BuildContext context) {
                  return const Text('AAA');
                },
              ),
            ),
          ];
        },
      );
    }

    // Popup menu with `MenuPosition.over (default) with default offset`.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: Material(child: buildMenuButton(position: PopupMenuPosition.over))),
      ),
    );

    // Open the popup menu.
    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    expect(
      tester.getTopLeft(
        find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_PopupMenu<int?>'),
      ),
      const Offset(8.0, 8.0),
    );

    // Close the popup menu.
    await tester.tapAt(Offset.zero);
    await tester.pump();

    // Popup menu with `MenuPosition.under`(custom) with default offset`.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(body: Material(child: buildMenuButton(position: PopupMenuPosition.under))),
      ),
    );

    // Open the popup menu.
    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    expect(
      tester.getTopLeft(
        find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_PopupMenu<int?>'),
      ),
      const Offset(8.0, 40.0),
    );

    // Close the popup menu.
    await tester.tapAt(Offset.zero);
    await tester.pump();

    // Popup menu with `MenuPosition.over (default) with custom offset`.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Material(
            child: PopupMenuButton<int>(
              offset: const Offset(0.0, 50),
              itemBuilder: (BuildContext context) {
                return <PopupMenuItem<int>>[
                  PopupMenuItem<int>(
                    value: 1,
                    child: Builder(
                      builder: (BuildContext context) {
                        return const Text('AAA');
                      },
                    ),
                  ),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Open the popup menu.
    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    expect(
      tester.getTopLeft(
        find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_PopupMenu<int?>'),
      ),
      const Offset(8.0, 50.0),
    );

    // Close the popup menu.
    await tester.tapAt(Offset.zero);
    await tester.pump();

    // Popup menu with `MenuPosition.under (custom) with custom offset`.
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Material(
            child: PopupMenuButton<int>(
              offset: const Offset(0.0, 50),
              position: PopupMenuPosition.under,
              itemBuilder: (BuildContext context) {
                return <PopupMenuItem<int>>[
                  PopupMenuItem<int>(
                    value: 1,
                    child: Builder(
                      builder: (BuildContext context) {
                        return const Text('AAA');
                      },
                    ),
                  ),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Open the popup menu.
    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    expect(
      tester.getTopLeft(
        find.byWidgetPredicate((Widget w) => '${w.runtimeType}' == '_PopupMenu<int?>'),
      ),
      const Offset(8.0, 90.0),
    );
  });

  testWidgets("PopupMenuButton icon inherits IconTheme's size", (WidgetTester tester) async {
    Widget buildPopupMenu({double? themeIconSize, double? iconSize}) {
      return MaterialApp(
        theme: ThemeData(iconTheme: IconThemeData(size: themeIconSize)),
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              iconSize: iconSize,
              itemBuilder:
                  (_) => <PopupMenuEntry<String>>[
                    const PopupMenuItem<String>(value: 'value', child: Text('Item 0')),
                  ],
            ),
          ),
        ),
      );
    }

    // Popup menu with default icon size.
    await tester.pumpWidget(buildPopupMenu());
    // Default PopupMenuButton icon size is 24.0.
    expect(tester.getSize(find.byIcon(Icons.more_vert)), const Size(24.0, 24.0));

    // Popup menu with custom theme icon size.
    await tester.pumpWidget(buildPopupMenu(themeIconSize: 30.0));
    await tester.pumpAndSettle();
    // PopupMenuButton icon inherits IconTheme's size.
    expect(tester.getSize(find.byIcon(Icons.more_vert)), const Size(30.0, 30.0));

    // Popup menu with custom icon size.
    await tester.pumpWidget(buildPopupMenu(themeIconSize: 30.0, iconSize: 50.0));
    await tester.pumpAndSettle();
    // PopupMenuButton icon size overrides IconTheme's size.
    expect(tester.getSize(find.byIcon(Icons.more_vert)), const Size(50.0, 50.0));
  });

  testWidgets('Popup menu clip behavior', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/107215
    final Key popupButtonKey = UniqueKey();
    const double radius = 20.0;

    Widget buildPopupMenu({required Clip clipBehavior}) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupButtonKey,
              shape: const RoundedRectangleBorder(
                borderRadius: BorderRadius.all(Radius.circular(radius)),
              ),
              clipBehavior: clipBehavior,
              itemBuilder:
                  (_) => <PopupMenuEntry<String>>[
                    const PopupMenuItem<String>(value: 'value', child: Text('Item 0')),
                  ],
            ),
          ),
        ),
      );
    }

    // Popup menu with default ClipBehavior.
    await tester.pumpWidget(buildPopupMenu(clipBehavior: Clip.none));

    // Open the popup to build and show the menu contents.
    await tester.tap(find.byKey(popupButtonKey));
    await tester.pumpAndSettle();

    Material material = tester.widget<Material>(find.byType(Material).last);
    expect(material.clipBehavior, Clip.none);

    // Close the popup menu.
    await tester.tapAt(Offset.zero);
    await tester.pumpAndSettle();

    // Popup menu with custom ClipBehavior.
    await tester.pumpWidget(buildPopupMenu(clipBehavior: Clip.hardEdge));

    // Open the popup to build and show the menu contents.
    await tester.tap(find.byKey(popupButtonKey));
    await tester.pumpAndSettle();

    material = tester.widget<Material>(find.byType(Material).last);
    expect(material.clipBehavior, Clip.hardEdge);
  });

  testWidgets('Uses closed loop focus traversal', (WidgetTester tester) async {
    FocusNode nodeA() => Focus.of(find.text('A').evaluate().single);
    FocusNode nodeB() => Focus.of(find.text('B').evaluate().single);

    final GlobalKey popupButtonKey = GlobalKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupButtonKey,
              itemBuilder:
                  (_) => const <PopupMenuEntry<String>>[
                    PopupMenuItem<String>(value: 'a', child: Text('A')),
                    PopupMenuItem<String>(value: 'b', child: Text('B')),
                  ],
            ),
          ),
        ),
      ),
    );

    // Open the popup to build and show the menu contents.
    await tester.tap(find.byKey(popupButtonKey));
    await tester.pumpAndSettle();

    Future<bool> nextFocus() async {
      final bool result = Actions.invoke(primaryFocus!.context!, const NextFocusIntent())! as bool;
      await tester.pump();
      return result;
    }

    Future<bool> previousFocus() async {
      final bool result =
          Actions.invoke(primaryFocus!.context!, const PreviousFocusIntent())! as bool;
      await tester.pump();
      return result;
    }

    // Start at A
    nodeA().requestFocus();
    await tester.pump();
    expect(nodeA().hasFocus, true);
    expect(nodeB().hasFocus, false);

    // A -> B
    expect(await nextFocus(), true);
    expect(nodeA().hasFocus, false);
    expect(nodeB().hasFocus, true);

    // B -> A (wrap around)
    expect(await nextFocus(), true);
    expect(nodeA().hasFocus, true);
    expect(nodeB().hasFocus, false);

    // B <- A
    expect(await previousFocus(), true);
    expect(nodeA().hasFocus, false);
    expect(nodeB().hasFocus, true);

    // A <- B (wrap around)
    expect(await previousFocus(), true);
    expect(nodeA().hasFocus, true);
    expect(nodeB().hasFocus, false);
  });

  testWidgets('Popup menu scrollbar inherits ScrollbarTheme', (WidgetTester tester) async {
    final Key popupButtonKey = UniqueKey();
    const ScrollbarThemeData scrollbarTheme = ScrollbarThemeData(
      thumbColor: MaterialStatePropertyAll<Color?>(Color(0xffff0000)),
      thumbVisibility: MaterialStatePropertyAll<bool?>(true),
    );
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(scrollbarTheme: scrollbarTheme, useMaterial3: true),
        home: Material(
          child: Column(
            children: <Widget>[
              PopupMenuButton<void>(
                key: popupButtonKey,
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<void>>[
                    const PopupMenuItem<void>(height: 1000, child: Text('Example')),
                  ];
                },
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(popupButtonKey));
    await tester.pumpAndSettle();

    expect(find.byType(Scrollbar), findsOneWidget);
    // Test Scrollbar thumb color.
    expect(find.byType(Scrollbar), paints..rrect(color: const Color(0xffff0000)));

    // Close the menu.
    await tester.tapAt(const Offset(20.0, 20.0));
    await tester.pumpAndSettle();

    // Test local ScrollbarTheme overrides global ScrollbarTheme.
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(scrollbarTheme: scrollbarTheme, useMaterial3: true),
        home: Material(
          child: Column(
            children: <Widget>[
              ScrollbarTheme(
                data: scrollbarTheme.copyWith(
                  thumbColor: const MaterialStatePropertyAll<Color?>(Color(0xff0000ff)),
                ),
                child: PopupMenuButton<void>(
                  key: popupButtonKey,
                  itemBuilder: (BuildContext context) {
                    return <PopupMenuEntry<void>>[
                      const PopupMenuItem<void>(height: 1000, child: Text('Example')),
                    ];
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
    await tester.tap(find.byKey(popupButtonKey));
    await tester.pumpAndSettle();

    expect(find.byType(Scrollbar), findsOneWidget);
    // Scrollbar thumb color should be updated.
    expect(find.byType(Scrollbar), paints..rrect(color: const Color(0xff0000ff)));
  }, variant: TargetPlatformVariant.desktop());

  testWidgets('Popup menu with RouteSettings', (WidgetTester tester) async {
    late RouteSettings currentRouteSetting;

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[
          _ClosureNavigatorObserver(
            onDidChange: (Route<dynamic> newRoute) {
              currentRouteSetting = newRoute.settings;
            },
          ),
        ],
        home: const Material(
          child: Center(child: ElevatedButton(onPressed: null, child: Text('Go'))),
        ),
      ),
    );

    final BuildContext context = tester.element(find.text('Go'));
    const RouteSettings exampleSetting = RouteSettings(name: 'simple');

    showMenu<void>(
      context: context,
      position: RelativeRect.fill,
      items: const <PopupMenuItem<void>>[PopupMenuItem<void>(child: Text('foo'))],
      routeSettings: exampleSetting,
    );

    await tester.pumpAndSettle();
    expect(find.text('foo'), findsOneWidget);
    expect(currentRouteSetting, exampleSetting);

    await tester.tap(find.text('foo'));
    await tester.pumpAndSettle();
    expect(currentRouteSetting.name, '/');
  });

  testWidgets('Popup menu is positioned under the child', (WidgetTester tester) async {
    final Key popupButtonKey = UniqueKey();
    final Key childKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        home: Material(
          child: Column(
            children: <Widget>[
              PopupMenuButton<void>(
                key: popupButtonKey,
                position: PopupMenuPosition.under,
                itemBuilder: (BuildContext context) {
                  return <PopupMenuEntry<void>>[const PopupMenuItem<void>(child: Text('Example'))];
                },
                child: SizedBox(key: childKey, height: 50, width: 50),
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.byKey(popupButtonKey));
    await tester.pumpAndSettle();

    final Offset childBottomLeft = tester.getBottomLeft(find.byKey(childKey));
    final Offset menuTopLeft = tester.getTopLeft(find.bySemanticsLabel('Popup menu'));
    expect(childBottomLeft, menuTopLeft);
  });

  testWidgets('PopupMenuItem onTap should be calling after Navigator.pop', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            actions: <Widget>[
              PopupMenuButton<int>(
                itemBuilder:
                    (BuildContext context) => <PopupMenuItem<int>>[
                      PopupMenuItem<int>(
                        onTap: () {
                          showModalBottomSheet<void>(
                            context: context,
                            builder: (BuildContext context) {
                              return const SizedBox(
                                height: 200.0,
                                child: Center(child: Text('ModalBottomSheet')),
                              );
                            },
                          );
                        },
                        value: 10,
                        child: const Text('ACTION'),
                      ),
                    ],
              ),
            ],
          ),
        ),
      ),
    );

    await tester.tap(find.byType(PopupMenuButton<int>));
    await tester.pumpAndSettle();

    await tester.tap(find.text('ACTION'));
    await tester.pumpAndSettle();

    // Verify that the ModalBottomSheet is displayed
    final Finder modalBottomSheet = find.text('ModalBottomSheet');
    expect(modalBottomSheet, findsOneWidget);
  });

  testWidgets('Material3 - CheckedPopupMenuItem.labelTextStyle uses correct text style', (
    WidgetTester tester,
  ) async {
    final Key popupMenuButtonKey = UniqueKey();
    ThemeData theme = ThemeData(useMaterial3: true);

    Widget buildMenu() {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          appBar: AppBar(
            actions: <Widget>[
              PopupMenuButton<void>(
                key: popupMenuButtonKey,
                itemBuilder:
                    (BuildContext context) => <PopupMenuItem<void>>[
                      const CheckedPopupMenuItem<void>(child: Text('Item 1')),
                      const CheckedPopupMenuItem<int>(checked: true, child: Text('Item 2')),
                    ],
              ),
            ],
          ),
        ),
      );
    }

    await tester.pumpWidget(buildMenu());

    // Show the menu
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // Test default text style.
    expect(_labelStyle(tester, 'Item 1')!.fontSize, 14.0);
    expect(_labelStyle(tester, 'Item 1')!.color, theme.colorScheme.onSurface);

    // Close the menu.
    await tester.tapAt(const Offset(20.0, 20.0));
    await tester.pumpAndSettle();

    // Test custom text theme.
    const TextStyle customTextStyle = TextStyle(
      fontSize: 20.0,
      fontWeight: FontWeight.bold,
      fontStyle: FontStyle.italic,
    );
    theme = theme.copyWith(textTheme: const TextTheme(labelLarge: customTextStyle));
    await tester.pumpWidget(buildMenu());

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // Test custom text theme.
    expect(_labelStyle(tester, 'Item 1')!.fontSize, customTextStyle.fontSize);
    expect(_labelStyle(tester, 'Item 1')!.fontWeight, customTextStyle.fontWeight);
    expect(_labelStyle(tester, 'Item 1')!.fontStyle, customTextStyle.fontStyle);
  });

  testWidgets('CheckedPopupMenuItem.labelTextStyle resolve material states', (
    WidgetTester tester,
  ) async {
    final Key popupMenuButtonKey = UniqueKey();
    final MaterialStateProperty<TextStyle?> labelTextStyle = MaterialStateProperty.resolveWith((
      Set<MaterialState> states,
    ) {
      if (states.contains(MaterialState.selected)) {
        return const TextStyle(color: Colors.red, fontSize: 24.0);
      }

      return const TextStyle(color: Colors.amber, fontSize: 20.0);
    });

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          appBar: AppBar(
            actions: <Widget>[
              PopupMenuButton<void>(
                key: popupMenuButtonKey,
                itemBuilder:
                    (BuildContext context) => <PopupMenuItem<void>>[
                      CheckedPopupMenuItem<void>(
                        labelTextStyle: labelTextStyle,
                        child: const Text('Item 1'),
                      ),
                      CheckedPopupMenuItem<int>(
                        checked: true,
                        labelTextStyle: labelTextStyle,
                        child: const Text('Item 2'),
                      ),
                    ],
              ),
            ],
          ),
        ),
      ),
    );

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    expect(_labelStyle(tester, 'Item 1'), labelTextStyle.resolve(<MaterialState>{}));
    expect(
      _labelStyle(tester, 'Item 2'),
      labelTextStyle.resolve(<MaterialState>{MaterialState.selected}),
    );
  });

  testWidgets('CheckedPopupMenuItem overrides redundant ListTile.contentPadding', (
    WidgetTester tester,
  ) async {
    final Key popupMenuButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  const CheckedPopupMenuItem<String>(value: '0', child: Text('Item 0')),
                  const CheckedPopupMenuItem<String>(
                    value: '1',
                    checked: true,
                    child: Text('Item 1'),
                  ),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    SafeArea getItemSafeArea(String label) {
      return tester.widget<SafeArea>(
        find.ancestor(of: find.text(label), matching: find.byType(SafeArea)),
      );
    }

    expect(getItemSafeArea('Item 0').minimum, EdgeInsets.zero);
    expect(getItemSafeArea('Item 1').minimum, EdgeInsets.zero);
  });

  testWidgets('PopupMenuItem overrides redundant ListTile.contentPadding', (
    WidgetTester tester,
  ) async {
    final Key popupMenuButtonKey = UniqueKey();
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(useMaterial3: false),
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  const PopupMenuItem<String>(
                    value: '0',
                    enabled: false,
                    child: ListTile(title: Text('Item 0')),
                  ),
                  const PopupMenuItem<String>(value: '1', child: ListTile(title: Text('Item 1'))),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    SafeArea getItemSafeArea(String label) {
      return tester.widget<SafeArea>(
        find.ancestor(of: find.text(label), matching: find.byType(SafeArea)),
      );
    }

    expect(getItemSafeArea('Item 0').minimum, EdgeInsets.zero);
    expect(getItemSafeArea('Item 1').minimum, EdgeInsets.zero);
  });

  testWidgets('Material3 - PopupMenuItem overrides ListTile.titleTextStyle', (
    WidgetTester tester,
  ) async {
    final Key popupMenuButtonKey = UniqueKey();
    ThemeData theme = ThemeData(useMaterial3: true);

    Widget buildMenu() {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  // Popup menu item with a Text widget.
                  const PopupMenuItem<String>(value: '0', child: Text('Item 0')),
                  // Popup menu item with a ListTile widget.
                  const PopupMenuItem<String>(value: '1', child: ListTile(title: Text('Item 1'))),
                ];
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildMenu());

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // Test popup menu item with a Text widget.
    expect(_labelStyle(tester, 'Item 0')!.fontSize, 14.0);
    expect(_labelStyle(tester, 'Item 0')!.color, theme.colorScheme.onSurface);

    // Test popup menu item with a ListTile widget.
    expect(_labelStyle(tester, 'Item 1')!.fontSize, 14.0);
    expect(_labelStyle(tester, 'Item 1')!.color, theme.colorScheme.onSurface);

    // Close the menu.
    await tester.tapAt(const Offset(20.0, 20.0));
    await tester.pumpAndSettle();

    // Test custom text theme.
    const TextStyle customTextStyle = TextStyle(
      fontSize: 20.0,
      fontWeight: FontWeight.bold,
      fontStyle: FontStyle.italic,
    );
    theme = theme.copyWith(textTheme: const TextTheme(labelLarge: customTextStyle));
    await tester.pumpWidget(buildMenu());

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // Test popup menu item with a Text widget with custom text theme.
    expect(_labelStyle(tester, 'Item 0')!.fontSize, customTextStyle.fontSize);
    expect(_labelStyle(tester, 'Item 0')!.fontWeight, customTextStyle.fontWeight);
    expect(_labelStyle(tester, 'Item 0')!.fontStyle, customTextStyle.fontStyle);

    // Test popup menu item with a ListTile widget with custom text theme.
    expect(_labelStyle(tester, 'Item 1')!.fontSize, customTextStyle.fontSize);
    expect(_labelStyle(tester, 'Item 1')!.fontWeight, customTextStyle.fontWeight);
    expect(_labelStyle(tester, 'Item 1')!.fontStyle, customTextStyle.fontStyle);
  });

  testWidgets('Material2 - PopupMenuItem overrides ListTile.titleTextStyle', (
    WidgetTester tester,
  ) async {
    final Key popupMenuButtonKey = UniqueKey();
    ThemeData theme = ThemeData(useMaterial3: false);

    Widget buildMenu() {
      return MaterialApp(
        theme: theme,
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              key: popupMenuButtonKey,
              child: const Text('button'),
              onSelected: (String result) {},
              itemBuilder: (BuildContext context) {
                return <PopupMenuEntry<String>>[
                  // Popup menu item with a Text widget.
                  const PopupMenuItem<String>(value: '0', child: Text('Item 0')),
                  // Popup menu item with a ListTile widget.
                  const PopupMenuItem<String>(value: '1', child: ListTile(title: Text('Item 1'))),
                ];
              },
            ),
          ),
        ),
      );
    }

    await tester.pumpWidget(buildMenu());

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // Test popup menu item with a Text widget.
    expect(_labelStyle(tester, 'Item 0')!.fontSize, 16.0);
    expect(_labelStyle(tester, 'Item 0')!.color, theme.textTheme.titleMedium!.color);

    // Test popup menu item with a ListTile widget.
    expect(_labelStyle(tester, 'Item 1')!.fontSize, 16.0);
    expect(_labelStyle(tester, 'Item 1')!.color, theme.textTheme.titleMedium!.color);

    // Close the menu.
    await tester.tapAt(const Offset(20.0, 20.0));
    await tester.pumpAndSettle();

    // Test custom text theme.
    const TextStyle customTextStyle = TextStyle(
      fontSize: 20.0,
      fontWeight: FontWeight.bold,
      fontStyle: FontStyle.italic,
    );
    theme = theme.copyWith(textTheme: const TextTheme(titleMedium: customTextStyle));
    await tester.pumpWidget(buildMenu());

    // Show the menu.
    await tester.tap(find.byKey(popupMenuButtonKey));
    await tester.pumpAndSettle();

    // Test popup menu item with a Text widget with custom text style.
    expect(_labelStyle(tester, 'Item 0')!.fontSize, customTextStyle.fontSize);
    expect(_labelStyle(tester, 'Item 0')!.fontWeight, customTextStyle.fontWeight);
    expect(_labelStyle(tester, 'Item 0')!.fontStyle, customTextStyle.fontStyle);

    // Test popup menu item with a ListTile widget with custom text style.
    expect(_labelStyle(tester, 'Item 1')!.fontSize, customTextStyle.fontSize);
    expect(_labelStyle(tester, 'Item 1')!.fontWeight, customTextStyle.fontWeight);
    expect(_labelStyle(tester, 'Item 1')!.fontStyle, customTextStyle.fontStyle);
  });

  testWidgets('CheckedPopupMenuItem.onTap callback is called when defined', (
    WidgetTester tester,
  ) async {
    int count = 0;
    await tester.pumpWidget(
      TestApp(
        textDirection: TextDirection.ltr,
        child: Material(
          child: RepaintBoundary(
            child: PopupMenuButton<String>(
              child: const Text('button'),
              itemBuilder: (BuildContext context) {
                return <PopupMenuItem<String>>[
                  CheckedPopupMenuItem<String>(
                    onTap: () {
                      count += 1;
                    },
                    value: 'item1',
                    child: const Text('Item with onTap'),
                  ),
                  const CheckedPopupMenuItem<String>(
                    value: 'item2',
                    child: Text('Item without onTap'),
                  ),
                ];
              },
            ),
          ),
        ),
      ),
    );

    // Tap a checked menu item with onTap.
    await tester.tap(find.text('button'));
    await tester.pumpAndSettle();
    await tester.tap(find.widgetWithText(CheckedPopupMenuItem<String>, 'Item with onTap'));
    await tester.pumpAndSettle();
    expect(count, 1);

    // Tap a checked menu item without onTap.
    await tester.tap(find.text('button'));
    await tester.pumpAndSettle();
    await tester.tap(find.widgetWithText(CheckedPopupMenuItem<String>, 'Item without onTap'));
    await tester.pumpAndSettle();
    expect(count, 1);
  });

  testWidgets('PopupMenuButton uses root navigator if useRootNavigator is true', (
    WidgetTester tester,
  ) async {
    final MenuObserver rootObserver = MenuObserver();
    final MenuObserver nestedObserver = MenuObserver();

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[nestedObserver],
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<dynamic>(
              builder: (BuildContext context) {
                return Material(
                  child: PopupMenuButton<String>(
                    useRootNavigator: true,
                    child: const Text('button'),
                    itemBuilder: (BuildContext context) {
                      return <PopupMenuItem<String>>[
                        const CheckedPopupMenuItem<String>(value: 'item1', child: Text('item 1')),
                        const CheckedPopupMenuItem<String>(value: 'item2', child: Text('item 2')),
                      ];
                    },
                  ),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.text('button'));

    expect(rootObserver.menuCount, 1);
    expect(nestedObserver.menuCount, 0);
  });

  testWidgets('PopupMenuButton does not use root navigator if useRootNavigator is false', (
    WidgetTester tester,
  ) async {
    final MenuObserver rootObserver = MenuObserver();
    final MenuObserver nestedObserver = MenuObserver();

    await tester.pumpWidget(
      MaterialApp(
        navigatorObservers: <NavigatorObserver>[rootObserver],
        home: Navigator(
          observers: <NavigatorObserver>[nestedObserver],
          onGenerateRoute: (RouteSettings settings) {
            return MaterialPageRoute<dynamic>(
              builder: (BuildContext context) {
                return Material(
                  child: PopupMenuButton<String>(
                    child: const Text('button'),
                    itemBuilder: (BuildContext context) {
                      return <PopupMenuItem<String>>[
                        const CheckedPopupMenuItem<String>(value: 'item1', child: Text('item 1')),
                        const CheckedPopupMenuItem<String>(value: 'item2', child: Text('item 2')),
                      ];
                    },
                  ),
                );
              },
            );
          },
        ),
      ),
    );

    // Open the dialog.
    await tester.tap(find.text('button'));

    expect(rootObserver.menuCount, 0);
    expect(nestedObserver.menuCount, 1);
  });

  testWidgets('Override Popup Menu animation using AnimationStyle', (WidgetTester tester) async {
    final Key targetKey = UniqueKey();

    Widget buildPopupMenu({AnimationStyle? animationStyle}) {
      return MaterialApp(
        home: Material(
          child: Center(
            child: PopupMenuButton<int>(
              key: targetKey,
              popUpAnimationStyle: animationStyle,
              itemBuilder: (BuildContext context) {
                return <PopupMenuItem<int>>[
                  const PopupMenuItem<int>(value: 1, child: Text('One')),
                  const PopupMenuItem<int>(value: 2, child: Text('Two')),
                  const PopupMenuItem<int>(value: 3, child: Text('Three')),
                ];
              },
            ),
          ),
        ),
      );
    }

    // Test default animation.
    await tester.pumpWidget(buildPopupMenu());

    await tester.tap(find.byKey(targetKey));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 100),
    ); // Advance the animation by 1/3 of its duration.

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(112.0, 80.0)),
    );

    await tester.pump(
      const Duration(milliseconds: 100),
    ); // Advance the animation by 2/3 of its duration.

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(112.0, 160.0)),
    );

    await tester.pumpAndSettle(); // Advance the animation to the end.

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(112.0, 160.0)),
    );

    // Tap outside to dismiss the menu.
    await tester.tapAt(const Offset(20.0, 20.0));
    await tester.pumpAndSettle();

    // Override the animation duration.
    await tester.pumpWidget(
      buildPopupMenu(animationStyle: AnimationStyle(duration: Duration.zero)),
    );

    await tester.tap(find.byKey(targetKey));
    await tester.pump();
    await tester.pump(const Duration(milliseconds: 1)); // Advance the animation by 1 millisecond.

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(112.0, 160.0)),
    );

    // Tap outside to dismiss the menu.
    await tester.tapAt(const Offset(20.0, 20.0));
    await tester.pumpAndSettle();

    // Override the animation curve.
    await tester.pumpWidget(
      buildPopupMenu(animationStyle: AnimationStyle(curve: Easing.emphasizedAccelerate)),
    );

    await tester.tap(find.byKey(targetKey));
    await tester.pump();
    await tester.pump(
      const Duration(milliseconds: 100),
    ); // Advance the animation by 1/3 of its duration.

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(32.4, 15.4)),
    );

    await tester.pump(
      const Duration(milliseconds: 100),
    ); // Advance the animation by 2/3 of its duration.

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(112.0, 72.2)),
    );

    await tester.pumpAndSettle(); // Advance the animation to the end.

    expect(
      tester.getSize(find.byType(Material).last),
      within(distance: 0.1, from: const Size(112.0, 160.0)),
    );
  });

  testWidgets('PopupMenuButton scrolls initial value/selected value to visible', (
    WidgetTester tester,
  ) async {
    const int length = 50;
    const int selectedValue = length - 1;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Align(
            alignment: Alignment.bottomCenter,
            child: PopupMenuButton<int>(
              itemBuilder: (BuildContext context) {
                return List<PopupMenuEntry<int>>.generate(length, (int index) {
                  return PopupMenuItem<int>(value: index, child: Text('item #$index'));
                });
              },
              popUpAnimationStyle: AnimationStyle.noAnimation,
              initialValue: selectedValue,
              child: const Text('click here'),
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.text('click here'));
    await tester.pump();

    // Set up finder and verify basic widget structure.
    final Finder item49 = find.text('item #49');
    expect(item49, findsOneWidget);

    // The initially selected menu item should be positioned on screen.
    final RenderBox initialItem = tester.renderObject<RenderBox>(item49);
    final Rect initialItemBounds = initialItem.localToGlobal(Offset.zero) & initialItem.size;
    final Size windowSize = tester.view.physicalSize / tester.view.devicePixelRatio;
    expect(initialItemBounds.bottomRight.dy, lessThanOrEqualTo(windowSize.height));

    // Select item 20.
    final Finder item20 = find.text('item #20');
    await tester.scrollUntilVisible(item20, 500);
    expect(item20, findsOneWidget);
    await tester.tap(item20);
    await tester.pump();

    // Open menu again.
    await tester.tap(find.text('click here'));
    await tester.pump();
    expect(item20, findsOneWidget);

    // The selected menu item should be positioned on screen.
    final RenderBox selectedItem = tester.renderObject<RenderBox>(item20);
    final Rect selectedItemBounds = selectedItem.localToGlobal(Offset.zero) & selectedItem.size;
    expect(selectedItemBounds.bottomRight.dy, lessThanOrEqualTo(windowSize.height));
  });

  testWidgets('PopupMenuButton properly positions a constrained-size popup', (
    WidgetTester tester,
  ) async {
    final Size windowSize = tester.view.physicalSize / tester.view.devicePixelRatio;
    const int length = 50;
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Padding(
            padding: const EdgeInsets.all(50),
            child: Align(
              alignment: Alignment.bottomCenter,
              child: PopupMenuButton<int>(
                itemBuilder: (BuildContext context) {
                  return List<PopupMenuEntry<int>>.generate(length, (int index) {
                    return PopupMenuItem<int>(value: index, child: Text('item #$index'));
                  });
                },
                constraints: BoxConstraints(maxHeight: windowSize.height / 3),
                popUpAnimationStyle: AnimationStyle.noAnimation,
                initialValue: length - 1,
                child: const Text('click here'),
              ),
            ),
          ),
        ),
      ),
    );
    await tester.tap(find.text('click here'));
    await tester.pump();

    // Set up finders and verify basic widget structure
    final Finder findButton = find.byType(PopupMenuButton<int>);
    final Finder findLastItem = find.text('item #49');
    final Finder findListBody = find.byType(ListBody);
    final Finder findListViewport = find.ancestor(
      of: findListBody,
      matching: find.byType(SingleChildScrollView),
    );
    expect(findButton, findsOne);
    expect(findLastItem, findsOne);
    expect(findListBody, findsOne);
    expect(findListViewport, findsOne);

    // The button and the list viewport should overlap
    final RenderBox button = tester.renderObject<RenderBox>(findButton);
    final Rect buttonBounds = button.localToGlobal(Offset.zero) & button.size;
    final RenderBox listViewport = tester.renderObject<RenderBox>(findListViewport);
    final Rect listViewportBounds = listViewport.localToGlobal(Offset.zero) & listViewport.size;
    expect(listViewportBounds.topLeft.dy, lessThanOrEqualTo(windowSize.height));
    expect(listViewportBounds.bottomRight.dy, lessThanOrEqualTo(windowSize.height));
    expect(listViewportBounds, overlaps(buttonBounds));
  });

  testWidgets('PopupMenuButton honors style', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: PopupMenuButton<int>(
            style: const ButtonStyle(iconColor: MaterialStatePropertyAll<Color>(Colors.red)),
            itemBuilder: (BuildContext context) {
              return <PopupMenuItem<int>>[const PopupMenuItem<int>(value: 1, child: Text('One'))];
            },
          ),
        ),
      ),
    );
    final RichText iconText = tester.firstWidget(
      find.descendant(of: find.byType(PopupMenuButton<int>), matching: find.byType(RichText)),
    );
    expect(iconText.text.style?.color, Colors.red);
  });

  testWidgets("Popup menu child's InkWell borderRadius", (WidgetTester tester) async {
    final BorderRadius borderRadius = BorderRadius.circular(20);

    Widget buildPopupMenu({required BorderRadius? borderRadius}) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              borderRadius: borderRadius,
              itemBuilder:
                  (_) => <PopupMenuEntry<String>>[
                    const PopupMenuItem<String>(value: 'value', child: Text('Item 0')),
                  ],
              child: const Row(
                mainAxisSize: MainAxisSize.min,
                children: <Widget>[Text('Pop up menu'), Icon(Icons.arrow_drop_down)],
              ),
            ),
          ),
        ),
      );
    }

    // Popup menu with default null borderRadius.
    await tester.pumpWidget(buildPopupMenu(borderRadius: null));
    await tester.pumpAndSettle();

    InkWell inkWell = tester.widget<InkWell>(find.byType(InkWell));
    expect(inkWell.borderRadius, isNull);

    // Popup menu with fixed borderRadius.
    await tester.pumpWidget(buildPopupMenu(borderRadius: borderRadius));
    await tester.pumpAndSettle();

    inkWell = tester.widget<InkWell>(find.byType(InkWell));
    expect(inkWell.borderRadius, borderRadius);
  });

  testWidgets('PopupMenuButton respects materialTapTargetSize', (WidgetTester tester) async {
    const double buttonSize = 10.0;

    Widget buildPopupMenu({required MaterialTapTargetSize tapTargetSize}) {
      return MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<String>(
              style: ButtonStyle(tapTargetSize: tapTargetSize),
              itemBuilder:
                  (_) => <PopupMenuEntry<String>>[
                    const PopupMenuItem<String>(value: 'value', child: Text('Item 0')),
                  ],
              child: const SizedBox(height: buttonSize, width: buttonSize),
            ),
          ),
        ),
      );
    }

    // Popup menu with MaterialTapTargetSize.padded.
    await tester.pumpWidget(buildPopupMenu(tapTargetSize: MaterialTapTargetSize.padded));
    await tester.pumpAndSettle();

    expect(tester.getSize(find.byType(InkWell)), const Size(48.0, 48.0));

    // Popup menu with MaterialTapTargetSize.shrinkWrap.
    await tester.pumpWidget(buildPopupMenu(tapTargetSize: MaterialTapTargetSize.shrinkWrap));
    await tester.pumpAndSettle();

    expect(tester.getSize(find.byType(InkWell)), const Size(buttonSize, buttonSize));
  });

  testWidgets(
    'If requestFocus is false, the original focus should be preserved upon menu appearance.',
    (WidgetTester tester) async {
      final FocusNode fieldFocusNode = FocusNode();
      addTearDown(fieldFocusNode.dispose);
      await tester.pumpWidget(
        MaterialApp(
          home: Scaffold(
            body: Column(
              children: <Widget>[
                TextField(focusNode: fieldFocusNode, autofocus: true),
                PopupMenuButton<int>(
                  style: const ButtonStyle(iconColor: MaterialStatePropertyAll<Color>(Colors.red)),
                  itemBuilder: (BuildContext context) {
                    return <PopupMenuItem<int>>[
                      const PopupMenuItem<int>(value: 1, child: Text('One')),
                    ];
                  },
                  requestFocus: false,
                  child: const Text('click here'),
                ),
              ],
            ),
          ),
        ),
      );
      expect(fieldFocusNode.hasFocus, isTrue);
      await tester.tap(find.text('click here'));
      await tester.pump();
      expect(fieldFocusNode.hasFocus, isTrue);
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/152475
  testWidgets('PopupMenuButton updates position on orientation change', (
    WidgetTester tester,
  ) async {
    const Size initialSize = Size(400, 800);
    const Size newSize = Size(1024, 768);

    await tester.binding.setSurfaceSize(initialSize);

    final GlobalKey buttonKey = GlobalKey();

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: PopupMenuButton<int>(
              key: buttonKey,
              itemBuilder:
                  (BuildContext context) => <PopupMenuItem<int>>[
                    const PopupMenuItem<int>(value: 1, child: Text('Option 1')),
                  ],
            ),
          ),
        ),
      ),
    );

    await tester.tap(find.byType(PopupMenuButton<int>));
    await tester.pumpAndSettle();

    final Rect initialButtonRect = tester.getRect(find.byKey(buttonKey));
    final Rect initialMenuRect = tester.getRect(find.text('Option 1'));

    await tester.binding.setSurfaceSize(newSize);
    await tester.pumpAndSettle();

    final Rect newButtonRect = tester.getRect(find.byKey(buttonKey));
    final Rect newMenuRect = tester.getRect(find.text('Option 1'));

    expect(newButtonRect, isNot(equals(initialButtonRect)));

    expect(newMenuRect, isNot(equals(initialMenuRect)));

    expect(
      newMenuRect.topLeft - newButtonRect.topLeft,
      initialMenuRect.topLeft - initialButtonRect.topLeft,
    );

    await tester.binding.setSurfaceSize(null);
  });
}

Matcher overlaps(Rect other) => OverlapsMatcher(other);

class OverlapsMatcher extends Matcher {
  OverlapsMatcher(this.other);

  final Rect other;

  @override
  Description describe(Description description) {
    return description.add('<Rect that overlaps with $other>');
  }

  @override
  bool matches(Object? item, Map<dynamic, dynamic> matchState) =>
      item is Rect && item.overlaps(other);

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    return mismatchDescription.add('does not overlap');
  }
}

class TestApp extends StatelessWidget {
  const TestApp({super.key, required this.textDirection, this.child});

  final TextDirection textDirection;
  final Widget? child;

  @override
  Widget build(BuildContext context) {
    return Localizations(
      locale: const Locale('en', 'US'),
      delegates: const <LocalizationsDelegate<dynamic>>[
        DefaultWidgetsLocalizations.delegate,
        DefaultMaterialLocalizations.delegate,
      ],
      child: MediaQuery(
        data: MediaQueryData.fromView(View.of(context)),
        child: Directionality(
          textDirection: textDirection,
          child: Navigator(
            onGenerateRoute: (RouteSettings settings) {
              assert(settings.name == '/');
              return MaterialPageRoute<void>(
                settings: settings,
                builder: (BuildContext context) => Material(child: child),
              );
            },
          ),
        ),
      ),
    );
  }
}

class MenuObserver extends NavigatorObserver {
  int menuCount = 0;

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) {
    if (route.toString().contains('_PopupMenuRoute')) {
      menuCount++;
    }
    super.didPush(route, previousRoute);
  }
}

class _ClosureNavigatorObserver extends NavigatorObserver {
  _ClosureNavigatorObserver({required this.onDidChange});

  final void Function(Route<dynamic> newRoute) onDidChange;

  @override
  void didPush(Route<dynamic> route, Route<dynamic>? previousRoute) => onDidChange(route);

  @override
  void didPop(Route<dynamic> route, Route<dynamic>? previousRoute) => onDidChange(previousRoute!);

  @override
  void didRemove(Route<dynamic> route, Route<dynamic>? previousRoute) =>
      onDidChange(previousRoute!);

  @override
  void didReplace({Route<dynamic>? newRoute, Route<dynamic>? oldRoute}) => onDidChange(newRoute!);
}

TextStyle? _labelStyle(WidgetTester tester, String label) {
  return tester
      .widget<RichText>(find.descendant(of: find.text(label), matching: find.byType(RichText)))
      .text
      .style;
}

TextStyle? _iconStyle(WidgetTester tester, IconData icon) {
  return tester
      .widget<RichText>(find.descendant(of: find.byIcon(icon), matching: find.byType(RichText)))
      .text
      .style;
}
