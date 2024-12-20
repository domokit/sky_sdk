// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@TestOn('!chrome')
library;

import 'dart:async';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  late FakeBuilder mockHelper;

  setUp(() {
    mockHelper = FakeBuilder();
  });

  int testListLength = 10;
  SliverList buildAListOfStuff() {
    return SliverList(
      delegate: SliverChildBuilderDelegate((BuildContext context, int index) {
        return SizedBox(height: 200.0, child: Center(child: Text(index.toString())));
      }, childCount: testListLength),
    );
  }

  void uiTestGroup() {
    testWidgets(
      "doesn't invoke anything without user interaction",
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(builder: mockHelper.builder),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        expect(mockHelper.invocations, isEmpty);

        expect(tester.getTopLeft(find.widgetWithText(SizedBox, '0')), Offset.zero);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'calls the indicator builder when starting to overscroll',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(builder: mockHelper.builder),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        // Drag down but not enough to trigger the refresh.
        await tester.drag(find.text('0'), const Offset(0.0, 50.0), touchSlopY: 0);
        await tester.pump();

        // The function is referenced once while passing into CupertinoSliverRefreshControl
        // and is called.
        expect(
          mockHelper.invocations.first,
          matchesBuilder(
            refreshState: RefreshIndicatorMode.drag,
            pulledExtent: 50,
            refreshTriggerPullDistance: 100, // default value.
            refreshIndicatorExtent: 60, // default value.
          ),
        );
        expect(mockHelper.invocations, hasLength(1));

        expect(tester.getTopLeft(find.widgetWithText(SizedBox, '0')), const Offset(0.0, 50.0));
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      "don't call the builder if overscroll doesn't move slivers like on Android",
      (WidgetTester tester) async {
        await tester.pumpWidget(
          Directionality(
            textDirection: TextDirection.ltr,
            child: MediaQuery(
              data: const MediaQueryData(),
              child: CustomScrollView(
                slivers: <Widget>[
                  CupertinoSliverRefreshControl(builder: mockHelper.builder),
                  buildAListOfStuff(),
                ],
              ),
            ),
          ),
        );

        // Drag down but not enough to trigger the refresh.
        await tester.drag(find.text('0'), const Offset(0.0, 50.0));
        await tester.pump();

        expect(mockHelper.invocations, isEmpty);

        expect(tester.getTopLeft(find.widgetWithText(SizedBox, '0')), Offset.zero);
      },
      variant: TargetPlatformVariant.only(TargetPlatform.android),
    );

    testWidgets(
      'let the builder update as canceled drag scrolls away',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(builder: mockHelper.builder),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        // Drag down but not enough to trigger the refresh.
        await tester.drag(find.text('0'), const Offset(0.0, 50.0), touchSlopY: 0);
        await tester.pump();
        await tester.pump(const Duration(milliseconds: 20));
        await tester.pump(const Duration(milliseconds: 20));
        await tester.pump(const Duration(seconds: 3));

        expect(
          mockHelper.invocations,
          containsAllInOrder(<void>[
            matchesBuilder(
              refreshState: RefreshIndicatorMode.drag,
              pulledExtent: 50,
              refreshTriggerPullDistance: 100, // default value.
              refreshIndicatorExtent: 60, // default value.
            ),
            if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS)
              matchesBuilder(
                refreshState: RefreshIndicatorMode.drag,
                pulledExtent: moreOrLessEquals(48.07979523362715),
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              )
            else
              matchesBuilder(
                refreshState: RefreshIndicatorMode.drag,
                pulledExtent: moreOrLessEquals(48.36801747187993),
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              ),
            if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS)
              matchesBuilder(
                refreshState: RefreshIndicatorMode.drag,
                pulledExtent: moreOrLessEquals(43.98499220391114),
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              )
            else
              matchesBuilder(
                refreshState: RefreshIndicatorMode.drag,
                pulledExtent: moreOrLessEquals(44.63031931875867),
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              ),
          ]),
        );
        // The builder isn't called again when the sliver completely goes away.
        expect(mockHelper.invocations, hasLength(3));

        expect(tester.getTopLeft(find.widgetWithText(SizedBox, '0')), Offset.zero);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'drag past threshold triggers refresh task',
      (WidgetTester tester) async {
        final List<MethodCall> platformCallLog = <MethodCall>[];

        tester.binding.defaultBinaryMessenger.setMockMethodCallHandler(SystemChannels.platform, (
          MethodCall methodCall,
        ) async {
          platformCallLog.add(methodCall);
          return null;
        });

        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        final TestGesture gesture = await tester.startGesture(Offset.zero);
        await gesture.moveBy(const Offset(0.0, 99.0));
        await tester.pump();
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          await gesture.moveBy(const Offset(0.0, -3.0));
        } else {
          await gesture.moveBy(const Offset(0.0, -30.0));
        }
        await tester.pump();
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          await gesture.moveBy(const Offset(0.0, 90.0));
        } else {
          await gesture.moveBy(const Offset(0.0, 50.0));
        }
        await tester.pump();

        expect(
          mockHelper.invocations,
          containsAllInOrder(<void>[
            matchesBuilder(
              refreshState: RefreshIndicatorMode.drag,
              pulledExtent: 99,
              refreshTriggerPullDistance: 100, // default value.
              refreshIndicatorExtent: 60, // default value.
            ),
            if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS)
              matchesBuilder(
                refreshState: RefreshIndicatorMode.drag,
                pulledExtent: moreOrLessEquals(96),
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              )
            else
              matchesBuilder(
                refreshState: RefreshIndicatorMode.drag,
                pulledExtent: moreOrLessEquals(86.78169),
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              ),
            if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS)
              matchesBuilder(
                refreshState: RefreshIndicatorMode.armed,
                pulledExtent: moreOrLessEquals(112.51104),
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              )
            else
              matchesBuilder(
                refreshState: RefreshIndicatorMode.armed,
                pulledExtent: moreOrLessEquals(105.80452021305739),
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              ),
          ]),
        );
        // The refresh callback is triggered after the frame.
        expect(mockHelper.invocations.last, const RefreshTaskInvocation());
        expect(mockHelper.invocations, hasLength(4));

        expect(
          platformCallLog.last,
          isMethodCall('HapticFeedback.vibrate', arguments: 'HapticFeedbackType.mediumImpact'),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'refreshing task keeps the sliver expanded forever until done',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        await tester.drag(find.text('0'), const Offset(0.0, 150.0), touchSlopY: 0);
        await tester.pump();
        // Let it start snapping back.
        await tester.pump(const Duration(milliseconds: 50));

        expect(
          mockHelper.invocations,
          containsAllInOrder(<Matcher>[
            matchesBuilder(
              refreshState: RefreshIndicatorMode.armed,
              pulledExtent: 150,
              refreshTriggerPullDistance: 100, // Default value.
              refreshIndicatorExtent: 60, // Default value.
            ),
            equals(const RefreshTaskInvocation()),
            if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS)
              matchesBuilder(
                refreshState: RefreshIndicatorMode.armed,
                pulledExtent: moreOrLessEquals(124.87933920045268),
                refreshTriggerPullDistance: 100, // Default value.
                refreshIndicatorExtent: 60, // Default value.
              )
            else
              matchesBuilder(
                refreshState: RefreshIndicatorMode.armed,
                pulledExtent: moreOrLessEquals(127.10396988577114),
                refreshTriggerPullDistance: 100, // Default value.
                refreshIndicatorExtent: 60, // Default value.
              ),
          ]),
        );

        // Reaches refresh state and sliver's at 60.0 in height after a while.
        await tester.pump(const Duration(seconds: 1));

        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.refresh,
              pulledExtent: 60,
              refreshIndicatorExtent: 60, // Default value.
              refreshTriggerPullDistance: 100, // Default value.
            ),
          ),
        );

        // Stays in that state forever until future completes.
        await tester.pump(const Duration(seconds: 1000));
        expect(tester.getTopLeft(find.widgetWithText(SizedBox, '0')), const Offset(0.0, 60.0));

        mockHelper.refreshCompleter.complete(null);
        await tester.pump();

        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.done,
              pulledExtent: 60,
              refreshIndicatorExtent: 60, // Default value.
              refreshTriggerPullDistance: 100, // Default value.
            ),
          ),
        );
        expect(mockHelper.invocations, hasLength(5));
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'refreshing task keeps the sliver expanded forever until completes with error',
      (WidgetTester tester) async {
        final FlutterError error = FlutterError('Oops');
        double errorCount = 0;
        final TargetPlatform? platform =
            debugDefaultTargetPlatformOverride; // Will not be correct within the zone.

        runZonedGuarded(
          () async {
            mockHelper.refreshCompleter = Completer<void>.sync();
            await tester.pumpWidget(
              CupertinoApp(
                home: CustomScrollView(
                  slivers: <Widget>[
                    CupertinoSliverRefreshControl(
                      builder: mockHelper.builder,
                      onRefresh: mockHelper.refreshTask,
                    ),
                    buildAListOfStuff(),
                  ],
                ),
              ),
            );

            await tester.drag(find.text('0'), const Offset(0.0, 150.0), touchSlopY: 0);
            await tester.pump();
            // Let it start snapping back.
            await tester.pump(const Duration(milliseconds: 50));

            expect(
              mockHelper.invocations,
              containsAllInOrder(<Matcher>[
                matchesBuilder(
                  refreshState: RefreshIndicatorMode.armed,
                  pulledExtent: 150,
                  refreshIndicatorExtent: 60, // Default value.
                  refreshTriggerPullDistance: 100, // Default value.
                ),
                equals(const RefreshTaskInvocation()),
                if (platform == TargetPlatform.macOS)
                  matchesBuilder(
                    refreshState: RefreshIndicatorMode.armed,
                    pulledExtent: moreOrLessEquals(124.87933920045268),
                    refreshTriggerPullDistance: 100, // Default value.
                    refreshIndicatorExtent: 60, // Default value.
                  )
                else
                  matchesBuilder(
                    refreshState: RefreshIndicatorMode.armed,
                    pulledExtent: moreOrLessEquals(127.10396988577114),
                    refreshIndicatorExtent: 60, // Default value.
                    refreshTriggerPullDistance: 100, // Default value.
                  ),
              ]),
            );

            // Reaches refresh state and sliver's at 60.0 in height after a while.
            await tester.pump(const Duration(seconds: 1));
            expect(
              mockHelper.invocations,
              contains(
                matchesBuilder(
                  refreshState: RefreshIndicatorMode.refresh,
                  pulledExtent: 60,
                  refreshIndicatorExtent: 60, // Default value.
                  refreshTriggerPullDistance: 100, // Default value.
                ),
              ),
            );

            // Stays in that state forever until future completes.
            await tester.pump(const Duration(seconds: 1000));
            expect(tester.getTopLeft(find.widgetWithText(SizedBox, '0')), const Offset(0.0, 60.0));

            mockHelper.refreshCompleter.completeError(error);
            await tester.pump();

            expect(
              mockHelper.invocations,
              contains(
                matchesBuilder(
                  refreshState: RefreshIndicatorMode.done,
                  pulledExtent: 60,
                  refreshIndicatorExtent: 60, // Default value.
                  refreshTriggerPullDistance: 100, // Default value.
                ),
              ),
            );
            expect(mockHelper.invocations, hasLength(5));
          },
          (Object e, StackTrace stack) {
            expect(e, error);
            expect(errorCount, 0);
            errorCount++;
          },
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'expanded refreshing sliver scrolls normally',
      (WidgetTester tester) async {
        mockHelper.refreshIndicator = const Center(child: Text('-1'));

        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        await tester.drag(find.text('0'), const Offset(0.0, 150.0), touchSlopY: 0);
        await tester.pump();

        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.armed,
              pulledExtent: 150,
              refreshIndicatorExtent: 60, // Default value.
              refreshTriggerPullDistance: 100, // Default value.
            ),
          ),
        );

        // Given a box constraint of 150, the Center will occupy all that height.
        expect(
          tester.getRect(find.widgetWithText(Center, '-1')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 150.0),
        );

        await tester.drag(
          find.text('0'),
          const Offset(0.0, -300.0),
          touchSlopY: 0,
          warnIfMissed: false,
        ); // hits the list
        await tester.pump();

        // Refresh indicator still being told to layout the same way.
        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.refresh,
              pulledExtent: 60,
              refreshIndicatorExtent: 60, // Default value.
              refreshTriggerPullDistance: 100, // Default value.
            ),
          ),
        );

        // Now the sliver is scrolled off screen.
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          expect(
            tester.getTopLeft(find.widgetWithText(Center, '-1', skipOffstage: false)).dy,
            moreOrLessEquals(-210.0),
          );
          expect(
            tester.getBottomLeft(find.widgetWithText(Center, '-1', skipOffstage: false)).dy,
            moreOrLessEquals(-150.0),
          );
          expect(tester.getTopLeft(find.widgetWithText(Center, '0')).dy, moreOrLessEquals(-150.0));
        } else {
          expect(
            tester.getTopLeft(find.widgetWithText(Center, '-1', skipOffstage: false)).dy,
            moreOrLessEquals(-175.38461538461536),
          );
          expect(
            tester.getBottomLeft(find.widgetWithText(Center, '-1', skipOffstage: false)).dy,
            moreOrLessEquals(-115.38461538461536),
          );
          expect(
            tester.getTopLeft(find.widgetWithText(Center, '0')).dy,
            moreOrLessEquals(-115.38461538461536),
          );
        }

        // Scroll the top of the refresh indicator back to overscroll, it will
        // snap to the size of the refresh indicator and stay there.
        await tester.drag(
          find.text('1'),
          const Offset(0.0, 200.0),
          warnIfMissed: false,
        ); // hits the list
        await tester.pump();
        await tester.pump(const Duration(seconds: 2));
        expect(
          tester.getRect(find.widgetWithText(Center, '-1')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 60.0),
        );
        expect(
          tester.getRect(find.widgetWithText(Center, '0')),
          const Rect.fromLTRB(0.0, 60.0, 800.0, 260.0),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'expanded refreshing sliver goes away when done',
      (WidgetTester tester) async {
        mockHelper.refreshIndicator = const Center(child: Text('-1'));

        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        await tester.drag(find.text('0'), const Offset(0.0, 150.0), touchSlopY: 0);
        await tester.pump();
        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.armed,
              pulledExtent: 150,
              refreshIndicatorExtent: 60, // Default value.
              refreshTriggerPullDistance: 100, // Default value.
            ),
          ),
        );
        expect(
          tester.getRect(find.widgetWithText(Center, '-1')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 150.0),
        );
        expect(mockHelper.invocations, contains(const RefreshTaskInvocation()));

        // Rebuilds the sliver with a layout extent now.
        await tester.pump();
        // Let it snap back to occupy the indicator's final sliver space only.
        await tester.pump(const Duration(seconds: 2));

        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.refresh,
              pulledExtent: 60,
              refreshIndicatorExtent: 60, // Default value.
              refreshTriggerPullDistance: 100, // Default value.
            ),
          ),
        );
        expect(
          tester.getRect(find.widgetWithText(Center, '-1')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 60.0),
        );
        expect(
          tester.getRect(find.widgetWithText(Center, '0')),
          const Rect.fromLTRB(0.0, 60.0, 800.0, 260.0),
        );

        mockHelper.refreshCompleter.complete(null);
        await tester.pump();
        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.done,
              pulledExtent: 60,
              refreshIndicatorExtent: 60, // Default value.
              refreshTriggerPullDistance: 100, // Default value.
            ),
          ),
        );

        await tester.pump(const Duration(seconds: 5));
        expect(find.text('-1'), findsNothing);
        expect(
          tester.getRect(find.widgetWithText(Center, '0')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'builder still called when sliver snapped back more than 90%',
      (WidgetTester tester) async {
        mockHelper.refreshIndicator = const Center(child: Text('-1'));

        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        await tester.drag(find.text('0'), const Offset(0.0, 150.0), touchSlopY: 0);
        await tester.pump();
        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.armed,
              pulledExtent: 150,
              refreshTriggerPullDistance: 100, // default value.
              refreshIndicatorExtent: 60, // default value.
            ),
          ),
        );
        expect(
          tester.getRect(find.widgetWithText(Center, '-1')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 150.0),
        );
        expect(mockHelper.invocations, contains(const RefreshTaskInvocation()));

        // Rebuilds the sliver with a layout extent now.
        await tester.pump();
        // Let it snap back to occupy the indicator's final sliver space only.
        await tester.pump(const Duration(seconds: 2));
        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.refresh,
              pulledExtent: 60,
              refreshTriggerPullDistance: 100, // default value.
              refreshIndicatorExtent: 60, // default value.
            ),
          ),
        );
        expect(
          tester.getRect(find.widgetWithText(Center, '-1')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 60.0),
        );
        expect(
          tester.getRect(find.widgetWithText(Center, '0')),
          const Rect.fromLTRB(0.0, 60.0, 800.0, 260.0),
        );

        mockHelper.refreshCompleter.complete(null);
        await tester.pump();

        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.done,
              pulledExtent: 60,
              refreshTriggerPullDistance: 100, // default value.
              refreshIndicatorExtent: 60, // default value.
            ),
          ),
        );

        // Waiting for refresh control to reach approximately 5% of height
        await tester.pump(const Duration(milliseconds: 400));

        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          expect(
            tester.getRect(find.widgetWithText(Center, '0')).top,
            moreOrLessEquals(3.9543032206542765, epsilon: 4e-1),
          );
          expect(
            tester.getRect(find.widgetWithText(Center, '-1')).height,
            moreOrLessEquals(3.9543032206542765, epsilon: 4e-1),
          );
          expect(
            mockHelper.invocations,
            contains(
              matchesBuilder(
                refreshState: RefreshIndicatorMode.inactive,
                pulledExtent: 3.9543032206542765, // ~5% of 60.0
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              ),
            ),
          );
        } else {
          expect(
            tester.getRect(find.widgetWithText(Center, '0')).top,
            moreOrLessEquals(3.0, epsilon: 4e-1),
          );
          expect(
            tester.getRect(find.widgetWithText(Center, '-1')).height,
            moreOrLessEquals(3.0, epsilon: 4e-1),
          );
          expect(
            mockHelper.invocations,
            contains(
              matchesBuilder(
                refreshState: RefreshIndicatorMode.inactive,
                pulledExtent: 2.6980688300546443, // ~5% of 60.0
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              ),
            ),
          );
        }
        expect(find.text('-1'), findsOneWidget);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'retracting sliver during done cannot be pulled to refresh again until fully retracted',
      (WidgetTester tester) async {
        mockHelper.refreshIndicator = const Center(child: Text('-1'));

        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        await tester.drag(find.text('0'), const Offset(0.0, 150.0), pointer: 1, touchSlopY: 0.0);
        await tester.pump();
        expect(mockHelper.invocations, contains(const RefreshTaskInvocation()));

        mockHelper.refreshCompleter.complete(null);
        await tester.pump();
        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.done,
              pulledExtent: 150.0, // Still overscrolled here.
              refreshTriggerPullDistance: 100, // default value.
              refreshIndicatorExtent: 60, // default value.
            ),
          ),
        );

        // Let it start going away but not fully.
        await tester.pump(const Duration(milliseconds: 100));
        // The refresh indicator is still building.
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          expect(
            mockHelper.invocations,
            contains(
              matchesBuilder(
                refreshState: RefreshIndicatorMode.done,
                pulledExtent: 90.13497854600749,
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              ),
            ),
          );
          expect(
            tester.getBottomLeft(find.widgetWithText(Center, '-1')).dy,
            moreOrLessEquals(90.13497854600749),
          );
        } else {
          expect(
            mockHelper.invocations,
            contains(
              matchesBuilder(
                refreshState: RefreshIndicatorMode.done,
                pulledExtent: 91.31180913199277,
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              ),
            ),
          );
          expect(
            tester.getBottomLeft(find.widgetWithText(Center, '-1')).dy,
            moreOrLessEquals(91.311809131992776),
          );
        }

        // Start another drag by an amount that would have been enough to
        // trigger another refresh if it were in the right state.
        await tester.drag(
          find.text('0'),
          const Offset(0.0, 150.0),
          pointer: 1,
          touchSlopY: 0.0,
          warnIfMissed: false,
        );
        await tester.pump();

        // Instead, it's still in the done state because the sliver never
        // fully retracted.
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          expect(
            mockHelper.invocations,
            contains(
              matchesBuilder(
                refreshState: RefreshIndicatorMode.done,
                pulledExtent: 118.29756539042118,
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              ),
            ),
          );
        } else {
          expect(
            mockHelper.invocations,
            contains(
              matchesBuilder(
                refreshState: RefreshIndicatorMode.done,
                pulledExtent: 147.3772721631821,
                refreshTriggerPullDistance: 100, // default value.
                refreshIndicatorExtent: 60, // default value.
              ),
            ),
          );
        }

        // Now let it fully go away.
        await tester.pump(const Duration(seconds: 5));
        expect(find.text('-1'), findsNothing);
        expect(
          tester.getRect(find.widgetWithText(Center, '0')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
        );

        // Start another drag. It's now in drag mode.
        await tester.drag(find.text('0'), const Offset(0.0, 40.0), pointer: 1, touchSlopY: 0.0);
        await tester.pump();
        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.drag,
              pulledExtent: 40,
              refreshTriggerPullDistance: 100, // default value.
              refreshIndicatorExtent: 60, // default value.
            ),
          ),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'sliver held in overscroll when task finishes completes normally',
      (WidgetTester tester) async {
        mockHelper.refreshIndicator = const Center(child: Text('-1'));

        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        final TestGesture gesture = await tester.startGesture(Offset.zero);
        // Start a refresh.
        await gesture.moveBy(const Offset(0.0, 150.0));
        await tester.pump();
        expect(mockHelper.invocations, contains(const RefreshTaskInvocation()));

        // Complete the task while held down.
        mockHelper.refreshCompleter.complete(null);
        await tester.pump();

        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.done,
              pulledExtent: 150.0, // Still overscrolled here.
              refreshTriggerPullDistance: 100, // default value.
              refreshIndicatorExtent: 60, // default value.
            ),
          ),
        );
        expect(
          tester.getRect(find.widgetWithText(Center, '0')),
          const Rect.fromLTRB(0.0, 150.0, 800.0, 350.0),
        );

        await gesture.up();
        await tester.pump();
        await tester.pump(const Duration(seconds: 5));
        expect(find.text('-1'), findsNothing);
        expect(
          tester.getRect(find.widgetWithText(Center, '0')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'sliver scrolled away when task completes properly removes itself',
      (WidgetTester tester) async {
        if (testListLength < 4) {
          // This test only makes sense when the list is long enough that
          // the indicator can be scrolled away while refreshing.
          return;
        }
        mockHelper.refreshIndicator = const Center(child: Text('-1'));

        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        // Start a refresh.
        await tester.drag(find.text('0'), const Offset(0.0, 150.0));
        await tester.pump();
        expect(mockHelper.invocations, contains(const RefreshTaskInvocation()));

        await tester.drag(find.text('0'), const Offset(0.0, -300.0));
        await tester.pump();

        // Refresh indicator still being told to layout the same way.
        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.done,
              pulledExtent: 60,
              refreshTriggerPullDistance: 100, // default value.
              refreshIndicatorExtent: 60, // default value.
            ),
          ),
        );

        // Now the sliver is scrolled off screen.
        expect(
          tester.getTopLeft(find.widgetWithText(Center, '-1', skipOffstage: false)).dy,
          moreOrLessEquals(-175.38461538461536),
        );
        expect(
          tester.getBottomLeft(find.widgetWithText(Center, '-1', skipOffstage: false)).dy,
          moreOrLessEquals(-115.38461538461536),
        );

        // Complete the task while scrolled away.
        mockHelper.refreshCompleter.complete(null);
        // The sliver is instantly gone since there is no overscroll physics
        // simulation.
        await tester.pump();

        // The next item's position is not disturbed.
        expect(
          tester.getTopLeft(find.widgetWithText(Center, '0')).dy,
          moreOrLessEquals(-115.38461538461536),
        );

        // Scrolling past the first item still results in a new overscroll.
        // The layout extent is gone.
        await tester.drag(find.text('1'), const Offset(0.0, 120.0));
        await tester.pump();

        expect(
          mockHelper.invocations,
          contains(
            matchesBuilder(
              refreshState: RefreshIndicatorMode.done,
              pulledExtent: 4.615384615384642,
              refreshTriggerPullDistance: 100, // default value.
              refreshIndicatorExtent: 60, // default value.
            ),
          ),
        );

        // Snaps away normally.
        await tester.pump();
        await tester.pump(const Duration(seconds: 2));
        expect(find.text('-1'), findsNothing);
        expect(
          tester.getRect(find.widgetWithText(Center, '0')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      "don't do anything unless it can be overscrolled at the start of the list",
      (WidgetTester tester) async {
        mockHelper.refreshIndicator = const Center(child: Text('-1'));

        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                buildAListOfStuff(),
                CupertinoSliverRefreshControl(
                  // it's in the middle now.
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        await tester.fling(find.byType(SizedBox).first, const Offset(0.0, 200.0), 2000.0);
        await tester.fling(
          find.byType(SizedBox).first,
          const Offset(0.0, -200.0),
          3000.0,
          warnIfMissed: false,
        ); // IgnorePointer is enabled while scroll is ballistic.

        expect(mockHelper.invocations, isEmpty);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'without an onRefresh, builder is called with arm for one frame then sliver goes away',
      (WidgetTester tester) async {
        mockHelper.refreshIndicator = const Center(child: Text('-1'));

        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(builder: mockHelper.builder),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        await tester.drag(find.text('0'), const Offset(0.0, 150.0), touchSlopY: 0.0);
        await tester.pump();

        expect(
          mockHelper.invocations.first,
          matchesBuilder(
            refreshState: RefreshIndicatorMode.armed,
            pulledExtent: 150.0,
            refreshTriggerPullDistance: 100.0, // Default value.
            refreshIndicatorExtent: 60.0, // Default value.
          ),
        );

        await tester.pump(const Duration(milliseconds: 10));

        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          expect(
            mockHelper.invocations.last,
            matchesBuilder(
              refreshState: RefreshIndicatorMode.done,
              pulledExtent: moreOrLessEquals(148.36088180097366),
              refreshTriggerPullDistance: 100.0, // Default value.
              refreshIndicatorExtent: 60.0, // Default value.
            ),
          );
        } else {
          expect(
            mockHelper.invocations.last,
            matchesBuilder(
              refreshState: RefreshIndicatorMode.done,
              pulledExtent: moreOrLessEquals(148.6463892921364),
              refreshTriggerPullDistance: 100.0, // Default value.
              refreshIndicatorExtent: 60.0, // Default value.
            ),
          );
        }

        await tester.pump(const Duration(seconds: 5));
        expect(find.text('-1'), findsNothing);
        expect(
          tester.getRect(find.widgetWithText(Center, '0')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'Should not crash when dragged',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              physics: const BouncingScrollPhysics(),
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  onRefresh: () async => Future<void>.delayed(const Duration(days: 2000)),
                ),
              ],
            ),
          ),
        );

        await tester.dragFrom(const Offset(100, 10), const Offset(0.0, 50.0), touchSlopY: 0);
        await tester.pump();

        await tester.dragFrom(const Offset(100, 10), const Offset(0, 500), touchSlopY: 0);
        await tester.pump();

        expect(tester.takeException(), isNull);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    // Test to make sure the refresh sliver's overscroll isn't eaten by the
    // nav bar sliver https://github.com/flutter/flutter/issues/74516.
    testWidgets(
      'properly displays when the refresh sliver is behind the large title nav bar sliver',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                const CupertinoSliverNavigationBar(largeTitle: Text('Title')),
                CupertinoSliverRefreshControl(builder: mockHelper.builder),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        final double initialFirstCellY = tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy;

        // Drag down but not enough to trigger the refresh.
        await tester.drag(find.text('0'), const Offset(0.0, 50.0), touchSlopY: 0);
        await tester.pump();

        expect(
          mockHelper.invocations.first,
          matchesBuilder(
            refreshState: RefreshIndicatorMode.drag,
            pulledExtent: 50,
            refreshTriggerPullDistance: 100, // default value.
            refreshIndicatorExtent: 60, // default value.
          ),
        );
        expect(mockHelper.invocations, hasLength(1));

        expect(tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy, initialFirstCellY + 50);
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );
  }

  void stateMachineTestGroup() {
    testWidgets(
      'starts in inactive state',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(builder: mockHelper.builder),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        expect(
          CupertinoSliverRefreshControl.state(
            tester.element(find.byType(LayoutBuilder, skipOffstage: false)),
          ),
          RefreshIndicatorMode.inactive,
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'goes to drag and returns to inactive in a small drag',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(builder: mockHelper.builder),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        await tester.drag(find.text('0'), const Offset(0.0, 20.0));
        await tester.pump();

        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.drag,
        );

        await tester.pump(const Duration(seconds: 2));

        expect(
          CupertinoSliverRefreshControl.state(
            tester.element(find.byType(LayoutBuilder, skipOffstage: false)),
          ),
          RefreshIndicatorMode.inactive,
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'goes to armed the frame it passes the threshold',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  refreshTriggerPullDistance: 80.0,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        final TestGesture gesture = await tester.startGesture(Offset.zero);
        await gesture.moveBy(const Offset(0.0, 79.0));
        await tester.pump();
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.drag,
        );
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          await gesture.moveBy(
            const Offset(0.0, 20.0),
          ); // Overscrolling, need to move more than 1px.
        } else {
          await gesture.moveBy(
            const Offset(0.0, 3.0),
          ); // Overscrolling, need to move more than 1px.
        }
        await tester.pump();
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.armed,
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'goes to refresh the frame it crossed back the refresh threshold',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                  refreshTriggerPullDistance: 90.0,
                  refreshIndicatorExtent: 50.0,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        final TestGesture gesture = await tester.startGesture(Offset.zero);
        await gesture.moveBy(const Offset(0.0, 90.0)); // Arm it.
        await tester.pump();
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.armed,
        );

        await gesture.moveBy(
          const Offset(0.0, -80.0),
        ); // Overscrolling, need to move more than -40.
        await tester.pump();
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          expect(
            tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy,
            moreOrLessEquals(10.0), // Below 50 now.
          );
        } else {
          expect(
            tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy,
            moreOrLessEquals(49.775111111111116), // Below 50 now.
          );
        }
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.refresh,
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'goes to done internally as soon as the task finishes',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        await tester.drag(find.text('0'), const Offset(0.0, 100.0), touchSlopY: 0.0);
        await tester.pump();
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.armed,
        );
        // The sliver scroll offset correction is applied on the next frame.
        await tester.pump();

        await tester.pump(const Duration(seconds: 2));
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.refresh,
        );
        expect(
          tester.getRect(find.widgetWithText(SizedBox, '0')),
          const Rect.fromLTRB(0.0, 60.0, 800.0, 260.0),
        );

        mockHelper.refreshCompleter.complete(null);
        // The task completed between frames. The internal state goes to done
        // right away even though the sliver gets a new offset correction the
        // next frame.
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.done,
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'goes back to inactive when retracting back past 10% of arming distance',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        final TestGesture gesture = await tester.startGesture(Offset.zero);
        await gesture.moveBy(const Offset(0.0, 150.0));
        await tester.pump();
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.armed,
        );

        mockHelper.refreshCompleter.complete(null);
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.done,
        );
        await tester.pump();

        // Now back in overscroll mode.
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          await gesture.moveBy(const Offset(0.0, -125.0));
        } else {
          await gesture.moveBy(const Offset(0.0, -200.0));
        }
        await tester.pump();
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          expect(tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy, moreOrLessEquals(25.0));
        } else {
          expect(
            tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy,
            moreOrLessEquals(27.944444444444457),
          );
        }
        // Need to bring it to 100 * 0.1 to reset to inactive.
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.done,
        );

        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          await gesture.moveBy(const Offset(0.0, -16.0));
        } else {
          await gesture.moveBy(const Offset(0.0, -35.0));
        }
        await tester.pump();
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          expect(tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy, moreOrLessEquals(9.0));
        } else {
          expect(
            tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy,
            moreOrLessEquals(9.313890708161875),
          );
        }
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.inactive,
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      'goes back to inactive if already scrolled away when task completes',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: mockHelper.builder,
                  onRefresh: mockHelper.refreshTask,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        final TestGesture gesture = await tester.startGesture(Offset.zero);
        await gesture.moveBy(const Offset(0.0, 150.0));
        await tester.pump();
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.armed,
        );
        await tester.pump(); // Sliver scroll offset correction is applied one frame later.

        await gesture.moveBy(const Offset(0.0, -300.0));
        double indicatorDestinationPosition = -145.0332383665717;
        if (debugDefaultTargetPlatformOverride == TargetPlatform.macOS) {
          indicatorDestinationPosition = -150.0;
        }
        await tester.pump();
        // The refresh indicator is offscreen now.
        expect(
          tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy,
          moreOrLessEquals(indicatorDestinationPosition),
        );
        expect(
          CupertinoSliverRefreshControl.state(
            tester.element(find.byType(LayoutBuilder, skipOffstage: false)),
          ),
          RefreshIndicatorMode.refresh,
        );

        mockHelper.refreshCompleter.complete(null);
        // The sliver layout extent is removed on next frame.
        await tester.pump();
        expect(
          CupertinoSliverRefreshControl.state(
            tester.element(find.byType(LayoutBuilder, skipOffstage: false)),
          ),
          RefreshIndicatorMode.inactive,
        );
        // Nothing moved.
        expect(
          tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy,
          moreOrLessEquals(indicatorDestinationPosition),
        );
        await tester.pump(const Duration(seconds: 2));
        // Everything stayed as is.
        expect(
          tester.getTopLeft(find.widgetWithText(SizedBox, '0')).dy,
          moreOrLessEquals(indicatorDestinationPosition),
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets(
      "don't have to build any indicators or occupy space during refresh",
      (WidgetTester tester) async {
        mockHelper.refreshIndicator = const Center(child: Text('-1'));

        await tester.pumpWidget(
          CupertinoApp(
            home: CustomScrollView(
              slivers: <Widget>[
                CupertinoSliverRefreshControl(
                  builder: null,
                  onRefresh: mockHelper.refreshTask,
                  refreshIndicatorExtent: 0.0,
                ),
                buildAListOfStuff(),
              ],
            ),
          ),
        );

        await tester.drag(find.text('0'), const Offset(0.0, 150.0));
        await tester.pump();
        expect(
          CupertinoSliverRefreshControl.state(tester.element(find.byType(LayoutBuilder))),
          RefreshIndicatorMode.armed,
        );

        await tester.pump();
        await tester.pump(const Duration(seconds: 5));
        // In refresh mode but has no UI.
        expect(
          CupertinoSliverRefreshControl.state(
            tester.element(find.byType(LayoutBuilder, skipOffstage: false)),
          ),
          RefreshIndicatorMode.refresh,
        );
        expect(
          tester.getRect(find.widgetWithText(Center, '0')),
          const Rect.fromLTRB(0.0, 0.0, 800.0, 200.0),
        );

        mockHelper.refreshCompleter.complete(null);
        await tester.pump();
        // Goes to inactive right away since the sliver is already collapsed.
        expect(
          CupertinoSliverRefreshControl.state(
            tester.element(find.byType(LayoutBuilder, skipOffstage: false)),
          ),
          RefreshIndicatorMode.inactive,
        );
      },
      variant: const TargetPlatformVariant(<TargetPlatform>{
        TargetPlatform.iOS,
        TargetPlatform.macOS,
      }),
    );

    testWidgets('buildRefreshIndicator progress', (WidgetTester tester) async {
      await tester.pumpWidget(
        CupertinoApp(
          home: Builder(
            builder: (BuildContext context) {
              return CupertinoSliverRefreshControl.buildRefreshIndicator(
                context,
                RefreshIndicatorMode.drag,
                10,
                100,
                10,
              );
            },
          ),
        ),
      );
      expect(
        tester.widget<CupertinoActivityIndicator>(find.byType(CupertinoActivityIndicator)).progress,
        10.0 / 100.0,
      );

      await tester.pumpWidget(
        CupertinoApp(
          home: Builder(
            builder: (BuildContext context) {
              return CupertinoSliverRefreshControl.buildRefreshIndicator(
                context,
                RefreshIndicatorMode.drag,
                26,
                100,
                10,
              );
            },
          ),
        ),
      );
      expect(
        tester.widget<CupertinoActivityIndicator>(find.byType(CupertinoActivityIndicator)).progress,
        26.0 / 100.0,
      );

      await tester.pumpWidget(
        CupertinoApp(
          home: Builder(
            builder: (BuildContext context) {
              return CupertinoSliverRefreshControl.buildRefreshIndicator(
                context,
                RefreshIndicatorMode.drag,
                100,
                100,
                10,
              );
            },
          ),
        ),
      );
      expect(
        tester.widget<CupertinoActivityIndicator>(find.byType(CupertinoActivityIndicator)).progress,
        100.0 / 100.0,
      );
    });

    testWidgets('indicator should not become larger when overscrolled', (
      WidgetTester tester,
    ) async {
      // test for https://github.com/flutter/flutter/issues/79841
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: Builder(
            builder: (BuildContext context) {
              return CupertinoSliverRefreshControl.buildRefreshIndicator(
                context,
                RefreshIndicatorMode.done,
                120,
                100,
                10,
              );
            },
          ),
        ),
      );

      expect(
        tester.widget<CupertinoActivityIndicator>(find.byType(CupertinoActivityIndicator)).radius,
        14.0,
      );
    });
  }

  group('UI tests long list', uiTestGroup);

  // Test the internal state machine directly to make sure the UI aren't just
  // correct by coincidence.
  group('state machine test long list', stateMachineTestGroup);

  // Retest everything and make sure that it still works when the whole list
  // is smaller than the viewport size.
  testListLength = 2;
  group('UI tests short list', uiTestGroup);

  // Test the internal state machine directly to make sure the UI aren't just
  // correct by coincidence.
  group('state machine test short list', stateMachineTestGroup);

  testWidgets('Does not crash when paintExtent > remainingPaintExtent', (
    WidgetTester tester,
  ) async {
    // Regression test for https://github.com/flutter/flutter/issues/46871.
    await tester.pumpWidget(
      CupertinoApp(
        home: CustomScrollView(
          physics: const BouncingScrollPhysics(),
          slivers: <Widget>[
            const CupertinoSliverRefreshControl(),
            SliverList(
              delegate: SliverChildBuilderDelegate(
                (BuildContext context, int index) => const SizedBox(height: 100),
                childCount: 20,
              ),
            ),
          ],
        ),
      ),
    );

    // Drag the content down far enough so that
    // geometry.paintExtent > constraints.maxPaintExtent
    await tester.dragFrom(const Offset(10, 10), const Offset(0, 500));
    await tester.pump();

    expect(tester.takeException(), isNull);
  });
}

class FakeBuilder {
  Completer<void> refreshCompleter = Completer<void>.sync();
  final List<MockHelperInvocation> invocations = <MockHelperInvocation>[];

  Widget refreshIndicator = Container();

  Widget builder(
    BuildContext context,
    RefreshIndicatorMode refreshState,
    double pulledExtent,
    double refreshTriggerPullDistance,
    double refreshIndicatorExtent,
  ) {
    if (pulledExtent < 0.0) {
      throw TestFailure('The pulledExtent should never be less than 0.0');
    }
    if (refreshTriggerPullDistance < 0.0) {
      throw TestFailure('The refreshTriggerPullDistance should never be less than 0.0');
    }
    if (refreshIndicatorExtent < 0.0) {
      throw TestFailure('The refreshIndicatorExtent should never be less than 0.0');
    }
    invocations.add(
      BuilderInvocation(
        refreshState: refreshState,
        pulledExtent: pulledExtent,
        refreshTriggerPullDistance: refreshTriggerPullDistance,
        refreshIndicatorExtent: refreshIndicatorExtent,
      ),
    );
    return refreshIndicator;
  }

  Future<void> refreshTask() {
    invocations.add(const RefreshTaskInvocation());
    return refreshCompleter.future;
  }
}

abstract class MockHelperInvocation {
  const MockHelperInvocation();
}

@immutable
class RefreshTaskInvocation extends MockHelperInvocation {
  const RefreshTaskInvocation();
}

@immutable
class BuilderInvocation extends MockHelperInvocation {
  const BuilderInvocation({
    required this.refreshState,
    required this.pulledExtent,
    required this.refreshIndicatorExtent,
    required this.refreshTriggerPullDistance,
  });

  final RefreshIndicatorMode refreshState;
  final double pulledExtent;
  final double refreshTriggerPullDistance;
  final double refreshIndicatorExtent;

  @override
  String toString() =>
      '{refreshState: $refreshState, pulledExtent: $pulledExtent, refreshTriggerPullDistance: $refreshTriggerPullDistance, refreshIndicatorExtent: $refreshIndicatorExtent}';
}

Matcher matchesBuilder({
  required RefreshIndicatorMode refreshState,
  required dynamic pulledExtent,
  required dynamic refreshTriggerPullDistance,
  required dynamic refreshIndicatorExtent,
}) {
  return isA<BuilderInvocation>()
      .having(
        (BuilderInvocation invocation) => invocation.refreshState,
        'refreshState',
        refreshState,
      )
      .having(
        (BuilderInvocation invocation) => invocation.pulledExtent,
        'pulledExtent',
        pulledExtent,
      )
      .having(
        (BuilderInvocation invocation) => invocation.refreshTriggerPullDistance,
        'refreshTriggerPullDistance',
        refreshTriggerPullDistance,
      )
      .having(
        (BuilderInvocation invocation) => invocation.refreshIndicatorExtent,
        'refreshIndicatorExtent',
        refreshIndicatorExtent,
      );
}
