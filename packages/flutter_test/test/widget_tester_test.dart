// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:matcher/expect.dart' as matcher;
import 'package:matcher/src/expect/async_matcher.dart'; // ignore: implementation_imports

import 'multi_view_testing.dart';

void main() {
  group('expectLater', () {
    testWidgets('completes when matcher completes', (WidgetTester tester) async {
      final Completer<void> completer = Completer<void>();
      final Future<void> future = expectLater(null, FakeMatcher(completer));
      String? result;
      future.then<void>((void value) {
        result = '123';
      });
      matcher.expect(result, isNull);
      completer.complete();
      matcher.expect(result, isNull);
      await future;
      await tester.pump();
      matcher.expect(result, '123');
    });

    testWidgets('respects the skip flag', (WidgetTester tester) async {
      final Completer<void> completer = Completer<void>();
      final Future<void> future = expectLater(
        null,
        FakeMatcher(completer),
        skip: 'testing skip',
      ); // [intended] API testing
      bool completed = false;
      future.then<void>((_) {
        completed = true;
      });
      matcher.expect(completed, isFalse);
      await future;
      matcher.expect(completed, isTrue);
    });
  });

  group('group retry flag allows test to run multiple times', () {
    bool retried = false;
    group('the group with retry flag', () {
      testWidgets('the test inside it', (WidgetTester tester) async {
        addTearDown(() => retried = true);
        if (!retried) {
          debugPrint('DISREGARD NEXT FAILURE, IT IS EXPECTED');
        }
        expect(retried, isTrue);
      });
    }, retry: 1);
  });

  group('testWidget retry flag allows test to run multiple times', () {
    bool retried = false;
    testWidgets('the test with retry flag', (WidgetTester tester) async {
      addTearDown(() => retried = true);
      if (!retried) {
        debugPrint('DISREGARD NEXT FAILURE, IT IS EXPECTED');
      }
      expect(retried, isTrue);
    }, retry: 1);
  });

  group('respects the group skip flag', () {
    testWidgets('should be skipped', (WidgetTester tester) async {
      expect(false, true);
    });
  }, skip: true); // [intended] API testing

  group('pumping', () {
    testWidgets('pumping', (WidgetTester tester) async {
      await tester.pumpWidget(const Text('foo', textDirection: TextDirection.ltr));
      int count;

      final AnimationController test = AnimationController(
        duration: const Duration(milliseconds: 5100),
        vsync: tester,
      );
      count = await tester.pumpAndSettle(const Duration(seconds: 1));
      expect(count, 1); // it always pumps at least one frame

      test.forward(from: 0.0);
      count = await tester.pumpAndSettle(const Duration(seconds: 1));
      // 1 frame at t=0, starting the animation
      // 1 frame at t=1
      // 1 frame at t=2
      // 1 frame at t=3
      // 1 frame at t=4
      // 1 frame at t=5
      // 1 frame at t=6, ending the animation
      expect(count, 7);

      test.forward(from: 0.0);
      await tester.pump(); // starts the animation
      count = await tester.pumpAndSettle(const Duration(seconds: 1));
      expect(count, 6);

      test.forward(from: 0.0);
      await tester.pump(); // starts the animation
      await tester.pump(); // has no effect
      count = await tester.pumpAndSettle(const Duration(seconds: 1));
      expect(count, 6);
    });

    testWidgets('pumpFrames', (WidgetTester tester) async {
      final List<int> logPaints = <int>[];
      int? initial;

      final Widget target = _AlwaysAnimating(
        onPaint: () {
          final int current = SchedulerBinding.instance.currentFrameTimeStamp.inMicroseconds;
          initial ??= current;
          logPaints.add(current - initial!);
        },
      );

      await tester.pumpFrames(target, const Duration(milliseconds: 55));

      // `pumpframes` defaults to 16 milliseconds and 683 microseconds per pump,
      // so we expect 4 pumps of 16683 microseconds each in the 55ms duration.
      expect(logPaints, <int>[0, 16683, 33366, 50049]);
      logPaints.clear();

      await tester.pumpFrames(
        target,
        const Duration(milliseconds: 30),
        const Duration(milliseconds: 10),
      );

      // Since `pumpFrames` was given a 10ms interval per pump, we expect the
      // results to continue from 50049 with 10000 microseconds per pump over
      // the 30ms duration.
      expect(logPaints, <int>[60049, 70049, 80049]);
    });
  });
  group('pageBack', () {
    testWidgets('fails when there are no back buttons', (WidgetTester tester) async {
      await tester.pumpWidget(Container());

      expect(expectAsync0(tester.pageBack), throwsA(isA<TestFailure>()));
    });

    testWidgets('successfully taps material back buttons', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: Builder(
              builder: (BuildContext context) {
                return ElevatedButton(
                  child: const Text('Next'),
                  onPressed: () {
                    Navigator.push<void>(
                      context,
                      MaterialPageRoute<void>(
                        builder: (BuildContext context) {
                          return Scaffold(appBar: AppBar(title: const Text('Page 2')));
                        },
                      ),
                    );
                  },
                );
              },
            ),
          ),
        ),
      );

      await tester.tap(find.text('Next'));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 400));

      await tester.pageBack();
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 400));

      expect(find.text('Next'), findsOneWidget);
      expect(find.text('Page 2'), findsNothing);
    });

    testWidgets('successfully taps cupertino back buttons', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: Builder(
              builder: (BuildContext context) {
                return CupertinoButton(
                  child: const Text('Next'),
                  onPressed: () {
                    Navigator.push<void>(
                      context,
                      CupertinoPageRoute<void>(
                        builder: (BuildContext context) {
                          return CupertinoPageScaffold(
                            navigationBar: const CupertinoNavigationBar(middle: Text('Page 2')),
                            child: Container(),
                          );
                        },
                      ),
                    );
                  },
                );
              },
            ),
          ),
        ),
      );

      await tester.tap(find.text('Next'));
      await tester.pump();
      await tester.pump(const Duration(milliseconds: 400));

      await tester.pageBack();
      await tester.pump();
      await tester.pumpAndSettle();

      expect(find.text('Next'), findsOneWidget);
      expect(find.text('Page 2'), findsNothing);
    });
  });

  testWidgets('hasRunningAnimations control test', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(
      duration: const Duration(seconds: 1),
      vsync: const TestVSync(),
    );
    expect(tester.hasRunningAnimations, isFalse);
    controller.forward();
    expect(tester.hasRunningAnimations, isTrue);
    controller.stop();
    expect(tester.hasRunningAnimations, isFalse);
    controller.forward();
    expect(tester.hasRunningAnimations, isTrue);
    await tester.pumpAndSettle();
    expect(tester.hasRunningAnimations, isFalse);
  });

  testWidgets('pumpAndSettle control test', (WidgetTester tester) async {
    final AnimationController controller = AnimationController(
      duration: const Duration(minutes: 525600),
      vsync: const TestVSync(),
    );
    expect(await tester.pumpAndSettle(), 1);
    controller.forward();
    try {
      await tester.pumpAndSettle();
      expect(true, isFalse);
    } catch (e) {
      expect(e, isFlutterError);
    }
    controller.stop();
    expect(await tester.pumpAndSettle(), 1);
    controller.duration = const Duration(seconds: 1);
    controller.forward();
    expect(
      await tester.pumpAndSettle(const Duration(milliseconds: 300)),
      5,
    ); // 0, 300, 600, 900, 1200ms
  });

  testWidgets('Input event array', (WidgetTester tester) async {
    final List<String> logs = <String>[];

    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: Listener(
          onPointerDown: (PointerDownEvent event) => logs.add('down ${event.buttons}'),
          onPointerMove: (PointerMoveEvent event) => logs.add('move ${event.buttons}'),
          onPointerUp: (PointerUpEvent event) => logs.add('up ${event.buttons}'),
          child: const Text('test'),
        ),
      ),
    );

    final Offset location = tester.getCenter(find.text('test'));
    final List<PointerEventRecord> records = <PointerEventRecord>[
      PointerEventRecord(Duration.zero, <PointerEvent>[
        // Typically PointerAddedEvent is not used in testers, but for records
        // captured on a device it is usually what start a gesture.
        PointerAddedEvent(position: location),
        PointerDownEvent(position: location, buttons: kSecondaryMouseButton, pointer: 1),
      ]),
      ...<PointerEventRecord>[
        for (
          Duration t = const Duration(milliseconds: 5);
          t < const Duration(milliseconds: 80);
          t += const Duration(milliseconds: 16)
        )
          PointerEventRecord(t, <PointerEvent>[
            PointerMoveEvent(
              timeStamp: t - const Duration(milliseconds: 1),
              position: location,
              buttons: kSecondaryMouseButton,
              pointer: 1,
            ),
          ]),
      ],
      PointerEventRecord(const Duration(milliseconds: 80), <PointerEvent>[
        PointerUpEvent(
          timeStamp: const Duration(milliseconds: 79),
          position: location,
          buttons: kSecondaryMouseButton,
          pointer: 1,
        ),
      ]),
    ];
    final List<Duration> timeDiffs = await tester.handlePointerEventRecord(records);
    expect(timeDiffs.length, records.length);
    for (final Duration diff in timeDiffs) {
      expect(diff, Duration.zero);
    }

    const String b = '$kSecondaryMouseButton';
    expect(logs.first, 'down $b');
    for (int i = 1; i < logs.length - 1; i++) {
      expect(logs[i], 'move $b');
    }
    expect(logs.last, 'up $b');
  });

  group('runAsync', () {
    testWidgets('works with no async calls', (WidgetTester tester) async {
      String? value;
      await tester.runAsync(() async {
        value = '123';
      });
      expect(value, '123');
    });

    testWidgets('works with real async calls', (WidgetTester tester) async {
      final StringBuffer buf = StringBuffer('1');
      await tester.runAsync(() async {
        buf.write('2');
        //ignore: avoid_slow_async_io
        await Directory.current.stat();
        buf.write('3');
      });
      buf.write('4');
      expect(buf.toString(), '1234');
    });

    testWidgets('propagates return values', (WidgetTester tester) async {
      final String? value = await tester.runAsync<String>(() async {
        return '123';
      });
      expect(value, '123');
    });

    testWidgets('reports errors via framework', (WidgetTester tester) async {
      final String? value = await tester.runAsync<String>(() async {
        throw ArgumentError();
      });
      expect(value, isNull);
      expect(tester.takeException(), isArgumentError);
    });

    testWidgets('disallows re-entry', (WidgetTester tester) async {
      final Completer<void> completer = Completer<void>();
      tester.runAsync<void>(() => completer.future);
      expect(() => tester.runAsync(() async {}), throwsA(isA<TestFailure>()));
      completer.complete();
    });

    testWidgets('maintains existing zone values', (WidgetTester tester) async {
      final Object key = Object();
      await runZoned<Future<void>>(() {
        expect(Zone.current[key], 'abczed');
        return tester.runAsync<void>(() async {
          expect(Zone.current[key], 'abczed');
        });
      }, zoneValues: <dynamic, dynamic>{key: 'abczed'});
    });

    testWidgets('control test (return value)', (WidgetTester tester) async {
      final String? result = await tester.binding.runAsync<String>(() async => 'Judy Turner');
      expect(result, 'Judy Turner');
    });

    testWidgets('async throw', (WidgetTester tester) async {
      final String? result = await tester.binding.runAsync<Never>(
        () async => throw Exception('Lois Dilettente'),
      );
      expect(result, isNull);
      expect(tester.takeException(), isNotNull);
    });

    testWidgets('sync throw', (WidgetTester tester) async {
      final String? result = await tester.binding.runAsync<Never>(
        () => throw Exception('Butch Barton'),
      );
      expect(result, isNull);
      expect(tester.takeException(), isNotNull);
    });
  });

  group('showKeyboard', () {
    testWidgets('can be called twice', (WidgetTester tester) async {
      await tester.pumpWidget(MaterialApp(home: Material(child: Center(child: TextFormField()))));
      await tester.showKeyboard(find.byType(TextField));
      await tester.testTextInput.receiveAction(TextInputAction.done);
      await tester.pump();
      await tester.showKeyboard(find.byType(TextField));
      await tester.testTextInput.receiveAction(TextInputAction.done);
      await tester.pump();
      await tester.showKeyboard(find.byType(TextField));
      await tester.showKeyboard(find.byType(TextField));
      await tester.pump();
    });

    testWidgets(
      'can focus on offstage text input field if finder says not to skip offstage nodes',
      (WidgetTester tester) async {
        await tester.pumpWidget(
          MaterialApp(home: Material(child: Offstage(child: TextFormField()))),
        );
        await tester.showKeyboard(find.byType(TextField, skipOffstage: false));
      },
    );
  });

  testWidgets('verifyTickersWereDisposed control test', (WidgetTester tester) async {
    late FlutterError error;
    final Ticker ticker = tester.createTicker((Duration duration) {});
    ticker.start();
    try {
      tester.verifyTickersWereDisposed('');
    } on FlutterError catch (e) {
      error = e;
    } finally {
      expect(error, isNotNull);
      expect(error.diagnostics.length, 4);
      expect(error.diagnostics[2].level, DiagnosticLevel.hint);
      expect(
        error.diagnostics[2].toStringDeep(),
        'Tickers used by AnimationControllers should be disposed by\n'
        'calling dispose() on the AnimationController itself. Otherwise,\n'
        'the ticker will leak.\n',
      );
      expect(error.diagnostics.last, isA<DiagnosticsProperty<Ticker>>());
      expect(error.diagnostics.last.value, ticker);
      expect(
        error.toStringDeep(),
        startsWith(
          'FlutterError\n'
          '   A Ticker was active .\n'
          '   All Tickers must be disposed.\n'
          '   Tickers used by AnimationControllers should be disposed by\n'
          '   calling dispose() on the AnimationController itself. Otherwise,\n'
          '   the ticker will leak.\n'
          '   The offending ticker was:\n'
          '     _TestTicker()\n',
        ),
      );
    }
    ticker.stop();
  });

  group('testWidgets variants work', () {
    int numberOfVariationsRun = 0;

    testWidgets(
      'variant tests run all values provided',
      (WidgetTester tester) async {
        if (debugDefaultTargetPlatformOverride == null) {
          expect(numberOfVariationsRun, equals(TargetPlatform.values.length));
        } else {
          numberOfVariationsRun += 1;
        }
      },
      variant: TargetPlatformVariant(TargetPlatform.values.toSet()),
    );

    testWidgets(
      'variant tests have descriptions with details',
      (WidgetTester tester) async {
        if (debugDefaultTargetPlatformOverride == null) {
          expect(tester.testDescription, equals('variant tests have descriptions with details'));
        } else {
          expect(
            tester.testDescription,
            equals(
              'variant tests have descriptions with details (variant: $debugDefaultTargetPlatformOverride)',
            ),
          );
        }
      },
      variant: TargetPlatformVariant(TargetPlatform.values.toSet()),
    );
  });

  group('TargetPlatformVariant', () {
    int numberOfVariationsRun = 0;
    TargetPlatform? origTargetPlatform;

    setUpAll(() {
      origTargetPlatform = debugDefaultTargetPlatformOverride;
    });

    tearDownAll(() {
      expect(debugDefaultTargetPlatformOverride, equals(origTargetPlatform));
    });

    testWidgets(
      'TargetPlatformVariant.only tests given value',
      (WidgetTester tester) async {
        expect(debugDefaultTargetPlatformOverride, equals(TargetPlatform.iOS));
        expect(defaultTargetPlatform, equals(TargetPlatform.iOS));
      },
      variant: TargetPlatformVariant.only(TargetPlatform.iOS),
    );

    group('all', () {
      testWidgets('TargetPlatformVariant.all tests run all variants', (WidgetTester tester) async {
        if (debugDefaultTargetPlatformOverride == null) {
          expect(numberOfVariationsRun, equals(TargetPlatform.values.length));
        } else {
          numberOfVariationsRun += 1;
        }
      }, variant: TargetPlatformVariant.all());

      const Set<TargetPlatform> excludePlatforms = <TargetPlatform>{
        TargetPlatform.android,
        TargetPlatform.linux,
      };
      testWidgets(
        'TargetPlatformVariant.all, excluding runs an all variants except those provided in excluding',
        (WidgetTester tester) async {
          if (debugDefaultTargetPlatformOverride == null) {
            expect(
              numberOfVariationsRun,
              equals(TargetPlatform.values.length - excludePlatforms.length),
            );
            expect(
              excludePlatforms,
              isNot(contains(debugDefaultTargetPlatformOverride)),
              reason: 'this test should not run on any platform in excludePlatforms',
            );
          } else {
            numberOfVariationsRun += 1;
          }
        },
        variant: TargetPlatformVariant.all(excluding: excludePlatforms),
      );
    });

    testWidgets('TargetPlatformVariant.desktop + mobile contains all TargetPlatform values', (
      WidgetTester tester,
    ) async {
      final TargetPlatformVariant all = TargetPlatformVariant.all();
      final TargetPlatformVariant desktop = TargetPlatformVariant.all();
      final TargetPlatformVariant mobile = TargetPlatformVariant.all();
      expect(desktop.values.union(mobile.values), equals(all.values));
    });
  });

  group('Pending timer', () {
    late TestExceptionReporter currentExceptionReporter;
    setUp(() {
      currentExceptionReporter = reportTestException;
    });

    tearDown(() {
      reportTestException = currentExceptionReporter;
    });

    test('Throws assertion message without code', () async {
      late FlutterErrorDetails flutterErrorDetails;
      reportTestException = (FlutterErrorDetails details, String testDescription) {
        flutterErrorDetails = details;
      };

      final TestWidgetsFlutterBinding binding = TestWidgetsFlutterBinding.ensureInitialized();
      debugPrint('DISREGARD NEXT PENDING TIMER LIST, IT IS EXPECTED');
      await binding.runTest(() async {
        final Timer timer = Timer(const Duration(seconds: 1), () {});
        expect(timer.isActive, true);
      }, () {});

      expect(flutterErrorDetails.exception, isA<AssertionError>());
      expect(
        (flutterErrorDetails.exception as AssertionError).message,
        'A Timer is still pending even after the widget tree was disposed.',
      );
      expect(binding.inTest, true);
      binding.postTest();
    });
  });

  group('Accessibility announcements testing API', () {
    testWidgets('Returns the list of announcements', (WidgetTester tester) async {
      // Make sure the handler is properly set
      expect(
        TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler(
          SystemChannels.accessibility.name,
          null,
        ),
        isFalse,
      );

      await SemanticsService.announce('announcement 1', TextDirection.ltr);
      await SemanticsService.announce(
        'announcement 2',
        TextDirection.rtl,
        assertiveness: Assertiveness.assertive,
      );
      await SemanticsService.announce('announcement 3', TextDirection.rtl);

      final List<CapturedAccessibilityAnnouncement> list = tester.takeAnnouncements();
      expect(list, hasLength(3));
      final CapturedAccessibilityAnnouncement first = list[0];
      expect(first.message, 'announcement 1');
      expect(first.textDirection, TextDirection.ltr);

      final CapturedAccessibilityAnnouncement second = list[1];
      expect(second.message, 'announcement 2');
      expect(second.textDirection, TextDirection.rtl);
      expect(second.assertiveness, Assertiveness.assertive);

      final CapturedAccessibilityAnnouncement third = list[2];
      expect(third.message, 'announcement 3');
      expect(third.textDirection, TextDirection.rtl);
      expect(third.assertiveness, Assertiveness.polite);

      final List<CapturedAccessibilityAnnouncement> emptyList = tester.takeAnnouncements();
      expect(emptyList, <CapturedAccessibilityAnnouncement>[]);
    });

    test('New test API is not breaking existing tests', () async {
      final List<Map<dynamic, dynamic>> log = <Map<dynamic, dynamic>>[];

      Future<dynamic> handleMessage(dynamic mockMessage) async {
        final Map<dynamic, dynamic> message = mockMessage as Map<dynamic, dynamic>;
        log.add(message);
      }

      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
          .setMockDecodedMessageHandler<dynamic>(SystemChannels.accessibility, handleMessage);

      await SemanticsService.announce(
        'announcement 1',
        TextDirection.rtl,
        assertiveness: Assertiveness.assertive,
      );
      expect(
        log,
        equals(<Map<String, dynamic>>[
          <String, dynamic>{
            'type': 'announce',
            'data': <String, dynamic>{
              'message': 'announcement 1',
              'textDirection': 0,
              'assertiveness': 1,
            },
          },
        ]),
      );

      // Remove the handler
      TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger
          .setMockDecodedMessageHandler<dynamic>(SystemChannels.accessibility, null);
    });

    tearDown(() {
      // Make sure that the handler is removed in [TestWidgetsFlutterBinding.postTest]
      expect(
        TestDefaultBinaryMessengerBinding.instance.defaultBinaryMessenger.checkMockMessageHandler(
          SystemChannels.accessibility.name,
          null,
        ),
        isTrue,
      );
    });
  });

  testWidgets('wrapWithView: false does not include View', (WidgetTester tester) async {
    FlutterView? flutterView;
    View? view;
    int builderCount = 0;
    await tester.pumpWidget(
      wrapWithView: false,
      Builder(
        builder: (BuildContext context) {
          builderCount++;
          flutterView = View.maybeOf(context);
          view = context.findAncestorWidgetOfExactType<View>();
          return const ViewCollection(views: <Widget>[]);
        },
      ),
    );

    expect(builderCount, 1);
    expect(view, isNull);
    expect(flutterView, isNull);
    expect(find.byType(View), findsNothing);
  });

  testWidgets('passing a view to pumpWidget with wrapWithView: true throws', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(View(view: FakeView(tester.view), child: const SizedBox.shrink()));
    expect(
      tester.takeException(),
      isFlutterError.having(
        (FlutterError e) => e.message,
        'message',
        contains('consider setting the "wrapWithView" parameter of that method to false'),
      ),
    );
  });

  testWidgets('can pass a View to pumpWidget when wrapWithView: false', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(
      wrapWithView: false,
      View(view: tester.view, child: const SizedBox.shrink()),
    );
    expect(find.byType(View), findsOne);
  });
}

class FakeMatcher extends AsyncMatcher {
  FakeMatcher(this.completer);

  final Completer<void> completer;

  @override
  Future<String?> matchAsync(dynamic object) {
    return completer.future.then<String?>((void value) {
      return object?.toString();
    });
  }

  @override
  Description describe(Description description) => description.add('--fake--');
}

class _AlwaysAnimating extends StatefulWidget {
  const _AlwaysAnimating({required this.onPaint});

  final VoidCallback onPaint;

  @override
  State<StatefulWidget> createState() => _AlwaysAnimatingState();
}

class _AlwaysAnimatingState extends State<_AlwaysAnimating> with SingleTickerProviderStateMixin {
  late AnimationController _controller;

  @override
  void initState() {
    super.initState();
    _controller = AnimationController(duration: const Duration(milliseconds: 100), vsync: this);
    _controller.repeat();
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AnimatedBuilder(
      animation: _controller.view,
      builder: (BuildContext context, Widget? child) {
        return CustomPaint(painter: _AlwaysRepaint(widget.onPaint));
      },
    );
  }
}

class _AlwaysRepaint extends CustomPainter {
  _AlwaysRepaint(this.onPaint);

  final VoidCallback onPaint;

  @override
  bool shouldRepaint(CustomPainter oldDelegate) => true;

  @override
  void paint(Canvas canvas, Size size) {
    onPaint();
  }
}
