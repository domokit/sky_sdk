// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'dart:async';

import 'package:file/file.dart';

import '../src/common.dart';
import 'test_data/project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

void main() {
  late Directory tempDir;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('break_on_framework_exceptions.');
  });

  tearDown(() async {
    tryToDelete(tempDir);
  });

  Future<void> expectException(TestProject project, String exceptionMessage) async {
    await _timeoutAfter(
      message: 'Timed out setting up project in $tempDir',
      work: () => project.setUpIn(tempDir),
    );

    final FlutterTestTestDriver flutter = FlutterTestTestDriver(tempDir);

    try {
      await _timeoutAfter(
        message: 'Timed out launching `flutter test`',
        work: () => flutter.test(withDebugger: true, pauseOnExceptions: true),
      );

      await _timeoutAfter(
        message: 'Timed out waiting for VM service pause debug event',
        work: flutter.waitForPause,
      );

      int? breakLine;
      await _timeoutAfter(
        message: 'Timed out getting source location of top stack frame',
        work: () async => breakLine = (await flutter.getSourceLocation())?.line,
      );

      expect(breakLine, project.lineContaining(project.test, exceptionMessage));
    } finally {
      // Some of the tests will quit naturally, and others won't.
      // By this point we don't need the tool anymore, so just force quit.
      await flutter.quit();
    }
  }

  testWithoutContext('breaks when AnimationController listener throws', () async {
    final TestProject project = TestProject(r'''
      AnimationController(vsync: TestVSync(), duration: Duration.zero)
        ..addListener(() {
          throw 'AnimationController listener';
        })
        ..forward();
      ''');

    await expectException(project, "throw 'AnimationController listener';");
  });

  testWithoutContext('breaks when AnimationController status listener throws', () async {
    final TestProject project = TestProject(r'''
      AnimationController(vsync: TestVSync(), duration: Duration.zero)
        ..addStatusListener((AnimationStatus _) {
          throw 'AnimationController status listener';
        })
        ..forward();
      ''');

    await expectException(project, "throw 'AnimationController status listener';");
  });

  testWithoutContext('breaks when ChangeNotifier listener throws', () async {
    final TestProject project = TestProject(r'''
       ValueNotifier<int>(0)
         ..addListener(() {
           throw 'ValueNotifier listener';
         })
         ..value = 1;
       ''');

    await expectException(project, "throw 'ValueNotifier listener';");
  });

  testWithoutContext('breaks when handling a gesture throws', () async {
    final TestProject project = TestProject(r'''
      await tester.pumpWidget(
        MaterialApp(
          home: Center(
            child: ElevatedButton(
              child: const Text('foo'),
              onPressed: () {
                throw 'while handling a gesture';
              },
            ),
          ),
        )
      );
      await tester.tap(find.byType(ElevatedButton));
      ''');

    await expectException(project, "throw 'while handling a gesture';");
  });

  testWithoutContext('breaks when platform message callback throws', () async {
    final TestProject project = TestProject(r'''
      BasicMessageChannel<String>('foo', const StringCodec()).setMessageHandler((_) {
        throw 'platform message callback';
      });
      tester.binding.defaultBinaryMessenger.handlePlatformMessage('foo', const StringCodec().encodeMessage('Hello'), (_) {});
      ''');

    await expectException(project, "throw 'platform message callback';");
  });

  testWithoutContext('breaks when SliverChildBuilderDelegate.builder throws', () async {
    final TestProject project = TestProject(r'''
      await tester.pumpWidget(MaterialApp(
        home: ListView.builder(
          itemBuilder: (BuildContext context, int index) {
            throw 'cannot build child';
          },
        ),
      ));
      ''');

    await expectException(project, "throw 'cannot build child';");
  });

  testWithoutContext('breaks when EditableText.onChanged throws', () async {
    final TestProject project = TestProject(r'''
      await tester.pumpWidget(MaterialApp(
        home: Material(
          child: TextField(
            onChanged: (String t) {
              throw 'onChanged';
            },
          ),
        ),
      ));
      await tester.enterText(find.byType(TextField), 'foo');
      ''');

    await expectException(project, "throw 'onChanged';");
  });

  testWithoutContext('breaks when EditableText.onEditingComplete throws', () async {
    final TestProject project = TestProject(r'''
      await tester.pumpWidget(MaterialApp(
        home: Material(
          child: TextField(
            onEditingComplete: () {
              throw 'onEditingComplete';
            },
          ),
        ),
      ));
      await tester.tap(find.byType(EditableText));
      await tester.pump();
      await tester.testTextInput.receiveAction(TextInputAction.done);
      ''');

    await expectException(project, "throw 'onEditingComplete';");
  });

  testWithoutContext('breaks when EditableText.onSelectionChanged throws', () async {
    final TestProject project = TestProject(r'''
      await tester.pumpWidget(MaterialApp(
        home: SelectableText('hello',
          onSelectionChanged: (TextSelection selection, SelectionChangedCause? cause) {
            throw 'onSelectionChanged';
          },
        ),
      ));
      await tester.tap(find.byType(SelectableText));
      ''');

    await expectException(project, "throw 'onSelectionChanged';");
  });

  testWithoutContext('breaks when Action listener throws', () async {
    final TestProject project = TestProject(r'''
      CallbackAction<Intent>(onInvoke: (Intent _) { })
        ..addActionListener((_) {
          throw 'action listener';
        })
        ..notifyActionListeners();
      ''');

    await expectException(project, "throw 'action listener';");
  });

  testWithoutContext('breaks when pointer route throws', () async {
    final TestProject project = TestProject(r'''
      PointerRouter()
        ..addRoute(2, (PointerEvent event) {
          throw 'pointer route';
        })
        ..route(TestPointer(2).down(Offset.zero));
      ''');

    await expectException(project, "throw 'pointer route';");
  });

  testWithoutContext('breaks when PointerSignalResolver callback throws', () async {
    final TestProject project = TestProject(r'''
      const PointerScrollEvent originalEvent = PointerScrollEvent();
      PointerSignalResolver()
        ..register(originalEvent, (PointerSignalEvent event) {
          throw 'PointerSignalResolver callback';
        })
        ..resolve(originalEvent);
      ''');

    await expectException(project, "throw 'PointerSignalResolver callback';");
  });

  testWithoutContext('breaks when PointerSignalResolver callback throws', () async {
    final TestProject project = TestProject(r'''
      FocusManager.instance
        ..addHighlightModeListener((_) {
          throw 'highlight mode listener';
        })
        ..highlightStrategy = FocusHighlightStrategy.alwaysTouch
        ..highlightStrategy = FocusHighlightStrategy.alwaysTraditional;
      ''');

    await expectException(project, "throw 'highlight mode listener';");
  });

  testWithoutContext('breaks when GestureBinding.dispatchEvent throws', () async {
    final TestProject project = TestProject(r'''
      await tester.pumpWidget(
        MouseRegion(
          onHover: (_) {
            throw 'onHover';
          },
        )
      );
      final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
      await gesture.addPointer(location: Offset.zero);
      await tester.pump();
      await gesture.moveTo(tester.getCenter(find.byType(MouseRegion)));
      await tester.pump();
      gesture.removePointer();
      ''');

    await expectException(project, "throw 'onHover';");
  });

  testWithoutContext('breaks when ImageStreamListener.onImage throws', () async {
    final TestProject project = TestProject(
      r'''
      final Completer<ImageInfo> completer = Completer<ImageInfo>();
      OneFrameImageStreamCompleter(completer.future)
        ..addListener(ImageStreamListener((ImageInfo _, bool __) {
          throw 'setImage';
        }));
      completer.complete(ImageInfo(image: image));
      ''',
      setup: r'''
        late ui.Image image;
        setUp(() async {
          image = await createTestImage();
        });
      ''',
    );

    await expectException(project, "throw 'setImage';");
  });

  testWithoutContext('breaks when ImageStreamListener.onError throws', () async {
    final TestProject project = TestProject(r'''
      final Completer<ImageInfo> completer = Completer<ImageInfo>();
      OneFrameImageStreamCompleter(completer.future)
        ..addListener(ImageStreamListener(
          (ImageInfo _, bool __) { },
          onError: (Object _, StackTrace? __) {
            throw 'onError';
          },
        ));
      completer.completeError('ERROR');
      ''');

    await expectException(project, "throw 'onError';");
  });

  testWithoutContext('breaks when LayoutBuilder.builder throws', () async {
    final TestProject project = TestProject(r'''
      await tester.pumpWidget(LayoutBuilder(
        builder: (_, __) {
          throw 'LayoutBuilder.builder';
        },
      ));
      ''');

    await expectException(project, "throw 'LayoutBuilder.builder';");
  });

  testWithoutContext('breaks when _CallbackHookProvider callback throws', () async {
    final TestProject project = TestProject(r'''
      RootBackButtonDispatcher()
        ..addCallback(() {
          throw '_CallbackHookProvider.callback';
        })
        ..invokeCallback(Future.value(false));
      ''');

    await expectException(project, "throw '_CallbackHookProvider.callback';");
  });

  testWithoutContext('breaks when TimingsCallback throws', () async {
    final TestProject project = TestProject(r'''
      SchedulerBinding.instance!.addTimingsCallback((List<FrameTiming> timings) {
        throw 'TimingsCallback';
      });
      ui.PlatformDispatcher.instance.onReportTimings!(<FrameTiming>[]);
      ''');

    await expectException(project, "throw 'TimingsCallback';");
  });

  testWithoutContext('breaks when TimingsCallback throws', () async {
    final TestProject project = TestProject(r'''
      SchedulerBinding.instance!.scheduleTask(
        () {
          throw 'scheduled task';
        },
        Priority.touch,
      );
      await tester.pumpAndSettle();
      ''');

    await expectException(project, "throw 'scheduled task';");
  });

  testWithoutContext('breaks when FrameCallback throws', () async {
    final TestProject project = TestProject(r'''
      SchedulerBinding.instance!.addPostFrameCallback((_) {
        throw 'FrameCallback';
      });
      await tester.pump();
      ''');

    await expectException(project, "throw 'FrameCallback';");
  });

  testWithoutContext('breaks when attaching to render tree throws', () async {
    final TestProject project = TestProject(
      r'''
      await tester.pumpWidget(TestWidget());
      ''',
      classes: r'''
        class TestWidget extends StatelessWidget {
          @override
          StatelessElement createElement() {
            throw 'create element';
          }

          @override
          Widget build(BuildContext context) => Container();
        }
      ''',
    );

    await expectException(project, "throw 'create element';");
  });

  testWithoutContext('breaks when RenderObject.performLayout throws', () async {
    final TestProject project = TestProject(
      r'''
      TestRender().layout(BoxConstraints());
      ''',
      classes: r'''
        class TestRender extends RenderBox {
          @override
          void performLayout() {
            throw 'performLayout';
          }
        }
      ''',
    );

    await expectException(project, "throw 'performLayout';");
  });

  testWithoutContext('breaks when RenderObject.performResize throws', () async {
    final TestProject project = TestProject(
      r'''
      TestRender().layout(BoxConstraints());
      ''',
      classes: r'''
        class TestRender extends RenderBox {
          @override
          bool get sizedByParent => true;

          @override
          void performResize() {
            throw 'performResize';
          }
        }
      ''',
    );

    await expectException(project, "throw 'performResize';");
  });

  testWithoutContext('breaks when RenderObject.performLayout (without resize) throws', () async {
    final TestProject project = TestProject(
      r'''
      await tester.pumpWidget(TestWidget());
      tester.renderObject<TestRender>(find.byType(TestWidget)).layoutThrows = true;
      await tester.pump();
      ''',
      classes: r'''
        class TestWidget extends LeafRenderObjectWidget {
          @override
          RenderObject createRenderObject(BuildContext context) => TestRender();
        }

        class TestRender extends RenderBox {
          bool get layoutThrows => _layoutThrows;
          bool _layoutThrows = false;
          set layoutThrows(bool value) {
            if (value == _layoutThrows) {
              return;
            }
            _layoutThrows = value;
            markNeedsLayout();
          }

          @override
          void performLayout() {
            if (layoutThrows) {
              throw 'performLayout without resize';
            }
            size = constraints.biggest;
          }
        }
      ''',
    );

    await expectException(project, "throw 'performLayout without resize';");
  });

  testWithoutContext('breaks when StatelessWidget.build throws', () async {
    final TestProject project = TestProject(
      r'''
      await tester.pumpWidget(TestWidget());
      ''',
      classes: r'''
        class TestWidget extends StatelessWidget {
          @override
          Widget build(BuildContext context) {
            throw 'StatelessWidget.build';
          }
        }
      ''',
    );

    await expectException(project, "throw 'StatelessWidget.build';");
  });

  testWithoutContext('breaks when StatefulWidget.build throws', () async {
    final TestProject project = TestProject(
      r'''
      await tester.pumpWidget(TestWidget());
      ''',
      classes: r'''
        class TestWidget extends StatefulWidget {
          @override
          _TestWidgetState createState() => _TestWidgetState();
        }

        class _TestWidgetState extends State<TestWidget> {
          @override
          Widget build(BuildContext context) {
            throw 'StatefulWidget.build';
          }
        }
      ''',
    );

    await expectException(project, "throw 'StatefulWidget.build';");
  });

  testWithoutContext('breaks when finalizing the tree throws', () async {
    final TestProject project = TestProject(
      r'''
      await tester.pumpWidget(TestWidget());
      ''',
      classes: r'''
        class TestWidget extends StatefulWidget {
          @override
          _TestWidgetState createState() => _TestWidgetState();
        }

        class _TestWidgetState extends State<TestWidget> {
          @override
          void dispose() {
            super.dispose();
            throw 'dispose';
          }

          @override
          Widget build(BuildContext context) => Container();
        }
      ''',
    );

    await expectException(project, "throw 'dispose';");
  });

  testWithoutContext('breaks when rebuilding dirty elements throws', () async {
    final TestProject project = TestProject(
      r'''
      await tester.pumpWidget(TestWidget());
      tester.element<TestElement>(find.byType(TestWidget)).throwOnRebuild = true;
      await tester.pump();
      ''',
      classes: r'''
        class TestWidget extends StatelessWidget {
          @override
          StatelessElement createElement() => TestElement(this);

          @override
          Widget build(BuildContext context) => Container();
        }

        class TestElement extends StatelessElement {
          TestElement(StatelessWidget widget) : super(widget);

          bool get throwOnRebuild => _throwOnRebuild;
          bool _throwOnRebuild = false;
          set throwOnRebuild(bool value) {
            if (value == _throwOnRebuild) {
              return;
            }
            _throwOnRebuild = value;
            markNeedsBuild();
          }

          @override
          void rebuild({bool force = false}) {
            if (_throwOnRebuild) {
              throw 'rebuild';
            }
            super.rebuild(force: force);
          }
        }
      ''',
    );

    await expectException(project, "throw 'rebuild';");
  });
}

/// A debugging wrapper to help diagnose tests that are already timing out.
///
/// When these tests are timed out by package:test (after 15 minutes), there
/// is no hint in logs as to where the test got stuck. By passing async calls
/// to this function with a [duration] less than that configured in
/// package:test can be set and a helpful message used to help debugging.
///
/// See https://github.com/flutter/flutter/issues/125241 for more context.
Future<void> _timeoutAfter({
  required String message,
  Duration duration = const Duration(minutes: 10),
  required Future<void> Function() work,
}) async {
  final Timer timer = Timer(duration, () => fail(message));
  await work();
  timer.cancel();
}

class TestProject extends Project {
  TestProject(this.testBody, {this.setup, this.classes});

  final String testBody;
  final String? setup;
  final String? classes;

  @override
  final String pubspec = '''
    name: test
    environment:
      sdk: ^3.7.0-0

    dependencies:
      flutter:
        sdk: flutter
    dev_dependencies:
      flutter_test:
        sdk: flutter
  ''';

  @override
  final String main = '';

  @override
  String get test => _test
      .replaceFirst('// SETUP', setup ?? '')
      .replaceFirst('// TEST_BODY', testBody)
      .replaceFirst('// CLASSES', classes ?? '');

  final String _test = r'''
    import 'dart:async';
    import 'dart:ui' as ui;

    import 'package:flutter/animation.dart';
    import 'package:flutter/foundation.dart';
    import 'package:flutter/gestures.dart';
    import 'package:flutter/material.dart';
    import 'package:flutter/scheduler.dart';
    import 'package:flutter/services.dart';
    import 'package:flutter_test/flutter_test.dart';

    void main() {
      // SETUP
      testWidgets('test', (WidgetTester tester) async {
        // TEST_BODY
      });
    }
    // CLASSES
  ''';
}
