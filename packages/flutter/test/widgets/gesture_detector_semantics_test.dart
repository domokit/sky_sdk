// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'semantics_tester.dart';

void main() {
  testWidgets('Vertical gesture detector has up/down actions', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    int callCount = 0;
    final GlobalKey detectorKey = GlobalKey();

    await tester.pumpWidget(
      Center(
        child: GestureDetector(
          key: detectorKey,
          onVerticalDragStart: (DragStartDetails _) {
            callCount += 1;
          },
          child: Container(),
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollDown],
      ),
    );

    final int detectorId = detectorKey.currentContext!.findRenderObject()!.debugSemantics!.id;
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollLeft,
    );
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollRight,
    );
    expect(callCount, 0);
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollUp,
    );
    expect(callCount, 1);
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollDown,
    );
    expect(callCount, 2);

    semantics.dispose();
  });

  testWidgets('Horizontal gesture detector has up/down actions', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    int callCount = 0;
    final GlobalKey detectorKey = GlobalKey();

    await tester.pumpWidget(
      Center(
        child: GestureDetector(
          key: detectorKey,
          onHorizontalDragStart: (DragStartDetails _) {
            callCount += 1;
          },
          child: Container(),
        ),
      ),
    );

    expect(
      semantics,
      includesNodeWith(
        actions: <SemanticsAction>[SemanticsAction.scrollLeft, SemanticsAction.scrollRight],
      ),
    );

    final int detectorId = detectorKey.currentContext!.findRenderObject()!.debugSemantics!.id;
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollUp,
    );
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollDown,
    );
    expect(callCount, 0);
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollLeft,
    );
    expect(callCount, 1);
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollRight,
    );
    expect(callCount, 2);

    semantics.dispose();
  });

  testWidgets('All registered handlers for the gesture kind are called', (
    WidgetTester tester,
  ) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    final Set<String> logs = <String>{};
    final GlobalKey detectorKey = GlobalKey();

    await tester.pumpWidget(
      Center(
        child: GestureDetector(
          key: detectorKey,
          onHorizontalDragStart: (_) {
            logs.add('horizontal');
          },
          onPanStart: (_) {
            logs.add('pan');
          },
          child: Container(),
        ),
      ),
    );

    final int detectorId = detectorKey.currentContext!.findRenderObject()!.debugSemantics!.id;
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollLeft,
    );
    expect(logs, <String>{'horizontal', 'pan'});

    semantics.dispose();
  });

  testWidgets('Replacing recognizers should update semantic handlers', (WidgetTester tester) async {
    final SemanticsTester semantics = SemanticsTester(tester);

    // How the test is set up:
    //
    //  * In the base state, RawGestureDetector's recognizer is a HorizontalGR
    //  * Calling `introduceLayoutPerformer()` adds a `_TestLayoutPerformer` as
    //    child of RawGestureDetector, which invokes a given callback during
    //    layout phase.
    //  * The aforementioned callback replaces the detector's recognizer with a
    //    TapGR.
    //  * This test makes sure the replacement correctly updates semantics.

    final Set<String> logs = <String>{};
    final GlobalKey<RawGestureDetectorState> detectorKey = GlobalKey();
    void performLayout() {
      detectorKey.currentState!.replaceGestureRecognizers(<Type, GestureRecognizerFactory>{
        TapGestureRecognizer: GestureRecognizerFactoryWithHandlers<TapGestureRecognizer>(
          () => TapGestureRecognizer(),
          (TapGestureRecognizer instance) {
            instance.onTap = () {
              logs.add('tap');
            };
          },
        ),
      });
    }

    bool hasLayoutPerformer = false;
    late VoidCallback introduceLayoutPerformer;
    await tester.pumpWidget(
      StatefulBuilder(
        builder: (BuildContext context, StateSetter setter) {
          introduceLayoutPerformer = () {
            setter(() {
              hasLayoutPerformer = true;
            });
          };
          return Center(
            child: RawGestureDetector(
              key: detectorKey,
              gestures: <Type, GestureRecognizerFactory>{
                HorizontalDragGestureRecognizer:
                    GestureRecognizerFactoryWithHandlers<HorizontalDragGestureRecognizer>(
                      () => HorizontalDragGestureRecognizer(),
                      (HorizontalDragGestureRecognizer instance) {
                        instance.onStart = (_) {
                          logs.add('horizontal');
                        };
                      },
                    ),
              },
              child: hasLayoutPerformer ? _TestLayoutPerformer(performLayout: performLayout) : null,
            ),
          );
        },
      ),
    );

    final int detectorId = detectorKey.currentContext!.findRenderObject()!.debugSemantics!.id;
    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollLeft,
    );
    expect(logs, <String>{'horizontal'});
    logs.clear();

    introduceLayoutPerformer();
    await tester.pumpAndSettle();

    tester.binding.pipelineOwner.semanticsOwner!.performAction(
      detectorId,
      SemanticsAction.scrollLeft,
    );
    tester.binding.pipelineOwner.semanticsOwner!.performAction(detectorId, SemanticsAction.tap);
    expect(logs, <String>{'tap'});
    logs.clear();

    semantics.dispose();
  });

  group("RawGestureDetector's custom semantics delegate", () {
    testWidgets('should update semantics notations when switching from the default delegate', (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final Map<Type, GestureRecognizerFactory> gestures = _buildGestureMap(
        () => LongPressGestureRecognizer(),
        null,
      )..addAll(_buildGestureMap(() => TapGestureRecognizer(), null));
      await tester.pumpWidget(
        Center(child: RawGestureDetector(gestures: gestures, child: Container())),
      );

      expect(
        semantics,
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.longPress, SemanticsAction.tap],
        ),
      );

      await tester.pumpWidget(
        Center(
          child: RawGestureDetector(
            gestures: gestures,
            semantics: _TestSemanticsGestureDelegate(onTap: () {}),
            child: Container(),
          ),
        ),
      );

      expect(semantics, includesNodeWith(actions: <SemanticsAction>[SemanticsAction.tap]));

      semantics.dispose();
    });

    testWidgets('should update semantics notations when switching to the default delegate', (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final Map<Type, GestureRecognizerFactory> gestures = _buildGestureMap(
        () => LongPressGestureRecognizer(),
        null,
      )..addAll(_buildGestureMap(() => TapGestureRecognizer(), null));
      await tester.pumpWidget(
        Center(
          child: RawGestureDetector(
            gestures: gestures,
            semantics: _TestSemanticsGestureDelegate(onTap: () {}),
            child: Container(),
          ),
        ),
      );

      expect(semantics, includesNodeWith(actions: <SemanticsAction>[SemanticsAction.tap]));

      await tester.pumpWidget(
        Center(child: RawGestureDetector(gestures: gestures, child: Container())),
      );

      expect(
        semantics,
        includesNodeWith(
          actions: <SemanticsAction>[SemanticsAction.longPress, SemanticsAction.tap],
        ),
      );

      semantics.dispose();
    });

    testWidgets(
      'should update semantics notations when switching from a different custom delegate',
      (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        final Map<Type, GestureRecognizerFactory> gestures = _buildGestureMap(
          () => LongPressGestureRecognizer(),
          null,
        )..addAll(_buildGestureMap(() => TapGestureRecognizer(), null));
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(
              gestures: gestures,
              semantics: _TestSemanticsGestureDelegate(onTap: () {}),
              child: Container(),
            ),
          ),
        );

        expect(semantics, includesNodeWith(actions: <SemanticsAction>[SemanticsAction.tap]));

        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(
              gestures: gestures,
              semantics: _TestSemanticsGestureDelegate(onLongPress: () {}),
              child: Container(),
            ),
          ),
        );

        expect(semantics, includesNodeWith(actions: <SemanticsAction>[SemanticsAction.longPress]));

        semantics.dispose();
      },
    );

    testWidgets('should correctly call callbacks', (WidgetTester tester) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      final List<String> logs = <String>[];
      final GlobalKey<RawGestureDetectorState> detectorKey = GlobalKey();
      await tester.pumpWidget(
        Center(
          child: RawGestureDetector(
            key: detectorKey,
            semantics: _TestSemanticsGestureDelegate(
              onTap: () {
                logs.add('tap');
              },
              onLongPress: () {
                logs.add('longPress');
              },
              onHorizontalDragUpdate: (_) {
                logs.add('horizontal');
              },
              onVerticalDragUpdate: (_) {
                logs.add('vertical');
              },
            ),
            child: Container(),
          ),
        ),
      );

      final int detectorId = detectorKey.currentContext!.findRenderObject()!.debugSemantics!.id;
      tester.binding.pipelineOwner.semanticsOwner!.performAction(detectorId, SemanticsAction.tap);
      expect(logs, <String>['tap']);
      logs.clear();

      tester.binding.pipelineOwner.semanticsOwner!.performAction(
        detectorId,
        SemanticsAction.longPress,
      );
      expect(logs, <String>['longPress']);
      logs.clear();

      tester.binding.pipelineOwner.semanticsOwner!.performAction(
        detectorId,
        SemanticsAction.scrollLeft,
      );
      expect(logs, <String>['horizontal']);
      logs.clear();

      tester.binding.pipelineOwner.semanticsOwner!.performAction(
        detectorId,
        SemanticsAction.scrollUp,
      );
      expect(logs, <String>['vertical']);
      logs.clear();

      semantics.dispose();
    });
  });

  group("RawGestureDetector's default semantics delegate", () {
    group('should map onTap to', () {
      testWidgets('null when there is no TapGR', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(gestures: _buildGestureMap(null, null), child: Container()),
          ),
        );

        expect(
          semantics,
          isNot(
            includesNodeWith(
              actions: <SemanticsAction>[SemanticsAction.tap, SemanticsAction.focus],
            ),
          ),
        );

        semantics.dispose();
      });

      testWidgets('non-null when there is TapGR with no callbacks', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(
              gestures: _buildGestureMap(() => TapGestureRecognizer(), null),
              child: Container(),
            ),
          ),
        );

        expect(semantics, includesNodeWith(actions: <SemanticsAction>[SemanticsAction.tap]));

        semantics.dispose();
      });

      testWidgets('a callback that correctly calls callbacks', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        final GlobalKey detectorKey = GlobalKey();
        final List<String> logs = <String>[];
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(
              key: detectorKey,
              gestures: _buildGestureMap(() => TapGestureRecognizer(), (TapGestureRecognizer tap) {
                tap
                  ..onTap = () {
                    logs.add('tap');
                  }
                  ..onTapUp = (_) {
                    logs.add('tapUp');
                  }
                  ..onTapDown = (_) {
                    logs.add('tapDown');
                  }
                  ..onTapCancel = () {
                    logs.add('WRONG');
                  }
                  ..onSecondaryTapDown = (_) {
                    logs.add('WRONG');
                  }
                  ..onTertiaryTapDown = (_) {
                    logs.add('WRONG');
                  };
              }),
              child: Container(),
            ),
          ),
        );

        final int detectorId = detectorKey.currentContext!.findRenderObject()!.debugSemantics!.id;
        tester.binding.pipelineOwner.semanticsOwner!.performAction(detectorId, SemanticsAction.tap);
        expect(logs, <String>['tapDown', 'tapUp', 'tap']);

        semantics.dispose();
      });
    });

    group('should map onLongPress to', () {
      testWidgets('null when there is no LongPressGR ', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(gestures: _buildGestureMap(null, null), child: Container()),
          ),
        );

        expect(
          semantics,
          isNot(includesNodeWith(actions: <SemanticsAction>[SemanticsAction.longPress])),
        );

        semantics.dispose();
      });

      testWidgets('non-null when there is LongPressGR with no callbacks', (
        WidgetTester tester,
      ) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(
              gestures: _buildGestureMap(() => LongPressGestureRecognizer(), null),
              child: Container(),
            ),
          ),
        );

        expect(semantics, includesNodeWith(actions: <SemanticsAction>[SemanticsAction.longPress]));

        semantics.dispose();
      });

      testWidgets('a callback that correctly calls callbacks', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        final GlobalKey detectorKey = GlobalKey();
        final List<String> logs = <String>[];
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(
              key: detectorKey,
              gestures: _buildGestureMap(() => LongPressGestureRecognizer(), (
                LongPressGestureRecognizer longPress,
              ) {
                longPress
                  ..onLongPress = () {
                    logs.add('LP');
                  }
                  ..onLongPressStart = (_) {
                    logs.add('LPStart');
                  }
                  ..onLongPressUp = () {
                    logs.add('LPUp');
                  }
                  ..onLongPressEnd = (_) {
                    logs.add('LPEnd');
                  }
                  ..onLongPressMoveUpdate = (_) {
                    logs.add('WRONG');
                  };
              }),
              child: Container(),
            ),
          ),
        );

        final int detectorId = detectorKey.currentContext!.findRenderObject()!.debugSemantics!.id;
        tester.binding.pipelineOwner.semanticsOwner!.performAction(
          detectorId,
          SemanticsAction.longPress,
        );
        expect(logs, <String>['LPStart', 'LP', 'LPEnd', 'LPUp']);

        semantics.dispose();
      });
    });

    group('should map onHorizontalDragUpdate to', () {
      testWidgets('null when there is no matching recognizers ', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(gestures: _buildGestureMap(null, null), child: Container()),
          ),
        );

        expect(
          semantics,
          isNot(
            includesNodeWith(
              actions: <SemanticsAction>[SemanticsAction.scrollLeft, SemanticsAction.scrollRight],
            ),
          ),
        );

        semantics.dispose();
      });

      testWidgets('non-null when there is either matching recognizer with no callbacks', (
        WidgetTester tester,
      ) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(
              gestures: _buildGestureMap(() => HorizontalDragGestureRecognizer(), null),
              child: Container(),
            ),
          ),
        );

        expect(
          semantics,
          includesNodeWith(
            actions: <SemanticsAction>[SemanticsAction.scrollLeft, SemanticsAction.scrollRight],
          ),
        );

        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(
              gestures: _buildGestureMap(() => PanGestureRecognizer(), null),
              child: Container(),
            ),
          ),
        );

        expect(
          semantics,
          includesNodeWith(
            actions: <SemanticsAction>[
              SemanticsAction.scrollLeft,
              SemanticsAction.scrollRight,
              SemanticsAction.scrollDown,
              SemanticsAction.scrollUp,
            ],
          ),
        );

        semantics.dispose();
      });

      testWidgets('a callback that correctly calls callbacks', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        final GlobalKey detectorKey = GlobalKey();
        final List<String> logs = <String>[];
        final Map<Type, GestureRecognizerFactory> gestures = _buildGestureMap(
          () => HorizontalDragGestureRecognizer(),
          (HorizontalDragGestureRecognizer horizontal) {
            horizontal
              ..onStart = (_) {
                logs.add('HStart');
              }
              ..onDown = (_) {
                logs.add('HDown');
              }
              ..onEnd = (_) {
                logs.add('HEnd');
              }
              ..onUpdate = (_) {
                logs.add('HUpdate');
              }
              ..onCancel = () {
                logs.add('WRONG');
              };
          },
        )..addAll(
          _buildGestureMap(() => PanGestureRecognizer(), (PanGestureRecognizer pan) {
            pan
              ..onStart = (_) {
                logs.add('PStart');
              }
              ..onDown = (_) {
                logs.add('PDown');
              }
              ..onEnd = (_) {
                logs.add('PEnd');
              }
              ..onUpdate = (_) {
                logs.add('PUpdate');
              }
              ..onCancel = () {
                logs.add('WRONG');
              };
          }),
        );
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(key: detectorKey, gestures: gestures, child: Container()),
          ),
        );

        final int detectorId = detectorKey.currentContext!.findRenderObject()!.debugSemantics!.id;
        tester.binding.pipelineOwner.semanticsOwner!.performAction(
          detectorId,
          SemanticsAction.scrollLeft,
        );
        expect(logs, <String>[
          'HDown',
          'HStart',
          'HUpdate',
          'HEnd',
          'PDown',
          'PStart',
          'PUpdate',
          'PEnd',
        ]);
        logs.clear();

        tester.binding.pipelineOwner.semanticsOwner!.performAction(
          detectorId,
          SemanticsAction.scrollLeft,
        );
        expect(logs, <String>[
          'HDown',
          'HStart',
          'HUpdate',
          'HEnd',
          'PDown',
          'PStart',
          'PUpdate',
          'PEnd',
        ]);

        semantics.dispose();
      });
    });

    group('should map onVerticalDragUpdate to', () {
      testWidgets('null when there is no matching recognizers ', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(gestures: _buildGestureMap(null, null), child: Container()),
          ),
        );

        expect(
          semantics,
          isNot(
            includesNodeWith(
              actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollDown],
            ),
          ),
        );

        semantics.dispose();
      });

      testWidgets('non-null when there is either matching recognizer with no callbacks', (
        WidgetTester tester,
      ) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(
              gestures: _buildGestureMap(() => VerticalDragGestureRecognizer(), null),
              child: Container(),
            ),
          ),
        );

        expect(
          semantics,
          includesNodeWith(
            actions: <SemanticsAction>[SemanticsAction.scrollUp, SemanticsAction.scrollDown],
          ),
        );

        // Pan has bene tested in Horizontal

        semantics.dispose();
      });

      testWidgets('a callback that correctly calls callbacks', (WidgetTester tester) async {
        final SemanticsTester semantics = SemanticsTester(tester);
        final GlobalKey detectorKey = GlobalKey();
        final List<String> logs = <String>[];
        final Map<Type, GestureRecognizerFactory> gestures = _buildGestureMap(
          () => VerticalDragGestureRecognizer(),
          (VerticalDragGestureRecognizer horizontal) {
            horizontal
              ..onStart = (_) {
                logs.add('VStart');
              }
              ..onDown = (_) {
                logs.add('VDown');
              }
              ..onEnd = (_) {
                logs.add('VEnd');
              }
              ..onUpdate = (_) {
                logs.add('VUpdate');
              }
              ..onCancel = () {
                logs.add('WRONG');
              };
          },
        )..addAll(
          _buildGestureMap(() => PanGestureRecognizer(), (PanGestureRecognizer pan) {
            pan
              ..onStart = (_) {
                logs.add('PStart');
              }
              ..onDown = (_) {
                logs.add('PDown');
              }
              ..onEnd = (_) {
                logs.add('PEnd');
              }
              ..onUpdate = (_) {
                logs.add('PUpdate');
              }
              ..onCancel = () {
                logs.add('WRONG');
              };
          }),
        );
        await tester.pumpWidget(
          Center(
            child: RawGestureDetector(key: detectorKey, gestures: gestures, child: Container()),
          ),
        );

        final int detectorId = detectorKey.currentContext!.findRenderObject()!.debugSemantics!.id;
        tester.binding.pipelineOwner.semanticsOwner!.performAction(
          detectorId,
          SemanticsAction.scrollUp,
        );
        expect(logs, <String>[
          'VDown',
          'VStart',
          'VUpdate',
          'VEnd',
          'PDown',
          'PStart',
          'PUpdate',
          'PEnd',
        ]);
        logs.clear();

        tester.binding.pipelineOwner.semanticsOwner!.performAction(
          detectorId,
          SemanticsAction.scrollDown,
        );
        expect(logs, <String>[
          'VDown',
          'VStart',
          'VUpdate',
          'VEnd',
          'PDown',
          'PStart',
          'PUpdate',
          'PEnd',
        ]);

        semantics.dispose();
      });
    });

    testWidgets('should update semantics notations when receiving new gestures', (
      WidgetTester tester,
    ) async {
      final SemanticsTester semantics = SemanticsTester(tester);
      await tester.pumpWidget(
        Center(
          child: RawGestureDetector(
            gestures: _buildGestureMap(() => LongPressGestureRecognizer(), null),
            child: Container(),
          ),
        ),
      );

      expect(semantics, includesNodeWith(actions: <SemanticsAction>[SemanticsAction.longPress]));

      await tester.pumpWidget(
        Center(
          child: RawGestureDetector(
            gestures: _buildGestureMap(() => TapGestureRecognizer(), null),
            child: Container(),
          ),
        ),
      );

      expect(semantics, includesNodeWith(actions: <SemanticsAction>[SemanticsAction.tap]));

      semantics.dispose();
    });
  });
}

class _TestLayoutPerformer extends SingleChildRenderObjectWidget {
  const _TestLayoutPerformer({required this.performLayout});

  final VoidCallback performLayout;

  @override
  _RenderTestLayoutPerformer createRenderObject(BuildContext context) {
    return _RenderTestLayoutPerformer(performLayout: performLayout);
  }
}

class _RenderTestLayoutPerformer extends RenderBox {
  _RenderTestLayoutPerformer({required VoidCallback performLayout})
    : _performLayout = performLayout;

  final VoidCallback _performLayout;

  @override
  Size computeDryLayout(BoxConstraints constraints) {
    return const Size(1, 1);
  }

  @override
  void performLayout() {
    size = const Size(1, 1);
    _performLayout();
  }
}

Map<Type, GestureRecognizerFactory> _buildGestureMap<T extends GestureRecognizer>(
  GestureRecognizerFactoryConstructor<T>? constructor,
  GestureRecognizerFactoryInitializer<T>? initializer,
) {
  if (constructor == null) {
    return <Type, GestureRecognizerFactory>{};
  }
  return <Type, GestureRecognizerFactory>{
    T: GestureRecognizerFactoryWithHandlers<T>(constructor, initializer ?? (T o) {}),
  };
}

class _TestSemanticsGestureDelegate extends SemanticsGestureDelegate {
  const _TestSemanticsGestureDelegate({
    this.onTap,
    this.onLongPress,
    this.onHorizontalDragUpdate,
    this.onVerticalDragUpdate,
  });

  final GestureTapCallback? onTap;
  final GestureLongPressCallback? onLongPress;
  final GestureDragUpdateCallback? onHorizontalDragUpdate;
  final GestureDragUpdateCallback? onVerticalDragUpdate;

  @override
  void assignSemantics(RenderSemanticsGestureHandler renderObject) {
    renderObject
      ..onTap = onTap
      ..onLongPress = onLongPress
      ..onHorizontalDragUpdate = onHorizontalDragUpdate
      ..onVerticalDragUpdate = onVerticalDragUpdate;
  }
}
