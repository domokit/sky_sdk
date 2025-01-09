// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

class MockOnEndFunction {
  int called = 0;

  void handler() {
    called++;
  }
}

const Duration animationDuration = Duration(milliseconds: 1000);
const Duration additionalDelay = Duration(milliseconds: 1);

void main() {
  late MockOnEndFunction mockOnEndFunction;
  const Key switchKey = Key('switchKey');

  setUp(() {
    mockOnEndFunction = MockOnEndFunction();
  });

  testWidgets('BoxConstraintsTween control test', (WidgetTester tester) async {
    final BoxConstraintsTween tween = BoxConstraintsTween(
      begin: BoxConstraints.tight(const Size(20.0, 50.0)),
      end: BoxConstraints.tight(const Size(10.0, 30.0)),
    );
    final BoxConstraints result = tween.lerp(0.25);
    expect(result.minWidth, 17.5);
    expect(result.maxWidth, 17.5);
    expect(result.minHeight, 45.0);
    expect(result.maxHeight, 45.0);
  });

  testWidgets('DecorationTween control test', (WidgetTester tester) async {
    final DecorationTween tween = DecorationTween(
      begin: const BoxDecoration(color: Color(0xFF00FF00)),
      end: const BoxDecoration(color: Color(0xFFFFFF00)),
    );
    final BoxDecoration result = tween.lerp(0.25) as BoxDecoration;
    expect(result.color, isSameColorAs(const Color(0xFF3FFF00)));
  });

  testWidgets('EdgeInsetsTween control test', (WidgetTester tester) async {
    final EdgeInsetsTween tween = EdgeInsetsTween(
      begin: const EdgeInsets.symmetric(vertical: 50.0),
      end: const EdgeInsets.only(top: 10.0, bottom: 30.0),
    );
    final EdgeInsets result = tween.lerp(0.25);
    expect(result.left, 0.0);
    expect(result.right, 0.0);
    expect(result.top, 40.0);
    expect(result.bottom, 45.0);
  });

  testWidgets('Matrix4Tween control test', (WidgetTester tester) async {
    final Matrix4Tween tween = Matrix4Tween(
      begin: Matrix4.translationValues(10.0, 20.0, 30.0),
      end: Matrix4.translationValues(14.0, 24.0, 34.0),
    );
    final Matrix4 result = tween.lerp(0.25);
    expect(result, equals(Matrix4.translationValues(11.0, 21.0, 31.0)));
  });

  testWidgets('AnimatedContainer onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedContainerWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);

    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedPadding onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedPaddingWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);

    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedAlign onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedAlignWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);

    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedPositioned onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedPositionedWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);

    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedPositionedDirectional onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedPositionedDirectionalWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);

    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedSlide onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedSlideWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);
    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedSlide transition test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(child: TestAnimatedWidget(switchKey: switchKey, state: _TestAnimatedSlideWidgetState())),
    );

    final RebuildCountingState<StatefulWidget> state =
        tester.widget<TestAnimatedWidget>(find.byType(TestAnimatedWidget)).rebuildState!;
    final Finder switchFinder = find.byKey(switchKey);
    final SlideTransition slideWidget = tester.widget<SlideTransition>(
      find.ancestor(of: find.byType(Placeholder), matching: find.byType(SlideTransition)).first,
    );

    expect(state.builds, equals(1));

    await tester.tap(switchFinder);
    expect(state.builds, equals(1));
    await tester.pump();
    expect(slideWidget.position.value, equals(Offset.zero));
    expect(state.builds, equals(2));

    await tester.pump(const Duration(milliseconds: 500));
    expect(slideWidget.position.value, equals(const Offset(0.5, 0.5)));
    expect(state.builds, equals(2));
    await tester.pump(const Duration(milliseconds: 250));
    expect(slideWidget.position.value, equals(const Offset(0.75, 0.75)));
    expect(state.builds, equals(2));
    await tester.pump(const Duration(milliseconds: 250));
    expect(slideWidget.position.value, equals(const Offset(1, 1)));
    expect(state.builds, equals(2));
  });

  testWidgets('AnimatedScale onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedScaleWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);
    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedScale transition test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(child: TestAnimatedWidget(switchKey: switchKey, state: _TestAnimatedScaleWidgetState())),
    );

    final RebuildCountingState<StatefulWidget> state =
        tester.widget<TestAnimatedWidget>(find.byType(TestAnimatedWidget)).rebuildState!;
    final Finder switchFinder = find.byKey(switchKey);
    final ScaleTransition scaleWidget = tester.widget<ScaleTransition>(
      find.ancestor(of: find.byType(Placeholder), matching: find.byType(ScaleTransition)).first,
    );

    expect(state.builds, equals(1));

    await tester.tap(switchFinder);
    expect(state.builds, equals(1));
    await tester.pump();
    expect(scaleWidget.scale.value, equals(1.0));
    expect(state.builds, equals(2));

    await tester.pump(const Duration(milliseconds: 500));
    expect(scaleWidget.scale.value, equals(1.5));
    expect(state.builds, equals(2));
    await tester.pump(const Duration(milliseconds: 250));
    expect(scaleWidget.scale.value, equals(1.75));
    expect(state.builds, equals(2));
    await tester.pump(const Duration(milliseconds: 250));
    expect(scaleWidget.scale.value, equals(2.0));
    expect(state.builds, equals(2));
  });

  testWidgets('AnimatedRotation onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedRotationWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);
    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedRotation transition test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(switchKey: switchKey, state: _TestAnimatedRotationWidgetState()),
      ),
    );

    final RebuildCountingState<StatefulWidget> state =
        tester.widget<TestAnimatedWidget>(find.byType(TestAnimatedWidget)).rebuildState!;
    final Finder switchFinder = find.byKey(switchKey);
    final RotationTransition rotationWidget = tester.widget<RotationTransition>(
      find.ancestor(of: find.byType(Placeholder), matching: find.byType(RotationTransition)).first,
    );

    expect(state.builds, equals(1));

    await tester.tap(switchFinder);
    expect(state.builds, equals(1));
    await tester.pump();
    expect(rotationWidget.turns.value, equals(0.0));
    expect(state.builds, equals(2));

    await tester.pump(const Duration(milliseconds: 500));
    expect(rotationWidget.turns.value, equals(0.75));
    expect(state.builds, equals(2));
    await tester.pump(const Duration(milliseconds: 250));
    expect(rotationWidget.turns.value, equals(1.125));
    expect(state.builds, equals(2));
    await tester.pump(const Duration(milliseconds: 250));
    expect(rotationWidget.turns.value, equals(1.5));
    expect(state.builds, equals(2));
  });

  testWidgets('AnimatedOpacity onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedOpacityWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);
    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedOpacity transition test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(switchKey: switchKey, state: _TestAnimatedOpacityWidgetState()),
      ),
    );

    final RebuildCountingState<StatefulWidget> state =
        tester.widget<TestAnimatedWidget>(find.byType(TestAnimatedWidget)).rebuildState!;
    final Finder switchFinder = find.byKey(switchKey);
    final FadeTransition opacityWidget = tester.widget<FadeTransition>(
      find.ancestor(of: find.byType(Placeholder), matching: find.byType(FadeTransition)).first,
    );

    expect(state.builds, equals(1));

    await tester.tap(switchFinder);
    expect(state.builds, equals(1));
    await tester.pump();
    expect(opacityWidget.opacity.value, equals(0.0));
    expect(state.builds, equals(2));

    await tester.pump(const Duration(milliseconds: 500));
    expect(opacityWidget.opacity.value, equals(0.5));
    expect(state.builds, equals(2));
    await tester.pump(const Duration(milliseconds: 250));
    expect(opacityWidget.opacity.value, equals(0.75));
    expect(state.builds, equals(2));
    await tester.pump(const Duration(milliseconds: 250));
    expect(opacityWidget.opacity.value, equals(1.0));
    expect(state.builds, equals(2));
  });

  testWidgets('AnimatedFractionallySizedBox onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedFractionallySizedBoxWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);
    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('SliverAnimatedOpacity onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      TestAnimatedWidget(
        callback: mockOnEndFunction.handler,
        switchKey: switchKey,
        state: _TestSliverAnimatedOpacityWidgetState(),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);

    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('SliverAnimatedOpacity transition test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          switchKey: switchKey,
          state: _TestSliverAnimatedOpacityWidgetState(),
        ),
      ),
    );

    final RebuildCountingState<StatefulWidget> state =
        tester.widget<TestAnimatedWidget>(find.byType(TestAnimatedWidget)).rebuildState!;
    final Finder switchFinder = find.byKey(switchKey);
    final SliverFadeTransition opacityWidget = tester.widget<SliverFadeTransition>(
      find
          .ancestor(of: find.byType(Placeholder), matching: find.byType(SliverFadeTransition))
          .first,
    );

    expect(state.builds, equals(1));

    await tester.tap(switchFinder);
    expect(state.builds, equals(1));
    await tester.pump();
    expect(opacityWidget.opacity.value, equals(0.0));
    expect(state.builds, equals(2));

    await tester.pump(const Duration(milliseconds: 500));
    expect(opacityWidget.opacity.value, equals(0.5));
    expect(state.builds, equals(2));
    await tester.pump(const Duration(milliseconds: 250));
    expect(opacityWidget.opacity.value, equals(0.75));
    expect(state.builds, equals(2));
    await tester.pump(const Duration(milliseconds: 250));
    expect(opacityWidget.opacity.value, equals(1.0));
    expect(state.builds, equals(2));
  });

  testWidgets('AnimatedDefaultTextStyle onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedDefaultTextStyleWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);

    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedPhysicalModel onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedPhysicalModelWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);

    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('TweenAnimationBuilder onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestTweenAnimationBuilderWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);

    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('AnimatedTheme onEnd callback test', (WidgetTester tester) async {
    await tester.pumpWidget(
      wrap(
        child: TestAnimatedWidget(
          callback: mockOnEndFunction.handler,
          switchKey: switchKey,
          state: _TestAnimatedThemeWidgetState(),
        ),
      ),
    );

    final Finder widgetFinder = find.byKey(switchKey);

    await tester.tap(widgetFinder);

    await tester.pump();
    expect(mockOnEndFunction.called, 0);
    await tester.pump(animationDuration);
    expect(mockOnEndFunction.called, 0);
    await tester.pump(additionalDelay);
    expect(mockOnEndFunction.called, 1);

    await tapTest2and3(tester, widgetFinder, mockOnEndFunction);
  });

  testWidgets('Ensure CurvedAnimations are disposed on widget change', (WidgetTester tester) async {
    final GlobalKey<ImplicitlyAnimatedWidgetState<AnimatedOpacity>> key =
        GlobalKey<ImplicitlyAnimatedWidgetState<AnimatedOpacity>>();
    final ValueNotifier<Curve> curve = ValueNotifier<Curve>(const Interval(0.0, 0.5));
    addTearDown(curve.dispose);
    await tester.pumpWidget(
      wrap(
        child: ValueListenableBuilder<Curve>(
          valueListenable: curve,
          builder:
              (_, Curve c, __) => AnimatedOpacity(
                key: key,
                opacity: 1.0,
                duration: const Duration(seconds: 1),
                curve: c,
                child: Container(color: Colors.green),
              ),
        ),
      ),
    );

    final ImplicitlyAnimatedWidgetState<AnimatedOpacity>? firstState = key.currentState;
    final Animation<double>? firstAnimation = firstState?.animation;
    if (firstAnimation == null) {
      fail('animation was null!');
    }

    final CurvedAnimation firstCurvedAnimation = firstAnimation as CurvedAnimation;

    expect(firstCurvedAnimation.isDisposed, isFalse);

    curve.value = const Interval(0.0, 0.6);
    await tester.pumpAndSettle();

    final ImplicitlyAnimatedWidgetState<AnimatedOpacity>? secondState = key.currentState;
    final Animation<double>? secondAnimation = secondState?.animation;
    if (secondAnimation == null) {
      fail('animation was null!');
    }

    final CurvedAnimation secondCurvedAnimation = secondAnimation as CurvedAnimation;

    expect(firstState, equals(secondState));
    expect(firstAnimation, isNot(equals(secondAnimation)));

    expect(firstCurvedAnimation.isDisposed, isTrue);
    expect(secondCurvedAnimation.isDisposed, isFalse);

    await tester.pumpWidget(wrap(child: const Offstage()));
    await tester.pumpAndSettle();

    expect(secondCurvedAnimation.isDisposed, isTrue);
  });

  group('Verify that default args match non-animated variants', () {
    const Widget child = SizedBox.shrink();
    const Color color = Color(0x00000000);

    testWidgets('PhysicalModel default args', (WidgetTester tester) async {
      const AnimatedPhysicalModel animatedPhysicalModel = AnimatedPhysicalModel(
        duration: Duration.zero,
        color: color,
        shadowColor: color,
        child: child,
      );
      const PhysicalModel physicalModel = PhysicalModel(
        color: color,
        shadowColor: color,
        child: child,
      );
      expect(identical(animatedPhysicalModel.shape, physicalModel.shape), isTrue);
      expect(identical(animatedPhysicalModel.clipBehavior, physicalModel.clipBehavior), isTrue);
      expect(identical(animatedPhysicalModel.borderRadius, physicalModel.borderRadius), isTrue);
    });
    // TODO(nate-thegrate): add every class!
  });
}

Future<void> tapTest2and3(
  WidgetTester tester,
  Finder widgetFinder,
  MockOnEndFunction mockOnEndFunction,
) async {
  await tester.tap(widgetFinder);

  await tester.pump();
  await tester.pump(animationDuration + additionalDelay);
  expect(mockOnEndFunction.called, 2);

  await tester.tap(widgetFinder);

  await tester.pump();
  await tester.pump(animationDuration + additionalDelay);
  expect(mockOnEndFunction.called, 3);
}

Widget wrap({required Widget child}) {
  return Directionality(
    textDirection: TextDirection.ltr,
    child: Material(child: Center(child: child)),
  );
}

abstract class RebuildCountingState<T extends StatefulWidget> extends State<T> {
  int builds = 0;
}

class TestAnimatedWidget extends StatefulWidget {
  const TestAnimatedWidget({
    super.key,
    this.callback,
    required this.switchKey,
    required this.state,
  });
  final VoidCallback? callback;
  final Key switchKey;
  final State<StatefulWidget> state;

  RebuildCountingState<StatefulWidget>? get rebuildState =>
      state is RebuildCountingState<StatefulWidget>
          ? state as RebuildCountingState<StatefulWidget>
          : null;

  @override
  State<StatefulWidget> createState() => state; // ignore: no_logic_in_create_state, this test predates the lint
}

abstract class _TestAnimatedWidgetState extends RebuildCountingState<TestAnimatedWidget> {
  bool toggle = false;
  final Widget child = const Placeholder();
  final Duration duration = animationDuration;

  void onChanged(bool v) {
    setState(() {
      toggle = v;
    });
  }

  Widget getAnimatedWidget();

  @override
  Widget build(BuildContext context) {
    builds++;
    final Widget animatedWidget = getAnimatedWidget();

    return Stack(
      children: <Widget>[
        animatedWidget,
        Switch(key: widget.switchKey, value: toggle, onChanged: onChanged),
      ],
    );
  }
}

class _TestAnimatedContainerWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedContainer(
      duration: duration,
      onEnd: widget.callback,
      width: toggle ? 10 : 20,
      foregroundDecoration: toggle ? const BoxDecoration() : null,
      child: child,
    );
  }
}

class _TestAnimatedPaddingWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedPadding(
      duration: duration,
      onEnd: widget.callback,
      padding: toggle ? const EdgeInsets.all(8.0) : const EdgeInsets.all(16.0),
      child: child,
    );
  }
}

class _TestAnimatedAlignWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedAlign(
      duration: duration,
      onEnd: widget.callback,
      alignment: toggle ? Alignment.topLeft : Alignment.bottomRight,
      child: child,
    );
  }
}

class _TestAnimatedPositionedWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedPositioned(
      duration: duration,
      onEnd: widget.callback,
      left: toggle ? 10 : 20,
      child: child,
    );
  }
}

class _TestAnimatedPositionedDirectionalWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedPositionedDirectional(
      duration: duration,
      onEnd: widget.callback,
      start: toggle ? 10 : 20,
      child: child,
    );
  }
}

class _TestAnimatedSlideWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedSlide(
      duration: duration,
      onEnd: widget.callback,
      offset: toggle ? const Offset(1, 1) : Offset.zero,
      child: child,
    );
  }
}

class _TestAnimatedScaleWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedScale(
      duration: duration,
      onEnd: widget.callback,
      scale: toggle ? 2.0 : 1.0,
      child: child,
    );
  }
}

class _TestAnimatedRotationWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedRotation(
      duration: duration,
      onEnd: widget.callback,
      turns: toggle ? 1.5 : 0.0,
      child: child,
    );
  }
}

class _TestAnimatedOpacityWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedOpacity(
      duration: duration,
      onEnd: widget.callback,
      opacity: toggle ? 1.0 : 0.0,
      child: child,
    );
  }
}

class _TestAnimatedFractionallySizedBoxWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedFractionallySizedBox(
      duration: duration,
      onEnd: widget.callback,
      heightFactor: toggle ? 0.25 : 0.75,
      widthFactor: toggle ? 0.25 : 0.75,
      child: child,
    );
  }
}

class _TestSliverAnimatedOpacityWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return SliverAnimatedOpacity(
      sliver: SliverToBoxAdapter(child: child),
      duration: duration,
      onEnd: widget.callback,
      opacity: toggle ? 1.0 : 0.0,
    );
  }

  @override
  Widget build(BuildContext context) {
    builds++;
    final Widget animatedWidget = getAnimatedWidget();

    return Material(
      child: Directionality(
        textDirection: TextDirection.ltr,
        child: CustomScrollView(
          slivers: <Widget>[
            animatedWidget,
            SliverToBoxAdapter(
              child: Switch(key: widget.switchKey, value: toggle, onChanged: onChanged),
            ),
          ],
        ),
      ),
    );
  }
}

class _TestAnimatedDefaultTextStyleWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedDefaultTextStyle(
      duration: duration,
      onEnd: widget.callback,
      style:
          toggle
              ? const TextStyle(fontStyle: FontStyle.italic)
              : const TextStyle(fontStyle: FontStyle.normal),
      child: child,
    );
  }
}

class _TestAnimatedPhysicalModelWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedPhysicalModel(
      duration: duration,
      onEnd: widget.callback,
      color: toggle ? Colors.red : Colors.green,
      shadowColor: Colors.blue,
      child: child,
    );
  }
}

class _TestTweenAnimationBuilderWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return TweenAnimationBuilder<double>(
      tween: toggle ? Tween<double>(begin: 1, end: 2) : Tween<double>(begin: 2, end: 1),
      duration: duration,
      onEnd: widget.callback,
      child: child,
      builder: (BuildContext context, double? size, Widget? child) {
        return SizedBox(width: size, height: size, child: child);
      },
    );
  }
}

class _TestAnimatedThemeWidgetState extends _TestAnimatedWidgetState {
  @override
  Widget getAnimatedWidget() {
    return AnimatedTheme(
      data: toggle ? ThemeData.dark() : ThemeData.light(),
      duration: duration,
      onEnd: widget.callback,
      child: child,
    );
  }
}
