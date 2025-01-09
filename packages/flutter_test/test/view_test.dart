// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:collection/collection.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'utils/fake_and_mock_utils.dart';

void main() {
  group('TestFlutterView', () {
    FlutterView trueImplicitView() => PlatformDispatcher.instance.implicitView!;
    FlutterView boundImplicitView() => WidgetsBinding.instance.platformDispatcher.implicitView!;

    tearDown(() {
      final TestFlutterView view =
          (WidgetsBinding.instance as TestWidgetsFlutterBinding).platformDispatcher.views.single;
      view.reset();
    });

    testWidgets('can handle new methods without breaking', (WidgetTester tester) async {
      final dynamic testView = tester.view;
      // ignore: avoid_dynamic_calls
      expect(testView.someNewProperty, null);
    });

    testWidgets('can fake devicePixelRatio', (WidgetTester tester) async {
      verifyPropertyFaked<double>(
        tester: tester,
        realValue: trueImplicitView().devicePixelRatio,
        fakeValue: 2.5,
        propertyRetriever: () => boundImplicitView().devicePixelRatio,
        propertyFaker: (_, double fakeValue) {
          tester.view.devicePixelRatio = fakeValue;
        },
      );
    });

    testWidgets('can reset devicePixelRatio', (WidgetTester tester) async {
      verifyPropertyReset<double>(
        tester: tester,
        fakeValue: 2.5,
        propertyRetriever: () => boundImplicitView().devicePixelRatio,
        propertyResetter: () {
          tester.view.resetDevicePixelRatio();
        },
        propertyFaker: (double fakeValue) {
          tester.view.devicePixelRatio = fakeValue;
        },
      );
    });

    testWidgets('updating devicePixelRatio also updates display.devicePixelRatio', (
      WidgetTester tester,
    ) async {
      tester.view.devicePixelRatio = tester.view.devicePixelRatio + 1;

      expect(tester.view.display.devicePixelRatio, tester.view.devicePixelRatio);
    });

    testWidgets('can fake displayFeatures', (WidgetTester tester) async {
      verifyPropertyFaked<List<DisplayFeature>>(
        tester: tester,
        realValue: trueImplicitView().displayFeatures,
        fakeValue: <DisplayFeature>[
          const DisplayFeature(
            bounds: Rect.fromLTWH(0, 0, 500, 30),
            type: DisplayFeatureType.unknown,
            state: DisplayFeatureState.unknown,
          ),
        ],
        propertyRetriever: () => boundImplicitView().displayFeatures,
        propertyFaker: (_, List<DisplayFeature> fakeValue) {
          tester.view.displayFeatures = fakeValue;
        },
      );
    });

    testWidgets('can reset displayFeatures', (WidgetTester tester) async {
      verifyPropertyReset<List<DisplayFeature>>(
        tester: tester,
        fakeValue: <DisplayFeature>[
          const DisplayFeature(
            bounds: Rect.fromLTWH(0, 0, 500, 30),
            type: DisplayFeatureType.unknown,
            state: DisplayFeatureState.unknown,
          ),
        ],
        propertyRetriever: () => boundImplicitView().displayFeatures,
        propertyResetter: () {
          tester.view.resetDisplayFeatures();
        },
        propertyFaker: (List<DisplayFeature> fakeValue) {
          tester.view.displayFeatures = fakeValue;
        },
      );
    });

    testWidgets('can fake padding', (WidgetTester tester) async {
      verifyPropertyFaked<ViewPadding>(
        tester: tester,
        realValue: trueImplicitView().padding,
        fakeValue: FakeViewPadding.zero,
        propertyRetriever: () => boundImplicitView().padding,
        propertyFaker: (_, ViewPadding fakeValue) {
          tester.view.padding = fakeValue as FakeViewPadding;
        },
        matcher: matchesViewPadding,
      );
    });

    testWidgets('can reset padding', (WidgetTester tester) async {
      verifyPropertyReset<ViewPadding>(
        tester: tester,
        fakeValue: FakeViewPadding.zero,
        propertyRetriever: () => boundImplicitView().padding,
        propertyResetter: () {
          tester.view.resetPadding();
        },
        propertyFaker: (ViewPadding fakeValue) {
          tester.view.padding = fakeValue as FakeViewPadding;
        },
        matcher: matchesViewPadding,
      );
    });

    testWidgets('can fake physicalSize', (WidgetTester tester) async {
      verifyPropertyFaked<Size>(
        tester: tester,
        realValue: trueImplicitView().physicalSize,
        fakeValue: const Size(50, 50),
        propertyRetriever: () => boundImplicitView().physicalSize,
        propertyFaker: (_, Size fakeValue) {
          tester.view.physicalSize = fakeValue;
        },
      );
    });

    testWidgets('faking physicalSize fakes physicalConstraints', (WidgetTester tester) async {
      const Size fakeSize = Size(50, 50);
      verifyPropertyFaked<ViewConstraints>(
        tester: tester,
        realValue: trueImplicitView().physicalConstraints,
        fakeValue: ViewConstraints.tight(fakeSize),
        propertyRetriever: () => boundImplicitView().physicalConstraints,
        propertyFaker: (_, __) {
          tester.view.physicalSize = fakeSize;
        },
      );
    });

    testWidgets('can reset physicalSize', (WidgetTester tester) async {
      verifyPropertyReset<Size>(
        tester: tester,
        fakeValue: const Size(50, 50),
        propertyRetriever: () => boundImplicitView().physicalSize,
        propertyResetter: () {
          tester.view.resetPhysicalSize();
        },
        propertyFaker: (Size fakeValue) {
          tester.view.physicalSize = fakeValue;
        },
      );
    });

    testWidgets('resetting physicalSize resets physicalConstraints', (WidgetTester tester) async {
      const Size fakeSize = Size(50, 50);
      verifyPropertyReset<ViewConstraints>(
        tester: tester,
        fakeValue: ViewConstraints.tight(fakeSize),
        propertyRetriever: () => boundImplicitView().physicalConstraints,
        propertyResetter: () {
          tester.view.resetPhysicalSize();
        },
        propertyFaker: (_) {
          tester.view.physicalSize = fakeSize;
        },
      );
    });

    testWidgets('can fake physicalConstraints', (WidgetTester tester) async {
      verifyPropertyFaked<ViewConstraints>(
        tester: tester,
        realValue: trueImplicitView().physicalConstraints,
        fakeValue: const ViewConstraints(minWidth: 1, maxWidth: 2, minHeight: 3, maxHeight: 4),
        propertyRetriever: () => boundImplicitView().physicalConstraints,
        propertyFaker: (_, ViewConstraints fakeValue) {
          tester.view.physicalConstraints = fakeValue;
        },
      );
    });

    testWidgets('can reset physicalConstraints', (WidgetTester tester) async {
      verifyPropertyReset<ViewConstraints>(
        tester: tester,
        fakeValue: const ViewConstraints(minWidth: 1, maxWidth: 2, minHeight: 3, maxHeight: 4),
        propertyRetriever: () => boundImplicitView().physicalConstraints,
        propertyResetter: () {
          tester.view.resetPhysicalConstraints();
        },
        propertyFaker: (ViewConstraints fakeValue) {
          tester.view.physicalConstraints = fakeValue;
        },
      );
    });

    testWidgets('can fake systemGestureInsets', (WidgetTester tester) async {
      verifyPropertyFaked<ViewPadding>(
        tester: tester,
        realValue: trueImplicitView().systemGestureInsets,
        fakeValue: FakeViewPadding.zero,
        propertyRetriever: () => boundImplicitView().systemGestureInsets,
        propertyFaker: (_, ViewPadding fakeValue) {
          tester.view.systemGestureInsets = fakeValue as FakeViewPadding;
        },
        matcher: matchesViewPadding,
      );
    });

    testWidgets('can reset systemGestureInsets', (WidgetTester tester) async {
      verifyPropertyReset<ViewPadding>(
        tester: tester,
        fakeValue: FakeViewPadding.zero,
        propertyRetriever: () => boundImplicitView().systemGestureInsets,
        propertyResetter: () {
          tester.view.resetSystemGestureInsets();
        },
        propertyFaker: (ViewPadding fakeValue) {
          tester.view.systemGestureInsets = fakeValue as FakeViewPadding;
        },
        matcher: matchesViewPadding,
      );
    });

    testWidgets('can fake viewInsets', (WidgetTester tester) async {
      verifyPropertyFaked<ViewPadding>(
        tester: tester,
        realValue: trueImplicitView().viewInsets,
        fakeValue: FakeViewPadding.zero,
        propertyRetriever: () => boundImplicitView().viewInsets,
        propertyFaker: (_, ViewPadding fakeValue) {
          tester.view.viewInsets = fakeValue as FakeViewPadding;
        },
        matcher: matchesViewPadding,
      );
    });

    testWidgets('can reset viewInsets', (WidgetTester tester) async {
      verifyPropertyReset<ViewPadding>(
        tester: tester,
        fakeValue: FakeViewPadding.zero,
        propertyRetriever: () => boundImplicitView().viewInsets,
        propertyResetter: () {
          tester.view.resetViewInsets();
        },
        propertyFaker: (ViewPadding fakeValue) {
          tester.view.viewInsets = fakeValue as FakeViewPadding;
        },
        matcher: matchesViewPadding,
      );
    });

    testWidgets('can fake viewPadding', (WidgetTester tester) async {
      verifyPropertyFaked<ViewPadding>(
        tester: tester,
        realValue: trueImplicitView().viewPadding,
        fakeValue: FakeViewPadding.zero,
        propertyRetriever: () => boundImplicitView().viewPadding,
        propertyFaker: (_, ViewPadding fakeValue) {
          tester.view.viewPadding = fakeValue as FakeViewPadding;
        },
        matcher: matchesViewPadding,
      );
    });

    testWidgets('can reset viewPadding', (WidgetTester tester) async {
      verifyPropertyReset<ViewPadding>(
        tester: tester,
        fakeValue: FakeViewPadding.zero,
        propertyRetriever: () => boundImplicitView().viewPadding,
        propertyResetter: () {
          tester.view.resetViewPadding();
        },
        propertyFaker: (ViewPadding fakeValue) {
          tester.view.viewPadding = fakeValue as FakeViewPadding;
        },
        matcher: matchesViewPadding,
      );
    });

    testWidgets('can clear out fake properties all at once', (WidgetTester tester) async {
      final FlutterViewSnapshot initial = FlutterViewSnapshot(tester.view);

      tester.view.devicePixelRatio = 7;
      tester.view.displayFeatures = <DisplayFeature>[
        const DisplayFeature(
          bounds: Rect.fromLTWH(0, 0, 20, 300),
          type: DisplayFeatureType.unknown,
          state: DisplayFeatureState.unknown,
        ),
      ];
      tester.view.padding = FakeViewPadding.zero;
      tester.view.systemGestureInsets = FakeViewPadding.zero;
      tester.view.viewInsets = FakeViewPadding.zero;
      tester.view.viewPadding = FakeViewPadding.zero;
      tester.view.gestureSettings = const GestureSettings(
        physicalTouchSlop: 4,
        physicalDoubleTapSlop: 5,
      );

      final FlutterViewSnapshot faked = FlutterViewSnapshot(tester.view);

      tester.view.reset();

      final FlutterViewSnapshot reset = FlutterViewSnapshot(tester.view);

      expect(initial, isNot(matchesSnapshot(faked)));
      expect(initial, matchesSnapshot(reset));
    });

    testWidgets('render is passed through to backing FlutterView', (WidgetTester tester) async {
      final Scene expectedScene = SceneBuilder().build();
      final _FakeFlutterView backingView = _FakeFlutterView();
      final TestFlutterView view = TestFlutterView(
        view: backingView,
        platformDispatcher: tester.binding.platformDispatcher,
        display: _FakeDisplay(),
      );

      view.render(expectedScene);

      expect(backingView.lastRenderedScene, isNotNull);
      expect(backingView.lastRenderedScene, expectedScene);
    });

    testWidgets('updateSemantics is passed through to backing FlutterView', (
      WidgetTester tester,
    ) async {
      final SemanticsUpdate expectedUpdate = SemanticsUpdateBuilder().build();
      final _FakeFlutterView backingView = _FakeFlutterView();
      final TestFlutterView view = TestFlutterView(
        view: backingView,
        platformDispatcher: tester.binding.platformDispatcher,
        display: _FakeDisplay(),
      );

      view.updateSemantics(expectedUpdate);

      expect(backingView.lastSemanticsUpdate, isNotNull);
      expect(backingView.lastSemanticsUpdate, expectedUpdate);
    });
  });
}

Matcher matchesSnapshot(FlutterViewSnapshot expected) => _FlutterViewSnapshotMatcher(expected);

class _FlutterViewSnapshotMatcher extends Matcher {
  _FlutterViewSnapshotMatcher(this.expected);

  final FlutterViewSnapshot expected;

  @override
  Description describe(Description description) {
    description.add('snapshot of a FlutterView matches');
    return description;
  }

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    assert(item is FlutterViewSnapshot, 'Can only match against snapshots of FlutterView.');
    final FlutterViewSnapshot actual = item as FlutterViewSnapshot;

    if (actual.devicePixelRatio != expected.devicePixelRatio) {
      mismatchDescription.add(
        'actual.devicePixelRatio (${actual.devicePixelRatio}) did not match expected.devicePixelRatio (${expected.devicePixelRatio})',
      );
    }
    if (!actual.displayFeatures.equals(expected.displayFeatures)) {
      mismatchDescription.add('actual.displayFeatures did not match expected.devicePixelRatio');
      mismatchDescription.addAll('Actual: [', ',', ']', actual.displayFeatures);
      mismatchDescription.addAll('Expected: [', ',', ']', expected.displayFeatures);
    }
    if (actual.gestureSettings != expected.gestureSettings) {
      mismatchDescription.add(
        'actual.gestureSettings (${actual.gestureSettings}) did not match expected.gestureSettings (${expected.gestureSettings})',
      );
    }

    final Matcher paddingMatcher = matchesViewPadding(expected.padding);
    if (!paddingMatcher.matches(actual.padding, matchState)) {
      mismatchDescription.add(
        'actual.padding (${actual.padding}) did not match expected.padding (${expected.padding})',
      );
      paddingMatcher.describeMismatch(actual.padding, mismatchDescription, matchState, verbose);
    }

    if (actual.physicalSize != expected.physicalSize) {
      mismatchDescription.add(
        'actual.physicalSize (${actual.physicalSize}) did not match expected.physicalSize (${expected.physicalSize})',
      );
    }

    final Matcher systemGestureInsetsMatcher = matchesViewPadding(expected.systemGestureInsets);
    if (!systemGestureInsetsMatcher.matches(actual.systemGestureInsets, matchState)) {
      mismatchDescription.add(
        'actual.systemGestureInsets (${actual.systemGestureInsets}) did not match expected.systemGestureInsets (${expected.systemGestureInsets})',
      );
      systemGestureInsetsMatcher.describeMismatch(
        actual.systemGestureInsets,
        mismatchDescription,
        matchState,
        verbose,
      );
    }

    if (actual.viewId != expected.viewId) {
      mismatchDescription.add(
        'actual.viewId (${actual.viewId}) did not match expected.viewId (${expected.viewId})',
      );
    }

    final Matcher viewInsetsMatcher = matchesViewPadding(expected.viewInsets);
    if (!viewInsetsMatcher.matches(actual.viewInsets, matchState)) {
      mismatchDescription.add(
        'actual.viewInsets (${actual.viewInsets}) did not match expected.viewInsets (${expected.viewInsets})',
      );
      viewInsetsMatcher.describeMismatch(
        actual.viewInsets,
        mismatchDescription,
        matchState,
        verbose,
      );
    }

    final Matcher viewPaddingMatcher = matchesViewPadding(expected.viewPadding);
    if (!viewPaddingMatcher.matches(actual.viewPadding, matchState)) {
      mismatchDescription.add(
        'actual.viewPadding (${actual.viewPadding}) did not match expected.devicePixelRatio (${expected.viewPadding})',
      );
      viewPaddingMatcher.describeMismatch(
        actual.viewPadding,
        mismatchDescription,
        matchState,
        verbose,
      );
    }

    return mismatchDescription;
  }

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    assert(item is FlutterViewSnapshot, 'Can only match against snapshots of FlutterView.');
    final FlutterViewSnapshot actual = item as FlutterViewSnapshot;

    return actual.devicePixelRatio == expected.devicePixelRatio &&
        actual.displayFeatures.equals(expected.displayFeatures) &&
        actual.gestureSettings == expected.gestureSettings &&
        matchesViewPadding(expected.padding).matches(actual.padding, matchState) &&
        actual.physicalSize == expected.physicalSize &&
        matchesViewPadding(expected.systemGestureInsets).matches(actual.padding, matchState) &&
        actual.viewId == expected.viewId &&
        matchesViewPadding(expected.viewInsets).matches(actual.viewInsets, matchState) &&
        matchesViewPadding(expected.viewPadding).matches(actual.viewPadding, matchState);
  }
}

class FlutterViewSnapshot {
  FlutterViewSnapshot(FlutterView view)
    : devicePixelRatio = view.devicePixelRatio,
      displayFeatures = <DisplayFeature>[...view.displayFeatures],
      gestureSettings = view.gestureSettings,
      padding = view.padding,
      physicalSize = view.physicalSize,
      systemGestureInsets = view.systemGestureInsets,
      viewId = view.viewId,
      viewInsets = view.viewInsets,
      viewPadding = view.viewPadding;

  final double devicePixelRatio;
  final List<DisplayFeature> displayFeatures;
  final GestureSettings gestureSettings;
  final ViewPadding padding;
  final Size physicalSize;
  final ViewPadding systemGestureInsets;
  final Object viewId;
  final ViewPadding viewInsets;
  final ViewPadding viewPadding;
}

class _FakeFlutterView extends Fake implements FlutterView {
  SemanticsUpdate? lastSemanticsUpdate;
  Scene? lastRenderedScene;

  @override
  void updateSemantics(SemanticsUpdate update) {
    lastSemanticsUpdate = update;
  }

  @override
  void render(Scene scene, {Size? size}) {
    lastRenderedScene = scene;
  }
}

class _FakeDisplay extends Fake implements TestDisplay {}
