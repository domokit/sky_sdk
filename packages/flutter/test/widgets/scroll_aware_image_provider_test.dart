// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui show Image;

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import '../painting/image_test_utils.dart';

void main() {
  late ui.Image testImage;

  ui.Image cloneImage() {
    final ui.Image clone = testImage.clone();
    addTearDown(clone.dispose);
    return clone;
  }

  setUpAll(() async {
    testImage = await createTestImage(width: 10, height: 10);
  });

  tearDownAll(() {
    testImage.dispose();
  });

  tearDown(() {
    imageCache.clear();
  });

  T findPhysics<T extends ScrollPhysics>(WidgetTester tester) {
    return Scrollable.of(find.byType(TestWidget).evaluate().first).position.physics as T;
  }

  ScrollMetrics findMetrics(WidgetTester tester) {
    return Scrollable.of(find.byType(TestWidget).evaluate().first).position;
  }

  testWidgets('ScrollAwareImageProvider does not delay if widget is not in scrollable', (
    WidgetTester tester,
  ) async {
    final GlobalKey<TestWidgetState> key = GlobalKey<TestWidgetState>();
    await tester.pumpWidget(TestWidget(key));

    final DisposableBuildContext context = DisposableBuildContext(key.currentState!);
    addTearDown(context.dispose);
    final TestImageProvider testImageProvider = TestImageProvider(testImage.clone());
    final ScrollAwareImageProvider<TestImageProvider> imageProvider =
        ScrollAwareImageProvider<TestImageProvider>(
          context: context,
          imageProvider: testImageProvider,
        );

    expect(testImageProvider.configuration, null);
    expect(imageCache.containsKey(testImageProvider), false);

    final ImageStream stream = imageProvider.resolve(ImageConfiguration.empty);

    expect(testImageProvider.configuration, ImageConfiguration.empty);
    expect(stream.completer, isNotNull);
    expect(stream.completer!.hasListeners, true);
    expect(imageCache.containsKey(testImageProvider), true);
    expect(imageCache.currentSize, 0);

    testImageProvider.complete();

    expect(imageCache.currentSize, 1);
  });

  testWidgets('ScrollAwareImageProvider does not delay if in scrollable that is not scrolling', (
    WidgetTester tester,
  ) async {
    final GlobalKey<TestWidgetState> key = GlobalKey<TestWidgetState>();
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(physics: RecordingPhysics(), children: <Widget>[TestWidget(key)]),
      ),
    );

    final DisposableBuildContext context = DisposableBuildContext(key.currentState!);
    addTearDown(context.dispose);
    final TestImageProvider testImageProvider = TestImageProvider(testImage.clone());
    final ScrollAwareImageProvider<TestImageProvider> imageProvider =
        ScrollAwareImageProvider<TestImageProvider>(
          context: context,
          imageProvider: testImageProvider,
        );

    expect(testImageProvider.configuration, null);
    expect(imageCache.containsKey(testImageProvider), false);

    final ImageStream stream = imageProvider.resolve(ImageConfiguration.empty);

    expect(testImageProvider.configuration, ImageConfiguration.empty);
    expect(stream.completer, isNotNull);
    expect(stream.completer!.hasListeners, true);
    expect(imageCache.containsKey(testImageProvider), true);
    expect(imageCache.currentSize, 0);

    testImageProvider.complete();

    expect(imageCache.currentSize, 1);
    expect(findPhysics<RecordingPhysics>(tester).velocities, <double>[0]);
  });

  testWidgets('ScrollAwareImageProvider does not delay if in scrollable that is scrolling slowly', (
    WidgetTester tester,
  ) async {
    final List<GlobalKey<TestWidgetState>> keys = <GlobalKey<TestWidgetState>>[];
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView.builder(
          physics: RecordingPhysics(),
          controller: scrollController,
          itemBuilder: (BuildContext context, int index) {
            keys.add(GlobalKey<TestWidgetState>());
            return TestWidget(keys.last);
          },
          itemCount: 50,
        ),
      ),
    );

    final DisposableBuildContext context = DisposableBuildContext(keys.last.currentState!);
    addTearDown(context.dispose);
    final TestImageProvider testImageProvider = TestImageProvider(testImage.clone());
    final ScrollAwareImageProvider<TestImageProvider> imageProvider =
        ScrollAwareImageProvider<TestImageProvider>(
          context: context,
          imageProvider: testImageProvider,
        );

    expect(testImageProvider.configuration, null);
    expect(imageCache.containsKey(testImageProvider), false);

    scrollController.animateTo(
      100,
      duration: const Duration(seconds: 2),
      curve: Curves.fastLinearToSlowEaseIn,
    );
    await tester.pump();
    final RecordingPhysics physics = findPhysics<RecordingPhysics>(tester);

    expect(physics.velocities.length, 0);
    final ImageStream stream = imageProvider.resolve(ImageConfiguration.empty);
    expect(physics.velocities.length, 1);
    expect(
      const ScrollPhysics().recommendDeferredLoading(
        physics.velocities.first,
        findMetrics(tester),
        find.byType(TestWidget).evaluate().first,
      ),
      false,
    );

    expect(testImageProvider.configuration, ImageConfiguration.empty);
    expect(stream.completer, isNotNull);
    expect(stream.completer!.hasListeners, true);
    expect(imageCache.containsKey(testImageProvider), true);
    expect(imageCache.currentSize, 0);

    testImageProvider.complete();

    expect(imageCache.currentSize, 1);
  });

  testWidgets('ScrollAwareImageProvider delays if in scrollable that is scrolling fast', (
    WidgetTester tester,
  ) async {
    final List<GlobalKey<TestWidgetState>> keys = <GlobalKey<TestWidgetState>>[];
    final ScrollController scrollController = ScrollController();
    addTearDown(scrollController.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView.builder(
          physics: RecordingPhysics(),
          controller: scrollController,
          itemBuilder: (BuildContext context, int index) {
            keys.add(GlobalKey<TestWidgetState>());
            return TestWidget(keys.last);
          },
          itemCount: 50,
        ),
      ),
    );

    final DisposableBuildContext context = DisposableBuildContext(keys.last.currentState!);
    addTearDown(context.dispose);
    final TestImageProvider testImageProvider = TestImageProvider(testImage.clone());
    final ScrollAwareImageProvider<TestImageProvider> imageProvider =
        ScrollAwareImageProvider<TestImageProvider>(
          context: context,
          imageProvider: testImageProvider,
        );

    expect(testImageProvider.configuration, null);
    expect(imageCache.containsKey(testImageProvider), false);

    scrollController.animateTo(
      3000,
      duration: const Duration(seconds: 2),
      curve: Curves.fastLinearToSlowEaseIn,
    );
    await tester.pump();
    final RecordingPhysics physics = findPhysics<RecordingPhysics>(tester);

    expect(physics.velocities.length, 0);
    final ImageStream stream = imageProvider.resolve(ImageConfiguration.empty);
    expect(physics.velocities.length, 1);
    expect(
      const ScrollPhysics().recommendDeferredLoading(
        physics.velocities.first,
        findMetrics(tester),
        find.byType(TestWidget).evaluate().first,
      ),
      true,
    );

    expect(testImageProvider.configuration, null);
    expect(stream.completer, null);

    expect(imageCache.containsKey(testImageProvider), false);
    expect(imageCache.currentSize, 0);

    await tester.pump(const Duration(seconds: 1));
    expect(physics.velocities.last, 0);

    expect(testImageProvider.configuration, ImageConfiguration.empty);
    expect(stream.completer, isNotNull);
    expect(stream.completer!.hasListeners, true);

    expect(imageCache.containsKey(testImageProvider), true);
    expect(imageCache.currentSize, 0);

    testImageProvider.complete();

    expect(imageCache.currentSize, 1);
  });

  testWidgets(
    'ScrollAwareImageProvider delays if in scrollable that is scrolling fast and fizzles if disposed',
    (WidgetTester tester) async {
      final List<GlobalKey<TestWidgetState>> keys = <GlobalKey<TestWidgetState>>[];
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: ListView.builder(
            physics: RecordingPhysics(),
            controller: scrollController,
            itemBuilder: (BuildContext context, int index) {
              keys.add(GlobalKey<TestWidgetState>());
              return TestWidget(keys.last);
            },
            itemCount: 50,
          ),
        ),
      );

      final DisposableBuildContext context = DisposableBuildContext(keys.last.currentState!);
      addTearDown(context.dispose);
      final TestImageProvider testImageProvider = TestImageProvider(cloneImage());
      final ScrollAwareImageProvider<TestImageProvider> imageProvider =
          ScrollAwareImageProvider<TestImageProvider>(
            context: context,
            imageProvider: testImageProvider,
          );

      expect(testImageProvider.configuration, null);
      expect(imageCache.containsKey(testImageProvider), false);

      scrollController.animateTo(
        3000,
        duration: const Duration(seconds: 2),
        curve: Curves.fastLinearToSlowEaseIn,
      );
      await tester.pump();
      final RecordingPhysics physics = findPhysics<RecordingPhysics>(tester);

      expect(physics.velocities.length, 0);
      final ImageStream stream = imageProvider.resolve(ImageConfiguration.empty);
      expect(physics.velocities.length, 1);
      expect(
        const ScrollPhysics().recommendDeferredLoading(
          physics.velocities.first,
          findMetrics(tester),
          find.byType(TestWidget).evaluate().first,
        ),
        true,
      );

      expect(testImageProvider.configuration, null);
      expect(stream.completer, null);

      expect(imageCache.containsKey(testImageProvider), false);
      expect(imageCache.currentSize, 0);

      // as if we had picked a context that scrolled out of the tree.
      context.dispose();

      await tester.pump(const Duration(seconds: 1));
      expect(physics.velocities.length, 1);

      expect(testImageProvider.configuration, null);
      expect(stream.completer, null);

      expect(imageCache.containsKey(testImageProvider), false);
      expect(imageCache.currentSize, 0);

      testImageProvider.complete();

      expect(imageCache.currentSize, 0);
    },
  );

  testWidgets(
    'ScrollAwareImageProvider resolves from ImageCache and does not set completer twice',
    (WidgetTester tester) async {
      final GlobalKey<TestWidgetState> key = GlobalKey<TestWidgetState>();
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: SingleChildScrollView(
            physics: ControllablePhysics(),
            controller: scrollController,
            child: TestWidget(key),
          ),
        ),
      );

      final DisposableBuildContext context = DisposableBuildContext(key.currentState!);
      addTearDown(context.dispose);
      final TestImageProvider testImageProvider = TestImageProvider(cloneImage());
      final ScrollAwareImageProvider<TestImageProvider> imageProvider =
          ScrollAwareImageProvider<TestImageProvider>(
            context: context,
            imageProvider: testImageProvider,
          );

      expect(testImageProvider.configuration, null);
      expect(imageCache.containsKey(testImageProvider), false);

      final ControllablePhysics physics = findPhysics<ControllablePhysics>(tester);
      physics.recommendDeferredLoadingValue = true;

      final ImageStream stream = imageProvider.resolve(ImageConfiguration.empty);

      expect(testImageProvider.configuration, null);
      expect(stream.completer, null);
      expect(imageCache.containsKey(testImageProvider), false);
      expect(imageCache.currentSize, 0);

      // Simulate a case where someone else has managed to complete this stream -
      // so it can land in the cache right before we stop scrolling fast.
      // If we miss the early return, we will fail.
      testImageProvider.complete();

      imageCache.putIfAbsent(
        testImageProvider,
        () => testImageProvider.loadImage(
          testImageProvider,
          PaintingBinding.instance.instantiateImageCodecWithSize,
        ),
      );
      // We've stopped scrolling fast.
      physics.recommendDeferredLoadingValue = false;
      await tester.idle();

      expect(imageCache.containsKey(testImageProvider), true);
      expect(imageCache.currentSize, 1);
      expect(testImageProvider.loadCallCount, 1);
      expect(stream.completer, null);
    },
  );

  testWidgets(
    'ScrollAwareImageProvider does not block LRU updates to image cache',
    experimentalLeakTesting: LeakTesting.settings.withIgnoredAll(),
    (WidgetTester tester) async {
      final int oldSize = imageCache.maximumSize;
      imageCache.maximumSize = 1;

      final GlobalKey<TestWidgetState> key = GlobalKey<TestWidgetState>();
      final ScrollController scrollController = ScrollController();
      addTearDown(scrollController.dispose);
      await tester.pumpWidget(
        Directionality(
          textDirection: TextDirection.ltr,
          child: SingleChildScrollView(
            physics: ControllablePhysics(),
            controller: scrollController,
            child: TestWidget(key),
          ),
        ),
      );

      final DisposableBuildContext context = DisposableBuildContext(key.currentState!);
      addTearDown(context.dispose);
      final TestImageProvider testImageProvider = TestImageProvider(testImage.clone());
      final ScrollAwareImageProvider<TestImageProvider> imageProvider =
          ScrollAwareImageProvider<TestImageProvider>(
            context: context,
            imageProvider: testImageProvider,
          );

      expect(testImageProvider.configuration, null);
      expect(imageCache.containsKey(testImageProvider), false);

      final ControllablePhysics physics = findPhysics<ControllablePhysics>(tester);
      physics.recommendDeferredLoadingValue = true;

      final ImageStream stream = imageProvider.resolve(ImageConfiguration.empty);

      expect(testImageProvider.configuration, null);
      expect(stream.completer, null);
      expect(imageCache.currentSize, 0);

      // Occupy the only slot in the cache with another image.
      final TestImageProvider testImageProvider2 = TestImageProvider(testImage.clone());
      testImageProvider2.complete();
      await precacheImage(testImageProvider2, context.context!);
      expect(imageCache.containsKey(testImageProvider), false);
      expect(imageCache.containsKey(testImageProvider2), true);
      expect(imageCache.currentSize, 1);

      // Complete the original image while we're still scrolling fast.
      testImageProvider.complete();
      stream.setCompleter(
        testImageProvider.loadImage(
          testImageProvider,
          PaintingBinding.instance.instantiateImageCodecWithSize,
        ),
      );

      // Verify that this hasn't changed the cache state yet
      expect(imageCache.containsKey(testImageProvider), false);
      expect(imageCache.containsKey(testImageProvider2), true);
      expect(imageCache.currentSize, 1);
      expect(testImageProvider.loadCallCount, 1);

      await tester.pump();

      // After pumping a frame, the original image should be in the cache because
      // it took the LRU slot.
      expect(imageCache.containsKey(testImageProvider), true);
      expect(imageCache.containsKey(testImageProvider2), false);
      expect(imageCache.currentSize, 1);
      expect(testImageProvider.loadCallCount, 1);

      imageCache.maximumSize = oldSize;
    },
  );
}

class TestWidget extends StatefulWidget {
  const TestWidget(Key? key) : super(key: key);

  @override
  State<TestWidget> createState() => TestWidgetState();
}

class TestWidgetState extends State<TestWidget> {
  @override
  Widget build(BuildContext context) => const SizedBox(height: 50);
}

class RecordingPhysics extends ScrollPhysics {
  RecordingPhysics({super.parent});

  final List<double> velocities = <double>[];

  @override
  RecordingPhysics applyTo(ScrollPhysics? ancestor) {
    return RecordingPhysics(parent: buildParent(ancestor));
  }

  @override
  bool recommendDeferredLoading(double velocity, ScrollMetrics metrics, BuildContext context) {
    velocities.add(velocity);
    return super.recommendDeferredLoading(velocity, metrics, context);
  }
}

// Ignore this so that we can mutate whether we defer loading or not at specific
// times without worrying about actual scrolling mechanics.
// ignore: must_be_immutable
class ControllablePhysics extends ScrollPhysics {
  ControllablePhysics({super.parent});

  bool recommendDeferredLoadingValue = false;

  @override
  ControllablePhysics applyTo(ScrollPhysics? ancestor) {
    return ControllablePhysics(parent: buildParent(ancestor));
  }

  @override
  bool recommendDeferredLoading(double velocity, ScrollMetrics metrics, BuildContext context) {
    return recommendDeferredLoadingValue;
  }
}
