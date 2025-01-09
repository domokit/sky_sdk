// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:ui';

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/scheduler.dart' show SchedulerBinding, timeDilation;
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import '../image_data.dart';
import 'fake_codec.dart';
import 'mocks_for_image_cache.dart';

class FakeFrameInfo implements FrameInfo {
  const FakeFrameInfo(this._duration, this._image);

  final Duration _duration;
  final Image _image;

  @override
  Duration get duration => _duration;

  @override
  Image get image => _image;

  FakeFrameInfo clone() {
    return FakeFrameInfo(_duration, _image.clone());
  }
}

class MockCodec implements Codec {
  @override
  late int frameCount;

  @override
  late int repetitionCount;

  int numFramesAsked = 0;

  Completer<FrameInfo> _nextFrameCompleter = Completer<FrameInfo>();

  @override
  Future<FrameInfo> getNextFrame() {
    numFramesAsked += 1;
    return _nextFrameCompleter.future;
  }

  void completeNextFrame(FrameInfo frameInfo) {
    _nextFrameCompleter.complete(frameInfo);
    _nextFrameCompleter = Completer<FrameInfo>();
  }

  void failNextFrame(String err) {
    _nextFrameCompleter.completeError(err);
  }

  @override
  void dispose() {}
}

class FakeEventReportingImageStreamCompleter extends ImageStreamCompleter {
  FakeEventReportingImageStreamCompleter({Stream<ImageChunkEvent>? chunkEvents}) {
    chunkEvents?.listen(reportImageChunkEvent);
  }
}

void main() {
  late Image image20x10;
  late Image image200x100;
  setUp(() async {
    image20x10 = await createTestImage(width: 20, height: 10);
    image200x100 = await createTestImage(width: 200, height: 100);
  });

  testWidgets('Codec future fails', (WidgetTester tester) async {
    final Completer<Codec> completer = Completer<Codec>();
    MultiFrameImageStreamCompleter(codec: completer.future, scale: 1.0);
    completer.completeError('failure message');
    await tester.idle();
    expect(tester.takeException(), 'failure message');
  });

  testWidgets('Decoding starts when a listener is added after codec is ready', (
    WidgetTester tester,
  ) async {
    final Completer<Codec> completer = Completer<Codec>();
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 1;
    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: completer.future,
      scale: 1.0,
    );

    completer.complete(mockCodec);
    await tester.idle();
    expect(mockCodec.numFramesAsked, 0);

    void listener(ImageInfo image, bool synchronousCall) {
      addTearDown(image.dispose);
    }

    imageStream.addListener(ImageStreamListener(listener));
    await tester.idle();
    expect(mockCodec.numFramesAsked, 1);
  });

  testWidgets('Decoding starts when a codec is ready after a listener is added', (
    WidgetTester tester,
  ) async {
    final Completer<Codec> completer = Completer<Codec>();
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 1;
    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: completer.future,
      scale: 1.0,
    );

    void listener(ImageInfo image, bool synchronousCall) {
      addTearDown(image.dispose);
    }

    imageStream.addListener(ImageStreamListener(listener));
    await tester.idle();
    expect(mockCodec.numFramesAsked, 0);

    completer.complete(mockCodec);
    await tester.idle();
    expect(mockCodec.numFramesAsked, 1);
  });

  testWidgets('Decoding does not crash when disposed', (WidgetTester tester) async {
    final Completer<Codec> completer = Completer<Codec>();
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 1;
    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: completer.future,
      scale: 1.0,
    );

    completer.complete(mockCodec);
    await tester.idle();
    expect(mockCodec.numFramesAsked, 0);

    void listener(ImageInfo image, bool synchronousCall) {
      addTearDown(image.dispose);
    }

    final ImageStreamListener streamListener = ImageStreamListener(listener);
    imageStream.addListener(streamListener);
    await tester.idle();
    expect(mockCodec.numFramesAsked, 1);

    final FrameInfo frame = FakeFrameInfo(const Duration(milliseconds: 200), image20x10);
    mockCodec.completeNextFrame(frame);
    imageStream.removeListener(streamListener);
    await tester.idle();
  });

  testWidgets('Chunk events of base ImageStreamCompleter are delivered', (
    WidgetTester tester,
  ) async {
    final List<ImageChunkEvent> chunkEvents = <ImageChunkEvent>[];
    final StreamController<ImageChunkEvent> streamController = StreamController<ImageChunkEvent>();
    final ImageStreamCompleter imageStream = FakeEventReportingImageStreamCompleter(
      chunkEvents: streamController.stream,
    );

    imageStream.addListener(
      ImageStreamListener(
        (ImageInfo image, bool synchronousCall) {},
        onChunk: (ImageChunkEvent event) {
          chunkEvents.add(event);
        },
      ),
    );
    streamController.add(const ImageChunkEvent(cumulativeBytesLoaded: 1, expectedTotalBytes: 3));
    streamController.add(const ImageChunkEvent(cumulativeBytesLoaded: 2, expectedTotalBytes: 3));
    await tester.idle();

    expect(chunkEvents.length, 2);
    expect(chunkEvents[0].cumulativeBytesLoaded, 1);
    expect(chunkEvents[0].expectedTotalBytes, 3);
    expect(chunkEvents[1].cumulativeBytesLoaded, 2);
    expect(chunkEvents[1].expectedTotalBytes, 3);
  });

  testWidgets(
    'Chunk events of base ImageStreamCompleter are not buffered before listener registration',
    (WidgetTester tester) async {
      final List<ImageChunkEvent> chunkEvents = <ImageChunkEvent>[];
      final StreamController<ImageChunkEvent> streamController =
          StreamController<ImageChunkEvent>();
      final ImageStreamCompleter imageStream = FakeEventReportingImageStreamCompleter(
        chunkEvents: streamController.stream,
      );

      streamController.add(const ImageChunkEvent(cumulativeBytesLoaded: 1, expectedTotalBytes: 3));
      await tester.idle();
      imageStream.addListener(
        ImageStreamListener(
          (ImageInfo image, bool synchronousCall) {},
          onChunk: (ImageChunkEvent event) {
            chunkEvents.add(event);
          },
        ),
      );
      streamController.add(const ImageChunkEvent(cumulativeBytesLoaded: 2, expectedTotalBytes: 3));
      await tester.idle();

      expect(chunkEvents.length, 1);
      expect(chunkEvents[0].cumulativeBytesLoaded, 2);
      expect(chunkEvents[0].expectedTotalBytes, 3);
    },
  );

  testWidgets('Chunk events of MultiFrameImageStreamCompleter are delivered', (
    WidgetTester tester,
  ) async {
    final List<ImageChunkEvent> chunkEvents = <ImageChunkEvent>[];
    final Completer<Codec> completer = Completer<Codec>();
    final StreamController<ImageChunkEvent> streamController = StreamController<ImageChunkEvent>();
    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: completer.future,
      chunkEvents: streamController.stream,
      scale: 1.0,
    );

    imageStream.addListener(
      ImageStreamListener(
        (ImageInfo image, bool synchronousCall) {},
        onChunk: (ImageChunkEvent event) {
          chunkEvents.add(event);
        },
      ),
    );
    streamController.add(const ImageChunkEvent(cumulativeBytesLoaded: 1, expectedTotalBytes: 3));
    streamController.add(const ImageChunkEvent(cumulativeBytesLoaded: 2, expectedTotalBytes: 3));
    await tester.idle();

    expect(chunkEvents.length, 2);
    expect(chunkEvents[0].cumulativeBytesLoaded, 1);
    expect(chunkEvents[0].expectedTotalBytes, 3);
    expect(chunkEvents[1].cumulativeBytesLoaded, 2);
    expect(chunkEvents[1].expectedTotalBytes, 3);
  });

  testWidgets(
    'Chunk events of MultiFrameImageStreamCompleter are not buffered before listener registration',
    (WidgetTester tester) async {
      final List<ImageChunkEvent> chunkEvents = <ImageChunkEvent>[];
      final Completer<Codec> completer = Completer<Codec>();
      final StreamController<ImageChunkEvent> streamController =
          StreamController<ImageChunkEvent>();
      final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
        codec: completer.future,
        chunkEvents: streamController.stream,
        scale: 1.0,
      );

      streamController.add(const ImageChunkEvent(cumulativeBytesLoaded: 1, expectedTotalBytes: 3));
      await tester.idle();
      imageStream.addListener(
        ImageStreamListener(
          (ImageInfo image, bool synchronousCall) {},
          onChunk: (ImageChunkEvent event) {
            chunkEvents.add(event);
          },
        ),
      );
      streamController.add(const ImageChunkEvent(cumulativeBytesLoaded: 2, expectedTotalBytes: 3));
      await tester.idle();

      expect(chunkEvents.length, 1);
      expect(chunkEvents[0].cumulativeBytesLoaded, 2);
      expect(chunkEvents[0].expectedTotalBytes, 3);
    },
  );

  testWidgets('Chunk errors are reported', (WidgetTester tester) async {
    final List<ImageChunkEvent> chunkEvents = <ImageChunkEvent>[];
    final Completer<Codec> completer = Completer<Codec>();
    final StreamController<ImageChunkEvent> streamController = StreamController<ImageChunkEvent>();
    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: completer.future,
      chunkEvents: streamController.stream,
      scale: 1.0,
    );

    imageStream.addListener(
      ImageStreamListener(
        (ImageInfo image, bool synchronousCall) {},
        onChunk: (ImageChunkEvent event) {
          chunkEvents.add(event);
        },
      ),
    );
    streamController.addError(Error());
    streamController.add(const ImageChunkEvent(cumulativeBytesLoaded: 2, expectedTotalBytes: 3));
    await tester.idle();

    expect(tester.takeException(), isNotNull);
    expect(chunkEvents.length, 1);
    expect(chunkEvents[0].cumulativeBytesLoaded, 2);
    expect(chunkEvents[0].expectedTotalBytes, 3);
  });

  testWidgets('getNextFrame future fails', (WidgetTester tester) async {
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 1;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    void listener(ImageInfo image, bool synchronousCall) {
      addTearDown(image.dispose);
    }

    imageStream.addListener(ImageStreamListener(listener));
    codecCompleter.complete(mockCodec);
    // MultiFrameImageStreamCompleter only sets an error handler for the next
    // frame future after the codec future has completed.
    // Idling here lets the MultiFrameImageStreamCompleter advance and set the
    // error handler for the nextFrame future.
    await tester.idle();

    mockCodec.failNextFrame('frame completion error');
    await tester.idle();

    expect(tester.takeException(), 'frame completion error');
  });

  testWidgets('ImageStream emits frame (static image)', (WidgetTester tester) async {
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 1;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    final List<ImageInfo> emittedImages = <ImageInfo>[];

    final ImageStreamListener listener = ImageStreamListener((
      ImageInfo image,
      bool synchronousCall,
    ) {
      emittedImages.add(image);
      addTearDown(image.dispose);
    });
    imageStream.addListener(listener);

    codecCompleter.complete(mockCodec);
    await tester.idle();

    final FrameInfo frame = FakeFrameInfo(const Duration(milliseconds: 200), image20x10);
    mockCodec.completeNextFrame(frame);
    await tester.idle();

    expect(emittedImages.every((ImageInfo info) => info.image.isCloneOf(frame.image)), true);

    imageStream.removeListener(listener);
    imageCache.clear();
  });

  testWidgets('ImageStream emits frames (animated images)', (WidgetTester tester) async {
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 2;
    mockCodec.repetitionCount = -1;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    final List<ImageInfo> emittedImages = <ImageInfo>[];
    final ImageStreamListener listener = ImageStreamListener((
      ImageInfo image,
      bool synchronousCall,
    ) {
      emittedImages.add(image);
      addTearDown(image.dispose);
    });
    imageStream.addListener(listener);

    codecCompleter.complete(mockCodec);
    await tester.idle();

    final FrameInfo frame1 = FakeFrameInfo(const Duration(milliseconds: 200), image20x10);
    mockCodec.completeNextFrame(frame1);
    await tester.idle();
    // We are waiting for the next animation tick, so at this point no frames
    // should have been emitted.
    expect(emittedImages.length, 0);

    await tester.pump();
    expect(emittedImages.single.image.isCloneOf(frame1.image), true);

    final FrameInfo frame2 = FakeFrameInfo(const Duration(milliseconds: 400), image200x100);
    mockCodec.completeNextFrame(frame2);

    await tester.pump(const Duration(milliseconds: 100));
    // The duration for the current frame was 200ms, so we don't emit the next
    // frame yet even though it is ready.
    expect(emittedImages.length, 1);

    await tester.pump(const Duration(milliseconds: 100));
    expect(emittedImages[0].image.isCloneOf(frame1.image), true);
    expect(emittedImages[1].image.isCloneOf(frame2.image), true);

    // Let the pending timer for the next frame to complete so we can cleanly
    // quit the test without pending timers.
    await tester.pump(const Duration(milliseconds: 400));

    imageStream.removeListener(listener);
    imageCache.clear();
  });

  testWidgets('animation wraps back', (WidgetTester tester) async {
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 2;
    mockCodec.repetitionCount = -1;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    final List<ImageInfo> emittedImages = <ImageInfo>[];
    final ImageStreamListener listener = ImageStreamListener((
      ImageInfo image,
      bool synchronousCall,
    ) {
      emittedImages.add(image);
      addTearDown(image.dispose);
    });
    imageStream.addListener(listener);

    codecCompleter.complete(mockCodec);
    await tester.idle();

    final FakeFrameInfo frame1 = FakeFrameInfo(const Duration(milliseconds: 200), image20x10);
    final FakeFrameInfo frame2 = FakeFrameInfo(const Duration(milliseconds: 400), image200x100);

    mockCodec.completeNextFrame(frame1.clone());
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(); // first animation frame shows on first app frame.
    mockCodec.completeNextFrame(frame2.clone());
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(const Duration(milliseconds: 200)); // emit 2nd frame.
    mockCodec.completeNextFrame(frame1.clone());
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(const Duration(milliseconds: 400)); // emit 3rd frame

    expect(emittedImages[0].image.isCloneOf(frame1.image), true);
    expect(emittedImages[1].image.isCloneOf(frame2.image), true);
    expect(emittedImages[2].image.isCloneOf(frame1.image), true);

    // Let the pending timer for the next frame to complete so we can cleanly
    // quit the test without pending timers.
    await tester.pump(const Duration(milliseconds: 200));

    imageStream.removeListener(listener);
    imageCache.clear();
  });

  testWidgets("animation doesn't repeat more than specified", (WidgetTester tester) async {
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 2;
    mockCodec.repetitionCount = 0;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    final List<ImageInfo> emittedImages = <ImageInfo>[];
    final ImageStreamListener listener = ImageStreamListener((
      ImageInfo image,
      bool synchronousCall,
    ) {
      emittedImages.add(image);
      addTearDown(image.dispose);
    });
    imageStream.addListener(listener);

    codecCompleter.complete(mockCodec);
    await tester.idle();

    final FrameInfo frame1 = FakeFrameInfo(const Duration(milliseconds: 200), image20x10);
    final FrameInfo frame2 = FakeFrameInfo(const Duration(milliseconds: 400), image200x100);

    mockCodec.completeNextFrame(frame1);
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(); // first animation frame shows on first app frame.
    mockCodec.completeNextFrame(frame2);
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(const Duration(milliseconds: 200)); // emit 2nd frame.
    mockCodec.completeNextFrame(frame1);
    // allow another frame to complete (but we shouldn't be asking for it as
    // this animation should not repeat.
    await tester.idle();
    await tester.pump(const Duration(milliseconds: 400));

    expect(emittedImages[0].image.isCloneOf(frame1.image), true);
    expect(emittedImages[1].image.isCloneOf(frame2.image), true);

    imageStream.removeListener(listener);
    imageCache.clear();
  });

  testWidgets('frames are only decoded when there are listeners', (WidgetTester tester) async {
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 2;
    mockCodec.repetitionCount = -1;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    final ImageStreamListener listener = ImageStreamListener((
      ImageInfo image,
      bool synchronousCall,
    ) {
      addTearDown(image.dispose);
    });
    imageStream.addListener(listener);
    final ImageStreamCompleterHandle handle = imageStream.keepAlive();

    codecCompleter.complete(mockCodec);
    await tester.idle();

    final FrameInfo frame1 = FakeFrameInfo(const Duration(milliseconds: 200), image20x10);
    final FrameInfo frame2 = FakeFrameInfo(const Duration(milliseconds: 400), image200x100);

    mockCodec.completeNextFrame(frame1);
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(); // first animation frame shows on first app frame.
    mockCodec.completeNextFrame(frame2);
    imageStream.removeListener(listener);
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(const Duration(milliseconds: 400)); // emit 2nd frame.

    // Decoding of the 3rd frame should not start as there are no registered
    // listeners to the stream
    expect(mockCodec.numFramesAsked, 2);

    imageStream.addListener(listener);
    await tester.idle(); // let nextFrameFuture complete
    expect(mockCodec.numFramesAsked, 3);

    handle.dispose();
    imageStream.removeListener(listener);
    imageCache.clear();
  });

  testWidgets('multiple stream listeners', (WidgetTester tester) async {
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 2;
    mockCodec.repetitionCount = -1;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    final List<ImageInfo> emittedImages1 = <ImageInfo>[];
    final ImageStreamListener listener1 = ImageStreamListener((
      ImageInfo image,
      bool synchronousCall,
    ) {
      emittedImages1.add(image);
      addTearDown(image.dispose);
    });
    final List<ImageInfo> emittedImages2 = <ImageInfo>[];
    final ImageStreamListener listener2 = ImageStreamListener((
      ImageInfo image,
      bool synchronousCall,
    ) {
      emittedImages2.add(image);
      addTearDown(image.dispose);
    });
    imageStream.addListener(listener1);
    imageStream.addListener(listener2);

    codecCompleter.complete(mockCodec);
    await tester.idle();

    final FrameInfo frame1 = FakeFrameInfo(const Duration(milliseconds: 200), image20x10);
    final FrameInfo frame2 = FakeFrameInfo(const Duration(milliseconds: 400), image200x100);

    mockCodec.completeNextFrame(frame1);
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(); // first animation frame shows on first app frame.

    expect(emittedImages1.single.image.isCloneOf(frame1.image), true);
    expect(emittedImages2.single.image.isCloneOf(frame1.image), true);

    mockCodec.completeNextFrame(frame2);
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(); // next app frame will schedule a timer.
    imageStream.removeListener(listener1);

    await tester.pump(const Duration(milliseconds: 400)); // emit 2nd frame.
    expect(emittedImages1.single.image.isCloneOf(frame1.image), true);
    expect(emittedImages2[0].image.isCloneOf(frame1.image), true);
    expect(emittedImages2[1].image.isCloneOf(frame2.image), true);

    imageStream.removeListener(listener2);
  });

  testWidgets('timer is canceled when listeners are removed', (WidgetTester tester) async {
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 2;
    mockCodec.repetitionCount = -1;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    void listener(ImageInfo image, bool synchronousCall) {
      addTearDown(image.dispose);
    }

    imageStream.addListener(ImageStreamListener(listener));

    codecCompleter.complete(mockCodec);
    await tester.idle();

    final FrameInfo frame1 = FakeFrameInfo(const Duration(milliseconds: 200), image20x10);
    final FrameInfo frame2 = FakeFrameInfo(const Duration(milliseconds: 400), image200x100);

    mockCodec.completeNextFrame(frame1);
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(); // first animation frame shows on first app frame.

    mockCodec.completeNextFrame(frame2);
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump();

    imageStream.removeListener(ImageStreamListener(listener));
    // The test framework will fail this if there are pending timers at this
    // point.
  });

  testWidgets('timeDilation affects animation frame timers', (WidgetTester tester) async {
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 2;
    mockCodec.repetitionCount = -1;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    final ImageStreamListener listener = ImageStreamListener((
      ImageInfo image,
      bool synchronousCall,
    ) {
      addTearDown(image.dispose);
    });
    imageStream.addListener(listener);

    codecCompleter.complete(mockCodec);
    await tester.idle();

    final FrameInfo frame1 = FakeFrameInfo(const Duration(milliseconds: 200), image20x10);
    final FrameInfo frame2 = FakeFrameInfo(const Duration(milliseconds: 400), image200x100);

    mockCodec.completeNextFrame(frame1);
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(); // first animation frame shows on first app frame.

    timeDilation = 2.0;
    mockCodec.completeNextFrame(frame2);
    await tester.idle(); // let nextFrameFuture complete
    await tester.pump(); // schedule next app frame
    await tester.pump(const Duration(milliseconds: 200)); // emit 2nd frame.
    // Decoding of the 3rd frame should not start after 200 ms, as time is
    // dilated by a factor of 2.
    expect(mockCodec.numFramesAsked, 2);
    await tester.pump(const Duration(milliseconds: 200)); // emit 2nd frame.
    expect(mockCodec.numFramesAsked, 3);
    timeDilation = 1.0; // restore time dilation, or it will affect other tests

    imageStream.removeListener(listener);
  });

  testWidgets('error handlers can intercept errors', (WidgetTester tester) async {
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 1;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter streamUnderTest = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    dynamic capturedException;
    void errorListener(dynamic exception, StackTrace? stackTrace) {
      capturedException = exception;
    }

    streamUnderTest.addListener(
      ImageStreamListener((ImageInfo image, bool synchronousCall) {}, onError: errorListener),
    );

    codecCompleter.complete(mockCodec);
    // MultiFrameImageStreamCompleter only sets an error handler for the next
    // frame future after the codec future has completed.
    // Idling here lets the MultiFrameImageStreamCompleter advance and set the
    // error handler for the nextFrame future.
    await tester.idle();

    mockCodec.failNextFrame('frame completion error');
    await tester.idle();

    // No exception is passed up.
    expect(tester.takeException(), isNull);
    expect(capturedException, 'frame completion error');
  });

  testWidgets(
    'remove and add listener ',
    experimentalLeakTesting:
        LeakTesting.settings
            .withIgnoredAll(), // leaking by design because imageStream does not have a listener
    (WidgetTester tester) async {
      final MockCodec mockCodec = MockCodec();
      mockCodec.frameCount = 3;
      mockCodec.repetitionCount = 0;
      final Completer<Codec> codecCompleter = Completer<Codec>();

      final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
        codec: codecCompleter.future,
        scale: 1.0,
      );

      void listener(ImageInfo image, bool synchronousCall) {
        addTearDown(image.dispose);
      }

      imageStream.addListener(ImageStreamListener(listener));

      codecCompleter.complete(mockCodec);

      await tester.idle(); // let nextFrameFuture complete

      imageStream.addListener(ImageStreamListener(listener));
      imageStream.removeListener(ImageStreamListener(listener));

      final FrameInfo frame1 = FakeFrameInfo(const Duration(milliseconds: 200), image20x10);

      mockCodec.completeNextFrame(frame1);
      await tester.idle(); // let nextFrameFuture complete
      await tester.pump(); // first animation frame shows on first app frame.

      await tester.pump(const Duration(milliseconds: 200)); // emit 2nd frame.
    },
  );

  testWidgets('ImageStreamListener hashCode and equals', (WidgetTester tester) async {
    void handleImage(ImageInfo image, bool synchronousCall) {}
    void handleImageDifferently(ImageInfo image, bool synchronousCall) {}
    void handleError(dynamic error, StackTrace? stackTrace) {}
    void handleChunk(ImageChunkEvent event) {}

    void compare({
      required ImageListener onImage1,
      required ImageListener onImage2,
      ImageChunkListener? onChunk1,
      ImageChunkListener? onChunk2,
      ImageErrorListener? onError1,
      ImageErrorListener? onError2,
      bool areEqual = true,
    }) {
      final ImageStreamListener l1 = ImageStreamListener(
        onImage1,
        onChunk: onChunk1,
        onError: onError1,
      );
      final ImageStreamListener l2 = ImageStreamListener(
        onImage2,
        onChunk: onChunk2,
        onError: onError2,
      );
      Matcher comparison(dynamic expected) => areEqual ? equals(expected) : isNot(equals(expected));
      expect(l1, comparison(l2));
      expect(l1.hashCode, comparison(l2.hashCode));
    }

    compare(onImage1: handleImage, onImage2: handleImage);
    compare(onImage1: handleImage, onImage2: handleImageDifferently, areEqual: false);
    compare(
      onImage1: handleImage,
      onChunk1: handleChunk,
      onImage2: handleImage,
      onChunk2: handleChunk,
    );
    compare(
      onImage1: handleImage,
      onChunk1: handleChunk,
      onError1: handleError,
      onImage2: handleImage,
      onChunk2: handleChunk,
      onError2: handleError,
    );
    compare(onImage1: handleImage, onChunk1: handleChunk, onImage2: handleImage, areEqual: false);
    compare(
      onImage1: handleImage,
      onChunk1: handleChunk,
      onError1: handleError,
      onImage2: handleImage,
      areEqual: false,
    );
    compare(
      onImage1: handleImage,
      onChunk1: handleChunk,
      onError1: handleError,
      onImage2: handleImage,
      onChunk2: handleChunk,
      areEqual: false,
    );
    compare(
      onImage1: handleImage,
      onChunk1: handleChunk,
      onError1: handleError,
      onImage2: handleImage,
      onError2: handleError,
      areEqual: false,
    );
  });

  testWidgets('Keep alive handles do not drive frames or prevent last listener callbacks', (
    WidgetTester tester,
  ) async {
    final Image image10x10 = (await tester.runAsync(() => createTestImage(width: 10, height: 10)))!;
    final MockCodec mockCodec = MockCodec();
    mockCodec.frameCount = 2;
    mockCodec.repetitionCount = -1;
    final Completer<Codec> codecCompleter = Completer<Codec>();

    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );

    int onImageCount = 0;
    void activeListener(ImageInfo image, bool synchronousCall) {
      onImageCount += 1;
      addTearDown(image.dispose);
    }

    bool lastListenerDropped = false;
    imageStream.addOnLastListenerRemovedCallback(() {
      lastListenerDropped = true;
    });

    expect(lastListenerDropped, false);
    final ImageStreamCompleterHandle handle = imageStream.keepAlive();
    expect(lastListenerDropped, false);
    SchedulerBinding.instance.debugAssertNoTransientCallbacks('Only passive listeners');

    codecCompleter.complete(mockCodec);
    await tester.idle();

    expect(onImageCount, 0);

    final FakeFrameInfo frame1 = FakeFrameInfo(Duration.zero, image20x10);
    mockCodec.completeNextFrame(frame1);
    await tester.idle();
    SchedulerBinding.instance.debugAssertNoTransientCallbacks('Only passive listeners');
    await tester.pump();
    expect(onImageCount, 0);

    imageStream.addListener(ImageStreamListener(activeListener));

    final FakeFrameInfo frame2 = FakeFrameInfo(Duration.zero, image10x10);
    mockCodec.completeNextFrame(frame2);
    await tester.idle();
    expect(SchedulerBinding.instance.transientCallbackCount, 1);
    await tester.pump();

    expect(onImageCount, 1);

    imageStream.removeListener(ImageStreamListener(activeListener));
    expect(lastListenerDropped, true);

    mockCodec.completeNextFrame(frame1);
    await tester.idle();
    expect(SchedulerBinding.instance.transientCallbackCount, 1);
    await tester.pump();

    expect(onImageCount, 1);

    SchedulerBinding.instance.debugAssertNoTransientCallbacks('Only passive listeners');

    mockCodec.completeNextFrame(frame2);
    await tester.idle();
    SchedulerBinding.instance.debugAssertNoTransientCallbacks('Only passive listeners');
    await tester.pump();

    expect(onImageCount, 1);

    handle.dispose();
  });

  test('MultiFrameImageStreamCompleter - one frame image should only be decoded once', () async {
    final FakeCodec oneFrameCodec = await FakeCodec.fromData(Uint8List.fromList(kTransparentImage));
    final Completer<Codec> codecCompleter = Completer<Codec>();
    final Completer<void> decodeCompleter = Completer<void>();
    final ImageStreamCompleter imageStream = MultiFrameImageStreamCompleter(
      codec: codecCompleter.future,
      scale: 1.0,
    );
    final ImageStreamListener imageListener = ImageStreamListener((ImageInfo info, bool syncCall) {
      decodeCompleter.complete();
    });

    imageStream.keepAlive(); // do not dispose
    imageStream.addListener(imageListener);
    codecCompleter.complete(oneFrameCodec);
    await decodeCompleter.future;

    imageStream.removeListener(imageListener);
    expect(oneFrameCodec.numFramesAsked, 1);

    // Adding a new listener for decoded imageSteam, the one frame image should
    // not be decoded again.
    imageStream.addListener(ImageStreamListener((ImageInfo info, bool syncCall) {}));
    expect(oneFrameCodec.numFramesAsked, 1);
  }); // https://github.com/flutter/flutter/issues/82532

  test('Multi-frame complete unsubscribes to chunk events when disposed', () async {
    final FakeCodec codec = await FakeCodec.fromData(Uint8List.fromList(kTransparentImage));
    final StreamController<ImageChunkEvent> chunkStream = StreamController<ImageChunkEvent>();

    final MultiFrameImageStreamCompleter completer = MultiFrameImageStreamCompleter(
      codec: Future<Codec>.value(codec),
      scale: 1.0,
      chunkEvents: chunkStream.stream,
    );

    expect(chunkStream.hasListener, true);

    chunkStream.add(const ImageChunkEvent(cumulativeBytesLoaded: 1, expectedTotalBytes: 3));

    final ImageStreamListener listener = ImageStreamListener((ImageInfo info, bool syncCall) {});
    // Cause the completer to dispose.
    completer.addListener(listener);
    completer.removeListener(listener);

    expect(chunkStream.hasListener, false);

    // The above expectation should cover this, but the point of this test is to
    // make sure the completer does not assert that it's disposed and still
    // receiving chunk events. Streams from the network can keep sending data
    // even after evicting an image from the cache, for example.
    chunkStream.add(const ImageChunkEvent(cumulativeBytesLoaded: 2, expectedTotalBytes: 3));
  });

  test('ImageStream, setCompleter before addListener - synchronousCall should be true', () async {
    final Image image = await createTestImage(width: 100, height: 100);
    final OneFrameImageStreamCompleter imageStreamCompleter = OneFrameImageStreamCompleter(
      SynchronousFuture<ImageInfo>(TestImageInfo(1, image: image)),
    );

    final ImageStream imageStream = ImageStream();
    imageStream.setCompleter(imageStreamCompleter);

    bool? synchronouslyCalled;
    imageStream.addListener(
      ImageStreamListener((ImageInfo image, bool synchronousCall) {
        synchronouslyCalled = synchronousCall;
      }),
    );

    expect(synchronouslyCalled, true);
  });

  test('ImageStream, setCompleter after addListener - synchronousCall should be false', () async {
    final Image image = await createTestImage(width: 100, height: 100);
    final OneFrameImageStreamCompleter imageStreamCompleter = OneFrameImageStreamCompleter(
      SynchronousFuture<ImageInfo>(TestImageInfo(1, image: image)),
    );

    final ImageStream imageStream = ImageStream();

    bool? synchronouslyCalled;
    imageStream.addListener(
      ImageStreamListener((ImageInfo image, bool synchronousCall) {
        synchronouslyCalled = synchronousCall;
      }),
    );

    imageStream.setCompleter(imageStreamCompleter);

    expect(synchronouslyCalled, false);
  });

  test('ImageStreamCompleterHandle dispatches memory events', () async {
    await expectLater(
      await memoryEvents(() {
        final StreamController<ImageChunkEvent> streamController =
            StreamController<ImageChunkEvent>();
        addTearDown(streamController.close);
        final ImageStreamCompleterHandle imageStreamCompleterHandle =
            FakeEventReportingImageStreamCompleter(
              chunkEvents: streamController.stream,
            ).keepAlive();
        imageStreamCompleterHandle.dispose();
      }, ImageStreamCompleterHandle),
      areCreateAndDispose,
    );
  });

  testWidgets('ImageInfo dispatches memory events', (WidgetTester tester) async {
    await expectLater(
      await memoryEvents(() async {
        final ImageInfo info = ImageInfo(image: image20x10);
        info.dispose();
      }, ImageInfo),
      areCreateAndDispose,
    );
  });
}
