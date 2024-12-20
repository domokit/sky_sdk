// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:fake_async/fake_async.dart';
import 'package:flutter_tools/src/proxied_devices/debounce_data_stream.dart';

import '../../src/common.dart';

void main() {
  group('debounceDataStreams', () {
    late FakeAsync fakeAsync;
    late StreamController<Uint8List> source;
    late Stream<Uint8List> output;
    const Duration debounceDuration = Duration(seconds: 10);
    const Duration smallDuration = Duration(milliseconds: 10);

    void addToSource(int value) {
      source.add(Uint8List.fromList(<int>[value]));
    }

    setUp(() {
      fakeAsync = FakeAsync();
      fakeAsync.run((FakeAsync time) {
        source = StreamController<Uint8List>();
        output = debounceDataStream(source.stream, debounceDuration);
      });
    });

    testWithoutContext('does not listen if returned stream is not listened to', () {
      expect(source.hasListener, false);
      output.listen(dummy);
      expect(source.hasListener, true);
    });

    testWithoutContext('forwards data normally is all data if longer than duration apart', () {
      fakeAsync.run((FakeAsync time) {
        final List<Uint8List> outputItems = <Uint8List>[];
        output.listen(outputItems.add);

        addToSource(1);
        time.elapse(debounceDuration + smallDuration);
        addToSource(2);
        time.elapse(debounceDuration + smallDuration);
        addToSource(3);
        time.elapse(debounceDuration + smallDuration);

        expect(outputItems, <List<int>>[
          <int>[1],
          <int>[2],
          <int>[3],
        ]);
      });
    });

    testWithoutContext('merge data after the first if sent within duration', () {
      fakeAsync.run((FakeAsync time) {
        final List<Uint8List> outputItems = <Uint8List>[];
        output.listen(outputItems.add);

        addToSource(1);
        time.elapse(smallDuration);
        addToSource(2);
        time.elapse(smallDuration);
        addToSource(3);
        time.elapse(debounceDuration + smallDuration);

        expect(outputItems, <List<int>>[
          <int>[1],
          <int>[2, 3],
        ]);
      });
    });

    testWithoutContext(
      'output data in separate chunks if time between them is longer than duration',
      () {
        fakeAsync.run((FakeAsync time) {
          final List<Uint8List> outputItems = <Uint8List>[];
          output.listen(outputItems.add);

          addToSource(1);
          time.elapse(smallDuration);
          addToSource(2);
          time.elapse(smallDuration);
          addToSource(3);
          time.elapse(debounceDuration + smallDuration);
          addToSource(4);
          time.elapse(smallDuration);
          addToSource(5);
          time.elapse(debounceDuration + smallDuration);

          expect(outputItems, <List<int>>[
            <int>[1],
            <int>[2, 3],
            <int>[4, 5],
          ]);
        });
      },
    );

    testWithoutContext('sends the last chunk after debounce duration', () {
      fakeAsync.run((FakeAsync time) {
        final List<Uint8List> outputItems = <Uint8List>[];
        output.listen(outputItems.add);

        addToSource(1);
        time.flushMicrotasks();
        expect(outputItems, <List<int>>[
          <int>[1],
        ]);

        time.elapse(smallDuration);
        addToSource(2);
        time.elapse(smallDuration);
        addToSource(3);
        expect(outputItems, <List<int>>[
          <int>[1],
        ]);

        time.elapse(debounceDuration + smallDuration);
        expect(outputItems, <List<int>>[
          <int>[1],
          <int>[2, 3],
        ]);
      });
    });

    testWithoutContext('close if source stream is closed', () {
      fakeAsync.run((FakeAsync time) {
        bool isDone = false;
        output.listen(dummy, onDone: () => isDone = true);
        expect(isDone, false);
        source.close();
        time.flushMicrotasks();
        expect(isDone, true);
      });
    });

    testWithoutContext('delay close until after last chunk is sent', () {
      fakeAsync.run((FakeAsync time) {
        final List<Uint8List> outputItems = <Uint8List>[];
        bool isDone = false;
        output.listen(outputItems.add, onDone: () => isDone = true);

        addToSource(1);
        time.flushMicrotasks();
        expect(outputItems, <List<int>>[
          <int>[1],
        ]);

        addToSource(2);
        source.close();
        time.elapse(smallDuration);
        expect(isDone, false);
        expect(outputItems, <List<int>>[
          <int>[1],
        ]);

        time.elapse(debounceDuration + smallDuration);
        expect(outputItems, <List<int>>[
          <int>[1],
          <int>[2],
        ]);
        expect(isDone, true);
      });
    });

    testWithoutContext('close if returned stream is closed', () {
      fakeAsync.run((FakeAsync time) {
        bool isCancelled = false;
        source.onCancel = () => isCancelled = true;
        final StreamSubscription<Uint8List> subscription = output.listen(dummy);
        expect(isCancelled, false);
        subscription.cancel();
        expect(isCancelled, true);
      });
    });
  });
}

Uint8List dummy(Uint8List data) => data;
