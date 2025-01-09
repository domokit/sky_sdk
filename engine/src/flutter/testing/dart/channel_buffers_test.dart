// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// KEEP THIS SYNCHRONIZED WITH ../../lib/web_ui/test/channel_buffers_test.dart

import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:test/test.dart';

ByteData _makeByteData(String str) {
  final Uint8List list = utf8.encode(str);
  final ByteBuffer buffer = list.buffer;
  return ByteData.view(buffer);
}

void _resize(ui.ChannelBuffers buffers, String name, int newSize) {
  buffers.handleMessage(_makeByteData('resize\r$name\r$newSize'));
}

void main() {
  test('push drain', () async {
    const String channel = 'foo';
    final ByteData data = _makeByteData('bar');
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    bool called = false;
    void callback(ByteData? responseData) {
      called = true;
    }

    buffers.push(channel, data, callback);
    await buffers.drain(channel, (
      ByteData? drainedData,
      ui.PlatformMessageResponseCallback drainedCallback,
    ) async {
      expect(drainedData, equals(data));
      expect(called, isFalse);
      drainedCallback(drainedData);
      expect(called, isTrue);
    });
  });

  test('drain is sync', () async {
    const String channel = 'foo';
    final ByteData data = _makeByteData('message');
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    void callback(ByteData? responseData) {}
    buffers.push(channel, data, callback);
    final List<String> log = <String>[];
    final Completer<void> completer = Completer<void>();
    scheduleMicrotask(() {
      log.add('before drain, microtask');
    });
    log.add('before drain');

    buffers.drain(channel, (
      ByteData? drainedData,
      ui.PlatformMessageResponseCallback drainedCallback,
    ) async {
      log.add('callback');
      completer.complete();
    });
    log.add('after drain, before await');
    await completer.future;
    log.add('after await');
    expect(log, <String>[
      'before drain',
      'callback',
      'after drain, before await',
      'before drain, microtask',
      'after await',
    ]);
  });

  test('push drain zero', () async {
    const String channel = 'foo';
    final ByteData data = _makeByteData('bar');
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    void callback(ByteData? responseData) {}
    _resize(buffers, channel, 0);
    buffers.push(channel, data, callback);
    bool didCall = false;
    await buffers.drain(channel, (
      ByteData? drainedData,
      ui.PlatformMessageResponseCallback drainedCallback,
    ) async {
      didCall = true;
    });
    expect(didCall, equals(false));
  });

  test('drain when empty', () async {
    const String channel = 'foo';
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    bool didCall = false;
    await buffers.drain(channel, (
      ByteData? drainedData,
      ui.PlatformMessageResponseCallback drainedCallback,
    ) async {
      didCall = true;
    });
    expect(didCall, equals(false));
  });

  test('overflow', () async {
    const String channel = 'foo';
    final ByteData one = _makeByteData('one');
    final ByteData two = _makeByteData('two');
    final ByteData three = _makeByteData('three');
    final ByteData four = _makeByteData('four');
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    void callback(ByteData? responseData) {}
    _resize(buffers, channel, 3);
    buffers.push(channel, one, callback);
    buffers.push(channel, two, callback);
    buffers.push(channel, three, callback);
    buffers.push(channel, four, callback);
    int counter = 0;
    await buffers.drain(channel, (
      ByteData? drainedData,
      ui.PlatformMessageResponseCallback drainedCallback,
    ) async {
      switch (counter) {
        case 0:
          expect(drainedData, equals(two));
        case 1:
          expect(drainedData, equals(three));
        case 2:
          expect(drainedData, equals(four));
      }
      counter += 1;
    });
    expect(counter, equals(3));
  });

  test('resize drop', () async {
    const String channel = 'foo';
    final ByteData one = _makeByteData('one');
    final ByteData two = _makeByteData('two');
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    _resize(buffers, channel, 100);
    void callback(ByteData? responseData) {}
    buffers.push(channel, one, callback);
    buffers.push(channel, two, callback);
    _resize(buffers, channel, 1);
    int counter = 0;
    await buffers.drain(channel, (
      ByteData? drainedData,
      ui.PlatformMessageResponseCallback drainedCallback,
    ) async {
      switch (counter) {
        case 0:
          expect(drainedData, equals(two));
      }
      counter += 1;
    });
    expect(counter, equals(1));
  });

  test('resize dropping calls callback', () async {
    const String channel = 'foo';
    final ByteData one = _makeByteData('one');
    final ByteData two = _makeByteData('two');
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    bool didCallCallback = false;
    void oneCallback(ByteData? responseData) {
      expect(responseData, isNull);
      didCallCallback = true;
    }

    void twoCallback(ByteData? responseData) {
      fail('wrong callback called');
    }

    _resize(buffers, channel, 100);
    buffers.push(channel, one, oneCallback);
    buffers.push(channel, two, twoCallback);
    expect(didCallCallback, equals(false));
    _resize(buffers, channel, 1);
    expect(didCallCallback, equals(true));
  });

  test('overflow calls callback', () async {
    const String channel = 'foo';
    final ByteData one = _makeByteData('one');
    final ByteData two = _makeByteData('two');
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    bool didCallCallback = false;
    void oneCallback(ByteData? responseData) {
      expect(responseData, isNull);
      didCallCallback = true;
    }

    void twoCallback(ByteData? responseData) {
      fail('wrong callback called');
    }

    _resize(buffers, channel, 1);
    buffers.push(channel, one, oneCallback);
    buffers.push(channel, two, twoCallback);
    expect(didCallCallback, equals(true));
  });

  test('handle garbage', () async {
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    expect(() => buffers.handleMessage(_makeByteData('asdfasdf')), throwsException);
  });

  test('handle resize garbage', () async {
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    expect(() => buffers.handleMessage(_makeByteData('resize\rfoo\rbar')), throwsException);
  });

  test('ChannelBuffers.setListener', () async {
    final List<String> log = <String>[];
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    final ByteData one = _makeByteData('one');
    final ByteData two = _makeByteData('two');
    final ByteData three = _makeByteData('three');
    final ByteData four = _makeByteData('four');
    final ByteData five = _makeByteData('five');
    final ByteData six = _makeByteData('six');
    final ByteData seven = _makeByteData('seven');
    buffers.push('a', one, (ByteData? data) {});
    buffers.push('b', two, (ByteData? data) {});
    buffers.push('a', three, (ByteData? data) {});
    log.add('top');
    buffers.setListener('a', (ByteData? data, ui.PlatformMessageResponseCallback callback) {
      expect(data, isNotNull);
      log.add('a1: ${utf8.decode(data!.buffer.asUint8List())}');
    });
    log.add('-1');
    await null;
    log.add('-2');
    buffers.setListener('a', (ByteData? data, ui.PlatformMessageResponseCallback callback) {
      expect(data, isNotNull);
      log.add('a2: ${utf8.decode(data!.buffer.asUint8List())}');
    });
    log.add('-3');
    await null;
    log.add('-4');
    buffers.setListener('b', (ByteData? data, ui.PlatformMessageResponseCallback callback) {
      expect(data, isNotNull);
      log.add('b: ${utf8.decode(data!.buffer.asUint8List())}');
    });
    log.add('-5');
    await null; // first microtask after setting listener drains the first message
    await null; // second microtask ends the draining.
    log.add('-6');
    buffers.push('b', four, (ByteData? data) {});
    buffers.push('a', five, (ByteData? data) {});
    log.add('-7');
    await null;
    log.add('-8');
    buffers.clearListener('a');
    buffers.push('a', six, (ByteData? data) {});
    buffers.push('b', seven, (ByteData? data) {});
    await null;
    log.add('-9');
    expect(log, <String>[
      'top',
      '-1',
      'a1: three',
      '-2',
      '-3',
      '-4',
      '-5',
      'b: two',
      '-6',
      'b: four',
      'a2: five',
      '-7',
      '-8',
      'b: seven',
      '-9',
    ]);
  });

  test('ChannelBuffers.clearListener', () async {
    final List<String> log = <String>[];
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    final ByteData one = _makeByteData('one');
    final ByteData two = _makeByteData('two');
    final ByteData three = _makeByteData('three');
    final ByteData four = _makeByteData('four');
    buffers.handleMessage(_makeByteData('resize\ra\r10'));
    buffers.push('a', one, (ByteData? data) {});
    buffers.push('a', two, (ByteData? data) {});
    buffers.push('a', three, (ByteData? data) {});
    log.add('-1');
    buffers.setListener('a', (ByteData? data, ui.PlatformMessageResponseCallback callback) {
      expect(data, isNotNull);
      log.add('a1: ${utf8.decode(data!.buffer.asUint8List())}');
    });
    await null; // handles one
    log.add('-2');
    buffers.clearListener('a');
    await null;
    log.add('-3');
    buffers.setListener('a', (ByteData? data, ui.PlatformMessageResponseCallback callback) {
      expect(data, isNotNull);
      log.add('a2: ${utf8.decode(data!.buffer.asUint8List())}');
    });
    log.add('-4');
    await null;
    buffers.push('a', four, (ByteData? data) {});
    log.add('-5');
    await null;
    log.add('-6');
    await null;
    log.add('-7');
    await null;
    expect(log, <String>[
      '-1',
      'a1: one',
      '-2',
      '-3',
      '-4',
      'a2: two',
      '-5',
      'a2: three',
      '-6',
      'a2: four',
      '-7',
    ]);
  });

  test('ChannelBuffers.handleMessage for resize', () async {
    final List<String> log = <String>[];
    final ui.ChannelBuffers buffers = _TestChannelBuffers(log);
    // Created as follows:
    //   print(StandardMethodCodec().encodeMethodCall(MethodCall('resize', ['abcdef', 12345])).buffer.asUint8List());
    // ...with three 0xFF bytes on either side to ensure the method works with an offset on the underlying buffer.
    buffers.handleMessage(
      ByteData.sublistView(
        Uint8List.fromList(<int>[
          255,
          255,
          255,
          7,
          6,
          114,
          101,
          115,
          105,
          122,
          101,
          12,
          2,
          7,
          6,
          97,
          98,
          99,
          100,
          101,
          102,
          3,
          57,
          48,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          255,
          255,
          255,
        ]),
        3,
        27,
      ),
    );
    expect(log, const <String>['resize abcdef 12345']);
  });

  test('ChannelBuffers.handleMessage for overflow', () async {
    final List<String> log = <String>[];
    final ui.ChannelBuffers buffers = _TestChannelBuffers(log);
    // Created as follows:
    //   print(StandardMethodCodec().encodeMethodCall(MethodCall('overflow', ['abcdef', false])).buffer.asUint8List());
    // ...with three 0xFF bytes on either side to ensure the method works with an offset on the underlying buffer.
    buffers.handleMessage(
      ByteData.sublistView(
        Uint8List.fromList(<int>[
          255,
          255,
          255,
          7,
          8,
          111,
          118,
          101,
          114,
          102,
          108,
          111,
          119,
          12,
          2,
          7,
          6,
          97,
          98,
          99,
          100,
          101,
          102,
          2,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          0,
          255,
          255,
          255,
        ]),
        3,
        24,
      ),
    );
    expect(log, const <String>['allowOverflow abcdef false']);
  });

  test('ChannelBuffers uses the right zones', () async {
    final List<String> log = <String>[];
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();
    final Zone zone1 = Zone.current.fork();
    final Zone zone2 = Zone.current.fork();
    zone1.run(() {
      log.add('first zone run: ${Zone.current == zone1}');
      buffers.setListener('a', (ByteData? data, ui.PlatformMessageResponseCallback callback) {
        log.add('callback1: ${Zone.current == zone1}');
        callback(data);
      });
    });
    zone2.run(() {
      log.add('second zone run: ${Zone.current == zone2}');
      buffers.push('a', ByteData.sublistView(Uint8List.fromList(<int>[]), 0, 0), (ByteData? data) {
        log.add('callback2: ${Zone.current == zone2}');
      });
    });
    await null;
    expect(log, <String>[
      'first zone run: true',
      'second zone run: true',
      'callback1: true',
      'callback2: true',
    ]);
  });

  test('ChannelBufferspush rejects names with nulls', () async {
    const String channel = 'foo\u0000bar';
    final ByteData blabla = _makeByteData('blabla');
    final ui.ChannelBuffers buffers = ui.ChannelBuffers();

    expect(
      () => buffers.push(channel, blabla, (ByteData? data) {}),
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('U+0000 NULL'),
        ),
      ),
    );

    expect(
      () => buffers.setListener(
        channel,
        (ByteData? data, ui.PlatformMessageResponseCallback callback) {},
      ),
      throwsA(
        isA<AssertionError>().having(
          (AssertionError e) => e.toString(),
          'toString',
          contains('U+0000 NULL'),
        ),
      ),
    );
  });
}

class _TestChannelBuffers extends ui.ChannelBuffers {
  _TestChannelBuffers(this.log);

  final List<String> log;

  @override
  void resize(String name, int newSize) {
    log.add('resize $name $newSize');
  }

  @override
  void allowOverflow(String name, bool allowed) {
    log.add('allowOverflow $name $allowed');
  }
}
