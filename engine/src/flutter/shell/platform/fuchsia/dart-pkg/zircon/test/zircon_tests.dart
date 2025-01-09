// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';
import 'dart:zircon';

import 'package:async_helper/async_minitest.dart';

/// Helper method to turn a [String] into a [ByteData] containing the
/// text of the string encoded as UTF-8.
ByteData utf8Bytes(final String text) {
  return ByteData.sublistView(utf8.encode(text));
}

// Take from zircon constants in zircon/errors.h, zircon/rights.h, zircon/types.h
abstract class ZX {
  ZX._();

  static const int OK = 0;
  static const int KOID_INVALID = 0;
  static const int ERR_BAD_HANDLE = -11;
  static const int ERR_SHOULD_WAIT = -22;
  static const int ERR_PEER_CLOSED = -24;
  static const int ERR_ACCESS_DENIED = -30;
  static const int EVENTPAIR_PEER_CLOSED = __ZX_OBJECT_PEER_CLOSED;
  static const int CHANNEL_READABLE = __ZX_OBJECT_READABLE;
  static const int CHANNEL_PEER_CLOSED = __ZX_OBJECT_PEER_CLOSED;
  static const int SOCKET_READABLE = __ZX_OBJECT_READABLE;
  static const int SOCKET_PEER_CLOSED = __ZX_OBJECT_PEER_CLOSED;
  static const int RIGHT_DUPLICATE = 1 << 0;
  static const int RIGHT_TRANSFER = 1 << 1;
  static const int RIGHT_READ = 1 << 2;
  static const int RIGHT_WRITE = 1 << 3;
  static const int RIGHT_GET_PROPERTY = 1 << 6;
  static const int RIGHT_SET_PROPERTY = 1 << 7;
  static const int RIGHT_MAP = 1 << 5;
  static const int RIGHT_SIGNAL = 1 << 12;
  static const int RIGHT_WAIT = 1 << 14;
  static const int RIGHT_INSPECT = 1 << 15;
  static const int RIGHT_SAME_RIGHTS = 1 << 31;
  static const int RIGHTS_BASIC = RIGHT_TRANSFER | RIGHT_DUPLICATE | RIGHT_WAIT | RIGHT_INSPECT;
  static const int RIGHTS_IO = RIGHT_READ | RIGHT_WRITE;
  static const int RIGHTS_PROPERTY = RIGHT_GET_PROPERTY | RIGHT_SET_PROPERTY;
  static const int DEFAULT_VMO_RIGHTS =
      RIGHTS_BASIC | RIGHTS_IO | RIGHTS_PROPERTY | RIGHT_MAP | RIGHT_SIGNAL;
  static const int OBJ_TYPE_VMO = 3;
  static const int OBJ_TYPE_CHANNEL = 4;
  static const int HANDLE_OP_MOVE = 0;
  static const int HANDLE_OP_DUPLICATE = 1;
  static const int __ZX_OBJECT_READABLE = 1 << 0;
  static const int __ZX_OBJECT_PEER_CLOSED = 1 << 2;
}

void main() {
  group('handle', () {
    test('create and duplicate handles', () {
      final HandlePairResult pair = System.eventpairCreate();
      expect(pair.status, equals(ZX.OK));
      expect(pair.first.isValid, isTrue);
      expect(pair.second.isValid, isTrue);

      final Handle duplicate = pair.first.duplicate(ZX.RIGHT_SAME_RIGHTS);
      expect(duplicate.isValid, isTrue);

      final Handle failedDuplicate = pair.first.duplicate(-1);
      expect(failedDuplicate.isValid, isFalse);
    });

    test('failure invalid rights', () {
      final HandleResult vmo = System.vmoCreate(0);
      expect(vmo.status, equals(ZX.OK));
      final Handle failedDuplicate = vmo.handle.duplicate(-1);
      expect(failedDuplicate.isValid, isFalse);
      expect(vmo.handle.isValid, isTrue);
    });

    test('failure invalid handle', () {
      final Handle handle = Handle.invalid();
      final Handle duplicate = handle.duplicate(ZX.RIGHT_SAME_RIGHTS);
      expect(duplicate.isValid, isFalse);
    });

    test('duplicated handle should have same koid', () {
      final HandlePairResult pair = System.eventpairCreate();
      expect(pair.status, equals(ZX.OK));
      expect(pair.first.isValid, isTrue);
      expect(pair.second.isValid, isTrue);

      final Handle duplicate = pair.first.duplicate(ZX.RIGHT_SAME_RIGHTS);
      expect(duplicate.isValid, isTrue);

      expect(pair.first.koid, duplicate.koid);
    });

    // TODO(fxbug.dev/77599): Simplify once zx_object_get_info is available.
    test('reduced rights', () {
      // Set up handle.
      final HandleResult vmo = System.vmoCreate(2);
      expect(vmo.status, equals(ZX.OK));

      // Duplicate the first handle.
      final Handle duplicate = vmo.handle.duplicate(ZX.RIGHTS_BASIC);
      expect(duplicate.isValid, isTrue);

      // Write bytes to the original handle.
      final ByteData data1 = utf8Bytes('a');
      final int status1 = System.vmoWrite(vmo.handle, 0, data1);
      expect(status1, equals(ZX.OK));

      // Write bytes to the duplicated handle.
      final ByteData data2 = utf8Bytes('b');
      final int status2 = System.vmoWrite(duplicate, 1, data2);
      expect(status2, equals(ZX.ERR_ACCESS_DENIED));

      // Read bytes.
      final ReadResult readResult = System.vmoRead(vmo.handle, 0, 2);
      expect(readResult.status, equals(ZX.OK));
      expect(readResult.numBytes, equals(2));
      expect(readResult.bytes.lengthInBytes, equals(2));
      expect(readResult.bytesAsUTF8String(), equals('a\x00'));
    });

    test('create and replace handles', () {
      final HandlePairResult pair = System.eventpairCreate();
      expect(pair.status, equals(ZX.OK));
      expect(pair.first.isValid, isTrue);
      expect(pair.second.isValid, isTrue);

      final Handle replaced = pair.first.replace(ZX.RIGHT_SAME_RIGHTS);
      expect(replaced.isValid, isTrue);
      expect(pair.first.isValid, isFalse);
    });

    test('failure invalid rights', () {
      final HandleResult vmo = System.vmoCreate(0);
      expect(vmo.status, equals(ZX.OK));
      final Handle failedDuplicate = vmo.handle.replace(-1);
      expect(failedDuplicate.isValid, isFalse);
      expect(vmo.handle.isValid, isFalse);
    });

    test('failure invalid handle', () {
      final Handle handle = Handle.invalid();
      final Handle replaced = handle.replace(ZX.RIGHT_SAME_RIGHTS);
      expect(handle.isValid, isFalse);
      expect(replaced.isValid, isFalse);
    });

    test('transferred handle should have same koid', () {
      final HandlePairResult pair = System.eventpairCreate();
      expect(pair.status, equals(ZX.OK));
      expect(pair.first.isValid, isTrue);
      expect(pair.second.isValid, isTrue);

      final int koid = pair.first.koid;
      final Handle replaced = pair.first.replace(ZX.RIGHT_SAME_RIGHTS);
      expect(replaced.isValid, isTrue);

      expect(koid, replaced.koid);
    });

    // TODO(fxbug.dev/77599): Simplify once zx_object_get_info is available.
    test('reduced rights', () {
      // Set up handle.
      final HandleResult vmo = System.vmoCreate(2);
      expect(vmo.status, equals(ZX.OK));

      // Replace the first handle.
      final Handle duplicate = vmo.handle.replace(ZX.RIGHTS_BASIC | ZX.RIGHT_READ);
      expect(duplicate.isValid, isTrue);

      // Write bytes to the original handle.
      final ByteData data1 = utf8Bytes('a');
      final int status1 = System.vmoWrite(vmo.handle, 0, data1);
      expect(status1, equals(ZX.ERR_BAD_HANDLE));

      // Write bytes to the duplicated handle.
      final ByteData data2 = utf8Bytes('b');
      final int status2 = System.vmoWrite(duplicate, 1, data2);
      expect(status2, equals(ZX.ERR_ACCESS_DENIED));

      // Read bytes.
      final ReadResult readResult = System.vmoRead(duplicate, 0, 2);
      expect(readResult.status, equals(ZX.OK));
      expect(readResult.numBytes, equals(2));
      expect(readResult.bytes.lengthInBytes, equals(2));
      expect(readResult.bytesAsUTF8String(), equals('\x00\x00'));
    });

    test('cache koid and invalidate', () {
      final HandleResult vmo = System.vmoCreate(0);
      expect(vmo.status, equals(ZX.OK));
      int originalKoid = vmo.handle.koid;
      expect(originalKoid != ZX.KOID_INVALID, true);
      // Cached koid should be same value.
      expect(originalKoid, equals(vmo.handle.koid));
      vmo.handle.close();
      // koid should be invalidated.
      expect(vmo.handle.koid, equals(ZX.KOID_INVALID));
    });

    test('store the disposition arguments correctly', () {
      final Handle handle = System.channelCreate().first;
      final HandleDisposition disposition = HandleDisposition(1, handle, 2, 3);
      expect(disposition.operation, equals(1));
      expect(disposition.handle, equals(handle));
      expect(disposition.type, equals(2));
      expect(disposition.rights, equals(3));
      expect(disposition.result, equals(ZX.OK));
    });
  });

  group('channel', () {
    test('create channel', () {
      final HandlePairResult pair = System.channelCreate();
      expect(pair.status, equals(ZX.OK));
      expect(pair.first.isValid, isTrue);
      expect(pair.second.isValid, isTrue);
    });

    test('close channel', () {
      final HandlePairResult pair = System.channelCreate();
      expect(pair.first.close(), equals(0));
      expect(pair.first.isValid, isFalse);
      expect(pair.second.isValid, isTrue);
      expect(System.channelWrite(pair.first, ByteData(1), <Handle>[]), equals(ZX.ERR_BAD_HANDLE));
      expect(System.channelWrite(pair.second, ByteData(1), <Handle>[]), equals(ZX.ERR_PEER_CLOSED));
    });

    test('channel bytes', () {
      final HandlePairResult pair = System.channelCreate();

      // When no data is available, ZX.ERR_SHOULD_WAIT is returned.
      expect(System.channelQueryAndRead(pair.second).status, equals(ZX.ERR_SHOULD_WAIT));

      // Write bytes.
      final ByteData data = utf8Bytes('Hello, world');
      final int status = System.channelWrite(pair.first, data, <Handle>[]);
      expect(status, equals(ZX.OK));

      // Read bytes.
      final ReadResult readResult = System.channelQueryAndRead(pair.second);
      expect(readResult.status, equals(ZX.OK));
      expect(readResult.numBytes, equals(data.lengthInBytes));
      expect(readResult.bytes.lengthInBytes, equals(data.lengthInBytes));
      expect(readResult.bytesAsUTF8String(), equals('Hello, world'));
      expect(readResult.handles.length, equals(0));
    });

    test('channel handles', () {
      final HandlePairResult pair = System.channelCreate();
      final ByteData data = utf8Bytes('');
      final HandlePairResult eventPair = System.eventpairCreate();
      final int status = System.channelWrite(pair.first, data, <Handle>[eventPair.first]);
      expect(status, equals(ZX.OK));
      expect(eventPair.first.isValid, isFalse);

      final ReadResult readResult = System.channelQueryAndRead(pair.second);
      expect(readResult.status, equals(ZX.OK));
      expect(readResult.numBytes, equals(0));
      expect(readResult.bytes.lengthInBytes, equals(0));
      expect(readResult.bytesAsUTF8String(), equals(''));
      expect(readResult.handles.length, equals(1));
      expect(readResult.handles[0].isValid, isTrue);
    });

    test('async wait channel read', () async {
      final HandlePairResult pair = System.channelCreate();
      final Completer<List<int>> completer = Completer<List<int>>();
      pair.first.asyncWait(ZX.CHANNEL_READABLE, (int status, int pending) {
        completer.complete(<int>[status, pending]);
      });

      expect(completer.isCompleted, isFalse);

      System.channelWrite(pair.second, utf8Bytes('Hi'), <Handle>[]);

      final List<int> result = await completer.future;
      expect(result[0], equals(ZX.OK)); // status
      expect(result[1] & ZX.CHANNEL_READABLE, equals(ZX.CHANNEL_READABLE)); // pending
    });

    test('async wait channel closed', () async {
      final HandlePairResult pair = System.channelCreate();
      final Completer<int> completer = Completer<int>();
      pair.first.asyncWait(ZX.CHANNEL_PEER_CLOSED, (int status, int pending) {
        completer.complete(status);
      });

      expect(completer.isCompleted, isFalse);

      pair.second.close();

      final int status = await completer.future;
      expect(status, equals(ZX.OK));
    });
  });

  group('channel etc functions', () {
    test('moved handle', () {
      final HandlePairResult pair = System.channelCreate();
      final ByteData data = utf8Bytes('');
      final HandlePairResult transferred = System.channelCreate();

      final HandleDisposition disposition = HandleDisposition(
        ZX.HANDLE_OP_MOVE,
        transferred.first,
        ZX.OBJ_TYPE_CHANNEL,
        ZX.RIGHTS_IO,
      );
      final int status = System.channelWriteEtc(pair.first, data, <HandleDisposition>[disposition]);
      expect(status, equals(ZX.OK));
      expect(disposition.result, equals(ZX.OK));
      expect(transferred.first.isValid, isFalse);

      final ReadEtcResult readResult = System.channelQueryAndReadEtc(pair.second);
      expect(readResult.status, equals(ZX.OK));
      expect(readResult.numBytes, equals(0));
      expect(readResult.bytes.lengthInBytes, equals(0));
      expect(readResult.bytesAsUTF8String(), equals(''));
      expect(readResult.handleInfos.length, equals(1));
      final HandleInfo handleInfo = readResult.handleInfos[0];
      expect(handleInfo.handle.isValid, isTrue);
      expect(handleInfo.type, equals(ZX.OBJ_TYPE_CHANNEL));
      expect(handleInfo.rights, equals(ZX.RIGHTS_IO));
    });

    test('copied handle', () {
      final HandlePairResult pair = System.channelCreate();
      final ByteData data = utf8Bytes('');
      final HandleResult vmo = System.vmoCreate(0);

      final HandleDisposition disposition = HandleDisposition(
        ZX.HANDLE_OP_DUPLICATE,
        vmo.handle,
        ZX.OBJ_TYPE_VMO,
        ZX.RIGHT_SAME_RIGHTS,
      );
      final int status = System.channelWriteEtc(pair.first, data, <HandleDisposition>[disposition]);
      expect(status, equals(ZX.OK));
      expect(disposition.result, equals(ZX.OK));
      expect(vmo.handle.isValid, isTrue);

      final ReadEtcResult readResult = System.channelQueryAndReadEtc(pair.second);
      expect(readResult.status, equals(ZX.OK));
      expect(readResult.numBytes, equals(0));
      expect(readResult.bytes.lengthInBytes, equals(0));
      expect(readResult.bytesAsUTF8String(), equals(''));
      expect(readResult.handleInfos.length, equals(1));
      final HandleInfo handleInfo = readResult.handleInfos[0];
      expect(handleInfo.handle.isValid, isTrue);
      expect(handleInfo.type, equals(ZX.OBJ_TYPE_VMO));
      expect(handleInfo.rights, equals(ZX.DEFAULT_VMO_RIGHTS));
    });

    test('closed handle should error', () {
      final HandlePairResult pair = System.channelCreate();
      final ByteData data = utf8Bytes('');
      final HandlePairResult closed = System.channelCreate();

      final HandleDisposition disposition = HandleDisposition(
        ZX.HANDLE_OP_MOVE,
        closed.first,
        ZX.OBJ_TYPE_CHANNEL,
        ZX.RIGHT_SAME_RIGHTS,
      );
      closed.first.close();
      final int status = System.channelWriteEtc(pair.first, data, <HandleDisposition>[disposition]);
      expect(status, equals(ZX.ERR_BAD_HANDLE));
      expect(disposition.result, equals(ZX.ERR_BAD_HANDLE));
      expect(closed.first.isValid, isFalse);

      final ReadEtcResult readResult = System.channelQueryAndReadEtc(pair.second);
      expect(readResult.status, equals(ZX.ERR_SHOULD_WAIT));
    });

    test('multiple handles', () {
      final HandlePairResult pair = System.channelCreate();
      final ByteData data = utf8Bytes('');
      final HandlePairResult transferred = System.channelCreate();
      final HandleResult vmo = System.vmoCreate(0);

      final List<HandleDisposition> dispositions = [
        HandleDisposition(ZX.HANDLE_OP_MOVE, transferred.first, ZX.OBJ_TYPE_CHANNEL, ZX.RIGHTS_IO),
        HandleDisposition(
          ZX.HANDLE_OP_DUPLICATE,
          vmo.handle,
          ZX.OBJ_TYPE_VMO,
          ZX.RIGHT_SAME_RIGHTS,
        ),
      ];
      final int status = System.channelWriteEtc(pair.first, data, dispositions);
      expect(status, equals(ZX.OK));
      expect(dispositions[0].result, equals(ZX.OK));
      expect(dispositions[1].result, equals(ZX.OK));
      expect(transferred.first.isValid, isFalse);
      expect(vmo.handle.isValid, isTrue);

      final ReadEtcResult readResult = System.channelQueryAndReadEtc(pair.second);
      expect(readResult.status, equals(ZX.OK));
      expect(readResult.numBytes, equals(0));
      expect(readResult.bytes.lengthInBytes, equals(0));
      expect(readResult.bytesAsUTF8String(), equals(''));

      expect(readResult.handleInfos.length, equals(2));
      final HandleInfo handleInfo = readResult.handleInfos[0];
      expect(handleInfo.handle.isValid, isTrue);
      expect(handleInfo.type, equals(ZX.OBJ_TYPE_CHANNEL));
      expect(handleInfo.rights, equals(ZX.RIGHTS_IO));
      final HandleInfo vmoInfo = readResult.handleInfos[1];
      expect(vmoInfo.handle.isValid, isTrue);
      expect(vmoInfo.type, equals(ZX.OBJ_TYPE_VMO));
      expect(vmoInfo.rights, equals(ZX.DEFAULT_VMO_RIGHTS));
    });
  });

  group('eventpair', () {
    test('create', () {
      final HandlePairResult pair = System.eventpairCreate();
      expect(pair.status, equals(ZX.OK));
      expect(pair.first.isValid, isTrue);
      expect(pair.second.isValid, isTrue);
    });

    test('duplicate', () {
      final HandlePairResult pair = System.eventpairCreate();
      expect(pair.status, equals(ZX.OK));
      expect(pair.first.isValid, isTrue);
      expect(pair.second.isValid, isTrue);

      expect(pair.first.duplicate(ZX.RIGHT_SAME_RIGHTS).isValid, isTrue);
      expect(pair.second.duplicate(ZX.RIGHT_SAME_RIGHTS).isValid, isTrue);
    });

    test('close', () {
      final HandlePairResult pair = System.eventpairCreate();
      expect(pair.first.close(), equals(0));
      expect(pair.first.isValid, isFalse);
      expect(pair.second.isValid, isTrue);

      expect(pair.second.close(), equals(0));
      expect(pair.second.isValid, isFalse);
    });

    test('async wait peer closed', () async {
      final HandlePairResult pair = System.eventpairCreate();
      final Completer<int> completer = Completer<int>();
      pair.first.asyncWait(ZX.EVENTPAIR_PEER_CLOSED, (int status, int pending) {
        completer.complete(status);
      });

      expect(completer.isCompleted, isFalse);
      pair.second.close();

      final int status = await completer.future;
      expect(status, equals(ZX.OK));
    });
  });

  // NOTE: This only tests stream sockets.
  // We should add tests for datagram sockets.
  group('socket', () {
    test('create socket', () {
      final HandlePairResult pair = System.socketCreate();
      expect(pair.status, equals(ZX.OK));
      expect(pair.first.isValid, isTrue);
      expect(pair.second.isValid, isTrue);
    });

    test('close socket', () {
      final HandlePairResult pair = System.socketCreate();
      expect(pair.first.close(), equals(0));
      expect(pair.first.isValid, isFalse);
      expect(pair.second.isValid, isTrue);
      final WriteResult firstResult = System.socketWrite(pair.first, ByteData(1), 0);
      expect(firstResult.status, equals(ZX.ERR_BAD_HANDLE));
      final WriteResult secondResult = System.socketWrite(pair.second, ByteData(1), 0);
      expect(secondResult.status, equals(ZX.ERR_PEER_CLOSED));
    });

    test('read write socket', () {
      final HandlePairResult pair = System.socketCreate();

      // When no data is available, ZX.ERR_SHOULD_WAIT is returned.
      expect(System.socketRead(pair.second, 1).status, equals(ZX.ERR_SHOULD_WAIT));

      final ByteData data = utf8Bytes('Hello, world');
      final WriteResult writeResult = System.socketWrite(pair.first, data, 0);
      expect(writeResult.status, equals(ZX.OK));

      final ReadResult readResult = System.socketRead(pair.second, data.lengthInBytes);
      expect(readResult.status, equals(ZX.OK));
      expect(readResult.numBytes, equals(data.lengthInBytes));
      expect(readResult.bytes.lengthInBytes, equals(data.lengthInBytes));
      expect(readResult.bytesAsUTF8String(), equals('Hello, world'));
    });

    test('partial read socket', () {
      final HandlePairResult pair = System.socketCreate();
      final ByteData data = utf8Bytes('Hello, world');
      final WriteResult writeResult = System.socketWrite(pair.first, data, 0);
      expect(writeResult.status, equals(ZX.OK));

      const int shortLength = 'Hello'.length;
      final ReadResult shortReadResult = System.socketRead(pair.second, shortLength);
      expect(shortReadResult.status, equals(ZX.OK));
      expect(shortReadResult.numBytes, equals(shortLength));
      expect(shortReadResult.bytes.lengthInBytes, equals(shortLength));
      expect(shortReadResult.bytesAsUTF8String(), equals('Hello'));

      final int longLength = data.lengthInBytes * 2;
      final ReadResult longReadResult = System.socketRead(pair.second, longLength);
      expect(longReadResult.status, equals(ZX.OK));
      expect(longReadResult.numBytes, equals(data.lengthInBytes - shortLength));
      expect(longReadResult.bytes.lengthInBytes, equals(longLength));
      expect(longReadResult.bytesAsUTF8String(), equals(', world'));
    });

    test('partial write socket', () {
      final HandlePairResult pair = System.socketCreate();
      final WriteResult writeResult1 = System.socketWrite(pair.first, utf8Bytes('Hello, '), 0);
      expect(writeResult1.status, equals(ZX.OK));
      final WriteResult writeResult2 = System.socketWrite(pair.first, utf8Bytes('world'), 0);
      expect(writeResult2.status, equals(ZX.OK));

      final ReadResult readResult = System.socketRead(pair.second, 100);
      expect(readResult.status, equals(ZX.OK));
      expect(readResult.numBytes, equals('Hello, world'.length));
      expect(readResult.bytes.lengthInBytes, equals(100));
      expect(readResult.bytesAsUTF8String(), equals('Hello, world'));
    });

    test('async wait socket read', () async {
      final HandlePairResult pair = System.socketCreate();
      final Completer<int> completer = Completer<int>();
      pair.first.asyncWait(ZX.SOCKET_READABLE, (int status, int pending) {
        completer.complete(status);
      });

      expect(completer.isCompleted, isFalse);

      System.socketWrite(pair.second, utf8Bytes('Hi'), 0);

      final int status = await completer.future;
      expect(status, equals(ZX.OK));
    });

    test('async wait socket closed', () async {
      final HandlePairResult pair = System.socketCreate();
      final Completer<int> completer = Completer<int>();
      pair.first.asyncWait(ZX.SOCKET_PEER_CLOSED, (int status, int pending) {
        completer.complete(status);
      });

      expect(completer.isCompleted, isFalse);

      pair.second.close();

      final int status = await completer.future;
      expect(status, equals(ZX.OK));
    });
  });

  group('vmo', () {
    test('fromFile', () {
      const String fuchsia = 'Fuchsia';
      File f =
          File('tmp/testdata')
            ..createSync()
            ..writeAsStringSync(fuchsia);
      String readFuchsia = f.readAsStringSync();
      expect(readFuchsia, equals(fuchsia));

      FromFileResult fileResult = System.vmoFromFile('tmp/testdata');
      expect(fileResult.status, equals(ZX.OK));
      MapResult mapResult = System.vmoMap(fileResult.handle);
      expect(mapResult.status, equals(ZX.OK));
      Uint8List fileData = mapResult.data.asUnmodifiableView();
      String fileString = utf8.decode(fileData.sublist(0, fileResult.numBytes));
      expect(fileString, equals(fuchsia));
    });

    test('duplicate', () {
      const String fuchsia = 'Fuchsia';
      Uint8List data = Uint8List.fromList(fuchsia.codeUnits);
      HandleResult createResult = System.vmoCreate(data.length);
      expect(createResult.status, equals(ZX.OK));
      int writeResult = System.vmoWrite(createResult.handle, 0, data.buffer.asByteData());
      expect(writeResult, equals(ZX.OK));
      Handle duplicate = createResult.handle.duplicate(
        ZX.RIGHTS_BASIC | ZX.RIGHT_READ | ZX.RIGHT_MAP,
      );
      expect(duplicate.isValid, isTrue);

      // Read from the duplicate.
      MapResult mapResult = System.vmoMap(duplicate);
      expect(mapResult.status, equals(ZX.OK));
      Uint8List vmoData = mapResult.data.asUnmodifiableView();
      String vmoString = utf8.decode(vmoData.sublist(0, data.length));
      expect(vmoString, equals(fuchsia));
    });
  });
}
