// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:typed_data';

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/proxied_devices/file_transfer.dart';

import '../../src/common.dart';

void main() {
  group('convertToChunks', () {
    test('works correctly', () async {
      final StreamController<Uint8List> controller = StreamController<Uint8List>();
      final Stream<Uint8List> chunked = convertToChunks(controller.stream, 4);
      final Future<List<Uint8List>> chunkedListFuture = chunked.toList();

      // Full chunk.
      controller.add(Uint8List.fromList(<int>[1, 2, 3, 4]));
      // Multiple of full chunks, on chunk boundaries.
      controller.add(Uint8List.fromList(<int>[5, 6, 7, 8, 9, 10, 11, 12]));
      // Larger than one chunk, starts on chunk boundary, ends not on chunk boundary.
      controller.add(Uint8List.fromList(<int>[13, 14, 15, 16, 17, 18]));
      // Larger than one chunk, starts not on chunk boundary, ends not on chunk boundary.
      controller.add(Uint8List.fromList(<int>[19, 20, 21, 22, 23]));
      // Larger than one chunk, starts not on chunk boundary, ends on chunk boundary.
      controller.add(Uint8List.fromList(<int>[24, 25, 26, 27, 28]));
      // Smaller than one chunk, starts on chunk boundary, ends not on chunk boundary.
      controller.add(Uint8List.fromList(<int>[29, 30]));
      // Smaller than one chunk, starts not on chunk boundary, ends not on chunk boundary.
      controller.add(Uint8List.fromList(<int>[31, 32, 33]));
      // Full chunk, not on chunk boundary.
      controller.add(Uint8List.fromList(<int>[34, 35, 36, 37]));
      // Smaller than one chunk, starts not on chunk boundary, ends on chunk boundary.
      controller.add(Uint8List.fromList(<int>[38, 39, 40]));
      // Empty chunk.
      controller.add(Uint8List.fromList(<int>[]));
      // Extra chunk.
      controller.add(Uint8List.fromList(<int>[41, 42]));

      await controller.close();

      final List<Uint8List> chunkedList = await chunkedListFuture;

      expect(chunkedList, hasLength(11));
      expect(chunkedList[0], <int>[1, 2, 3, 4]);
      expect(chunkedList[1], <int>[5, 6, 7, 8]);
      expect(chunkedList[2], <int>[9, 10, 11, 12]);
      expect(chunkedList[3], <int>[13, 14, 15, 16]);
      expect(chunkedList[4], <int>[17, 18, 19, 20]);
      expect(chunkedList[5], <int>[21, 22, 23, 24]);
      expect(chunkedList[6], <int>[25, 26, 27, 28]);
      expect(chunkedList[7], <int>[29, 30, 31, 32]);
      expect(chunkedList[8], <int>[33, 34, 35, 36]);
      expect(chunkedList[9], <int>[37, 38, 39, 40]);
      expect(chunkedList[10], <int>[41, 42]);
    });
  });

  group('adler32Hash', () {
    test('works correctly', () {
      final int hash = adler32Hash(Uint8List.fromList(utf8.encode('abcdefg')));
      expect(hash, 0x0adb02bd);
    });
  });

  group('RollingAdler32', () {
    test('works correctly without rolling', () {
      final RollingAdler32 adler32 = RollingAdler32(7);
      utf8.encode('abcdefg').forEach(adler32.push);
      expect(adler32.hash, adler32Hash(Uint8List.fromList(utf8.encode('abcdefg'))));
    });

    test('works correctly after rolling once', () {
      final RollingAdler32 adler32 = RollingAdler32(7);
      utf8.encode('12abcdefg').forEach(adler32.push);
      expect(adler32.hash, adler32Hash(Uint8List.fromList(utf8.encode('abcdefg'))));
    });

    test('works correctly after rolling multiple cycles', () {
      final RollingAdler32 adler32 = RollingAdler32(7);
      utf8.encode('1234567890123456789abcdefg').forEach(adler32.push);
      expect(adler32.hash, adler32Hash(Uint8List.fromList(utf8.encode('abcdefg'))));
    });

    test('works correctly after reset', () {
      final RollingAdler32 adler32 = RollingAdler32(7);
      utf8.encode('1234567890123456789abcdefg').forEach(adler32.push);
      adler32.reset();
      utf8.encode('abcdefg').forEach(adler32.push);
      expect(adler32.hash, adler32Hash(Uint8List.fromList(utf8.encode('abcdefg'))));
    });

    test('currentBlock returns the correct entry when read less than one block', () {
      final RollingAdler32 adler32 = RollingAdler32(7);
      utf8.encode('abcd').forEach(adler32.push);
      expect(adler32.currentBlock(), utf8.encode('abcd'));
    });

    test('currentBlock returns the correct entry when read exactly one block', () {
      final RollingAdler32 adler32 = RollingAdler32(7);
      utf8.encode('abcdefg').forEach(adler32.push);
      expect(adler32.currentBlock(), utf8.encode('abcdefg'));
    });

    test('currentBlock returns the correct entry when read more than one block', () {
      final RollingAdler32 adler32 = RollingAdler32(7);
      utf8.encode('123456789abcdefg').forEach(adler32.push);
      expect(adler32.currentBlock(), utf8.encode('abcdefg'));
    });
  });

  group('FileTransfer', () {
    const String content1 = 'a...b...c...d...e.';
    const String content2 = 'b...c...d...a...f...g...b...h..';
    const List<FileDeltaBlock> expectedDelta = <FileDeltaBlock>[
      FileDeltaBlock.fromDestination(start: 4, size: 12),
      FileDeltaBlock.fromDestination(start: 0, size: 4),
      FileDeltaBlock.fromSource(start: 16, size: 8),
      FileDeltaBlock.fromDestination(start: 4, size: 4),
      FileDeltaBlock.fromSource(start: 28, size: 3),
    ];
    const String expectedBinaryForRebuilding = 'f...g...h..';
    late MemoryFileSystem fileSystem;
    setUp(() {
      fileSystem = MemoryFileSystem();
    });

    test('calculateBlockHashesOfFile works normally', () async {
      final File file = fileSystem.file('test')..writeAsStringSync(content1);

      final BlockHashes hashes = await const FileTransfer().calculateBlockHashesOfFile(
        file,
        blockSize: 4,
      );
      expect(hashes.blockSize, 4);
      expect(hashes.totalSize, content1.length);
      expect(hashes.adler32, hasLength(5));
      expect(hashes.adler32, <int>[0x029c00ec, 0x02a000ed, 0x02a400ee, 0x02a800ef, 0x00fa0094]);
      expect(hashes.md5, hasLength(5));
      expect(hashes.md5, <String>[
        'zB0S8R/fGt05GcI5v8AjIQ==',
        'uZCZ4i/LUGFYAD+K1ZD0Wg==',
        '6kbZGS8T1NJl/naWODQcNw==',
        'kKh/aA2XAhR/r0HdZa3Bxg==',
        '34eF7Bs/OhfoJ5+sAw0zyw==',
      ]);
      expect(hashes.fileMd5, 'VT/gkSEdctzUEUJCxclxuQ==');
    });

    test('computeDelta returns empty list if file is identical', () async {
      final File file1 = fileSystem.file('file1')..writeAsStringSync(content1);
      final File file2 = fileSystem.file('file1')..writeAsStringSync(content1);

      final BlockHashes hashes = await const FileTransfer().calculateBlockHashesOfFile(
        file1,
        blockSize: 4,
      );
      final List<FileDeltaBlock> delta = await const FileTransfer().computeDelta(file2, hashes);

      expect(delta, isEmpty);
    });

    test('computeDelta returns the correct delta', () async {
      final File file1 = fileSystem.file('file1')..writeAsStringSync(content1);
      final File file2 = fileSystem.file('file2')..writeAsStringSync(content2);

      final BlockHashes hashes = await const FileTransfer().calculateBlockHashesOfFile(
        file1,
        blockSize: 4,
      );
      final List<FileDeltaBlock> delta = await const FileTransfer().computeDelta(file2, hashes);

      expect(delta, expectedDelta);
    });

    test('binaryForRebuilding returns the correct binary', () async {
      final File file = fileSystem.file('file')..writeAsStringSync(content2);
      final List<int> binaryForRebuilding = await const FileTransfer().binaryForRebuilding(
        file,
        expectedDelta,
      );
      expect(binaryForRebuilding, utf8.encode(expectedBinaryForRebuilding));
    });

    test('rebuildFile can rebuild the correct file', () async {
      final File file = fileSystem.file('file')..writeAsStringSync(content1);
      await const FileTransfer().rebuildFile(
        file,
        expectedDelta,
        Stream<List<int>>.fromIterable(<List<int>>[utf8.encode(expectedBinaryForRebuilding)]),
      );
      expect(file.readAsStringSync(), content2);
    });
  });

  group('BlockHashes', () {
    test('json conversion works normally', () {
      const String json = '''
{
  "blockSize":4,
  "totalSize":18,
  "adler32":"7ACcAu0AoALuAKQC7wCoApQA+gA=",
  "md5": [
    "zB0S8R/fGt05GcI5v8AjIQ==",
    "uZCZ4i/LUGFYAD+K1ZD0Wg==",
    "6kbZGS8T1NJl/naWODQcNw==",
    "kKh/aA2XAhR/r0HdZa3Bxg==",
    "34eF7Bs/OhfoJ5+sAw0zyw=="
  ],
  "fileMd5":"VT/gkSEdctzUEUJCxclxuQ=="
}
''';
      final Map<String, Object?> decodedJson = jsonDecode(json) as Map<String, Object?>;
      expect(BlockHashes.fromJson(decodedJson).toJson(), decodedJson);
    });
  });
}
