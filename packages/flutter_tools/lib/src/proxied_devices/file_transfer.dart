// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:math';
import 'dart:typed_data';

import 'package:crypto/crypto.dart';
import 'package:meta/meta.dart';

import '../base/file_system.dart';
import '../build_system/hash.dart';
import '../convert.dart';

/// Adler-32 and MD5 hashes of blocks in files.
class BlockHashes {
  const BlockHashes({
    required this.blockSize,
    required this.totalSize,
    required this.adler32,
    required this.md5,
    required this.fileMd5,
  });

  BlockHashes.fromJson(Map<String, Object?> obj)
    : blockSize = obj['blockSize']! as int,
      totalSize = obj['totalSize']! as int,
      adler32 = Uint32List.view(base64.decode(obj['adler32']! as String).buffer),
      md5 = (obj['md5']! as List<Object?>).cast<String>(),
      fileMd5 = obj['fileMd5']! as String;

  /// The block size used to generate the hashes.
  final int blockSize;

  /// Total size of the file.
  final int totalSize;

  /// List of adler32 hashes of each block in the file.
  final List<int> adler32;

  /// List of MD5 hashes of each block in the file.
  final List<String> md5;

  /// MD5 hash of the whole file.
  final String fileMd5;

  Map<String, Object> toJson() => <String, Object>{
    'blockSize': blockSize,
    'totalSize': totalSize,
    'adler32': base64.encode(Uint8List.view(Uint32List.fromList(adler32).buffer)),
    'md5': md5,
    'fileMd5': fileMd5,
  };
}

/// Converts a stream of bytes, into a stream of bytes of fixed chunk size.
@visibleForTesting
Stream<Uint8List> convertToChunks(Stream<Uint8List> source, int chunkSize) {
  final BytesBuilder bytesBuilder = BytesBuilder(copy: false);
  final StreamController<Uint8List> controller = StreamController<Uint8List>();
  final StreamSubscription<Uint8List> subscription = source.listen(
    (Uint8List chunk) {
      int start = 0;
      while (start < chunk.length) {
        final int sizeToTake = min(chunkSize - bytesBuilder.length, chunk.length - start);
        assert(sizeToTake > 0);
        assert(sizeToTake <= chunkSize);

        final Uint8List sublist = chunk.sublist(start, start + sizeToTake);
        start += sizeToTake;

        if (bytesBuilder.isEmpty && sizeToTake == chunkSize) {
          controller.add(sublist);
        } else {
          bytesBuilder.add(sublist);
          assert(bytesBuilder.length <= chunkSize);
          if (bytesBuilder.length == chunkSize) {
            controller.add(bytesBuilder.takeBytes());
          }
        }
      }
    },
    onDone: () {
      if (controller.hasListener && !controller.isClosed) {
        if (bytesBuilder.isNotEmpty) {
          controller.add(bytesBuilder.takeBytes());
        }
        controller.close();
      }
    },
    onError: (Object error, StackTrace stackTrace) {
      controller.addError(error, stackTrace);
    },
  );

  controller.onCancel = subscription.cancel;
  controller.onPause = subscription.pause;
  controller.onResume = subscription.resume;

  return controller.stream;
}

const int _adler32Prime = 65521;

/// Helper function to calculate Adler32 hash of a binary.
@visibleForTesting
int adler32Hash(Uint8List binary) {
  // The maximum integer that can be stored in the `int` data type.
  const int maxInt = 0x1fffffffffffff;
  // maxChunkSize is the maximum number of bytes we can sum without
  // performing the modulus operation, without overflow.
  // n * (n + 1) / 2 * 255 < maxInt
  // n < sqrt(maxInt / 255) - 1
  final int maxChunkSize = sqrt(maxInt / 255).floor() - 1;

  int a = 1;
  int b = 0;

  final int length = binary.length;
  for (int i = 0; i < length; i += maxChunkSize) {
    final int end = i + maxChunkSize < length ? i + maxChunkSize : length;
    for (int j = i; j < end; j++) {
      a += binary[j];
      b += a;
    }
    a %= _adler32Prime;
    b %= _adler32Prime;
  }

  return ((b & 0xffff) << 16) | (a & 0xffff);
}

/// Helper to calculate rolling Adler32 hash of a file.
@visibleForTesting
class RollingAdler32 {
  RollingAdler32(this.blockSize) : _buffer = Uint8List(blockSize);

  /// Block size of the rolling hash calculation.
  final int blockSize;

  int processedBytes = 0;

  final Uint8List _buffer;
  int _cur = 0;
  int _a = 1;
  int _b = 0;

  /// The current rolling hash value.
  int get hash => ((_b & 0xffff) << 16) | (_a & 0xffff);

  /// Push a new character into the rolling chunk window, and returns the
  /// current hash value.
  int push(int char) {
    processedBytes++;

    if (processedBytes > blockSize) {
      final int prev = _buffer[_cur];
      _b -= prev * blockSize + 1;
      _a -= prev;
    }

    _a += char;
    _b += _a;

    _buffer[_cur] = char;
    _cur++;
    if (_cur == blockSize) {
      _cur = 0;
    }

    _a %= _adler32Prime;
    _b %= _adler32Prime;

    return hash;
  }

  /// Returns a [Uint8List] of size [blockSize] that was used to calculate the
  /// current Adler32 hash.
  Uint8List currentBlock() {
    if (processedBytes < blockSize) {
      return Uint8List.sublistView(_buffer, 0, processedBytes);
    } else if (_cur == 0) {
      return _buffer;
    } else {
      final BytesBuilder builder =
          BytesBuilder(copy: false)
            ..add(Uint8List.sublistView(_buffer, _cur))
            ..add(Uint8List.sublistView(_buffer, 0, _cur));
      return builder.takeBytes();
    }
  }

  void reset() {
    _a = 1;
    _b = 0;
    processedBytes = 0;
  }
}

/// Helper for rsync-like file transfer.
///
/// The algorithm works as follows.
///
/// First, in the destination device, calculate hashes of the every block of
/// the same size. Two hashes are used, Adler-32 for the rolling hash, and MD5
/// as a hash with a lower chance of collision.
///
/// The block size is chosen to balance between the amount of data required in
/// the initial transmission, and the amount of data needed for rebuilding the
/// file.
///
/// Next, on the machine that contains the source file, we calculate the
/// rolling hash of the source file, for every possible position. If the hash
/// is found on the block hashes, we then compare the MD5 of the block. If both
/// the Adler-32 and MD5 hash match, we consider that the block is identical.
///
/// For each block that can be found, we will generate the instruction asking
/// the destination machine to read block from the destination block. For
/// blocks that can't be found, we will transfer the content of the blocks.
///
/// On the receiving end, it will build a copy of the source file from the
/// given instructions.
class FileTransfer {
  const FileTransfer();

  /// Calculate hashes of blocks in the file.
  Future<BlockHashes> calculateBlockHashesOfFile(File file, {int? blockSize}) async {
    final int totalSize = await file.length();
    blockSize ??= max(sqrt(totalSize).ceil(), 2560);

    final Stream<Uint8List> fileContentStream = file.openRead().map(
      (List<int> chunk) => chunk is Uint8List ? chunk : Uint8List.fromList(chunk),
    );

    final List<int> adler32Results = <int>[];
    final List<String> md5Results = <String>[];

    await convertToChunks(fileContentStream, blockSize).forEach((Uint8List chunk) {
      adler32Results.add(adler32Hash(chunk));
      md5Results.add(base64.encode(md5.convert(chunk).bytes));
    });

    // Handle whole file md5 separately. Md5Hash requires the chunk size to be a multiple of 64.
    final String fileMd5 = await _md5OfFile(file);

    return BlockHashes(
      blockSize: blockSize,
      totalSize: totalSize,
      adler32: adler32Results,
      md5: md5Results,
      fileMd5: fileMd5,
    );
  }

  /// Compute the instructions to rebuild the source [file] with the block
  /// hashes of the destination file.
  ///
  /// Returns an empty list if the destination file is exactly the same as the
  /// source file.
  Future<List<FileDeltaBlock>> computeDelta(File file, BlockHashes hashes) async {
    // Skip computing delta if the destination file matches the source file.
    if (await file.length() == hashes.totalSize && await _md5OfFile(file) == hashes.fileMd5) {
      return <FileDeltaBlock>[];
    }

    final Stream<List<int>> fileContentStream = file.openRead();
    final int blockSize = hashes.blockSize;

    // Generate a lookup for adler32 hash to block index.
    final Map<int, List<int>> adler32ToBlockIndex = <int, List<int>>{};
    for (int i = 0; i < hashes.adler32.length; i++) {
      (adler32ToBlockIndex[hashes.adler32[i]] ??= <int>[]).add(i);
    }

    final RollingAdler32 adler32 = RollingAdler32(blockSize);

    // Number of bytes read.
    int size = 0;

    // Offset of the beginning of the current block.
    int start = 0;

    final List<FileDeltaBlock> blocks = <FileDeltaBlock>[];

    await fileContentStream.forEach((List<int> chunk) {
      for (int i = 0; i < chunk.length; i++) {
        final int c = chunk[i];
        final int hash = adler32.push(c);
        size++;

        if (size - start < blockSize) {
          // Ignore if we have not processed enough bytes.
          continue;
        }

        if (!adler32ToBlockIndex.containsKey(hash)) {
          // Adler32 hash of the current block does not match the destination file.
          continue;
        }

        // The indices of possible matching blocks.
        final List<int> blockIndices = adler32ToBlockIndex[hash]!;
        final String md5Hash = base64.encode(md5.convert(adler32.currentBlock()).bytes);

        // Verify if any of our findings actually matches the destination block by comparing its MD5.
        for (final int blockIndex in blockIndices) {
          if (hashes.md5[blockIndex] != md5Hash) {
            // Adler-32 hash collision. This is not an actual match.
            continue;
          }

          // Found matching entry, generate instruction for reconstructing the file.

          // Copy the previously unmatched data from the source file.
          if (size - start > blockSize) {
            blocks.add(FileDeltaBlock.fromSource(start: start, size: size - start - blockSize));
          }

          start = size;

          // Try to extend the previous entry.
          if (blocks.isNotEmpty && blocks.last.copyFromDestination) {
            final int lastBlockIndex = (blocks.last.start + blocks.last.size) ~/ blockSize;
            if (hashes.md5[lastBlockIndex] == md5Hash) {
              // We can extend the previous entry.
              final FileDeltaBlock last = blocks.removeLast();
              blocks.add(
                FileDeltaBlock.fromDestination(start: last.start, size: last.size + blockSize),
              );
              break;
            }
          }

          blocks.add(
            FileDeltaBlock.fromDestination(start: blockIndex * blockSize, size: blockSize),
          );
          break;
        }
      }
    });

    // For the remaining content that is not matched, copy from the source.
    if (start < size) {
      blocks.add(FileDeltaBlock.fromSource(start: start, size: size - start));
    }

    return blocks;
  }

  /// Generates the binary blocks that need to be transferred to the remote
  /// end to regenerate the file.
  Future<Uint8List> binaryForRebuilding(File file, List<FileDeltaBlock> delta) async {
    final RandomAccessFile binaryView = await file.open();
    final Iterable<FileDeltaBlock> toTransfer = delta.where(
      (FileDeltaBlock block) => !block.copyFromDestination,
    );
    final int totalSize = toTransfer
        .map((FileDeltaBlock i) => i.size)
        .reduce((int a, int b) => a + b);
    final Uint8List buffer = Uint8List(totalSize);
    int start = 0;
    for (final FileDeltaBlock current in toTransfer) {
      await binaryView.setPosition(current.start);
      await binaryView.readInto(buffer, start, start + current.size);
      start += current.size;
    }

    assert(start == buffer.length);

    return buffer;
  }

  /// Generate the new destination file from the source file, with the
  /// [blocks] and [binary] stream given.
  Future<bool> rebuildFile(File file, List<FileDeltaBlock> delta, Stream<List<int>> binary) async {
    final RandomAccessFile fileView = await file.open();

    // Buffer used to hold the file content in memory.
    final BytesBuilder buffer = BytesBuilder(copy: false);

    final StreamIterator<List<int>> iterator = StreamIterator<List<int>>(binary);
    int currentIteratorStart = -1;

    bool iteratorMoveNextReturnValue = true;

    for (final FileDeltaBlock current in delta) {
      if (current.copyFromDestination) {
        await fileView.setPosition(current.start);
        buffer.add(await fileView.read(current.size));
      } else {
        int toRead = current.size;
        while (toRead > 0) {
          if (currentIteratorStart >= 0 && currentIteratorStart < iterator.current.length) {
            final int size = iterator.current.length - currentIteratorStart;
            final int sizeToRead = min(toRead, size);
            buffer.add(
              iterator.current.sublist(currentIteratorStart, currentIteratorStart + sizeToRead),
            );
            currentIteratorStart += sizeToRead;
            toRead -= sizeToRead;
          } else {
            currentIteratorStart = 0;
            iteratorMoveNextReturnValue = await iterator.moveNext();
          }
        }
      }
    }

    await file.writeAsBytes(buffer.takeBytes(), flush: true);

    // Drain the stream iterator if needed.
    while (iteratorMoveNextReturnValue) {
      iteratorMoveNextReturnValue = await iterator.moveNext();
    }

    return true;
  }

  Future<String> _md5OfFile(File file) async {
    final Md5Hash fileMd5Hash = Md5Hash();
    await file.openRead().forEach(
      (List<int> chunk) =>
          fileMd5Hash.addChunk(chunk is Uint8List ? chunk : Uint8List.fromList(chunk)),
    );
    return base64.encode(fileMd5Hash.finalize().buffer.asUint8List());
  }
}

/// Represents a single line of instruction on how to generate the target file.
@immutable
class FileDeltaBlock {
  const FileDeltaBlock.fromSource({required this.start, required this.size})
    : copyFromDestination = false;
  const FileDeltaBlock.fromDestination({required this.start, required this.size})
    : copyFromDestination = true;

  /// If true, this block should be read from the destination file.
  final bool copyFromDestination;

  /// The size of the current block.
  final int size;

  /// Byte offset in the destination file from which the block should be read.
  final int start;

  Map<String, Object> toJson() => <String, Object>{
    if (copyFromDestination) 'start': start,
    'size': size,
  };

  static List<FileDeltaBlock> fromJsonList(List<Map<String, Object?>> jsonList) {
    return jsonList.map((Map<String, Object?> json) {
      if (json.containsKey('start')) {
        return FileDeltaBlock.fromDestination(
          start: json['start']! as int,
          size: json['size']! as int,
        );
      } else {
        // The start position does not matter on the destination machine.
        return FileDeltaBlock.fromSource(start: 0, size: json['size']! as int);
      }
    }).toList();
  }

  @override
  bool operator ==(Object other) {
    if (other is! FileDeltaBlock) {
      return false;
    }
    return other.copyFromDestination == copyFromDestination &&
        other.size == size &&
        other.start == start;
  }

  @override
  int get hashCode => Object.hash(copyFromDestination, size, start);
}
