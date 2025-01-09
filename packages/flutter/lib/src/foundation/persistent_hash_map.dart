// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// A collection of key/value pairs which provides efficient retrieval of
/// value by key.
///
/// This class implements a persistent map: extending this map with a new
/// key/value pair does not modify an existing instance but instead creates a
/// new instance.
///
/// Unlike [Map], this class does not support `null` as a key value and
/// implements only a functionality needed for a specific use case at the
/// core of the framework.
///
/// Underlying implementation uses a variation of *hash array mapped trie*
/// data structure with compressed (bitmap indexed) nodes.
///
/// See also:
///
///  * [Bagwell, Phil. Ideal hash trees.](https://infoscience.epfl.ch/record/64398);
///  * [Steindorfer, Michael J., and Jurgen J. Vinju. "Optimizing hash-array mapped tries for fast and lean immutable JVM collections."](https://dl.acm.org/doi/abs/10.1145/2814270.2814312);
///  * [Clojure's `PersistentHashMap`](https://github.com/clojure/clojure/blob/master/src/jvm/clojure/lang/PersistentHashMap.java).
///
class PersistentHashMap<K extends Object, V> {
  /// Creates an empty hash map.
  const PersistentHashMap.empty() : this._(null);

  const PersistentHashMap._(this._root);

  final _TrieNode? _root;

  /// If this map does not already contain the given [key] to [value]
  /// mapping then create a new version of the map which contains
  /// all mappings from the current one plus the given [key] to [value]
  /// mapping.
  PersistentHashMap<K, V> put(K key, V value) {
    final _TrieNode newRoot = (_root ?? _CompressedNode.empty).put(0, key, key.hashCode, value);
    if (newRoot == _root) {
      return this;
    }
    return PersistentHashMap<K, V>._(newRoot);
  }

  /// Returns value associated with the given [key] or `null` if [key]
  /// is not in the map.
  @pragma('dart2js:as:trust')
  V? operator [](K key) {
    // Unfortunately can not use unsafeCast<V?>(...) here because it leads
    // to worse code generation on VM.
    return _root?.get(0, key, key.hashCode) as V?;
  }
}

/// Base class for nodes in a hash trie.
///
/// This trie is keyed by hash code bits using [hashBitsPerLevel] bits
/// at each level.
abstract class _TrieNode {
  static const int hashBitsPerLevel = 5;
  static const int hashBitsPerLevelMask = (1 << hashBitsPerLevel) - 1;

  @pragma('dart2js:tryInline')
  @pragma('vm:prefer-inline')
  @pragma('wasm:prefer-inline')
  static int trieIndex(int hash, int bitIndex) {
    return (hash >>> bitIndex) & hashBitsPerLevelMask;
  }

  /// Insert [key] to [value] mapping into the trie using bits from [keyHash]
  /// starting at [bitIndex].
  _TrieNode put(int bitIndex, Object key, int keyHash, Object? value);

  /// Lookup a value associated with the given [key] using bits from [keyHash]
  /// starting at [bitIndex].
  Object? get(int bitIndex, Object key, int keyHash);
}

/// A full (uncompressed) node in the trie.
///
/// It contains an array with `1<<_hashBitsPerLevel` elements which
/// are references to deeper nodes.
class _FullNode extends _TrieNode {
  _FullNode(this.descendants);

  static const int numElements = 1 << _TrieNode.hashBitsPerLevel;

  // Caveat: this array is actually List<_TrieNode?> but typing it like that
  // will introduce a type check when copying this array. For performance
  // reasons we instead omit the type and use (implicit) casts when accessing
  // it instead.
  final List<Object?> descendants;

  @override
  _TrieNode put(int bitIndex, Object key, int keyHash, Object? value) {
    final int index = _TrieNode.trieIndex(keyHash, bitIndex);
    final _TrieNode node = _unsafeCast<_TrieNode?>(descendants[index]) ?? _CompressedNode.empty;
    final _TrieNode newNode = node.put(bitIndex + _TrieNode.hashBitsPerLevel, key, keyHash, value);
    return identical(newNode, node) ? this : _FullNode(_copy(descendants)..[index] = newNode);
  }

  @override
  Object? get(int bitIndex, Object key, int keyHash) {
    final int index = _TrieNode.trieIndex(keyHash, bitIndex);

    final _TrieNode? node = _unsafeCast<_TrieNode?>(descendants[index]);
    return node?.get(bitIndex + _TrieNode.hashBitsPerLevel, key, keyHash);
  }
}

/// Compressed node in the trie.
///
/// Instead of storing the full array of outgoing edges this node uses a
/// compressed representation:
///
///   * [_CompressedNode.occupiedIndices] has a bit set for indices which are
///     occupied.
///   * furthermore, each occupied index can either be a `(key, value)` pair
///     representing an actual key/value mapping or a `(null, trieNode)` pair
///     representing a descendant node.
///
/// Keys and values are stored together in a single array (instead of two
/// parallel arrays) for performance reasons: this improves memory access
/// locality and reduces memory usage (two arrays of length N take slightly
/// more space than one array of length 2*N).
class _CompressedNode extends _TrieNode {
  _CompressedNode(this.occupiedIndices, this.keyValuePairs);
  _CompressedNode._empty() : this(0, _emptyArray);

  factory _CompressedNode.single(int bitIndex, int keyHash, _TrieNode node) {
    final int bit = 1 << _TrieNode.trieIndex(keyHash, bitIndex);
    // A single (null, node) pair.
    final List<Object?> keyValuePairs = _makeArray(2)..[1] = node;
    return _CompressedNode(bit, keyValuePairs);
  }

  static final _CompressedNode empty = _CompressedNode._empty();

  // Caveat: do not replace with <Object?>[] or const <Object?>[] this will
  // introduce polymorphism in the keyValuePairs field and significantly
  // degrade performance on the VM because it will no longer be able to
  // devirtualize method calls on keyValuePairs.
  static final List<Object?> _emptyArray = _makeArray(0);

  // This bitmap only uses 32bits due to [_TrieNode.hashBitsPerLevel] being `5`.
  final int occupiedIndices;
  final List<Object?> keyValuePairs;

  @override
  _TrieNode put(int bitIndex, Object key, int keyHash, Object? value) {
    final int bit = 1 << _TrieNode.trieIndex(keyHash, bitIndex);
    final int index = _compressedIndex(bit);

    if ((occupiedIndices & bit) != 0) {
      // Index is occupied.
      final Object? keyOrNull = keyValuePairs[2 * index];
      final Object? valueOrNode = keyValuePairs[2 * index + 1];

      // Is this a (null, trieNode) pair?
      if (identical(keyOrNull, null)) {
        final _TrieNode newNode = _unsafeCast<_TrieNode>(
          valueOrNode,
        ).put(bitIndex + _TrieNode.hashBitsPerLevel, key, keyHash, value);
        if (newNode == valueOrNode) {
          return this;
        }
        return _CompressedNode(occupiedIndices, _copy(keyValuePairs)..[2 * index + 1] = newNode);
      }

      if (key == keyOrNull) {
        // Found key/value pair with a matching key. If values match
        // then avoid doing anything otherwise copy and update.
        return identical(value, valueOrNode)
            ? this
            : _CompressedNode(occupiedIndices, _copy(keyValuePairs)..[2 * index + 1] = value);
      }

      // Two different keys at the same index, resolve collision.
      final _TrieNode newNode = _resolveCollision(
        bitIndex + _TrieNode.hashBitsPerLevel,
        keyOrNull,
        valueOrNode,
        key,
        keyHash,
        value,
      );
      return _CompressedNode(
        occupiedIndices,
        _copy(keyValuePairs)
          ..[2 * index] = null
          ..[2 * index + 1] = newNode,
      );
    } else {
      // Adding new key/value mapping.
      final int occupiedCount = _bitCount(occupiedIndices);
      if (occupiedCount >= 16) {
        // Too many occupied: inflate compressed node into full node and
        // update descendant at the corresponding index.
        return _inflate(bitIndex)
          ..descendants[_TrieNode.trieIndex(keyHash, bitIndex)] = _CompressedNode.empty.put(
            bitIndex + _TrieNode.hashBitsPerLevel,
            key,
            keyHash,
            value,
          );
      } else {
        // Grow keyValuePairs by inserting key/value pair at the given
        // index.
        final int prefixLength = 2 * index;
        final int totalLength = 2 * occupiedCount;
        final List<Object?> newKeyValuePairs = _makeArray(totalLength + 2);
        for (int srcIndex = 0; srcIndex < prefixLength; srcIndex++) {
          newKeyValuePairs[srcIndex] = keyValuePairs[srcIndex];
        }
        newKeyValuePairs[prefixLength] = key;
        newKeyValuePairs[prefixLength + 1] = value;
        for (
          int srcIndex = prefixLength, dstIndex = prefixLength + 2;
          srcIndex < totalLength;
          srcIndex++, dstIndex++
        ) {
          newKeyValuePairs[dstIndex] = keyValuePairs[srcIndex];
        }
        return _CompressedNode(occupiedIndices | bit, newKeyValuePairs);
      }
    }
  }

  @override
  Object? get(int bitIndex, Object key, int keyHash) {
    final int bit = 1 << _TrieNode.trieIndex(keyHash, bitIndex);
    if ((occupiedIndices & bit) == 0) {
      return null;
    }
    final int index = _compressedIndex(bit);
    final Object? keyOrNull = keyValuePairs[2 * index];
    final Object? valueOrNode = keyValuePairs[2 * index + 1];
    if (keyOrNull == null) {
      final _TrieNode node = _unsafeCast<_TrieNode>(valueOrNode);
      return node.get(bitIndex + _TrieNode.hashBitsPerLevel, key, keyHash);
    }
    if (key == keyOrNull) {
      return valueOrNode;
    }
    return null;
  }

  /// Convert this node into an equivalent [_FullNode].
  _FullNode _inflate(int bitIndex) {
    final List<Object?> nodes = _makeArray(_FullNode.numElements);
    int srcIndex = 0;
    for (int dstIndex = 0; dstIndex < _FullNode.numElements; dstIndex++) {
      if (((occupiedIndices >>> dstIndex) & 1) != 0) {
        final Object? keyOrNull = keyValuePairs[srcIndex];
        if (keyOrNull == null) {
          nodes[dstIndex] = keyValuePairs[srcIndex + 1];
        } else {
          nodes[dstIndex] = _CompressedNode.empty.put(
            bitIndex + _TrieNode.hashBitsPerLevel,
            keyOrNull,
            keyValuePairs[srcIndex].hashCode,
            keyValuePairs[srcIndex + 1],
          );
        }
        srcIndex += 2;
      }
    }
    return _FullNode(nodes);
  }

  @pragma('dart2js:tryInline')
  @pragma('vm:prefer-inline')
  @pragma('wasm:prefer-inline')
  int _compressedIndex(int bit) {
    return _bitCount(occupiedIndices & (bit - 1));
  }

  static _TrieNode _resolveCollision(
    int bitIndex,
    Object existingKey,
    Object? existingValue,
    Object key,
    int keyHash,
    Object? value,
  ) {
    final int existingKeyHash = existingKey.hashCode;
    // Check if this is a full hash collision and use _HashCollisionNode
    // in this case.
    return (existingKeyHash == keyHash)
        ? _HashCollisionNode.fromCollision(keyHash, existingKey, existingValue, key, value)
        : _CompressedNode.empty
            .put(bitIndex, existingKey, existingKeyHash, existingValue)
            .put(bitIndex, key, keyHash, value);
  }
}

/// Trie node representing a full hash collision.
///
/// Stores a list of key/value pairs (where all keys have the same hash code).
class _HashCollisionNode extends _TrieNode {
  _HashCollisionNode(this.hash, this.keyValuePairs);

  factory _HashCollisionNode.fromCollision(
    int keyHash,
    Object keyA,
    Object? valueA,
    Object keyB,
    Object? valueB,
  ) {
    final List<Object?> list = _makeArray(4);
    list[0] = keyA;
    list[1] = valueA;
    list[2] = keyB;
    list[3] = valueB;
    return _HashCollisionNode(keyHash, list);
  }

  final int hash;
  final List<Object?> keyValuePairs;

  @override
  _TrieNode put(int bitIndex, Object key, int keyHash, Object? val) {
    // Is this another full hash collision?
    if (keyHash == hash) {
      final int index = _indexOf(key);
      if (index != -1) {
        return identical(keyValuePairs[index + 1], val)
            ? this
            : _HashCollisionNode(keyHash, _copy(keyValuePairs)..[index + 1] = val);
      }
      final int length = keyValuePairs.length;
      final List<Object?> newArray = _makeArray(length + 2);
      for (int i = 0; i < length; i++) {
        newArray[i] = keyValuePairs[i];
      }
      newArray[length] = key;
      newArray[length + 1] = val;
      return _HashCollisionNode(keyHash, newArray);
    }

    // Not a full hash collision, need to introduce a _CompressedNode which
    // uses previously unused bits.
    return _CompressedNode.single(bitIndex, hash, this).put(bitIndex, key, keyHash, val);
  }

  @override
  Object? get(int bitIndex, Object key, int keyHash) {
    final int index = _indexOf(key);
    return index < 0 ? null : keyValuePairs[index + 1];
  }

  int _indexOf(Object key) {
    final int length = keyValuePairs.length;
    for (int i = 0; i < length; i += 2) {
      if (key == keyValuePairs[i]) {
        return i;
      }
    }
    return -1;
  }
}

/// Returns number of bits set in a 32bit integer.
///
/// dart2js safe because we work with 32bit integers.
@pragma('dart2js:tryInline')
@pragma('vm:prefer-inline')
@pragma('wasm:prefer-inline')
int _bitCount(int n) {
  assert((n & 0xFFFFFFFF) == n);
  n = n - ((n >> 1) & 0x55555555);
  n = (n & 0x33333333) + ((n >>> 2) & 0x33333333);
  n = (n + (n >> 4)) & 0x0F0F0F0F;
  n = n + (n >> 8);
  n = n + (n >> 16);
  return n & 0x0000003F;
}

/// Create a copy of the given array.
///
/// Caveat: do not replace with List.of or similar methods. They are
/// considerably slower.
@pragma('dart2js:tryInline')
@pragma('vm:prefer-inline')
@pragma('wasm:prefer-inline')
List<Object?> _copy(List<Object?> array) {
  final List<Object?> clone = _makeArray(array.length);
  for (int j = 0; j < array.length; j++) {
    clone[j] = array[j];
  }
  return clone;
}

/// Create a fixed-length array of the given length filled with `null`.
///
/// We are using fixed length arrays because they are smaller and
/// faster to access on VM. Growable arrays are represented by 2 objects
/// (growable array instance pointing to a fixed array instance) and
/// consequently fixed length arrays are faster to allocated, require less
/// memory and are faster to access (less indirections).
@pragma('dart2js:tryInline')
@pragma('vm:prefer-inline')
@pragma('wasm:prefer-inline')
List<Object?> _makeArray(int length) {
  return List<Object?>.filled(length, null);
}

/// This helper method becomes an no-op when compiled with dart2js on
/// with high level of optimizations enabled.
@pragma('dart2js:as:trust')
@pragma('dart2js:tryInline')
@pragma('vm:prefer-inline')
@pragma('wasm:prefer-inline')
T _unsafeCast<T>(Object? o) {
  return o as T;
}
