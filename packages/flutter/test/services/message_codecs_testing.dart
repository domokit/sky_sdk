// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:flutter/services.dart';
import 'package:flutter_test/flutter_test.dart';

void checkEncoding<T>(MessageCodec<T> codec, T message, List<int> expectedBytes) {
  final ByteData encoded = codec.encodeMessage(message)!;
  expect(encoded.buffer.asUint8List(0, encoded.lengthInBytes), orderedEquals(expectedBytes));
}

void checkEncodeDecode<T>(MessageCodec<T> codec, T message) {
  final ByteData? encoded = codec.encodeMessage(message);
  final T? decoded = codec.decodeMessage(encoded);
  if (message == null) {
    expect(encoded, isNull);
    expect(decoded, isNull);
  } else {
    expect(deepEquals(message, decoded), isTrue);
    final ByteData? encodedAgain = codec.encodeMessage(decoded as T);
    expect(encodedAgain!.buffer.asUint8List(), orderedEquals(encoded!.buffer.asUint8List()));
  }
}

bool deepEquals(dynamic valueA, dynamic valueB) {
  if (valueA is TypedData) {
    return valueB is TypedData && deepEqualsTypedData(valueA, valueB);
  }
  if (valueA is List) {
    return valueB is List && deepEqualsList(valueA, valueB);
  }
  if (valueA is Map) {
    return valueB is Map && deepEqualsMap(valueA, valueB);
  }
  if (valueA is double && valueA.isNaN) {
    return valueB is double && valueB.isNaN;
  }
  return valueA == valueB;
}

bool deepEqualsTypedData(TypedData valueA, TypedData valueB) {
  if (valueA is ByteData) {
    return valueB is ByteData &&
        deepEqualsList(valueA.buffer.asUint8List(), valueB.buffer.asUint8List());
  }

  return switch (valueA) {
    Uint8List() => valueB is Uint8List && deepEqualsList(valueA, valueB),
    Int32List() => valueB is Int32List && deepEqualsList(valueA, valueB),
    Int64List() => valueB is Int64List && deepEqualsList(valueA, valueB),
    Float32List() => valueB is Float32List && deepEqualsList(valueA, valueB),
    Float64List() => valueB is Float64List && deepEqualsList(valueA, valueB),
    _ => throw 'Unexpected typed data: $valueA',
  };
}

bool deepEqualsList(List<dynamic> valueA, List<dynamic> valueB) {
  if (valueA.length != valueB.length) {
    return false;
  }
  for (int i = 0; i < valueA.length; i++) {
    if (!deepEquals(valueA[i], valueB[i])) {
      return false;
    }
  }
  return true;
}

bool deepEqualsMap(Map<dynamic, dynamic> valueA, Map<dynamic, dynamic> valueB) {
  if (valueA.length != valueB.length) {
    return false;
  }
  for (final dynamic key in valueA.keys) {
    if (!valueB.containsKey(key) || !deepEquals(valueA[key], valueB[key])) {
      return false;
    }
  }
  return true;
}
