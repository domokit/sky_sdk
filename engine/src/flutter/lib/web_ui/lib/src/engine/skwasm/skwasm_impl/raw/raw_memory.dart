// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@DefaultAsset('skwasm')
library skwasm_impl;

import 'dart:convert';
import 'dart:ffi';
import 'dart:typed_data';

import 'package:ui/ui.dart' as ui;

final class Stack extends Opaque {}

typedef StackPointer = Pointer<Stack>;

/// Generic linear memory allocation
@Native<StackPointer Function(Size)>(symbol: '_emscripten_stack_alloc', isLeaf: true)
external StackPointer stackAlloc(int length);

@Native<StackPointer Function()>(symbol: 'emscripten_stack_get_current', isLeaf: true)
external StackPointer stackSave();

@Native<Void Function(StackPointer)>(symbol: '_emscripten_stack_restore', isLeaf: true)
external void stackRestore(StackPointer pointer);

class StackScope {
  Pointer<Int8> convertStringToNative(String string) {
    final Uint8List encoded = utf8.encode(string);
    final Pointer<Int8> pointer = allocInt8Array(encoded.length + 1);
    for (int i = 0; i < encoded.length; i++) {
      pointer[i] = encoded[i];
    }
    pointer[encoded.length] = 0;
    return pointer;
  }

  Pointer<Float> convertMatrix4toSkMatrix(List<double> matrix4) {
    final Pointer<Float> pointer = allocFloatArray(9);
    final int matrixLength = matrix4.length;

    double getVal(int index) {
      return (index < matrixLength) ? matrix4[index] : 0.0;
    }

    pointer[0] = getVal(0);
    pointer[1] = getVal(4);
    pointer[2] = getVal(12);

    pointer[3] = getVal(1);
    pointer[4] = getVal(5);
    pointer[5] = getVal(13);

    pointer[6] = getVal(3);
    pointer[7] = getVal(7);
    pointer[8] = getVal(15);

    return pointer;
  }

  Pointer<Float> convertMatrix44toNative(Float64List matrix4) {
    assert(matrix4.length == 16);
    final Pointer<Float> pointer = allocFloatArray(16);
    for (int i = 0; i < 16; i++) {
      pointer[i] = matrix4[i];
    }
    return pointer;
  }

  Float64List convertMatrix44FromNative(Pointer<Float> buffer) {
    final Float64List matrix = Float64List(16);
    for (int i = 0; i < 16; i++) {
      matrix[i] = buffer[i];
    }
    return matrix;
  }

  Pointer<Float> convertRectToNative(ui.Rect rect) {
    final Pointer<Float> pointer = allocFloatArray(4);
    pointer[0] = rect.left;
    pointer[1] = rect.top;
    pointer[2] = rect.right;
    pointer[3] = rect.bottom;
    return pointer;
  }

  Pointer<Float> convertRectsToNative(List<ui.Rect> rects) {
    final Pointer<Float> pointer = allocFloatArray(rects.length * 4);
    for (int i = 0; i < rects.length; i++) {
      final ui.Rect rect = rects[i];
      pointer[i * 4] = rect.left;
      pointer[i * 4 + 1] = rect.top;
      pointer[i * 4 + 2] = rect.right;
      pointer[i * 4 + 3] = rect.bottom;
    }
    return pointer;
  }

  ui.Rect convertRectFromNative(Pointer<Float> buffer) {
    return ui.Rect.fromLTRB(buffer[0], buffer[1], buffer[2], buffer[3]);
  }

  Pointer<Int32> convertIRectToNative(ui.Rect rect) {
    final Pointer<Int32> pointer = allocInt32Array(4);
    pointer[0] = rect.left.floor();
    pointer[1] = rect.top.floor();
    pointer[2] = rect.right.ceil();
    pointer[3] = rect.bottom.ceil();
    return pointer;
  }

  ui.Rect convertIRectFromNative(Pointer<Int32> buffer) {
    return ui.Rect.fromLTRB(
      buffer[0].toDouble(),
      buffer[1].toDouble(),
      buffer[2].toDouble(),
      buffer[3].toDouble(),
    );
  }

  Pointer<Float> convertRRectToNative(ui.RRect rect) {
    final Pointer<Float> pointer = allocFloatArray(12);
    pointer[0] = rect.left;
    pointer[1] = rect.top;
    pointer[2] = rect.right;
    pointer[3] = rect.bottom;

    pointer[4] = rect.tlRadiusX;
    pointer[5] = rect.tlRadiusY;
    pointer[6] = rect.trRadiusX;
    pointer[7] = rect.trRadiusY;

    pointer[8] = rect.brRadiusX;
    pointer[9] = rect.brRadiusY;
    pointer[10] = rect.blRadiusX;
    pointer[11] = rect.blRadiusY;

    return pointer;
  }

  Pointer<Float> convertRSTransformsToNative(List<ui.RSTransform> transforms) {
    final Pointer<Float> pointer = allocFloatArray(transforms.length * 4);
    for (int i = 0; i < transforms.length; i++) {
      final ui.RSTransform transform = transforms[i];
      pointer[i * 4] = transform.scos;
      pointer[i * 4 + 1] = transform.ssin;
      pointer[i * 4 + 2] = transform.tx;
      pointer[i * 4 + 3] = transform.ty;
    }
    return pointer;
  }

  Pointer<Float> convertDoublesToNative(List<double> values) {
    final Pointer<Float> pointer = allocFloatArray(values.length);
    for (int i = 0; i < values.length; i++) {
      pointer[i] = values[i];
    }
    return pointer;
  }

  Pointer<Uint16> convertIntsToUint16Native(List<int> values) {
    final Pointer<Uint16> pointer = allocUint16Array(values.length);
    for (int i = 0; i < values.length; i++) {
      pointer[i] = values[i];
    }
    return pointer;
  }

  Pointer<Uint32> convertIntsToUint32Native(List<int> values) {
    final Pointer<Uint32> pointer = allocUint32Array(values.length);
    for (int i = 0; i < values.length; i++) {
      pointer[i] = values[i];
    }
    return pointer;
  }

  Pointer<Float> convertPointArrayToNative(List<ui.Offset> points) {
    final Pointer<Float> pointer = allocFloatArray(points.length * 2);
    for (int i = 0; i < points.length; i++) {
      pointer[i * 2] = points[i].dx;
      pointer[i * 2 + 1] = points[i].dy;
    }
    return pointer;
  }

  Pointer<Uint32> convertColorArrayToNative(List<ui.Color> colors) {
    final Pointer<Uint32> pointer = allocUint32Array(colors.length);
    for (int i = 0; i < colors.length; i++) {
      pointer[i] = colors[i].value;
    }
    return pointer;
  }

  Pointer<Bool> allocBoolArray(int count) {
    final int length = count * sizeOf<Bool>();
    return stackAlloc(length).cast<Bool>();
  }

  Pointer<Int8> allocInt8Array(int count) {
    final int length = count * sizeOf<Int8>();
    return stackAlloc(length).cast<Int8>();
  }

  Pointer<Uint16> allocUint16Array(int count) {
    final int length = count * sizeOf<Uint16>();
    return stackAlloc(length).cast<Uint16>();
  }

  Pointer<Int32> allocInt32Array(int count) {
    final int length = count * sizeOf<Int32>();
    return stackAlloc(length).cast<Int32>();
  }

  Pointer<Uint32> allocUint32Array(int count) {
    final int length = count * sizeOf<Uint32>();
    return stackAlloc(length).cast<Uint32>();
  }

  Pointer<Float> allocFloatArray(int count) {
    final int length = count * sizeOf<Float>();
    return stackAlloc(length).cast<Float>();
  }

  Pointer<Pointer<Void>> allocPointerArray(int count) {
    final int length = count * sizeOf<Pointer<Void>>();
    return stackAlloc(length).cast<Pointer<Void>>();
  }
}

T withStackScope<T>(T Function(StackScope scope) f) {
  final StackPointer stack = stackSave();
  final T result = f(StackScope());
  stackRestore(stack);
  return result;
}
