// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:_wasm';
import 'dart:async';
import 'dart:ffi';
import 'dart:js_interop';
import 'dart:typed_data';

import 'package:ui/src/engine.dart';
import 'package:ui/src/engine/skwasm/skwasm_impl.dart';
import 'package:ui/ui.dart' as ui;

@JS()
@staticInterop
@anonymous
class RasterResult {}

extension RasterResultExtension on RasterResult {
  external JSNumber get rasterStartMilliseconds;
  external JSNumber get rasterEndMilliseconds;
  external JSArray<JSAny> get imageBitmaps;
}

@pragma('wasm:export')
WasmVoid callbackHandler(WasmI32 callbackId, WasmI32 context, WasmExternRef? jsContext) {
  // Actually hide this call behind whether skwasm is enabled. Otherwise, the SkwasmCallbackHandler
  // won't actually be tree-shaken, and we end up with skwasm imports in non-skwasm builds.
  if (FlutterConfiguration.flutterWebUseSkwasm) {
    SkwasmCallbackHandler.instance.handleCallback(callbackId, context, jsContext);
  }
  return WasmVoid();
}

// This class handles callbacks coming from Skwasm by keeping a map of callback IDs to Completers
class SkwasmCallbackHandler {
  SkwasmCallbackHandler._withCallbackPointer(this.callbackPointer);

  factory SkwasmCallbackHandler._() {
    final WasmFuncRef wasmFunction =
        WasmFunction<WasmVoid Function(WasmI32, WasmI32, WasmExternRef?)>.fromFunction(
          callbackHandler,
        );
    final int functionIndex = addFunction(wasmFunction).toIntUnsigned();
    return SkwasmCallbackHandler._withCallbackPointer(
      OnRenderCallbackHandle.fromAddress(functionIndex),
    );
  }
  static SkwasmCallbackHandler instance = SkwasmCallbackHandler._();

  final OnRenderCallbackHandle callbackPointer;
  final Map<CallbackId, Completer<JSAny>> _pendingCallbacks = <int, Completer<JSAny>>{};

  // Returns a future that will resolve when Skwasm calls back with the given callbackID
  Future<JSAny> registerCallback(int callbackId) {
    final Completer<JSAny> completer = Completer<JSAny>();
    _pendingCallbacks[callbackId] = completer;
    return completer.future;
  }

  void handleCallback(WasmI32 callbackId, WasmI32 context, WasmExternRef? jsContext) {
    // Skwasm can either callback with a JS object (an externref) or it can call back
    // with a simple integer, which usually refers to a pointer on its heap. In order
    // to coerce these into a single type, we just make the completers take a JSAny
    // that either contains the JS object or a JSNumber that contains the integer value.
    final Completer<JSAny> completer = _pendingCallbacks.remove(callbackId.toIntUnsigned())!;
    if (!jsContext.isNull) {
      completer.complete(jsContext!.toJS);
    } else {
      completer.complete(context.toIntUnsigned().toJS);
    }
  }
}

class SkwasmSurface {
  factory SkwasmSurface() {
    final SurfaceHandle surfaceHandle = withStackScope((StackScope scope) {
      return surfaceCreate();
    });
    final SkwasmSurface surface = SkwasmSurface._fromHandle(surfaceHandle);
    surface._initialize();
    return surface;
  }

  SkwasmSurface._fromHandle(this.handle) : threadId = surfaceGetThreadId(handle);
  final SurfaceHandle handle;

  final int threadId;

  void _initialize() {
    surfaceSetCallbackHandler(handle, SkwasmCallbackHandler.instance.callbackPointer);
  }

  Future<RenderResult> renderPictures(List<SkwasmPicture> pictures) =>
      withStackScope((StackScope scope) async {
        final Pointer<PictureHandle> pictureHandles =
            scope.allocPointerArray(pictures.length).cast<PictureHandle>();
        for (int i = 0; i < pictures.length; i++) {
          pictureHandles[i] = pictures[i].handle;
        }
        final int callbackId = surfaceRenderPictures(handle, pictureHandles, pictures.length);
        final RasterResult rasterResult =
            (await SkwasmCallbackHandler.instance.registerCallback(callbackId)) as RasterResult;
        final RenderResult result = (
          imageBitmaps: rasterResult.imageBitmaps.toDart.cast<DomImageBitmap>(),
          rasterStartMicros: (rasterResult.rasterStartMilliseconds.toDartDouble * 1000).toInt(),
          rasterEndMicros: (rasterResult.rasterEndMilliseconds.toDartDouble * 1000).toInt(),
        );
        return result;
      });

  Future<ByteData> rasterizeImage(SkwasmImage image, ui.ImageByteFormat format) async {
    final int callbackId = surfaceRasterizeImage(handle, image.handle, format.index);
    final int context =
        (await SkwasmCallbackHandler.instance.registerCallback(callbackId) as JSNumber).toDartInt;
    final SkDataHandle dataHandle = SkDataHandle.fromAddress(context);
    final int byteCount = skDataGetSize(dataHandle);
    final Pointer<Uint8> dataPointer = skDataGetConstPointer(dataHandle).cast<Uint8>();
    final Uint8List output = Uint8List(byteCount);
    for (int i = 0; i < byteCount; i++) {
      output[i] = dataPointer[i];
    }
    skDataDispose(dataHandle);
    return ByteData.sublistView(output);
  }

  void dispose() {
    surfaceDestroy(handle);
  }
}
