// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

part of zircon;

@pragma('vm:entry-point')
class ZDChannel {
  static ZDChannel? create([int options = 0]) {
    final Pointer<zircon_dart_handle_pair_t>? channelPtr = zirconFFIBindings
        ?.zircon_dart_channel_create(options);
    if (channelPtr == null || channelPtr.address == 0) {
      throw Exception('Unable to create a channel');
    }
    return ZDChannel._(ZDHandlePair._(channelPtr));
  }

  static int _write(ZDHandle channel, ByteData data, List<ZDHandle> handles) {
    final Pointer<zircon_dart_handle_list_t> handleList =
        zirconFFIBindings!.zircon_dart_handle_list_create();
    handles.forEach((ZDHandle handle) {
      zirconFFIBindings!.zircon_dart_handle_list_append(handleList, handle._ptr);
    });

    final Uint8List dataAsBytes = data.buffer.asUint8List();
    final Pointer<zircon_dart_byte_array_t> byteArray = zirconFFIBindings!
        .zircon_dart_byte_array_create(dataAsBytes.length);
    for (int i = 0; i < dataAsBytes.length; i++) {
      zirconFFIBindings!.zircon_dart_byte_array_set_value(byteArray, i, dataAsBytes.elementAt(i));
    }
    int ret = zirconFFIBindings!.zircon_dart_channel_write(channel._ptr, byteArray, handleList);

    zirconFFIBindings!.zircon_dart_byte_array_free(byteArray);
    zirconFFIBindings!.zircon_dart_handle_list_free(handleList);
    return ret;
  }

  int writeLeft(ByteData data, List<ZDHandle> handles) {
    return _write(handlePair.left, data, handles);
  }

  int writeRight(ByteData data, List<ZDHandle> handles) {
    return _write(handlePair.right, data, handles);
  }

  @pragma('vm:entry-point')
  ZDChannel._(this.handlePair);

  final ZDHandlePair handlePair;

  @override
  String toString() => 'Channel(handlePair=$handlePair)';
}
