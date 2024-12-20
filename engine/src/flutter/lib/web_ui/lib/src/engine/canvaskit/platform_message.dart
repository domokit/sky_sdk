// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

// TODO(hterkelsen): Delete this once the slots change lands?
class PlatformMessage {
  PlatformMessage(this.channel, this.data, this.response);

  final String channel;
  final ByteData data;
  final PlatformMessageResponse response;
}

class PlatformMessageResponse {
  void complete(Uint8List data) {}
  void completeEmpty() {}
}
