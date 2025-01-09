// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

/// A class representing rich content (such as a PNG image) inserted via the
/// system input method.
///
/// The following data is represented in this class:
///  - MIME Type
///  - Bytes
///  - URI
@immutable
class KeyboardInsertedContent {
  /// Creates an object to represent content that is inserted from the virtual
  /// keyboard.
  ///
  /// The mime type and URI will always be provided, but the bytedata may be null.
  const KeyboardInsertedContent({required this.mimeType, required this.uri, this.data});

  /// Converts JSON received from the Flutter Engine into the Dart class.
  KeyboardInsertedContent.fromJson(Map<String, dynamic> metadata)
    : mimeType = metadata['mimeType'] as String,
      uri = metadata['uri'] as String,
      data =
          metadata['data'] != null
              ? Uint8List.fromList(List<int>.from(metadata['data'] as Iterable<dynamic>))
              : null;

  /// The mime type of the inserted content.
  final String mimeType;

  /// The URI (location) of the inserted content, usually a "content://" URI.
  final String uri;

  /// The bytedata of the inserted content.
  final Uint8List? data;

  /// Convenience getter to check if bytedata is available for the inserted content.
  bool get hasData => data?.isNotEmpty ?? false;

  @override
  String toString() =>
      '${objectRuntimeType(this, 'KeyboardInsertedContent')}($mimeType, $uri, $data)';

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is KeyboardInsertedContent &&
        other.mimeType == mimeType &&
        other.uri == uri &&
        other.data == data;
  }

  @override
  int get hashCode => Object.hash(mimeType, uri, data);
}
