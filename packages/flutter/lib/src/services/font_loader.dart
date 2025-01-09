// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/foundation.dart';

export 'dart:typed_data' show ByteData;

/// A class that enables the dynamic loading of fonts at runtime.
///
/// The [FontLoader] class provides a builder pattern, where the caller builds
/// up the assets that make up a font family, then calls [load] to load the
/// entire font family into a running Flutter application.
class FontLoader {
  /// Creates a new [FontLoader] that will load font assets for the specified
  /// [family].
  ///
  /// The font family will not be available for use until [load] has been
  /// called.
  FontLoader(this.family) : _loaded = false, _fontFutures = <Future<Uint8List>>[];

  /// The font family being loaded.
  ///
  /// The family groups a series of related font assets, each of which defines
  /// how to render a specific [FontWeight] and [FontStyle] within the family.
  final String family;

  /// Registers a font asset to be loaded by this font loader.
  ///
  /// The [bytes] argument specifies the actual font asset bytes. Currently,
  /// only OpenType (OTF) and TrueType (TTF) fonts are supported.
  void addFont(Future<ByteData> bytes) {
    if (_loaded) {
      throw StateError('FontLoader is already loaded');
    }

    _fontFutures.add(
      bytes.then(
        (ByteData data) => Uint8List.view(data.buffer, data.offsetInBytes, data.lengthInBytes),
      ),
    );
  }

  /// Loads this font loader's font [family] and all of its associated assets
  /// into the Flutter engine, making the font available to the current
  /// application.
  ///
  /// This method should only be called once per font loader. Attempts to
  /// load fonts from the same loader more than once will cause a [StateError]
  /// to be thrown.
  ///
  /// The returned future will complete with an error if any of the font asset
  /// futures yield an error.
  Future<void> load() async {
    if (_loaded) {
      throw StateError('FontLoader is already loaded');
    }
    _loaded = true;

    final Iterable<Future<void>> loadFutures = _fontFutures.map(
      (Future<Uint8List> f) => f.then<void>((Uint8List list) => loadFont(list, family)),
    );
    await Future.wait(loadFutures.toList());
  }

  /// Hook called to load a font asset into the engine.
  ///
  /// Subclasses may override this to replace the default loading logic with
  /// custom logic (for example, to mock the underlying engine API in tests).
  @protected
  @visibleForTesting
  Future<void> loadFont(Uint8List list, String family) {
    return loadFontFromList(list, fontFamily: family);
  }

  bool _loaded;
  final List<Future<Uint8List>> _fontFutures;
}
