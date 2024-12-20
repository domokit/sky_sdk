// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:ui/ui.dart' as ui;

import '../validators.dart';
import 'canvaskit_api.dart';
import 'image.dart';
import 'native_memory.dart';

/// Refines the generic [ui.Shader] interface with CanvasKit-specific features.
abstract class CkShader implements ui.Shader {
  /// Returns a Skia shader at requested filter quality, if that shader supports
  /// altering its filter quality.
  ///
  /// If the implementation supports changing filter quality, and the value of
  /// [contextualQuality] is the same as a previously passed value, the
  /// implementation may return the same object as before. If a new Skia object
  /// is created, the previous object is released and cannot be used again. For
  /// this reason, do not store the returned value long-term to prevent dangling
  /// pointer errors.
  SkShader getSkShader(ui.FilterQuality contextualQuality);
}

/// Base class for shader implementations with a simple memory model that do not
/// support contextual filter quality.
///
/// Provides common memory mangement logic for shaders that map one-to-one to a
/// [SkShader] object. The lifetime of this shader is hard-linked to the
/// lifetime of the [SkShader]. [getSkShader] always returns the one and only
/// [SkShader] object, ignoring contextual filter quality.
abstract class SimpleCkShader implements CkShader {
  SimpleCkShader() {
    _ref = UniqueRef<SkShader>(this, createSkiaObject(), debugOwnerLabel);
  }

  late final UniqueRef<SkShader> _ref;

  @override
  SkShader getSkShader(ui.FilterQuality contextualQuality) => _ref.nativeObject;

  String get debugOwnerLabel;
  SkShader createSkiaObject();

  @override
  bool get debugDisposed => _ref.isDisposed;

  @override
  void dispose() {
    _ref.dispose();
  }

  @override
  String toString() => 'Gradient()';
}

class CkGradientSweep extends SimpleCkShader implements ui.Gradient {
  CkGradientSweep(
    this.center,
    this.colors,
    this.colorStops,
    this.tileMode,
    this.startAngle,
    this.endAngle,
    this.matrix4,
  ) : assert(offsetIsValid(center)),
      assert(startAngle < endAngle),
      assert(matrix4 == null || matrix4IsValid(matrix4)) {
    validateColorStops(colors, colorStops);
  }

  @override
  String get debugOwnerLabel => 'Gradient.sweep';

  final ui.Offset center;
  final List<ui.Color> colors;
  final List<double>? colorStops;
  final ui.TileMode tileMode;
  final double startAngle;
  final double endAngle;
  final Float32List? matrix4;

  @override
  SkShader createSkiaObject() {
    const double toDegrees = 180.0 / math.pi;
    return canvasKit.Shader.MakeSweepGradient(
      center.dx,
      center.dy,
      toFlatColors(colors),
      toSkColorStops(colorStops),
      toSkTileMode(tileMode),
      matrix4 != null ? toSkMatrixFromFloat32(matrix4!) : null,
      0,
      toDegrees * startAngle,
      toDegrees * endAngle,
    );
  }
}

class CkGradientLinear extends SimpleCkShader implements ui.Gradient {
  CkGradientLinear(
    this.from,
    this.to,
    this.colors,
    this.colorStops,
    this.tileMode,
    Float32List? matrix,
  ) : assert(offsetIsValid(from)),
      assert(offsetIsValid(to)),
      matrix4 = matrix {
    assert(matrix4 == null || matrix4IsValid(matrix4!));
    // ignore: prefer_asserts_in_initializer_lists
    assert(() {
      validateColorStops(colors, colorStops);
      return true;
    }());
  }

  final ui.Offset from;
  final ui.Offset to;
  final List<ui.Color> colors;
  final List<double>? colorStops;
  final ui.TileMode tileMode;
  final Float32List? matrix4;

  @override
  String get debugOwnerLabel => 'Gradient.linear';

  @override
  SkShader createSkiaObject() {
    return canvasKit.Shader.MakeLinearGradient(
      toSkPoint(from),
      toSkPoint(to),
      toFlatColors(colors),
      toSkColorStops(colorStops),
      toSkTileMode(tileMode),
      matrix4 != null ? toSkMatrixFromFloat32(matrix4!) : null,
    );
  }

  @override
  String toString() => 'Gradient()';
}

class CkGradientRadial extends SimpleCkShader implements ui.Gradient {
  CkGradientRadial(
    this.center,
    this.radius,
    this.colors,
    this.colorStops,
    this.tileMode,
    this.matrix4,
  );

  final ui.Offset center;
  final double radius;
  final List<ui.Color> colors;
  final List<double>? colorStops;
  final ui.TileMode tileMode;
  final Float32List? matrix4;

  @override
  String get debugOwnerLabel => 'Gradient.radial';

  @override
  SkShader createSkiaObject() {
    return canvasKit.Shader.MakeRadialGradient(
      toSkPoint(center),
      radius,
      toFlatColors(colors),
      toSkColorStops(colorStops),
      toSkTileMode(tileMode),
      matrix4 != null ? toSkMatrixFromFloat32(matrix4!) : null,
      0,
    );
  }

  @override
  String toString() => 'Gradient()';
}

class CkGradientConical extends SimpleCkShader implements ui.Gradient {
  CkGradientConical(
    this.focal,
    this.focalRadius,
    this.center,
    this.radius,
    this.colors,
    this.colorStops,
    this.tileMode,
    this.matrix4,
  );

  final ui.Offset focal;
  final double focalRadius;
  final ui.Offset center;
  final double radius;
  final List<ui.Color> colors;
  final List<double>? colorStops;
  final ui.TileMode tileMode;
  final Float32List? matrix4;

  @override
  String get debugOwnerLabel => 'Gradient.radial(conical)';

  @override
  SkShader createSkiaObject() {
    return canvasKit.Shader.MakeTwoPointConicalGradient(
      toSkPoint(focal),
      focalRadius,
      toSkPoint(center),
      radius,
      toFlatColors(colors),
      toSkColorStops(colorStops),
      toSkTileMode(tileMode),
      matrix4 != null ? toSkMatrixFromFloat32(matrix4!) : null,
      0,
    );
  }
}

/// Implements [ui.ImageShader] for CanvasKit.
///
/// The memory management model is different from other shaders (backed by
/// [SimpleCkShader]) in that this object is not one-to-one to its Skia
/// counterpart [SkShader]. During initialization a default [SkShader] is
/// created based on the [filterQuality] specified in the constructor. However,
/// when [withQuality] is called with a different [ui.FilterQuality] value the
/// previous [SkShader] is discarded and a new [SkShader] is created. Therefore,
/// over the lifetime of this object, multiple [SkShader] instances may be
/// generated depending on the _usage_ of this object in [ui.Paint] and other
/// scenarios that want a shader at different filter quality levels.
class CkImageShader implements ui.ImageShader, CkShader {
  CkImageShader(ui.Image image, this.tileModeX, this.tileModeY, this.matrix4, this.filterQuality)
    : _image = image as CkImage {
    _initializeSkImageShader(filterQuality ?? ui.FilterQuality.none);
  }

  final ui.TileMode tileModeX;
  final ui.TileMode tileModeY;
  final Float64List matrix4;
  final ui.FilterQuality? filterQuality;
  final CkImage _image;

  /// Owns the reference to the currently [SkShader].
  ///
  /// This reference changes when [withQuality] is called with different filter
  /// quality levels.
  @visibleForTesting
  UniqueRef<SkShader>? ref;

  /// The filter quality at which the latest [SkShader] was initialized.
  @visibleForTesting
  late ui.FilterQuality currentQuality;

  int get imageWidth => _image.width;

  int get imageHeight => _image.height;

  @override
  SkShader getSkShader(ui.FilterQuality contextualQuality) {
    assert(!debugDisposed, 'Cannot make a copy of a disposed ImageShader.');
    final ui.FilterQuality quality = filterQuality ?? contextualQuality;
    if (currentQuality != quality) {
      _initializeSkImageShader(quality);
    }
    return ref!.nativeObject;
  }

  void _initializeSkImageShader(ui.FilterQuality quality) {
    final SkShader skShader;
    if (quality == ui.FilterQuality.high) {
      skShader = _image.skImage.makeShaderCubic(
        toSkTileMode(tileModeX),
        toSkTileMode(tileModeY),
        1.0 / 3.0,
        1.0 / 3.0,
        toSkMatrixFromFloat64(matrix4),
      );
    } else {
      skShader = _image.skImage.makeShaderOptions(
        toSkTileMode(tileModeX),
        toSkTileMode(tileModeY),
        toSkFilterMode(quality),
        toSkMipmapMode(quality),
        toSkMatrixFromFloat64(matrix4),
      );
    }

    currentQuality = quality;
    ref?.dispose();
    ref = UniqueRef<SkShader>(this, skShader, 'ImageShader');
  }

  bool _isDisposed = false;

  @override
  bool get debugDisposed => _isDisposed;

  @override
  void dispose() {
    assert(!_isDisposed, 'Cannot dispose ImageShader more than once.');
    _isDisposed = true;
    _image.dispose();
    ref?.dispose();
    ref = null;
  }
}
