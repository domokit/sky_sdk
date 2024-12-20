// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/rendering.dart';
/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'box_decoration.dart';
/// @docImport 'image_resolution.dart';
library;

import 'dart:developer' as developer;
import 'dart:math' as math;
import 'dart:ui' as ui show FlutterView, Image;

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';

import 'alignment.dart';
import 'basic_types.dart';
import 'binding.dart';
import 'borders.dart';
import 'box_fit.dart';
import 'debug.dart';
import 'image_provider.dart';
import 'image_stream.dart';

/// How to paint any portions of a box not covered by an image.
enum ImageRepeat {
  /// Repeat the image in both the x and y directions until the box is filled.
  repeat,

  /// Repeat the image in the x direction until the box is filled horizontally.
  repeatX,

  /// Repeat the image in the y direction until the box is filled vertically.
  repeatY,

  /// Leave uncovered portions of the box transparent.
  noRepeat,
}

/// An image for a box decoration.
///
/// The image is painted using [paintImage], which describes the meanings of the
/// various fields on this class in more detail.
@immutable
class DecorationImage {
  /// Creates an image to show in a [BoxDecoration].
  const DecorationImage({
    required this.image,
    this.onError,
    this.colorFilter,
    this.fit,
    this.alignment = Alignment.center,
    this.centerSlice,
    this.repeat = ImageRepeat.noRepeat,
    this.matchTextDirection = false,
    this.scale = 1.0,
    this.opacity = 1.0,
    this.filterQuality = FilterQuality.medium,
    this.invertColors = false,
    this.isAntiAlias = false,
  });

  /// The image to be painted into the decoration.
  ///
  /// Typically this will be an [AssetImage] (for an image shipped with the
  /// application) or a [NetworkImage] (for an image obtained from the network).
  final ImageProvider image;

  /// An optional error callback for errors emitted when loading [image].
  final ImageErrorListener? onError;

  /// A color filter to apply to the image before painting it.
  final ColorFilter? colorFilter;

  /// How the image should be inscribed into the box.
  ///
  /// The default is [BoxFit.scaleDown] if [centerSlice] is null, and
  /// [BoxFit.fill] if [centerSlice] is not null.
  ///
  /// See the discussion at [paintImage] for more details.
  final BoxFit? fit;

  /// How to align the image within its bounds.
  ///
  /// The alignment aligns the given position in the image to the given position
  /// in the layout bounds. For example, an [Alignment] alignment of (-1.0,
  /// -1.0) aligns the image to the top-left corner of its layout bounds, while a
  /// [Alignment] alignment of (1.0, 1.0) aligns the bottom right of the
  /// image with the bottom right corner of its layout bounds. Similarly, an
  /// alignment of (0.0, 1.0) aligns the bottom middle of the image with the
  /// middle of the bottom edge of its layout bounds.
  ///
  /// To display a subpart of an image, consider using a [CustomPainter] and
  /// [Canvas.drawImageRect].
  ///
  /// If the [alignment] is [TextDirection]-dependent (i.e. if it is a
  /// [AlignmentDirectional]), then a [TextDirection] must be available
  /// when the image is painted.
  ///
  /// Defaults to [Alignment.center].
  ///
  /// See also:
  ///
  ///  * [Alignment], a class with convenient constants typically used to
  ///    specify an [AlignmentGeometry].
  ///  * [AlignmentDirectional], like [Alignment] for specifying alignments
  ///    relative to text direction.
  final AlignmentGeometry alignment;

  /// The center slice for a nine-patch image.
  ///
  /// The region of the image inside the center slice will be stretched both
  /// horizontally and vertically to fit the image into its destination. The
  /// region of the image above and below the center slice will be stretched
  /// only horizontally and the region of the image to the left and right of
  /// the center slice will be stretched only vertically.
  ///
  /// The stretching will be applied in order to make the image fit into the box
  /// specified by [fit]. When [centerSlice] is not null, [fit] defaults to
  /// [BoxFit.fill], which distorts the destination image size relative to the
  /// image's original aspect ratio. Values of [BoxFit] which do not distort the
  /// destination image size will result in [centerSlice] having no effect
  /// (since the nine regions of the image will be rendered with the same
  /// scaling, as if it wasn't specified).
  final Rect? centerSlice;

  /// How to paint any portions of the box that would not otherwise be covered
  /// by the image.
  final ImageRepeat repeat;

  /// Whether to paint the image in the direction of the [TextDirection].
  ///
  /// If this is true, then in [TextDirection.ltr] contexts, the image will be
  /// drawn with its origin in the top left (the "normal" painting direction for
  /// images); and in [TextDirection.rtl] contexts, the image will be drawn with
  /// a scaling factor of -1 in the horizontal direction so that the origin is
  /// in the top right.
  final bool matchTextDirection;

  /// Defines image pixels to be shown per logical pixels.
  ///
  /// By default the value of scale is 1.0. The scale for the image is
  /// calculated by multiplying [scale] with [scale] of the given [ImageProvider].
  final double scale;

  /// If non-null, the value is multiplied with the opacity of each image
  /// pixel before painting onto the canvas.
  ///
  /// This is more efficient than using [Opacity] or [FadeTransition] to
  /// change the opacity of an image.
  final double opacity;

  /// Used to set the filterQuality of the image.
  ///
  /// Defaults to [FilterQuality.medium].
  final FilterQuality filterQuality;

  /// Whether the colors of the image are inverted when drawn.
  ///
  /// Inverting the colors of an image applies a new color filter to the paint.
  /// If there is another specified color filter, the invert will be applied
  /// after it. This is primarily used for implementing smart invert on iOS.
  ///
  /// See also:
  ///
  ///  * [Paint.invertColors], for the dart:ui implementation.
  final bool invertColors;

  /// Whether to paint the image with anti-aliasing.
  ///
  /// Anti-aliasing alleviates the sawtooth artifact when the image is rotated.
  final bool isAntiAlias;

  /// Creates a [DecorationImagePainter] for this [DecorationImage].
  ///
  /// The `onChanged` argument will be called whenever the image needs to be
  /// repainted, e.g. because it is loading incrementally or because it is
  /// animated.
  DecorationImagePainter createPainter(VoidCallback onChanged) {
    return _DecorationImagePainter._(this, onChanged);
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is DecorationImage &&
        other.image == image &&
        other.colorFilter == colorFilter &&
        other.fit == fit &&
        other.alignment == alignment &&
        other.centerSlice == centerSlice &&
        other.repeat == repeat &&
        other.matchTextDirection == matchTextDirection &&
        other.scale == scale &&
        other.opacity == opacity &&
        other.filterQuality == filterQuality &&
        other.invertColors == invertColors &&
        other.isAntiAlias == isAntiAlias;
  }

  @override
  int get hashCode => Object.hash(
    image,
    colorFilter,
    fit,
    alignment,
    centerSlice,
    repeat,
    matchTextDirection,
    scale,
    opacity,
    filterQuality,
    invertColors,
    isAntiAlias,
  );

  @override
  String toString() {
    final List<String> properties = <String>[
      '$image',
      if (colorFilter != null) '$colorFilter',
      if (fit != null &&
          !(fit == BoxFit.fill && centerSlice != null) &&
          !(fit == BoxFit.scaleDown && centerSlice == null))
        '$fit',
      '$alignment',
      if (centerSlice != null) 'centerSlice: $centerSlice',
      if (repeat != ImageRepeat.noRepeat) '$repeat',
      if (matchTextDirection) 'match text direction',
      'scale ${scale.toStringAsFixed(1)}',
      'opacity ${opacity.toStringAsFixed(1)}',
      '$filterQuality',
      if (invertColors) 'invert colors',
      if (isAntiAlias) 'use anti-aliasing',
    ];
    return '${objectRuntimeType(this, 'DecorationImage')}(${properties.join(", ")})';
  }

  /// Linearly interpolates between two [DecorationImage]s.
  ///
  /// The `t` argument represents position on the timeline, with 0.0 meaning
  /// that the interpolation has not started, returning `a`, 1.0 meaning that
  /// the interpolation has finished, returning `b`, and values in between
  /// meaning that the interpolation is at the relevant point on the timeline
  /// between `a` and `this`. The interpolation can be extrapolated beyond 0.0
  /// and 1.0, so negative values and values greater than 1.0 are valid (and can
  /// easily be generated by curves such as [Curves.elasticInOut]).
  ///
  /// Values for `t` are usually obtained from an [Animation<double>], such as
  /// an [AnimationController].
  static DecorationImage? lerp(DecorationImage? a, DecorationImage? b, double t) {
    if (identical(a, b) || t == 0.0) {
      return a;
    }
    if (t == 1.0) {
      return b;
    }
    return _BlendedDecorationImage(a, b, t);
  }
}

/// The painter for a [DecorationImage].
///
/// To obtain a painter, call [DecorationImage.createPainter].
///
/// To paint, call [paint]. The `onChanged` callback passed to
/// [DecorationImage.createPainter] will be called if the image needs to paint
/// again (e.g. because it is animated or because it had not yet loaded the
/// first time the [paint] method was called).
///
/// This object should be disposed using the [dispose] method when it is no
/// longer needed.
abstract interface class DecorationImagePainter {
  /// Draw the image onto the given canvas.
  ///
  /// The image is drawn at the position and size given by the `rect` argument.
  ///
  /// The image is clipped to the given `clipPath`, if any.
  ///
  /// The `configuration` object is used to resolve the image (e.g. to pick
  /// resolution-specific assets), and to implement the
  /// [DecorationImage.matchTextDirection] feature.
  ///
  /// If the image needs to be painted again, e.g. because it is animated or
  /// because it had not yet been loaded the first time this method was called,
  /// then the `onChanged` callback passed to [DecorationImage.createPainter]
  /// will be called.
  ///
  /// The `blend` argument specifies the opacity that should be applied to the
  /// image due to this image being blended with another. The `blendMode`
  /// argument can be specified to override the [DecorationImagePainter]'s
  /// default [BlendMode] behavior. It is usually set to [BlendMode.srcOver] if
  /// this is the first or only image being blended, and [BlendMode.plus] if it
  /// is being blended with an image below.
  void paint(
    Canvas canvas,
    Rect rect,
    Path? clipPath,
    ImageConfiguration configuration, {
    double blend = 1.0,
    BlendMode blendMode = BlendMode.srcOver,
  });

  /// Releases the resources used by this painter.
  ///
  /// This should be called whenever the painter is no longer needed.
  ///
  /// After this method has been called, the object is no longer usable.
  void dispose();
}

class _DecorationImagePainter implements DecorationImagePainter {
  _DecorationImagePainter._(this._details, this._onChanged) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/painting.dart',
        className: '$_DecorationImagePainter',
        object: this,
      );
    }
  }

  final DecorationImage _details;
  final VoidCallback _onChanged;

  ImageStream? _imageStream;
  ImageInfo? _image;

  @override
  void paint(
    Canvas canvas,
    Rect rect,
    Path? clipPath,
    ImageConfiguration configuration, {
    double blend = 1.0,
    BlendMode blendMode = BlendMode.srcOver,
  }) {
    bool flipHorizontally = false;
    if (_details.matchTextDirection) {
      assert(() {
        // We check this first so that the assert will fire immediately, not just
        // when the image is ready.
        if (configuration.textDirection == null) {
          throw FlutterError.fromParts(<DiagnosticsNode>[
            ErrorSummary(
              'DecorationImage.matchTextDirection can only be used when a TextDirection is available.',
            ),
            ErrorDescription(
              'When DecorationImagePainter.paint() was called, there was no text direction provided '
              'in the ImageConfiguration object to match.',
            ),
            DiagnosticsProperty<DecorationImage>(
              'The DecorationImage was',
              _details,
              style: DiagnosticsTreeStyle.errorProperty,
            ),
            DiagnosticsProperty<ImageConfiguration>(
              'The ImageConfiguration was',
              configuration,
              style: DiagnosticsTreeStyle.errorProperty,
            ),
          ]);
        }
        return true;
      }());
      if (configuration.textDirection == TextDirection.rtl) {
        flipHorizontally = true;
      }
    }

    final ImageStream newImageStream = _details.image.resolve(configuration);
    if (newImageStream.key != _imageStream?.key) {
      final ImageStreamListener listener = ImageStreamListener(
        _handleImage,
        onError: _details.onError,
      );
      _imageStream?.removeListener(listener);
      _imageStream = newImageStream;
      _imageStream!.addListener(listener);
    }
    if (_image == null) {
      return;
    }

    if (clipPath != null) {
      canvas.save();
      canvas.clipPath(clipPath);
    }

    paintImage(
      canvas: canvas,
      rect: rect,
      image: _image!.image,
      debugImageLabel: _image!.debugLabel,
      scale: _details.scale * _image!.scale,
      colorFilter: _details.colorFilter,
      fit: _details.fit,
      alignment: _details.alignment.resolve(configuration.textDirection),
      centerSlice: _details.centerSlice,
      repeat: _details.repeat,
      flipHorizontally: flipHorizontally,
      opacity: _details.opacity * blend,
      filterQuality: _details.filterQuality,
      invertColors: _details.invertColors,
      isAntiAlias: _details.isAntiAlias,
      blendMode: blendMode,
    );

    if (clipPath != null) {
      canvas.restore();
    }
  }

  void _handleImage(ImageInfo value, bool synchronousCall) {
    if (_image == value) {
      return;
    }
    if (_image != null && _image!.isCloneOf(value)) {
      value.dispose();
      return;
    }
    _image?.dispose();
    _image = value;
    if (!synchronousCall) {
      _onChanged();
    }
  }

  @override
  void dispose() {
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    _imageStream?.removeListener(ImageStreamListener(_handleImage, onError: _details.onError));
    _image?.dispose();
    _image = null;
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, 'DecorationImagePainter')}(stream: $_imageStream, image: $_image) for $_details';
  }
}

/// Used by [paintImage] to report image sizes drawn at the end of the frame.
Map<String, ImageSizeInfo> _pendingImageSizeInfo = <String, ImageSizeInfo>{};

/// [ImageSizeInfo]s that were reported on the last frame.
///
/// Used to prevent duplicative reports from frame to frame.
Set<ImageSizeInfo> _lastFrameImageSizeInfo = <ImageSizeInfo>{};

/// Flushes inter-frame tracking of image size information from [paintImage].
///
/// Has no effect if asserts are disabled.
@visibleForTesting
void debugFlushLastFrameImageSizeInfo() {
  assert(() {
    _lastFrameImageSizeInfo = <ImageSizeInfo>{};
    return true;
  }());
}

/// Paints an image into the given rectangle on the canvas.
///
/// The arguments have the following meanings:
///
///  * `canvas`: The canvas onto which the image will be painted.
///
///  * `rect`: The region of the canvas into which the image will be painted.
///    The image might not fill the entire rectangle (e.g., depending on the
///    `fit`). If `rect` is empty, nothing is painted.
///
///  * `image`: The image to paint onto the canvas.
///
///  * `scale`: The number of image pixels for each logical pixel.
///
///  * `opacity`: The opacity to paint the image onto the canvas with.
///
///  * `colorFilter`: If non-null, the color filter to apply when painting the
///    image.
///
///  * `fit`: How the image should be inscribed into `rect`. If null, the
///    default behavior depends on `centerSlice`. If `centerSlice` is also null,
///    the default behavior is [BoxFit.scaleDown]. If `centerSlice` is
///    non-null, the default behavior is [BoxFit.fill]. See [BoxFit] for
///    details.
///
///  * `alignment`: How the destination rectangle defined by applying `fit` is
///    aligned within `rect`. For example, if `fit` is [BoxFit.contain] and
///    `alignment` is [Alignment.bottomRight], the image will be as large
///    as possible within `rect` and placed with its bottom right corner at the
///    bottom right corner of `rect`. Defaults to [Alignment.center].
///
///  * `centerSlice`: The image is drawn in nine portions described by splitting
///    the image by drawing two horizontal lines and two vertical lines, where
///    `centerSlice` describes the rectangle formed by the four points where
///    these four lines intersect each other. (This forms a 3-by-3 grid
///    of regions, the center region being described by `centerSlice`.)
///    The four regions in the corners are drawn, without scaling, in the four
///    corners of the destination rectangle defined by applying `fit`. The
///    remaining five regions are drawn by stretching them to fit such that they
///    exactly cover the destination rectangle while maintaining their relative
///    positions. See also [Canvas.drawImageNine].
///
///  * `repeat`: If the image does not fill `rect`, whether and how the image
///    should be repeated to fill `rect`. By default, the image is not repeated.
///    See [ImageRepeat] for details.
///
///  * `flipHorizontally`: Whether to flip the image horizontally. This is
///    occasionally used with images in right-to-left environments, for images
///    that were designed for left-to-right locales (or vice versa). Be careful,
///    when using this, to not flip images with integral shadows, text, or other
///    effects that will look incorrect when flipped.
///
///  * `invertColors`: Inverting the colors of an image applies a new color
///    filter to the paint. If there is another specified color filter, the
///    invert will be applied after it. This is primarily used for implementing
///    smart invert on iOS.
///
///  * `filterQuality`: Use this to change the quality when scaling an image.
///     Defaults to [FilterQuality.medium].
///
/// See also:
///
///  * [paintBorder], which paints a border around a rectangle on a canvas.
///  * [DecorationImage], which holds a configuration for calling this function.
///  * [BoxDecoration], which uses this function to paint a [DecorationImage].
void paintImage({
  required Canvas canvas,
  required Rect rect,
  required ui.Image image,
  String? debugImageLabel,
  double scale = 1.0,
  double opacity = 1.0,
  ColorFilter? colorFilter,
  BoxFit? fit,
  Alignment alignment = Alignment.center,
  Rect? centerSlice,
  ImageRepeat repeat = ImageRepeat.noRepeat,
  bool flipHorizontally = false,
  bool invertColors = false,
  FilterQuality filterQuality = FilterQuality.medium,
  bool isAntiAlias = false,
  BlendMode blendMode = BlendMode.srcOver,
}) {
  assert(
    image.debugGetOpenHandleStackTraces()?.isNotEmpty ?? true,
    'Cannot paint an image that is disposed.\n'
    'The caller of paintImage is expected to wait to dispose the image until '
    'after painting has completed.',
  );
  if (rect.isEmpty) {
    return;
  }
  Size outputSize = rect.size;
  Size inputSize = Size(image.width.toDouble(), image.height.toDouble());
  Offset? sliceBorder;
  if (centerSlice != null) {
    sliceBorder = inputSize / scale - centerSlice.size as Offset;
    outputSize = outputSize - sliceBorder as Size;
    inputSize = inputSize - sliceBorder * scale as Size;
  }
  fit ??= centerSlice == null ? BoxFit.scaleDown : BoxFit.fill;
  assert(centerSlice == null || (fit != BoxFit.none && fit != BoxFit.cover));
  final FittedSizes fittedSizes = applyBoxFit(fit, inputSize / scale, outputSize);
  final Size sourceSize = fittedSizes.source * scale;
  Size destinationSize = fittedSizes.destination;
  if (centerSlice != null) {
    outputSize += sliceBorder!;
    destinationSize += sliceBorder;
    // We don't have the ability to draw a subset of the image at the same time
    // as we apply a nine-patch stretch.
    assert(
      sourceSize == inputSize,
      'centerSlice was used with a BoxFit that does not guarantee that the image is fully visible.',
    );
  }

  if (repeat != ImageRepeat.noRepeat && destinationSize == outputSize) {
    // There's no need to repeat the image because we're exactly filling the
    // output rect with the image.
    repeat = ImageRepeat.noRepeat;
  }
  final Paint paint = Paint()..isAntiAlias = isAntiAlias;
  if (colorFilter != null) {
    paint.colorFilter = colorFilter;
  }
  paint.color = Color.fromRGBO(0, 0, 0, clampDouble(opacity, 0.0, 1.0));
  paint.filterQuality = filterQuality;
  paint.invertColors = invertColors;
  paint.blendMode = blendMode;
  final double halfWidthDelta = (outputSize.width - destinationSize.width) / 2.0;
  final double halfHeightDelta = (outputSize.height - destinationSize.height) / 2.0;
  final double dx =
      halfWidthDelta + (flipHorizontally ? -alignment.x : alignment.x) * halfWidthDelta;
  final double dy = halfHeightDelta + alignment.y * halfHeightDelta;
  final Offset destinationPosition = rect.topLeft.translate(dx, dy);
  final Rect destinationRect = destinationPosition & destinationSize;

  // Set to true if we added a saveLayer to the canvas to invert/flip the image.
  bool invertedCanvas = false;
  // Output size and destination rect are fully calculated.

  // Implement debug-mode and profile-mode features:
  //  - cacheWidth/cacheHeight warning
  //  - debugInvertOversizedImages
  //  - debugOnPaintImage
  //  - Flutter.ImageSizesForFrame events in timeline
  if (!kReleaseMode) {
    // We can use the devicePixelRatio of the views directly here (instead of
    // going through a MediaQuery) because if it changes, whatever is aware of
    // the MediaQuery will be repainting the image anyways.
    // Furthermore, for the memory check below we just assume that all images
    // are decoded for the view with the highest device pixel ratio and use that
    // as an upper bound for the display size of the image.
    final double maxDevicePixelRatio = PaintingBinding.instance.platformDispatcher.views.fold(
      0.0,
      (double previousValue, ui.FlutterView view) => math.max(previousValue, view.devicePixelRatio),
    );
    final ImageSizeInfo sizeInfo = ImageSizeInfo(
      // Some ImageProvider implementations may not have given this.
      source: debugImageLabel ?? '<Unknown Image(${image.width}×${image.height})>',
      imageSize: Size(image.width.toDouble(), image.height.toDouble()),
      displaySize: outputSize * maxDevicePixelRatio,
    );
    assert(() {
      if (debugInvertOversizedImages &&
          sizeInfo.decodedSizeInBytes > sizeInfo.displaySizeInBytes + debugImageOverheadAllowance) {
        final int overheadInKilobytes =
            (sizeInfo.decodedSizeInBytes - sizeInfo.displaySizeInBytes) ~/ 1024;
        final int outputWidth = sizeInfo.displaySize.width.toInt();
        final int outputHeight = sizeInfo.displaySize.height.toInt();
        FlutterError.reportError(
          FlutterErrorDetails(
            exception:
                'Image $debugImageLabel has a display size of '
                '$outputWidth×$outputHeight but a decode size of '
                '${image.width}×${image.height}, which uses an additional '
                '${overheadInKilobytes}KB (assuming a device pixel ratio of '
                '$maxDevicePixelRatio).\n\n'
                'Consider resizing the asset ahead of time, supplying a cacheWidth '
                'parameter of $outputWidth, a cacheHeight parameter of '
                '$outputHeight, or using a ResizeImage.',
            library: 'painting library',
            context: ErrorDescription('while painting an image'),
          ),
        );
        // Invert the colors of the canvas.
        canvas.saveLayer(
          destinationRect,
          Paint()
            ..colorFilter = const ColorFilter.matrix(<double>[
              -1,
              0,
              0,
              0,
              255,
              0,
              -1,
              0,
              0,
              255,
              0,
              0,
              -1,
              0,
              255,
              0,
              0,
              0,
              1,
              0,
            ]),
        );
        // Flip the canvas vertically.
        final double dy = -(rect.top + rect.height / 2.0);
        canvas.translate(0.0, -dy);
        canvas.scale(1.0, -1.0);
        canvas.translate(0.0, dy);
        invertedCanvas = true;
      }
      return true;
    }());
    // Avoid emitting events that are the same as those emitted in the last frame.
    if (!_lastFrameImageSizeInfo.contains(sizeInfo)) {
      final ImageSizeInfo? existingSizeInfo = _pendingImageSizeInfo[sizeInfo.source];
      if (existingSizeInfo == null ||
          existingSizeInfo.displaySizeInBytes < sizeInfo.displaySizeInBytes) {
        _pendingImageSizeInfo[sizeInfo.source!] = sizeInfo;
      }
      debugOnPaintImage?.call(sizeInfo);
      SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) {
        _lastFrameImageSizeInfo = _pendingImageSizeInfo.values.toSet();
        if (_pendingImageSizeInfo.isEmpty) {
          return;
        }
        developer.postEvent('Flutter.ImageSizesForFrame', <String, Object>{
          for (final ImageSizeInfo imageSizeInfo in _pendingImageSizeInfo.values)
            imageSizeInfo.source!: imageSizeInfo.toJson(),
        });
        _pendingImageSizeInfo = <String, ImageSizeInfo>{};
      }, debugLabel: 'paintImage.recordImageSizes');
    }
  }

  final bool needSave = centerSlice != null || repeat != ImageRepeat.noRepeat || flipHorizontally;
  if (needSave) {
    canvas.save();
  }
  if (repeat != ImageRepeat.noRepeat) {
    canvas.clipRect(rect);
  }
  if (flipHorizontally) {
    final double dx = -(rect.left + rect.width / 2.0);
    canvas.translate(-dx, 0.0);
    canvas.scale(-1.0, 1.0);
    canvas.translate(dx, 0.0);
  }
  if (centerSlice == null) {
    final Rect sourceRect = alignment.inscribe(sourceSize, Offset.zero & inputSize);
    if (repeat == ImageRepeat.noRepeat) {
      canvas.drawImageRect(image, sourceRect, destinationRect, paint);
    } else {
      for (final Rect tileRect in _generateImageTileRects(rect, destinationRect, repeat)) {
        canvas.drawImageRect(image, sourceRect, tileRect, paint);
      }
    }
  } else {
    canvas.scale(1 / scale);
    if (repeat == ImageRepeat.noRepeat) {
      canvas.drawImageNine(
        image,
        _scaleRect(centerSlice, scale),
        _scaleRect(destinationRect, scale),
        paint,
      );
    } else {
      for (final Rect tileRect in _generateImageTileRects(rect, destinationRect, repeat)) {
        canvas.drawImageNine(
          image,
          _scaleRect(centerSlice, scale),
          _scaleRect(tileRect, scale),
          paint,
        );
      }
    }
  }
  if (needSave) {
    canvas.restore();
  }

  if (invertedCanvas) {
    canvas.restore();
  }
}

Iterable<Rect> _generateImageTileRects(Rect outputRect, Rect fundamentalRect, ImageRepeat repeat) {
  int startX = 0;
  int startY = 0;
  int stopX = 0;
  int stopY = 0;
  final double strideX = fundamentalRect.width;
  final double strideY = fundamentalRect.height;

  if (repeat == ImageRepeat.repeat || repeat == ImageRepeat.repeatX) {
    startX = ((outputRect.left - fundamentalRect.left) / strideX).floor();
    stopX = ((outputRect.right - fundamentalRect.right) / strideX).ceil();
  }

  if (repeat == ImageRepeat.repeat || repeat == ImageRepeat.repeatY) {
    startY = ((outputRect.top - fundamentalRect.top) / strideY).floor();
    stopY = ((outputRect.bottom - fundamentalRect.bottom) / strideY).ceil();
  }

  return <Rect>[
    for (int i = startX; i <= stopX; ++i)
      for (int j = startY; j <= stopY; ++j) fundamentalRect.shift(Offset(i * strideX, j * strideY)),
  ];
}

Rect _scaleRect(Rect rect, double scale) =>
    Rect.fromLTRB(rect.left * scale, rect.top * scale, rect.right * scale, rect.bottom * scale);

// Implements DecorationImage.lerp when the image is different.
//
// This class just paints both decorations on top of each other, blended together.
//
// The Decoration properties are faked by just forwarded to the target image.
class _BlendedDecorationImage implements DecorationImage {
  const _BlendedDecorationImage(this.a, this.b, this.t) : assert(a != null || b != null);

  final DecorationImage? a;
  final DecorationImage? b;
  final double t;

  @override
  ImageProvider get image => b?.image ?? a!.image;
  @override
  ImageErrorListener? get onError => b?.onError ?? a!.onError;
  @override
  ColorFilter? get colorFilter => b?.colorFilter ?? a!.colorFilter;
  @override
  BoxFit? get fit => b?.fit ?? a!.fit;
  @override
  AlignmentGeometry get alignment => b?.alignment ?? a!.alignment;
  @override
  Rect? get centerSlice => b?.centerSlice ?? a!.centerSlice;
  @override
  ImageRepeat get repeat => b?.repeat ?? a!.repeat;
  @override
  bool get matchTextDirection => b?.matchTextDirection ?? a!.matchTextDirection;
  @override
  double get scale => b?.scale ?? a!.scale;
  @override
  double get opacity => b?.opacity ?? a!.opacity;
  @override
  FilterQuality get filterQuality => b?.filterQuality ?? a!.filterQuality;
  @override
  bool get invertColors => b?.invertColors ?? a!.invertColors;
  @override
  bool get isAntiAlias => b?.isAntiAlias ?? a!.isAntiAlias;

  @override
  DecorationImagePainter createPainter(VoidCallback onChanged) {
    return _BlendedDecorationImagePainter._(
      a?.createPainter(onChanged),
      b?.createPainter(onChanged),
      t,
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _BlendedDecorationImage && other.a == a && other.b == b && other.t == t;
  }

  @override
  int get hashCode => Object.hash(a, b, t);

  @override
  String toString() {
    return '${objectRuntimeType(this, '_BlendedDecorationImage')}($a, $b, $t)';
  }
}

class _BlendedDecorationImagePainter implements DecorationImagePainter {
  _BlendedDecorationImagePainter._(this.a, this.b, this.t) {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/painting.dart',
        className: '$_BlendedDecorationImagePainter',
        object: this,
      );
    }
  }

  final DecorationImagePainter? a;
  final DecorationImagePainter? b;
  final double t;

  @override
  void paint(
    Canvas canvas,
    Rect rect,
    Path? clipPath,
    ImageConfiguration configuration, {
    double blend = 1.0,
    BlendMode blendMode = BlendMode.srcOver,
  }) {
    canvas.saveLayer(null, Paint());
    a?.paint(canvas, rect, clipPath, configuration, blend: blend * (1.0 - t), blendMode: blendMode);
    b?.paint(
      canvas,
      rect,
      clipPath,
      configuration,
      blend: blend * t,
      blendMode: a != null ? BlendMode.plus : blendMode,
    );
    canvas.restore();
  }

  @override
  void dispose() {
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    a?.dispose();
    b?.dispose();
  }

  @override
  String toString() {
    return '${objectRuntimeType(this, '_BlendedDecorationImagePainter')}($a, $b, $t)';
  }
}
