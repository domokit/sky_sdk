// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

import 'basic.dart';
import 'framework.dart';
import 'image.dart';
import 'implicit_animations.dart';

// Examples can assume:
// late Uint8List bytes;

/// An image that shows a [placeholder] image while the target [image] is
/// loading, then fades in the new image when it loads.
///
/// Use this class to display long-loading images, such as [NetworkImage.new],
/// so that the image appears on screen with a graceful animation rather than
/// abruptly popping onto the screen.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=pK738Pg9cxc}
///
/// If the [image] emits an [ImageInfo] synchronously, such as when the image
/// has been loaded and cached, the [image] is displayed immediately, and the
/// [placeholder] is never displayed.
///
/// The [fadeOutDuration] and [fadeOutCurve] properties control the fade-out
/// animation of the [placeholder].
///
/// The [fadeInDuration] and [fadeInCurve] properties control the fade-in
/// animation of the target [image].
///
/// Prefer a [placeholder] that's already cached so that it is displayed
/// immediately. This prevents it from popping onto the screen.
///
/// When [image] changes, it is resolved to a new [ImageStream]. If the new
/// [ImageStream.key] is different, this widget subscribes to the new stream and
/// replaces the displayed image with images emitted by the new stream.
///
/// When [placeholder] changes and the [image] has not yet emitted an
/// [ImageInfo], then [placeholder] is resolved to a new [ImageStream]. If the
/// new [ImageStream.key] is different, this widget subscribes to the new stream
/// and replaces the displayed image to images emitted by the new stream.
///
/// When either [placeholder] or [image] changes, this widget continues showing
/// the previously loaded image (if any) until the new image provider provides a
/// different image. This is known as "gapless playback" (see also
/// [Image.gaplessPlayback]).
///
/// {@tool snippet}
///
/// ```dart
/// FadeInImage(
///   // here `bytes` is a Uint8List containing the bytes for the in-memory image
///   placeholder: MemoryImage(bytes),
///   image: const NetworkImage('https://backend.example.com/image.png'),
/// )
/// ```
/// {@end-tool}
class FadeInImage extends StatefulWidget {
  /// Creates a widget that displays a [placeholder] while an [image] is loading,
  /// then fades-out the placeholder and fades-in the image.
  ///
  /// The [placeholder] and [image] may be composed in a [ResizeImage] to provide
  /// a custom decode/cache size.
  ///
  /// The [placeholder] and [image] may have their own BoxFit settings via [fit]
  /// and [placeholderFit].
  ///
  /// The [placeholder] and [image] may have their own FilterQuality settings via [filterQuality]
  /// and [placeholderFilterQuality].
  ///
  /// If [excludeFromSemantics] is true, then [imageSemanticLabel] will be ignored.
  const FadeInImage({
    super.key,
    required this.placeholder,
    this.placeholderErrorBuilder,
    required this.image,
    this.imageErrorBuilder,
    this.excludeFromSemantics = false,
    this.imageSemanticLabel,
    this.fadeOutDuration = const Duration(milliseconds: 300),
    this.fadeOutCurve = Curves.easeOut,
    this.fadeInDuration = const Duration(milliseconds: 700),
    this.fadeInCurve = Curves.easeIn,
    this.color,
    this.colorBlendMode,
    this.placeholderColor,
    this.placeholderColorBlendMode,
    this.width,
    this.height,
    this.fit,
    this.placeholderFit,
    this.filterQuality = FilterQuality.medium,
    this.placeholderFilterQuality,
    this.alignment = Alignment.center,
    this.repeat = ImageRepeat.noRepeat,
    this.matchTextDirection = false,
  });

  /// Creates a widget that uses a placeholder image stored in memory while
  /// loading the final image from the network.
  ///
  /// The `placeholder` argument contains the bytes of the in-memory image.
  ///
  /// The `image` argument is the URL of the final image.
  ///
  /// The `placeholderScale` and `imageScale` arguments are passed to their
  /// respective [ImageProvider]s (see also [ImageInfo.scale]).
  ///
  /// If [placeholderCacheWidth], [placeholderCacheHeight], [imageCacheWidth],
  /// or [imageCacheHeight] are provided, it indicates to the
  /// engine that the respective image should be decoded at the specified size.
  /// The image will be rendered to the constraints of the layout or [width]
  /// and [height] regardless of these parameters. These parameters are primarily
  /// intended to reduce the memory usage of [ImageCache].
  ///
  /// The [placeholder], [image], [placeholderScale], [imageScale],
  /// [fadeOutDuration], [fadeOutCurve], [fadeInDuration], [fadeInCurve],
  /// [alignment], [repeat], and [matchTextDirection] arguments must not be
  /// null.
  ///
  /// See also:
  ///
  ///  * [Image.memory], which has more details about loading images from
  ///    memory.
  ///  * [Image.network], which has more details about loading images from
  ///    the network.
  FadeInImage.memoryNetwork({
    super.key,
    required Uint8List placeholder,
    this.placeholderErrorBuilder,
    required String image,
    this.imageErrorBuilder,
    double placeholderScale = 1.0,
    double imageScale = 1.0,
    this.excludeFromSemantics = false,
    this.imageSemanticLabel,
    this.fadeOutDuration = const Duration(milliseconds: 300),
    this.fadeOutCurve = Curves.easeOut,
    this.fadeInDuration = const Duration(milliseconds: 700),
    this.fadeInCurve = Curves.easeIn,
    this.width,
    this.height,
    this.fit,
    this.color,
    this.colorBlendMode,
    this.placeholderColor,
    this.placeholderColorBlendMode,
    this.placeholderFit,
    this.filterQuality = FilterQuality.medium,
    this.placeholderFilterQuality,
    this.alignment = Alignment.center,
    this.repeat = ImageRepeat.noRepeat,
    this.matchTextDirection = false,
    int? placeholderCacheWidth,
    int? placeholderCacheHeight,
    int? imageCacheWidth,
    int? imageCacheHeight,
  }) : placeholder = ResizeImage.resizeIfNeeded(
         placeholderCacheWidth,
         placeholderCacheHeight,
         MemoryImage(placeholder, scale: placeholderScale),
       ),
       image = ResizeImage.resizeIfNeeded(
         imageCacheWidth,
         imageCacheHeight,
         NetworkImage(image, scale: imageScale),
       );

  /// Creates a widget that uses a placeholder image stored in an asset bundle
  /// while loading the final image from the network.
  ///
  /// The `placeholder` argument is the key of the image in the asset bundle.
  ///
  /// The `image` argument is the URL of the final image.
  ///
  /// The `placeholderScale` and `imageScale` arguments are passed to their
  /// respective [ImageProvider]s (see also [ImageInfo.scale]).
  ///
  /// If `placeholderScale` is omitted or is null, pixel-density-aware asset
  /// resolution will be attempted for the [placeholder] image. Otherwise, the
  /// exact asset specified will be used.
  ///
  /// If [placeholderCacheWidth], [placeholderCacheHeight], [imageCacheWidth],
  /// or [imageCacheHeight] are provided, it indicates to the
  /// engine that the respective image should be decoded at the specified size.
  /// The image will be rendered to the constraints of the layout or [width]
  /// and [height] regardless of these parameters. These parameters are primarily
  /// intended to reduce the memory usage of [ImageCache].
  ///
  /// See also:
  ///
  ///  * [Image.asset], which has more details about loading images from
  ///    asset bundles.
  ///  * [Image.network], which has more details about loading images from
  ///    the network.
  FadeInImage.assetNetwork({
    super.key,
    required String placeholder,
    this.placeholderErrorBuilder,
    required String image,
    this.imageErrorBuilder,
    AssetBundle? bundle,
    double? placeholderScale,
    double imageScale = 1.0,
    this.excludeFromSemantics = false,
    this.imageSemanticLabel,
    this.fadeOutDuration = const Duration(milliseconds: 300),
    this.fadeOutCurve = Curves.easeOut,
    this.fadeInDuration = const Duration(milliseconds: 700),
    this.fadeInCurve = Curves.easeIn,
    this.width,
    this.height,
    this.fit,
    this.color,
    this.colorBlendMode,
    this.placeholderColor,
    this.placeholderColorBlendMode,
    this.placeholderFit,
    this.filterQuality = FilterQuality.medium,
    this.placeholderFilterQuality,
    this.alignment = Alignment.center,
    this.repeat = ImageRepeat.noRepeat,
    this.matchTextDirection = false,
    int? placeholderCacheWidth,
    int? placeholderCacheHeight,
    int? imageCacheWidth,
    int? imageCacheHeight,
  }) : placeholder =
           placeholderScale != null
               ? ResizeImage.resizeIfNeeded(
                 placeholderCacheWidth,
                 placeholderCacheHeight,
                 ExactAssetImage(placeholder, bundle: bundle, scale: placeholderScale),
               )
               : ResizeImage.resizeIfNeeded(
                 placeholderCacheWidth,
                 placeholderCacheHeight,
                 AssetImage(placeholder, bundle: bundle),
               ),
       image = ResizeImage.resizeIfNeeded(
         imageCacheWidth,
         imageCacheHeight,
         NetworkImage(image, scale: imageScale),
       );

  /// Image displayed while the target [image] is loading.
  final ImageProvider placeholder;

  /// A builder function that is called if an error occurs during placeholder
  /// image loading.
  ///
  /// If this builder is not provided, any exceptions will be reported to
  /// [FlutterError.onError]. If it is provided, the caller should either handle
  /// the exception by providing a replacement widget, or rethrow the exception.
  final ImageErrorWidgetBuilder? placeholderErrorBuilder;

  /// The target image that is displayed once it has loaded.
  final ImageProvider image;

  /// A builder function that is called if an error occurs during image loading.
  ///
  /// If this builder is not provided, any exceptions will be reported to
  /// [FlutterError.onError]. If it is provided, the caller should either handle
  /// the exception by providing a replacement widget, or rethrow the exception.
  final ImageErrorWidgetBuilder? imageErrorBuilder;

  /// The duration of the fade-out animation for the [placeholder].
  final Duration fadeOutDuration;

  /// The curve of the fade-out animation for the [placeholder].
  final Curve fadeOutCurve;

  /// The duration of the fade-in animation for the [image].
  final Duration fadeInDuration;

  /// The curve of the fade-in animation for the [image].
  final Curve fadeInCurve;

  /// If non-null, require the image to have this width.
  ///
  /// If null, the image will pick a size that best preserves its intrinsic
  /// aspect ratio. This may result in a sudden change if the size of the
  /// placeholder image does not match that of the target image. The size is
  /// also affected by the scale factor.
  final double? width;

  /// If non-null, this color is blended with each image pixel using [colorBlendMode].
  ///
  /// Color applies to the [image].
  ///
  /// See Also:
  ///
  ///  * [placeholderColor], the color which applies to the [placeholder].
  final Color? color;

  /// Used to combine [color] with this [image].
  ///
  /// The default is [BlendMode.srcIn]. In terms of the blend mode, [color] is
  /// the source and this image is the destination.
  ///
  /// See also:
  ///
  ///  * [BlendMode], which includes an illustration of the effect of each blend mode.
  ///  * [placeholderColorBlendMode], the color blend mode which applies to the [placeholder].
  final BlendMode? colorBlendMode;

  /// If non-null, this color is blended with each placeholder image pixel using [placeholderColorBlendMode].
  ///
  /// Color applies to the [placeholder].
  ///
  /// See Also:
  ///
  ///  * [color], the color which applies to the [image].
  final Color? placeholderColor;

  /// Used to combine [placeholderColor] with the [placeholder] image.
  ///
  /// The default is [BlendMode.srcIn]. In terms of the blend mode, [placeholderColor] is
  /// the source and this placeholder is the destination.
  ///
  /// See also:
  ///
  ///  * [BlendMode], which includes an illustration of the effect of each blend mode.
  ///  * [colorBlendMode], the color blend mode which applies to the [image].
  final BlendMode? placeholderColorBlendMode;

  /// If non-null, require the image to have this height.
  ///
  /// If null, the image will pick a size that best preserves its intrinsic
  /// aspect ratio. This may result in a sudden change if the size of the
  /// placeholder image does not match that of the target image. The size is
  /// also affected by the scale factor.
  final double? height;

  /// How to inscribe the image into the space allocated during layout.
  ///
  /// The default varies based on the other fields. See the discussion at
  /// [paintImage].
  final BoxFit? fit;

  /// How to inscribe the placeholder image into the space allocated during layout.
  ///
  /// If not value set, it will fallback to [fit].
  final BoxFit? placeholderFit;

  /// The rendering quality of the image.
  ///
  /// {@macro flutter.widgets.image.filterQuality}
  final FilterQuality filterQuality;

  /// The rendering quality of the placeholder image.
  ///
  /// {@macro flutter.widgets.image.filterQuality}
  final FilterQuality? placeholderFilterQuality;

  /// How to align the image within its bounds.
  ///
  /// The alignment aligns the given position in the image to the given position
  /// in the layout bounds. For example, an [Alignment] alignment of (-1.0,
  /// -1.0) aligns the image to the top-left corner of its layout bounds, while an
  /// [Alignment] alignment of (1.0, 1.0) aligns the bottom right of the
  /// image with the bottom right corner of its layout bounds. Similarly, an
  /// alignment of (0.0, 1.0) aligns the bottom middle of the image with the
  /// middle of the bottom edge of its layout bounds.
  ///
  /// If the [alignment] is [TextDirection]-dependent (i.e. if it is a
  /// [AlignmentDirectional]), then an ambient [Directionality] widget
  /// must be in scope.
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

  /// How to paint any portions of the layout bounds not covered by the image.
  final ImageRepeat repeat;

  /// Whether to paint the image in the direction of the [TextDirection].
  ///
  /// If this is true, then in [TextDirection.ltr] contexts, the image will be
  /// drawn with its origin in the top left (the "normal" painting direction for
  /// images); and in [TextDirection.rtl] contexts, the image will be drawn with
  /// a scaling factor of -1 in the horizontal direction so that the origin is
  /// in the top right.
  ///
  /// This is occasionally used with images in right-to-left environments, for
  /// images that were designed for left-to-right locales. Be careful, when
  /// using this, to not flip images with integral shadows, text, or other
  /// effects that will look incorrect when flipped.
  ///
  /// If this is true, there must be an ambient [Directionality] widget in
  /// scope.
  final bool matchTextDirection;

  /// Whether to exclude this image from semantics.
  ///
  /// This is useful for images which do not contribute meaningful information
  /// to an application.
  final bool excludeFromSemantics;

  /// A semantic description of the [image].
  ///
  /// Used to provide a description of the [image] to TalkBack on Android, and
  /// VoiceOver on iOS.
  ///
  /// This description will be used both while the [placeholder] is shown and
  /// once the image has loaded.
  final String? imageSemanticLabel;

  @override
  State<FadeInImage> createState() => _FadeInImageState();
}

class _FadeInImageState extends State<FadeInImage> {
  static const Animation<double> _kOpaqueAnimation = AlwaysStoppedAnimation<double>(1.0);
  bool targetLoaded = false;

  // These ProxyAnimations are changed to the fade in animation by
  // [_AnimatedFadeOutFadeInState]. Otherwise these animations are reset to
  // their defaults by [_resetAnimations].
  final ProxyAnimation _imageAnimation = ProxyAnimation(_kOpaqueAnimation);
  final ProxyAnimation _placeholderAnimation = ProxyAnimation(_kOpaqueAnimation);

  Image _image({
    required ImageProvider image,
    ImageErrorWidgetBuilder? errorBuilder,
    ImageFrameBuilder? frameBuilder,
    BoxFit? fit,
    Color? color,
    BlendMode? colorBlendMode,
    required FilterQuality filterQuality,
    required Animation<double> opacity,
  }) {
    return Image(
      image: image,
      errorBuilder: errorBuilder,
      frameBuilder: frameBuilder,
      opacity: opacity,
      width: widget.width,
      height: widget.height,
      fit: fit,
      color: color,
      colorBlendMode: colorBlendMode,
      filterQuality: filterQuality,
      alignment: widget.alignment,
      repeat: widget.repeat,
      matchTextDirection: widget.matchTextDirection,
      gaplessPlayback: true,
      excludeFromSemantics: true,
    );
  }

  @override
  Widget build(BuildContext context) {
    Widget result = _image(
      image: widget.image,
      errorBuilder: widget.imageErrorBuilder,
      opacity: _imageAnimation,
      fit: widget.fit,
      color: widget.color,
      colorBlendMode: widget.colorBlendMode,
      filterQuality: widget.filterQuality,
      frameBuilder: (BuildContext context, Widget child, int? frame, bool wasSynchronouslyLoaded) {
        if (wasSynchronouslyLoaded || frame != null) {
          targetLoaded = true;
        }
        return _AnimatedFadeOutFadeIn(
          target: child,
          targetProxyAnimation: _imageAnimation,
          placeholder: _image(
            image: widget.placeholder,
            errorBuilder: widget.placeholderErrorBuilder,
            opacity: _placeholderAnimation,
            color: widget.placeholderColor,
            colorBlendMode: widget.placeholderColorBlendMode,
            fit: widget.placeholderFit ?? widget.fit,
            filterQuality: widget.placeholderFilterQuality ?? widget.filterQuality,
          ),
          placeholderProxyAnimation: _placeholderAnimation,
          isTargetLoaded: targetLoaded,
          wasSynchronouslyLoaded: wasSynchronouslyLoaded,
          fadeInDuration: widget.fadeInDuration,
          fadeOutDuration: widget.fadeOutDuration,
          fadeInCurve: widget.fadeInCurve,
          fadeOutCurve: widget.fadeOutCurve,
        );
      },
    );

    if (!widget.excludeFromSemantics) {
      result = Semantics(
        container: widget.imageSemanticLabel != null,
        image: true,
        label: widget.imageSemanticLabel ?? '',
        child: result,
      );
    }

    return result;
  }
}

class _AnimatedFadeOutFadeIn extends ImplicitlyAnimatedWidget {
  const _AnimatedFadeOutFadeIn({
    required this.target,
    required this.targetProxyAnimation,
    required this.placeholder,
    required this.placeholderProxyAnimation,
    required this.isTargetLoaded,
    required this.fadeOutDuration,
    required this.fadeOutCurve,
    required this.fadeInDuration,
    required this.fadeInCurve,
    required this.wasSynchronouslyLoaded,
  }) : assert(!wasSynchronouslyLoaded || isTargetLoaded),
       super(duration: fadeInDuration + fadeOutDuration);

  final Widget target;
  final ProxyAnimation targetProxyAnimation;
  final Widget placeholder;
  final ProxyAnimation placeholderProxyAnimation;
  final bool isTargetLoaded;
  final Duration fadeInDuration;
  final Duration fadeOutDuration;
  final Curve fadeInCurve;
  final Curve fadeOutCurve;
  final bool wasSynchronouslyLoaded;

  @override
  _AnimatedFadeOutFadeInState createState() => _AnimatedFadeOutFadeInState();
}

class _AnimatedFadeOutFadeInState extends ImplicitlyAnimatedWidgetState<_AnimatedFadeOutFadeIn> {
  Tween<double>? _targetOpacity;
  Tween<double>? _placeholderOpacity;
  Animation<double>? _targetOpacityAnimation;
  Animation<double>? _placeholderOpacityAnimation;

  @override
  void forEachTween(TweenVisitor<dynamic> visitor) {
    _targetOpacity =
        visitor(
              _targetOpacity,
              widget.isTargetLoaded ? 1.0 : 0.0,
              (dynamic value) => Tween<double>(begin: value as double),
            )
            as Tween<double>?;
    _placeholderOpacity =
        visitor(
              _placeholderOpacity,
              widget.isTargetLoaded ? 0.0 : 1.0,
              (dynamic value) => Tween<double>(begin: value as double),
            )
            as Tween<double>?;
  }

  @override
  void didUpdateTweens() {
    if (widget.wasSynchronouslyLoaded) {
      // Opacity animations should not be reset if image was synchronously loaded.
      return;
    }

    _placeholderOpacityAnimation = animation.drive(
      TweenSequence<double>(<TweenSequenceItem<double>>[
        TweenSequenceItem<double>(
          tween: _placeholderOpacity!.chain(CurveTween(curve: widget.fadeOutCurve)),
          weight: widget.fadeOutDuration.inMilliseconds.toDouble(),
        ),
        TweenSequenceItem<double>(
          tween: ConstantTween<double>(0),
          weight: widget.fadeInDuration.inMilliseconds.toDouble(),
        ),
      ]),
    )..addStatusListener((AnimationStatus status) {
      if (_placeholderOpacityAnimation!.isCompleted) {
        // Need to rebuild to remove placeholder now that it is invisible.
        setState(() {});
      }
    });

    _targetOpacityAnimation = animation.drive(
      TweenSequence<double>(<TweenSequenceItem<double>>[
        TweenSequenceItem<double>(
          tween: ConstantTween<double>(0),
          weight: widget.fadeOutDuration.inMilliseconds.toDouble(),
        ),
        TweenSequenceItem<double>(
          tween: _targetOpacity!.chain(CurveTween(curve: widget.fadeInCurve)),
          weight: widget.fadeInDuration.inMilliseconds.toDouble(),
        ),
      ]),
    );

    widget.targetProxyAnimation.parent = _targetOpacityAnimation;
    widget.placeholderProxyAnimation.parent = _placeholderOpacityAnimation;
  }

  @override
  Widget build(BuildContext context) {
    if (widget.wasSynchronouslyLoaded || (_placeholderOpacityAnimation?.isCompleted ?? true)) {
      return widget.target;
    }

    return Stack(
      fit: StackFit.passthrough,
      alignment: AlignmentDirectional.center,
      // Text direction is irrelevant here since we're using center alignment,
      // but it allows the Stack to avoid a call to Directionality.of()
      textDirection: TextDirection.ltr,
      children: <Widget>[widget.target, widget.placeholder],
    );
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      DiagnosticsProperty<Animation<double>>('targetOpacity', _targetOpacityAnimation),
    );
    properties.add(
      DiagnosticsProperty<Animation<double>>('placeholderOpacity', _placeholderOpacityAnimation),
    );
  }
}
