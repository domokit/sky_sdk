// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
///
/// @docImport 'app.dart';
/// @docImport 'fade_in_image.dart';
/// @docImport 'icon.dart';
/// @docImport 'transitions.dart';
library;

import 'dart:async';
import 'dart:io' show File;

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/semantics.dart';

import '../painting/_web_image_info_io.dart'
    if (dart.library.js_util) '../painting/_web_image_info_web.dart';
import '_web_image_io.dart' if (dart.library.js_util) '_web_image_web.dart';
import 'basic.dart';
import 'binding.dart';
import 'disposable_build_context.dart';
import 'framework.dart';
import 'localizations.dart';
import 'media_query.dart';
import 'placeholder.dart';
import 'scroll_aware_image_provider.dart';
import 'text.dart';
import 'ticker_provider.dart';

export 'package:flutter/painting.dart'
    show
        AssetImage,
        ExactAssetImage,
        FileImage,
        FilterQuality,
        ImageConfiguration,
        ImageInfo,
        ImageProvider,
        ImageStream,
        MemoryImage,
        NetworkImage;

// Examples can assume:
// late Widget image;
// late ImageProvider _image;

/// Creates an [ImageConfiguration] based on the given [BuildContext] (and
/// optionally size).
///
/// This is the object that must be passed to [BoxPainter.paint] and to
/// [ImageProvider.resolve].
///
/// If this is not called from a build method, then it should be reinvoked
/// whenever the dependencies change, e.g. by calling it from
/// [State.didChangeDependencies], so that any changes in the environment are
/// picked up (e.g. if the device pixel ratio changes).
///
/// See also:
///
///  * [ImageProvider], which has an example showing how this might be used.
ImageConfiguration createLocalImageConfiguration(BuildContext context, {Size? size}) {
  return ImageConfiguration(
    bundle: DefaultAssetBundle.of(context),
    devicePixelRatio: MediaQuery.maybeDevicePixelRatioOf(context) ?? 1.0,
    locale: Localizations.maybeLocaleOf(context),
    textDirection: Directionality.maybeOf(context),
    size: size,
    platform: defaultTargetPlatform,
  );
}

/// Prefetches an image into the image cache.
///
/// Returns a [Future] that will complete when the first image yielded by the
/// [ImageProvider] is available or failed to load.
///
/// If the image is later used by an [Image] or [BoxDecoration] or [FadeInImage],
/// it will probably be loaded faster. The consumer of the image does not need
/// to use the same [ImageProvider] instance. The [ImageCache] will find the image
/// as long as both images share the same key, and the image is held by the
/// cache.
///
/// The cache may refuse to hold the image if it is disabled, the image is too
/// large, or some other criteria implemented by a custom [ImageCache]
/// implementation.
///
/// The [ImageCache] holds a reference to all images passed to
/// [ImageCache.putIfAbsent] as long as their [ImageStreamCompleter] has at
/// least one listener. This method will wait until the end of the frame after
/// its future completes before releasing its own listener. This gives callers a
/// chance to listen to the stream if necessary. A caller can determine if the
/// image ended up in the cache by calling [ImageProvider.obtainCacheStatus]. If
/// it is only held as [ImageCacheStatus.live], and the caller wishes to keep
/// the resolved image in memory, the caller should immediately call
/// `provider.resolve` and add a listener to the returned [ImageStream]. The
/// image will remain pinned in memory at least until the caller removes its
/// listener from the stream, even if it would not otherwise fit into the cache.
///
/// Callers should be cautious about pinning large images or a large number of
/// images in memory, as this can result in running out of memory and being
/// killed by the operating system. The lower the available physical memory, the
/// more susceptible callers will be to running into OOM issues. These issues
/// manifest as immediate process death, sometimes with no other error messages.
///
/// The [BuildContext] and [Size] are used to select an image configuration
/// (see [createLocalImageConfiguration]).
///
/// The returned future will not complete with error, even if precaching
/// failed. The `onError` argument can be used to manually handle errors while
/// pre-caching.
///
/// See also:
///
///  * [ImageCache], which holds images that may be reused.
Future<void> precacheImage(
  ImageProvider provider,
  BuildContext context, {
  Size? size,
  ImageErrorListener? onError,
}) {
  final ImageConfiguration config = createLocalImageConfiguration(context, size: size);
  final Completer<void> completer = Completer<void>();
  final ImageStream stream = provider.resolve(config);
  ImageStreamListener? listener;
  listener = ImageStreamListener(
    (ImageInfo? image, bool sync) {
      if (!completer.isCompleted) {
        completer.complete();
      }
      // Give callers until at least the end of the frame to subscribe to the
      // image stream.
      // See ImageCache._liveImages
      SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) {
        image?.dispose();
        stream.removeListener(listener!);
      }, debugLabel: 'precacheImage.removeListener');
    },
    onError: (Object exception, StackTrace? stackTrace) {
      if (!completer.isCompleted) {
        completer.complete();
      }
      stream.removeListener(listener!);
      if (onError != null) {
        onError(exception, stackTrace);
      } else {
        FlutterError.reportError(
          FlutterErrorDetails(
            context: ErrorDescription('image failed to precache'),
            library: 'image resource service',
            exception: exception,
            stack: stackTrace,
            silent: true,
          ),
        );
      }
    },
  );
  stream.addListener(listener);
  return completer.future;
}

/// Signature used by [Image.frameBuilder] to control the widget that will be
/// used when an [Image] is built.
///
/// The `child` argument contains the default image widget.
/// Typically, this builder will wrap the `child` widget in some
/// way and return the wrapped widget. If this builder returns `child` directly,
/// it will yield the same result as if [Image.frameBuilder] was null.
///
/// The `frame` argument specifies the index of the current image frame being
/// rendered. It will be null before the first image frame is ready, and zero
/// for the first image frame. For single-frame images, it will never be greater
/// than zero. For multi-frame images (such as animated GIFs), it will increase
/// by one every time a new image frame is shown (including when the image
/// animates in a loop).
///
/// The `wasSynchronouslyLoaded` argument specifies whether the image was
/// available synchronously (on the same
/// [rendering pipeline frame](rendering/RendererBinding/drawFrame.html) as the
/// `Image` widget itself was created) and thus able to be painted immediately.
/// If this is false, then there was one or more rendering pipeline frames where
/// the image wasn't yet available to be painted. For multi-frame images (such
/// as animated GIFs), the value of this argument will be the same for all image
/// frames. In other words, if the first image frame was available immediately,
/// then this argument will be true for all image frames.
///
/// See also:
///
///  * [Image.frameBuilder], which makes use of this signature in the [Image]
///    widget.
typedef ImageFrameBuilder =
    Widget Function(BuildContext context, Widget child, int? frame, bool wasSynchronouslyLoaded);

/// Signature used by [Image.loadingBuilder] to build a representation of the
/// image's loading progress.
///
/// This is useful for images that are incrementally loaded (e.g. over a local
/// file system or a network), and the application wishes to give the user an
/// indication of when the image will be displayed.
///
/// The `child` argument contains the default image widget and is guaranteed to
/// be non-null. Typically, this builder will wrap the `child` widget in some
/// way and return the wrapped widget. If this builder returns `child` directly,
/// it will yield the same result as if [Image.loadingBuilder] was null.
///
/// The `loadingProgress` argument contains the current progress towards loading
/// the image. This argument will be non-null while the image is loading, but it
/// will be null in the following cases:
///
///   * When the widget is first rendered before any bytes have been loaded.
///   * When an image has been fully loaded and is available to be painted.
///
/// If callers are implementing custom [ImageProvider] and [ImageStream]
/// instances (which is very rare), it's possible to produce image streams that
/// continue to fire image chunk events after an image frame has been loaded.
/// In such cases, the `child` parameter will represent the current
/// fully-loaded image frame.
///
/// See also:
///
///  * [Image.loadingBuilder], which makes use of this signature in the [Image]
///    widget.
///  * [ImageChunkListener], a lower-level signature for listening to raw
///    [ImageChunkEvent]s.
typedef ImageLoadingBuilder =
    Widget Function(BuildContext context, Widget child, ImageChunkEvent? loadingProgress);

/// Signature used by [Image.errorBuilder] to create a replacement widget to
/// render instead of the image.
typedef ImageErrorWidgetBuilder =
    Widget Function(BuildContext context, Object error, StackTrace? stackTrace);

/// A widget that displays an image.
///
/// {@youtube 560 315 https://www.youtube.com/watch?v=7oIAs-0G4mw}
///
/// Several constructors are provided for the various ways that an image can be
/// specified:
///
///  * [Image.new], for obtaining an image from an [ImageProvider].
///  * [Image.asset], for obtaining an image from an [AssetBundle]
///    using a key.
///  * [Image.network], for obtaining an image from a URL.
///  * [Image.file], for obtaining an image from a [File].
///  * [Image.memory], for obtaining an image from a [Uint8List].
///
/// The following image formats are supported: {@macro dart.ui.imageFormats}
///
/// To automatically perform pixel-density-aware asset resolution, specify the
/// image using an [AssetImage] and make sure that a [MaterialApp], [WidgetsApp],
/// or [MediaQuery] widget exists above the [Image] widget in the widget tree.
///
/// The image is painted using [paintImage], which describes the meanings of the
/// various fields on this class in more detail.
///
/// {@tool snippet}
/// The default constructor can be used with any [ImageProvider], such as a
/// [NetworkImage], to display an image from the internet.
///
/// ![An image of an owl displayed by the image widget](https://flutter.github.io/assets-for-api-docs/assets/widgets/owl.jpg)
///
/// ```dart
/// const Image(
///   image: NetworkImage('https://flutter.github.io/assets-for-api-docs/assets/widgets/owl.jpg'),
/// )
/// ```
/// {@end-tool}
///
/// {@tool snippet}
/// The [Image] Widget also provides several constructors to display different
/// types of images for convenience. In this example, use the [Image.network]
/// constructor to display an image from the internet.
///
/// ![An image of an owl displayed by the image widget using the shortcut constructor](https://flutter.github.io/assets-for-api-docs/assets/widgets/owl-2.jpg)
///
/// ```dart
/// Image.network('https://flutter.github.io/assets-for-api-docs/assets/widgets/owl-2.jpg')
/// ```
/// {@end-tool}
///
/// ## Memory usage
///
/// The image is stored in memory in uncompressed form (so that it can be
/// rendered). Large images will use a lot of memory: a 4K image (3840×2160)
/// will use over 30MB of RAM (assuming 32 bits per pixel).
///
/// This problem is exacerbated by the images being cached in the [ImageCache],
/// so large images can use memory for even longer than they are displayed.
///
/// The [Image.asset], [Image.network], [Image.file], and [Image.memory]
/// constructors allow a custom decode size to be specified through `cacheWidth`
/// and `cacheHeight` parameters. The engine will then decode and store the
/// image at the specified size, instead of the image's natural size.
///
/// This can significantly reduce the memory usage. For example, a 4K image that
/// will be rendered at only 384×216 pixels (one-tenth the horizontal and
/// vertical dimensions) would only use 330KB if those dimensions are specified
/// using the `cacheWidth` and `cacheHeight` parameters, a 100-fold reduction in
/// memory usage.
///
/// ## Custom image providers
///
/// {@tool dartpad}
/// In this example, a variant of [NetworkImage] is created that passes all the
/// [ImageConfiguration] information (locale, platform, size, etc) to the server
/// using query arguments in the image URL.
///
/// ** See code in examples/api/lib/painting/image_provider/image_provider.0.dart **
/// {@end-tool}
///
/// See also:
///
///  * [Icon], which shows an image from a font.
///  * [Ink.image], which is the preferred way to show an image in a
///    material application (especially if the image is in a [Material] and will
///    have an [InkWell] on top of it).
///  * [Image](dart-ui/Image-class.html), the class in the [dart:ui] library.
///  * Cookbook: [Display images from the internet](https://docs.flutter.dev/cookbook/images/network-image)
///  * Cookbook: [Fade in images with a placeholder](https://docs.flutter.dev/cookbook/images/fading-in-images)
class Image extends StatefulWidget {
  /// Creates a widget that displays an image.
  ///
  /// To show an image from the network or from an asset bundle, consider using
  /// [Image.network] and [Image.asset] respectively.
  ///
  /// The `scale` argument specifies the linear scale factor for drawing this
  /// image at its intended size and applies to both the width and the height.
  /// {@macro flutter.painting.imageInfo.scale}
  ///
  /// Either the [width] and [height] arguments should be specified, or the
  /// widget should be placed in a context that sets tight layout constraints.
  /// Otherwise, the image dimensions will change as the image is loaded, which
  /// will result in ugly layout changes.
  ///
  /// {@template flutter.widgets.image.filterQualityParameter}
  /// Use [filterQuality] to specify the rendering quality of the image.
  /// {@endtemplate}
  ///
  /// If [excludeFromSemantics] is true, then [semanticLabel] will be ignored.
  const Image({
    super.key,
    required this.image,
    this.frameBuilder,
    this.loadingBuilder,
    this.errorBuilder,
    this.semanticLabel,
    this.excludeFromSemantics = false,
    this.width,
    this.height,
    this.color,
    this.opacity,
    this.colorBlendMode,
    this.fit,
    this.alignment = Alignment.center,
    this.repeat = ImageRepeat.noRepeat,
    this.centerSlice,
    this.matchTextDirection = false,
    this.gaplessPlayback = false,
    this.isAntiAlias = false,
    this.filterQuality = FilterQuality.medium,
  });

  /// Creates a widget that displays an [ImageStream] obtained from the network.
  ///
  /// Either the [width] and [height] arguments should be specified, or the
  /// widget should be placed in a context that sets tight layout constraints.
  /// Otherwise, the image dimensions will change as the image is loaded, which
  /// will result in ugly layout changes.
  ///
  /// The `scale` argument specifies the linear scale factor for drawing this
  /// image at its intended size and applies to both the width and the height.
  /// {@macro flutter.painting.imageInfo.scale}
  ///
  /// All network images are cached regardless of HTTP headers.
  ///
  /// An optional [headers] argument can be used to send custom HTTP headers
  /// with the image request.
  ///
  /// {@macro flutter.widgets.image.filterQualityParameter}
  ///
  /// If [excludeFromSemantics] is true, then [semanticLabel] will be ignored.
  ///
  /// If `cacheWidth` or `cacheHeight` are provided, they indicate to the
  /// engine that the image should be decoded at the specified size. The image
  /// will be rendered to the constraints of the layout or [width] and [height]
  /// regardless of these parameters. These parameters are primarily intended
  /// to reduce the memory usage of [ImageCache].
  ///
  /// In the case where the network image is on the Web platform, the [cacheWidth]
  /// and [cacheHeight] parameters are ignored as the web engine delegates
  /// image decoding to the web which does not support custom decode sizes.
  ///
  /// ### Same-origin policy on Web
  ///
  /// Due to browser restriction on Cross-Origin Resource Sharing (CORS),
  /// Flutter on the Web platform can not fetch images from other origins
  /// (domain, scheme, or port) than the origin that hosts the app, unless the
  /// image hosting origin explicitly allows so. CORS errors can be resolved
  /// by configuring the image hosting server. More information can be
  /// found at Mozilla's introduction on
  /// [same-origin policy](https://developer.mozilla.org/en-US/docs/Web/Security/Same-origin_policy)
  /// and
  /// [CORS errors](https://developer.mozilla.org/en-US/docs/Web/HTTP/CORS/Errors).
  ///
  /// If it's not possible to configure the host, such as when images are hosted
  /// on a CDN or from arbitrary URLs, the app can set the
  /// `webHtmlElementStrategy` parameter of [Image.network] to display the image
  /// in an HTML element, which bypasses the same-origin policy.
  ///
  /// The HTML element is placed in a platform view, and therefore has the
  /// following drawbacks:
  ///
  ///  * Suboptimal performance.
  ///  * Can't be captured by screenshot widgets.
  ///  * The `headers` argument must be null or empty.
  ///  * Some image options are ignored, including [opacity], [colorBlendMode],
  ///    [repeat], filtering, and blurring.
  ///
  /// By default, this feature is turned off ([WebHtmlElementStrategy.never]).
  Image.network(
    String src, {
    super.key,
    double scale = 1.0,
    this.frameBuilder,
    this.loadingBuilder,
    this.errorBuilder,
    this.semanticLabel,
    this.excludeFromSemantics = false,
    this.width,
    this.height,
    this.color,
    this.opacity,
    this.colorBlendMode,
    this.fit,
    this.alignment = Alignment.center,
    this.repeat = ImageRepeat.noRepeat,
    this.centerSlice,
    this.matchTextDirection = false,
    this.gaplessPlayback = false,
    this.filterQuality = FilterQuality.medium,
    this.isAntiAlias = false,
    Map<String, String>? headers,
    int? cacheWidth,
    int? cacheHeight,
    WebHtmlElementStrategy webHtmlElementStrategy = WebHtmlElementStrategy.never,
  }) : image = ResizeImage.resizeIfNeeded(
         cacheWidth,
         cacheHeight,
         NetworkImage(
           src,
           scale: scale,
           headers: headers,
           webHtmlElementStrategy: webHtmlElementStrategy,
         ),
       ),
       assert(cacheWidth == null || cacheWidth > 0),
       assert(cacheHeight == null || cacheHeight > 0);

  /// Creates a widget that displays an [ImageStream] obtained from a [File].
  ///
  /// The `scale` argument specifies the linear scale factor for drawing this
  /// image at its intended size and applies to both the width and the height.
  /// {@macro flutter.painting.imageInfo.scale}
  ///
  /// Either the [width] and [height] arguments should be specified, or the
  /// widget should be placed in a context that sets tight layout constraints.
  /// Otherwise, the image dimensions will change as the image is loaded, which
  /// will result in ugly layout changes.
  ///
  /// On Android, this may require the
  /// `android.permission.READ_EXTERNAL_STORAGE` permission.
  ///
  /// {@macro flutter.widgets.image.filterQualityParameter}
  ///
  /// If [excludeFromSemantics] is true, then [semanticLabel] will be ignored.
  ///
  /// If `cacheWidth` or `cacheHeight` are provided, they indicate to the
  /// engine that the image must be decoded at the specified size. The image
  /// will be rendered to the constraints of the layout or [width] and [height]
  /// regardless of these parameters. These parameters are primarily intended
  /// to reduce the memory usage of [ImageCache].
  ///
  /// Loading an image from a file creates an in memory copy of the file,
  /// which is retained in the [ImageCache]. The underlying file is not
  /// monitored for changes. If it does change, the application should evict
  /// the entry from the [ImageCache].
  ///
  /// See also:
  ///
  ///  * [FileImage] provider for evicting the underlying file easily.
  Image.file(
    File file, {
    super.key,
    double scale = 1.0,
    this.frameBuilder,
    this.errorBuilder,
    this.semanticLabel,
    this.excludeFromSemantics = false,
    this.width,
    this.height,
    this.color,
    this.opacity,
    this.colorBlendMode,
    this.fit,
    this.alignment = Alignment.center,
    this.repeat = ImageRepeat.noRepeat,
    this.centerSlice,
    this.matchTextDirection = false,
    this.gaplessPlayback = false,
    this.isAntiAlias = false,
    this.filterQuality = FilterQuality.medium,
    int? cacheWidth,
    int? cacheHeight,
  }) : // FileImage is not supported on Flutter Web therefore neither this method.
       assert(
         !kIsWeb,
         'Image.file is not supported on Flutter Web. '
         'Consider using either Image.asset or Image.network instead.',
       ),
       image = ResizeImage.resizeIfNeeded(cacheWidth, cacheHeight, FileImage(file, scale: scale)),
       loadingBuilder = null,
       assert(cacheWidth == null || cacheWidth > 0),
       assert(cacheHeight == null || cacheHeight > 0);

  // TODO(ianh): Implement the following (see ../services/image_resolution.dart):
  //
  // * If [width] and [height] are both specified, and [scale] is not, then
  //   size-aware asset resolution will be attempted also, with the given
  //   dimensions interpreted as logical pixels.
  //
  // * If the images have platform, locale, or directionality variants, the
  //   current platform, locale, and directionality are taken into account
  //   during asset resolution as well.
  /// Creates a widget that displays an [ImageStream] obtained from an asset
  /// bundle. The key for the image is given by the `name` argument.
  ///
  /// The `package` argument must be non-null when displaying an image from a
  /// package and null otherwise. See the `Assets in packages` section for
  /// details.
  ///
  /// If the `bundle` argument is omitted or null, then the
  /// [DefaultAssetBundle] will be used.
  ///
  /// The `scale` argument specifies the linear scale factor for drawing this
  /// image at its intended size and applies to both the width and the height.
  /// {@macro flutter.painting.imageInfo.scale}
  ///
  /// By default, the pixel-density-aware asset resolution will be attempted. In
  /// addition:
  ///
  /// * If the `scale` argument is provided and is not null, then the exact
  /// asset specified will be used. To display an image variant with a specific
  /// density, the exact path must be provided (e.g. `images/2x/cat.png`).
  ///
  /// If [excludeFromSemantics] is true, then [semanticLabel] will be ignored.
  ///
  /// If `cacheWidth` or `cacheHeight` are provided, they indicate to the
  /// engine that the image must be decoded at the specified size. The image
  /// will be rendered to the constraints of the layout or [width] and [height]
  /// regardless of these parameters. These parameters are primarily intended
  /// to reduce the memory usage of [ImageCache].
  ///
  /// Either the [width] and [height] arguments should be specified, or the
  /// widget should be placed in a context that sets tight layout constraints.
  /// Otherwise, the image dimensions will change as the image is loaded, which
  /// will result in ugly layout changes.
  ///
  /// {@macro flutter.widgets.image.filterQualityParameter}
  ///
  /// {@tool snippet}
  ///
  /// Suppose that the project's `pubspec.yaml` file contains the following:
  ///
  /// ```yaml
  /// flutter:
  ///   assets:
  ///     - images/cat.png
  ///     - images/2x/cat.png
  ///     - images/3.5x/cat.png
  /// ```
  /// {@end-tool}
  ///
  /// On a screen with a device pixel ratio of 2.0, the following widget would
  /// render the `images/2x/cat.png` file:
  ///
  /// ```dart
  /// Image.asset('images/cat.png')
  /// ```
  ///
  /// This corresponds to the file that is in the project's `images/2x/`
  /// directory with the name `cat.png` (the paths are relative to the
  /// `pubspec.yaml` file).
  ///
  /// On a device with a 4.0 device pixel ratio, the `images/3.5x/cat.png` asset
  /// would be used. On a device with a 1.0 device pixel ratio, the
  /// `images/cat.png` resource would be used.
  ///
  /// The `images/cat.png` image can be omitted from disk (though it must still
  /// be present in the manifest). If it is omitted, then on a device with a 1.0
  /// device pixel ratio, the `images/2x/cat.png` image would be used instead.
  ///
  ///
  /// ## Assets in packages
  ///
  /// To create the widget with an asset from a package, the [package] argument
  /// must be provided. For instance, suppose a package called `my_icons` has
  /// `icons/heart.png` .
  ///
  /// {@tool snippet}
  /// Then to display the image, use:
  ///
  /// ```dart
  /// Image.asset('icons/heart.png', package: 'my_icons')
  /// ```
  /// {@end-tool}
  ///
  /// Assets used by the package itself should also be displayed using the
  /// [package] argument as above.
  ///
  /// If the desired asset is specified in the `pubspec.yaml` of the package, it
  /// is bundled automatically with the app. In particular, assets used by the
  /// package itself must be specified in its `pubspec.yaml`.
  ///
  /// A package can also choose to have assets in its 'lib/' folder that are not
  /// specified in its `pubspec.yaml`. In this case for those images to be
  /// bundled, the app has to specify which ones to include. For instance a
  /// package named `fancy_backgrounds` could have:
  ///
  ///     lib/backgrounds/background1.png
  ///     lib/backgrounds/background2.png
  ///     lib/backgrounds/background3.png
  ///
  /// To include, say the first image, the `pubspec.yaml` of the app should
  /// specify it in the assets section:
  ///
  /// ```yaml
  ///   assets:
  ///     - packages/fancy_backgrounds/backgrounds/background1.png
  /// ```
  ///
  /// The `lib/` is implied, so it should not be included in the asset path.
  ///
  ///
  /// See also:
  ///
  ///  * [AssetImage], which is used to implement the behavior when the scale is
  ///    omitted.
  ///  * [ExactAssetImage], which is used to implement the behavior when the
  ///    scale is present.
  ///  * <https://docs.flutter.dev/ui/assets/assets-and-images>, an introduction to assets in
  ///    Flutter.
  Image.asset(
    String name, {
    super.key,
    AssetBundle? bundle,
    this.frameBuilder,
    this.errorBuilder,
    this.semanticLabel,
    this.excludeFromSemantics = false,
    double? scale,
    this.width,
    this.height,
    this.color,
    this.opacity,
    this.colorBlendMode,
    this.fit,
    this.alignment = Alignment.center,
    this.repeat = ImageRepeat.noRepeat,
    this.centerSlice,
    this.matchTextDirection = false,
    this.gaplessPlayback = false,
    this.isAntiAlias = false,
    String? package,
    this.filterQuality = FilterQuality.medium,
    int? cacheWidth,
    int? cacheHeight,
  }) : image = ResizeImage.resizeIfNeeded(
         cacheWidth,
         cacheHeight,
         scale != null
             ? ExactAssetImage(name, bundle: bundle, scale: scale, package: package)
             : AssetImage(name, bundle: bundle, package: package),
       ),
       loadingBuilder = null,
       assert(cacheWidth == null || cacheWidth > 0),
       assert(cacheHeight == null || cacheHeight > 0);

  /// Creates a widget that displays an [ImageStream] obtained from a [Uint8List].
  ///
  /// The `bytes` argument specifies encoded image bytes, which can be encoded
  /// in any of the following supported image formats:
  /// {@macro dart.ui.imageFormats}
  ///
  /// The `scale` argument specifies the linear scale factor for drawing this
  /// image at its intended size and applies to both the width and the height.
  /// {@macro flutter.painting.imageInfo.scale}
  ///
  /// This only accepts compressed image formats (e.g. PNG). Uncompressed
  /// formats like rawRgba (the default format of [dart:ui.Image.toByteData])
  /// will lead to exceptions.
  ///
  /// Either the [width] and [height] arguments should be specified, or the
  /// widget should be placed in a context that sets tight layout constraints.
  /// Otherwise, the image dimensions will change as the image is loaded, which
  /// will result in ugly layout changes.
  ///
  /// {@macro flutter.widgets.image.filterQualityParameter}
  ///
  /// If [excludeFromSemantics] is true, then [semanticLabel] will be ignored.
  ///
  /// If `cacheWidth` or `cacheHeight` are provided, they indicate to the
  /// engine that the image must be decoded at the specified size. The image
  /// will be rendered to the constraints of the layout or [width] and [height]
  /// regardless of these parameters. These parameters are primarily intended
  /// to reduce the memory usage of [ImageCache].
  Image.memory(
    Uint8List bytes, {
    super.key,
    double scale = 1.0,
    this.frameBuilder,
    this.errorBuilder,
    this.semanticLabel,
    this.excludeFromSemantics = false,
    this.width,
    this.height,
    this.color,
    this.opacity,
    this.colorBlendMode,
    this.fit,
    this.alignment = Alignment.center,
    this.repeat = ImageRepeat.noRepeat,
    this.centerSlice,
    this.matchTextDirection = false,
    this.gaplessPlayback = false,
    this.isAntiAlias = false,
    this.filterQuality = FilterQuality.medium,
    int? cacheWidth,
    int? cacheHeight,
  }) : image = ResizeImage.resizeIfNeeded(
         cacheWidth,
         cacheHeight,
         MemoryImage(bytes, scale: scale),
       ),
       loadingBuilder = null,
       assert(cacheWidth == null || cacheWidth > 0),
       assert(cacheHeight == null || cacheHeight > 0);

  /// The image to display.
  final ImageProvider image;

  /// A builder function responsible for creating the widget that represents
  /// this image.
  ///
  /// If this is null, this widget will display an image that is painted as
  /// soon as the first image frame is available (and will appear to "pop" in
  /// if it becomes available asynchronously). Callers might use this builder to
  /// add effects to the image (such as fading the image in when it becomes
  /// available) or to display a placeholder widget while the image is loading.
  ///
  /// For more information on how to interpret the arguments that are passed to
  /// this builder, see the documentation on [ImageFrameBuilder].
  ///
  /// To have finer-grained control over the way that an image's loading
  /// progress is communicated to the user, see [loadingBuilder].
  ///
  /// ## Chaining with [loadingBuilder]
  ///
  /// If a [loadingBuilder] has _also_ been specified for an image, the two
  /// builders will be chained together: the _result_ of this builder will
  /// be passed as the `child` argument to the [loadingBuilder]. For example,
  /// consider the following builders used in conjunction:
  ///
  /// {@template flutter.widgets.Image.frameBuilder.chainedBuildersExample}
  /// ```dart
  /// Image(
  ///   image: _image,
  ///   frameBuilder: (BuildContext context, Widget child, int? frame, bool? wasSynchronouslyLoaded) {
  ///     return Padding(
  ///       padding: const EdgeInsets.all(8.0),
  ///       child: child,
  ///     );
  ///   },
  ///   loadingBuilder: (BuildContext context, Widget child, ImageChunkEvent? loadingProgress) {
  ///     return Center(child: child);
  ///   },
  /// )
  /// ```
  ///
  /// In this example, the widget hierarchy will contain the following:
  ///
  /// ```dart
  /// Center(
  ///   child: Padding(
  ///     padding: const EdgeInsets.all(8.0),
  ///     child: image,
  ///   ),
  /// ),
  /// ```
  /// {@endtemplate}
  ///
  /// {@tool dartpad}
  /// The following sample demonstrates how to use this builder to implement an
  /// image that fades in once it's been loaded.
  ///
  /// This sample contains a limited subset of the functionality that the
  /// [FadeInImage] widget provides out of the box.
  ///
  /// ** See code in examples/api/lib/widgets/image/image.frame_builder.0.dart **
  /// {@end-tool}
  final ImageFrameBuilder? frameBuilder;

  /// A builder that specifies the widget to display to the user while an image
  /// is still loading.
  ///
  /// If this is null, and the image is loaded incrementally (e.g. over a
  /// network), the user will receive no indication of the progress as the
  /// bytes of the image are loaded.
  ///
  /// For more information on how to interpret the arguments that are passed to
  /// this builder, see the documentation on [ImageLoadingBuilder].
  ///
  /// ## Performance implications
  ///
  /// If a [loadingBuilder] is specified for an image, the [Image] widget is
  /// likely to be rebuilt on every
  /// [rendering pipeline frame](rendering/RendererBinding/drawFrame.html) until
  /// the image has loaded. This is useful for cases such as displaying a loading
  /// progress indicator, but for simpler cases such as displaying a placeholder
  /// widget that doesn't depend on the loading progress (e.g. static "loading"
  /// text), [frameBuilder] will likely work and not incur as much cost.
  ///
  /// ## Chaining with [frameBuilder]
  ///
  /// If a [frameBuilder] has _also_ been specified for an image, the two
  /// builders will be chained together: the `child` argument to this
  /// builder will contain the _result_ of the [frameBuilder]. For example,
  /// consider the following builders used in conjunction:
  ///
  /// {@macro flutter.widgets.Image.frameBuilder.chainedBuildersExample}
  ///
  /// {@tool dartpad}
  /// The following sample uses [loadingBuilder] to show a
  /// [CircularProgressIndicator] while an image loads over the network.
  ///
  /// ** See code in examples/api/lib/widgets/image/image.loading_builder.0.dart **
  /// {@end-tool}
  ///
  /// Run against a real-world image on a slow network, the previous example
  /// renders the following loading progress indicator while the image loads
  /// before rendering the completed image.
  ///
  /// {@animation 400 400 https://flutter.github.io/assets-for-api-docs/assets/widgets/loading_progress_image.mp4}
  final ImageLoadingBuilder? loadingBuilder;

  /// A builder function that is called if an error occurs during image loading.
  ///
  /// If this builder is not provided, any exceptions will be reported to
  /// [FlutterError.onError]. If it is provided, the caller should either handle
  /// the exception by providing a replacement widget, or rethrow the exception.
  ///
  /// {@tool dartpad}
  /// The following sample uses [errorBuilder] to show a '😢' in place of the
  /// image that fails to load, and prints the error to the console.
  ///
  /// ** See code in examples/api/lib/widgets/image/image.error_builder.0.dart **
  /// {@end-tool}
  final ImageErrorWidgetBuilder? errorBuilder;

  /// If non-null, require the image to have this width (in logical pixels).
  ///
  /// If null, the image will pick a size that best preserves its intrinsic
  /// aspect ratio.
  ///
  /// It is strongly recommended that either both the [width] and the [height]
  /// be specified, or that the widget be placed in a context that sets tight
  /// layout constraints, so that the image does not change size as it loads.
  /// Consider using [fit] to adapt the image's rendering to fit the given width
  /// and height if the exact image dimensions are not known in advance.
  final double? width;

  /// If non-null, require the image to have this height (in logical pixels).
  ///
  /// If null, the image will pick a size that best preserves its intrinsic
  /// aspect ratio.
  ///
  /// It is strongly recommended that either both the [width] and the [height]
  /// be specified, or that the widget be placed in a context that sets tight
  /// layout constraints, so that the image does not change size as it loads.
  /// Consider using [fit] to adapt the image's rendering to fit the given width
  /// and height if the exact image dimensions are not known in advance.
  final double? height;

  /// If non-null, this color is blended with each image pixel using [colorBlendMode].
  final Color? color;

  /// If non-null, the value from the [Animation] is multiplied with the opacity
  /// of each image pixel before painting onto the canvas.
  ///
  /// This is more efficient than using [FadeTransition] to change the opacity
  /// of an image, since this avoids creating a new composited layer. Composited
  /// layers may double memory usage as the image is painted onto an offscreen
  /// render target.
  ///
  /// See also:
  ///
  ///  * [AlwaysStoppedAnimation], which allows you to create an [Animation]
  ///    from a single opacity value.
  final Animation<double>? opacity;

  /// The rendering quality of the image.
  ///
  /// {@template flutter.widgets.image.filterQuality}
  /// If the image is of a high quality and its pixels are perfectly aligned
  /// with the physical screen pixels, extra quality enhancement may not be
  /// necessary. If so, then [FilterQuality.none] would be the most efficient.
  ///
  /// If the pixels are not perfectly aligned with the screen pixels, or if the
  /// image itself is of a low quality, [FilterQuality.none] may produce
  /// undesirable artifacts. Consider using other [FilterQuality] values to
  /// improve the rendered image quality in this case. Pixels may be misaligned
  /// with the screen pixels as a result of transforms or scaling.
  ///
  /// Defaults to [FilterQuality.medium].
  ///
  /// See also:
  ///
  ///  * [FilterQuality], the enum containing all possible filter quality
  ///    options.
  /// {@endtemplate}
  final FilterQuality filterQuality;

  /// Used to combine [color] with this image.
  ///
  /// The default is [BlendMode.srcIn]. In terms of the blend mode, [color] is
  /// the source and this image is the destination.
  ///
  /// See also:
  ///
  ///  * [BlendMode], which includes an illustration of the effect of each blend mode.
  final BlendMode? colorBlendMode;

  /// How to inscribe the image into the space allocated during layout.
  ///
  /// The default varies based on the other fields. See the discussion at
  /// [paintImage].
  final BoxFit? fit;

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
  /// To display a subpart of an image, consider using a [CustomPainter] and
  /// [Canvas.drawImageRect].
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

  /// The center slice for a nine-patch image.
  ///
  /// The region of the image inside the center slice will be stretched both
  /// horizontally and vertically to fit the image into its destination. The
  /// region of the image above and below the center slice will be stretched
  /// only horizontally and the region of the image to the left and right of
  /// the center slice will be stretched only vertically.
  final Rect? centerSlice;

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

  /// Whether to continue showing the old image (true), or briefly show nothing
  /// (false), when the image provider changes. The default value is false.
  ///
  /// ## Design discussion
  ///
  /// ### Why is the default value of [gaplessPlayback] false?
  ///
  /// Having the default value of [gaplessPlayback] be false helps prevent
  /// situations where stale or misleading information might be presented.
  /// Consider the following case:
  ///
  /// We have constructed a 'Person' widget that displays an avatar [Image] of
  /// the currently loaded person along with their name. We could request for a
  /// new person to be loaded into the widget at any time. Suppose we have a
  /// person currently loaded and the widget loads a new person. What happens
  /// if the [Image] fails to load?
  ///
  /// * Option A ([gaplessPlayback] = false): The new person's name is coupled
  /// with a blank image.
  ///
  /// * Option B ([gaplessPlayback] = true): The widget displays the avatar of
  /// the previous person and the name of the newly loaded person.
  ///
  /// This is why the default value is false. Most of the time, when you change
  /// the image provider you're not just changing the image, you're removing the
  /// old widget and adding a new one and not expecting them to have any
  /// relationship. With [gaplessPlayback] on you might accidentally break this
  /// expectation and re-use the old widget.
  final bool gaplessPlayback;

  /// A Semantic description of the image.
  ///
  /// Used to provide a description of the image to TalkBack on Android, and
  /// VoiceOver on iOS.
  final String? semanticLabel;

  /// Whether to exclude this image from semantics.
  ///
  /// Useful for images which do not contribute meaningful information to an
  /// application.
  final bool excludeFromSemantics;

  /// Whether to paint the image with anti-aliasing.
  ///
  /// Anti-aliasing alleviates the sawtooth artifact when the image is rotated.
  final bool isAntiAlias;

  @override
  State<Image> createState() => _ImageState();

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(DiagnosticsProperty<ImageProvider>('image', image));
    properties.add(DiagnosticsProperty<Function>('frameBuilder', frameBuilder));
    properties.add(DiagnosticsProperty<Function>('loadingBuilder', loadingBuilder));
    properties.add(DoubleProperty('width', width, defaultValue: null));
    properties.add(DoubleProperty('height', height, defaultValue: null));
    properties.add(ColorProperty('color', color, defaultValue: null));
    properties.add(DiagnosticsProperty<Animation<double>?>('opacity', opacity, defaultValue: null));
    properties.add(EnumProperty<BlendMode>('colorBlendMode', colorBlendMode, defaultValue: null));
    properties.add(EnumProperty<BoxFit>('fit', fit, defaultValue: null));
    properties.add(
      DiagnosticsProperty<AlignmentGeometry>('alignment', alignment, defaultValue: null),
    );
    properties.add(EnumProperty<ImageRepeat>('repeat', repeat, defaultValue: ImageRepeat.noRepeat));
    properties.add(DiagnosticsProperty<Rect>('centerSlice', centerSlice, defaultValue: null));
    properties.add(
      FlagProperty('matchTextDirection', value: matchTextDirection, ifTrue: 'match text direction'),
    );
    properties.add(StringProperty('semanticLabel', semanticLabel, defaultValue: null));
    properties.add(DiagnosticsProperty<bool>('this.excludeFromSemantics', excludeFromSemantics));
    properties.add(EnumProperty<FilterQuality>('filterQuality', filterQuality));
  }
}

class _ImageState extends State<Image> with WidgetsBindingObserver {
  ImageStream? _imageStream;
  ImageInfo? _imageInfo;
  ImageChunkEvent? _loadingProgress;
  bool _isListeningToStream = false;
  late bool _invertColors;
  int? _frameNumber;
  bool _wasSynchronouslyLoaded = false;
  late DisposableBuildContext<State<Image>> _scrollAwareContext;
  Object? _lastException;
  StackTrace? _lastStack;
  ImageStreamCompleterHandle? _completerHandle;

  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
    _scrollAwareContext = DisposableBuildContext<State<Image>>(this);
  }

  @override
  void dispose() {
    assert(_imageStream != null);
    WidgetsBinding.instance.removeObserver(this);
    _stopListeningToStream();
    _completerHandle?.dispose();
    _scrollAwareContext.dispose();
    _replaceImage(info: null);
    super.dispose();
  }

  @override
  void didChangeDependencies() {
    _updateInvertColors();
    _resolveImage();

    if (TickerMode.of(context)) {
      _listenToStream();
    } else {
      _stopListeningToStream(keepStreamAlive: true);
    }

    super.didChangeDependencies();
  }

  @override
  void didUpdateWidget(Image oldWidget) {
    super.didUpdateWidget(oldWidget);
    if (_isListeningToStream &&
        (widget.loadingBuilder == null) != (oldWidget.loadingBuilder == null)) {
      final ImageStreamListener oldListener = _getListener();
      _imageStream!.addListener(_getListener(recreateListener: true));
      _imageStream!.removeListener(oldListener);
    }
    if (widget.image != oldWidget.image) {
      _resolveImage();
    }
  }

  @override
  void didChangeAccessibilityFeatures() {
    super.didChangeAccessibilityFeatures();
    setState(() {
      _updateInvertColors();
    });
  }

  @override
  void reassemble() {
    _resolveImage(); // in case the image cache was flushed
    super.reassemble();
  }

  void _updateInvertColors() {
    _invertColors =
        MediaQuery.maybeInvertColorsOf(context) ??
        SemanticsBinding.instance.accessibilityFeatures.invertColors;
  }

  void _resolveImage() {
    final ScrollAwareImageProvider provider = ScrollAwareImageProvider<Object>(
      context: _scrollAwareContext,
      imageProvider: widget.image,
    );
    final ImageStream newStream = provider.resolve(
      createLocalImageConfiguration(
        context,
        size:
            widget.width != null && widget.height != null
                ? Size(widget.width!, widget.height!)
                : null,
      ),
    );
    _updateSourceStream(newStream);
  }

  ImageStreamListener? _imageStreamListener;
  ImageStreamListener _getListener({bool recreateListener = false}) {
    if (_imageStreamListener == null || recreateListener) {
      _lastException = null;
      _lastStack = null;
      _imageStreamListener = ImageStreamListener(
        _handleImageFrame,
        onChunk: widget.loadingBuilder == null ? null : _handleImageChunk,
        onError:
            widget.errorBuilder != null || kDebugMode
                ? (Object error, StackTrace? stackTrace) {
                  setState(() {
                    _lastException = error;
                    _lastStack = stackTrace;
                  });
                  assert(() {
                    if (widget.errorBuilder == null) {
                      // ignore: only_throw_errors, since we're just proxying the error.
                      throw error; // Ensures the error message is printed to the console.
                    }
                    return true;
                  }());
                }
                : null,
      );
    }
    return _imageStreamListener!;
  }

  void _handleImageFrame(ImageInfo imageInfo, bool synchronousCall) {
    setState(() {
      _replaceImage(info: imageInfo);
      _loadingProgress = null;
      _lastException = null;
      _lastStack = null;
      _frameNumber = _frameNumber == null ? 0 : _frameNumber! + 1;
      _wasSynchronouslyLoaded = _wasSynchronouslyLoaded | synchronousCall;
    });
  }

  void _handleImageChunk(ImageChunkEvent event) {
    assert(widget.loadingBuilder != null);
    setState(() {
      _loadingProgress = event;
      _lastException = null;
      _lastStack = null;
    });
  }

  void _replaceImage({required ImageInfo? info}) {
    final ImageInfo? oldImageInfo = _imageInfo;
    SchedulerBinding.instance.addPostFrameCallback(
      (_) => oldImageInfo?.dispose(),
      debugLabel: 'Image.disposeOldInfo',
    );
    _imageInfo = info;
  }

  // Updates _imageStream to newStream, and moves the stream listener
  // registration from the old stream to the new stream (if a listener was
  // registered).
  void _updateSourceStream(ImageStream newStream) {
    if (_imageStream?.key == newStream.key) {
      return;
    }

    if (_isListeningToStream) {
      _imageStream!.removeListener(_getListener());
    }

    if (!widget.gaplessPlayback) {
      setState(() {
        _replaceImage(info: null);
      });
    }

    setState(() {
      _loadingProgress = null;
      _frameNumber = null;
      _wasSynchronouslyLoaded = false;
    });

    _imageStream = newStream;
    if (_isListeningToStream) {
      _imageStream!.addListener(_getListener());
    }
  }

  void _listenToStream() {
    if (_isListeningToStream) {
      return;
    }

    _imageStream!.addListener(_getListener());
    _completerHandle?.dispose();
    _completerHandle = null;

    _isListeningToStream = true;
  }

  /// Stops listening to the image stream, if this state object has attached a
  /// listener.
  ///
  /// If the listener from this state is the last listener on the stream, the
  /// stream will be disposed. To keep the stream alive, set `keepStreamAlive`
  /// to true, which create [ImageStreamCompleterHandle] to keep the completer
  /// alive and is compatible with the [TickerMode] being off.
  void _stopListeningToStream({bool keepStreamAlive = false}) {
    if (!_isListeningToStream) {
      return;
    }

    if (keepStreamAlive && _completerHandle == null && _imageStream?.completer != null) {
      _completerHandle = _imageStream!.completer!.keepAlive();
    }

    _imageStream!.removeListener(_getListener());
    _isListeningToStream = false;
  }

  Widget _debugBuildErrorWidget(BuildContext context, Object error) {
    return Stack(
      alignment: Alignment.center,
      children: <Widget>[
        const Positioned.fill(child: Placeholder(color: Color(0xCF8D021F))),
        Padding(
          padding: const EdgeInsets.all(4.0),
          child: FittedBox(
            child: Text(
              '$error',
              textAlign: TextAlign.center,
              textDirection: TextDirection.ltr,
              style: const TextStyle(shadows: <Shadow>[Shadow(blurRadius: 1.0)]),
            ),
          ),
        ),
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    if (_lastException != null) {
      if (widget.errorBuilder != null) {
        return widget.errorBuilder!(context, _lastException!, _lastStack);
      }
      if (kDebugMode) {
        return _debugBuildErrorWidget(context, _lastException!);
      }
    }

    late Widget result;
    if (_imageInfo case final WebImageInfo webImage) {
      // TODO(harryterkelsen): Support the remaining properties that are
      // supported by `RawImage` but not `RawWebImage`. See the following issue
      // above for a discussion of the missing properties and suggestions for
      // how they can be implemented, https://github.com/flutter/flutter/issues/159565.
      result = RawWebImage(
        image: webImage,
        debugImageLabel: _imageInfo?.debugLabel,
        width: widget.width,
        height: widget.height,
        fit: widget.fit,
        alignment: widget.alignment,
        matchTextDirection: widget.matchTextDirection,
      );
    } else {
      result = RawImage(
        // Do not clone the image, because RawImage is a stateless wrapper.
        // The image will be disposed by this state object when it is not needed
        // anymore, such as when it is unmounted or when the image stream pushes
        // a new image.
        image: _imageInfo?.image,
        debugImageLabel: _imageInfo?.debugLabel,
        width: widget.width,
        height: widget.height,
        scale: _imageInfo?.scale ?? 1.0,
        color: widget.color,
        opacity: widget.opacity,
        colorBlendMode: widget.colorBlendMode,
        fit: widget.fit,
        alignment: widget.alignment,
        repeat: widget.repeat,
        centerSlice: widget.centerSlice,
        matchTextDirection: widget.matchTextDirection,
        invertColors: _invertColors,
        isAntiAlias: widget.isAntiAlias,
        filterQuality: widget.filterQuality,
      );
    }

    if (!widget.excludeFromSemantics) {
      result = Semantics(
        container: widget.semanticLabel != null,
        image: true,
        label: widget.semanticLabel ?? '',
        child: result,
      );
    }

    if (widget.frameBuilder != null) {
      result = widget.frameBuilder!(context, result, _frameNumber, _wasSynchronouslyLoaded);
    }

    if (widget.loadingBuilder != null) {
      result = widget.loadingBuilder!(context, result, _loadingProgress);
    }

    return result;
  }

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(DiagnosticsProperty<ImageStream>('stream', _imageStream));
    description.add(DiagnosticsProperty<ImageInfo>('pixels', _imageInfo));
    description.add(DiagnosticsProperty<ImageChunkEvent>('loadingProgress', _loadingProgress));
    description.add(DiagnosticsProperty<int>('frameNumber', _frameNumber));
    description.add(DiagnosticsProperty<bool>('wasSynchronouslyLoaded', _wasSynchronouslyLoaded));
  }
}
