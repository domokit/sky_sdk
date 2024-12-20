// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Examples can assume:
// late BuildContext context;

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:async';
import 'dart:io';
import 'dart:math' as math;
import 'dart:ui' as ui;
import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';

import '_network_image_io.dart'
    if (dart.library.js_util) '_network_image_web.dart'
    as network_image;
import 'binding.dart';
import 'image_cache.dart';
import 'image_stream.dart';

/// Signature for the callback taken by [ImageProvider._createErrorHandlerAndKey].
typedef _KeyAndErrorHandlerCallback<T> = void Function(T key, ImageErrorListener handleError);

/// Signature used for error handling by [ImageProvider._createErrorHandlerAndKey].
typedef _AsyncKeyErrorHandler<T> =
    Future<void> Function(T key, Object exception, StackTrace? stack);

/// Configuration information passed to the [ImageProvider.resolve] method to
/// select a specific image.
///
/// See also:
///
///  * [createLocalImageConfiguration], which creates an [ImageConfiguration]
///    based on ambient configuration in a [Widget] environment.
///  * [ImageProvider], which uses [ImageConfiguration] objects to determine
///    which image to obtain.
@immutable
class ImageConfiguration {
  /// Creates an object holding the configuration information for an [ImageProvider].
  ///
  /// All the arguments are optional. Configuration information is merely
  /// advisory and best-effort.
  const ImageConfiguration({
    this.bundle,
    this.devicePixelRatio,
    this.locale,
    this.textDirection,
    this.size,
    this.platform,
  });

  /// Creates an object holding the configuration information for an [ImageProvider].
  ///
  /// All the arguments are optional. Configuration information is merely
  /// advisory and best-effort.
  ImageConfiguration copyWith({
    AssetBundle? bundle,
    double? devicePixelRatio,
    ui.Locale? locale,
    TextDirection? textDirection,
    Size? size,
    TargetPlatform? platform,
  }) {
    return ImageConfiguration(
      bundle: bundle ?? this.bundle,
      devicePixelRatio: devicePixelRatio ?? this.devicePixelRatio,
      locale: locale ?? this.locale,
      textDirection: textDirection ?? this.textDirection,
      size: size ?? this.size,
      platform: platform ?? this.platform,
    );
  }

  /// The preferred [AssetBundle] to use if the [ImageProvider] needs one and
  /// does not have one already selected.
  final AssetBundle? bundle;

  /// The device pixel ratio where the image will be shown.
  final double? devicePixelRatio;

  /// The language and region for which to select the image.
  final ui.Locale? locale;

  /// The reading direction of the language for which to select the image.
  final TextDirection? textDirection;

  /// The size at which the image will be rendered.
  final Size? size;

  /// The [TargetPlatform] for which assets should be used. This allows images
  /// to be specified in a platform-neutral fashion yet use different assets on
  /// different platforms, to match local conventions e.g. for color matching or
  /// shadows.
  final TargetPlatform? platform;

  /// An image configuration that provides no additional information.
  ///
  /// Useful when resolving an [ImageProvider] without any context.
  static const ImageConfiguration empty = ImageConfiguration();

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ImageConfiguration &&
        other.bundle == bundle &&
        other.devicePixelRatio == devicePixelRatio &&
        other.locale == locale &&
        other.textDirection == textDirection &&
        other.size == size &&
        other.platform == platform;
  }

  @override
  int get hashCode => Object.hash(bundle, devicePixelRatio, locale, size, platform);

  @override
  String toString() {
    final StringBuffer result = StringBuffer();
    result.write('ImageConfiguration(');
    bool hasArguments = false;
    if (bundle != null) {
      result.write('bundle: $bundle');
      hasArguments = true;
    }
    if (devicePixelRatio != null) {
      if (hasArguments) {
        result.write(', ');
      }
      result.write('devicePixelRatio: ${devicePixelRatio!.toStringAsFixed(1)}');
      hasArguments = true;
    }
    if (locale != null) {
      if (hasArguments) {
        result.write(', ');
      }
      result.write('locale: $locale');
      hasArguments = true;
    }
    if (textDirection != null) {
      if (hasArguments) {
        result.write(', ');
      }
      result.write('textDirection: $textDirection');
      hasArguments = true;
    }
    if (size != null) {
      if (hasArguments) {
        result.write(', ');
      }
      result.write('size: $size');
      hasArguments = true;
    }
    if (platform != null) {
      if (hasArguments) {
        result.write(', ');
      }
      result.write('platform: ${platform!.name}');
      hasArguments = true;
    }
    result.write(')');
    return result.toString();
  }
}

/// Performs the decode process for use in [ImageProvider.loadBuffer].
///
/// This callback allows decoupling of the `cacheWidth`, `cacheHeight`, and
/// `allowUpscaling` parameters from implementations of [ImageProvider] that do
/// not expose them.
///
/// See also:
///
///  * [ResizeImage], which uses this to override the `cacheWidth`,
///    `cacheHeight`, and `allowUpscaling` parameters.
@Deprecated(
  'Use ImageDecoderCallback with ImageProvider.loadImage instead. '
  'This feature was deprecated after v3.7.0-1.4.pre.',
)
typedef DecoderBufferCallback =
    Future<ui.Codec> Function(
      ui.ImmutableBuffer buffer, {
      int? cacheWidth,
      int? cacheHeight,
      bool allowUpscaling,
    });

// Method signature for _loadAsync decode callbacks.
typedef _SimpleDecoderCallback = Future<ui.Codec> Function(ui.ImmutableBuffer buffer);

/// Performs the decode process for use in [ImageProvider.loadImage].
///
/// This callback allows decoupling of the `getTargetSize` parameter from
/// implementations of [ImageProvider] that do not expose it.
///
/// See also:
///
///  * [ResizeImage], which uses this to load images at specific sizes.
typedef ImageDecoderCallback =
    Future<ui.Codec> Function(
      ui.ImmutableBuffer buffer, {
      ui.TargetImageSizeCallback? getTargetSize,
    });

/// Identifies an image without committing to the precise final asset. This
/// allows a set of images to be identified and for the precise image to later
/// be resolved based on the environment, e.g. the device pixel ratio.
///
/// To obtain an [ImageStream] from an [ImageProvider], call [resolve],
/// passing it an [ImageConfiguration] object.
///
/// [ImageProvider] uses the global [imageCache] to cache images.
///
/// The type argument `T` is the type of the object used to represent a resolved
/// configuration. This is also the type used for the key in the image cache. It
/// should be immutable and implement the [==] operator and the [hashCode]
/// getter. Subclasses should subclass a variant of [ImageProvider] with an
/// explicit `T` type argument.
///
/// The type argument does not have to be specified when using the type as an
/// argument (where any image provider is acceptable).
///
/// The following image formats are supported: {@macro dart.ui.imageFormats}
///
/// ## Lifecycle of resolving an image
///
/// The [ImageProvider] goes through the following lifecycle to resolve an
/// image, once the [resolve] method is called:
///
///   1. Create an [ImageStream] using [createStream] to return to the caller.
///      This stream will be used to communicate back to the caller when the
///      image is decoded and ready to display, or when an error occurs.
///   2. Obtain the key for the image using [obtainKey].
///      Calling this method can throw exceptions into the zone asynchronously
///      or into the call stack synchronously. To handle that, an error handler
///      is created that catches both synchronous and asynchronous errors, to
///      make sure errors can be routed to the correct consumers.
///      The error handler is passed on to [resolveStreamForKey] and the
///      [ImageCache].
///   3. If the key is successfully obtained, schedule resolution of the image
///      using that key. This is handled by [resolveStreamForKey]. That method
///      may fizzle if it determines the image is no longer necessary, use the
///      provided [ImageErrorListener] to report an error, set the completer
///      from the cache if possible, or call [loadImage] to fetch the encoded image
///      bytes and schedule decoding.
///   4. The [loadImage] method is responsible for both fetching the encoded bytes
///      and decoding them using the provided [ImageDecoderCallback]. It is called
///      in a context that uses the [ImageErrorListener] to report errors back.
///
/// Subclasses normally only have to implement the [loadImage] and [obtainKey]
/// methods. A subclass that needs finer grained control over the [ImageStream]
/// type must override [createStream]. A subclass that needs finer grained
/// control over the resolution, such as delaying calling [loadImage], must override
/// [resolveStreamForKey].
///
/// The [resolve] method is marked as [nonVirtual] so that [ImageProvider]s can
/// be properly composed, and so that the base class can properly set up error
/// handling for subsequent methods.
///
/// ## Using an [ImageProvider]
///
/// {@tool snippet}
///
/// The following shows the code required to write a widget that fully conforms
/// to the [ImageProvider] and [Widget] protocols. (It is essentially a
/// bare-bones version of the [widgets.Image] widget.)
///
/// ```dart
/// class MyImage extends StatefulWidget {
///   const MyImage({
///     super.key,
///     required this.imageProvider,
///   });
///
///   final ImageProvider imageProvider;
///
///   @override
///   State<MyImage> createState() => _MyImageState();
/// }
///
/// class _MyImageState extends State<MyImage> {
///   ImageStream? _imageStream;
///   ImageInfo? _imageInfo;
///
///   @override
///   void didChangeDependencies() {
///     super.didChangeDependencies();
///     // We call _getImage here because createLocalImageConfiguration() needs to
///     // be called again if the dependencies changed, in case the changes relate
///     // to the DefaultAssetBundle, MediaQuery, etc, which that method uses.
///     _getImage();
///   }
///
///   @override
///   void didUpdateWidget(MyImage oldWidget) {
///     super.didUpdateWidget(oldWidget);
///     if (widget.imageProvider != oldWidget.imageProvider) {
///       _getImage();
///     }
///   }
///
///   void _getImage() {
///     final ImageStream? oldImageStream = _imageStream;
///     _imageStream = widget.imageProvider.resolve(createLocalImageConfiguration(context));
///     if (_imageStream!.key != oldImageStream?.key) {
///       // If the keys are the same, then we got the same image back, and so we don't
///       // need to update the listeners. If the key changed, though, we must make sure
///       // to switch our listeners to the new image stream.
///       final ImageStreamListener listener = ImageStreamListener(_updateImage);
///       oldImageStream?.removeListener(listener);
///       _imageStream!.addListener(listener);
///     }
///   }
///
///   void _updateImage(ImageInfo imageInfo, bool synchronousCall) {
///     setState(() {
///       // Trigger a build whenever the image changes.
///       _imageInfo?.dispose();
///       _imageInfo = imageInfo;
///     });
///   }
///
///   @override
///   void dispose() {
///     _imageStream?.removeListener(ImageStreamListener(_updateImage));
///     _imageInfo?.dispose();
///     _imageInfo = null;
///     super.dispose();
///   }
///
///   @override
///   Widget build(BuildContext context) {
///     return RawImage(
///       image: _imageInfo?.image, // this is a dart:ui Image object
///       scale: _imageInfo?.scale ?? 1.0,
///     );
///   }
/// }
/// ```
/// {@end-tool}
///
/// ## Creating an [ImageProvider]
///
/// {@tool dartpad}
/// In this example, a variant of [NetworkImage] is created that passes all the
/// [ImageConfiguration] information (locale, platform, size, etc) to the server
/// using query arguments in the image URL.
///
/// ** See code in examples/api/lib/painting/image_provider/image_provider.0.dart **
/// {@end-tool}
@optionalTypeArgs
abstract class ImageProvider<T extends Object> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const ImageProvider();

  /// Resolves this image provider using the given `configuration`, returning
  /// an [ImageStream].
  ///
  /// This is the public entry-point of the [ImageProvider] class hierarchy.
  ///
  /// Subclasses should implement [obtainKey] and [loadImage], which are used by
  /// this method. If they need to change the implementation of [ImageStream]
  /// used, they should override [createStream]. If they need to manage the
  /// actual resolution of the image, they should override [resolveStreamForKey].
  ///
  /// See the Lifecycle documentation on [ImageProvider] for more information.
  @nonVirtual
  ImageStream resolve(ImageConfiguration configuration) {
    final ImageStream stream = createStream(configuration);
    // Load the key (potentially asynchronously), set up an error handling zone,
    // and call resolveStreamForKey.
    _createErrorHandlerAndKey(
      configuration,
      (T key, ImageErrorListener errorHandler) {
        resolveStreamForKey(configuration, stream, key, errorHandler);
      },
      (T? key, Object exception, StackTrace? stack) async {
        await null; // wait an event turn in case a listener has been added to the image stream.
        InformationCollector? collector;
        assert(() {
          collector =
              () => <DiagnosticsNode>[
                DiagnosticsProperty<ImageProvider>('Image provider', this),
                DiagnosticsProperty<ImageConfiguration>('Image configuration', configuration),
                DiagnosticsProperty<T>('Image key', key, defaultValue: null),
              ];
          return true;
        }());
        if (stream.completer == null) {
          stream.setCompleter(_ErrorImageCompleter());
        }
        stream.completer!.reportError(
          exception: exception,
          stack: stack,
          context: ErrorDescription('while resolving an image'),
          silent: true, // could be a network error or whatnot
          informationCollector: collector,
        );
      },
    );
    return stream;
  }

  /// Called by [resolve] to create the [ImageStream] it returns.
  ///
  /// Subclasses should override this instead of [resolve] if they need to
  /// return some subclass of [ImageStream]. The stream created here will be
  /// passed to [resolveStreamForKey].
  @protected
  ImageStream createStream(ImageConfiguration configuration) {
    return ImageStream();
  }

  /// Returns the cache location for the key that this [ImageProvider] creates.
  ///
  /// The location may be [ImageCacheStatus.untracked], indicating that this
  /// image provider's key is not available in the [ImageCache].
  ///
  /// If the `handleError` parameter is null, errors will be reported to
  /// [FlutterError.onError], and the method will return null.
  ///
  /// A completed return value of null indicates that an error has occurred.
  Future<ImageCacheStatus?> obtainCacheStatus({
    required ImageConfiguration configuration,
    ImageErrorListener? handleError,
  }) {
    final Completer<ImageCacheStatus?> completer = Completer<ImageCacheStatus?>();
    _createErrorHandlerAndKey(
      configuration,
      (T key, ImageErrorListener innerHandleError) {
        completer.complete(PaintingBinding.instance.imageCache.statusForKey(key));
      },
      (T? key, Object exception, StackTrace? stack) async {
        if (handleError != null) {
          handleError(exception, stack);
        } else {
          InformationCollector? collector;
          assert(() {
            collector =
                () => <DiagnosticsNode>[
                  DiagnosticsProperty<ImageProvider>('Image provider', this),
                  DiagnosticsProperty<ImageConfiguration>('Image configuration', configuration),
                  DiagnosticsProperty<T>('Image key', key, defaultValue: null),
                ];
            return true;
          }());
          FlutterError.reportError(
            FlutterErrorDetails(
              context: ErrorDescription('while checking the cache location of an image'),
              informationCollector: collector,
              exception: exception,
              stack: stack,
            ),
          );
          completer.complete();
        }
      },
    );
    return completer.future;
  }

  /// This method is used by both [resolve] and [obtainCacheStatus] to ensure
  /// that errors thrown during key creation are handled whether synchronous or
  /// asynchronous.
  void _createErrorHandlerAndKey(
    ImageConfiguration configuration,
    _KeyAndErrorHandlerCallback<T> successCallback,
    _AsyncKeyErrorHandler<T?> errorCallback,
  ) {
    T? obtainedKey;
    bool didError = false;
    Future<void> handleError(Object exception, StackTrace? stack) async {
      if (didError) {
        return;
      }
      if (!didError) {
        didError = true;
        errorCallback(obtainedKey, exception, stack);
      }
    }

    Future<T> key;
    try {
      key = obtainKey(configuration);
    } catch (error, stackTrace) {
      handleError(error, stackTrace);
      return;
    }
    key
        .then<void>((T key) {
          obtainedKey = key;
          try {
            successCallback(key, handleError);
          } catch (error, stackTrace) {
            handleError(error, stackTrace);
          }
        })
        .catchError(handleError);
  }

  /// Called by [resolve] with the key returned by [obtainKey].
  ///
  /// Subclasses should override this method rather than calling [obtainKey] if
  /// they need to use a key directly. The [resolve] method installs appropriate
  /// error handling guards so that errors will bubble up to the right places in
  /// the framework, and passes those guards along to this method via the
  /// [handleError] parameter.
  ///
  /// It is safe for the implementation of this method to call [handleError]
  /// multiple times if multiple errors occur, or if an error is thrown both
  /// synchronously into the current part of the stack and thrown into the
  /// enclosing [Zone].
  ///
  /// The default implementation uses the key to interact with the [ImageCache],
  /// calling [ImageCache.putIfAbsent] and notifying listeners of the [stream].
  /// Implementers that do not call super are expected to correctly use the
  /// [ImageCache].
  @protected
  void resolveStreamForKey(
    ImageConfiguration configuration,
    ImageStream stream,
    T key,
    ImageErrorListener handleError,
  ) {
    // This is an unusual edge case where someone has told us that they found
    // the image we want before getting to this method. We should avoid calling
    // load again, but still update the image cache with LRU information.
    if (stream.completer != null) {
      final ImageStreamCompleter? completer = PaintingBinding.instance.imageCache.putIfAbsent(
        key,
        () => stream.completer!,
        onError: handleError,
      );
      assert(identical(completer, stream.completer));
      return;
    }
    final ImageStreamCompleter? completer = PaintingBinding.instance.imageCache.putIfAbsent(
      key,
      () {
        ImageStreamCompleter result = loadImage(
          key,
          PaintingBinding.instance.instantiateImageCodecWithSize,
        );
        // This check exists as a fallback for backwards compatibility until the
        // deprecated `loadBuffer()` method is removed. Until then, ImageProvider
        // subclasses may have only overridden `loadBuffer()`, in which case the
        // base implementation of `loadWithSize()` will return a sentinel value
        // of type `_AbstractImageStreamCompleter`.
        if (result is _AbstractImageStreamCompleter) {
          result = loadBuffer(key, PaintingBinding.instance.instantiateImageCodecFromBuffer);
        }
        return result;
      },
      onError: handleError,
    );
    if (completer != null) {
      stream.setCompleter(completer);
    }
  }

  /// Evicts an entry from the image cache.
  ///
  /// Returns a [Future] which indicates whether the value was successfully
  /// removed.
  ///
  /// The [ImageProvider] used does not need to be the same instance that was
  /// passed to an [Image] widget, but it does need to create a key which is
  /// equal to one.
  ///
  /// The [cache] is optional and defaults to the global image cache.
  ///
  /// The [configuration] is optional and defaults to
  /// [ImageConfiguration.empty].
  ///
  /// {@tool snippet}
  ///
  /// The following sample code shows how an image loaded using the [Image]
  /// widget can be evicted using a [NetworkImage] with a matching URL.
  ///
  /// ```dart
  /// class MyWidget extends StatelessWidget {
  ///   const MyWidget({
  ///     super.key,
  ///     this.url = ' ... ',
  ///   });
  ///
  ///   final String url;
  ///
  ///   @override
  ///   Widget build(BuildContext context) {
  ///     return Image.network(url);
  ///   }
  ///
  ///   void evictImage() {
  ///     final NetworkImage provider = NetworkImage(url);
  ///     provider.evict().then<void>((bool success) {
  ///       if (success) {
  ///         debugPrint('removed image!');
  ///       }
  ///     });
  ///   }
  /// }
  /// ```
  /// {@end-tool}
  Future<bool> evict({
    ImageCache? cache,
    ImageConfiguration configuration = ImageConfiguration.empty,
  }) async {
    cache ??= imageCache;
    final T key = await obtainKey(configuration);
    return cache.evict(key);
  }

  /// Converts an [ImageProvider]'s settings plus an [ImageConfiguration] to a key
  /// that describes the precise image to load.
  ///
  /// The type of the key is determined by the subclass. It is a value that
  /// unambiguously identifies the image (_including its scale_) that the
  /// [loadImage] method will fetch. Different [ImageProvider]s given the same
  /// constructor arguments and [ImageConfiguration] objects should return keys
  /// that are '==' to each other (possibly by using a class for the key that
  /// itself implements [==]).
  ///
  /// If the result can be determined synchronously, this function should return
  /// a [SynchronousFuture]. This allows image resolution to progress
  /// synchronously during a frame rather than delaying image loading.
  Future<T> obtainKey(ImageConfiguration configuration);

  /// Converts a key into an [ImageStreamCompleter], and begins fetching the
  /// image.
  ///
  /// This method is deprecated. Implement [loadImage] instead.
  ///
  /// The [decode] callback provides the logic to obtain the codec for the
  /// image.
  ///
  /// See also:
  ///
  ///  * [ResizeImage], for modifying the key to account for cache dimensions.
  @protected
  @Deprecated(
    'Implement loadImage for image loading. '
    'This feature was deprecated after v3.7.0-1.4.pre.',
  )
  ImageStreamCompleter loadBuffer(T key, DecoderBufferCallback decode) {
    return _AbstractImageStreamCompleter();
  }

  /// Converts a key into an [ImageStreamCompleter], and begins fetching the
  /// image.
  ///
  /// For backwards-compatibility the default implementation of this method returns
  /// an object that will cause [resolveStreamForKey] to consult [loadBuffer].
  /// However, implementors of this interface should only override this method
  /// and not [loadBuffer], which is deprecated.
  ///
  /// The [decode] callback provides the logic to obtain the codec for the
  /// image.
  ///
  /// See also:
  ///
  ///  * [ResizeImage], for modifying the key to account for cache dimensions.
  // TODO(tvolkert): make abstract (https://github.com/flutter/flutter/issues/119209)
  @protected
  ImageStreamCompleter loadImage(T key, ImageDecoderCallback decode) {
    return _AbstractImageStreamCompleter();
  }

  @override
  String toString() => '${objectRuntimeType(this, 'ImageConfiguration')}()';
}

/// A class that exists to facilitate backwards compatibility in the transition
/// from [ImageProvider.loadBuffer] to [ImageProvider.loadImage]
class _AbstractImageStreamCompleter extends ImageStreamCompleter {}

/// Key for the image obtained by an [AssetImage] or [ExactAssetImage].
///
/// This is used to identify the precise resource in the [imageCache].
@immutable
class AssetBundleImageKey {
  /// Creates the key for an [AssetImage] or [AssetBundleImageProvider].
  const AssetBundleImageKey({required this.bundle, required this.name, required this.scale});

  /// The bundle from which the image will be obtained.
  ///
  /// The image is obtained by calling [AssetBundle.load] on the given [bundle]
  /// using the key given by [name].
  final AssetBundle bundle;

  /// The key to use to obtain the resource from the [bundle]. This is the
  /// argument passed to [AssetBundle.load].
  final String name;

  /// The scale to place in the [ImageInfo] object of the image.
  final double scale;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is AssetBundleImageKey &&
        other.bundle == bundle &&
        other.name == name &&
        other.scale == scale;
  }

  @override
  int get hashCode => Object.hash(bundle, name, scale);

  @override
  String toString() =>
      '${objectRuntimeType(this, 'AssetBundleImageKey')}(bundle: $bundle, name: "$name", scale: $scale)';
}

/// A subclass of [ImageProvider] that knows about [AssetBundle]s.
///
/// This factors out the common logic of [AssetBundle]-based [ImageProvider]
/// classes, simplifying what subclasses must implement to just [obtainKey].
abstract class AssetBundleImageProvider extends ImageProvider<AssetBundleImageKey> {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const AssetBundleImageProvider();

  @override
  ImageStreamCompleter loadImage(AssetBundleImageKey key, ImageDecoderCallback decode) {
    InformationCollector? collector;
    assert(() {
      collector =
          () => <DiagnosticsNode>[
            DiagnosticsProperty<ImageProvider>('Image provider', this),
            DiagnosticsProperty<AssetBundleImageKey>('Image key', key),
          ];
      return true;
    }());
    return MultiFrameImageStreamCompleter(
      codec: _loadAsync(key, decode: decode),
      scale: key.scale,
      debugLabel: key.name,
      informationCollector: collector,
    );
  }

  /// Converts a key into an [ImageStreamCompleter], and begins fetching the
  /// image.
  @override
  ImageStreamCompleter loadBuffer(AssetBundleImageKey key, DecoderBufferCallback decode) {
    InformationCollector? collector;
    assert(() {
      collector =
          () => <DiagnosticsNode>[
            DiagnosticsProperty<ImageProvider>('Image provider', this),
            DiagnosticsProperty<AssetBundleImageKey>('Image key', key),
          ];
      return true;
    }());
    return MultiFrameImageStreamCompleter(
      codec: _loadAsync(key, decode: decode),
      scale: key.scale,
      debugLabel: key.name,
      informationCollector: collector,
    );
  }

  /// Fetches the image from the asset bundle, decodes it, and returns a
  /// corresponding [ImageInfo] object.
  ///
  /// This function is used by [loadImage].
  @protected
  Future<ui.Codec> _loadAsync(
    AssetBundleImageKey key, {
    required _SimpleDecoderCallback decode,
  }) async {
    final ui.ImmutableBuffer buffer;
    // Hot reload/restart could change whether an asset bundle or key in a
    // bundle are available, or if it is a network backed bundle.
    try {
      buffer = await key.bundle.loadBuffer(key.name);
    } on FlutterError {
      PaintingBinding.instance.imageCache.evict(key);
      rethrow;
    }
    return decode(buffer);
  }
}

/// Key used internally by [ResizeImage].
///
/// This is used to identify the precise resource in the [imageCache].
@immutable
class ResizeImageKey {
  // Private constructor so nobody from the outside can poison the image cache
  // with this key. It's only accessible to [ResizeImage] internally.
  const ResizeImageKey._(
    this._providerCacheKey,
    this._policy,
    this._width,
    this._height,
    this._allowUpscaling,
  );

  final Object _providerCacheKey;
  final ResizeImagePolicy _policy;
  final int? _width;
  final int? _height;
  final bool _allowUpscaling;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ResizeImageKey &&
        other._providerCacheKey == _providerCacheKey &&
        other._policy == _policy &&
        other._width == _width &&
        other._height == _height &&
        other._allowUpscaling == _allowUpscaling;
  }

  @override
  int get hashCode => Object.hash(_providerCacheKey, _policy, _width, _height, _allowUpscaling);
}

/// Configures the behavior for [ResizeImage].
///
/// This is used in [ResizeImage.policy] to affect how the [ResizeImage.width]
/// and [ResizeImage.height] properties are interpreted.
enum ResizeImagePolicy {
  /// Sizes the image to the exact width and height specified by
  /// [ResizeImage.width] and [ResizeImage.height].
  ///
  /// If [ResizeImage.width] and [ResizeImage.height] are both non-null, the
  /// output image will have the specified width and height (with the
  /// corresponding aspect ratio) regardless of whether it matches the source
  /// image's intrinsic aspect ratio. This case is similar to [BoxFit.fill].
  ///
  /// If only one of `width` and `height` is non-null, then the output image
  /// will be scaled to the associated width or height, and the other dimension
  /// will take whatever value is needed to maintain the image's original aspect
  /// ratio. These cases are similar to [BoxFit.fitWidth] and
  /// [BoxFit.fitHeight], respectively.
  ///
  /// If [ResizeImage.allowUpscaling] is false (the default), the width and the
  /// height of the output image will each be clamped to the intrinsic width and
  /// height of the image. This may result in a different aspect ratio than the
  /// aspect ratio specified by the target width and height (e.g. if the height
  /// gets clamped downwards but the width does not).
  ///
  /// ## Examples
  ///
  /// The examples below show how [ResizeImagePolicy.exact] works in various
  /// scenarios. In each example, the source image has a size of 300x200
  /// (landscape orientation), the red box is a 150x150 square, and the green
  /// box is a 400x400 square.
  ///
  /// <table>
  /// <tr>
  /// <td>Scenario</td>
  /// <td>Output</td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   width: 150,
  ///   height: 150,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_exact_150x150_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   width: 150,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_exact_150xnull_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   height: 150,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_exact_nullx150_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   width: 400,
  ///   height: 400,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_exact_400x400_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   width: 400,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_exact_400xnull_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   height: 400,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_exact_nullx400_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   width: 400,
  ///   height: 400,
  ///   allowUpscaling: true,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_exact_400x400_true.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   width: 400,
  ///   allowUpscaling: true,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_exact_400xnull_true.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   height: 400,
  ///   allowUpscaling: true,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_exact_nullx400_true.png)
  ///
  /// </td>
  /// </tr>
  /// </table>
  exact,

  /// Scales the image as necessary to ensure that it fits within the bounding
  /// box specified by [ResizeImage.width] and [ResizeImage.height] while
  /// maintaining its aspect ratio.
  ///
  /// If [ResizeImage.allowUpscaling] is true, the image will be scaled up or
  /// down to best fit the bounding box; otherwise it will only ever be scaled
  /// down.
  ///
  /// This is conceptually similar to [BoxFit.contain].
  ///
  /// ## Examples
  ///
  /// The examples below show how [ResizeImagePolicy.fit] works in various
  /// scenarios. In each example, the source image has a size of 300x200
  /// (landscape orientation), the red box is a 150x150 square, and the green
  /// box is a 400x400 square.
  ///
  /// <table>
  /// <tr>
  /// <td>Scenario</td>
  /// <td>Output</td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   policy: ResizeImagePolicy.fit,
  ///   width: 150,
  ///   height: 150,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_fit_150x150_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   policy: ResizeImagePolicy.fit,
  ///   width: 150,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_fit_150xnull_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   policy: ResizeImagePolicy.fit,
  ///   height: 150,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_fit_nullx150_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   policy: ResizeImagePolicy.fit,
  ///   width: 400,
  ///   height: 400,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_fit_400x400_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   policy: ResizeImagePolicy.fit,
  ///   width: 400,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_fit_400xnull_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   policy: ResizeImagePolicy.fit,
  ///   height: 400,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_fit_nullx400_false.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   policy: ResizeImagePolicy.fit,
  ///   width: 400,
  ///   height: 400,
  ///   allowUpscaling: true,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_fit_400x400_true.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   policy: ResizeImagePolicy.fit,
  ///   width: 400,
  ///   allowUpscaling: true,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_fit_400xnull_true.png)
  ///
  /// </td>
  /// </tr>
  /// <tr>
  /// <td>
  ///
  /// ```dart
  /// const ResizeImage(
  ///   AssetImage('dragon_cake.jpg'),
  ///   policy: ResizeImagePolicy.fit,
  ///   height: 400,
  ///   allowUpscaling: true,
  /// )
  /// ```
  ///
  /// </td>
  /// <td>
  ///
  /// ![](https://flutter.github.io/assets-for-api-docs/assets/painting/resize_image_policy_fit_nullx400_true.png)
  ///
  /// </td>
  /// </tr>
  /// </table>
  fit,
}

/// Instructs Flutter to decode the image at the specified dimensions
/// instead of at its native size.
///
/// This allows finer control of the size of the image in [ImageCache] and is
/// generally used to reduce the memory footprint of [ImageCache].
///
/// The decoded image may still be displayed at sizes other than the
/// cached size provided here.
///
/// The [width] and [height] parameters determine the image resolution.
/// These values also set the image's width & height in logical pixels
/// if it is unconstrained.
///
/// {@tool snippet}
/// This example shows how to size the image to half of the screen's width.
///
/// ```dart
///    Image(
///      image: ResizeImage(
///        FileImage(File('path/to/image')),
///        width: MediaQuery.sizeOf(context).width ~/ 2, // Half of the screen's width.
///      ),
///    );
/// ```
/// {@end-tool}
///
/// See also:
///
///  * [ui.FlutterView.devicePixelRatio], used to convert between physical and
///    logical pixels.
class ResizeImage extends ImageProvider<ResizeImageKey> {
  /// Creates an ImageProvider that decodes the image to the specified size.
  ///
  /// The cached image will be directly decoded and stored at the resolution
  /// defined by `width` and `height`. The image will lose detail and
  /// use less memory if resized to a size smaller than the native size.
  ///
  /// At least one of `width` and `height` must be non-null.
  const ResizeImage(
    this.imageProvider, {
    this.width,
    this.height,
    this.policy = ResizeImagePolicy.exact,
    this.allowUpscaling = false,
  }) : assert(width != null || height != null);

  /// The [ImageProvider] that this class wraps.
  final ImageProvider imageProvider;

  /// The width the image should decode to and cache.
  ///
  /// At least one of this and [height] must be non-null.
  final int? width;

  /// The height the image should decode to and cache.
  ///
  /// At least one of this and [width] must be non-null.
  final int? height;

  /// The policy that determines how [width] and [height] are interpreted.
  ///
  /// Defaults to [ResizeImagePolicy.exact].
  final ResizeImagePolicy policy;

  /// Whether the [width] and [height] parameters should be clamped to the
  /// intrinsic width and height of the image.
  ///
  /// In general, it is better for memory usage to avoid scaling the image
  /// beyond its intrinsic dimensions when decoding it. If there is a need to
  /// scale an image larger, it is better to apply a scale to the canvas, or
  /// to use an appropriate [Image.fit].
  final bool allowUpscaling;

  /// Composes the `provider` in a [ResizeImage] only when `cacheWidth` and
  /// `cacheHeight` are not both null.
  ///
  /// When `cacheWidth` and `cacheHeight` are both null, this will return the
  /// `provider` directly.
  static ImageProvider<Object> resizeIfNeeded(
    int? cacheWidth,
    int? cacheHeight,
    ImageProvider<Object> provider,
  ) {
    if (cacheWidth != null || cacheHeight != null) {
      return ResizeImage(provider, width: cacheWidth, height: cacheHeight);
    }
    return provider;
  }

  @override
  @Deprecated(
    'Implement loadImage for image loading. '
    'This feature was deprecated after v3.7.0-1.4.pre.',
  )
  ImageStreamCompleter loadBuffer(ResizeImageKey key, DecoderBufferCallback decode) {
    Future<ui.Codec> decodeResize(
      ui.ImmutableBuffer buffer, {
      int? cacheWidth,
      int? cacheHeight,
      bool? allowUpscaling,
    }) {
      assert(
        cacheWidth == null && cacheHeight == null && allowUpscaling == null,
        'ResizeImage cannot be composed with another ImageProvider that applies '
        'cacheWidth, cacheHeight, or allowUpscaling.',
      );
      return decode(
        buffer,
        cacheWidth: width,
        cacheHeight: height,
        allowUpscaling: this.allowUpscaling,
      );
    }

    final ImageStreamCompleter completer = imageProvider.loadBuffer(
      key._providerCacheKey,
      decodeResize,
    );
    if (!kReleaseMode) {
      completer.debugLabel = '${completer.debugLabel} - Resized(${key._width}×${key._height})';
    }
    _configureErrorListener(completer, key);
    return completer;
  }

  @override
  ImageStreamCompleter loadImage(ResizeImageKey key, ImageDecoderCallback decode) {
    Future<ui.Codec> decodeResize(
      ui.ImmutableBuffer buffer, {
      ui.TargetImageSizeCallback? getTargetSize,
    }) {
      assert(
        getTargetSize == null,
        'ResizeImage cannot be composed with another ImageProvider that applies '
        'getTargetSize.',
      );
      return decode(
        buffer,
        getTargetSize: (int intrinsicWidth, int intrinsicHeight) {
          switch (policy) {
            case ResizeImagePolicy.exact:
              int? targetWidth = width;
              int? targetHeight = height;

              if (!allowUpscaling) {
                if (targetWidth != null && targetWidth > intrinsicWidth) {
                  targetWidth = intrinsicWidth;
                }
                if (targetHeight != null && targetHeight > intrinsicHeight) {
                  targetHeight = intrinsicHeight;
                }
              }

              return ui.TargetImageSize(width: targetWidth, height: targetHeight);
            case ResizeImagePolicy.fit:
              final double aspectRatio = intrinsicWidth / intrinsicHeight;
              final int maxWidth = width ?? intrinsicWidth;
              final int maxHeight = height ?? intrinsicHeight;
              int targetWidth = intrinsicWidth;
              int targetHeight = intrinsicHeight;

              if (targetWidth > maxWidth) {
                targetWidth = maxWidth;
                targetHeight = targetWidth ~/ aspectRatio;
              }

              if (targetHeight > maxHeight) {
                targetHeight = maxHeight;
                targetWidth = (targetHeight * aspectRatio).floor();
              }

              if (allowUpscaling) {
                if (width == null) {
                  assert(height != null);
                  targetHeight = height!;
                  targetWidth = (targetHeight * aspectRatio).floor();
                } else if (height == null) {
                  targetWidth = width!;
                  targetHeight = targetWidth ~/ aspectRatio;
                } else {
                  final int derivedMaxWidth = (maxHeight * aspectRatio).floor();
                  final int derivedMaxHeight = maxWidth ~/ aspectRatio;
                  targetWidth = math.min(maxWidth, derivedMaxWidth);
                  targetHeight = math.min(maxHeight, derivedMaxHeight);
                }
              }

              return ui.TargetImageSize(width: targetWidth, height: targetHeight);
          }
        },
      );
    }

    final ImageStreamCompleter completer = imageProvider.loadImage(
      key._providerCacheKey,
      decodeResize,
    );
    if (!kReleaseMode) {
      completer.debugLabel = '${completer.debugLabel} - Resized(${key._width}×${key._height})';
    }
    _configureErrorListener(completer, key);
    return completer;
  }

  void _configureErrorListener(ImageStreamCompleter completer, ResizeImageKey key) {
    completer.addEphemeralErrorListener((Object exception, StackTrace? stackTrace) {
      // The microtask is scheduled because of the same reason as NetworkImage:
      // Depending on where the exception was thrown, the image cache may not
      // have had a chance to track the key in the cache at all.
      // Schedule a microtask to give the cache a chance to add the key.
      scheduleMicrotask(() {
        PaintingBinding.instance.imageCache.evict(key);
      });
    });
  }

  @override
  Future<ResizeImageKey> obtainKey(ImageConfiguration configuration) {
    Completer<ResizeImageKey>? completer;
    // If the imageProvider.obtainKey future is synchronous, then we will be able to fill in result with
    // a value before completer is initialized below.
    SynchronousFuture<ResizeImageKey>? result;
    imageProvider.obtainKey(configuration).then((Object key) {
      if (completer == null) {
        // This future has completed synchronously (completer was never assigned),
        // so we can directly create the synchronous result to return.
        result = SynchronousFuture<ResizeImageKey>(
          ResizeImageKey._(key, policy, width, height, allowUpscaling),
        );
      } else {
        // This future did not synchronously complete.
        completer.complete(ResizeImageKey._(key, policy, width, height, allowUpscaling));
      }
    });
    if (result != null) {
      return result!;
    }
    // If the code reaches here, it means the imageProvider.obtainKey was not
    // completed sync, so we initialize the completer for completion later.
    completer = Completer<ResizeImageKey>();
    return completer.future;
  }
}

/// Fetches the given URL from the network, associating it with the given scale.
///
/// The image will be cached regardless of cache headers from the server.
///
/// See also:
///
///  * [Image.network] for a shorthand of an [Image] widget backed by [NetworkImage].
///  * The example at [ImageProvider], which shows a custom variant of this class
///    that applies different logic for fetching the image.
// TODO(ianh): Find some way to honor cache headers to the extent that when the
// last reference to an image is released, we proactively evict the image from
// our cache if the headers describe the image as having expired at that point.
abstract class NetworkImage extends ImageProvider<NetworkImage> {
  /// Creates an object that fetches the image at the given URL.
  ///
  /// The [scale] argument is the linear scale factor for drawing this image at
  /// its intended size. See [ImageInfo.scale] for more information.
  const factory NetworkImage(String url, {double scale, Map<String, String>? headers}) =
      network_image.NetworkImage;

  /// The URL from which the image will be fetched.
  String get url;

  /// The scale to place in the [ImageInfo] object of the image.
  double get scale;

  /// The HTTP headers that will be used with [HttpClient.get] to fetch image from network.
  ///
  /// When running Flutter on the web, headers are not used.
  Map<String, String>? get headers;

  @override
  ImageStreamCompleter loadBuffer(NetworkImage key, DecoderBufferCallback decode);

  @override
  ImageStreamCompleter loadImage(NetworkImage key, ImageDecoderCallback decode);
}

/// Decodes the given [File] object as an image, associating it with the given
/// scale.
///
/// The provider does not monitor the file for changes. If you expect the
/// underlying data to change, you should call the [evict] method.
///
/// See also:
///
///  * [Image.file] for a shorthand of an [Image] widget backed by [FileImage].
@immutable
class FileImage extends ImageProvider<FileImage> {
  /// Creates an object that decodes a [File] as an image.
  const FileImage(this.file, {this.scale = 1.0});

  /// The file to decode into an image.
  final File file;

  /// The scale to place in the [ImageInfo] object of the image.
  final double scale;

  @override
  Future<FileImage> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<FileImage>(this);
  }

  @override
  ImageStreamCompleter loadBuffer(FileImage key, DecoderBufferCallback decode) {
    return MultiFrameImageStreamCompleter(
      codec: _loadAsync(key, decode: decode),
      scale: key.scale,
      debugLabel: key.file.path,
      informationCollector: () => <DiagnosticsNode>[ErrorDescription('Path: ${file.path}')],
    );
  }

  @override
  @protected
  ImageStreamCompleter loadImage(FileImage key, ImageDecoderCallback decode) {
    return MultiFrameImageStreamCompleter(
      codec: _loadAsync(key, decode: decode),
      scale: key.scale,
      debugLabel: key.file.path,
      informationCollector: () => <DiagnosticsNode>[ErrorDescription('Path: ${file.path}')],
    );
  }

  Future<ui.Codec> _loadAsync(FileImage key, {required _SimpleDecoderCallback decode}) async {
    assert(key == this);
    // TODO(jonahwilliams): making this sync caused test failures that seem to
    // indicate that we can fail to call evict unless at least one await has
    // occurred in the test.
    // https://github.com/flutter/flutter/issues/113044
    final int lengthInBytes = await file.length();
    if (lengthInBytes == 0) {
      // The file may become available later.
      PaintingBinding.instance.imageCache.evict(key);
      throw StateError('$file is empty and cannot be loaded as an image.');
    }
    return (file.runtimeType == File)
        ? decode(await ui.ImmutableBuffer.fromFilePath(file.path))
        : decode(await ui.ImmutableBuffer.fromUint8List(await file.readAsBytes()));
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is FileImage && other.file.path == file.path && other.scale == scale;
  }

  @override
  int get hashCode => Object.hash(file.path, scale);

  @override
  String toString() =>
      '${objectRuntimeType(this, 'FileImage')}("${file.path}", scale: ${scale.toStringAsFixed(1)})';
}

/// Decodes the given [Uint8List] buffer as an image, associating it with the
/// given scale.
///
/// The provided [bytes] buffer should not be changed after it is provided
/// to a [MemoryImage]. To provide an [ImageStream] that represents an image
/// that changes over time, consider creating a new subclass of [ImageProvider]
/// whose [loadImage] method returns a subclass of [ImageStreamCompleter] that
/// can handle providing multiple images.
///
/// See also:
///
///  * [Image.memory] for a shorthand of an [Image] widget backed by [MemoryImage].
@immutable
class MemoryImage extends ImageProvider<MemoryImage> {
  /// Creates an object that decodes a [Uint8List] buffer as an image.
  const MemoryImage(this.bytes, {this.scale = 1.0});

  /// The bytes to decode into an image.
  ///
  /// The bytes represent encoded image bytes and can be encoded in any of the
  /// following supported image formats: {@macro dart.ui.imageFormats}
  ///
  /// See also:
  ///
  ///  * [PaintingBinding.instantiateImageCodecWithSize]
  final Uint8List bytes;

  /// The scale to place in the [ImageInfo] object of the image.
  ///
  /// See also:
  ///
  ///  * [ImageInfo.scale], which gives more information on how this scale is
  ///    applied.
  final double scale;

  @override
  Future<MemoryImage> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<MemoryImage>(this);
  }

  @override
  ImageStreamCompleter loadBuffer(MemoryImage key, DecoderBufferCallback decode) {
    assert(key == this);
    return MultiFrameImageStreamCompleter(
      codec: _loadAsync(key, decode: decode),
      scale: key.scale,
      debugLabel: 'MemoryImage(${describeIdentity(key.bytes)})',
    );
  }

  @override
  ImageStreamCompleter loadImage(MemoryImage key, ImageDecoderCallback decode) {
    return MultiFrameImageStreamCompleter(
      codec: _loadAsync(key, decode: decode),
      scale: key.scale,
      debugLabel: 'MemoryImage(${describeIdentity(key.bytes)})',
    );
  }

  Future<ui.Codec> _loadAsync(MemoryImage key, {required _SimpleDecoderCallback decode}) async {
    assert(key == this);
    return decode(await ui.ImmutableBuffer.fromUint8List(bytes));
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is MemoryImage && other.bytes == bytes && other.scale == scale;
  }

  @override
  int get hashCode => Object.hash(bytes.hashCode, scale);

  @override
  String toString() =>
      '${objectRuntimeType(this, 'MemoryImage')}(${describeIdentity(bytes)}, scale: ${scale.toStringAsFixed(1)})';
}

/// Fetches an image from an [AssetBundle], associating it with the given scale.
///
/// This implementation requires an explicit final [assetName] and [scale] on
/// construction, and ignores the device pixel ratio and size in the
/// configuration passed into [resolve]. For a resolution-aware variant that
/// uses the configuration to pick an appropriate image based on the device
/// pixel ratio and size, see [AssetImage].
///
/// ## Fetching assets
///
/// When fetching an image provided by the app itself, use the [assetName]
/// argument to name the asset to choose. For instance, consider a directory
/// `icons` with an image `heart.png`. First, the `pubspec.yaml` of the project
/// should specify its assets in the `flutter` section:
///
/// ```yaml
/// flutter:
///   assets:
///     - icons/heart.png
/// ```
///
/// Then, to fetch the image and associate it with scale `1.5`, use:
///
/// {@tool snippet}
/// ```dart
/// const ExactAssetImage('icons/heart.png', scale: 1.5)
/// ```
/// {@end-tool}
///
/// ## Assets in packages
///
/// To fetch an asset from a package, the [package] argument must be provided.
/// For instance, suppose the structure above is inside a package called
/// `my_icons`. Then to fetch the image, use:
///
/// {@tool snippet}
/// ```dart
/// const ExactAssetImage('icons/heart.png', scale: 1.5, package: 'my_icons')
/// ```
/// {@end-tool}
///
/// Assets used by the package itself should also be fetched using the [package]
/// argument as above.
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
/// To include, say the first image, the `pubspec.yaml` of the app should specify
/// it in the `assets` section:
///
/// ```yaml
///   assets:
///     - packages/fancy_backgrounds/backgrounds/background1.png
/// ```
///
/// The `lib/` is implied, so it should not be included in the asset path.
///
/// See also:
///
///  * [Image.asset] for a shorthand of an [Image] widget backed by
///    [ExactAssetImage] when using a scale.
@immutable
class ExactAssetImage extends AssetBundleImageProvider {
  /// Creates an object that fetches the given image from an asset bundle.
  ///
  /// The [scale] argument defaults to 1. The [bundle] argument may be null, in
  /// which case the bundle provided in the [ImageConfiguration] passed to the
  /// [resolve] call will be used instead.
  ///
  /// The [package] argument must be non-null when fetching an asset that is
  /// included in a package. See the documentation for the [ExactAssetImage] class
  /// itself for details.
  const ExactAssetImage(this.assetName, {this.scale = 1.0, this.bundle, this.package});

  /// The name of the asset.
  final String assetName;

  /// The key to use to obtain the resource from the [bundle]. This is the
  /// argument passed to [AssetBundle.load].
  String get keyName => package == null ? assetName : 'packages/$package/$assetName';

  /// The scale to place in the [ImageInfo] object of the image.
  final double scale;

  /// The bundle from which the image will be obtained.
  ///
  /// If the provided [bundle] is null, the bundle provided in the
  /// [ImageConfiguration] passed to the [resolve] call will be used instead. If
  /// that is also null, the [rootBundle] is used.
  ///
  /// The image is obtained by calling [AssetBundle.load] on the given [bundle]
  /// using the key given by [keyName].
  final AssetBundle? bundle;

  /// The name of the package from which the image is included. See the
  /// documentation for the [ExactAssetImage] class itself for details.
  final String? package;

  @override
  Future<AssetBundleImageKey> obtainKey(ImageConfiguration configuration) {
    return SynchronousFuture<AssetBundleImageKey>(
      AssetBundleImageKey(
        bundle: bundle ?? configuration.bundle ?? rootBundle,
        name: keyName,
        scale: scale,
      ),
    );
  }

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ExactAssetImage &&
        other.keyName == keyName &&
        other.scale == scale &&
        other.bundle == bundle;
  }

  @override
  int get hashCode => Object.hash(keyName, scale, bundle);

  @override
  String toString() =>
      '${objectRuntimeType(this, 'ExactAssetImage')}(name: "$keyName", scale: ${scale.toStringAsFixed(1)}, bundle: $bundle)';
}

// A completer used when resolving an image fails sync.
class _ErrorImageCompleter extends ImageStreamCompleter {}

/// The exception thrown when the HTTP request to load a network image fails.
class NetworkImageLoadException implements Exception {
  /// Creates a [NetworkImageLoadException] with the specified http [statusCode]
  /// and [uri].
  NetworkImageLoadException({required this.statusCode, required this.uri})
    : _message = 'HTTP request failed, statusCode: $statusCode, $uri';

  /// The HTTP status code from the server.
  final int statusCode;

  /// A human-readable error message.
  final String _message;

  /// Resolved URL of the requested image.
  final Uri uri;

  @override
  String toString() => _message;
}
