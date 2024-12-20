// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:ui';
///
/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'image_cache.dart';
/// @docImport 'image_provider.dart';
library;

import 'dart:async';
import 'dart:ui' as ui show Codec, FrameInfo, Image;

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';

const String _flutterPaintingLibrary = 'package:flutter/painting.dart';

/// A [dart:ui.Image] object with its corresponding scale.
///
/// ImageInfo objects are used by [ImageStream] objects to represent the
/// actual data of the image once it has been obtained.
///
/// The disposing contract for [ImageInfo] (as well as for [ui.Image])
/// is different from traditional one, where
/// an object should dispose a member if the object created the member.
/// Instead, the disposal contract is as follows:
///
/// * [ImageInfo] disposes [image], even if it is received as a constructor argument.
/// * [ImageInfo] is expected to be disposed not by the object, that created it,
/// but by the object that owns reference to it.
/// * It is expected that only one object owns reference to [ImageInfo] object.
///
/// Safety tips:
///
///  * To share the [ImageInfo] or [ui.Image] between objects, use the [clone] method,
/// which will not clone the entire underlying image, but only reference to it and information about it.
///  * After passing a [ui.Image] or [ImageInfo] reference to another object,
/// release the reference.
@immutable
class ImageInfo {
  /// Creates an [ImageInfo] object for the given [image] and [scale].
  ///
  /// The [debugLabel] may be used to identify the source of this image.
  ///
  /// See details for disposing contract in the class description.
  ImageInfo({required this.image, this.scale = 1.0, this.debugLabel}) {
    if (kFlutterMemoryAllocationsEnabled) {
      MemoryAllocations.instance.dispatchObjectCreated(
        library: _flutterPaintingLibrary,
        className: '$ImageInfo',
        object: this,
      );
    }
  }

  /// Creates an [ImageInfo] with a cloned [image].
  ///
  /// This method must be used in cases where a client holding an [ImageInfo]
  /// needs to share the image info object with another client and will still
  /// need to access the underlying image data at some later point, e.g. to
  /// share it again with another client.
  ///
  /// See details for disposing contract in the class description.
  ///
  /// See also:
  ///
  ///  * [ui.Image.clone], which describes how and why to clone images.
  ImageInfo clone() {
    return ImageInfo(image: image.clone(), scale: scale, debugLabel: debugLabel);
  }

  /// Whether this [ImageInfo] is a [clone] of the `other`.
  ///
  /// This method is a convenience wrapper for [ui.Image.isCloneOf], and is
  /// useful for clients that are trying to determine whether new layout or
  /// painting logic is required when receiving a new image reference.
  ///
  /// {@tool snippet}
  ///
  /// The following sample shows how to appropriately check whether the
  /// [ImageInfo] reference refers to new image data or not (in this case in a
  /// setter).
  ///
  /// ```dart
  /// ImageInfo? get imageInfo => _imageInfo;
  /// ImageInfo? _imageInfo;
  /// set imageInfo (ImageInfo? value) {
  ///   // If the image reference is exactly the same, do nothing.
  ///   if (value == _imageInfo) {
  ///     return;
  ///   }
  ///   // If it is a clone of the current reference, we must dispose of it and
  ///   // can do so immediately. Since the underlying image has not changed,
  ///   // We don't have any additional work to do here.
  ///   if (value != null && _imageInfo != null && value.isCloneOf(_imageInfo!)) {
  ///     value.dispose();
  ///     return;
  ///   }
  ///   // It is a new image. Dispose of the old one and take a reference
  ///   // to the new one.
  ///   _imageInfo?.dispose();
  ///   _imageInfo = value;
  ///   // Perform work to determine size, paint the image, etc.
  ///   // ...
  /// }
  /// ```
  /// {@end-tool}
  bool isCloneOf(ImageInfo other) {
    return other.image.isCloneOf(image) && scale == scale && other.debugLabel == debugLabel;
  }

  /// The raw image pixels.
  ///
  /// This is the object to pass to the [Canvas.drawImage],
  /// [Canvas.drawImageRect], or [Canvas.drawImageNine] methods when painting
  /// the image.
  final ui.Image image;

  /// The size of raw image pixels in bytes.
  int get sizeBytes => image.height * image.width * 4;

  /// The linear scale factor for drawing this image at its intended size.
  ///
  /// The scale factor applies to the width and the height.
  ///
  /// {@template flutter.painting.imageInfo.scale}
  /// For example, if this is 2.0, it means that there are four image pixels for
  /// every one logical pixel, and the image's actual width and height (as given
  /// by the [dart:ui.Image.width] and [dart:ui.Image.height] properties) are
  /// double the height and width that should be used when painting the image
  /// (e.g. in the arguments given to [Canvas.drawImage]).
  /// {@endtemplate}
  final double scale;

  /// A string used for debugging purposes to identify the source of this image.
  final String? debugLabel;

  /// Disposes of this object.
  ///
  /// Once this method has been called, the object should not be used anymore,
  /// and no clones of it or the image it contains can be made.
  void dispose() {
    assert((image.debugGetOpenHandleStackTraces()?.length ?? 1) > 0);
    if (kFlutterMemoryAllocationsEnabled) {
      MemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    image.dispose();
  }

  @override
  String toString() =>
      '${debugLabel != null ? '$debugLabel ' : ''}$image @ ${debugFormatDouble(scale)}x';

  @override
  int get hashCode => Object.hash(image, scale, debugLabel);

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ImageInfo &&
        other.image == image &&
        other.scale == scale &&
        other.debugLabel == debugLabel;
  }
}

/// Interface for receiving notifications about the loading of an image.
///
/// This class overrides [operator ==] and [hashCode] to compare the individual
/// callbacks in the listener, meaning that if you add an instance of this class
/// as a listener (e.g. via [ImageStream.addListener]), you can instantiate a
/// _different_ instance of this class when you remove the listener, and the
/// listener will be properly removed as long as all associated callbacks are
/// equal.
///
/// Used by [ImageStream] and [ImageStreamCompleter].
@immutable
class ImageStreamListener {
  /// Creates a new [ImageStreamListener].
  const ImageStreamListener(this.onImage, {this.onChunk, this.onError});

  /// Callback for getting notified that an image is available.
  ///
  /// This callback may fire multiple times (e.g. if the [ImageStreamCompleter]
  /// that drives the notifications fires multiple times). An example of such a
  /// case would be an image with multiple frames within it (such as an animated
  /// GIF).
  ///
  /// For more information on how to interpret the parameters to the callback,
  /// see the documentation on [ImageListener].
  ///
  /// See also:
  ///
  ///  * [onError], which will be called instead of [onImage] if an error occurs
  ///    during loading.
  final ImageListener onImage;

  /// Callback for getting notified when a chunk of bytes has been received
  /// during the loading of the image.
  ///
  /// This callback may fire many times (e.g. when used with a [NetworkImage],
  /// where the image bytes are loaded incrementally over the wire) or not at
  /// all (e.g. when used with a [MemoryImage], where the image bytes are
  /// already available in memory).
  ///
  /// This callback may also continue to fire after the [onImage] callback has
  /// fired (e.g. for multi-frame images that continue to load after the first
  /// frame is available).
  final ImageChunkListener? onChunk;

  /// Callback for getting notified when an error occurs while loading an image.
  ///
  /// If an error occurs during loading, [onError] will be called instead of
  /// [onImage].
  ///
  /// If [onError] is called and does not throw, then the error is considered to
  /// be handled. An error handler can explicitly rethrow the exception reported
  /// to it to safely indicate that it did not handle the exception.
  ///
  /// If an image stream has no listeners that handled the error when the error
  /// was first encountered, then the error is reported using
  /// [FlutterError.reportError], with the [FlutterErrorDetails.silent] flag set
  /// to true.
  final ImageErrorListener? onError;

  @override
  int get hashCode => Object.hash(onImage, onChunk, onError);

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ImageStreamListener &&
        other.onImage == onImage &&
        other.onChunk == onChunk &&
        other.onError == onError;
  }
}

/// Signature for callbacks reporting that an image is available.
///
/// Used in [ImageStreamListener].
///
/// The `image` argument contains information about the image to be rendered.
/// The implementer of [ImageStreamListener.onImage] is expected to call dispose
/// on the [ui.Image] it receives.
///
/// The `synchronousCall` argument is true if the listener is being invoked
/// during the call to `addListener`. This can be useful if, for example,
/// [ImageStream.addListener] is invoked during a frame, so that a new rendering
/// frame is requested if the call was asynchronous (after the current frame)
/// and no rendering frame is requested if the call was synchronous (within the
/// same stack frame as the call to [ImageStream.addListener]).
typedef ImageListener = void Function(ImageInfo image, bool synchronousCall);

/// Signature for listening to [ImageChunkEvent] events.
///
/// Used in [ImageStreamListener].
typedef ImageChunkListener = void Function(ImageChunkEvent event);

/// Signature for reporting errors when resolving images.
///
/// Used in [ImageStreamListener], as well as by [ImageCache.putIfAbsent] and
/// [precacheImage], to report errors.
typedef ImageErrorListener = void Function(Object exception, StackTrace? stackTrace);

/// An immutable notification of image bytes that have been incrementally loaded.
///
/// Chunk events represent progress notifications while an image is being
/// loaded (e.g. from disk or over the network).
///
/// See also:
///
///  * [ImageChunkListener], the means by which callers get notified of
///    these events.
@immutable
class ImageChunkEvent with Diagnosticable {
  /// Creates a new chunk event.
  const ImageChunkEvent({required this.cumulativeBytesLoaded, required this.expectedTotalBytes})
    : assert(cumulativeBytesLoaded >= 0),
      assert(expectedTotalBytes == null || expectedTotalBytes >= 0);

  /// The number of bytes that have been received across the wire thus far.
  final int cumulativeBytesLoaded;

  /// The expected number of bytes that need to be received to finish loading
  /// the image.
  ///
  /// This value is not necessarily equal to the expected _size_ of the image
  /// in bytes, as the bytes required to load the image may be compressed.
  ///
  /// This value will be null if the number is not known in advance.
  ///
  /// When this value is null, the chunk event may still be useful as an
  /// indication that data is loading (and how much), but it cannot represent a
  /// loading completion percentage.
  final int? expectedTotalBytes;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(IntProperty('cumulativeBytesLoaded', cumulativeBytesLoaded));
    properties.add(IntProperty('expectedTotalBytes', expectedTotalBytes));
  }
}

/// A handle to an image resource.
///
/// ImageStream represents a handle to a [dart:ui.Image] object and its scale
/// (together represented by an [ImageInfo] object). The underlying image object
/// might change over time, either because the image is animating or because the
/// underlying image resource was mutated.
///
/// ImageStream objects can also represent an image that hasn't finished
/// loading.
///
/// ImageStream objects are backed by [ImageStreamCompleter] objects.
///
/// The [ImageCache] will consider an image to be live until the listener count
/// drops to zero after adding at least one listener. The
/// [ImageStreamCompleter.addOnLastListenerRemovedCallback] method is used for
/// tracking this information.
///
/// See also:
///
///  * [ImageProvider], which has an example that includes the use of an
///    [ImageStream] in a [Widget].
class ImageStream with Diagnosticable {
  /// Create an initially unbound image stream.
  ///
  /// Once an [ImageStreamCompleter] is available, call [setCompleter].
  ImageStream();

  /// The completer that has been assigned to this image stream.
  ///
  /// Generally there is no need to deal with the completer directly.
  ImageStreamCompleter? get completer => _completer;
  ImageStreamCompleter? _completer;

  List<ImageStreamListener>? _listeners;

  /// Assigns a particular [ImageStreamCompleter] to this [ImageStream].
  ///
  /// This is usually done automatically by the [ImageProvider] that created the
  /// [ImageStream].
  ///
  /// This method can only be called once per stream. To have an [ImageStream]
  /// represent multiple images over time, assign it a completer that
  /// completes several images in succession.
  void setCompleter(ImageStreamCompleter value) {
    assert(_completer == null);
    _completer = value;
    if (_listeners != null) {
      final List<ImageStreamListener> initialListeners = _listeners!;
      _listeners = null;
      _completer!._addingInitialListeners = true;
      initialListeners.forEach(_completer!.addListener);
      _completer!._addingInitialListeners = false;
    }
  }

  /// Adds a listener callback that is called whenever a new concrete [ImageInfo]
  /// object is available. If a concrete image is already available, this object
  /// will call the listener synchronously.
  ///
  /// If the assigned [completer] completes multiple images over its lifetime,
  /// this listener will fire multiple times.
  ///
  /// {@template flutter.painting.imageStream.addListener}
  /// The listener will be passed a flag indicating whether a synchronous call
  /// occurred. If the listener is added within a render object paint function,
  /// then use this flag to avoid calling [RenderObject.markNeedsPaint] during
  /// a paint.
  ///
  /// If a duplicate `listener` is registered N times, then it will be called N
  /// times when the image stream completes (whether because a new image is
  /// available or because an error occurs). Likewise, to remove all instances
  /// of the listener, [removeListener] would need to called N times as well.
  ///
  /// When a `listener` receives an [ImageInfo] object, the `listener` is
  /// responsible for disposing of the [ImageInfo.image].
  /// {@endtemplate}
  void addListener(ImageStreamListener listener) {
    if (_completer != null) {
      return _completer!.addListener(listener);
    }
    _listeners ??= <ImageStreamListener>[];
    _listeners!.add(listener);
  }

  /// Stops listening for events from this stream's [ImageStreamCompleter].
  ///
  /// If [listener] has been added multiple times, this removes the _first_
  /// instance of the listener.
  void removeListener(ImageStreamListener listener) {
    if (_completer != null) {
      return _completer!.removeListener(listener);
    }
    assert(_listeners != null);
    for (int i = 0; i < _listeners!.length; i += 1) {
      if (_listeners![i] == listener) {
        _listeners!.removeAt(i);
        break;
      }
    }
  }

  /// Returns an object which can be used with `==` to determine if this
  /// [ImageStream] shares the same listeners list as another [ImageStream].
  ///
  /// This can be used to avoid un-registering and re-registering listeners
  /// after calling [ImageProvider.resolve] on a new, but possibly equivalent,
  /// [ImageProvider].
  ///
  /// The key may change once in the lifetime of the object. When it changes, it
  /// will go from being different than other [ImageStream]'s keys to
  /// potentially being the same as others'. No notification is sent when this
  /// happens.
  Object get key => _completer ?? this;

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    properties.add(
      ObjectFlagProperty<ImageStreamCompleter>(
        'completer',
        _completer,
        ifPresent: _completer?.toStringShort(),
        ifNull: 'unresolved',
      ),
    );
    properties.add(
      ObjectFlagProperty<List<ImageStreamListener>>(
        'listeners',
        _listeners,
        ifPresent: '${_listeners?.length} listener${_listeners?.length == 1 ? "" : "s"}',
        ifNull: 'no listeners',
        level: _completer != null ? DiagnosticLevel.hidden : DiagnosticLevel.info,
      ),
    );
    _completer?.debugFillProperties(properties);
  }
}

/// An opaque handle that keeps an [ImageStreamCompleter] alive even if it has
/// lost its last listener.
///
/// To create a handle, use [ImageStreamCompleter.keepAlive].
///
/// Such handles are useful when an image cache needs to keep a completer alive
/// but does not actually have a listener subscribed, or when a widget that
/// displays an image needs to temporarily unsubscribe from the completer but
/// may re-subscribe in the future, for example when the [TickerMode] changes.
class ImageStreamCompleterHandle {
  ImageStreamCompleterHandle._(ImageStreamCompleter this._completer) {
    _completer!._keepAliveHandles += 1;
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: _flutterPaintingLibrary,
        className: '$ImageStreamCompleterHandle',
        object: this,
      );
    }
  }

  ImageStreamCompleter? _completer;

  /// Call this method to signal the [ImageStreamCompleter] that it can now be
  /// disposed when its last listener drops.
  ///
  /// This method must only be called once per object.
  void dispose() {
    assert(_completer != null);
    assert(_completer!._keepAliveHandles > 0);
    assert(!_completer!._disposed);

    _completer!._keepAliveHandles -= 1;
    _completer!._maybeDispose();
    _completer = null;
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
  }
}

/// Base class for those that manage the loading of [dart:ui.Image] objects for
/// [ImageStream]s.
///
/// [ImageStreamListener] objects are rarely constructed directly. Generally, an
/// [ImageProvider] subclass will return an [ImageStream] and automatically
/// configure it with the right [ImageStreamCompleter] when possible.
abstract class ImageStreamCompleter with Diagnosticable {
  final List<ImageStreamListener> _listeners = <ImageStreamListener>[];
  final List<ImageErrorListener> _ephemeralErrorListeners = <ImageErrorListener>[];
  ImageInfo? _currentImage;
  FlutterErrorDetails? _currentError;

  /// A string identifying the source of the underlying image.
  String? debugLabel;

  /// Whether any listeners are currently registered.
  ///
  /// Clients should not depend on this value for their behavior, because having
  /// one listener's logic change when another listener happens to start or stop
  /// listening will lead to extremely hard-to-track bugs. Subclasses might use
  /// this information to determine whether to do any work when there are no
  /// listeners, however; for example, [MultiFrameImageStreamCompleter] uses it
  /// to determine when to iterate through frames of an animated image.
  ///
  /// Typically this is used by overriding [addListener], checking if
  /// [hasListeners] is false before calling `super.addListener()`, and if so,
  /// starting whatever work is needed to determine when to notify listeners;
  /// and similarly, by overriding [removeListener], checking if [hasListeners]
  /// is false after calling `super.removeListener()`, and if so, stopping that
  /// same work.
  ///
  /// The ephemeral error listeners (added through [addEphemeralErrorListener])
  /// will not be taken into consideration in this property.
  @protected
  @visibleForTesting
  bool get hasListeners => _listeners.isNotEmpty;

  /// We must avoid disposing a completer if it has never had a listener, even
  /// if all [keepAlive] handles get disposed.
  bool _hadAtLeastOneListener = false;

  /// Whether the future listeners added to this completer are initial listeners.
  ///
  /// This can be set to true when an [ImageStream] adds its initial listeners to
  /// this completer. This ultimately controls the synchronousCall parameter for
  /// the listener callbacks. When adding cached listeners to a completer,
  /// [_addingInitialListeners] can be set to false to indicate to the listeners
  /// that they are being called asynchronously.
  bool _addingInitialListeners = false;

  /// Adds a listener callback that is called whenever a new concrete [ImageInfo]
  /// object is available or an error is reported. If a concrete image is
  /// already available, or if an error has been already reported, this object
  /// will notify the listener synchronously.
  ///
  /// If the [ImageStreamCompleter] completes multiple images over its lifetime,
  /// this listener's [ImageStreamListener.onImage] will fire multiple times.
  ///
  /// {@macro flutter.painting.imageStream.addListener}
  ///
  /// See also:
  ///
  ///  * [addEphemeralErrorListener], which adds an error listener that is
  ///    automatically removed after first image load or error.
  void addListener(ImageStreamListener listener) {
    _checkDisposed();
    _hadAtLeastOneListener = true;
    _listeners.add(listener);
    if (_currentImage != null) {
      try {
        listener.onImage(_currentImage!.clone(), !_addingInitialListeners);
      } catch (exception, stack) {
        reportError(
          context: ErrorDescription('by a synchronously-called image listener'),
          exception: exception,
          stack: stack,
        );
      }
    }
    if (_currentError != null && listener.onError != null) {
      try {
        listener.onError!(_currentError!.exception, _currentError!.stack);
      } catch (newException, newStack) {
        if (newException != _currentError!.exception) {
          FlutterError.reportError(
            FlutterErrorDetails(
              exception: newException,
              library: 'image resource service',
              context: ErrorDescription('by a synchronously-called image error listener'),
              stack: newStack,
            ),
          );
        }
      }
    }
  }

  /// Adds an error listener callback that is called when the first error is reported.
  ///
  /// The callback will be removed automatically after the first successful
  /// image load or the first error - that is why it is called "ephemeral".
  ///
  /// If a concrete image is already available, the listener will be discarded
  /// synchronously. If an error has been already reported, the listener
  /// will be notified synchronously.
  ///
  /// The presence of a listener will affect neither the lifecycle of this object
  /// nor what [hasListeners] reports.
  ///
  /// It is different from [addListener] in a few points: Firstly, this one only
  /// listens to errors, while [addListener] listens to all kinds of events.
  /// Secondly, this listener will be automatically removed according to the
  /// rules mentioned above, while [addListener] will need manual removal.
  /// Thirdly, this listener will not affect how this object is disposed, while
  /// any non-removed listener added via [addListener] will forbid this object
  /// from disposal.
  ///
  /// When you want to know full information and full control, use [addListener].
  /// When you only want to get notified for an error ephemerally, use this function.
  ///
  /// See also:
  ///
  ///  * [addListener], which adds a full-featured listener and needs manual
  ///    removal.
  void addEphemeralErrorListener(ImageErrorListener listener) {
    _checkDisposed();
    if (_currentError != null) {
      // immediately fire the listener, and no need to add to _ephemeralErrorListeners
      try {
        listener(_currentError!.exception, _currentError!.stack);
      } catch (newException, newStack) {
        if (newException != _currentError!.exception) {
          FlutterError.reportError(
            FlutterErrorDetails(
              exception: newException,
              library: 'image resource service',
              context: ErrorDescription('by a synchronously-called image error listener'),
              stack: newStack,
            ),
          );
        }
      }
    } else if (_currentImage == null) {
      // add to _ephemeralErrorListeners to wait for the error,
      // only if no image has been loaded
      _ephemeralErrorListeners.add(listener);
    }
  }

  int _keepAliveHandles = 0;

  /// Creates an [ImageStreamCompleterHandle] that will prevent this stream from
  /// being disposed at least until the handle is disposed.
  ///
  /// Such handles are useful when an image cache needs to keep a completer
  /// alive but does not itself have a listener subscribed, or when a widget
  /// that displays an image needs to temporarily unsubscribe from the completer
  /// but may re-subscribe in the future, for example when the [TickerMode]
  /// changes.
  ImageStreamCompleterHandle keepAlive() {
    _checkDisposed();
    return ImageStreamCompleterHandle._(this);
  }

  /// Stops the specified [listener] from receiving image stream events.
  ///
  /// If [listener] has been added multiple times, this removes the _first_
  /// instance of the listener.
  ///
  /// Once all listeners have been removed and all [keepAlive] handles have been
  /// disposed, this image stream is no longer usable.
  void removeListener(ImageStreamListener listener) {
    _checkDisposed();
    for (int i = 0; i < _listeners.length; i += 1) {
      if (_listeners[i] == listener) {
        _listeners.removeAt(i);
        break;
      }
    }
    if (_listeners.isEmpty) {
      final List<VoidCallback> callbacks = _onLastListenerRemovedCallbacks.toList();
      for (final VoidCallback callback in callbacks) {
        callback();
      }
      _onLastListenerRemovedCallbacks.clear();
      _maybeDispose();
    }
  }

  bool _disposed = false;

  /// Called when this [ImageStreamCompleter] has actually been disposed.
  ///
  /// Subclasses should override this if they need to clean up resources when
  /// they are disposed.
  @mustCallSuper
  @protected
  void onDisposed() {}

  /// Disposes this [ImageStreamCompleter] unless:
  ///   1. It has never had a listener
  ///   2. It is already disposed
  ///   3. It has listeners.
  ///   4. It has active "keep alive" handles.
  @nonVirtual
  void maybeDispose() {
    _maybeDispose();
  }

  @mustCallSuper
  void _maybeDispose() {
    if (!_hadAtLeastOneListener || _disposed || _listeners.isNotEmpty || _keepAliveHandles != 0) {
      return;
    }

    _ephemeralErrorListeners.clear();
    _currentImage?.dispose();
    _currentImage = null;
    _disposed = true;
    onDisposed();
  }

  void _checkDisposed() {
    if (_disposed) {
      throw StateError(
        'Stream has been disposed.\n'
        'An ImageStream is considered disposed once at least one listener has '
        'been added and subsequently all listeners have been removed and no '
        'handles are outstanding from the keepAlive method.\n'
        'To resolve this error, maintain at least one listener on the stream, '
        'or create an ImageStreamCompleterHandle from the keepAlive '
        'method, or create a new stream for the image.',
      );
    }
  }

  final List<VoidCallback> _onLastListenerRemovedCallbacks = <VoidCallback>[];

  /// Adds a callback to call when [removeListener] results in an empty
  /// list of listeners and there are no [keepAlive] handles outstanding.
  ///
  /// This callback will never fire if [removeListener] is never called.
  void addOnLastListenerRemovedCallback(VoidCallback callback) {
    _checkDisposed();
    _onLastListenerRemovedCallbacks.add(callback);
  }

  /// Removes a callback previously supplied to
  /// [addOnLastListenerRemovedCallback].
  void removeOnLastListenerRemovedCallback(VoidCallback callback) {
    _checkDisposed();
    _onLastListenerRemovedCallbacks.remove(callback);
  }

  /// Calls all the registered listeners to notify them of a new image.
  @protected
  @pragma('vm:notify-debugger-on-exception')
  void setImage(ImageInfo image) {
    _checkDisposed();
    _currentImage?.dispose();
    _currentImage = image;

    _ephemeralErrorListeners.clear();

    if (_listeners.isEmpty) {
      return;
    }
    // Make a copy to allow for concurrent modification.
    final List<ImageStreamListener> localListeners = List<ImageStreamListener>.of(_listeners);
    for (final ImageStreamListener listener in localListeners) {
      try {
        listener.onImage(image.clone(), false);
      } catch (exception, stack) {
        reportError(
          context: ErrorDescription('by an image listener'),
          exception: exception,
          stack: stack,
        );
      }
    }
  }

  /// Calls all the registered error listeners to notify them of an error that
  /// occurred while resolving the image.
  ///
  /// If no error listeners (listeners with an [ImageStreamListener.onError]
  /// specified) are attached, or if the handlers all rethrow the exception
  /// verbatim (with `throw exception`), a [FlutterError] will be reported using
  /// [FlutterError.reportError].
  ///
  /// The `context` should be a string describing where the error was caught, in
  /// a form that will make sense in English when following the word "thrown",
  /// as in "thrown while obtaining the image from the network" (for the context
  /// "while obtaining the image from the network").
  ///
  /// The `exception` is the error being reported; the `stack` is the
  /// [StackTrace] associated with the exception.
  ///
  /// The `informationCollector` is a callback (of type [InformationCollector])
  /// that is called when the exception is used by [FlutterError.reportError].
  /// It is used to obtain further details to include in the logs, which may be
  /// expensive to collect, and thus should only be collected if the error is to
  /// be logged in the first place.
  ///
  /// The `silent` argument causes the exception to not be reported to the logs
  /// in release builds, if passed to [FlutterError.reportError]. (It is still
  /// sent to error handlers.) It should be set to true if the error is one that
  /// is expected to be encountered in release builds, for example network
  /// errors. That way, logs on end-user devices will not have spurious
  /// messages, but errors during development will still be reported.
  ///
  /// See [FlutterErrorDetails] for further details on these values.
  @pragma('vm:notify-debugger-on-exception')
  void reportError({
    DiagnosticsNode? context,
    required Object exception,
    StackTrace? stack,
    InformationCollector? informationCollector,
    bool silent = false,
  }) {
    _currentError = FlutterErrorDetails(
      exception: exception,
      stack: stack,
      library: 'image resource service',
      context: context,
      informationCollector: informationCollector,
      silent: silent,
    );

    // Make a copy to allow for concurrent modification.
    final List<ImageErrorListener> localErrorListeners = <ImageErrorListener>[
      ..._listeners
          .map<ImageErrorListener?>((ImageStreamListener listener) => listener.onError)
          .whereType<ImageErrorListener>(),
      ..._ephemeralErrorListeners,
    ];

    _ephemeralErrorListeners.clear();

    bool handled = false;
    for (final ImageErrorListener errorListener in localErrorListeners) {
      try {
        errorListener(exception, stack);
        handled = true;
      } catch (newException, newStack) {
        if (newException != exception) {
          FlutterError.reportError(
            FlutterErrorDetails(
              context: ErrorDescription('when reporting an error to an image listener'),
              library: 'image resource service',
              exception: newException,
              stack: newStack,
            ),
          );
        }
      }
    }
    if (!handled) {
      FlutterError.reportError(_currentError!);
    }
  }

  /// Calls all the registered [ImageChunkListener]s (listeners with an
  /// [ImageStreamListener.onChunk] specified) to notify them of a new
  /// [ImageChunkEvent].
  @protected
  void reportImageChunkEvent(ImageChunkEvent event) {
    _checkDisposed();
    if (hasListeners) {
      // Make a copy to allow for concurrent modification.
      final List<ImageChunkListener> localListeners =
          _listeners
              .map<ImageChunkListener?>((ImageStreamListener listener) => listener.onChunk)
              .whereType<ImageChunkListener>()
              .toList();
      for (final ImageChunkListener listener in localListeners) {
        listener(event);
      }
    }
  }

  /// Accumulates a list of strings describing the object's state. Subclasses
  /// should override this to have their information included in [toString].
  @override
  void debugFillProperties(DiagnosticPropertiesBuilder description) {
    super.debugFillProperties(description);
    description.add(
      DiagnosticsProperty<ImageInfo>(
        'current',
        _currentImage,
        ifNull: 'unresolved',
        showName: false,
      ),
    );
    description.add(
      ObjectFlagProperty<List<ImageStreamListener>>(
        'listeners',
        _listeners,
        ifPresent: '${_listeners.length} listener${_listeners.length == 1 ? "" : "s"}',
      ),
    );
    description.add(
      ObjectFlagProperty<List<ImageErrorListener>>(
        'ephemeralErrorListeners',
        _ephemeralErrorListeners,
        ifPresent:
            '${_ephemeralErrorListeners.length} ephemeralErrorListener${_ephemeralErrorListeners.length == 1 ? "" : "s"}',
      ),
    );
    description.add(FlagProperty('disposed', value: _disposed, ifTrue: '<disposed>'));
  }
}

/// Manages the loading of [dart:ui.Image] objects for static [ImageStream]s (those
/// with only one frame).
class OneFrameImageStreamCompleter extends ImageStreamCompleter {
  /// Creates a manager for one-frame [ImageStream]s.
  ///
  /// The image resource awaits the given [Future]. When the future resolves,
  /// it notifies the [ImageListener]s that have been registered with
  /// [addListener].
  ///
  /// The [InformationCollector], if provided, is invoked if the given [Future]
  /// resolves with an error, and can be used to supplement the reported error
  /// message (for example, giving the image's URL).
  ///
  /// Errors are reported using [FlutterError.reportError] with the `silent`
  /// argument on [FlutterErrorDetails] set to true, meaning that by default the
  /// message is only dumped to the console in debug mode (see [
  /// FlutterErrorDetails]).
  OneFrameImageStreamCompleter(
    Future<ImageInfo> image, {
    InformationCollector? informationCollector,
  }) {
    image.then<void>(
      setImage,
      onError: (Object error, StackTrace stack) {
        reportError(
          context: ErrorDescription('resolving a single-frame image stream'),
          exception: error,
          stack: stack,
          informationCollector: informationCollector,
          silent: true,
        );
      },
    );
  }
}

/// Manages the decoding and scheduling of image frames.
///
/// New frames will only be emitted while there are registered listeners to the
/// stream (registered with [addListener]).
///
/// This class deals with 2 types of frames:
///
///  * image frames - image frames of an animated image.
///  * app frames - frames that the flutter engine is drawing to the screen to
///    show the app GUI.
///
/// For single frame images the stream will only complete once.
///
/// For animated images, this class eagerly decodes the next image frame,
/// and notifies the listeners that a new frame is ready on the first app frame
/// that is scheduled after the image frame duration has passed.
///
/// Scheduling new timers only from scheduled app frames, makes sure we pause
/// the animation when the app is not visible (as new app frames will not be
/// scheduled).
///
/// See the following timeline example:
///
///     | Time | Event                                      | Comment                   |
///     |------|--------------------------------------------|---------------------------|
///     | t1   | App frame scheduled (image frame A posted) |                           |
///     | t2   | App frame scheduled                        |                           |
///     | t3   | App frame scheduled                        |                           |
///     | t4   | Image frame B decoded                      |                           |
///     | t5   | App frame scheduled                        | t5 - t1 < frameB_duration |
///     | t6   | App frame scheduled (image frame B posted) | t6 - t1 > frameB_duration |
///
class MultiFrameImageStreamCompleter extends ImageStreamCompleter {
  /// Creates a image stream completer.
  ///
  /// Immediately starts decoding the first image frame when the codec is ready.
  ///
  /// The `codec` parameter is a future for an initialized [ui.Codec] that will
  /// be used to decode the image.
  ///
  /// The `scale` parameter is the linear scale factor for drawing this frames
  /// of this image at their intended size.
  ///
  /// The `tag` parameter is passed on to created [ImageInfo] objects to
  /// help identify the source of the image.
  ///
  /// The `chunkEvents` parameter is an optional stream of notifications about
  /// the loading progress of the image. If this stream is provided, the events
  /// produced by the stream will be delivered to registered [ImageChunkListener]s
  /// (see [addListener]).
  MultiFrameImageStreamCompleter({
    required Future<ui.Codec> codec,
    required double scale,
    String? debugLabel,
    Stream<ImageChunkEvent>? chunkEvents,
    InformationCollector? informationCollector,
  }) : _informationCollector = informationCollector,
       _scale = scale {
    this.debugLabel = debugLabel;
    codec.then<void>(
      _handleCodecReady,
      onError: (Object error, StackTrace stack) {
        reportError(
          context: ErrorDescription('resolving an image codec'),
          exception: error,
          stack: stack,
          informationCollector: informationCollector,
          silent: true,
        );
      },
    );
    if (chunkEvents != null) {
      _chunkSubscription = chunkEvents.listen(
        reportImageChunkEvent,
        onError: (Object error, StackTrace stack) {
          reportError(
            context: ErrorDescription('loading an image'),
            exception: error,
            stack: stack,
            informationCollector: informationCollector,
            silent: true,
          );
        },
      );
    }
  }

  StreamSubscription<ImageChunkEvent>? _chunkSubscription;
  ui.Codec? _codec;
  final double _scale;
  final InformationCollector? _informationCollector;
  ui.FrameInfo? _nextFrame;
  // When the current was first shown.
  late Duration _shownTimestamp;
  // The requested duration for the current frame;
  Duration? _frameDuration;
  // How many frames have been emitted so far.
  int _framesEmitted = 0;
  Timer? _timer;

  // Used to guard against registering multiple _handleAppFrame callbacks for the same frame.
  bool _frameCallbackScheduled = false;

  void _handleCodecReady(ui.Codec codec) {
    _codec = codec;
    assert(_codec != null);

    if (hasListeners) {
      _decodeNextFrameAndSchedule();
    }
  }

  void _handleAppFrame(Duration timestamp) {
    _frameCallbackScheduled = false;
    if (!hasListeners) {
      return;
    }
    assert(_nextFrame != null);
    if (_isFirstFrame() || _hasFrameDurationPassed(timestamp)) {
      _emitFrame(
        ImageInfo(image: _nextFrame!.image.clone(), scale: _scale, debugLabel: debugLabel),
      );
      _shownTimestamp = timestamp;
      _frameDuration = _nextFrame!.duration;
      _nextFrame!.image.dispose();
      _nextFrame = null;
      final int completedCycles = _framesEmitted ~/ _codec!.frameCount;
      if (_codec!.repetitionCount == -1 || completedCycles <= _codec!.repetitionCount) {
        _decodeNextFrameAndSchedule();
      }
      return;
    }
    final Duration delay = _frameDuration! - (timestamp - _shownTimestamp);
    _timer = Timer(delay * timeDilation, () {
      _scheduleAppFrame();
    });
  }

  bool _isFirstFrame() {
    return _frameDuration == null;
  }

  bool _hasFrameDurationPassed(Duration timestamp) {
    return timestamp - _shownTimestamp >= _frameDuration!;
  }

  Future<void> _decodeNextFrameAndSchedule() async {
    // This will be null if we gave it away. If not, it's still ours and it
    // must be disposed of.
    _nextFrame?.image.dispose();
    _nextFrame = null;
    try {
      _nextFrame = await _codec!.getNextFrame();
    } catch (exception, stack) {
      reportError(
        context: ErrorDescription('resolving an image frame'),
        exception: exception,
        stack: stack,
        informationCollector: _informationCollector,
        silent: true,
      );
      return;
    }
    if (_codec!.frameCount == 1) {
      // ImageStreamCompleter listeners removed while waiting for next frame to
      // be decoded.
      // There's no reason to emit the frame without active listeners.
      if (!hasListeners) {
        return;
      }
      // This is not an animated image, just return it and don't schedule more
      // frames.
      _emitFrame(
        ImageInfo(image: _nextFrame!.image.clone(), scale: _scale, debugLabel: debugLabel),
      );
      _nextFrame!.image.dispose();
      _nextFrame = null;
      return;
    }
    _scheduleAppFrame();
  }

  void _scheduleAppFrame() {
    if (_frameCallbackScheduled) {
      return;
    }
    _frameCallbackScheduled = true;
    SchedulerBinding.instance.scheduleFrameCallback(_handleAppFrame);
  }

  void _emitFrame(ImageInfo imageInfo) {
    setImage(imageInfo);
    _framesEmitted += 1;
  }

  @override
  void addListener(ImageStreamListener listener) {
    if (!hasListeners && _codec != null && (_currentImage == null || _codec!.frameCount > 1)) {
      _decodeNextFrameAndSchedule();
    }
    super.addListener(listener);
  }

  @override
  void removeListener(ImageStreamListener listener) {
    super.removeListener(listener);
    if (!hasListeners) {
      _timer?.cancel();
      _timer = null;
    }
  }

  @override
  void _maybeDispose() {
    super._maybeDispose();
    if (_disposed) {
      _chunkSubscription?.onData(null);
      _chunkSubscription?.cancel();
      _chunkSubscription = null;
    }
  }
}
