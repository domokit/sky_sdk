// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
///
/// @docImport 'binding.dart';
/// @docImport 'image_provider.dart';
library;

import 'dart:developer';

import 'package:flutter/foundation.dart';
import 'package:flutter/scheduler.dart';

import 'image_stream.dart';

const int _kDefaultSize = 1000;
const int _kDefaultSizeBytes = 100 << 20; // 100 MiB

/// Class for caching images.
///
/// Implements a least-recently-used cache of up to 1000 images, and up to 100
/// MB. The maximum size can be adjusted using [maximumSize] and
/// [maximumSizeBytes].
///
/// The cache also holds a list of 'live' references. An image is considered
/// live if its [ImageStreamCompleter]'s listener count has never dropped to
/// zero after adding at least one listener. The cache uses
/// [ImageStreamCompleter.addOnLastListenerRemovedCallback] to determine when
/// this has happened.
///
/// The [putIfAbsent] method is the main entry-point to the cache API. It
/// returns the previously cached [ImageStreamCompleter] for the given key, if
/// available; if not, it calls the given callback to obtain it first. In either
/// case, the key is moved to the 'most recently used' position.
///
/// A caller can determine whether an image is already in the cache by using
/// [containsKey], which will return true if the image is tracked by the cache
/// in a pending or completed state. More fine grained information is available
/// by using the [statusForKey] method.
///
/// Generally this class is not used directly. The [ImageProvider] class and its
/// subclasses automatically handle the caching of images.
///
/// A shared instance of this cache is retained by [PaintingBinding] and can be
/// obtained via the [imageCache] top-level property in the [painting] library.
///
/// {@tool snippet}
///
/// This sample shows how to supply your own caching logic and replace the
/// global [imageCache] variable.
///
/// ```dart
/// /// This is the custom implementation of [ImageCache] where we can override
/// /// the logic.
/// class MyImageCache extends ImageCache {
///   @override
///   void clear() {
///     print('Clearing cache!');
///     super.clear();
///   }
/// }
///
/// class MyWidgetsBinding extends WidgetsFlutterBinding {
///   @override
///   ImageCache createImageCache() => MyImageCache();
/// }
///
/// void main() {
///   // The constructor sets global variables.
///   MyWidgetsBinding();
///   runApp(const MyApp());
/// }
///
/// class MyApp extends StatelessWidget {
///   const MyApp({super.key});
///
///   @override
///   Widget build(BuildContext context) {
///     return Container();
///   }
/// }
/// ```
/// {@end-tool}
class ImageCache {
  final Map<Object, _PendingImage> _pendingImages = <Object, _PendingImage>{};
  final Map<Object, _CachedImage> _cache = <Object, _CachedImage>{};

  /// ImageStreamCompleters with at least one listener. These images may or may
  /// not fit into the _pendingImages or _cache objects.
  ///
  /// Unlike _cache, the [_CachedImage] for this may have a null byte size.
  final Map<Object, _LiveImage> _liveImages = <Object, _LiveImage>{};

  /// Maximum number of entries to store in the cache.
  ///
  /// Once this many entries have been cached, the least-recently-used entry is
  /// evicted when adding a new entry.
  int get maximumSize => _maximumSize;
  int _maximumSize = _kDefaultSize;

  /// Changes the maximum cache size.
  ///
  /// If the new size is smaller than the current number of elements, the
  /// extraneous elements are evicted immediately. Setting this to zero and then
  /// returning it to its original value will therefore immediately clear the
  /// cache.
  set maximumSize(int value) {
    assert(value >= 0);
    if (value == maximumSize) {
      return;
    }
    TimelineTask? debugTimelineTask;
    if (!kReleaseMode) {
      debugTimelineTask =
          TimelineTask()
            ..start('ImageCache.setMaximumSize', arguments: <String, dynamic>{'value': value});
    }
    _maximumSize = value;
    if (maximumSize == 0) {
      clear();
    } else {
      _checkCacheSize(debugTimelineTask);
    }
    if (!kReleaseMode) {
      debugTimelineTask!.finish();
    }
  }

  /// The current number of cached entries.
  int get currentSize => _cache.length;

  /// Maximum size of entries to store in the cache in bytes.
  ///
  /// Once more than this amount of bytes have been cached, the
  /// least-recently-used entry is evicted until there are fewer than the
  /// maximum bytes.
  int get maximumSizeBytes => _maximumSizeBytes;
  int _maximumSizeBytes = _kDefaultSizeBytes;

  /// Changes the maximum cache bytes.
  ///
  /// If the new size is smaller than the current size in bytes, the
  /// extraneous elements are evicted immediately. Setting this to zero and then
  /// returning it to its original value will therefore immediately clear the
  /// cache.
  set maximumSizeBytes(int value) {
    assert(value >= 0);
    if (value == _maximumSizeBytes) {
      return;
    }
    TimelineTask? debugTimelineTask;
    if (!kReleaseMode) {
      debugTimelineTask =
          TimelineTask()
            ..start('ImageCache.setMaximumSizeBytes', arguments: <String, dynamic>{'value': value});
    }
    _maximumSizeBytes = value;
    if (_maximumSizeBytes == 0) {
      clear();
    } else {
      _checkCacheSize(debugTimelineTask);
    }
    if (!kReleaseMode) {
      debugTimelineTask!.finish();
    }
  }

  /// The current size of cached entries in bytes.
  int get currentSizeBytes => _currentSizeBytes;
  int _currentSizeBytes = 0;

  /// Evicts all pending and keepAlive entries from the cache.
  ///
  /// This is useful if, for instance, the root asset bundle has been updated
  /// and therefore new images must be obtained.
  ///
  /// Images which have not finished loading yet will not be removed from the
  /// cache, and when they complete they will be inserted as normal.
  ///
  /// This method does not clear live references to images, since clearing those
  /// would not reduce memory pressure. Such images still have listeners in the
  /// application code, and will still remain resident in memory.
  ///
  /// To clear live references, use [clearLiveImages].
  void clear() {
    if (!kReleaseMode) {
      Timeline.instantSync(
        'ImageCache.clear',
        arguments: <String, dynamic>{
          'pendingImages': _pendingImages.length,
          'keepAliveImages': _cache.length,
          'liveImages': _liveImages.length,
          'currentSizeInBytes': _currentSizeBytes,
        },
      );
    }
    for (final _CachedImage image in _cache.values) {
      image.dispose();
    }
    _cache.clear();
    for (final _PendingImage pendingImage in _pendingImages.values) {
      pendingImage.removeListener();
    }
    _pendingImages.clear();
    _currentSizeBytes = 0;
  }

  /// Evicts a single entry from the cache, returning true if successful.
  ///
  /// Pending images waiting for completion are removed as well, returning true
  /// if successful. When a pending image is removed the listener on it is
  /// removed as well to prevent it from adding itself to the cache if it
  /// eventually completes.
  ///
  /// If this method removes a pending image, it will also remove
  /// the corresponding live tracking of the image, since it is no longer clear
  /// if the image will ever complete or have any listeners, and failing to
  /// remove the live reference could leave the cache in a state where all
  /// subsequent calls to [putIfAbsent] will return an [ImageStreamCompleter]
  /// that will never complete.
  ///
  /// If this method removes a completed image, it will _not_ remove the live
  /// reference to the image, which will only be cleared when the listener
  /// count on the completer drops to zero. To clear live image references,
  /// whether completed or not, use [clearLiveImages].
  ///
  /// The `key` must be equal to an object used to cache an image in
  /// [ImageCache.putIfAbsent].
  ///
  /// If the key is not immediately available, as is common, consider using
  /// [ImageProvider.evict] to call this method indirectly instead.
  ///
  /// The `includeLive` argument determines whether images that still have
  /// listeners in the tree should be evicted as well. This parameter should be
  /// set to true in cases where the image may be corrupted and needs to be
  /// completely discarded by the cache. It should be set to false when calls
  /// to evict are trying to relieve memory pressure, since an image with a
  /// listener will not actually be evicted from memory, and subsequent attempts
  /// to load it will end up allocating more memory for the image again.
  ///
  /// See also:
  ///
  ///  * [ImageProvider], for providing images to the [Image] widget.
  bool evict(Object key, {bool includeLive = true}) {
    if (includeLive) {
      // Remove from live images - the cache will not be able to mark
      // it as complete, and it might be getting evicted because it
      // will never complete, e.g. it was loaded in a FakeAsync zone.
      // In such a case, we need to make sure subsequent calls to
      // putIfAbsent don't return this image that may never complete.
      final _LiveImage? image = _liveImages.remove(key);
      image?.dispose();
    }
    final _PendingImage? pendingImage = _pendingImages.remove(key);
    if (pendingImage != null) {
      if (!kReleaseMode) {
        Timeline.instantSync('ImageCache.evict', arguments: <String, dynamic>{'type': 'pending'});
      }
      pendingImage.removeListener();
      return true;
    }
    final _CachedImage? image = _cache.remove(key);
    if (image != null) {
      if (!kReleaseMode) {
        Timeline.instantSync(
          'ImageCache.evict',
          arguments: <String, dynamic>{'type': 'keepAlive', 'sizeInBytes': image.sizeBytes},
        );
      }
      _currentSizeBytes -= image.sizeBytes!;
      image.dispose();
      return true;
    }
    if (!kReleaseMode) {
      Timeline.instantSync('ImageCache.evict', arguments: <String, dynamic>{'type': 'miss'});
    }
    return false;
  }

  /// Updates the least recently used image cache with this image, if it is
  /// less than the [maximumSizeBytes] of this cache.
  ///
  /// Resizes the cache as appropriate to maintain the constraints of
  /// [maximumSize] and [maximumSizeBytes].
  void _touch(Object key, _CachedImage image, TimelineTask? timelineTask) {
    if (image.sizeBytes != null && image.sizeBytes! <= maximumSizeBytes && maximumSize > 0) {
      _currentSizeBytes += image.sizeBytes!;
      _cache[key] = image;
      _checkCacheSize(timelineTask);
    } else {
      image.dispose();
    }
  }

  void _trackLiveImage(Object key, ImageStreamCompleter completer, int? sizeBytes) {
    // Avoid adding unnecessary callbacks to the completer.
    _liveImages.putIfAbsent(key, () {
          // Even if no callers to ImageProvider.resolve have listened to the stream,
          // the cache is listening to the stream and will remove itself once the
          // image completes to move it from pending to keepAlive.
          // Even if the cache size is 0, we still add this tracker, which will add
          // a keep alive handle to the stream.
          return _LiveImage(completer, () {
            _liveImages.remove(key);
          });
        }).sizeBytes ??=
        sizeBytes;
  }

  /// Returns the previously cached [ImageStream] for the given key, if available;
  /// if not, calls the given callback to obtain it first. In either case, the
  /// key is moved to the 'most recently used' position.
  ///
  /// In the event that the loader throws an exception, it will be caught only if
  /// `onError` is also provided. When an exception is caught resolving an image,
  /// no completers are cached and `null` is returned instead of a new
  /// completer.
  ///
  /// Images that are larger than [maximumSizeBytes] are not cached, and do not
  /// cause other images in the cache to be evicted.
  ImageStreamCompleter? putIfAbsent(
    Object key,
    ImageStreamCompleter Function() loader, {
    ImageErrorListener? onError,
  }) {
    TimelineTask? debugTimelineTask;
    if (!kReleaseMode) {
      debugTimelineTask =
          TimelineTask()
            ..start('ImageCache.putIfAbsent', arguments: <String, dynamic>{'key': key.toString()});
    }
    ImageStreamCompleter? result = _pendingImages[key]?.completer;
    // Nothing needs to be done because the image hasn't loaded yet.
    if (result != null) {
      if (!kReleaseMode) {
        debugTimelineTask!.finish(arguments: <String, dynamic>{'result': 'pending'});
      }
      return result;
    }
    // Remove the provider from the list so that we can move it to the
    // recently used position below.
    // Don't use _touch here, which would trigger a check on cache size that is
    // not needed since this is just moving an existing cache entry to the head.
    final _CachedImage? image = _cache.remove(key);
    if (image != null) {
      if (!kReleaseMode) {
        debugTimelineTask!.finish(arguments: <String, dynamic>{'result': 'keepAlive'});
      }
      // The image might have been keptAlive but had no listeners (so not live).
      // Make sure the cache starts tracking it as live again.
      _trackLiveImage(key, image.completer, image.sizeBytes);
      _cache[key] = image;
      return image.completer;
    }

    final _LiveImage? liveImage = _liveImages[key];
    if (liveImage != null) {
      _touch(
        key,
        _CachedImage(liveImage.completer, sizeBytes: liveImage.sizeBytes),
        debugTimelineTask,
      );
      if (!kReleaseMode) {
        debugTimelineTask!.finish(arguments: <String, dynamic>{'result': 'keepAlive'});
      }
      return liveImage.completer;
    }

    try {
      result = loader();
      _trackLiveImage(key, result, null);
    } catch (error, stackTrace) {
      if (!kReleaseMode) {
        debugTimelineTask!.finish(
          arguments: <String, dynamic>{
            'result': 'error',
            'error': error.toString(),
            'stackTrace': stackTrace.toString(),
          },
        );
      }
      if (onError != null) {
        onError(error, stackTrace);
        return null;
      } else {
        rethrow;
      }
    }

    if (!kReleaseMode) {
      debugTimelineTask!.start('listener');
    }
    // A multi-frame provider may call the listener more than once. We need do make
    // sure that some cleanup works won't run multiple times, such as finishing the
    // tracing task or removing the listeners
    bool listenedOnce = false;

    // We shouldn't use the _pendingImages map if the cache is disabled, but we
    // will have to listen to the image at least once so we don't leak it in
    // the live image tracking.
    final bool trackPendingImage = maximumSize > 0 && maximumSizeBytes > 0;
    late _PendingImage pendingImage;
    void listener(ImageInfo? info, bool syncCall) {
      int? sizeBytes;
      if (info != null) {
        sizeBytes = info.sizeBytes;
        info.dispose();
      }
      final _CachedImage image = _CachedImage(result!, sizeBytes: sizeBytes);

      _trackLiveImage(key, result, sizeBytes);

      // Only touch if the cache was enabled when resolve was initially called.
      if (trackPendingImage) {
        _touch(key, image, debugTimelineTask);
      } else {
        image.dispose();
      }

      _pendingImages.remove(key);
      if (!listenedOnce) {
        pendingImage.removeListener();
      }
      if (!kReleaseMode && !listenedOnce) {
        debugTimelineTask!
          ..finish(arguments: <String, dynamic>{'syncCall': syncCall, 'sizeInBytes': sizeBytes})
          ..finish(
            arguments: <String, dynamic>{
              'currentSizeBytes': currentSizeBytes,
              'currentSize': currentSize,
            },
          );
      }
      listenedOnce = true;
    }

    final ImageStreamListener streamListener = ImageStreamListener(listener);
    pendingImage = _PendingImage(result, streamListener);
    if (trackPendingImage) {
      _pendingImages[key] = pendingImage;
    }
    // Listener is removed in [_PendingImage.removeListener].
    result.addListener(streamListener);

    return result;
  }

  /// The [ImageCacheStatus] information for the given `key`.
  ImageCacheStatus statusForKey(Object key) {
    return ImageCacheStatus._(
      pending: _pendingImages.containsKey(key),
      keepAlive: _cache.containsKey(key),
      live: _liveImages.containsKey(key),
    );
  }

  /// Returns whether this `key` has been previously added by [putIfAbsent].
  bool containsKey(Object key) {
    return _pendingImages[key] != null || _cache[key] != null;
  }

  /// The number of live images being held by the [ImageCache].
  ///
  /// Compare with [ImageCache.currentSize] for keepAlive images.
  int get liveImageCount => _liveImages.length;

  /// The number of images being tracked as pending in the [ImageCache].
  ///
  /// Compare with [ImageCache.currentSize] for keepAlive images.
  int get pendingImageCount => _pendingImages.length;

  /// Clears any live references to images in this cache.
  ///
  /// An image is considered live if its [ImageStreamCompleter] has never hit
  /// zero listeners after adding at least one listener. The
  /// [ImageStreamCompleter.addOnLastListenerRemovedCallback] is used to
  /// determine when this has happened.
  ///
  /// This is called after a hot reload to evict any stale references to image
  /// data for assets that have changed. Calling this method does not relieve
  /// memory pressure, since the live image caching only tracks image instances
  /// that are also being held by at least one other object.
  void clearLiveImages() {
    for (final _LiveImage image in _liveImages.values) {
      image.dispose();
    }
    _liveImages.clear();
  }

  // Remove images from the cache until both the length and bytes are below
  // maximum, or the cache is empty.
  void _checkCacheSize(TimelineTask? timelineTask) {
    final Map<String, dynamic> finishArgs = <String, dynamic>{};
    if (!kReleaseMode) {
      timelineTask!.start('checkCacheSize');
      finishArgs['evictedKeys'] = <String>[];
      finishArgs['currentSize'] = currentSize;
      finishArgs['currentSizeBytes'] = currentSizeBytes;
    }
    while (_currentSizeBytes > _maximumSizeBytes || _cache.length > _maximumSize) {
      final Object key = _cache.keys.first;
      final _CachedImage image = _cache[key]!;
      _currentSizeBytes -= image.sizeBytes!;
      image.dispose();
      _cache.remove(key);
      if (!kReleaseMode) {
        (finishArgs['evictedKeys'] as List<String>).add(key.toString());
      }
    }
    if (!kReleaseMode) {
      finishArgs['endSize'] = currentSize;
      finishArgs['endSizeBytes'] = currentSizeBytes;
      timelineTask!.finish(arguments: finishArgs);
    }
    assert(_currentSizeBytes >= 0);
    assert(_cache.length <= maximumSize);
    assert(_currentSizeBytes <= maximumSizeBytes);
  }
}

/// Information about how the [ImageCache] is tracking an image.
///
/// A [pending] image is one that has not completed yet. It may also be tracked
/// as [live] because something is listening to it.
///
/// A [keepAlive] image is being held in the cache, which uses Least Recently
/// Used semantics to determine when to evict an image. These images are subject
/// to eviction based on [ImageCache.maximumSizeBytes] and
/// [ImageCache.maximumSize]. It may be [live], but not [pending].
///
/// A [live] image is being held until its [ImageStreamCompleter] has no more
/// listeners. It may also be [pending] or [keepAlive].
///
/// An [untracked] image is not being cached.
///
/// To obtain an [ImageCacheStatus], use [ImageCache.statusForKey] or
/// [ImageProvider.obtainCacheStatus].
@immutable
class ImageCacheStatus {
  const ImageCacheStatus._({this.pending = false, this.keepAlive = false, this.live = false})
    : assert(!pending || !keepAlive);

  /// An image that has been submitted to [ImageCache.putIfAbsent], but
  /// not yet completed.
  final bool pending;

  /// An image that has been submitted to [ImageCache.putIfAbsent], has
  /// completed, fits based on the sizing rules of the cache, and has not been
  /// evicted.
  ///
  /// Such images will be kept alive even if [live] is false, as long
  /// as they have not been evicted from the cache based on its sizing rules.
  final bool keepAlive;

  /// An image that has been submitted to [ImageCache.putIfAbsent] and has at
  /// least one listener on its [ImageStreamCompleter].
  ///
  /// Such images may also be [keepAlive] if they fit in the cache based on its
  /// sizing rules. They may also be [pending] if they have not yet resolved.
  final bool live;

  /// An image that is tracked in some way by the [ImageCache], whether
  /// [pending], [keepAlive], or [live].
  bool get tracked => pending || keepAlive || live;

  /// An image that either has not been submitted to
  /// [ImageCache.putIfAbsent] or has otherwise been evicted from the
  /// [keepAlive] and [live] caches.
  bool get untracked => !pending && !keepAlive && !live;

  @override
  bool operator ==(Object other) {
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ImageCacheStatus &&
        other.pending == pending &&
        other.keepAlive == keepAlive &&
        other.live == live;
  }

  @override
  int get hashCode => Object.hash(pending, keepAlive, live);

  @override
  String toString() =>
      '${objectRuntimeType(this, 'ImageCacheStatus')}(pending: $pending, live: $live, keepAlive: $keepAlive)';
}

/// Base class for [_CachedImage] and [_LiveImage].
///
/// Exists primarily so that a [_LiveImage] cannot be added to the
/// [ImageCache._cache].
abstract class _CachedImageBase {
  _CachedImageBase(this.completer, {this.sizeBytes}) : handle = completer.keepAlive() {
    // TODO(polina-c): stop duplicating code across disposables
    // https://github.com/flutter/flutter/issues/137435
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectCreated(
        library: 'package:flutter/painting.dart',
        className: '$_CachedImageBase',
        object: this,
      );
    }
  }

  final ImageStreamCompleter completer;
  int? sizeBytes;
  ImageStreamCompleterHandle? handle;

  @mustCallSuper
  void dispose() {
    assert(handle != null);
    if (kFlutterMemoryAllocationsEnabled) {
      FlutterMemoryAllocations.instance.dispatchObjectDisposed(object: this);
    }
    // Give any interested parties a chance to listen to the stream before we
    // potentially dispose it.
    SchedulerBinding.instance.addPostFrameCallback((Duration timeStamp) {
      assert(handle != null);
      handle?.dispose();
      handle = null;
    }, debugLabel: 'CachedImage.disposeHandle');
  }
}

class _CachedImage extends _CachedImageBase {
  _CachedImage(super.completer, {super.sizeBytes});
}

class _LiveImage extends _CachedImageBase {
  _LiveImage(ImageStreamCompleter completer, VoidCallback handleRemove, {int? sizeBytes})
    : super(completer, sizeBytes: sizeBytes) {
    _handleRemove = () {
      handleRemove();
      dispose();
    };
    completer.addOnLastListenerRemovedCallback(_handleRemove);
  }

  late VoidCallback _handleRemove;

  @override
  void dispose() {
    completer.removeOnLastListenerRemovedCallback(_handleRemove);
    super.dispose();
  }

  @override
  String toString() => describeIdentity(this);
}

class _PendingImage {
  _PendingImage(this.completer, this.listener);

  final ImageStreamCompleter completer;
  final ImageStreamListener listener;

  void removeListener() {
    completer.removeListener(listener);
  }
}
