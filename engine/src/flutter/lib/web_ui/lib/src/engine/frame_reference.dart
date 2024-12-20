// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// A monotonically increasing frame number being rendered.
///
/// Used for debugging only.
int debugFrameNumber = 1;

List<FrameReference<dynamic>> frameReferences = <FrameReference<dynamic>>[];

/// A temporary reference to a value of type [V].
///
/// The value automatically gets set to null after the current frame is
/// rendered.
///
/// It is useful to think of this as a weak reference that's scoped to a
/// single frame.
class FrameReference<V> {
  /// Creates a frame reference to a value.
  FrameReference([this.value]) {
    frameReferences.add(this);
  }

  /// The current value of this reference.
  V? value;
}

/// Cache where items cached before frame(N) is committed, can be reused in
/// frame(N+1) and are evicted if not.
///
/// A typical use case is image elements. As images are created and added to
/// DOM when painting a scene they are cached and if possible reused on next
/// update. If the next update does not reuse the element, it is evicted.
///
/// Maps are lazily allocated since many pictures don't contain cachable images
/// at all.
class CrossFrameCache<T> {
  // Cached items in a scene.
  Map<String, List<_CrossFrameCacheItem<T>>>? _cache;

  // Cached items that have been committed, ready for reuse on next frame.
  Map<String, List<_CrossFrameCacheItem<T>>>? _reusablePool;

  // Called when a scene or picture update is committed.
  void commitFrame() {
    // Evict unused items from prior frame.
    if (_reusablePool != null) {
      for (final List<_CrossFrameCacheItem<T>> items in _reusablePool!.values) {
        for (final _CrossFrameCacheItem<T> item in items) {
          item.evict();
        }
      }
    }
    // Move cached items to reusable pool.
    _reusablePool = _cache;
    _cache = null;
  }

  /// Caches an item for reuse on next update frame.
  ///
  /// Duplicate keys are allowed. For example the same image url may be used
  /// to create multiple instances of [ImageElement] to be reused in the future.
  void cache(String key, T value, [CrossFrameCacheEvictCallback<T>? callback]) {
    _addToCache(key, _CrossFrameCacheItem<T>(value, callback));
  }

  void _addToCache(String key, _CrossFrameCacheItem<T> item) {
    _cache ??= <String, List<_CrossFrameCacheItem<T>>>{};
    (_cache![key] ??= <_CrossFrameCacheItem<T>>[]).add(item);
  }

  /// Given a key, consumes an item that has been cached in a prior frame.
  T? reuse(String key) {
    if (_reusablePool == null) {
      return null;
    }
    final List<_CrossFrameCacheItem<T>>? items = _reusablePool![key];
    if (items == null || items.isEmpty) {
      return null;
    }
    final _CrossFrameCacheItem<T> item = items.removeAt(0);
    _addToCache(key, item);
    return item.value;
  }
}

class _CrossFrameCacheItem<T> {
  _CrossFrameCacheItem(this.value, this.evictCallback);
  final T value;
  final CrossFrameCacheEvictCallback<T>? evictCallback;
  void evict() {
    if (evictCallback != null) {
      evictCallback!(value);
    }
  }
}

typedef CrossFrameCacheEvictCallback<T> = void Function(T value);
