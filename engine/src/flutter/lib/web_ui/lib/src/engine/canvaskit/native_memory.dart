// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';
import 'package:meta/meta.dart';
import 'package:ui/src/engine.dart';

/// Collects native objects that weren't explicitly disposed of using
/// [UniqueRef.dispose] or [CountedRef.unref].
///
/// We use this to delete Skia objects when their "Ck" wrapper is garbage
/// collected.
///
/// Example sequence of events:
///
/// 1. A (CkPaint, SkPaint) pair created.
/// 2. The paint is used to paint some picture.
/// 3. CkPaint is dropped by the app.
/// 4. GC decides to perform a GC cycle and collects CkPaint.
/// 5. The finalizer function is called with the SkPaint as the sole argument.
/// 6. We call `delete` on SkPaint.
DomFinalizationRegistry _finalizationRegistry = DomFinalizationRegistry(
  ((ExternalDartReference<UniqueRef<Object>> boxedUniq) => boxedUniq.toDartObject.collect()).toJS,
);

NativeMemoryFinalizationRegistry nativeMemoryFinalizationRegistry =
    NativeMemoryFinalizationRegistry();

/// An indirection to [DomFinalizationRegistry] to enable tests provide a
/// mock implementation of a finalization registry.
class NativeMemoryFinalizationRegistry {
  void register(Object owner, UniqueRef<Object> ref) {
    if (browserSupportsFinalizationRegistry) {
      _finalizationRegistry.register(owner.toExternalReference, ref.toExternalReference);
    }
  }
}

/// Manages the lifecycle of a C++ object referenced by a single Dart object.
///
/// It is expected that when the C++ object is no longer needed [dispose] is
/// called.
///
/// To prevent memory leaks, the underlying C++ object is deleted by the GC if
/// it wasn't previously disposed of explicitly.
class UniqueRef<T extends Object> {
  UniqueRef(Object owner, T nativeObject, this._debugOwnerLabel) {
    _nativeObject = nativeObject;
    if (Instrumentation.enabled) {
      Instrumentation.instance.incrementCounter('$_debugOwnerLabel Created');
    }
    nativeMemoryFinalizationRegistry.register(owner, this);
  }

  T? _nativeObject;
  final String _debugOwnerLabel;

  /// Returns the underlying native object reference, if it has not been
  /// disposed of yet.
  ///
  /// The returned reference must not be stored. I should only be borrowed
  /// temporarily. Storing this reference may result in dangling pointer errors.
  T get nativeObject {
    assert(!isDisposed, 'The native object of $_debugOwnerLabel was disposed.');
    return _nativeObject!;
  }

  /// Returns whether the underlying native object has been disposed and
  /// therefore can no longer be used.
  bool get isDisposed => _nativeObject == null;

  /// Disposes the underlying native object.
  ///
  /// The underlying object may be deleted or its ref count may be bumped down.
  /// The exact action taken depends on the sharing model of that particular
  /// object. For example, an [SkImage] may not be immediately deleted if a
  /// [SkPicture] exists that still references it. On the other hand, [SkPaint]
  /// is deleted eagerly.
  void dispose() {
    assert(!isDisposed, 'A native object reference cannot be disposed more than once.');
    if (Instrumentation.enabled) {
      Instrumentation.instance.incrementCounter('$_debugOwnerLabel Deleted');
    }
    final SkDeletable object = nativeObject as SkDeletable;
    if (!object.isDeleted()) {
      object.delete();
    }
    _nativeObject = null;
  }

  /// Called by the garbage [Collector] when the owner of this handle is
  /// collected.
  ///
  /// Garbage collection is used as a back-up for the cases when the handle
  /// isn't disposed of explicitly by calling [dispose]. It most likely
  /// indicates a memory leak or inefficiency in the framework or application
  /// code.
  @visibleForTesting
  void collect() {
    if (!isDisposed) {
      if (Instrumentation.enabled) {
        Instrumentation.instance.incrementCounter('$_debugOwnerLabel Leaked');
      }
      dispose();
    }
  }
}

/// Interface that classes wrapping [UniqueRef] must implement.
///
/// Used to collect stack traces in debug mode.
abstract class StackTraceDebugger {
  /// The stack trace pointing to code location that created or upreffed a
  /// [CountedRef].
  StackTrace get debugStackTrace;
}

/// Manages the lifecycle of a C++ object referenced by multiple Dart objects.
///
/// Uses reference counting to manage the lifecycle of the C++ object.
///
/// If the C++ object has a unique owner, use [UniqueRef] instead.
///
/// The [ref] method can be used to increment the refcount to tell this box to
/// keep the underlying C++ object alive.
///
/// The [unref] method can be used to decrement the refcount indicating that a
/// referring object no longer needs it. When the refcount drops to zero the
/// underlying C++ object is deleted.
///
/// In addition to ref counting, this object is also managed by GC. When this
/// reference is garbage collected, the underlying C++ object is automatically
/// deleted. This is mostly done to prevent memory leaks in production. Well
/// behaving framework and app code are expected to rely on [ref] and [unref]
/// for timely collection of resources.
class CountedRef<R extends StackTraceDebugger, T extends Object> {
  /// Creates a counted reference.
  CountedRef(T nativeObject, R debugReferrer, String debugLabel) {
    _ref = UniqueRef<T>(this, nativeObject, debugLabel);
    assert(() {
      debugReferrers.add(debugReferrer);
      return true;
    }());
    assert(refCount == debugReferrers.length);
  }

  /// The native object reference whose lifecycle is being managed by this ref
  /// count.
  ///
  /// Do not store this value outside this class.
  late final UniqueRef<T> _ref;

  /// Returns the underlying native object reference, if it has not been
  /// disposed of yet.
  ///
  /// The returned reference must not be stored. I should only be borrowed
  /// temporarily. Storing this reference may result in dangling pointer errors.
  T get nativeObject => _ref.nativeObject;

  /// The number of objects sharing references to this box.
  ///
  /// When this count reaches zero, the underlying [nativeObject] is scheduled
  /// for deletion.
  int get refCount => _refCount;
  int _refCount = 1;

  /// Whether the underlying [nativeObject] has been disposed and is no longer
  /// accessible.
  bool get isDisposed => _ref.isDisposed;

  /// When assertions are enabled, stores all objects that share this box.
  ///
  /// The length of this list is always identical to [refCount].
  ///
  /// This list can be used for debugging ref counting issues.
  final Set<R> debugReferrers = <R>{};

  /// If asserts are enabled, the [StackTrace]s representing when a reference
  /// was created.
  List<StackTrace> debugGetStackTraces() {
    List<StackTrace>? result;
    assert(() {
      result = debugReferrers.map<StackTrace>((R referrer) => referrer.debugStackTrace).toList();
      return true;
    }());

    if (result != null) {
      return result!;
    }

    throw UnsupportedError('');
  }

  /// Increases the reference count of this box because a new object began
  /// sharing ownership of the underlying [nativeObject].
  void ref(R debugReferrer) {
    assert(!_ref.isDisposed, 'Cannot increment ref count on a deleted handle.');
    assert(_refCount > 0);
    assert(
      debugReferrers.add(debugReferrer),
      'Attempted to increment ref count by the same referrer more than once.',
    );
    _refCount += 1;
    assert(refCount == debugReferrers.length);
  }

  /// Decrements the reference count for the [nativeObject].
  ///
  /// Does nothing if the object has already been deleted.
  ///
  /// If this causes the reference count to drop to zero, deletes the
  /// [nativeObject].
  void unref(R debugReferrer) {
    assert(!_ref.isDisposed, 'Attempted to unref an already deleted native object.');
    assert(
      debugReferrers.remove(debugReferrer),
      'Attempted to decrement ref count by the same referrer more than once.',
    );
    _refCount -= 1;
    assert(refCount == debugReferrers.length);
    if (_refCount == 0) {
      _ref.dispose();
    }
  }
}
