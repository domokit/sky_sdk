// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/src/engine.dart';

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  setUpCanvasKitTest();

  late _MockNativeMemoryFinalizationRegistry mockFinalizationRegistry;

  setUp(() {
    TestSkDeletableMock.deleteCount = 0;
    nativeMemoryFinalizationRegistry =
        mockFinalizationRegistry = _MockNativeMemoryFinalizationRegistry();
  });

  tearDown(() {
    nativeMemoryFinalizationRegistry = NativeMemoryFinalizationRegistry();
  });

  group(UniqueRef, () {
    test('create-dispose-collect cycle', () {
      expect(mockFinalizationRegistry.registeredPairs, hasLength(0));
      final Object owner = Object();
      final TestSkDeletable nativeObject = TestSkDeletable();
      final UniqueRef<TestSkDeletable> ref = UniqueRef<TestSkDeletable>(
        owner,
        nativeObject,
        'TestSkDeletable',
      );
      expect(ref.isDisposed, isFalse);
      expect(ref.nativeObject, same(nativeObject));
      expect(TestSkDeletableMock.deleteCount, 0);
      expect(mockFinalizationRegistry.registeredPairs, hasLength(1));
      expect(mockFinalizationRegistry.registeredPairs.single.owner, same(owner));
      expect(mockFinalizationRegistry.registeredPairs.single.ref, same(ref));

      ref.dispose();
      expect(TestSkDeletableMock.deleteCount, 1);
      expect(ref.isDisposed, isTrue);
      expect(
        reason: 'Cannot access object that was disposed',
        () => ref.nativeObject,
        throwsA(isA<AssertionError>()),
      );
      expect(
        reason: 'Cannot dispose object more than once',
        () => ref.dispose(),
        throwsA(isA<AssertionError>()),
      );
      expect(TestSkDeletableMock.deleteCount, 1);

      // Simulate a GC
      mockFinalizationRegistry.registeredPairs.single.ref.collect();
      expect(
        reason: 'Manually disposed object should not be deleted again by GC.',
        TestSkDeletableMock.deleteCount,
        1,
      );
    });

    test('create-collect cycle', () {
      expect(mockFinalizationRegistry.registeredPairs, hasLength(0));
      final Object owner = Object();
      final TestSkDeletable nativeObject = TestSkDeletable();
      final UniqueRef<TestSkDeletable> ref = UniqueRef<TestSkDeletable>(
        owner,
        nativeObject,
        'TestSkDeletable',
      );
      expect(ref.isDisposed, isFalse);
      expect(ref.nativeObject, same(nativeObject));
      expect(TestSkDeletableMock.deleteCount, 0);
      expect(mockFinalizationRegistry.registeredPairs, hasLength(1));

      ref.collect();
      expect(TestSkDeletableMock.deleteCount, 1);
      // There's nothing else to test for any practical gain. UniqueRef.collect
      // is called when GC decided that the owner is no longer reachable. So
      // there must not be anything else calling into this object for anything
      // useful.
    });

    test('dispose instrumentation', () {
      Instrumentation.enabled = true;
      Instrumentation.instance.debugCounters.clear();

      final Object owner = Object();
      final TestSkDeletable nativeObject = TestSkDeletable();

      expect(Instrumentation.instance.debugCounters, <String, int>{});
      final UniqueRef<TestSkDeletable> ref = UniqueRef<TestSkDeletable>(
        owner,
        nativeObject,
        'TestSkDeletable',
      );
      expect(Instrumentation.instance.debugCounters, <String, int>{'TestSkDeletable Created': 1});
      ref.dispose();
      expect(Instrumentation.instance.debugCounters, <String, int>{
        'TestSkDeletable Created': 1,
        'TestSkDeletable Deleted': 1,
      });
    });

    test('collect instrumentation', () {
      Instrumentation.enabled = true;
      Instrumentation.instance.debugCounters.clear();

      final Object owner = Object();
      final TestSkDeletable nativeObject = TestSkDeletable();

      expect(Instrumentation.instance.debugCounters, <String, int>{});
      final UniqueRef<TestSkDeletable> ref = UniqueRef<TestSkDeletable>(
        owner,
        nativeObject,
        'TestSkDeletable',
      );
      expect(Instrumentation.instance.debugCounters, <String, int>{'TestSkDeletable Created': 1});
      ref.collect();
      expect(Instrumentation.instance.debugCounters, <String, int>{
        'TestSkDeletable Created': 1,
        'TestSkDeletable Leaked': 1,
        'TestSkDeletable Deleted': 1,
      });
    });
  });

  group(CountedRef, () {
    test('single owner', () {
      expect(mockFinalizationRegistry.registeredPairs, hasLength(0));
      final TestSkDeletable nativeObject = TestSkDeletable();
      final TestCountedRefOwner owner = TestCountedRefOwner(nativeObject);
      expect(owner.ref.debugReferrers, hasLength(1));
      expect(owner.ref.debugReferrers.single, owner);
      expect(owner.ref.refCount, 1);
      expect(owner.ref.nativeObject, nativeObject);
      expect(TestSkDeletableMock.deleteCount, 0);
      expect(mockFinalizationRegistry.registeredPairs, hasLength(1));

      owner.dispose();
      expect(owner.ref.debugReferrers, isEmpty);
      expect(owner.ref.refCount, 0);
      expect(
        reason: 'Cannot access object that was disposed',
        () => owner.ref.nativeObject,
        throwsA(isA<AssertionError>()),
      );
      expect(TestSkDeletableMock.deleteCount, 1);

      expect(
        reason: 'Cannot dispose object more than once',
        () => owner.dispose(),
        throwsA(isA<AssertionError>()),
      );
    });

    test('multiple owners', () {
      expect(mockFinalizationRegistry.registeredPairs, hasLength(0));
      final TestSkDeletable nativeObject = TestSkDeletable();
      final TestCountedRefOwner owner1 = TestCountedRefOwner(nativeObject);
      expect(owner1.ref.debugReferrers, hasLength(1));
      expect(owner1.ref.debugReferrers.single, owner1);
      expect(owner1.ref.refCount, 1);
      expect(owner1.ref.nativeObject, nativeObject);
      expect(TestSkDeletableMock.deleteCount, 0);
      expect(mockFinalizationRegistry.registeredPairs, hasLength(1));

      final TestCountedRefOwner owner2 = owner1.clone();
      expect(owner2.ref, same(owner1.ref));
      expect(owner2.ref.debugReferrers, hasLength(2));
      expect(owner2.ref.debugReferrers.first, owner1);
      expect(owner2.ref.debugReferrers.last, owner2);
      expect(owner2.ref.refCount, 2);
      expect(owner2.ref.nativeObject, nativeObject);
      expect(TestSkDeletableMock.deleteCount, 0);
      expect(
        reason:
            'Second owner does not add more native object owners. '
            'The underlying shared UniqueRef is the only one.',
        mockFinalizationRegistry.registeredPairs,
        hasLength(1),
      );

      owner1.dispose();
      expect(owner2.ref.debugReferrers, hasLength(1));
      expect(owner2.ref.debugReferrers.single, owner2);
      expect(owner2.ref.refCount, 1);
      expect(owner2.ref.nativeObject, nativeObject);
      expect(TestSkDeletableMock.deleteCount, 0);
      expect(
        reason:
            'The same owner cannot dispose its CountedRef more than once, even when CountedRef is still alive.',
        () => owner1.dispose(),
        throwsA(isA<AssertionError>()),
      );

      owner2.dispose();
      expect(owner2.ref.debugReferrers, isEmpty);
      expect(owner2.ref.refCount, 0);
      expect(
        reason: 'Cannot access object that was disposed',
        () => owner2.ref.nativeObject,
        throwsA(isA<AssertionError>()),
      );
      expect(TestSkDeletableMock.deleteCount, 1);

      expect(
        reason: 'The same owner cannot dispose its CountedRef more than once.',
        () => owner2.dispose(),
        throwsA(isA<AssertionError>()),
      );

      // Simulate a GC
      mockFinalizationRegistry.registeredPairs.single.ref.collect();
      expect(
        reason: 'Manually disposed object should not be deleted again by GC.',
        TestSkDeletableMock.deleteCount,
        1,
      );
    });
  });
}

class TestSkDeletableMock {
  static int deleteCount = 0;

  bool isDeleted() => _isDeleted;
  bool _isDeleted = false;

  void delete() {
    expect(
      _isDeleted,
      isFalse,
      reason: 'CanvasKit does not allow deleting the same object more than once.',
    );
    _isDeleted = true;
    deleteCount++;
  }

  JsConstructor get constructor => TestJsConstructor(name: 'TestSkDeletable');
}

@JS()
@anonymous
@staticInterop
class TestSkDeletable implements SkDeletable {
  factory TestSkDeletable() {
    final TestSkDeletableMock mock = TestSkDeletableMock();
    return TestSkDeletable._(
      isDeleted:
          () {
            return mock.isDeleted();
          }.toJS,
      delete:
          () {
            return mock.delete();
          }.toJS,
      constructor: mock.constructor,
    );
  }

  external factory TestSkDeletable._({
    JSFunction isDeleted,
    JSFunction delete,
    JsConstructor constructor,
  });
}

@JS()
@anonymous
@staticInterop
class TestJsConstructor implements JsConstructor {
  external factory TestJsConstructor({String name});
}

class TestCountedRefOwner implements StackTraceDebugger {
  TestCountedRefOwner(TestSkDeletable nativeObject) {
    assert(() {
      _debugStackTrace = StackTrace.current;
      return true;
    }());
    ref = CountedRef<TestCountedRefOwner, TestSkDeletable>(
      nativeObject,
      this,
      'TestCountedRefOwner',
    );
  }

  TestCountedRefOwner.cloneOf(this.ref) {
    assert(() {
      _debugStackTrace = StackTrace.current;
      return true;
    }());
    ref.ref(this);
  }

  @override
  StackTrace get debugStackTrace => _debugStackTrace;
  late StackTrace _debugStackTrace;

  late final CountedRef<TestCountedRefOwner, TestSkDeletable> ref;

  void dispose() {
    ref.unref(this);
  }

  TestCountedRefOwner clone() => TestCountedRefOwner.cloneOf(ref);
}

class _MockNativeMemoryFinalizationRegistry implements NativeMemoryFinalizationRegistry {
  final List<_MockPair> registeredPairs = <_MockPair>[];

  @override
  void register(Object owner, UniqueRef<Object> ref) {
    registeredPairs.add(_MockPair(owner, ref));
  }
}

class _MockPair {
  _MockPair(this.owner, this.ref);

  Object owner;
  UniqueRef<Object> ref;
}
