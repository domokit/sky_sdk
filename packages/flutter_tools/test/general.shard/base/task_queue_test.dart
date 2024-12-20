// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/base/task_queue.dart';

import '../../src/common.dart';

void main() {
  group('TaskQueue', () {
    /// A special test designed to check shared [TaskQueue]
    /// behavior when exceptions occur after a delay in the passed closures to
    /// [TaskQueue.add].
    test('no deadlock when delayed exceptions fire in closures', () async {
      final TaskQueue<void> sharedTracker = TaskQueue<void>(maxJobs: 2);
      expect(() async {
        final Future<void> t = Future<void>.delayed(
          const Duration(milliseconds: 10),
          () => throw TestException(),
        );
        await sharedTracker.add(() => t);
        return t;
      }, throwsA(const TypeMatcher<TestException>()));
      expect(() async {
        final Future<void> t = Future<void>.delayed(
          const Duration(milliseconds: 10),
          () => throw TestException(),
        );
        await sharedTracker.add(() => t);
        return t;
      }, throwsA(const TypeMatcher<TestException>()));
      expect(() async {
        final Future<void> t = Future<void>.delayed(
          const Duration(milliseconds: 10),
          () => throw TestException(),
        );
        await sharedTracker.add(() => t);
        return t;
      }, throwsA(const TypeMatcher<TestException>()));
      expect(() async {
        final Future<void> t = Future<void>.delayed(
          const Duration(milliseconds: 10),
          () => throw TestException(),
        );
        await sharedTracker.add(() => t);
        return t;
      }, throwsA(const TypeMatcher<TestException>()));

      /// We deadlock here if the exception is not handled properly.
      await sharedTracker.tasksComplete;
    });

    test('basic sequential processing works with no deadlock', () async {
      final Set<int> completed = <int>{};
      final TaskQueue<void> tracker = TaskQueue<void>(maxJobs: 1);
      await tracker.add(() async => completed.add(1));
      await tracker.add(() async => completed.add(2));
      await tracker.add(() async => completed.add(3));
      await tracker.tasksComplete;
      expect(completed.length, equals(3));
    });

    test('basic sequential processing works on exceptions', () async {
      final Set<int> completed = <int>{};
      final TaskQueue<void> tracker = TaskQueue<void>(maxJobs: 1);
      await tracker.add(() async => completed.add(0));
      await tracker.add(() async => throw TestException()).then((_) {}, onError: (Object _) {});
      await tracker.add(() async => throw TestException()).then((_) {}, onError: (Object _) {});
      await tracker.add(() async => completed.add(3));
      await tracker.tasksComplete;
      expect(completed.length, equals(2));
    });

    /// Verify that if there are more exceptions than the maximum number
    /// of in-flight [Future]s that there is no deadlock.
    test('basic parallel processing works with no deadlock', () async {
      final Set<int> completed = <int>{};
      final TaskQueue<void> tracker = TaskQueue<void>(maxJobs: 10);
      for (int i = 0; i < 100; i++) {
        await tracker.add(() async => completed.add(i));
      }
      await tracker.tasksComplete;
      expect(completed.length, equals(100));
    });

    test('basic parallel processing works on exceptions', () async {
      final Set<int> completed = <int>{};
      final TaskQueue<void> tracker = TaskQueue<void>(maxJobs: 10);
      for (int i = 0; i < 50; i++) {
        await tracker.add(() async => completed.add(i));
      }
      for (int i = 50; i < 65; i++) {
        try {
          await tracker.add(() async => throw TestException());
        } on TestException {
          // Ignore
        }
      }
      for (int i = 65; i < 100; i++) {
        await tracker.add(() async => completed.add(i));
      }
      await tracker.tasksComplete;
      expect(completed.length, equals(85));
    });
  });
}

class TestException implements Exception {}
