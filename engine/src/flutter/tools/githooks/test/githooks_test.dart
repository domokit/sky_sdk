// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:githooks/githooks.dart';
import 'package:test/test.dart';

void main() {
  test('Fails gracefully without a command', () async {
    int? result;
    try {
      result = await run(<String>[]);
    } catch (e, st) {
      fail('Unexpected exception: $e\n$st');
    }
    expect(result, equals(1));
  });

  test('Fails gracefully with an unknown command', () async {
    int? result;
    try {
      result = await run(<String>['blah']);
    } catch (e, st) {
      fail('Unexpected exception: $e\n$st');
    }
    expect(result, equals(1));
  });

  test('Fails gracefully without --flutter', () async {
    int? result;
    try {
      result = await run(<String>['pre-push']);
    } catch (e, st) {
      fail('Unexpected exception: $e\n$st');
    }
    expect(result, equals(1));
  });

  test('Fails gracefully when --flutter is not an absolute path', () async {
    int? result;
    try {
      result = await run(<String>['pre-push', '--flutter', 'non/absolute']);
    } catch (e, st) {
      fail('Unexpected exception: $e\n$st');
    }
    expect(result, equals(1));
  });

  test('Fails gracefully when --flutter does not exist', () async {
    int? result;
    try {
      result = await run(<String>[
        'pre-push',
        '--flutter',
        if (io.Platform.isWindows) r'C:\does\not\exist' else '/does/not/exist',
      ]);
    } catch (e, st) {
      fail('Unexpected exception: $e\n$st');
    }
    expect(result, equals(1));
  });

  test('post-merge runs successfully', () async {
    int? result;
    try {
      final io.Directory flutterPath = io.File(io.Platform.script.path).parent.parent.parent;
      result = await run(<String>['post-merge', '--flutter', flutterPath.path]);
    } catch (e, st) {
      fail('Unexpected exception: $e\n$st');
    }
    expect(result, equals(0));
  });

  test('pre-rebase runs successfully', () async {
    int? result;
    try {
      final io.Directory flutterPath = io.File(io.Platform.script.path).parent.parent.parent;
      result = await run(<String>['pre-rebase', '--flutter', flutterPath.path]);
    } catch (e, st) {
      fail('Unexpected exception: $e\n$st');
    }
    expect(result, equals(0));
  });

  test('post-checkout runs successfully', () async {
    int? result;
    try {
      final io.Directory flutterPath = io.File(io.Platform.script.path).parent.parent.parent;
      result = await run(<String>['post-checkout', '--flutter', flutterPath.path]);
    } catch (e, st) {
      fail('Unexpected exception: $e\n$st');
    }
    expect(result, equals(0));
  });
}
