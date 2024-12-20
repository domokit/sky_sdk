// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:engine_build_configs/src/merge_gn_args.dart';
import 'package:test/test.dart';

void main() {
  test('refuses to merge with arguments that do not start with --', () {
    expect(() => mergeGnArgs(buildArgs: [], extraArgs: ['foo']), throwsArgumentError);
  });

  test('refuses to merge with arguments that contain spaces', () {
    expect(() => mergeGnArgs(buildArgs: [], extraArgs: ['--foo bar']), throwsArgumentError);
  });

  test('refuses to merge with arguments that contain equals', () {
    expect(() => mergeGnArgs(buildArgs: [], extraArgs: ['--foo=bar']), throwsArgumentError);
  });

  test('appends if there are no matching arguments', () {
    expect(mergeGnArgs(buildArgs: ['--foo'], extraArgs: ['--bar']), ['--foo', '--bar']);
  });

  test('replaces --foo with --no-foo', () {
    expect(mergeGnArgs(buildArgs: ['--foo'], extraArgs: ['--no-foo']), ['--no-foo']);
  });

  test('replaces --no-foo with --foo', () {
    expect(mergeGnArgs(buildArgs: ['--no-foo'], extraArgs: ['--foo']), ['--foo']);
  });
}
