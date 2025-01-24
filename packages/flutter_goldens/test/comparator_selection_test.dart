// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_goldens/flutter_goldens.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:platform/platform.dart';

enum _Comparator { post, pre, skip, local }

_Comparator _testRecommendations({
  bool hasFlutterRoot = false,
  bool hasLuci = false,
  bool hasGold = false,
  bool hasTryJob = false,
  String branch = 'main',
  String os = 'macos',
}) {
  final Platform platform = FakePlatform(
    environment: <String, String>{
      if (hasFlutterRoot) 'FLUTTER_ROOT': '/flutter',
      if (hasLuci) 'SWARMING_TASK_ID': '8675309',
      if (hasGold) 'GOLDCTL': 'goldctl',
      if (hasTryJob) 'GOLD_TRYJOB': 'git/ref/12345/head',
      'GIT_BRANCH': branch,
    },
    operatingSystem: os,
  );
  if (FlutterPostSubmitFileComparator.isForEnvironment(platform)) {
    return _Comparator.post;
  }
  if (FlutterPreSubmitFileComparator.isForEnvironment(platform)) {
    return _Comparator.pre;
  }
  if (FlutterSkippingFileComparator.isForEnvironment(platform)) {
    return _Comparator.skip;
  }
  return _Comparator.local;
}

void main() {
  test('Comparator recommendations - main branch', () {
    // If we're running locally (no CI), use a local comparator.
    expect(_testRecommendations(), _Comparator.local);
    expect(_testRecommendations(hasFlutterRoot: true), _Comparator.local);
    expect(_testRecommendations(hasGold: true), _Comparator.local);
    expect(_testRecommendations(hasFlutterRoot: true, hasGold: true), _Comparator.local);

    // If we don't have gold but are on CI, we skip regardless.
    expect(_testRecommendations(hasLuci: true), _Comparator.skip);
    expect(_testRecommendations(hasLuci: true, hasTryJob: true), _Comparator.skip);
    expect(_testRecommendations(hasFlutterRoot: true, hasLuci: true), _Comparator.skip);
    expect(
      _testRecommendations(hasFlutterRoot: true, hasLuci: true, hasTryJob: true),
      _Comparator.skip,
    );

    // On Luci, with Gold, post-submit. Flutter root and LUCI variables should have no effect.
    expect(_testRecommendations(hasGold: true, hasLuci: true), _Comparator.post);
    expect(
      _testRecommendations(hasGold: true, hasLuci: true, hasFlutterRoot: true),
      _Comparator.post,
    );

    // On Luci, with Gold, pre-submit. Flutter root and LUCI variables should have no effect.
    expect(_testRecommendations(hasGold: true, hasLuci: true, hasTryJob: true), _Comparator.pre);
    expect(
      _testRecommendations(hasGold: true, hasLuci: true, hasFlutterRoot: true, hasTryJob: true),
      _Comparator.pre,
    );
  });

  test('Comparator recommendations - release branch', () {
    // If we're running locally (no CI), use a local comparator.
    expect(_testRecommendations(branch: 'flutter-3.16-candidate.0'), _Comparator.local);
    expect(
      _testRecommendations(branch: 'flutter-3.16-candidate.0', hasFlutterRoot: true),
      _Comparator.local,
    );
    expect(
      _testRecommendations(branch: 'flutter-3.16-candidate.0', hasGold: true),
      _Comparator.local,
    );
    expect(
      _testRecommendations(branch: 'flutter-3.16-candidate.0', hasFlutterRoot: true, hasGold: true),
      _Comparator.local,
    );

    // If we don't have gold but are on CI, we skip regardless.
    expect(
      _testRecommendations(branch: 'flutter-3.16-candidate.0', hasLuci: true),
      _Comparator.skip,
    );
    expect(
      _testRecommendations(branch: 'flutter-3.16-candidate.0', hasLuci: true, hasTryJob: true),
      _Comparator.skip,
    );
    expect(
      _testRecommendations(branch: 'flutter-3.16-candidate.0', hasFlutterRoot: true, hasLuci: true),
      _Comparator.skip,
    );
    expect(
      _testRecommendations(
        branch: 'flutter-3.16-candidate.0',
        hasFlutterRoot: true,
        hasLuci: true,
        hasTryJob: true,
      ),
      _Comparator.skip,
    );

    // On Luci, with Gold, post-submit. Flutter root and LUCI variables should have no effect. Branch should make us skip.
    expect(
      _testRecommendations(branch: 'flutter-3.16-candidate.0', hasGold: true, hasLuci: true),
      _Comparator.skip,
    );
    expect(
      _testRecommendations(
        branch: 'flutter-3.16-candidate.0',
        hasGold: true,
        hasLuci: true,
        hasFlutterRoot: true,
      ),
      _Comparator.skip,
    );

    // On Luci, with Gold, pre-submit. Flutter root and LUCI variables should have no effect. Branch should make us skip.
    expect(
      _testRecommendations(
        branch: 'flutter-3.16-candidate.0',
        hasGold: true,
        hasLuci: true,
        hasTryJob: true,
      ),
      _Comparator.skip,
    );
    expect(
      _testRecommendations(
        branch: 'flutter-3.16-candidate.0',
        hasGold: true,
        hasLuci: true,
        hasFlutterRoot: true,
        hasTryJob: true,
      ),
      _Comparator.skip,
    );
  });

  test('Comparator recommendations - Linux', () {
    // If we're running locally (no CI), use a local comparator.
    expect(_testRecommendations(os: 'linux'), _Comparator.local);
    expect(_testRecommendations(os: 'linux', hasFlutterRoot: true), _Comparator.local);
    expect(_testRecommendations(os: 'linux', hasGold: true), _Comparator.local);
    expect(
      _testRecommendations(os: 'linux', hasFlutterRoot: true, hasGold: true),
      _Comparator.local,
    );

    // If we don't have gold but are on CI, we skip regardless.
    expect(_testRecommendations(os: 'linux', hasLuci: true), _Comparator.skip);
    expect(_testRecommendations(os: 'linux', hasLuci: true, hasTryJob: true), _Comparator.skip);
    expect(
      _testRecommendations(os: 'linux', hasFlutterRoot: true, hasLuci: true),
      _Comparator.skip,
    );
    expect(
      _testRecommendations(os: 'linux', hasFlutterRoot: true, hasLuci: true, hasTryJob: true),
      _Comparator.skip,
    );

    // On Luci, with Gold, post-submit. Flutter root has no effect.
    expect(_testRecommendations(os: 'linux', hasGold: true, hasLuci: true), _Comparator.post);
    expect(
      _testRecommendations(os: 'linux', hasGold: true, hasLuci: true, hasFlutterRoot: true),
      _Comparator.post,
    );

    // On Luci, with Gold, pre-submit. Flutter root should have no effect.
    expect(
      _testRecommendations(os: 'linux', hasGold: true, hasLuci: true, hasTryJob: true),
      _Comparator.pre,
    );
    expect(
      _testRecommendations(
        os: 'linux',
        hasGold: true,
        hasLuci: true,
        hasFlutterRoot: true,
        hasTryJob: true,
      ),
      _Comparator.pre,
    );
  });

  test('Branch names', () {
    expect(
      _testRecommendations(hasLuci: true, hasGold: true, hasFlutterRoot: true),
      _Comparator.post,
    );
    expect(
      _testRecommendations(branch: 'master', hasLuci: true, hasGold: true, hasFlutterRoot: true),
      _Comparator.post,
    );
    expect(
      _testRecommendations(
        branch: 'the_master_of_justice',
        hasLuci: true,
        hasGold: true,
        hasFlutterRoot: true,
      ),
      _Comparator.skip,
    );
    expect(
      _testRecommendations(
        branch: 'maintain_accuracy',
        hasLuci: true,
        hasGold: true,
        hasFlutterRoot: true,
      ),
      _Comparator.skip,
    );
  });
}
