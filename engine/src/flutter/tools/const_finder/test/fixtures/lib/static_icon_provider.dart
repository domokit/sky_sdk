// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'target.dart';

void main() {
  Targets.used1.hit();
  Targets.used2.hit();
  final Target nonConstUsed3 = helper(Target.new);
  nonConstUsed3.hit();
}

Target helper(Target Function(String, int, Target?) tearOff) {
  return tearOff('from tear-off', 3, null);
}

@staticIconProvider
class Targets {
  static const Target used1 = Target('used1', 1, null);
  static const Target used2 = Target('used2', 2, null);
  static const Target unused1 = Target('unused1', 1, null); // ignore: unreachable_from_main
}

// const_finder explicitly does not retain constants appearing within a class
// with this annotation.
class StaticIconProvider {
  const StaticIconProvider();
}

const StaticIconProvider staticIconProvider = StaticIconProvider();
