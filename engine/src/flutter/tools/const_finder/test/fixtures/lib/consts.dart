// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: prefer_const_constructors, unused_local_variable, depend_on_referenced_packages
import 'dart:core';

import 'package:const_finder_fixtures_package/package.dart';

import 'target.dart';

void main() {
  const Target target1 = Target('1', 1, null);
  const Target target2 = Target('2', 2, Target('4', 4, null));
  const Target target3 = Target('3', 3, Target('5', 5, null)); // should be tree shaken out.
  target1.hit();
  target2.hit();

  blah(const Target('6', 6, null));

  const IgnoreMe ignoreMe = IgnoreMe(Target('7', 7, null)); // IgnoreMe is ignored but 7 is not.
  final IgnoreMe ignoreMe2 = IgnoreMe(const Target('8', 8, null));
  final IgnoreMe ignoreMe3 = IgnoreMe(const Target('9', 9, Target('10', 10, null)));
  print(ignoreMe);
  print(ignoreMe2);
  print(ignoreMe3);

  createTargetInPackage();

  final StaticConstInitializer staticConstMap = StaticConstInitializer();
  staticConstMap.useOne(1);

  const ExtendsTarget extendsTarget = ExtendsTarget('11', 11, null);
  extendsTarget.hit();
  const ImplementsTarget implementsTarget = ImplementsTarget('12', 12, null);
  implementsTarget.hit();

  const MixedInTarget mixedInTraget = MixedInTarget('13');
  mixedInTraget.hit();
}

class IgnoreMe {
  const IgnoreMe(this.target);

  final Target target;

  @override
  String toString() => target.toString();
}

class StaticConstInitializer {
  static const List<Target> targets = <Target>[
    Target('100', 100, null),
    Target('101', 101, Target('102', 102, null)),
  ];

  static const Set<Target> targetSet = <Target>{
    Target('103', 103, null),
    Target('104', 104, Target('105', 105, null)),
  };

  static const Map<int, Target> targetMap = <int, Target>{
    0: Target('106', 106, null),
    1: Target('107', 107, Target('108', 108, null)),
  };

  void useOne(int index) {
    targets[index].hit();
    targetSet.skip(index).first.hit();
    targetMap[index]!.hit();
  }
}

void blah(Target target) {
  print(target);
}
