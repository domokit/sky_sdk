// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

class Target {
  const Target(this.stringValue, this.intValue, this.targetValue);

  final String stringValue;
  final int intValue;
  final Target? targetValue;

  void hit() {
    print('$stringValue $intValue');
  }
}

class ExtendsTarget extends Target {
  const ExtendsTarget(super.stringValue, super.intValue, super.targetValue);
}

class ImplementsTarget implements Target {
  const ImplementsTarget(this.stringValue, this.intValue, this.targetValue);

  @override
  final String stringValue;
  @override
  final int intValue;
  @override
  final Target? targetValue;

  @override
  void hit() {
    print('ImplementsTarget - $stringValue $intValue');
  }
}

mixin MixableTarget {
  String get val;

  void hit() {
    print(val);
  }
}

class MixedInTarget with MixableTarget {
  const MixedInTarget(this.val);

  @override
  final String val;
}
