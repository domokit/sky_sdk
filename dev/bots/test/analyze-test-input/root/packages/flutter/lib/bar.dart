// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

class _DebugOnly {
  const _DebugOnly();
}

const _DebugOnly _debugOnly = _DebugOnly();
const bool kDebugMode = bool.fromEnvironment('test-only');

class Foo {
  @_debugOnly
  final Map<String, String>? foo = kDebugMode ? <String, String>{} : null;

  @_debugOnly
  final Map<String, String>? bar = kDebugMode ? null : <String, String>{};
}

/// Simply avoid this
/// and simply do that.

class ClassWithAClampMethod {
  ClassWithAClampMethod clamp(double min, double max) => this;
}

void testNoDoubleClamp(int input) {
  final ClassWithAClampMethod nonDoubleClamp = ClassWithAClampMethod();
  // ignore: unnecessary_nullable_for_final_variable_declarations
  final ClassWithAClampMethod? nonDoubleClamp2 = nonDoubleClamp;
  // ignore: unnecessary_nullable_for_final_variable_declarations
  final int? nullableInt = input;
  final double? nullableDouble = nullableInt?.toDouble();

  nonDoubleClamp.clamp(0, 2);
  input.clamp(0, 2);
  input.clamp(0.0, 2); // bad.
  input.toDouble().clamp(0, 2); // bad.

  nonDoubleClamp2?.clamp(0, 2);
  nullableInt?.clamp(0, 2);
  nullableInt?.clamp(0, 2.0); // bad
  nullableDouble?.clamp(0, 2); // bad.

  // ignore: unused_local_variable
  final ClassWithAClampMethod Function(double, double)? tearOff1 = nonDoubleClamp2?.clamp;
  // ignore: unused_local_variable
  final num Function(num, num)? tearOff2 = nullableInt?.clamp; // bad.
  // ignore: unused_local_variable
  final num Function(num, num)? tearOff3 = nullableDouble?.clamp; // bad.
}
