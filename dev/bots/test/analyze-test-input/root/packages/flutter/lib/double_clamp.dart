// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

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
  input.clamp(0.0, 2); // ERROR: input.clamp(0.0, 2)
  input.toDouble().clamp(0, 2); // ERROR: input.toDouble().clamp(0, 2)

  nonDoubleClamp2?.clamp(0, 2);
  nullableInt?.clamp(0, 2);
  nullableInt?.clamp(0, 2.0); // ERROR: nullableInt?.clamp(0, 2.0)
  nullableDouble?.clamp(0, 2); // ERROR: nullableDouble?.clamp(0, 2)

  // ignore: unused_local_variable
  final ClassWithAClampMethod Function(double, double)? tearOff1 = nonDoubleClamp2?.clamp;
  // ignore: unused_local_variable
  final num Function(num, num)? tearOff2 = nullableInt?.clamp; // ERROR: nullableInt?.clamp
  // ignore: unused_local_variable
  final num Function(num, num)? tearOff3 = nullableDouble?.clamp; // ERROR: nullableDouble?.clamp
}
