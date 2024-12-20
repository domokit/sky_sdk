// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:ui';

void main() {
  final Paint paint = Paint()..color = Color(0xFFFFFFFF);
  print(jsonEncode(<String, String>{
    'Paint.toString': paint.toString(),
    'Brightness.toString': Brightness.dark.toString(),
    'Foo.toString': Foo().toString(),
    'Keep.toString': Keep().toString(),
  }));
}

class Foo {
  @override
  String toString() => 'I am a Foo';
}

class Keep {
  @keepToString
  @override
  String toString() => 'I am a Keep';
}
