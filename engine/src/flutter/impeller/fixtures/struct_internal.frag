// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

out vec4 frag_color;

struct TestStruct {
  float a;
  float b;
  float c;
};

void main() {
  TestStruct ts = TestStruct(1.0, 0.5, 0.25);
  frag_color = vec4(ts.c, ts.b, ts.a, 1.0);
}
