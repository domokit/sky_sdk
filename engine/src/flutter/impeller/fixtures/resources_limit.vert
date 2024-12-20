// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform sampler1D tex;
void main() {
  vec4 x = textureOffset(tex, 1.0, -10);
}
