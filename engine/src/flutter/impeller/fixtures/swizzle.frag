// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

out vec4 frag_color;

layout(input_attachment_index = 0) uniform subpassInputMS subpass_input;

void main() {
  frag_color = subpassLoad(subpass_input, 0).gbra;
}
