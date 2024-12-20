// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

out vec4 frag_color;

layout(input_attachment_index = 0) uniform subpassInputMS subpass_input;

void main() {
  // https://github.com/chinmaygarde/merle/blob/3eecb311ac8862c41f0c53a5d9b360be923142bb/src/texture.cc#L195
  const mat4 sepia_matrix = mat4(0.3588, 0.2990, 0.2392, 0.0000,  //
                                 0.7044, 0.5870, 0.4696, 0.0000,  //
                                 0.1368, 0.1140, 0.0912, 0.0000,  //
                                 0.0000, 0.0000, 0.0000, 1.0000   //
  );
  frag_color = sepia_matrix * subpassLoad(subpass_input, 0);
}
