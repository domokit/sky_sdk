// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

struct VertexInput {
  float3 position : POSITION;
};

struct VertexOutput {
  float4 position : SV_POSITION;
};

VertexOutput VertexShader(VertexInput input) {
  VertexOutput output;
  output.position = float4(input.position, 1.0);
  return output;
}

float4 FragmentShader(VertexOutput input) {
  return input.position;
}
