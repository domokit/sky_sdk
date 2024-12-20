#include <flutter/runtime_effect.glsl>

uniform vec2 uSize;

out vec4 fragColor;

void main() {
  float v = FlutterFragCoord().y / uSize.y;
  fragColor = vec4(v, v, v, 1);
}
