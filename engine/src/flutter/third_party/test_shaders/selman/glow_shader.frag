// Directional glow GLSL fragment shader.
// Based on the "Let it Glow!" pen by "Selman Ay"
// (https://codepen.io/selmanays/pen/yLVmEqY)
precision highp float;

// Float uniforms
uniform float width;    // Width of the canvas
uniform float height;   // Height of the canvas
uniform float sourceX;  // X position of the light source
uniform float
    scrollFraction;     // Scroll fraction of the page in relation to the canvas
uniform float density;  // Density of the "smoke" effect
uniform float lightStrength;  // Strength of the light
uniform float weight;         // Weight of the "smoke" effect

// Sampler uniforms
uniform sampler2D tInput;  // Input texture (the application canvas)
uniform sampler2D tNoise;  // Some texture

out vec4 fragColor;

float sourceY = scrollFraction;
vec2 resolution = vec2(width, height);
vec2 lightSource = vec2(sourceX, sourceY);

const int samples = 20;  // The number of "copies" of the canvas made to emulate
                         // the "smoke" effect
const float decay = 0.88;   // Decay of the light in each sample
const float exposure = .9;  // The exposure to the light

float random2d(vec2 uv) {
  uv /= 256.;
  vec4 tex = texture(tNoise, uv);
  return mix(tex.r, tex.g, tex.a);
}

float random(vec3 xyz) {
  return fract(sin(dot(xyz, vec3(12.9898, 78.233, 151.7182))) * 43758.5453);
}

vec4 sampleTexture(vec2 uv) {
  vec4 textColor = texture(tInput, uv);
  return textColor;
}

vec4 occlusion(vec2 uv, vec2 lightpos, vec4 objects) {
  return (1. - smoothstep(0.0, lightStrength, length(lightpos - uv))) *
         (objects);
}

vec4 fragment(vec2 uv, vec2 fragCoord) {
  vec3 colour = vec3(0);

  vec4 obj = sampleTexture(uv);
  vec4 map = occlusion(uv, lightSource, obj);

  float random = random(vec3(fragCoord, 1.0));
  ;

  float exposure = exposure + (sin(random) * .5 + 1.) * .05;

  vec2 _uv = uv;
  vec2 distance = (_uv - lightSource) * (1. / float(samples) * density);

  float illumination_decay = 1.;
  for (int i = 0; i < samples; i++) {
    _uv -= distance;

    float movement = random * 20. * float(i + 1);
    float dither =
        random2d(uv +
                 mod(vec2(movement * sin(random * .5), -movement), 1000.)) *
        2.;

    vec4 stepped_map =
        occlusion(uv, lightSource, sampleTexture(_uv + distance * dither));
    stepped_map *= illumination_decay * weight;

    illumination_decay *= decay;
    map += stepped_map;
  }

  float lum = dot(map.rgb, vec3(0.2126, 0.7152, 0.0722));

  colour += vec3(map.rgb * exposure);
  return vec4(colour, lum);
}

void main() {
  vec2 pos = gl_FragCoord.xy;
  vec2 uv = pos / vec2(width, height);
  fragColor = fragment(uv, pos);
}
