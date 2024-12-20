uniform FragInfo {
  vec2 resolution;
  float time;
  float speed;
  float planet_size;
  float show_normals;
  float show_noise;
  float seed_value;
}
frag_info;

in vec2 v_screen_position;
out vec4 frag_color;

float inverseLerp(float v, float min_value, float max_value) {
  return (v - min_value) / (max_value - min_value);
}

float remap(float v, float in_min, float in_max, float out_min, float out_max) {
  float t = inverseLerp(v, in_min, in_max);
  return mix(out_min, out_max, t);
}

float saturate(float x) {
  return clamp(x, 0.0, 1.0);
}

// Copyright (C) 2011 by Ashima Arts (Simplex noise)
// Copyright (C) 2011-2016 by Stefan Gustavson (Classic noise and others)
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or
// substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
// WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
// TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
// FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
// THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// https://github.com/ashima/webgl-noise/tree/master/src
vec3 mod289(vec3 x) {
  return x - floor(x / 289.0) * 289.0;
}

vec4 mod289(vec4 x) {
  return x - floor(x / 289.0) * 289.0;
}

vec4 permute(vec4 x) {
  return mod289((x * 34.0 + 1.0) * x);
}

vec4 taylorInvSqrt(vec4 r) {
  return 1.79284291400159 - r * 0.85373472095314;
}

vec4 snoise(vec3 v) {
  const vec2 C = vec2(1.0 / 6.0, 1.0 / 3.0);

  // First corner
  vec3 i = floor(v + dot(v, vec3(C.y)));
  vec3 x0 = v - i + dot(i, vec3(C.x));

  // Other corners
  vec3 g = step(x0.yzx, x0.xyz);
  vec3 l = 1.0 - g;
  vec3 i1 = min(g.xyz, l.zxy);
  vec3 i2 = max(g.xyz, l.zxy);

  vec3 x1 = x0 - i1 + C.x;
  vec3 x2 = x0 - i2 + C.y;
  vec3 x3 = x0 - 0.5;

  // Permutations
  i = mod289(i);  // Avoid truncation effects in permutation
  vec4 p = permute(permute(permute(i.z + vec4(0.0, i1.z, i2.z, 1.0)) + i.y +
                           vec4(0.0, i1.y, i2.y, 1.0)) +
                   i.x + vec4(0.0, i1.x, i2.x, 1.0));

  // Gradients: 7x7 points over a square, mapped onto an octahedron.
  // The ring size 17*17 = 289 is close to a multiple of 49 (49*6 = 294)
  vec4 j = p - 49.0 * floor(p / 49.0);  // mod(p,7*7)

  vec4 x_ = floor(j / 7.0);
  vec4 y_ = floor(j - 7.0 * x_);

  vec4 x = (x_ * 2.0 + 0.5) / 7.0 - 1.0;
  vec4 y = (y_ * 2.0 + 0.5) / 7.0 - 1.0;

  vec4 h = 1.0 - abs(x) - abs(y);

  vec4 b0 = vec4(x.xy, y.xy);
  vec4 b1 = vec4(x.zw, y.zw);

  vec4 s0 = floor(b0) * 2.0 + 1.0;
  vec4 s1 = floor(b1) * 2.0 + 1.0;
  vec4 sh = -step(h, vec4(0.0));

  vec4 a0 = b0.xzyw + s0.xzyw * sh.xxyy;
  vec4 a1 = b1.xzyw + s1.xzyw * sh.zzww;

  vec3 g0 = vec3(a0.xy, h.x);
  vec3 g1 = vec3(a0.zw, h.y);
  vec3 g2 = vec3(a1.xy, h.z);
  vec3 g3 = vec3(a1.zw, h.w);

  // Normalize gradients
  vec4 norm =
      taylorInvSqrt(vec4(dot(g0, g0), dot(g1, g1), dot(g2, g2), dot(g3, g3)));
  g0 *= norm.x;
  g1 *= norm.y;
  g2 *= norm.z;
  g3 *= norm.w;

  // Compute noise and gradient at P
  vec4 m =
      max(0.6 - vec4(dot(x0, x0), dot(x1, x1), dot(x2, x2), dot(x3, x3)), 0.0);
  vec4 m2 = m * m;
  vec4 m3 = m2 * m;
  vec4 m4 = m2 * m2;
  vec3 grad = -6.0 * m3.x * x0 * dot(x0, g0) + m4.x * g0 +
              -6.0 * m3.y * x1 * dot(x1, g1) + m4.y * g1 +
              -6.0 * m3.z * x2 * dot(x2, g2) + m4.z * g2 +
              -6.0 * m3.w * x3 * dot(x3, g3) + m4.w * g3;
  vec4 px = vec4(dot(x0, g0), dot(x1, g1), dot(x2, g2), dot(x3, g3));
  return 42.0 * vec4(grad, dot(m4, px));
}

// The MIT License
// Copyright © 2013 Inigo Quilez
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or
// substantial portions of the Software. THE SOFTWARE IS PROVIDED "AS IS",
// WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
// TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
// FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
// THE USE OR OTHER DEALINGS IN THE SOFTWARE.
// https://www.youtube.com/c/InigoQuilez
// https://iquilezles.org/
//
// https://www.shadertoy.com/view/Xsl3Dl
vec3 hash3(vec3 p)  // replace this by something better
{
  p = vec3(dot(p, vec3(127.1, 311.7, 74.7)), dot(p, vec3(269.5, 183.3, 246.1)),
           dot(p, vec3(113.5, 271.9, 124.6)));

  return -1.0 + 2.0 * fract(sin(p) * 43758.5453123);
}

float noise(in vec3 p) {
  vec3 i = floor(p);
  vec3 f = fract(p);

  vec3 u = f * f * (3.0 - 2.0 * f);

  return mix(
      mix(mix(dot(hash3(i + vec3(0.0, 0.0, 0.0)), f - vec3(0.0, 0.0, 0.0)),
              dot(hash3(i + vec3(1.0, 0.0, 0.0)), f - vec3(1.0, 0.0, 0.0)),
              u.x),
          mix(dot(hash3(i + vec3(0.0, 1.0, 0.0)), f - vec3(0.0, 1.0, 0.0)),
              dot(hash3(i + vec3(1.0, 1.0, 0.0)), f - vec3(1.0, 1.0, 0.0)),
              u.x),
          u.y),
      mix(mix(dot(hash3(i + vec3(0.0, 0.0, 1.0)), f - vec3(0.0, 0.0, 1.0)),
              dot(hash3(i + vec3(1.0, 0.0, 1.0)), f - vec3(1.0, 0.0, 1.0)),
              u.x),
          mix(dot(hash3(i + vec3(0.0, 1.0, 1.0)), f - vec3(0.0, 1.0, 1.0)),
              dot(hash3(i + vec3(1.0, 1.0, 1.0)), f - vec3(1.0, 1.0, 1.0)),
              u.x),
          u.y),
      u.z);
}

float fbm6(vec3 p, float persistence, float lacunarity, float exponentiation) {
  float amplitude = 0.5;
  float frequency = 1.0;
  float total = 0.0;
  float normalization = 0.0;

  p = p + frag_info.seed_value;
  for (int i = 0; i < 6; ++i) {
    float noiseValue = snoise(p * frequency).w;
    total += noiseValue * amplitude;
    normalization += amplitude;
    amplitude *= persistence;
    frequency *= lacunarity;
  }

  total /= normalization;
  total = total * 0.5 + 0.5;
  total = pow(total, exponentiation);

  return total;
}

float fbm2(vec3 p, float persistence, float lacunarity, float exponentiation) {
  float amplitude = 0.5;
  float frequency = 1.0;
  float total = 0.0;
  float normalization = 0.0;
  p = p + frag_info.seed_value;

  for (int i = 0; i < 2; ++i) {
    float noiseValue = snoise(p * frequency).w;
    total += noiseValue * amplitude;
    normalization += amplitude;
    amplitude *= persistence;
    frequency *= lacunarity;
  }

  total /= normalization;
  total = total * 0.5 + 0.5;
  total = pow(total, exponentiation);

  return total;
}

vec3 GenerateStarGrid(vec2 pixel_coords,
                      float cell_width,
                      float star_radius,
                      float seed,
                      bool twinkle) {
  // fract() gives you 0.0 to 1.0 for the cell_width
  // - 0.5 will move the origin from the bottom left to the cetner
  vec2 cell_coords = fract(pixel_coords / cell_width) - 0.5;
  // "each cell is now scaled in terms of pixels"
  cell_coords *= cell_width;

  vec2 cell_id = (floor(pixel_coords / cell_width) + seed) / 100.0;  // "(x,y)"
  vec3 cell_hash_value =
      hash3(vec3(cell_id, 0.0));  // [-1, 1] - note; "z" is unused right now.

  // Hash gives you -1 to 1; saturate clamps to 0,1.  This will effectivly
  // kill some of the stars (wanted) vs remap giving you a star per-cell
  float starBrighness = saturate(cell_hash_value.z);  // -1,1 -> 0->1

  // Get a star position and
  vec2 star_position = vec2(0.0);
  star_position += cell_hash_value.xy * (cell_width * 0.5 - star_radius * 4.0);

  // float distance_to_star = length(cell_coords); // stars in the middle
  float distance_to_star = length(cell_coords + star_position);

  // better falloff
  float glow = exp(-2.0 * distance_to_star / star_radius);

  if (twinkle) {
    // verticle and horizontal flare
    float noise_sample = noise(vec3(cell_id, frag_info.time * 1.5));
    float twinkle_size =
        remap(noise_sample, -1.0, 1.0, 1.0, 0.1) * star_radius * 6.0;

    vec2 abs_distance =
        abs(cell_coords - star_position);  // manhattan distance to cell center.

    // horizontal
    float twinkleValue = smoothstep(star_radius * 0.25, 0.0, abs_distance.y) *
                         smoothstep(twinkle_size, 0.0, abs_distance.x);
    // vertical
    twinkleValue += smoothstep(star_radius * 0.25, 0.0, abs_distance.x) *
                    smoothstep(twinkle_size, 0.0, abs_distance.y);

    glow += twinkleValue;
  }

  return vec3(glow * starBrighness);
}

vec3 GenerateStars(vec2 pixel_coords) {
  vec3 stars = vec3(0.0);
  float size = 4.0;
  float cell_width = 500.0;
  for (float i = 0.0; i <= 2.0; i++) {
    stars += GenerateStarGrid(pixel_coords, cell_width, size,
                              i + frag_info.seed_value, true);
    size *= 0.5;
    cell_width *= 0.35;
  }

  for (float i = 3.0; i <= 5.0; i++) {
    stars += GenerateStarGrid(pixel_coords, cell_width, size,
                              i + frag_info.seed_value, false);
    size *= 0.5;
    cell_width *= 0.35;
  }
  return stars;
}

// 2D circle
float sdfCircle(vec2 p, float r) {
  return length(p) - r;
}

float map(vec3 pos) {
  return fbm6(pos, 0.5, 2.0, 4.0);
}

vec3 calcNormal(vec3 pos, vec3 n) {
  // if you sample the noise field along each axis, a small amount of distance
  // away from the position you're interested, you'll get a gradient
  vec2 e = vec2(0.0001, 0.0);
  /* -500.0 was added without comment, but it gives bump */
  return normalize(n + -500.0 * vec3(map(pos + e.xyy) - map(pos - e.xyy),
                                     map(pos + e.yxy) - map(pos - e.yxy),
                                     map(pos + e.yyx) - map(pos - e.yyx)));
}

mat3 rotateY(float radians) {
  float s = sin(radians);
  float c = cos(radians);
  return mat3(        // split
      c, 0.0, s,      // 1
      0.0, 1.0, 0.0,  // 2
      -s, 0.0, c);    // 3
}

vec3 DrawPlanet(vec2 pixel_coords,
                vec3 color,
                float planet_size,
                float atmosphere_thickness,
                float rotation_speed) {
  vec3 planet_color = vec3(1.0);

  // Get a nice big 2D circle and the distance to the edge.
  float d = sdfCircle(pixel_coords, planet_size);

  if (d <= 0.0) {
    // inside the planet.
    float x = pixel_coords.x / planet_size;
    float y = pixel_coords.y / planet_size;

    // surface area of a sphere is x^2 + y^2 + z^2 = 1, so...
    // z = 1 - x^2 - y^2
    float z = sqrt(1.0 - x * x - y * y);

    // sping around; right round...
    mat3 planet_rotation = rotateY(frag_info.time * rotation_speed);

    // veiw space normal;
    vec3 view_normal = vec3(x, y, z);
    vec3 worldspace_position = planet_rotation * view_normal;
    vec3 worldspace_normal = planet_rotation * normalize(worldspace_position);
    vec3 wsViewDir = planet_rotation * vec3(0.0, 0.0, 1.0);

    vec3 noise_coord = worldspace_position * 2.0;
    float noise_sample = fbm6(noise_coord, 0.5, 2.0, 4.0);
    float moistureMap = fbm2(noise_coord * 0.5 + vec3(20.0), 0.5, 2.0, 4.0);

    vec3 shallowWaterColor = vec3(0.01, 0.09, 0.55);  // light blue
    vec3 deepWater = vec3(0.09, 0.26, 0.57);
    vec3 waterColor =
        mix(shallowWaterColor, deepWater,
            smoothstep(0.02 /*deep*/, 0.06 /* shallow */, noise_sample));

    vec3 coast_land = vec3(0.5, 1.0, 0.3);
    vec3 jungle_land = vec3(0.0, 0.7, 0.0);
    vec3 land_color =
        mix(coast_land, jungle_land, smoothstep(0.05, 0.1, noise_sample));

    vec3 sandyColor = vec3(1.0, 1.0, 0.5);
    land_color =
        mix(sandyColor, land_color, smoothstep(0.05, 0.1, moistureMap));

    // Put in some mountains and snow...
    vec3 mountainColor = vec3(0.5);
    land_color =
        mix(land_color, mountainColor, smoothstep(0.1, 0.2, noise_sample));

    vec3 snowColor = vec3(1.0);
    land_color =
        mix(land_color, snowColor, smoothstep(0.15, 0.3, noise_sample));

    // Take care of the poles...
    land_color =
        mix(land_color, vec3(0.9), smoothstep(0.6, 0.9, abs(view_normal.y)));

    planet_color =
        mix(waterColor, land_color, smoothstep(0.05, 0.06, noise_sample));

    // Lighting
    // Check out the previous sections on lighting.

    // specularity of water vs land is different.
    float water_selector = smoothstep(0.05, 0.06, noise_sample);
    vec2 spec_params = mix(vec2(0.5, 32.0),  // land
                           vec2(0.01, 2.0),  // sea
                           water_selector);

    vec3 worldspace_light_direction =
        planet_rotation * normalize(vec3(0.5, 1.0, 0.5));

    // update: make water flat - though from space we should see some waves
    // (to-do)
    vec3 worldspace_surface_normal =
        mix(worldspace_normal, calcNormal(noise_coord, worldspace_normal),
            water_selector);

    if (frag_info.show_normals > 0.0) {
      planet_color = worldspace_surface_normal;
    }
    float wrap = 0.05;
    // float dp = max(0.0, dot(worldspace_light_direction,
    // worldspace_surface_normal));  // dot product nvida surface scattering
    // trick
    float dp = max(
        0.0,
        (dot(worldspace_light_direction, worldspace_surface_normal) + wrap) /
            (1.0 + wrap));

    vec3 darkRed = vec3(0.25, 0.0, 0.0);
    vec3 lightColor = mix(darkRed, vec3(0.75), smoothstep(0.05, 0.5, dp));

    vec3 ambient = vec3(0.002);  // lets not have complete darkness
    vec3 diffuse = lightColor * dp;

    vec3 r = normalize(
        reflect(-worldspace_light_direction, worldspace_surface_normal));
    float phongValue = max(0.0, dot(wsViewDir, r));
    phongValue = pow(phongValue, spec_params.y);

    vec3 specular = vec3(phongValue) * spec_params.x * diffuse;

    vec3 planetShading = planet_color * (diffuse + ambient) + specular;
    planet_color = planetShading;

    // Fresnel for atmosphere
    float fresnel = smoothstep(1.0, 0.1, view_normal.z);  // z == from camera.
    // "blue halo goes around the dark side of the planet
    // fresnel = pow(fresnel, 8.0);
    fresnel = pow(fresnel, 8.0) * dp;
    planet_color = mix(planet_color, vec3(0.0, 0.5, 1.0), fresnel);

    if (frag_info.show_noise > 0.0) {
      planet_color = vec3(noise_sample);
    }
  }
  // (color -> planet_color) when "d" is between 0 and -1 (inside the circle)
  color = mix(color, planet_color, smoothstep(0.0, -1.0, d));

  // Atmospheric glow
  // 40 pixels outside the planet
  if (d < atmosphere_thickness && d >= -1.0) {
    // color = vec3(1.0); // draw the halo
    float planetSizeAtmos = planet_size + atmosphere_thickness;
    mat3 planet_rotation = rotateY(frag_info.time * rotation_speed);

    float x = pixel_coords.x / planetSizeAtmos;
    float y = pixel_coords.y / planetSizeAtmos;
    float z = sqrt(1.0 - x * x - y * y);
    vec3 normal = planet_rotation * vec3(x, y, z);

    float lighting = dot(normal, normalize(vec3(0.5, 1.0, 0.5)));
    lighting =
        smoothstep(-0.15, 1.0, lighting);  // same as above; just no wrap.

    vec3 glow_color =
        vec3(0.05, 0.3, 0.9) * exp(-0.01 * d * d) * lighting * 0.75;
    color += glow_color;
  }

  return color;
}

void main() {
  vec2 vUvs = (gl_FragCoord.xy - 0.5) / frag_info.resolution;
  vUvs.y = 1.0 - vUvs.y;  // flip flutter's upside down.
  vec2 pixel_coords = (vUvs - 0.5) * frag_info.resolution;

  vec3 color = vec3(0.0);
  color = GenerateStars(pixel_coords);
  color = DrawPlanet(pixel_coords, color, frag_info.planet_size,
                     frag_info.planet_size * 0.1, frag_info.speed);

  frag_color = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}
