// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

uniform samplerCube cube_map;
uniform sampler2D blue_noise;

uniform FragInfo {
  vec2 texture_size;
  float time;
}
frag_info;

in vec2 v_screen_position;
out vec4 frag_color;

const float kPi = acos(-1.0);
const float kHalfSqrtTwo = sqrt(2.0) / 2.0;
const float kEpsilon = 0.001;

// Materials (Albedo + reflectivity)
const vec4 kBottomColor = vec4(0.0, 0.34, 0.61, 0.5);
const vec4 kMiddleColor = vec4(0.16, 0.71, 0.96, 0.5);
const vec4 kTopColor = vec4(0.33, 0.77, 0.97, 0.5);
const vec4 kImpellerOuterColor = vec4(0.16, 0.71, 0.96, 0.5);
const vec4 kImpellerRimColor = vec4(0.1, 0.1, 0.1, 0.0);
const vec4 kImpellerBladeColor = vec4(0.1, 0.1, 0.1, 1.3);

// Scene
const int kMaxSteps = 70;
const float kMaxDistance = 300.0;
const vec3 kSunDirection = normalize(vec3(2, -5, 3));
const float kGlowBlend = 1.1;
const vec4 kGlowColor = vec4(0.86, 0.98, 1.0, 1);
const vec4 kGlowColor2 = vec4(1.66, 0.98, 0.5, 1);
// These refraction ratios are inverted for style purposes.
const float kAirToGlassIOR = 1.10;
const float kGlassToAirIOR = 1.0 / kAirToGlassIOR;

// Camera
const float kFocalLength = 12.0;
const float kApertureSize = 0.5;
const int kRaysPerFrag = 4;

mat3 RotateEuler(vec3 r) {
  return mat3(
      cos(r.x) * cos(r.y), cos(r.x) * sin(r.y) * sin(r.z) - sin(r.x) * cos(r.z),
      cos(r.x) * sin(r.y) * cos(r.z) + sin(r.x) * sin(r.z), sin(r.x) * cos(r.y),
      sin(r.x) * sin(r.y) * sin(r.z) + cos(r.x) * cos(r.z),
      sin(r.x) * sin(r.y) * cos(r.z) - cos(r.x) * sin(r.z), -sin(r.y),
      cos(r.y) * sin(r.z), cos(r.y) * cos(r.z));
}

//------------------------------------------------------------------------------
/// Noise functions.
///

vec2 Hash(float seed) {
  vec2 n = vec2(dot(vec2(seed, -0.1), vec2(13.8767971, 22.2091485)),
                dot(vec2(seed, -0.2), vec2(12.3432217, 48.0579381)));
  return fract(sin(n) * 24791.8159993);
}

vec4 BlueNoise(vec2 uv) {
  return texture(blue_noise, uv);
}

vec4 BlueNoiseWithRandomOffset(vec2 screen_position, float seed) {
  return BlueNoise(screen_position / 256.0 + Hash(seed));
}

//------------------------------------------------------------------------------
/// Primitive distance functions.
///

float SphereDistance(vec3 sample_position,
                     vec3 sphere_position,
                     float sphere_size) {
  return length(sample_position - sphere_position) - sphere_size;
}

float CuboidDistance(vec3 sample_position, vec3 cuboid_size) {
  vec3 space = abs(sample_position) - cuboid_size;
  return length(max(space, 0.0)) +
         min(max(space.x, max(space.y, space.z)), 0.0);
}

//------------------------------------------------------------------------------
/// Scene distance functions.
///

float GlassBox(vec3 pos) {
  mat3 basis = RotateEuler(vec3(frag_info.time * 0.21, frag_info.time * 0.24,
                                frag_info.time * 0.17));
  vec3 glass_box_pos = pos + vec3(0, -4.5 + sin(frag_info.time), 0.0);
  return CuboidDistance(basis * glass_box_pos, vec3(1, 1, 1)) - 3.0;
}

vec2 FlutterLogoField(vec3 pos) {
  pos *= 1.3;  // Scale down a bit.

  // The shape below is made up of three parallelepipeds, each of which is a
  // cuboid in scaled + skewed space. These shape fields are multiplied by the
  // inverse of the max basis vector length of the space (i.e. 1 / sqrt(2); the
  // same as kHalfSqrtTwo), which scales down the ray march step size by the
  // right amount to avoid overstepping errors.
  const float kFieldScale = kHalfSqrtTwo * 1.0 / 1.3;

  vec3 r = vec3(sin(frag_info.time * 1.137) / 7.0,        //
                sin(frag_info.time * 1.398 + 0.7) / 8.0,  //
                sin(frag_info.time * 0.873 + 0.3) / 5.0);
  // This homegrown rotation matrix isn't perfect, but it's fine for the < PI/2
  // rotation being applied to the logo.
  mat3 logo_basis = mat3(cos(r.z) * cos(r.y), sin(r.z), -sin(r.y),   //
                         -sin(r.z), cos(r.z) * cos(r.x), -sin(r.x),  //
                         sin(r.y), sin(r.x), cos(r.x) * cos(r.y));
  vec3 logo_pos =
      logo_basis * pos + vec3(-1.0, -4.0 + sin(frag_info.time), 0.0);

  // Bottom prism.

  float logo0 =
      CuboidDistance(logo_pos + vec3(-logo_pos.y, 0, 0), vec3(1, 2, 0.6)) *
      kFieldScale;
  float logo0_cutoff_plane =
      dot(logo_pos + vec3(0.5, 0.5, 0), normalize(vec3(1, 1, 0)));
  logo0 = max(logo0, logo0_cutoff_plane);

  float dist = logo0;
  float material = 1.0;

  // Middle prism.

  float logo1 =
      CuboidDistance(logo_pos + vec3(logo_pos.y, 0, 0), vec3(1, 2, 0.7)) *
      kFieldScale;
  float logo1_cutoff_plane =
      dot(logo_pos + vec3(-0.5, 0.5, 0), normalize(vec3(1, -1, 0)));
  logo1 = max(logo1, logo1_cutoff_plane);

  if (logo1 < dist) {
    dist = logo1;

    float material_cutoff_plane =
        dot(logo_pos + vec3(0.5, -0.5, 0), normalize(vec3(1, -1, 0)));
    material = material_cutoff_plane > 0.0 ? 2.0 : 3.0;
  }

  // Top prism.

  float logo2 = CuboidDistance(logo_pos + vec3(logo_pos.y - 3.0, -2, 0),
                               vec3(1, 3.5, 0.7)) *
                kFieldScale;
  logo2 = max(logo2, logo1_cutoff_plane);

  if (logo2 < dist) {
    dist = logo2;
    material = 3.0;
  }

  return vec2(dist, material);
}

vec2 InnerGlassBoxField(vec3 pos) {
  vec2 flutter_logo = FlutterLogoField(pos);
  float dist = flutter_logo.x;
  float material = flutter_logo.y;

  // Inner glass box.
  float glass_box = -GlassBox(pos);
  if (glass_box < dist) {
    dist = glass_box;
    material = -3.0;  // Transfer from glass to air.
  }

  return vec2(dist, material);
}

vec2 ImpellerField(vec3 pos) {
  float xz_dist = length(pos.xz);
  float impeller = min(0.5, xz_dist / 3.0) *
                   sin(xz_dist * 2.0 - mod(frag_info.time, kPi) * 30.0 +
                       atan(pos.z, pos.x) * 6.0) *
                   1.5;
  float impeller_side = xz_dist / 2.0 - 4.0;
  float stage_height =
      mix(impeller, impeller_side, clamp(xz_dist - 4.6, 0.0, 1.0));
  float stage_plane =
      dot(pos + vec3(0, 3.0 + stage_height, 0), normalize(vec3(0, 1, 0))) * 0.5;
  float stage_sphere = SphereDistance(pos + vec3(0, 2, 0), vec3(0), 6.0);
  float stage = max(stage_plane, stage_sphere);

  float material = 4.0;
  if (xz_dist < 5.6 && pos.y > -7.0) {
    material = (pos.y > -2.36 && pos.y < -2.1) ? 5.0 : 6.0;
  } else {
    material = 4.0;
  }

  return vec2(stage, material);
}

vec2 SceneField(vec3 pos) {
  float glass_box = GlassBox(pos);
  float dist = glass_box;
  float material = -2.0;  // Transfer from air to glass.

  vec2 impeller = ImpellerField(pos);
  if (impeller.x < dist) {
    dist = impeller.x;
    material = impeller.y;
  }

  return vec2(dist - 0.01, material);
}

/// For shadows, just ignore the glass box.
vec2 ShadowField(vec3 pos) {
  vec2 flutter_logo = FlutterLogoField(pos);
  float dist = flutter_logo.x;
  float material = flutter_logo.y;

  vec2 impeller = ImpellerField(pos);
  if (impeller.x < dist) {
    dist = impeller.x;
    material = impeller.y;
  }

  return vec2(dist, material);
}

//------------------------------------------------------------------------------
/// Surface computation.
///

vec2 March(vec3 sample_position,
           vec3 dir,
           out int steps_taken,
           bool inside_glass_box,
           bool shadow) {
  float depth = 0.0;
  for (int i = 0; i < kMaxSteps; i++) {
    if (depth > kMaxDistance) {
      steps_taken = i;
      return vec2(kMaxDistance, -1.0);
    }

    vec3 pos = sample_position + dir * depth;
    vec2 result;
    if (shadow) {
      result = ShadowField(pos);
    } else {
      result = inside_glass_box ? InnerGlassBoxField(pos) : SceneField(pos);
    }
    if (abs(result.x) < kEpsilon) {
      steps_taken = i;
      return vec2(depth, result.y);
    }

    depth += result.x;
  }
  steps_taken = kMaxSteps;
  return vec2(kMaxDistance, -1.0);
}

vec3 SceneGradient(vec3 sample_position) {
  return normalize(
      vec3(SceneField(sample_position + vec3(kEpsilon, 0, 0)).x -
               SceneField(sample_position + vec3(-kEpsilon, 0, 0)).x,
           SceneField(sample_position + vec3(0, kEpsilon, 0)).x -
               SceneField(sample_position + vec3(0, -kEpsilon, 0)).x,
           SceneField(sample_position + vec3(0, 0, kEpsilon)).x -
               SceneField(sample_position + vec3(0, 0, -kEpsilon)).x));
}

vec3 InnerGlassGradient(vec3 sample_position) {
  return normalize(
      vec3(InnerGlassBoxField(sample_position + vec3(kEpsilon, 0, 0)).x -
               InnerGlassBoxField(sample_position + vec3(-kEpsilon, 0, 0)).x,
           InnerGlassBoxField(sample_position + vec3(0, kEpsilon, 0)).x -
               InnerGlassBoxField(sample_position + vec3(0, -kEpsilon, 0)).x,
           InnerGlassBoxField(sample_position + vec3(0, 0, kEpsilon)).x -
               InnerGlassBoxField(sample_position + vec3(0, 0, -kEpsilon)).x));
}

float MarchShadow(vec3 position) {
  int shadow_steps;
  vec2 shadow_result = March(position + -kSunDirection * 0.03, -kSunDirection,
                             shadow_steps, false, true);
  float shadow_percentage = (float(shadow_steps)) / float(kMaxSteps);
  float shadow_multiplier = 1.6 - shadow_percentage;
  if (shadow_result.x < kMaxDistance) {
    shadow_multiplier = 0.6;
  }
  return shadow_multiplier;
}

//------------------------------------------------------------------------------
/// Color composition.
///

vec4 EnvironmentColor(vec3 ray_direction) {
  return texture(cube_map, ray_direction);
}

vec4 SurfaceColor(vec3 ray_direction,
                  vec3 surface_position,
                  vec3 surface_normal,
                  float material,
                  float shadow_multiplier) {
  vec3 reflection_direction = reflect(ray_direction, surface_normal);
  vec4 reflection_color = texture(cube_map, reflection_direction);

  vec4 material_value;
  if (material < 1.5) {
    material_value = kBottomColor;
  } else if (material < 2.5) {
    material_value = kMiddleColor;
  } else if (material < 3.5) {
    material_value = kTopColor;
  } else if (material < 4.5) {
    material_value = kImpellerOuterColor;
  } else if (material < 5.5) {
    material_value = kImpellerRimColor;
  } else {
    material_value = kImpellerBladeColor;
  }

  return mix(vec4(material_value.rgb * shadow_multiplier, 1.0),
             reflection_color,
             dot(-ray_direction, surface_normal) - 1.0 + material_value.a);
}

vec4 SceneColor(vec3 ray_position,
                vec3 ray_direction,
                vec3 surface_normal,
                float dist,
                float material,
                int steps_taken,
                float shadow_multiplier,
                vec4 ray_noise) {
  vec4 result_color;
  if (dist >= kMaxDistance) {
    result_color = EnvironmentColor(ray_direction);
  } else {
    vec3 surface_position = ray_position + ray_direction * dist;
    result_color = SurfaceColor(ray_direction, surface_position, surface_normal,
                                material, shadow_multiplier);
  }
  float glow_factor = float(steps_taken) / float(kMaxSteps);
  vec4 glow_color =
      mix(kGlowColor, kGlowColor2, sin(frag_info.time / 3.0) * 0.5 + 0.5);
  return mix(result_color, glow_color, glow_factor * kGlowBlend);
}

vec4 CombinedColor(vec3 ray_position, vec3 ray_direction, vec4 ray_noise) {
  int steps_taken;
  vec2 result = March(ray_position, ray_direction, steps_taken, false, false);
  ray_position = ray_position + ray_direction * result.x;
  vec3 surface_normal = SceneGradient(ray_position);

  float glass_reflection_factor = 0.0;
  vec4 glass_reflection_color = vec4(0);
  if (result.y == -2.0) {  // March into the glass.
    vec3 glass_reflection_direction = reflect(ray_direction, surface_normal);
    glass_reflection_color = EnvironmentColor(glass_reflection_direction);
    glass_reflection_factor =
        0.5 - dot(glass_reflection_direction, surface_normal) * 0.6;

    ray_direction = refract(ray_direction, surface_normal, kAirToGlassIOR);
    ray_position += ray_direction * 0.5;
    int steps;
    result = March(ray_position, ray_direction, steps, true, false);
    steps_taken += steps;

    ray_position = ray_position + ray_direction * result.x;
    surface_normal = InnerGlassGradient(ray_position);
  }

  if (result.y == -3.0) {  // March out of the glass.
    ray_direction = refract(ray_direction, surface_normal, kGlassToAirIOR);
    ray_position += ray_direction * 1.0;
    int steps;
    result = March(ray_position + ray_direction * result.x, ray_direction,
                   steps, false, false);
    steps_taken += steps;
    ray_position = ray_position + ray_direction * result.x;
    surface_normal = SceneGradient(ray_position);
  }

  float shadow_multiplier = MarchShadow(ray_position);
  vec4 scene_color =
      SceneColor(ray_position, ray_direction, surface_normal, result.x,
                 result.y, steps_taken, shadow_multiplier, ray_noise);

  return mix(scene_color, glass_reflection_color, glass_reflection_factor);
}

//------------------------------------------------------------------------------
/// Camera/lens.
///

vec3 GetFragDirection(vec2 uv, vec3 cam_forward) {
  vec2 lens_uv =
      (uv - 0.5 * frag_info.texture_size) / frag_info.texture_size.xx;
  vec3 cam_right = cross(cam_forward, vec3(0, 1, 0));
  vec3 cam_up = cross(cam_forward, cam_right);

  float fov = 65.0 * kPi / 180.0;
  return normalize(cam_forward * cos(fov) + cam_right * lens_uv.x * sin(fov) +
                   cam_up * lens_uv.y * sin(fov));
}

void main() {
  float cam_time = frag_info.time / 2.0;
  vec3 cam_position =
      vec3(-sin(cam_time + 0.2) * 6.25, -cos(cam_time + 0.3) * 2.9 + 1.0,
           -cos(cam_time - 0.1) * 5.4) *
      2.0;
  vec3 cam_direction = normalize(-cam_position);
  cam_position += vec3(0, 2, 0);

  vec3 ray_direction = GetFragDirection(v_screen_position, cam_direction);
  vec3 lens_position = cam_position + ray_direction * kFocalLength;

  for (int i = 0; i < kRaysPerFrag; i++) {
    vec4 ray_noise = BlueNoiseWithRandomOffset(
        v_screen_position, float(i) + mod(frag_info.time, 10.0));
    // The rays should be starting from a flat position on the lens, but just
    // jittering them around in a 3d box looks good enough.
    vec3 ray_start = cam_position + ray_noise.xyz * kApertureSize;
    vec3 ray_direction = normalize(lens_position - ray_start);

    vec4 result_color = CombinedColor(ray_start, ray_direction, ray_noise);
    frag_color += result_color / float(kRaysPerFrag);
  }
}
