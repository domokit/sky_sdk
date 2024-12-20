# Specialization Constants

A specialization constant is a named variable that is known to be constant at runtime but not when the shader is authored. These variables are bound to specific values when the shader is compiled on application start up and allow the backend to perform optimizations such as branch elimination and constant folding.

Specialization constants have two possible benefits when used in a shader:

  * Improving performance, by removing branching and conditional code.
  * Code organization/size, by removing the number of shader source files required.

These goals are related: The number of shaders can be reduce by adding runtime branching to create more generic shaders. Alternatively, branching can be reduced by adding more specialized shader variants. Specialization constants provide a happy medium where the source files can be combined with branching but done so in a way that has no runtime cost.

## Example Usage

Consider the case of the "decal" texture sampling mode. This is implement via clamp-to-border with
a border color set to transparent black. While this functionality is well supported on the Metal and
Vulkan backends, the GLES backend needs to support devices that do not have this extension. As a
result, the following code was used to conditionally decal:

```glsl
// Decal sample if necessary.
vec4 Sample(sampler2D sampler, vec2 coord) {
#ifdef GLES
  return IPSampleDecal(sampler, coord)
#else
  return texture(sampler, coord);
#endif
}
```

This works great as long as we know that the GLES backend can never do the decal sample mode. This is also "free" as the ifdef branch is evaluated in the compiler. But eventually, we added a runtime check for decal mode as we need to support this on GLES. So the code turned into (approximately) the following:

```glsl
#ifdef GLES
uniform float supports_decal;
#endif

// Decal sample if necessary.
vec4 Sample(sampler2D sampler, vec2 coord) {
#ifdef GLES
  if (supports_decal) {
    return texture(sampler, coord);
  }
  return IPSampleDecal(sampler, coord)
#else
  return texture(sampler, coord);
#endif
}
```

Now we've got decal support, but we've also got new problems:

* The code is actually quite messy. We have to track different uniform values depending on the backend.
* The GLES backend is still paying some cost for branching, even though we "know" that decal is or isn't supported when the shader is compiled.

### Specialization constants to the rescue

Instead of using a runtime check, we can create a specialization constant that is set when compiling the
shader. This constant will be `1` if decal is supported and `0` otherwise.

```glsl
layout(constant_id = 0) const float supports_decal = 1.0;

vec4 Sample(sampler2D sampler, vec2 coord) {
  if (supports_decal) {
    return texture(sampler, coord);
  }
  return IPSampleDecal(sampler, coord)
}

```

Immediately we realize a number of benefits:

* Code is the same across all backends
* Runtime branching cost is removed as the branch is compiled out.


## Implementation

Const values are floats and can be used to represent:

* true/false via 0/1.
* function selection, such as advanced blends. The specialization value maps to a specific blend function. For example, 0 maps to screen and 1 to overlay via a giant if/else macro.
* Only fragment shaders can be specialized. This limitation could be removed with more investment.

*AVOID* adding specialization constants for color values or anything more complex.

Specialization constants are provided to the CreateDefault argument in content_context.cc and aren't a
part of variants. This is intentional: specialization constants shouldn't be used to create (potentially unlimited) runtime variants of a shader.

Backend specific information:
* In the Metal backend, the specialization constants are mapped to a MTLFunctionConstantValues. See also: https://developer.apple.com/documentation/metal/using_function_specialization_to_build_pipeline_variants?language=objc
* In the Vulkan backend, the specialization constants are mapped to VkSpecializationINfo. See also: https://blogs.igalia.com/itoral/2018/03/20/improving-shader-performance-with-vulkans-specialization-constants/
* In the GLES backend, the SPIRV Cross compiler will generate defines named `#ifdef SPIRV_CROSS_CONSTANT_i`, where i is the index of constant. The Impeller runtime will insert `#define SPIRV_CROSS_CONSTANT_i` in the header of the shader.