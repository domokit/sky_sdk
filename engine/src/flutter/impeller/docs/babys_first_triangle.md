# Baby's First Triangle

This guide details how to render a triangle using Impeller. We will use the lowest layer of the stack to do this and explore Impellers HAL and shader compilation machinery.

A complete code example of this tutorial is in [`renderer_unittests.cc`](../renderer/renderer_unittests.cc) in `RendererTest.BabysFirstTriangle`.

## The Pipeline

Before we do any rendering, we need to create a pipeline. A pipeline describes the fixed function and programmable stages of a rasterization job. Pipelines are expensive to create. So these are usually created upfront during setup.

### The Shaders

> [!TIP]
> When you decide what to render, it is usually a good idea to start writing the shaders first. It will give you an overview of the inputs you are going to provide to the shaders. The shader compiler will then generate the necessary interfaces. You then just wire it up in code with the niceties of code completion and such. Impeller wants you to work this way and the compiler will help you.

Shaders define the programmable stages of a pipeline. We are going to be define a vertex and fragment shader for our triangle.

The job of a vertex shader is to transform the vertices of our triangle into [normalized device coordinates](coordinate_system.md) (NDC). The rasterizer will then take these coordinates and convert them to 2D coordinates in the framebuffer.

On the other hand, the job of the fragment shader is to shade (color) the pixels covered by the triangle.

#### Vertex Shader

Let's first create `baby.vert`:

```glsl
in vec2 position;

void main() {
  gl_Position = vec4(position, 0.0, 1.0);
}
```

This shader, which expects to run once per vertex, takes a `vec2` and converts into into NDC. Are you can see, there is no "conversion" going on. That's because our vertices will already be in NDC. Since we are only drawing a simple triangle, we will need to give it three vertices. We'll discuss that in later setup.

#### Fragment Shader

And, `baby.frag`:

```glsl
out vec4 frag_color;

void main() {
  frag_color = vec4(1.0, 0.0, 0.0, 1.0);
}

```

This shader, which expects to run once per texture element (texel) covered by the triangle just returns a solid red color as output.

### The Pipeline Descriptor

Invoking the shader compiler will generate the backend specific shaders (GLSL ES for OpenGL ES, Metal Shading Language code for Metal, and SPIRV for Vulkan). Along with these artifacts, the compiler will generate a couple of C++ header files that contain the interfaces and metadata you will need to create a pipeline with these shaders at runtime. Find those somewhere in the generated artifacts. These will be called `baby.vert.h` and `baby.frag.h` for each of our two shaders. Include then in your translation unit. We need these for the pipeline descriptor.

But first, let's take a peek inside `baby.vert.h` to look at what the compiler gave us. There should be a whole bunch of metadata you don't really need to care about. But there is one struct called `PerVertexData` that looks interesting:

```c++
struct PerVertexData {
  Point position;  // (offset 0, size 8)
};  // struct PerVertexData (size 8)
```

The compiler has detected that the shader expects one point position per vertex. It is going to be our job to fill this in during rendering.

This struct is handy because as you tinker on your shader, the compiler will add, remove, and reorder the fields. If there are alignment considerations for the GPU, the compiler knows about these and it will add the appropriate padding between these fields so you all you have to worry about is filling in the position. You don't have to use this struct directly, but trusting the compiler will greatly simplify your experience.

All these interfaces and metadata are in a struct called `BabyVertexShader`. Find a similar struct called `BabyFragmentShader` in `baby.frag.h`. [Tinker around with the shader in the compiler explorer](https://tinyurl.com/28fypq2b) to see what the compiler generates.

Now, let's put together a pipeline. First you need a pipeline descriptor.

```c++
// Declare a shorthand for the shaders we are going to use.
using VS = BabyVertexShader;
using FS = BabyFragmentShader;

// Create a pipeline descriptor that uses the shaders together and default
// initializes the fixed function state.
//
// If the vertex shader outputs disagree with the fragment shader inputs, this
// will be a compile time error.
auto desc = PipelineBuilder<VS, FS>::MakeDefaultPipelineDescriptor(*context);
```

`MakeDefaultPipelineDescriptor` fills in the default values for the fixed function stages for the pipeline. Depending on the environment, some minor tweaks to this descriptor might be necessary.

Astute readers will notice that this is the first time we have tried to fuse the vertex and fragment stages together. But what happens if they aren't compatible? For instance, what if the fragment stage expects an input the vertex stage doesn't provide? The compiler has thought of this for you and the act of putting together a pipeline builder via `PipelineBuilder<VS, FS>` will statically (at build time) perform the check for you. If the stages aren't compatible the C++ code will refuse to compile.

Finally, we have everything we need, let's create the pipeline:

```c++
auto pipeline = context->GetPipelineLibrary()->GetPipeline(desc).Get();
```

This call creates a pipeline on a background thread and returns a future. But we need the pipeline right away. Just `Get` wait on the pipeline creation. Remember, pipelines are expensive to create. So just do this once upfront and keep them around for as long as possible. And don't create these in frame workloads unless you want jank.

### Vertex Data

Remember the `PerVertexData` struct the compiler generated for us? It is our job to fill in the the vertex information and provide it as a GPU buffer allocation to the draw call. Handily, there is another C++ utility called the `VertexBufferBuilder<T>` that can create this buffer for us. The template parameter T is the `PerVertexData` struct.

Let's create the data we will be giving our draw call. Remember, our vertex shader is a bit of slacker and doesn't do anything to the vertex information to convert it to normalized device coordinates. So we need to make sure the information is already in [normalized device coordinate](coordinate_system.md).

```c++
VertexBufferBuilder<VS::PerVertexData> vertex_buffer_builder;
vertex_buffer_builder.AddVertices({
    {{-0.5, -0.5}},
    {{0.0, 0.5}},
    {{0.5, -0.5}},
});
```

Ask the vertex buffer builder to create the device buffer for us. Since we aren't going to be changing the coordinates of the vertex per frame, we can do this once upfront and keep referencing the same buffer over and over in our draw calls.

```c++
auto vertex_buffer = vertex_buffer_builder.CreateVertexBuffer(
      *context->GetResourceAllocator());
```

If you change the vertex information in your shader, the `PerVertexData` struct will change and cause a compile time error where you setup your vertex buffer. This way, you can be confident that refactoring your shaders will immediately flag the instances where you are specifying vertex data to your shader.

### Draw

You've done all the heavy lifting already. Per frame, you only need to set the pipeline and vertex buffer you've stashed in the render pass and invoke a draw call.

```c++
pass.SetPipeline(pipeline);
pass.SetVertexBuffer(vertex_buffer);
pass.Draw();
```

And, in ~10 lines of C++ code and some simple GLSL, you should see a glorious red triangle. The compiler has done the heavy lifting of converting the GLSL to Metal Shading Language and figuring out the metadata to put together the pipeline for us.

![Red Triangle](https://raw.githubusercontent.com/flutter/assets-for-api-docs//5da33067f5cfc7f177d9c460d618397aad9082ca/assets/engine/impeller/babys_first_triangle/baby_triangle_red.avif "Our First Triangle!")

## Extra Credit

### Varyings

We have a triangle. But our shaders are really simple. They don't talk to one another. Let's demonstrate that. Along with the position for each vertex, let's also give each color a vertex. Then, in the fragment shader, use the position within the triangle to determine how to mix the color contributions of each vertex.

First, update `baby.vert` to indicate an additional input and output.

```glsl
in vec4 color;
out vec4 v_color;
```

In the main body of the vertex shader, just pass the input to the output.

```glsl
v_color = color;
```

In the fragment shader `baby.frag`, declare an input for the color from previous stage.

```glsl
in vec4 v_color;
```

And in the body, set the color of the fragment to the this input.

```glsl
frag_color = v_color;
```

We didn't do anything to perform the color mixing. That's because the rasterizer interpolates the values between stages. Since the varies depending on the pixel, we call these "varyings" and use the `v_` prefix for such variables.

We are done with the shaders. But the compiler now warns that the vertex buffer builder can no longer build our vertex buffer! And its right because each vertex now needs to be supplied a color as well. Patch this in.

```c++
vertex_buffer_builder.AddVertices({
    {{-0.5, -0.5}, Color::Red()},
    {{0.0, 0.5}, Color::Green()},
    {{0.5, -0.5}, Color::Blue()},
});
```

We should now see a triangle with each pixel shaded differently.

![Varying Triangle](https://raw.githubusercontent.com/flutter/assets-for-api-docs//5da33067f5cfc7f177d9c460d618397aad9082ca/assets/engine/impeller/babys_first_triangle/baby_triangle_varying.avif "Varying Triangle")

### Uniforms

You're now a pro and drawing a triangle. There is one more thing we should probably discuss that's used quite extensively. Uniform data.

So far, the data you've provided to the shader was submitted to the vertex shader and transformed by the varying fixed function unit. But perhaps you need to submit data to either shader that is constant across all invocations. This constant information called uniform data.

Before we demonstrate its use, let's decide how we want to setup our demo. I say we provide each vertex two colors and animate between the two based on the time (in seconds). The time, which is going to be uniform across all fragment shader invocations, is going to be provided as uniform data.

Patching the vertex and fragment shaders to provide data for the second color per vertex and passing that along to the fragment shader as a varying is left as an exercise to the reader. We just discussed it in the section above.

In the fragment shader (where we need to the current time in seconds), add a uniform block like so:

```glsl
uniform FragInfo {
  float time;
} frag_info;
```

The compiler will helpfully spit out a C++ struct that looks similar. Like the `PerVertexData`, the compiler knows the alignment of the various fields and will place them with the right padding as necessary. All you need to do is fill this in in C++ code and provide that data to GPU as a device buffer. Let's take a peek at what the compiler generated for us:

```c++
struct FragInfo {
  Scalar time; // (offset 0, size 4)
}; // struct FragInfo (size 4)
```

Seems pretty straightforward. Just to prove a point, I'll add a few more fields that I know need padding. [Tinker around in the compiler explorer](https://tinyurl.com/23q4o4cf) to learn more about what the compiler generates.

```glsl
uniform FragInfo {
  float time;
  vec2 bar;
  vec4 baz;
} frag_info;
```

in GLSL generates the following C++ struct. Notice the additional padding you didn't have to think about.

```c++
struct FragInfo {
  Scalar time;               // (offset 0, size 4)
  Padding<4> _PADDING_bar_;  // (offset 4, size 4)
  Point bar;                 // (offset 8, size 8)
  Vector4 baz;               // (offset 16, size 16)
};                           // struct FragInfo (size 32)
```

In our playground, create this `FragInfo` struct and set the time value as the current value in seconds.

```c++
FS::FragInfo frag_info;
frag_info.time = fml::TimePoint::Now().ToEpochDelta().ToSecondsF();
```

We need to place this in GPU accessible memory. Here, we will use another utility called the host buffer.

#### Host & Device Buffers

Unlike `malloc` buffers that can be resized via `realloc`, device buffers cannot be resized in Impeller. Also, it is bad form to create many small device buffers. For this reason, Impeller prefers to stage all data (uniform, vertex, or otherwise) into one large allocation. Then at draw time, each draw call references information at a specific offset and length into that larger allocation.

An easy to way to achieve this scheme is to use a `HostBuffer`. A host buffer is a buffer allocated on the heap (using `malloc` or similar) whose main usage is to grow as quickly as possible. Impeller stages all data for the frame in such a buffer. As the buffer is being constructed, views (offset and length) into this buffer are noted in the command stream. Just before the draw call is submitted, this information is uploaded to the GPU at which time with buffers being reused if necessary.

In the Impeller codebase, such buffers are called jumbo buffers.

We will use the same scheme to upload our uniform data to the GPU. Except, our buffers aren't going to be all that jumbo. But hopefully, this gives an idea of how the rest of Impeller works.

Let's put our FragInfo struct into a newly created host buffer (per frame) and bind it to our render pass.

```c++
auto host_buffer = HostBuffer::Create(context->GetResourceAllocator());
FS::BindFragInfo(pass, host_buffer->EmplaceUniform(frag_info));
```

But wait, where did `BindFragInfo` come frame. Well, it was generated by the compiler because the compiler know how to bind the buffer to that specific stage based on the metadata it generated. If you change the shader, you will get compiler error till you fixup all call sites, making refactoring shaders easier.

Next, lets patch our fragment shader to mix between the two values after taking into the current time into account.

```glsl
void main() {
  float floor = floor(frag_info.time);
  float fract = frag_info.time - floor;
  if (mod(int(floor), 2) == 0) {
    fract = 1.0 - fract;
  }
  frag_color = mix(v_color, v_color2, fract);
}
```

And with that, you should see animated triangle shading.

![Animating Triangle](https://raw.githubusercontent.com/flutter/assets-for-api-docs//5da33067f5cfc7f177d9c460d618397aad9082ca/assets/engine/impeller/babys_first_triangle/baby_triangle_anim.webp "Animating Triangle")

## Conclusion

You have learned how to draw a triangle, modify its vertices, access varying information in the fragment shader, and specify uniform data to your shaders. You'll find that its somewhat annoying that you can't color outside the confines of your triangle. So add more, the rest is Flutter doesn't do anything more (conceptually) complicated.

