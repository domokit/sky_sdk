# Using Impeller as a Standalone Rendering Library (with OpenGL ES)

This guide describes how to use Impeller as a standalone rendering library using OpenGL ES. Additionally, some form of Window System Integration (WSI) is essential. Since EGL is the most popular form of performing WSI on platforms with OpenGL ES, Impeller has a toolkit that assists in working with EGL. This guide will use that toolkit.

While this guide focuses on OpenGL ES with EGL, the steps to set up rendering with another client rendering API (Metal and Vulkan) are fairly similar and you should be able to follow the same pattern for other backends.

This guide details extremely low-level setup and the `//impeller/renderer` API directly above the Hardware Abstraction Layer (HAL). Most users of Impeller will likely use the API using convenience wrappers already written for the platform. Interacting directly with the HAL is extremely powerful but also verbose. Applications are likely to also use higher level frameworks like Aiks or Display Lists.

Building Impeller for the target platform is outside the scope of this guide.

> [!CAUTION]
> The code provided inline is pseudo-code and doesn't include error handling. See the headerdocs for more on error handling and failure modes. All classes are assumed to be in the `impeller` namespace. For a more complete example of setting up standalone Impeller, see [this patch](https://github.com/flutter/engine/pull/52472/files) that adds support for Impeller rendering via Wasm [in the browser and WebGL 2](https://public.chinmaygarde.com/impeller/wasm/wasm.html).

# Set up

To get started with Impeller rendering, you need to set up a context and a renderer. For backend-specific classes, the convention in Impeller is to append the backend name as a suffix. So if you need to create an instance of an `impeller::Context` for OpenGLES, look for `impeller::ContextGLES`.

In our case, we need a `impeller::ContextGLES` and `impeller::Renderer`.

## Setting up WSI with the EGL Toolkit

Before we get to `OpenGLES` we need to do a bit of WSI. While we can resort to using EGL directly, Impeller has a handy toolkit for it.

First create an EGL display connection:

```c++
egl::Display display;
```

Ask the display for a valid EGL configuration. Impeller needs an OpenGL ES 2.0 configuration.

```c++
egl::ConfigDescriptor egl_desc;
egl_desc.api = egl::API::kOpenGLES2;
egl_desc.samples = egl::Samples::kOne;
egl_desc.color_format = egl::ColorFormat::kRGBA8888;
egl_desc.stencil_bits = egl::StencilBits::kZero;
egl_desc.depth_bits = egl::DepthBits::kZero;
egl_desc.surface_type = egl::SurfaceType::kWindow;

auto config = display.ChooseConfig(egl_desc);
```

Once a valid config has been obtained, create a context and window surface. Creating the window surface requires a native window handle. Get the appropriate one for your platform. For instance, on Android this is an `ANativeWindow`.

```c++
auto context = display.CreateContext(*config, /* sharegroup= */ nullptr );
auto surface = display.CreateWindowSurface(*config, native_window_handle);
```

Now that we have a context, make it current on the calling thread. This will complete the setup of WSI.

```c++
context->MakeCurrent(*surface);
```

## Creating the OpenGL ES Context

Impeller doesn't statically link against OpenGL ES. You need to give it a callback the returns the appropriate OpenGL ES function for given name. With EGL, this can be something as simple as:

```c++
auto resolver = [](const char* name) -> void* {
  return reinterpret_cast<void*>(::eglGetProcAddress(name));
};
```

Adjust as necessary.

Once you have the resolver, you need to create an OpenGL ES proc table. The proc table contains the function pointers for the subset of the OpenGL ES API used by Impeller.

```c++
auto gl = std::make_unique<ProcTableGLES>(resolver);
```

Once the proc table is created, the resolver will no longer be invoked.

Then you need to provide the context a shader library that contains a manifest of all the shaders the context will need at runtime. Remember, Impeller doesn't generate shaders at runtime. Instead `impellerc` generates blobs that can either be delivered out of band or be embedded directly in the binary. When embedding the blobs directly in the binary, look for the symbols referring to the shader blob somewhere in the generated build artifacts. A vector of mappings to these blobs needs to be provided to create context creation factory.

An example of creating an embedded mapping is provided below. Adjust as necessary depending on how to plan on delivering shader blobs to Impeller at during setup.
```c++

#include "impeller/fixtures/gles/fixtures_shaders_gles.h" // <---- Depends on your application.

static std::vector<std::shared_ptr<fml::Mapping>>
ShaderLibraryMappingsForApplication() {
  return {
      std::make_shared<fml::NonOwnedMapping>(
          impeller_fixtures_shaders_gles_data,
          impeller_fixtures_shaders_gles_length),
  };
}

// In the setup routine.

auto mappings = ShaderLibraryMappingsForApplication();
```

And that's it. You have all the ingredients necessary to create a context. Create one now.

```c++
auto context = ContextGLES::Create(
    std::move(gl),        // proc table
    std::move(mappings),  // shader libraries
    false                 // enable tracing
);
```

Now for a tricky bit about OpenGL ES. Impeller is multi-threaded. Even when using OpenGL ES, objects above the Impeller HAL can be created, used, and consumed on any thread. But OpenGL ES isn't. Impeller also doesn't know anything about EGL. So it is your responsibility to tell Impeller which threads are safe to use OpenGL ES on.

In our little toy setup, we only have a single thread. We tell Impeller which thread to use by creating a subclass of an `impeller::ReactorGLES::Worker`. Let's create a simple worker:

```c++
class ReactorWorker final : public impeller::ReactorGLES::Worker {
 public:
  ReactorWorker() = default;

  // |ReactorGLES::Worker|
  ~ReactorWorker() override = default;

  ReactorWorker(const ReactorWorker&) = delete;

  ReactorWorker& operator=(const ReactorWorker&) = delete;

  // |ReactorGLES::Worker|
  bool CanReactorReactOnCurrentThreadNow(
      const ReactorGLES& reactor) const override {
    return true;
  }
};
```

Add an instance of this reactor worker to the context. Whew, that was unnecessarily complicated. But this is only necessary for OpenGL ES because of its tricky threading. Skip this step for Vulkan and Metal. Those APIs are already thread safe.

```c++
context->AddReactorWorker(worker);
```

Once you have the context, use it to create a renderer.

```c++
auto renderer = std::make_shared<Renderer>(context);
```

And setup is done. Keep the context and renderer around. We will be using it during frame rendering.

# Rendering Frames

Rendering frames is a matter of:
* Wrapping the onscreen texture as a Surface (`impeller::SurfaceGLES` in our case).
* Giving the surface to our renderer to render.
* After the renderer prepares a render target from this surface and invokes a supplied callback with a render target, setting up a render pass directed at that render target.
* Populating the render pass how we see fit.
* Telling our reactor worker that all operations need to be flushed to OpenGL.
* Presenting the onscreen surface.
* Repeating for a new frame.

## Wrap the Onscreen Surface

Per frame, the onscreen surface can be wrapped using `SurfaceGLES::WrapFBO` where the default framebuffer in our case is FBO 0. Take care to ensure that the pixel format matches the one we used to choose the EGL config. Figuring out the pixel size is left as an exercise for the reader.

```c++
auto surface =
    SurfaceGLES::WrapFBO(context,                         // context
                         swap_callback,                   // swap callback
                         0u,                              // fbo
                         PixelFormat::kR8G8B8A8UNormInt,  // pixel format
                         GetWindowSize()                  // surface size
    );
```

## Set the the Swap Callback

The swap callback will get invoked when the renderer presents the surface. Remember in our list of things to do, we need to first tell the reactor worker to flush all pending OpenGL operations and then present the surface. Set the swap callback appropriately.

```c++
SurfaceGLES::SwapCallback swap_callback =
    [surface, context]() -> bool {
  context->GetReactor()->React();
  return surface->Present();
};
```

## Render to the Surface

Give the surface to the renderer along with a callback that details how you will populate the render target the renderer sets up that is directed at that surface.

```c++
renderer_->Render(std::move(surface),
                 [&](RenderTarget& render_target) -> bool {
                    // Do things that render into the render target.
                    return true;
                 });
```

And that's it. Now you have a functional WSI and render loop. Higher level frameworks like Aiks and DisplayList use the render target to render their rendering intent onto to the surface.
