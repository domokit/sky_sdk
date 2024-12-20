# OpenGL ES Development Setup

## Setting up Playgrounds

Out of the box, playgrounds should be functional with OpenGL ES.

### Interactive Playgrounds

Interactive playground windows are disabled by default and need to be explicitly enabled.

When working with playgrounds, it is often a good idea to disable the test timeout watchdog that kills tests that appear to have hung (the default timeout is 300 seconds).

Do both using:

```sh
--enable_playground --timeout=0
```

By default, the playgrounds will try to use the default OpenGL ES driver available on the host. This usually works fine on Linux and Windows but it may be a good idea to use the Angle OpenGL ES emulation layer if you don't have the default driver. The macOS OpenGL driver is a bit of a disaster and Angle will be used by default.

```sh
--use_angle
```

To verify that you are using Angle, refer to the window title which specifies if Angle (or SwiftShader with Vulkan) is being used.

![Use Angle](./assets/opengles_development_setup/angle.png)

Your playgrounds will run one after another. You can skip to the next one by pressing either `ESC`, `q`, or closing the window. To skip all remaining tests, press `Shift` + `ESC`.

If you are visually verifying a subset of tests quickly, you can specify how long each playground window will remain open. The next playground window will open after the timeout.

```sh
--playground_timeout_ms=1000
```

> [!TIP]
> To render one and only frame of each playground, specify the a timeout of 0 milliseconds.

### Selecting a Subset of Tests to Run

You are typically only going to run a small subset of tests during development. Pass a regex to the GTest filter to restrict running just the playgrounds you are interested in.

To construct the regex, remember the following convention:

* All playground tests are part of a single suite with the `"Play/"` prefix.
* All playgrounds tests are parameterized by the rendering backend. Today, there are three backends; Metal, Vulkan, and OpenGLES. The backend appears as a suffix towards the end of the test case. So `"/Metal"`, `"/OpenGLES"`, and `"/Vulkan"`.

If you are running just the playgrounds with the OpenGL ES backend, you'd do something like:

```sh
--gtest_filter="Play/*Foo*/OpenGLES"
```

On the other hand, if you are comparing the results of different backends, try something like:

```sh
--gtest_filter="Play/*Foo*/*"
```

The backend used along with any driver specific modifier (like the use of Angle or SwiftShader) will appear in the Window title.

### Frame-capture, Debugging, and Profiling OpenGL ES

The best OpenGL ES frame debugger and profiler on macOS is a Metal frame debugger and profiler.

Setup Xcode for frame captures with Playgrounds [using the documented instructions](./xcode_frame_capture.md). You should already be [familiar with reading Metal frame captures](./read_frame_captures.md).

We are going to use Angle to translate OpenGL ES calls into Metal calls and then debug/profile the results. You can switch backends by tinkering with the filters as command line arguments in Xcode run scheme.

> [!TIP]
> You'll be editing the Xcode run scheme a lot as you switch backends and tests. The shortcut to edit the scheme is `⌘ + ⇧ + r`.

An alternative on non-macOS platforms is RenderDoc. Instructions to [setup RenderDoc are here](./renderdoc_frame_capture.md). RenderDoc is **not** available on macOS.

#### What Works

* **1-1 relationship between most OpenGL ES and Metal resources (see exceptions)**.

![Resources](./assets/opengles_development_setup/resources.png)

* **Stepping into the Angle driver** and tracing how it converts OpenGL calls to Metal.
* **Verifying load-store actions on render pass attachments**: This comes in handy when verifying correctness around `EXT_discard_framebuffer` and memory usage.

![Load Store Actions](./assets/opengles_development_setup/load_store.png)

* **Pass dependency viewer**: While viewing the dependencies works, the dependencies seem to be over-specified. That is, Angle seems to be inserting dependencies based on completion of entire passes instead of waiting for resources in the pass to be ready for the next pass. Be aware that this will differ when comparing directly with Metal. Though this is less efficient, it is likely easier to read and comprehend in the trace.

![Pass Dependencies](./assets/opengles_development_setup/pass_deps.png)

* **The Performance HUD**: This can be [setup the same as Metal](./metal_validation.md). Remember, we are running Angle over Metal still. In most tests, you can expect OpenGL ES to take about 33% more memory because of sub-optimal load-store attachment actions and a final copy for composition. Instead of comparing performance across backends, look for trends and improvements within a test case.

![Performance HUD](./assets/opengles_development_setup/hud.png)

* **The Geometry Viewer**: While the vertex buffers may not identical (see the note below), the geometry viewer should still be functional along with the vertex debugger. Even though the vertex debugger is almost entirely useless because you'll be debugging the extremely verbose Angle generated shader code, you should be able to spot issues due to buffer corruption and [global transformations](./coordinate_system.md).

![Geometry Viewer](./assets/opengles_development_setup/geometry_viewer.png)

* **Fragment Shader Debugger**: While the fragment shader debugger technically works, it is effectively useless. Angle generated shaders are extremely verbose and large. Our shader compiler is generally pretty good at generating readable Metal code directly and also generate functionally identical OpenGL ES code. It is recommended to debug shaders using the Metal backend directly.

#### What Doesn't Work

* **1-1 relationship between vertex, index buffers, and uniform buffers**: Uniform buffers are emulated in Impeller, so there is no way such a buffer will be passed along to Metal. Angle also seems to be using intermediate allocations for the rest. Just don't expect to see the buffers you put together carefully to show up directly in the debugger.

![Buffer Soup](./assets/opengles_development_setup/buffer_soup.png)

* **Labeling metal resources**: Though Angle tracks debug labels, the corresponding Metal resources are not tagged with the same labels. Admittedly, this can get contentious as not all OpenGL resources have Metal counterparts. Butm Angle doesn't seem to tag even resources like textures. Getting the label you just set in code does work however. Its just the resource inspector resources that are unlabelled.
* **Pushing and popping debug groups**: Angle assumes these are no-ops. Rather unnecessarily, Angle does log these messages to the console.
* **1-1 relationship between render passes above the Impeller HAL and the render passes constructed by Angle**: This is because OpenGL ES doesn't have the concept of a render pass itself. While you can be fairly confident that a render pass in Impeller maps to at least one render pass in the generated Metal command stream, the use of techniques like framebuffer-fetch and final composition will mess up this relationship.
