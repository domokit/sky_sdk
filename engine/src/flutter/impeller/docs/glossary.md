# Glossary

### Device & Host

In the context of graphics and Impeller, the device is the GPU and the host, the CPU.

### Client Rendering API

The API Impeller uses to talk to devices. Examples of these are OpenGL, Metal, Vulkan, Direct X.

### Window System Integration (WSI)

Impeller can render using one of the client rendering APIs into a render target. However, that render target needs to be presented in the platform's window system. A Window System Integration API does this and is usually extremely platform specific. For instance, OpenGL may be available on macOS and Linux, but the WSI API for macOS is EAGL and usually (but not always) EGL for Linux.

### Varying

In the context of shaders, a varying is a value that is interpolated between two vertices (specified by the vertex shader) and provided to the fragment shader.

### OpenGL

[OpenGL and OpenGL ES (Embedded Systems)](https://www.opengl.org/) are [client rendering APIs](#client-rendering-api). Impeller uses these on older versions of Android today.

### Vulkan

[Vulkan](https://www.vulkan.org/) is a modern [client rendering API](#client-rendering-api) used by Impeller on Android. It is also available natively on the major non-Apple platforms. On the Apple platforms, Vulkan is implemented on top of [Metal](#metal) via a translation layer called [MoltenVK](https://github.com/KhronosGroup/MoltenVK).

Impeller supports a baseline of Vulkan 1.1 and uses extensions where available.

### Metal

[Metal](https://developer.apple.com/metal/) is a modern [client rendering API](#client-rendering-api) used by Impeller on macOS and iOS. It is not available on non-Apple platforms.

### EGL

[EGL](https://www.khronos.org/egl) provides [WSI](#window-system-integration-wsi) for OpenGL ES.

### Android Hardware Buffers (AHB)

Available only on Android and used by Impeller on API levels at or above 29, [AHBs](https://developer.android.com/ndk/reference/group/a-hardware-buffer) are resources that can be treated as textures by both [OpenGL](#opengl) and [Vulkan](#vulkan) and shared with the system compositor for [WSI](#window-system-integration-wsi).

In the Impeller codebase, classes that deal with AHBs have the `ahb_` prefix.
