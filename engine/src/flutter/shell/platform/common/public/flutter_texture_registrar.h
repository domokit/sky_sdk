// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_TEXTURE_REGISTRAR_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_TEXTURE_REGISTRAR_H_

#include <stddef.h>
#include <stdint.h>

#include "flutter_export.h"

#if defined(__cplusplus)
extern "C" {
#endif

struct FlutterDesktopTextureRegistrar;
// Opaque reference to a texture registrar.
typedef struct FlutterDesktopTextureRegistrar*
    FlutterDesktopTextureRegistrarRef;

// Possible values for the type specified in FlutterDesktopTextureInfo.
// Additional types may be added in the future.
typedef enum {
  // A Pixel buffer-based texture.
  kFlutterDesktopPixelBufferTexture,
  // A platform-specific GPU surface-backed texture.
  kFlutterDesktopGpuSurfaceTexture
} FlutterDesktopTextureType;

// Supported GPU surface types.
typedef enum {
  // Uninitialized.
  kFlutterDesktopGpuSurfaceTypeNone,
  // A DXGI shared texture handle (Windows only).
  // See
  // https://docs.microsoft.com/en-us/windows/win32/api/dxgi/nf-dxgi-idxgiresource-getsharedhandle
  kFlutterDesktopGpuSurfaceTypeDxgiSharedHandle,
  // A |ID3D11Texture2D| (Windows only).
  kFlutterDesktopGpuSurfaceTypeD3d11Texture2D
} FlutterDesktopGpuSurfaceType;

// Supported pixel formats.
typedef enum {
  // Uninitialized.
  kFlutterDesktopPixelFormatNone,
  // Represents a 32-bit RGBA color format with 8 bits each for red, green, blue
  // and alpha.
  kFlutterDesktopPixelFormatRGBA8888,
  // Represents a 32-bit BGRA color format with 8 bits each for blue, green, red
  // and alpha.
  kFlutterDesktopPixelFormatBGRA8888
} FlutterDesktopPixelFormat;

// An image buffer object.
typedef struct {
  // The pixel data buffer.
  const uint8_t* buffer;
  // Width of the pixel buffer.
  size_t width;
  // Height of the pixel buffer.
  size_t height;
  // An optional callback that gets invoked when the |buffer| can be released.
  void (*release_callback)(void* release_context);
  // Opaque data passed to |release_callback|.
  void* release_context;
} FlutterDesktopPixelBuffer;

// A GPU surface descriptor.
typedef struct {
  // The size of this struct. Must be
  // sizeof(FlutterDesktopGpuSurfaceDescriptor).
  size_t struct_size;
  // The surface handle. The expected type depends on the
  // |FlutterDesktopGpuSurfaceType|.
  //
  // Provide a |ID3D11Texture2D*| when using
  // |kFlutterDesktopGpuSurfaceTypeD3d11Texture2D| or a |HANDLE| when using
  // |kFlutterDesktopGpuSurfaceTypeDxgiSharedHandle|.
  //
  // The referenced resource needs to stay valid until it has been opened by
  // Flutter. Consider incrementing the resource's reference count in the
  // |FlutterDesktopGpuSurfaceTextureCallback| and registering a
  // |release_callback| for decrementing the reference count once it has been
  // opened.
  void* handle;
  // The physical width.
  size_t width;
  // The physical height.
  size_t height;
  // The visible width.
  // It might be less or equal to the physical |width|.
  size_t visible_width;
  // The visible height.
  // It might be less or equal to the physical |height|.
  size_t visible_height;
  // The pixel format which might be optional depending on the surface type.
  FlutterDesktopPixelFormat format;
  // An optional callback that gets invoked when the |handle| has been opened.
  void (*release_callback)(void* release_context);
  // Opaque data passed to |release_callback|.
  void* release_context;
} FlutterDesktopGpuSurfaceDescriptor;

// The pixel buffer copy callback definition provided to
// the Flutter engine to copy the texture.
// It is invoked with the intended surface size specified by |width| and
// |height| and the |user_data| held by
// |FlutterDesktopPixelBufferTextureConfig|.
//
// As this is usually called from the render thread, the callee must take
// care of proper synchronization. It also needs to be ensured that the
// returned |FlutterDesktopPixelBuffer| isn't released prior to unregistering
// the corresponding texture.
typedef const FlutterDesktopPixelBuffer* (
    *FlutterDesktopPixelBufferTextureCallback)(size_t width,
                                               size_t height,
                                               void* user_data);

// The GPU surface callback definition provided to the Flutter engine to obtain
// the surface. It is invoked with the intended surface size specified by
// |width| and |height| and the |user_data| held by
// |FlutterDesktopGpuSurfaceTextureConfig|.
typedef const FlutterDesktopGpuSurfaceDescriptor* (
    *FlutterDesktopGpuSurfaceTextureCallback)(size_t width,
                                              size_t height,
                                              void* user_data);

// An object used to configure pixel buffer textures.
typedef struct {
  // The callback used by the engine to copy the pixel buffer object.
  FlutterDesktopPixelBufferTextureCallback callback;
  // Opaque data that will get passed to the provided |callback|.
  void* user_data;
} FlutterDesktopPixelBufferTextureConfig;

// An object used to configure GPU-surface textures.
typedef struct {
  // The size of this struct. Must be
  // sizeof(FlutterDesktopGpuSurfaceTextureConfig).
  size_t struct_size;
  // The concrete surface type (e.g.
  // |kFlutterDesktopGpuSurfaceTypeDxgiSharedHandle|)
  FlutterDesktopGpuSurfaceType type;
  // The callback used by the engine to obtain the surface descriptor.
  FlutterDesktopGpuSurfaceTextureCallback callback;
  // Opaque data that will get passed to the provided |callback|.
  void* user_data;
} FlutterDesktopGpuSurfaceTextureConfig;

typedef struct {
  FlutterDesktopTextureType type;
  union {
    FlutterDesktopPixelBufferTextureConfig pixel_buffer_config;
    FlutterDesktopGpuSurfaceTextureConfig gpu_surface_config;
  };
} FlutterDesktopTextureInfo;

// Registers a new texture with the Flutter engine and returns the texture ID.
// This function can be called from any thread.
FLUTTER_EXPORT int64_t FlutterDesktopTextureRegistrarRegisterExternalTexture(
    FlutterDesktopTextureRegistrarRef texture_registrar,
    const FlutterDesktopTextureInfo* info);

// Asynchronously unregisters the texture identified by |texture_id| from the
// Flutter engine.
// An optional |callback| gets invoked upon completion.
// This function can be called from any thread.
FLUTTER_EXPORT void FlutterDesktopTextureRegistrarUnregisterExternalTexture(
    FlutterDesktopTextureRegistrarRef texture_registrar,
    int64_t texture_id,
    void (*callback)(void* user_data),
    void* user_data);

// Marks that a new texture frame is available for a given |texture_id|.
// Returns true on success or false if the specified texture doesn't exist.
// This function can be called from any thread.
FLUTTER_EXPORT bool
FlutterDesktopTextureRegistrarMarkExternalTextureFrameAvailable(
    FlutterDesktopTextureRegistrarRef texture_registrar,
    int64_t texture_id);

#if defined(__cplusplus)
}  // extern "C"
#endif

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_PUBLIC_FLUTTER_TEXTURE_REGISTRAR_H_
