// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <assert.h>
#include <stdio.h>

#define GLFW_INCLUDE_NONE
#include "GLFW/glfw3.h"
#define GLFW_EXPOSE_NATIVE_COCOA
#import "GLFW/glfw3native.h"

#include "impeller.h"

#include <AppKit/AppKit.h>
#include <Metal/Metal.h>
#include <QuartzCore/QuartzCore.h>

void GLFWErrorCallback(int error, const char* description) {
  // NOLINTNEXTLINE(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
  fprintf(stderr, "GLFW Error (%d): %s\n", error, description);
  fflush(stderr);
}

int main(int argc, char const* argv[]) {
  glfwSetErrorCallback(GLFWErrorCallback);
  [[maybe_unused]] int result = glfwInit();
  assert(result == GLFW_TRUE);

  if (glfwGetPlatform() != GLFW_PLATFORM_COCOA) {
    // NOLINTNEXTLINE(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
    fprintf(stderr,
            "Metal is only available on macOS. Please try either Vulkan or "
            "OpenGL (ES).\n");
    fflush(stderr);
    return -1;
  }

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

  GLFWwindow* window =
      glfwCreateWindow(800, 600, "Impeller Example (Metal)", NULL, NULL);
  assert(window != NULL);

  int framebuffer_width, framebuffer_height;
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);

  ImpellerContext context = ImpellerContextCreateMetalNew(IMPELLER_VERSION);
  assert(context != NULL);

  // This example assumes Automatic Reference Counting (ARC) in Objective-C is
  // enabled.
  NSWindow* cocoa_window = glfwGetCocoaWindow(window);
  assert(cocoa_window != NULL);
  CAMetalLayer* layer = [CAMetalLayer layer];
  layer.framebufferOnly = NO;
  layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
  layer.device = MTLCreateSystemDefaultDevice();
  cocoa_window.contentView.layer = layer;
  cocoa_window.contentView.wantsLayer = YES;

  ImpellerDisplayList dl = NULL;

  {
    ImpellerDisplayListBuilder builder = ImpellerDisplayListBuilderNew(NULL);
    ImpellerPaint paint = ImpellerPaintNew();

    // Clear the background to a white color.
    ImpellerColor clear_color = {1.0, 1.0, 1.0, 1.0};
    ImpellerPaintSetColor(paint, &clear_color);
    ImpellerDisplayListBuilderDrawPaint(builder, paint);

    // Draw a red box.
    ImpellerColor box_color = {1.0, 0.0, 0.0, 1.0};
    ImpellerPaintSetColor(paint, &box_color);
    ImpellerRect box_rect = {10, 10, 100, 100};
    ImpellerDisplayListBuilderDrawRect(builder, &box_rect, paint);

    dl = ImpellerDisplayListBuilderCreateDisplayListNew(builder);

    ImpellerPaintRelease(paint);
    ImpellerDisplayListBuilderRelease(builder);
  }

  assert(dl != NULL);

  while (!glfwWindowShouldClose(window)) {
    glfwWaitEvents();

    // React to window resizes.
    layer.drawableSize = layer.bounds.size;

    ImpellerSurface surface = ImpellerSurfaceCreateWrappedMetalDrawableNew(
        context, (__bridge void*)layer.nextDrawable);
    assert(surface != NULL);
    ImpellerSurfaceDrawDisplayList(surface, dl);
    ImpellerSurfacePresent(surface);
    ImpellerSurfaceRelease(surface);
  }

  ImpellerDisplayListRelease(dl);
  ImpellerContextRelease(context);

  glfwMakeContextCurrent(NULL);

  glfwDestroyWindow(window);

  glfwTerminate();
  return 0;
}
