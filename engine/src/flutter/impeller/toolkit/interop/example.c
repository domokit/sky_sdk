// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <assert.h>
#include <stdio.h>

#include "GLFW/glfw3.h"
#include "impeller.h"

void GLFWErrorCallback(int error, const char* description) {
  // NOLINTNEXTLINE(clang-analyzer-security.insecureAPI.DeprecatedOrUnsafeBufferHandling)
  fprintf(stderr, "GLFW Error (%d): %s\n", error, description);
  fflush(stderr);
}

void* ProcAddressCallback(const char* proc_name, void* user_data) {
  return glfwGetProcAddress(proc_name);
}

int main(int argc, char const* argv[]) {
  glfwSetErrorCallback(GLFWErrorCallback);
  [[maybe_unused]] int result = glfwInit();
  assert(result == GLFW_TRUE);

  glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);

  GLFWwindow* window =
      glfwCreateWindow(800, 600, "Impeller Example", NULL, NULL);
  assert(window != NULL);

  int framebuffer_width, framebuffer_height;
  glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);

  // The GL context must be current on the calling thread.
  glfwMakeContextCurrent(window);

  ImpellerContext context = ImpellerContextCreateOpenGLESNew(
      IMPELLER_VERSION, ProcAddressCallback, NULL);
  assert(context != NULL);

  ImpellerISize surface_size = {};
  surface_size.width = framebuffer_width;
  surface_size.height = framebuffer_height;

  ImpellerSurface surface = ImpellerSurfaceCreateWrappedFBONew(
      context, 0u, kImpellerPixelFormatRGBA8888, &surface_size);
  assert(surface != NULL);

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
    ImpellerSurfaceDrawDisplayList(surface, dl);
    glfwSwapBuffers(window);
  }

  ImpellerDisplayListRelease(dl);
  ImpellerSurfaceRelease(surface);
  ImpellerContextRelease(context);

  glfwMakeContextCurrent(NULL);

  glfwDestroyWindow(window);

  glfwTerminate();
  return 0;
}
