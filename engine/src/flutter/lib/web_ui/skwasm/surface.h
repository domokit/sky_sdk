// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_WEB_UI_SKWASM_SURFACE_H_
#define FLUTTER_LIB_WEB_UI_SKWASM_SURFACE_H_

#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <emscripten.h>
#include <emscripten/html5_webgl.h>
#include <emscripten/threading.h>
#include <webgl/webgl1.h>
#include <cassert>
#include "export.h"
#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkColorSpace.h"
#include "third_party/skia/include/core/SkPicture.h"
#include "third_party/skia/include/core/SkSurface.h"
#include "third_party/skia/include/encode/SkPngEncoder.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/SkSurfaceGanesh.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLInterface.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLTypes.h"
#include "wrappers.h"

namespace Skwasm {
// This must be kept in sync with the `ImageByteFormat` enum in dart:ui.
enum class ImageByteFormat {
  rawRgba,
  rawStraightRgba,
  rawUnmodified,
  png,
};

class TextureSourceWrapper {
 public:
  TextureSourceWrapper(unsigned long threadId, SkwasmObject textureSource);
  ~TextureSourceWrapper();

  SkwasmObject getTextureSource();

 private:
  unsigned long _rasterThreadId;
};

class Surface {
 public:
  using CallbackHandler = void(uint32_t, void*, SkwasmObject);

  // Main thread only
  Surface();

  unsigned long getThreadId() { return _thread; }

  // Main thread only
  void dispose();
  uint32_t renderPictures(SkPicture** picture, int count);
  uint32_t rasterizeImage(SkImage* image, ImageByteFormat format);
  void setCallbackHandler(CallbackHandler* callbackHandler);
  void onRenderComplete(uint32_t callbackId, SkwasmObject imageBitmap);
  void onRasterizeComplete(uint32_t callbackId, SkData* data);

  // Any thread
  std::unique_ptr<TextureSourceWrapper> createTextureSourceWrapper(
      SkwasmObject textureSource);

  // Worker thread
  void renderPicturesOnWorker(sk_sp<SkPicture>* picture,
                              int pictureCount,
                              uint32_t callbackId,
                              double rasterStart);
  void rasterizeImageOnWorker(SkImage* image,
                              ImageByteFormat format,
                              uint32_t callbackId);

 private:
  void _runWorker();
  void _init();
  void _resizeCanvasToFit(int width, int height);
  void _recreateSurface();

  std::string _canvasID;
  CallbackHandler* _callbackHandler = nullptr;
  uint32_t _currentCallbackId = 0;

  int _canvasWidth = 0;
  int _canvasHeight = 0;

  EMSCRIPTEN_WEBGL_CONTEXT_HANDLE _glContext = 0;
  sk_sp<GrDirectContext> _grContext = nullptr;
  sk_sp<SkSurface> _surface = nullptr;
  GrGLFramebufferInfo _fbInfo;
  GrGLint _sampleCount;
  GrGLint _stencil;

  pthread_t _thread;
};
}  // namespace Skwasm

#endif  // FLUTTER_LIB_WEB_UI_SKWASM_SURFACE_H_
