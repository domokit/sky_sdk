// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "surface.h"
#include <algorithm>

#include "skwasm_support.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/gl/GrGLMakeWebGLInterface.h"

using namespace Skwasm;

// Worker thread only
void Surface::dispose() {
  delete this;
}

// Main thread only
uint32_t Surface::renderPictures(SkPicture** pictures, int count) {
  assert(emscripten_is_main_browser_thread());
  uint32_t callbackId = ++_currentCallbackId;
  std::unique_ptr<sk_sp<SkPicture>[]> picturePointers =
      std::make_unique<sk_sp<SkPicture>[]>(count);
  for (int i = 0; i < count; i++) {
    picturePointers[i] = sk_ref_sp(pictures[i]);
  }

  // Releasing picturePointers here and will recreate the unique_ptr on the
  // other thread See surface_renderPicturesOnWorker
  skwasm_dispatchRenderPictures(_thread, this, picturePointers.release(), count,
                                callbackId);
  return callbackId;
}

// Main thread only
uint32_t Surface::rasterizeImage(SkImage* image, ImageByteFormat format) {
  assert(emscripten_is_main_browser_thread());
  uint32_t callbackId = ++_currentCallbackId;
  image->ref();

  skwasm_dispatchRasterizeImage(_thread, this, image, format, callbackId);
  return callbackId;
}

std::unique_ptr<TextureSourceWrapper> Surface::createTextureSourceWrapper(
    SkwasmObject textureSource) {
  return std::unique_ptr<TextureSourceWrapper>(
      new TextureSourceWrapper(_thread, textureSource));
}

// Main thread only
void Surface::setCallbackHandler(CallbackHandler* callbackHandler) {
  assert(emscripten_is_main_browser_thread());
  _callbackHandler = callbackHandler;
}

// Worker thread only
void Surface::_runWorker() {
  _init();
  emscripten_exit_with_live_runtime();
}

// Worker thread only
void Surface::_init() {
  // Listen to messages from the main thread
  skwasm_connectThread(0);
  _glContext = skwasm_createOffscreenCanvas(256, 256);
  if (!_glContext) {
    printf("Failed to create context!\n");
    return;
  }

  makeCurrent(_glContext);
  emscripten_webgl_enable_extension(_glContext, "WEBGL_debug_renderer_info");

  _grContext = GrDirectContexts::MakeGL(GrGLInterfaces::MakeWebGL());

  // WebGL should already be clearing the color and stencil buffers, but do it
  // again here to ensure Skia receives them in the expected state.
  emscripten_glBindFramebuffer(GL_FRAMEBUFFER, 0);
  emscripten_glClearColor(0, 0, 0, 0);
  emscripten_glClearStencil(0);
  emscripten_glClear(GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
  _grContext->resetContext(kRenderTarget_GrGLBackendState |
                           kMisc_GrGLBackendState);

  // The on-screen canvas is FBO 0. Wrap it in a Skia render target so Skia
  // can render to it.
  _fbInfo.fFBOID = 0;
  _fbInfo.fFormat = GL_RGBA8_OES;

  emscripten_glGetIntegerv(GL_SAMPLES, &_sampleCount);
  emscripten_glGetIntegerv(GL_STENCIL_BITS, &_stencil);
}

// Worker thread only
void Surface::_resizeCanvasToFit(int width, int height) {
  if (!_surface || width > _canvasWidth || height > _canvasHeight) {
    _canvasWidth = std::max(width, _canvasWidth);
    _canvasHeight = std::max(height, _canvasHeight);
    _recreateSurface();
  }
}

// Worker thread only
void Surface::_recreateSurface() {
  makeCurrent(_glContext);
  skwasm_resizeCanvas(_glContext, _canvasWidth, _canvasHeight);
  auto target = GrBackendRenderTargets::MakeGL(_canvasWidth, _canvasHeight,
                                               _sampleCount, _stencil, _fbInfo);
  _surface = SkSurfaces::WrapBackendRenderTarget(
      _grContext.get(), target, kBottomLeft_GrSurfaceOrigin,
      kRGBA_8888_SkColorType, SkColorSpace::MakeSRGB(), nullptr);
}

// Worker thread only
void Surface::renderPicturesOnWorker(sk_sp<SkPicture>* pictures,
                                     int pictureCount,
                                     uint32_t callbackId,
                                     double rasterStart) {
  // This is populated by the `captureImageBitmap` call the first time it is
  // passed in.
  SkwasmObject imagePromiseArray = __builtin_wasm_ref_null_extern();
  for (int i = 0; i < pictureCount; i++) {
    sk_sp<SkPicture> picture = pictures[i];
    SkRect pictureRect = picture->cullRect();
    SkIRect roundedOutRect;
    pictureRect.roundOut(&roundedOutRect);
    _resizeCanvasToFit(roundedOutRect.width(), roundedOutRect.height());
    SkMatrix matrix =
        SkMatrix::Translate(-roundedOutRect.fLeft, -roundedOutRect.fTop);
    makeCurrent(_glContext);
    auto canvas = _surface->getCanvas();
    canvas->drawColor(SK_ColorTRANSPARENT, SkBlendMode::kSrc);
    canvas->drawPicture(picture, &matrix, nullptr);
    _grContext->flush(_surface.get());
    imagePromiseArray =
        skwasm_captureImageBitmap(_glContext, roundedOutRect.width(),
                                  roundedOutRect.height(), imagePromiseArray);
  }
  skwasm_resolveAndPostImages(this, imagePromiseArray, rasterStart, callbackId);
}

// Worker thread only
void Surface::rasterizeImageOnWorker(SkImage* image,
                                     ImageByteFormat format,
                                     uint32_t callbackId) {
  // We handle PNG encoding with browser APIs so that we can omit libpng from
  // skia to save binary size.
  assert(format != ImageByteFormat::png);
  sk_sp<SkData> data;
  SkAlphaType alphaType = format == ImageByteFormat::rawStraightRgba
                              ? SkAlphaType::kUnpremul_SkAlphaType
                              : SkAlphaType::kPremul_SkAlphaType;
  SkImageInfo info = SkImageInfo::Make(image->width(), image->height(),
                                       SkColorType::kRGBA_8888_SkColorType,
                                       alphaType, SkColorSpace::MakeSRGB());
  size_t bytesPerRow = 4 * image->width();
  size_t byteSize = info.computeByteSize(bytesPerRow);
  data = SkData::MakeUninitialized(byteSize);
  uint8_t* pixels = reinterpret_cast<uint8_t*>(data->writable_data());

  // TODO(jacksongardner):
  // Normally we'd just call `readPixels` on the image. However, this doesn't
  // actually work in some cases due to a skia bug. Instead, we just draw the
  // image to our scratch canvas and grab the pixels out directly with
  // `glReadPixels`. Once the skia bug is fixed, we should switch back to using
  // `SkImage::readPixels` instead.
  // See https://g-issues.skia.org/issues/349201915
  _resizeCanvasToFit(image->width(), image->height());
  auto canvas = _surface->getCanvas();
  canvas->drawColor(SK_ColorTRANSPARENT, SkBlendMode::kSrc);

  // We want the pixels from the upper left corner, but glReadPixels gives us
  // the pixels from the lower left corner. So we have to flip the image when we
  // are drawing it to get the pixels in the desired order.
  canvas->save();
  canvas->scale(1, -1);
  canvas->drawImage(image, 0, -_canvasHeight);
  canvas->restore();
  _grContext->flush(_surface.get());

  emscripten_glReadPixels(0, 0, image->width(), image->height(), GL_RGBA,
                          GL_UNSIGNED_BYTE, reinterpret_cast<void*>(pixels));

  image->unref();
  skwasm_postRasterizeResult(this, data.release(), callbackId);
}

void Surface::onRasterizeComplete(uint32_t callbackId, SkData* data) {
  _callbackHandler(callbackId, data, __builtin_wasm_ref_null_extern());
}

// Main thread only
void Surface::onRenderComplete(uint32_t callbackId, SkwasmObject imageBitmap) {
  assert(emscripten_is_main_browser_thread());
  _callbackHandler(callbackId, nullptr, imageBitmap);
}

TextureSourceWrapper::TextureSourceWrapper(unsigned long threadId,
                                           SkwasmObject textureSource)
    : _rasterThreadId(threadId) {
  skwasm_setAssociatedObjectOnThread(_rasterThreadId, this, textureSource);
}

TextureSourceWrapper::~TextureSourceWrapper() {
  skwasm_disposeAssociatedObjectOnThread(_rasterThreadId, this);
}

SkwasmObject TextureSourceWrapper::getTextureSource() {
  return skwasm_getAssociatedObject(this);
}

SKWASM_EXPORT Surface* surface_create() {
  return new Surface();
}

SKWASM_EXPORT unsigned long surface_getThreadId(Surface* surface) {
  return surface->getThreadId();
}

SKWASM_EXPORT void surface_setCallbackHandler(
    Surface* surface,
    Surface::CallbackHandler* callbackHandler) {
  surface->setCallbackHandler(callbackHandler);
}

SKWASM_EXPORT void surface_destroy(Surface* surface) {
  // Dispatch to the worker
  skwasm_dispatchDisposeSurface(surface->getThreadId(), surface);
}

SKWASM_EXPORT void surface_dispose(Surface* surface) {
  // This should be called directly only on the worker
  surface->dispose();
}

SKWASM_EXPORT uint32_t surface_renderPictures(Surface* surface,
                                              SkPicture** pictures,
                                              int count) {
  return surface->renderPictures(pictures, count);
}

SKWASM_EXPORT void surface_renderPicturesOnWorker(Surface* surface,
                                                  sk_sp<SkPicture>* pictures,
                                                  int pictureCount,
                                                  uint32_t callbackId,
                                                  double rasterStart) {
  // This will release the pictures when they leave scope.
  std::unique_ptr<sk_sp<SkPicture>[]> picturesPointer =
      std::unique_ptr<sk_sp<SkPicture>[]>(pictures);
  surface->renderPicturesOnWorker(pictures, pictureCount, callbackId,
                                  rasterStart);
}

SKWASM_EXPORT uint32_t surface_rasterizeImage(Surface* surface,
                                              SkImage* image,
                                              ImageByteFormat format) {
  return surface->rasterizeImage(image, format);
}

SKWASM_EXPORT void surface_rasterizeImageOnWorker(Surface* surface,
                                                  SkImage* image,
                                                  ImageByteFormat format,
                                                  uint32_t callbackId) {
  surface->rasterizeImageOnWorker(image, format, callbackId);
}

// This is used by the skwasm JS support code to call back into C++ when the
// we finish creating the image bitmap, which is an asynchronous operation.
SKWASM_EXPORT void surface_onRenderComplete(Surface* surface,
                                            uint32_t callbackId,
                                            SkwasmObject imageBitmap) {
  surface->onRenderComplete(callbackId, imageBitmap);
}

SKWASM_EXPORT void surface_onRasterizeComplete(Surface* surface,
                                               SkData* data,
                                               uint32_t callbackId) {
  surface->onRasterizeComplete(callbackId, data);
}
