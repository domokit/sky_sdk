// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_WEB_UI_SKWASM_SKWASM_SUPPORT_H_
#define FLUTTER_LIB_WEB_UI_SKWASM_SKWASM_SUPPORT_H_

#include <emscripten/threading.h>
#include <cinttypes>
#include "surface.h"
#include "third_party/skia/include/core/SkPicture.h"

using SkwasmObject = __externref_t;

extern "C" {
extern void skwasm_setAssociatedObjectOnThread(unsigned long threadId,
                                               void* pointer,
                                               SkwasmObject object);
extern SkwasmObject skwasm_getAssociatedObject(void* pointer);
extern void skwasm_disposeAssociatedObjectOnThread(unsigned long threadId,
                                                   void* pointer);
extern void skwasm_connectThread(pthread_t threadId);
extern void skwasm_dispatchRenderPictures(unsigned long threadId,
                                          Skwasm::Surface* surface,
                                          sk_sp<SkPicture>* pictures,
                                          int count,
                                          uint32_t callbackId);
extern uint32_t skwasm_createOffscreenCanvas(int width, int height);
extern void skwasm_resizeCanvas(uint32_t contextHandle, int width, int height);
extern SkwasmObject skwasm_captureImageBitmap(uint32_t contextHandle,
                                              int width,
                                              int height,
                                              SkwasmObject imagePromises);
extern void skwasm_resolveAndPostImages(Skwasm::Surface* surface,
                                        SkwasmObject imagePromises,
                                        double rasterStart,
                                        uint32_t callbackId);
extern unsigned int skwasm_createGlTextureFromTextureSource(
    SkwasmObject textureSource,
    int width,
    int height);
extern void skwasm_dispatchDisposeSurface(unsigned long threadId,
                                          Skwasm::Surface* surface);
extern void skwasm_dispatchRasterizeImage(unsigned long threadId,
                                          Skwasm::Surface* surface,
                                          SkImage* image,
                                          Skwasm::ImageByteFormat format,
                                          uint32_t callbackId);
extern void skwasm_postRasterizeResult(Skwasm::Surface* surface,
                                       SkData* data,
                                       uint32_t callbackId);
}

#endif  // FLUTTER_LIB_WEB_UI_SKWASM_SKWASM_SUPPORT_H_
