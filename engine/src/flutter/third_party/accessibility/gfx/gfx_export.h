// Copyright (c) 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_GFX_GFX_EXPORT_H_
#define UI_GFX_GFX_EXPORT_H_

#if defined(COMPONENT_BUILD)
#if defined(WIN32)

#if defined(GFX_IMPLEMENTATION)
#define GFX_EXPORT __declspec(dllexport)
#else
#define GFX_EXPORT __declspec(dllimport)
#endif  // defined(GFX_IMPLEMENTATION)

#else  // defined(WIN32)
#if defined(GFX_IMPLEMENTATION)
#define GFX_EXPORT __attribute__((visibility("default")))
#else
#define GFX_EXPORT
#endif
#endif

#else  // defined(COMPONENT_BUILD)
#define GFX_EXPORT
#endif

#endif  // UI_GFX_GFX_EXPORT_H_
