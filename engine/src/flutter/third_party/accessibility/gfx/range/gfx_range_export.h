// Copyright 2016 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef GFX_RANGE_EXPORT_H_
#define GFX_RANGE_EXPORT_H_

#if defined(COMPONENT_BUILD)
#if defined(WIN32)

#if defined(GFX_RANGE_IMPLEMENTATION)
#define GFX_RANGE_EXPORT __declspec(dllexport)
#else
#define GFX_RANGE_EXPORT __declspec(dllimport)
#endif  // defined(GFX_RANGE_IMPLEMENTATION)

#else  // defined(WIN32)
#if defined(GFX_RANGE_IMPLEMENTATION)
#define GFX_RANGE_EXPORT __attribute__((visibility("default")))
#else
#define GFX_RANGE_EXPORT
#endif
#endif

#else  // defined(COMPONENT_BUILD)
#define GFX_RANGE_EXPORT
#endif

#endif  // GFX_RANGE_EXPORT_H_
