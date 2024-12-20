// Copyright 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef UI_ACCESSIBILITY_AX_EXPORT_H_
#define UI_ACCESSIBILITY_AX_EXPORT_H_

// Defines AX_EXPORT so that functionality implemented by the
// ui/accessibility module can be exported to consumers.

#if defined(COMPONENT_BUILD)
#if defined(WIN32)

#if defined(AX_IMPLEMENTATION)
#define AX_EXPORT __declspec(dllexport)
#else
#define AX_EXPORT __declspec(dllimport)
#endif  // defined(ACCESSIBILITY_IMPLEMENTATION)

#else  // defined(WIN32)
#if defined(AX_IMPLEMENTATION)
#define AX_EXPORT __attribute__((visibility("default")))
#else
#define AX_EXPORT
#endif
#endif

#else  // defined(COMPONENT_BUILD)
#define AX_EXPORT
#endif

#endif  // UI_ACCESSIBILITY_AX_EXPORT_H_
