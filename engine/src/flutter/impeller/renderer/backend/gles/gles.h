// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_GLES_H_
#define FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_GLES_H_

// IWYU pragma: begin_exports
#include "GLES3/gl3.h"

// Defines for extension enums.
#define IMPELLER_GL_CLAMP_TO_BORDER 0x812D
#define IMPELLER_GL_TEXTURE_BORDER_COLOR 0x1004

#define GL_GLEXT_PROTOTYPES
#include "GLES2/gl2ext.h"
// IWYU pragma: end_exports

#endif  // FLUTTER_IMPELLER_RENDERER_BACKEND_GLES_GLES_H_
