// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/common/graphics/gl_context_switch.h"

namespace flutter {

SwitchableGLContext::SwitchableGLContext() = default;

SwitchableGLContext::~SwitchableGLContext() = default;

GLContextResult::GLContextResult() = default;

GLContextResult::~GLContextResult() = default;

GLContextResult::GLContextResult(bool static_result) : result_(static_result){};

bool GLContextResult::GetResult() {
  return result_;
};

GLContextDefaultResult::GLContextDefaultResult(bool static_result)
    : GLContextResult(static_result){};

GLContextDefaultResult::~GLContextDefaultResult() = default;

GLContextSwitch::GLContextSwitch(std::unique_ptr<SwitchableGLContext> context)
    : context_(std::move(context)) {
  FML_CHECK(context_ != nullptr);
  result_ = context_->SetCurrent();
};

GLContextSwitch::~GLContextSwitch() {
  context_->RemoveCurrent();
};

}  // namespace flutter
