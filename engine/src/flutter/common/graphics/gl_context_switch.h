// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_COMMON_GRAPHICS_GL_CONTEXT_SWITCH_H_
#define FLUTTER_COMMON_GRAPHICS_GL_CONTEXT_SWITCH_H_

#include <functional>
#include <memory>
#include <vector>

#include "flutter/fml/logging.h"
#include "flutter/fml/macros.h"

namespace flutter {

// This interface represents a gl context that can be switched by
// |GLContextSwitch|.
//
// The implementation should wrap a "Context" object inside this class. For
// example, in iOS while using a GL rendering surface, the implementation should
// wrap an |EAGLContext|.
class SwitchableGLContext {
 public:
  SwitchableGLContext();

  virtual ~SwitchableGLContext();

  // Implement this to set the context wrapped by this |SwitchableGLContext|
  // object to the current context.
  virtual bool SetCurrent() = 0;

  // Implement this to remove the context wrapped by this |SwitchableGLContext|
  // object from current context;
  virtual bool RemoveCurrent() = 0;

  FML_DISALLOW_COPY_AND_ASSIGN(SwitchableGLContext);
};

// Represents the result of setting a gl context.
//
// This class exists because context results are used in places applies to all
// the platforms. On certain platforms(for example lower end iOS devices that
// uses gl), a |GLContextSwitch| is required to protect flutter's gl contect
// from being polluted by other programs(embedded platform views). A
// |GLContextSwitch| is a subclass of |GLContextResult|, which can be returned
// on platforms that requires context switching. A |GLContextDefaultResult| is
// also a subclass of |GLContextResult|, which can be returned on platforms
// that doesn't require context switching.
class GLContextResult {
 public:
  GLContextResult();
  virtual ~GLContextResult();

  //----------------------------------------------------------------------------
  // Returns true if the gl context is set successfully.
  bool GetResult();

 protected:
  explicit GLContextResult(bool static_result);
  bool result_;

  FML_DISALLOW_COPY_AND_ASSIGN(GLContextResult);
};

//------------------------------------------------------------------------------
/// The default implementation of |GLContextResult|.
///
/// Use this class on platforms that doesn't require gl context switching.
/// * See also |GLContextSwitch| if the platform requires gl context switching.
class GLContextDefaultResult : public GLContextResult {
 public:
  //----------------------------------------------------------------------------
  /// Constructs a |GLContextDefaultResult| with a static result.
  ///
  /// Used this on platforms that doesn't require gl context switching. (For
  /// example, metal on iOS)
  ///
  /// @param  static_result a static value that will be returned from
  /// |GetResult|
  explicit GLContextDefaultResult(bool static_result);

  ~GLContextDefaultResult() override;

  FML_DISALLOW_COPY_AND_ASSIGN(GLContextDefaultResult);
};

//------------------------------------------------------------------------------
/// Switches the gl context to the a context that is passed in the
/// constructor.
///
/// In destruction, it should restore the current context to what was
/// before the construction of this switch.
class GLContextSwitch final : public GLContextResult {
 public:
  //----------------------------------------------------------------------------
  /// Constructs a |GLContextSwitch|.
  ///
  /// @param  context The context that is going to be set as the current
  /// context. The |GLContextSwitch| should not outlive the owner of the gl
  /// context wrapped inside the `context`.
  explicit GLContextSwitch(std::unique_ptr<SwitchableGLContext> context);

  ~GLContextSwitch() override;

 private:
  std::unique_ptr<SwitchableGLContext> context_;

  FML_DISALLOW_COPY_AND_ASSIGN(GLContextSwitch);
};

}  // namespace flutter

#endif  // FLUTTER_COMMON_GRAPHICS_GL_CONTEXT_SWITCH_H_
