// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_EGL_CONTEXT_H_
#define FLUTTER_IMPELLER_TOOLKIT_EGL_CONTEXT_H_

#include <functional>

#include "impeller/base/comparable.h"
#include "impeller/base/thread.h"
#include "impeller/toolkit/egl/egl.h"

namespace impeller {
namespace egl {

class Surface;
class Display;

//------------------------------------------------------------------------------
/// @brief      An instance of an EGL context.
///
///             An EGL context can only be used on a single thread at a given
///             time. A thread can only have a single context current at any
///             given time.
///
///             Context cannot be created directly. Only a valid instance of an
///             egl::Display can create a context.
///
class Context {
 public:
  ~Context();

  //----------------------------------------------------------------------------
  /// @brief      Determines if a valid context could be created. The context
  ///             still needs to be made current on the thread for it to be
  ///             useful.
  ///
  /// @return     True if valid, False otherwise.
  ///
  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Get the underlying handle to the EGL context.
  ///
  /// @return     The handle.
  ///
  const EGLContext& GetHandle() const;

  //----------------------------------------------------------------------------
  /// @brief      Make the context current on the calling thread. It is the
  ///             caller responsibility to ensure that any context previously
  ///             current on the thread must be cleared via `ClearCurrent`.
  ///
  /// @important  The config used to create the surface must match the config
  ///             used to create this context instance.
  ///
  /// @param[in]  surface  The surface to use to make the context current.
  ///
  /// @return     If the context could be made current on the callers thread.
  ///
  bool MakeCurrent(const Surface& surface) const;

  //----------------------------------------------------------------------------
  /// @brief      Clear the thread association of this context.
  ///
  /// @return     If the thread association could be cleared.
  ///
  bool ClearCurrent() const;

  enum class LifecycleEvent {
    kDidMakeCurrent,
    kWillClearCurrent,
  };
  using LifecycleListener = std::function<void(LifecycleEvent)>;
  //----------------------------------------------------------------------------
  /// @brief      Add a listener that gets invoked when the context is made and
  ///             cleared current from the thread. Applications typically use
  ///             this to manage workers that schedule OpenGL API calls that
  ///             need to be careful about the context being current when
  ///             called.
  ///
  /// @param[in]  listener  The listener
  ///
  /// @return     A unique ID for the listener that can used used in
  ///             `RemoveLifecycleListener` to remove a previously added
  ///             listener.
  ///
  std::optional<UniqueID> AddLifecycleListener(
      const LifecycleListener& listener);

  //----------------------------------------------------------------------------
  /// @brief      Remove a previously added context listener.
  ///
  /// @param[in]  id    The identifier obtained via a previous call to
  ///                   `AddLifecycleListener`.
  ///
  /// @return     True if the listener could be removed.
  ///
  bool RemoveLifecycleListener(UniqueID id);

  //----------------------------------------------------------------------------
  /// @return     True if the context is current and attached to any surface,
  ///             False otherwise.
  ///
  bool IsCurrent() const;

 private:
  friend class Display;

  EGLDisplay display_ = EGL_NO_DISPLAY;
  EGLContext context_ = EGL_NO_CONTEXT;
  mutable RWMutex listeners_mutex_;
  std::map<UniqueID, LifecycleListener> listeners_ IPLR_GUARDED_BY(
      listeners_mutex_);

  Context(EGLDisplay display, EGLContext context);

  void DispatchLifecyleEvent(LifecycleEvent event) const;

  Context(const Context&) = delete;

  Context& operator=(const Context&) = delete;
};

}  // namespace egl
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_TOOLKIT_EGL_CONTEXT_H_
