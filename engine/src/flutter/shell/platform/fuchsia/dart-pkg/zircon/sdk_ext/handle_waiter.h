// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_SDK_EXT_HANDLE_WAITER_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_SDK_EXT_HANDLE_WAITER_H_

#include <lib/async/cpp/wait.h>
#include <lib/zx/handle.h>

#include "flutter/fml/memory/ref_counted.h"
#include "third_party/tonic/dart_wrappable.h"

namespace tonic {
class DartLibraryNatives;
}  // namespace tonic

namespace zircon {
namespace dart {

class Handle;

class HandleWaiter : public fml::RefCountedThreadSafe<HandleWaiter>,
                     public tonic::DartWrappable {
  DEFINE_WRAPPERTYPEINFO();
  FML_FRIEND_REF_COUNTED_THREAD_SAFE(HandleWaiter);
  FML_FRIEND_MAKE_REF_COUNTED(HandleWaiter);

 public:
  static fml::RefPtr<HandleWaiter> Create(Handle* handle,
                                          zx_signals_t signals,
                                          Dart_Handle callback);

  void Cancel();

  bool is_pending() { return wait_.is_pending(); }

  static void RegisterNatives(tonic::DartLibraryNatives* natives);

 private:
  explicit HandleWaiter(Handle* handle,
                        zx_signals_t signals,
                        Dart_Handle callback);
  ~HandleWaiter();

  void OnWaitComplete(async_dispatcher_t* dispatcher,
                      async::WaitBase* wait,
                      zx_status_t status,
                      const zx_packet_signal_t* signal);

  void RetainDartWrappableReference() const override { AddRef(); }

  void ReleaseDartWrappableReference() const override { Release(); }

  async::WaitMethod<HandleWaiter, &HandleWaiter::OnWaitComplete> wait_;
  Handle* handle_;
  tonic::DartPersistentValue callback_;
};

}  // namespace dart
}  // namespace zircon

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_SDK_EXT_HANDLE_WAITER_H_
