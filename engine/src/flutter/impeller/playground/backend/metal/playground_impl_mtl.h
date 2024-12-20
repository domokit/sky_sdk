// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_PLAYGROUND_BACKEND_METAL_PLAYGROUND_IMPL_MTL_H_
#define FLUTTER_IMPELLER_PLAYGROUND_BACKEND_METAL_PLAYGROUND_IMPL_MTL_H_

#include <memory>

#include "flutter/fml/concurrent_message_loop.h"
#include "flutter/fml/synchronization/sync_switch.h"
#include "impeller/playground/playground_impl.h"

namespace impeller {

// Forward declared to avoid objc in a C++ header.
class ContextMTL;

class PlaygroundImplMTL final : public PlaygroundImpl {
 public:
  explicit PlaygroundImplMTL(PlaygroundSwitches switches);

  ~PlaygroundImplMTL();

  fml::Status SetCapabilities(
      const std::shared_ptr<Capabilities>& capabilities) override;

 private:
  struct Data;

  static void DestroyWindowHandle(WindowHandle handle);
  using UniqueHandle = std::unique_ptr<void, decltype(&DestroyWindowHandle)>;
  UniqueHandle handle_;

  // To ensure that ObjC stuff doesn't leak into C++ TUs.
  std::unique_ptr<Data> data_;
  std::shared_ptr<ContextMTL> context_;
  std::shared_ptr<fml::ConcurrentMessageLoop> concurrent_loop_;
  std::shared_ptr<const fml::SyncSwitch> is_gpu_disabled_sync_switch_;

  // |PlaygroundImpl|
  std::shared_ptr<Context> GetContext() const override;

  // |PlaygroundImpl|
  WindowHandle GetWindowHandle() const override;

  // |PlaygroundImpl|
  std::unique_ptr<Surface> AcquireSurfaceFrame(
      std::shared_ptr<Context> context) override;

  PlaygroundImplMTL(const PlaygroundImplMTL&) = delete;

  PlaygroundImplMTL& operator=(const PlaygroundImplMTL&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_PLAYGROUND_BACKEND_METAL_PLAYGROUND_IMPL_MTL_H_
