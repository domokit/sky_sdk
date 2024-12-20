// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_DART_VM_LIFECYCLE_H_
#define FLUTTER_RUNTIME_DART_VM_LIFECYCLE_H_

#include <memory>

#include "flutter/fml/macros.h"
#include "flutter/lib/ui/isolate_name_server/isolate_name_server.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/runtime/service_protocol.h"
#include "third_party/dart/runtime/include/dart_tools_api.h"

namespace flutter {

// A strong reference to the Dart VM. There can only be one VM running in the
// process at any given time. A reference to the VM may only be obtained via the
// |Create| method. In case there is already a running instance of the VM in the
// process, a strong reference to that VM is obtained and the arguments to the
// |Create| call ignored. If there is no VM already running in the process, a VM
// is initialized in a thread safe manner and returned to the caller. The VM
// will shutdown only when all callers relinquish their references (by
// collecting their instances of this class).
//
// DartVMRef instances may be created on any thread.
class DartVMRef {
 public:
  [[nodiscard]] static DartVMRef Create(
      const Settings& settings,
      fml::RefPtr<const DartSnapshot> vm_snapshot = nullptr,
      fml::RefPtr<const DartSnapshot> isolate_snapshot = nullptr);

  DartVMRef(const DartVMRef&) = default;

  DartVMRef(DartVMRef&&);

  ~DartVMRef();

  // This is an inherently racy way to check if a VM instance is running and
  // should not be used outside of unit-tests where there is a known threading
  // model.
  static bool IsInstanceRunning();

  static std::shared_ptr<const DartVMData> GetVMData();

  static std::shared_ptr<ServiceProtocol> GetServiceProtocol();

  static std::shared_ptr<IsolateNameServer> GetIsolateNameServer();

  explicit operator bool() const { return static_cast<bool>(vm_); }

  DartVM* get() {
    FML_DCHECK(vm_);
    return vm_.get();
  }

  const DartVM* get() const {
    FML_DCHECK(vm_);
    return vm_.get();
  }

  DartVM* operator->() {
    FML_DCHECK(vm_);
    return vm_.get();
  }

  const DartVM* operator->() const {
    FML_DCHECK(vm_);
    return vm_.get();
  }

  // NOLINTNEXTLINE(google-runtime-operator)
  DartVM* operator&() {
    FML_DCHECK(vm_);
    return vm_.get();
  }

 private:
  friend class DartIsolate;

  std::shared_ptr<DartVM> vm_;

  explicit DartVMRef(std::shared_ptr<DartVM> vm);

  // Only used by Dart Isolate to register itself with the VM.
  static DartVM* GetRunningVM();
};

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_DART_VM_LIFECYCLE_H_
