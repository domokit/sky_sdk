// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_DART_VM_H_
#define FLUTTER_RUNTIME_DART_VM_H_

#include <memory>
#include <string>

#include "flutter/common/settings.h"
#include "flutter/fml/build_config.h"
#include "flutter/fml/closure.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/memory/ref_counted.h"
#include "flutter/fml/memory/ref_ptr.h"
#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/message_loop.h"
#include "flutter/lib/ui/isolate_name_server/isolate_name_server.h"
#include "flutter/runtime/dart_isolate.h"
#include "flutter/runtime/dart_snapshot.h"
#include "flutter/runtime/dart_vm_data.h"
#include "flutter/runtime/service_protocol.h"
#include "flutter/runtime/skia_concurrent_executor.h"
#include "third_party/dart/runtime/include/dart_api.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Describes a running instance of the Dart VM. There may only be
///             one running instance of the Dart VM in the process at any given
///             time. The Dart VM may be created and destroyed on any thread.
///             Typically, the first Flutter shell instance running in the
///             process bootstraps the Dart VM in the process as it starts up.
///             This cost is borne on the platform task runner of that first
///             Flutter shell. When the last Flutter shell instance is
///             destroyed, the VM is destroyed as well if all shell instances
///             were launched with the `Settings::leak_vm` flag set to false. If
///             there is any shell launch in the process with `leak_vm` set to
///             true, the VM is never shut down in the process. When the VM is
///             shutdown, the cost of the shutdown is borne on the platform task
///             runner of the last shell instance to be shut down.
///
///             Due to threading considerations, callers may never create an
///             instance of the DartVM directly. All constructors to the DartVM
///             are private. Instead, all callers that need a running VM
///             reference need to access it via the `DartVMRef::Create` call.
///             This call returns a strong reference to the running VM if one
///             exists in the process already. If a running VM instance is not
///             available in the process, a new instance is created and a strong
///             reference returned to the callers. The DartVMRef::Create call
///             ensures that there are no data races during the creation or
///             shutdown of a Dart VM (since a VM may be created and destroyed
///             on any thread). Due to this behavior, all callers needing a
///             running VM instance must provide snapshots and VM settings
///             necessary to create a VM (even if they end up not being used).
///
///             In a running VM instance, the service isolate is launched by
///             default if the VM is configured to do so. All root isolates must
///             be launched and referenced explicitly.
class DartVM {
 public:
  ~DartVM();

  //----------------------------------------------------------------------------
  /// @brief      Checks if VM instances in the process can run precompiled
  ///             code. This call can be made at any time and does not depend on
  ///             a running VM instance. There are no threading restrictions.
  ///
  /// @return     If VM instances in the process run precompiled code.
  ///
  static bool IsRunningPrecompiledCode();

  //----------------------------------------------------------------------------
  /// @brief      The number of times the VM has been launched in the process.
  ///             This call is inherently racy because the VM could be in the
  ///             process of starting up on another thread between the time the
  ///             caller makes this call and uses to result. For this purpose,
  ///             this call is only meant to be used as a debugging aid and
  ///             primarily only used in tests where the threading model is
  ///             consistent.
  ///
  /// @return     The number of times the VM has been launched.
  ///
  static size_t GetVMLaunchCount();

  //----------------------------------------------------------------------------
  /// @brief      The settings used to launch the running VM instance.
  ///
  /// @attention  Even though all callers that need to acquire a strong
  ///             reference to a VM need to provide a valid settings object, the
  ///             VM will only reference the settings used by the first caller
  ///             that bootstraps the VM in the process.
  ///
  /// @return     A valid setting object.
  ///
  const Settings& GetSettings() const;

  //----------------------------------------------------------------------------
  /// @brief      The VM and isolate snapshots used by this running Dart VM
  ///             instance.
  ///
  /// @return     A valid VM data instance.
  ///
  std::shared_ptr<const DartVMData> GetVMData() const;

  //----------------------------------------------------------------------------
  /// @brief      The service protocol instance associated with this running
  ///             Dart VM instance. This object manages native handlers for
  ///             engine vended service protocol methods.
  ///
  /// @return     The service protocol for this Dart VM instance.
  ///
  std::shared_ptr<ServiceProtocol> GetServiceProtocol() const;

  //----------------------------------------------------------------------------
  /// @brief      The isolate name server for this running VM instance. The
  ///             isolate name server maps names (strings) to Dart ports.
  ///             Running isolates can discover and communicate with each other
  ///             by advertising and resolving ports at well known names.
  ///
  /// @return     The isolate name server.
  ///
  std::shared_ptr<IsolateNameServer> GetIsolateNameServer() const;

  //----------------------------------------------------------------------------
  /// @brief      The task runner whose tasks may be executed concurrently on a
  ///             pool of worker threads. All subsystems within a running shell
  ///             instance use this worker pool for their concurrent tasks. This
  ///             also means that the concurrent worker pool may service tasks
  ///             from multiple shell instances. The number of workers in a
  ///             concurrent worker pool depends on the hardware concurrency
  ///             of the target device (usually equal to the number of logical
  ///             CPU cores).
  ///
  ///
  /// @attention  Even though concurrent task queue is associated with a running
  ///             Dart VM instance, the worker pool used by the Flutter engine
  ///             is NOT shared with the Dart VM internal worker pool. The
  ///             presence of this worker pool as member of the Dart VM is
  ///             merely to utilize the strong thread safety guarantees around
  ///             Dart VM lifecycle for the lifecycle of the concurrent worker
  ///             pool as well.
  ///
  /// @return     The task runner for the concurrent worker thread pool.
  ///
  std::shared_ptr<fml::ConcurrentTaskRunner> GetConcurrentWorkerTaskRunner()
      const;

  //----------------------------------------------------------------------------
  /// @brief      The concurrent message loop hosts threads that are used by the
  ///             engine to perform tasks long running background tasks.
  ///             Typically, to post tasks to this message loop, the
  ///             `GetConcurrentWorkerTaskRunner` method may be used.
  ///
  /// @see        GetConcurrentWorkerTaskRunner
  ///
  /// @return     The concurrent message loop used by this running Dart VM
  ///             instance.
  ///
  std::shared_ptr<fml::ConcurrentMessageLoop> GetConcurrentMessageLoop();

 private:
  const Settings settings_;
  std::shared_ptr<fml::ConcurrentMessageLoop> concurrent_message_loop_;
  SkiaConcurrentExecutor skia_concurrent_executor_;
  std::shared_ptr<const DartVMData> vm_data_;
  const std::shared_ptr<IsolateNameServer> isolate_name_server_;
  const std::shared_ptr<ServiceProtocol> service_protocol_;

  friend class DartVMRef;
  friend class DartIsolate;

  static std::shared_ptr<DartVM> Create(
      const Settings& settings,
      fml::RefPtr<const DartSnapshot> vm_snapshot,
      fml::RefPtr<const DartSnapshot> isolate_snapshot,
      std::shared_ptr<IsolateNameServer> isolate_name_server);

  DartVM(const std::shared_ptr<const DartVMData>& data,
         std::shared_ptr<IsolateNameServer> isolate_name_server);

  FML_DISALLOW_COPY_AND_ASSIGN(DartVM);
};

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_DART_VM_H_
