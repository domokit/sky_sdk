// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_ISOLATE_CONFIGURATION_H_
#define FLUTTER_RUNTIME_ISOLATE_CONFIGURATION_H_

#include <future>
#include <memory>

#include "flutter/assets/asset_manager.h"
#include "flutter/assets/asset_resolver.h"
#include "flutter/common/settings.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/mapping.h"
#include "flutter/runtime/dart_isolate.h"

namespace flutter {

/// Describes whether the isolate is part of a group or not.
///
/// If the isolate is part of a group, it avoids reloading the kernel snapshot.
enum class IsolateLaunchType {
  /// The isolate is launched as a solo isolate or to start a new group.
  kNewGroup,
  /// The isolate is launched as part of a group, and avoids reloading the
  /// kernel snapshot.
  kExistingGroup,
};

//------------------------------------------------------------------------------
/// @brief      An isolate configuration is a collection of snapshots and asset
///             managers that the engine will use to configure the isolate
///             before invoking its root entrypoint. The set of snapshots must
///             be sufficient for the engine to move the isolate from the
///             |DartIsolate::Phase::LibrariesSetup| phase to the
///             |DartIsolate::Phase::Ready| phase. Note that the isolate
///             configuration will not be collected till the isolate tied to the
///             configuration as well as any and all child isolates of that
///             isolate are collected. The engine may ask the configuration to
///             prepare multiple isolates. All subclasses of this class must be
///             thread safe as the configuration may be created, collected and
///             used on multiple threads. Usually these threads are engine or VM
///             managed so care must be taken to ensure that subclasses do not
///             reference any thread local state.
///
class IsolateConfiguration {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Attempts to infer the isolate configuration from the
  ///             `Settings` object. If the VM is configured for AOT mode,
  ///             snapshot resolution is attempted with predefined symbols
  ///             present in the currently loaded process. In JIT mode, Dart
  ///             kernel file resolution is attempted in the assets directory.
  ///             If an IO worker is specified, snapshot resolution may be
  ///             attempted on the serial worker task runner. The worker task
  ///             runner thread must remain valid and running till after the
  ///             shell associated with the engine used to launch the isolate
  ///             for which this run configuration is used is collected.
  ///
  /// @param[in]  settings       The settings
  /// @param[in]  asset_manager  The optional asset manager. This is used when
  ///                            using the legacy settings fields that specify
  ///                            the asset by name instead of a mappings
  ///                            callback.
  /// @param[in]  io_worker      An optional IO worker. Specify `nullptr` if a
  ///                            worker should not be used or one is not
  ///                            available.
  /// @param[in]  launch_type    Whether the isolate is launching to form a new
  ///                            group or as part of an existing group. If it is
  ///                            part of an existing group, the isolate will
  ///                            reuse resources if it can.
  ///
  /// @return     An isolate configuration if one can be inferred from the
  ///             settings. If not, returns `nullptr`.
  ///
  [[nodiscard]] static std::unique_ptr<IsolateConfiguration> InferFromSettings(
      const Settings& settings,
      const std::shared_ptr<AssetManager>& asset_manager = nullptr,
      const fml::RefPtr<fml::TaskRunner>& io_worker = nullptr,
      IsolateLaunchType launch_type = IsolateLaunchType::kNewGroup);

  //----------------------------------------------------------------------------
  /// @brief      Creates an AOT isolate configuration using snapshot symbols
  ///             present in the currently loaded process. These symbols need to
  ///             be given to the Dart VM on bootstrap and hence have already
  ///             been resolved.
  ///
  /// @return     An AOT isolate configuration.
  ///
  static std::unique_ptr<IsolateConfiguration> CreateForAppSnapshot();

  //----------------------------------------------------------------------------
  /// @brief      Creates a JIT isolate configuration using a list of futures to
  ///             snapshots defining the ready isolate state. In environments
  ///             where snapshot resolution is extremely expensive, embedders
  ///             attempt to resolve snapshots on worker thread(s) and return
  ///             the future of the promise of snapshot resolution to this
  ///             method. That way, snapshot resolution begins well before
  ///             isolate launch is attempted by the engine.
  ///
  /// @param[in]  kernel_pieces  The list of futures to Dart kernel snapshots.
  ///
  /// @return     A JIT isolate configuration.
  ///
  static std::unique_ptr<IsolateConfiguration> CreateForKernelList(
      std::vector<std::future<std::unique_ptr<const fml::Mapping>>>
          kernel_pieces);

  //----------------------------------------------------------------------------
  /// @brief      Creates a JIT isolate configuration using the specified
  ///             snapshot. This is a convenience method for the
  ///             `CreateForKernelList` method that takes a list of futures to
  ///             Dart kernel snapshots.
  ///
  /// @see        CreateForKernelList()
  ///
  /// @param[in]  kernel  The kernel snapshot.
  ///
  /// @return     A JIT isolate configuration.
  ///
  static std::unique_ptr<IsolateConfiguration> CreateForKernel(
      std::unique_ptr<const fml::Mapping> kernel);

  //----------------------------------------------------------------------------
  /// @brief      Creates a JIT isolate configuration using the specified
  ///              snapshots. This is a convenience method for the
  ///             `CreateForKernelList` method that takes a list of futures to
  ///             Dart kernel snapshots.
  ///
  /// @see        CreateForKernelList()
  ///
  /// @param[in]  kernel_pieces  The kernel pieces
  ///
  /// @return     { description_of_the_return_value }
  ///
  static std::unique_ptr<IsolateConfiguration> CreateForKernelList(
      std::vector<std::unique_ptr<const fml::Mapping>> kernel_pieces);

  //----------------------------------------------------------------------------
  /// @brief      Create an isolate configuration. This has no threading
  /// restrictions.
  ///
  IsolateConfiguration();

  //----------------------------------------------------------------------------
  /// @brief      Destroys an isolate configuration. This has no threading
  ///             restrictions and may be collection of configurations may occur
  ///             on any thread (and usually happens on an internal VM managed
  ///             thread pool thread).
  ///
  virtual ~IsolateConfiguration();

  //----------------------------------------------------------------------------
  /// @brief      When an isolate is created and sufficiently initialized to
  ///             move it into the `DartIsolate::Phase::LibrariesSetup` phase,
  ///             this method is invoked on the isolate to then move the isolate
  ///             into the `DartIsolate::Phase::Ready` phase. Then isolate's
  ///             main entrypoint is then invoked to move it into the
  ///             `DartIsolate::Phase::Running` phase. This method will be
  ///             called each time the root isolate is launched (which may be
  ///             multiple times in cold-restart scenarios) as well as one each
  ///             for any child isolates referenced by that isolate.
  ///
  /// @param      isolate  The isolate which is already in the
  ///                      `DartIsolate::Phase::LibrariesSetup` phase.
  ///
  /// @return     Returns true if the isolate could be configured. Unless this
  ///             returns true, the engine will not move the isolate to the
  ///             `DartIsolate::Phase::Ready` phase for subsequent run.
  ///
  [[nodiscard]] bool PrepareIsolate(DartIsolate& isolate);

  virtual bool IsNullSafetyEnabled(const DartSnapshot& snapshot) = 0;

 protected:
  virtual bool DoPrepareIsolate(DartIsolate& isolate) = 0;

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(IsolateConfiguration);
};

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_ISOLATE_CONFIGURATION_H_
