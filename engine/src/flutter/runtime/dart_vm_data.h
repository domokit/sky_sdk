// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_DART_VM_DATA_H_
#define FLUTTER_RUNTIME_DART_VM_DATA_H_

#include "flutter/fml/macros.h"
#include "flutter/runtime/dart_snapshot.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Provides thread-safe access to data that is necessary to
///             bootstrap a new Dart VM instance. All snapshots referenced by
///             this object are read-only.
///
class DartVMData {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Creates a new instance of `DartVMData`. Both the VM and
  ///             isolate snapshot members are optional and may be `nullptr`. If
  ///             `nullptr`, the snapshot resolvers present in the settings
  ///             object are used to infer the snapshots. If the snapshots
  ///             cannot be inferred from the settings object, this method
  ///             return `nullptr`.
  ///
  /// @param[in]  settings          The settings used to infer the VM and
  ///                               isolate snapshots if they are not provided
  ///                               directly.
  /// @param[in]  vm_snapshot       The VM snapshot or `nullptr`.
  /// @param[in]  isolate_snapshot  The isolate snapshot or `nullptr`.
  ///
  /// @return     A new instance of VM data that can be used to bootstrap a Dart
  ///             VM. `nullptr` if the snapshots are not provided and cannot be
  ///             inferred from the settings object.
  ///
  static std::shared_ptr<const DartVMData> Create(
      const Settings& settings,
      fml::RefPtr<const DartSnapshot> vm_snapshot,
      fml::RefPtr<const DartSnapshot> isolate_snapshot);

  //----------------------------------------------------------------------------
  /// @brief      Collect the DartVMData instance.
  ///
  ~DartVMData();

  //----------------------------------------------------------------------------
  /// @brief      The settings object from which the Dart snapshots were
  ///             inferred.
  ///
  /// @return     The settings.
  ///
  const Settings& GetSettings() const;

  //----------------------------------------------------------------------------
  /// @brief      Gets the VM snapshot. This can be in the call to bootstrap
  ///             the Dart VM via `Dart_Initialize`.
  ///
  /// @return     The VM snapshot.
  ///
  const DartSnapshot& GetVMSnapshot() const;

  //----------------------------------------------------------------------------
  /// @brief      Get the isolate snapshot necessary to launch isolates in the
  ///             Dart VM. The Dart VM instance in which these isolates are
  ///             launched must be the same as the VM created using snapshot
  ///             accessed via `GetVMSnapshot`.
  ///
  /// @return     The isolate snapshot.
  ///
  fml::RefPtr<const DartSnapshot> GetIsolateSnapshot() const;

  //----------------------------------------------------------------------------
  /// @brief      Get the isolate snapshot used to launch the service isolate
  ///             in the Dart VM.
  ///
  /// @return     The service isolate snapshot.
  ///
  fml::RefPtr<const DartSnapshot> GetServiceIsolateSnapshot() const;

  //----------------------------------------------------------------------------
  /// @brief      Returns whether the service isolate snapshot requires null
  ///             safety in the Dart_IsolateFlags used to create the isolate.
  ///
  /// @return     True if the snapshot requires null safety.
  ///
  bool GetServiceIsolateSnapshotNullSafety() const;

 private:
  const Settings settings_;
  const fml::RefPtr<const DartSnapshot> vm_snapshot_;
  const fml::RefPtr<const DartSnapshot> isolate_snapshot_;
  const fml::RefPtr<const DartSnapshot> service_isolate_snapshot_;

  DartVMData(const Settings& settings,
             fml::RefPtr<const DartSnapshot> vm_snapshot,
             fml::RefPtr<const DartSnapshot> isolate_snapshot,
             fml::RefPtr<const DartSnapshot> service_isolate_snapshot);

  FML_DISALLOW_COPY_AND_ASSIGN(DartVMData);
};

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_DART_VM_DATA_H_
