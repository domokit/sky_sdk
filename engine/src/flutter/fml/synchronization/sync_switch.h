// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_SYNCHRONIZATION_SYNC_SWITCH_H_
#define FLUTTER_FML_SYNCHRONIZATION_SYNC_SWITCH_H_

#include <functional>
#include <memory>
#include <shared_mutex>
#include <vector>

#include "flutter/fml/macros.h"

namespace fml {

/// A threadsafe structure that allows you to switch between 2 different
/// execution paths.
///
/// Execution and setting the switch is exclusive, i.e. only one will happen
/// at a time.
class SyncSwitch {
 public:
  /// Observes changes to the SyncSwitch.
  class Observer {
   public:
    virtual ~Observer() = default;
    /// `new_is_disabled` not guaranteed to be the value of the SyncSwitch
    /// during execution, it should be checked with calls to
    /// SyncSwitch::Execute.
    virtual void OnSyncSwitchUpdate(bool new_is_disabled) = 0;
  };

  /// Represents the 2 code paths available when calling |SyncSwitch::Execute|.
  struct Handlers {
    /// Sets the handler that will be executed if the |SyncSwitch| is true.
    Handlers& SetIfTrue(const std::function<void()>& handler);

    /// Sets the handler that will be executed if the |SyncSwitch| is false.
    Handlers& SetIfFalse(const std::function<void()>& handler);

    std::function<void()> true_handler = [] {};
    std::function<void()> false_handler = [] {};
  };

  /// Create a |SyncSwitch| with the specified value.
  ///
  /// @param[in]  value  Default value for the |SyncSwitch|.
  explicit SyncSwitch(bool value = false);

  /// Diverge execution between true and false values of the SyncSwitch.
  ///
  /// This can be called on any thread.  Note that attempting to call
  /// |SetSwitch| inside of the handlers will result in a self deadlock.
  ///
  /// @param[in]  handlers  Called for the correct value of the |SyncSwitch|.
  void Execute(const Handlers& handlers) const;

  /// Set the value of the SyncSwitch.
  ///
  /// This can be called on any thread.
  ///
  /// @param[in]  value  New value for the |SyncSwitch|.
  void SetSwitch(bool value);

  /// Threadsafe.
  void AddObserver(Observer* observer) const;

  /// Threadsafe.
  void RemoveObserver(Observer* observer) const;

 private:
  mutable std::shared_mutex mutex_;
  mutable std::vector<Observer*> observers_;
  bool value_;

  FML_DISALLOW_COPY_AND_ASSIGN(SyncSwitch);
};

}  // namespace fml

#endif  // FLUTTER_FML_SYNCHRONIZATION_SYNC_SWITCH_H_
