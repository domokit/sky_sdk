// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_TOOLKIT_ANDROID_SURFACE_TRANSACTION_H_
#define FLUTTER_IMPELLER_TOOLKIT_ANDROID_SURFACE_TRANSACTION_H_

#include <functional>
#include <map>

#include "flutter/fml/unique_fd.h"
#include "flutter/fml/unique_object.h"
#include "impeller/geometry/color.h"
#include "impeller/toolkit/android/proc_table.h"

namespace impeller::android {

class SurfaceControl;
class HardwareBuffer;

//------------------------------------------------------------------------------
/// @brief      A wrapper for ASurfaceTransaction.
///             https://developer.android.com/ndk/reference/group/native-activity#asurfacetransaction
///
///             A surface transaction is a collection of updates to the
///             hierarchy of surfaces (represented by `ASurfaceControl`
///             instances) that are applied atomically in the compositor.
///
///             This wrapper is only available on Android API 29 and above.
///
/// @note       Transactions should be short lived objects (create, apply,
///             collect). But, if these are used on multiple threads, they must
///             be externally synchronized.
///
class SurfaceTransaction {
 public:
  //----------------------------------------------------------------------------
  /// @return     `true` if any surface transactions can be created on this
  ///             platform.
  ///
  static bool IsAvailableOnPlatform();

  SurfaceTransaction();

  ~SurfaceTransaction();

  SurfaceTransaction(const SurfaceTransaction&) = delete;

  SurfaceTransaction& operator=(const SurfaceTransaction&) = delete;

  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Encodes that the updated contents of a surface control are
  ///             specified by the given hardware buffer. The update will not be
  ///             committed till the call to `Apply` however.
  ///
  /// @see        `SurfaceTransaction::Apply`.
  ///
  /// @param[in]  control         The control.
  /// @param[in]  buffer          The hardware buffer.
  /// @param[in]  acquire_fence   The fence to wait on before setting the
  ///                             contents.
  ///
  /// @return     If the update was encoded in the transaction.
  ///
  [[nodiscard]] bool SetContents(const SurfaceControl* control,
                                 const HardwareBuffer* buffer,
                                 fml::UniqueFD acquire_fence = {});

  //----------------------------------------------------------------------------
  /// @brief      Encodes the updated background color of the surface control.
  ///             The update will not be committed till the call to `Apply`
  ///             however.
  ///
  /// @see        `SurfaceTransaction::Apply`.
  ///
  /// @param[in]  control  The control
  /// @param[in]  color    The color
  ///
  /// @return     `true` if the background control will be set when transaction
  ///              is applied.
  ///
  [[nodiscard]] bool SetBackgroundColor(const SurfaceControl& control,
                                        const Color& color);

  using OnCompleteCallback = std::function<void(ASurfaceTransactionStats*)>;

  //----------------------------------------------------------------------------
  /// @brief      Applies the updated encoded in the transaction and invokes the
  ///             callback when the updated are complete.
  ///
  /// @warning    The callback will be invoked on a system managed thread.
  ///
  /// @note       It is fine to immediately destroy the transaction after the
  ///             call to apply. It is not necessary to wait for transaction
  ///             completion to collect the transaction handle.
  ///
  /// @param[in]  callback  The callback
  ///
  /// @return     `true` if the surface transaction was applied. `true` does not
  ///             indicate the application was completed however. Only the
  ///             invocation of the callback denotes transaction completion.
  ///
  [[nodiscard]] bool Apply(OnCompleteCallback callback = nullptr);

  //----------------------------------------------------------------------------
  /// @brief      Set the new parent control of the given control. If the new
  ///             parent is null, it is removed from the control hierarchy.
  ///
  /// @param[in]  control     The control
  /// @param[in]  new_parent  The new parent
  ///
  /// @return     `true` if the control will be re-parented when the transaction
  ///             is applied.
  ///
  [[nodiscard]] bool SetParent(const SurfaceControl& control,
                               const SurfaceControl* new_parent = nullptr);

 private:
  struct UniqueASurfaceTransactionTraits {
    static ASurfaceTransaction* InvalidValue() { return nullptr; }

    static bool IsValid(ASurfaceTransaction* value) {
      return value != InvalidValue();
    }

    static void Free(ASurfaceTransaction* value) {
      GetProcTable().ASurfaceTransaction_delete(value);
    }
  };

  fml::UniqueObject<ASurfaceTransaction*, UniqueASurfaceTransactionTraits>
      transaction_;
};

}  // namespace impeller::android

#endif  // FLUTTER_IMPELLER_TOOLKIT_ANDROID_SURFACE_TRANSACTION_H_
