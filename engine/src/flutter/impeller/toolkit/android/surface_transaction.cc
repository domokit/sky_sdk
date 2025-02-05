// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/impeller/toolkit/android/surface_transaction.h"

#include "flutter/impeller/toolkit/android/hardware_buffer.h"
#include "flutter/impeller/toolkit/android/surface_control.h"
#include "impeller/base/validation.h"

namespace impeller::android {

SurfaceTransaction::SurfaceTransaction()
    : transaction_(
          WrappedSurfaceTransaction{GetProcTable().ASurfaceTransaction_create(),
                                    /*owned=*/true}) {}

SurfaceTransaction::SurfaceTransaction(ASurfaceTransaction* transaction)
    : transaction_(WrappedSurfaceTransaction{transaction, /*owned=*/false}) {}

SurfaceTransaction::~SurfaceTransaction() = default;

bool SurfaceTransaction::IsValid() const {
  return transaction_.is_valid();
}

struct TransactionInFlightData {
  SurfaceTransaction::OnCompleteCallback callback;
};

bool SurfaceTransaction::Apply(OnCompleteCallback callback) {
  if (!IsValid()) {
    return false;
  }

  if (!callback) {
    callback = [](auto) {};
  }

  const auto& proc_table = GetProcTable();

  auto data = std::make_unique<TransactionInFlightData>();
  data->callback = callback;
  proc_table.ASurfaceTransaction_setOnComplete(
      transaction_.get().tx,  //
      data.release(),         //
      [](void* context, ASurfaceTransactionStats* stats) -> void {
        auto data = reinterpret_cast<TransactionInFlightData*>(context);
        data->callback(stats);
        delete data;
      });
  // If the transaction was created in Java, then it must be applied in
  // the Java PlatformViewController and not as a part of the engine render
  // loop.
  if (!transaction_.get().owned) {
    transaction_.reset();
    return true;
  }

  proc_table.ASurfaceTransaction_apply(transaction_.get().tx);

  // Transactions may not be applied over and over.
  transaction_.reset();
  return true;
}

bool SurfaceTransaction::SetContents(const SurfaceControl* control,
                                     const HardwareBuffer* buffer,
                                     fml::UniqueFD acquire_fence) {
  if (control == nullptr || buffer == nullptr) {
    VALIDATION_LOG << "Invalid control or buffer.";
    return false;
  }
  GetProcTable().ASurfaceTransaction_setBuffer(
      transaction_.get().tx,                                   //
      control->GetHandle(),                                    //
      buffer->GetHandle(),                                     //
      acquire_fence.is_valid() ? acquire_fence.release() : -1  //
  );
  return true;
}

bool SurfaceTransaction::SetBackgroundColor(const SurfaceControl& control,
                                            const Color& color) {
  if (!IsValid() || !control.IsValid()) {
    return false;
  }
  GetProcTable().ASurfaceTransaction_setColor(transaction_.get().tx,  //
                                              control.GetHandle(),    //
                                              color.red,              //
                                              color.green,            //
                                              color.blue,             //
                                              color.alpha,            //
                                              ADATASPACE_SRGB_LINEAR  //
  );
  return true;
}

bool SurfaceTransaction::SetParent(const SurfaceControl& control,
                                   const SurfaceControl* new_parent) {
  if (!IsValid() || !control.IsValid()) {
    return false;
  }
  if (new_parent && !new_parent->IsValid()) {
    return false;
  }
  GetProcTable().ASurfaceTransaction_reparent(
      transaction_.get().tx,                                     //
      control.GetHandle(),                                       //
      new_parent == nullptr ? nullptr : new_parent->GetHandle()  //
  );
  return true;
}

bool SurfaceTransaction::IsAvailableOnPlatform() {
  return GetProcTable().IsValid() &&
         GetProcTable().ASurfaceTransaction_create.IsAvailable();
}

}  // namespace impeller::android
