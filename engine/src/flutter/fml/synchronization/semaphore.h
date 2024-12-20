// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_SYNCHRONIZATION_SEMAPHORE_H_
#define FLUTTER_FML_SYNCHRONIZATION_SEMAPHORE_H_

#include <memory>

#include "flutter/fml/macros.h"

namespace fml {

class PlatformSemaphore;

//------------------------------------------------------------------------------
/// @brief      A traditional counting semaphore.  `Wait`s decrement the counter
///             and `Signal` increments it.
///
///             This is a cross-platform replacement for std::counting_semaphore
///             which is only available since C++20. Once Flutter migrates past
///             that point, this class should become obsolete and must be
///             replaced.
///
class Semaphore {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Initializes the counting semaphore to a specified start count.
  ///
  /// @warning    Callers must check if the handle could be successfully created
  ///             by calling the `IsValid` method. `Wait`s on an invalid
  ///             semaphore will always fail and signals will fail silently.
  ///
  /// @param[in]  count  The starting count of the counting semaphore.
  ///
  explicit Semaphore(uint32_t count);

  //----------------------------------------------------------------------------
  /// @brief      Destroy the counting semaphore.
  ///
  ~Semaphore();

  //----------------------------------------------------------------------------
  /// @brief      Check if the underlying semaphore handle could be created.
  ///             Failure modes are platform specific and may occur due to issue
  ///             like handle exhaustion. All `Wait`s on invalid semaphore
  ///             handles will fail and `Signal` calls will be ignored.
  ///
  /// @return     True if valid, False otherwise.
  ///
  bool IsValid() const;

  //----------------------------------------------------------------------------
  /// @brief      Decrements the count and waits indefinitely if the value is
  ///             less than zero for a `Signal`.
  ///
  /// @return     If the `Wait` call was successful. See `IsValid` for failure.
  ///
  [[nodiscard]] bool Wait();

  //----------------------------------------------------------------------------
  /// @brief      Decrement the counts if it is greater than zero. Returns false
  ///             if the counter is already at zero.
  ///
  /// @warning    False is also returned if the semaphore handle is invalid.
  ///             Which makes doing the validity check before this call doubly
  ///             important.
  ///
  /// @return     If the count could be decremented.
  ///
  [[nodiscard]] bool TryWait();

  //----------------------------------------------------------------------------
  /// @brief      Increment the count by one. Any pending `Wait`s will be
  ///             resolved at this point.
  ///
  void Signal();

 private:
  std::unique_ptr<PlatformSemaphore> impl_;

  FML_DISALLOW_COPY_AND_ASSIGN(Semaphore);
};

}  // namespace fml

#endif  // FLUTTER_FML_SYNCHRONIZATION_SEMAPHORE_H_
