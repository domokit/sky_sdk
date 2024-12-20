// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_BASE_ALLOCATION_H_
#define FLUTTER_IMPELLER_BASE_ALLOCATION_H_

#include <cstdint>
#include <memory>

#include "flutter/fml/mapping.h"
#include "impeller/base/allocation_size.h"

namespace impeller {

//------------------------------------------------------------------------------
/// @brief      Describes an allocation on the heap.
///
///             Managing allocations through this utility makes it harder to
///             miss allocation failures.
///
class Allocation {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Constructs a new zero-sized allocation.
  ///
  Allocation();

  //----------------------------------------------------------------------------
  /// @brief      Destroys the allocation.
  ///
  ~Allocation();

  //----------------------------------------------------------------------------
  /// @brief      Gets the pointer to the start of the allocation.
  ///
  ///             This pointer is only valid till the next call to `Truncate`.
  ///
  /// @return     The pointer to the start of the allocation.
  ///
  uint8_t* GetBuffer() const;

  //----------------------------------------------------------------------------
  /// @brief      Gets the length of the allocation.
  ///
  /// @return     The length.
  ///
  Bytes GetLength() const;

  //----------------------------------------------------------------------------
  /// @brief      Gets the reserved length of the allocation. Calls to truncate
  ///             may be ignored till the length exceeds the reserved length.
  ///
  /// @return     The reserved length.
  ///
  Bytes GetReservedLength() const;

  //----------------------------------------------------------------------------
  /// @brief      Resize the underlying allocation to at least given number of
  ///             bytes.
  ///
  ///             In case of failure, false is returned and the underlying
  ///             allocation remains unchanged.
  ///
  /// @warning    Pointers to buffers obtained via previous calls to `GetBuffer`
  ///             may become invalid at this point.
  ///
  /// @param[in]  length  The length.
  /// @param[in]  npot    Whether to round up the length to the next power of
  ///                     two.
  ///
  /// @return     If the underlying allocation was resized to the new size.
  ///
  [[nodiscard]] bool Truncate(Bytes length, bool npot = true);

  //----------------------------------------------------------------------------
  /// @brief      Gets the next power of two size.
  ///
  /// @param[in]  x     The size.
  ///
  /// @return     The next power of two of x.
  ///
  static uint32_t NextPowerOfTwoSize(uint32_t x);

 private:
  uint8_t* buffer_ = nullptr;
  Bytes length_;
  Bytes reserved_;

  [[nodiscard]] bool Reserve(Bytes reserved);

  [[nodiscard]] bool ReserveNPOT(Bytes reserved);

  Allocation(const Allocation&) = delete;

  Allocation& operator=(const Allocation&) = delete;
};

//------------------------------------------------------------------------------
/// @brief      Creates a mapping with copy of the bytes.
///
/// @param[in]  contents  The contents
/// @param[in]  length    The length
///
/// @return     The new mapping or nullptr if the copy could not be performed.
///
std::shared_ptr<fml::Mapping> CreateMappingWithCopy(const uint8_t* contents,
                                                    Bytes length);

//------------------------------------------------------------------------------
/// @brief      Creates a mapping from allocation.
///
///             No data copy occurs. Only a reference to the underlying
///             allocation is bumped up.
///
///             Changes to the underlying allocation will not be reflected in
///             the mapping and must not change.
///
/// @param[in]  allocation  The allocation.
///
/// @return     A new mapping or nullptr if the argument allocation was invalid.
///
std::shared_ptr<fml::Mapping> CreateMappingFromAllocation(
    const std::shared_ptr<Allocation>& allocation);

//------------------------------------------------------------------------------
/// @brief      Creates a mapping with string data.
///
///             Only a reference to the underlying string is bumped up and the
///             string is not copied.
///
/// @param[in]  string  The string
///
/// @return     A new mapping or nullptr in case of allocation failures.
///
std::shared_ptr<fml::Mapping> CreateMappingWithString(
    std::shared_ptr<const std::string> string);

//------------------------------------------------------------------------------
/// @brief      Creates a mapping with string data.
///
///             The string is copied.
///
/// @param[in]  string  The string
///
/// @return     A new mapping or nullptr in case of allocation failures.
///
std::shared_ptr<fml::Mapping> CreateMappingWithString(std::string string);

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_BASE_ALLOCATION_H_
