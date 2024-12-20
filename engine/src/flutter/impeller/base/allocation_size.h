// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_BASE_ALLOCATION_SIZE_H_
#define FLUTTER_IMPELLER_BASE_ALLOCATION_SIZE_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace impeller {

enum class FromBytesTag { kFromBytes };

//------------------------------------------------------------------------------
/// @brief      Represents the size of an allocation in different units.
///
///             Refer to the typedefs for Bytes, KiloBytes, MegaBytes,
///             Gigabytes, KibiBytes, MebiBytes, and GibiBytes below when using.
///
///             Storage and all operations are always on unsigned units of
///             bytes.
///
/// @tparam     Period   The number of bytes in 1 unit of the allocation size.
///
template <size_t Period>
class AllocationSize {
 public:
  //----------------------------------------------------------------------------
  /// @brief      Create a zero allocation size.
  ///
  constexpr AllocationSize() = default;

  //----------------------------------------------------------------------------
  /// @brief      Create an allocation size with the amount in the `Period`
  ///             number of bytes.
  ///
  /// @param[in]  size  The size in `Period` number of bytes.
  ///
  template <class T, class = std::enable_if_t<std::is_arithmetic_v<T>>>
  explicit constexpr AllocationSize(T size)
      : bytes_(std::ceil(size) * Period) {}

  //----------------------------------------------------------------------------
  /// @brief      Create an allocation size from another instance with a
  ///             different period.
  ///
  /// @param[in]  other        The other allocation size.
  ///
  /// @tparam     OtherPeriod  The period of the other allocation.
  ///
  template <size_t OtherPeriod>
  explicit constexpr AllocationSize(const AllocationSize<OtherPeriod>& other)
      : bytes_(other.GetByteSize()) {}

  //----------------------------------------------------------------------------
  /// @brief      Create an allocation size with the amount directly specified
  ///             in bytes.
  ///
  /// @param[in]  byte_size  The byte size.
  /// @param[in]  tag        A tag for this constructor.
  ///
  constexpr AllocationSize(uint64_t byte_size, FromBytesTag)
      : bytes_(byte_size) {}

  //----------------------------------------------------------------------------
  /// @return     The byte size.
  ///
  constexpr uint64_t GetByteSize() const { return bytes_; }

  //----------------------------------------------------------------------------
  /// @return     The number of `Periods` of bytes.
  ///
  constexpr double GetSize() const {
    return GetByteSize() / static_cast<double>(Period);
  }

  //----------------------------------------------------------------------------
  /// @brief      Convert the allocation size from one unit to another.
  ///
  ///             Conversions are non-truncating.
  ///
  /// @tparam     AllocationSize  The allocation size to convert to.
  ///
  /// @return     The new allocation size.
  ///
  template <class AllocationSize>
  constexpr AllocationSize ConvertTo() {
    return AllocationSize{GetByteSize(), FromBytesTag::kFromBytes};
  }

  // The following relational operators can be replaced with a defaulted
  // spaceship operator post C++20.

  constexpr bool operator<(const AllocationSize& other) const {
    return bytes_ < other.bytes_;
  }

  constexpr bool operator>(const AllocationSize& other) const {
    return bytes_ > other.bytes_;
  }

  constexpr bool operator>=(const AllocationSize& other) const {
    return bytes_ >= other.bytes_;
  }

  constexpr bool operator<=(const AllocationSize& other) const {
    return bytes_ <= other.bytes_;
  }

  constexpr bool operator==(const AllocationSize& other) const {
    return bytes_ == other.bytes_;
  }

  constexpr bool operator!=(const AllocationSize& other) const {
    return bytes_ != other.bytes_;
  }

  // Explicit casts.

  explicit constexpr operator bool() const { return bytes_ != 0u; }

  // Arithmetic operators (overflows are caller error).

  constexpr AllocationSize operator+(const AllocationSize& other) const {
    return AllocationSize(bytes_ + other.GetByteSize(),
                          FromBytesTag::kFromBytes);
  }

  constexpr AllocationSize operator-(const AllocationSize& other) const {
    return AllocationSize(bytes_ - other.GetByteSize(),
                          FromBytesTag::kFromBytes);
  }

  constexpr AllocationSize& operator+=(const AllocationSize& other) {
    bytes_ += other.GetByteSize();
    return *this;
  }

  constexpr AllocationSize& operator-=(const AllocationSize& other) {
    bytes_ -= other.GetByteSize();
    return *this;
  }

 private:
  uint64_t bytes_ = {};
};

using Bytes = AllocationSize<1u>;

using KiloBytes = AllocationSize<1'000u>;
using MegaBytes = AllocationSize<1'000u * 1'000u>;
using GigaBytes = AllocationSize<1'000u * 1'000u * 1'000u>;

using KibiBytes = AllocationSize<1'024u>;
using MebiBytes = AllocationSize<1'024u * 1'024u>;
using GibiBytes = AllocationSize<1'024u * 1'024u * 1'024u>;

inline namespace allocation_size_literals {

// NOLINTNEXTLINE
constexpr Bytes operator"" _bytes(unsigned long long int size) {
  return Bytes{size};
}

// NOLINTNEXTLINE
constexpr KiloBytes operator"" _kb(unsigned long long int size) {
  return KiloBytes{size};
}

// NOLINTNEXTLINE
constexpr MegaBytes operator"" _mb(unsigned long long int size) {
  return MegaBytes{size};
}

// NOLINTNEXTLINE
constexpr GigaBytes operator"" _gb(unsigned long long int size) {
  return GigaBytes{size};
}

// NOLINTNEXTLINE
constexpr KibiBytes operator"" _kib(unsigned long long int size) {
  return KibiBytes{size};
}

// NOLINTNEXTLINE
constexpr MebiBytes operator"" _mib(unsigned long long int size) {
  return MebiBytes{size};
}

// NOLINTNEXTLINE
constexpr GibiBytes operator"" _gib(unsigned long long int size) {
  return GibiBytes{size};
}

}  // namespace allocation_size_literals

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_BASE_ALLOCATION_SIZE_H_
