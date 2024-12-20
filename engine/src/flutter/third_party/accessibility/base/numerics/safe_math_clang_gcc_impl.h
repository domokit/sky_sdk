// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_NUMERICS_SAFE_MATH_CLANG_GCC_IMPL_H_
#define BASE_NUMERICS_SAFE_MATH_CLANG_GCC_IMPL_H_

#include <cassert>
#include <limits>
#include <type_traits>

#include "base/numerics/safe_conversions.h"

#if !defined(__native_client__) && (defined(__ARMEL__) || defined(__arch64__))
#include "base/numerics/safe_math_arm_impl.h"
#define BASE_HAS_ASSEMBLER_SAFE_MATH (1)
#else
#define BASE_HAS_ASSEMBLER_SAFE_MATH (0)
#endif

namespace base {
namespace internal {

// These are the non-functioning boilerplate implementations of the optimized
// safe math routines.
#if !BASE_HAS_ASSEMBLER_SAFE_MATH
template <typename T, typename U>
struct CheckedMulFastAsmOp {
  static const bool is_supported = false;
  template <typename V>
  static constexpr bool Do(T, U, V*) {
    // Force a compile failure if instantiated.
    return CheckOnFailure::template HandleFailure<bool>();
  }
};

template <typename T, typename U>
struct ClampedAddFastAsmOp {
  static const bool is_supported = false;
  template <typename V>
  static constexpr V Do(T, U) {
    // Force a compile failure if instantiated.
    return CheckOnFailure::template HandleFailure<V>();
  }
};

template <typename T, typename U>
struct ClampedSubFastAsmOp {
  static const bool is_supported = false;
  template <typename V>
  static constexpr V Do(T, U) {
    // Force a compile failure if instantiated.
    return CheckOnFailure::template HandleFailure<V>();
  }
};

template <typename T, typename U>
struct ClampedMulFastAsmOp {
  static const bool is_supported = false;
  template <typename V>
  static constexpr V Do(T, U) {
    // Force a compile failure if instantiated.
    return CheckOnFailure::template HandleFailure<V>();
  }
};
#endif  // BASE_HAS_ASSEMBLER_SAFE_MATH
#undef BASE_HAS_ASSEMBLER_SAFE_MATH

template <typename T, typename U>
struct CheckedAddFastOp {
  static const bool is_supported = true;
  template <typename V>
  __attribute__((always_inline)) static constexpr bool Do(T x, U y, V* result) {
    return !__builtin_add_overflow(x, y, result);
  }
};

template <typename T, typename U>
struct CheckedSubFastOp {
  static const bool is_supported = true;
  template <typename V>
  __attribute__((always_inline)) static constexpr bool Do(T x, U y, V* result) {
    return !__builtin_sub_overflow(x, y, result);
  }
};

template <typename T, typename U>
struct CheckedMulFastOp {
#if defined(__clang__)
  // TODO(jschuh): Get the Clang runtime library issues sorted out so we can
  // support full-width, mixed-sign multiply builtins.
  // https://crbug.com/613003
  // We can support intptr_t, uintptr_t, or a smaller common type.
  static const bool is_supported =
      (IsTypeInRangeForNumericType<intptr_t, T>::value &&
       IsTypeInRangeForNumericType<intptr_t, U>::value) ||
      (IsTypeInRangeForNumericType<uintptr_t, T>::value &&
       IsTypeInRangeForNumericType<uintptr_t, U>::value);
#else
  static const bool is_supported = true;
#endif
  template <typename V>
  __attribute__((always_inline)) static constexpr bool Do(T x, U y, V* result) {
    return CheckedMulFastAsmOp<T, U>::is_supported
               ? CheckedMulFastAsmOp<T, U>::Do(x, y, result)
               : !__builtin_mul_overflow(x, y, result);
  }
};

template <typename T, typename U>
struct ClampedAddFastOp {
  static const bool is_supported = ClampedAddFastAsmOp<T, U>::is_supported;
  template <typename V>
  __attribute__((always_inline)) static V Do(T x, U y) {
    return ClampedAddFastAsmOp<T, U>::template Do<V>(x, y);
  }
};

template <typename T, typename U>
struct ClampedSubFastOp {
  static const bool is_supported = ClampedSubFastAsmOp<T, U>::is_supported;
  template <typename V>
  __attribute__((always_inline)) static V Do(T x, U y) {
    return ClampedSubFastAsmOp<T, U>::template Do<V>(x, y);
  }
};

template <typename T, typename U>
struct ClampedMulFastOp {
  static const bool is_supported = ClampedMulFastAsmOp<T, U>::is_supported;
  template <typename V>
  __attribute__((always_inline)) static V Do(T x, U y) {
    return ClampedMulFastAsmOp<T, U>::template Do<V>(x, y);
  }
};

template <typename T>
struct ClampedNegFastOp {
  static const bool is_supported = std::is_signed<T>::value;
  __attribute__((always_inline)) static T Do(T value) {
    // Use this when there is no assembler path available.
    if (!ClampedSubFastAsmOp<T, T>::is_supported) {
      T result;
      return !__builtin_sub_overflow(T(0), value, &result)
                 ? result
                 : std::numeric_limits<T>::max();
    }

    // Fallback to the normal subtraction path.
    return ClampedSubFastOp<T, T>::template Do<T>(T(0), value);
  }
};

}  // namespace internal
}  // namespace base

#endif  // BASE_NUMERICS_SAFE_MATH_CLANG_GCC_IMPL_H_
