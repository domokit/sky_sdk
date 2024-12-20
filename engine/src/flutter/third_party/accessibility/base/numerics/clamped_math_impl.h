// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_NUMERICS_CLAMPED_MATH_IMPL_H_
#define BASE_NUMERICS_CLAMPED_MATH_IMPL_H_

#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <type_traits>

#include "base/numerics/checked_math.h"
#include "base/numerics/safe_conversions.h"
#include "base/numerics/safe_math_shared_impl.h"

namespace base {
namespace internal {

template <typename T,
          typename std::enable_if<std::is_integral<T>::value &&
                                  std::is_signed<T>::value>::type* = nullptr>
constexpr T SaturatedNegWrapper(T value) {
  return MustTreatAsConstexpr(value) || !ClampedNegFastOp<T>::is_supported
             ? (NegateWrapper(value) != std::numeric_limits<T>::lowest()
                    ? NegateWrapper(value)
                    : std::numeric_limits<T>::max())
             : ClampedNegFastOp<T>::Do(value);
}

template <typename T,
          typename std::enable_if<std::is_integral<T>::value &&
                                  !std::is_signed<T>::value>::type* = nullptr>
constexpr T SaturatedNegWrapper(T value) {
  return T(0);
}

template <
    typename T,
    typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr>
constexpr T SaturatedNegWrapper(T value) {
  return -value;
}

template <typename T,
          typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
constexpr T SaturatedAbsWrapper(T value) {
  // The calculation below is a static identity for unsigned types, but for
  // signed integer types it provides a non-branching, saturated absolute value.
  // This works because SafeUnsignedAbs() returns an unsigned type, which can
  // represent the absolute value of all negative numbers of an equal-width
  // integer type. The call to IsValueNegative() then detects overflow in the
  // special case of numeric_limits<T>::min(), by evaluating the bit pattern as
  // a signed integer value. If it is the overflow case, we end up subtracting
  // one from the unsigned result, thus saturating to numeric_limits<T>::max().
  return static_cast<T>(SafeUnsignedAbs(value) -
                        IsValueNegative<T>(SafeUnsignedAbs(value)));
}

template <
    typename T,
    typename std::enable_if<std::is_floating_point<T>::value>::type* = nullptr>
constexpr T SaturatedAbsWrapper(T value) {
  return value < 0 ? -value : value;
}

template <typename T, typename U, class Enable = void>
struct ClampedAddOp {};

template <typename T, typename U>
struct ClampedAddOp<T,
                    U,
                    typename std::enable_if<std::is_integral<T>::value &&
                                            std::is_integral<U>::value>::type> {
  using result_type = typename MaxExponentPromotion<T, U>::type;
  template <typename V = result_type>
  static constexpr V Do(T x, U y) {
    if (ClampedAddFastOp<T, U>::is_supported)
      return ClampedAddFastOp<T, U>::template Do<V>(x, y);

    static_assert(std::is_same<V, result_type>::value ||
                      IsTypeInRangeForNumericType<U, V>::value,
                  "The saturation result cannot be determined from the "
                  "provided types.");
    const V saturated = CommonMaxOrMin<V>(IsValueNegative(y));
    V result = {};
    return BASE_NUMERICS_LIKELY((CheckedAddOp<T, U>::Do(x, y, &result)))
               ? result
               : saturated;
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedSubOp {};

template <typename T, typename U>
struct ClampedSubOp<T,
                    U,
                    typename std::enable_if<std::is_integral<T>::value &&
                                            std::is_integral<U>::value>::type> {
  using result_type = typename MaxExponentPromotion<T, U>::type;
  template <typename V = result_type>
  static constexpr V Do(T x, U y) {
    // TODO(jschuh) Make this "constexpr if" once we're C++17.
    if (ClampedSubFastOp<T, U>::is_supported)
      return ClampedSubFastOp<T, U>::template Do<V>(x, y);

    static_assert(std::is_same<V, result_type>::value ||
                      IsTypeInRangeForNumericType<U, V>::value,
                  "The saturation result cannot be determined from the "
                  "provided types.");
    const V saturated = CommonMaxOrMin<V>(!IsValueNegative(y));
    V result = {};
    return BASE_NUMERICS_LIKELY((CheckedSubOp<T, U>::Do(x, y, &result)))
               ? result
               : saturated;
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedMulOp {};

template <typename T, typename U>
struct ClampedMulOp<T,
                    U,
                    typename std::enable_if<std::is_integral<T>::value &&
                                            std::is_integral<U>::value>::type> {
  using result_type = typename MaxExponentPromotion<T, U>::type;
  template <typename V = result_type>
  static constexpr V Do(T x, U y) {
    // TODO(jschuh) Make this "constexpr if" once we're C++17.
    if (ClampedMulFastOp<T, U>::is_supported)
      return ClampedMulFastOp<T, U>::template Do<V>(x, y);

    V result = {};
    const V saturated =
        CommonMaxOrMin<V>(IsValueNegative(x) ^ IsValueNegative(y));
    return BASE_NUMERICS_LIKELY((CheckedMulOp<T, U>::Do(x, y, &result)))
               ? result
               : saturated;
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedDivOp {};

template <typename T, typename U>
struct ClampedDivOp<T,
                    U,
                    typename std::enable_if<std::is_integral<T>::value &&
                                            std::is_integral<U>::value>::type> {
  using result_type = typename MaxExponentPromotion<T, U>::type;
  template <typename V = result_type>
  static constexpr V Do(T x, U y) {
    V result = {};
    if (BASE_NUMERICS_LIKELY((CheckedDivOp<T, U>::Do(x, y, &result))))
      return result;
    // Saturation goes to max, min, or NaN (if x is zero).
    return x ? CommonMaxOrMin<V>(IsValueNegative(x) ^ IsValueNegative(y))
             : SaturationDefaultLimits<V>::NaN();
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedModOp {};

template <typename T, typename U>
struct ClampedModOp<T,
                    U,
                    typename std::enable_if<std::is_integral<T>::value &&
                                            std::is_integral<U>::value>::type> {
  using result_type = typename MaxExponentPromotion<T, U>::type;
  template <typename V = result_type>
  static constexpr V Do(T x, U y) {
    V result = {};
    return BASE_NUMERICS_LIKELY((CheckedModOp<T, U>::Do(x, y, &result)))
               ? result
               : x;
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedLshOp {};

// Left shift. Non-zero values saturate in the direction of the sign. A zero
// shifted by any value always results in zero.
template <typename T, typename U>
struct ClampedLshOp<T,
                    U,
                    typename std::enable_if<std::is_integral<T>::value &&
                                            std::is_integral<U>::value>::type> {
  using result_type = T;
  template <typename V = result_type>
  static constexpr V Do(T x, U shift) {
    static_assert(!std::is_signed<U>::value, "Shift value must be unsigned.");
    if (BASE_NUMERICS_LIKELY(shift < std::numeric_limits<T>::digits)) {
      // Shift as unsigned to avoid undefined behavior.
      V result = static_cast<V>(as_unsigned(x) << shift);
      // If the shift can be reversed, we know it was valid.
      if (BASE_NUMERICS_LIKELY(result >> shift == x))
        return result;
    }
    return x ? CommonMaxOrMin<V>(IsValueNegative(x)) : 0;
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedRshOp {};

// Right shift. Negative values saturate to -1. Positive or 0 saturates to 0.
template <typename T, typename U>
struct ClampedRshOp<T,
                    U,
                    typename std::enable_if<std::is_integral<T>::value &&
                                            std::is_integral<U>::value>::type> {
  using result_type = T;
  template <typename V = result_type>
  static constexpr V Do(T x, U shift) {
    static_assert(!std::is_signed<U>::value, "Shift value must be unsigned.");
    // Signed right shift is odd, because it saturates to -1 or 0.
    const V saturated = as_unsigned(V(0)) - IsValueNegative(x);
    return BASE_NUMERICS_LIKELY(shift < IntegerBitsPlusSign<T>::value)
               ? saturated_cast<V>(x >> shift)
               : saturated;
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedAndOp {};

template <typename T, typename U>
struct ClampedAndOp<T,
                    U,
                    typename std::enable_if<std::is_integral<T>::value &&
                                            std::is_integral<U>::value>::type> {
  using result_type = typename std::make_unsigned<
      typename MaxExponentPromotion<T, U>::type>::type;
  template <typename V>
  static constexpr V Do(T x, U y) {
    return static_cast<result_type>(x) & static_cast<result_type>(y);
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedOrOp {};

// For simplicity we promote to unsigned integers.
template <typename T, typename U>
struct ClampedOrOp<T,
                   U,
                   typename std::enable_if<std::is_integral<T>::value &&
                                           std::is_integral<U>::value>::type> {
  using result_type = typename std::make_unsigned<
      typename MaxExponentPromotion<T, U>::type>::type;
  template <typename V>
  static constexpr V Do(T x, U y) {
    return static_cast<result_type>(x) | static_cast<result_type>(y);
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedXorOp {};

// For simplicity we support only unsigned integers.
template <typename T, typename U>
struct ClampedXorOp<T,
                    U,
                    typename std::enable_if<std::is_integral<T>::value &&
                                            std::is_integral<U>::value>::type> {
  using result_type = typename std::make_unsigned<
      typename MaxExponentPromotion<T, U>::type>::type;
  template <typename V>
  static constexpr V Do(T x, U y) {
    return static_cast<result_type>(x) ^ static_cast<result_type>(y);
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedMaxOp {};

template <typename T, typename U>
struct ClampedMaxOp<
    T,
    U,
    typename std::enable_if<std::is_arithmetic<T>::value &&
                            std::is_arithmetic<U>::value>::type> {
  using result_type = typename MaxExponentPromotion<T, U>::type;
  template <typename V = result_type>
  static constexpr V Do(T x, U y) {
    return IsGreater<T, U>::Test(x, y) ? saturated_cast<V>(x)
                                       : saturated_cast<V>(y);
  }
};

template <typename T, typename U, class Enable = void>
struct ClampedMinOp {};

template <typename T, typename U>
struct ClampedMinOp<
    T,
    U,
    typename std::enable_if<std::is_arithmetic<T>::value &&
                            std::is_arithmetic<U>::value>::type> {
  using result_type = typename LowestValuePromotion<T, U>::type;
  template <typename V = result_type>
  static constexpr V Do(T x, U y) {
    return IsLess<T, U>::Test(x, y) ? saturated_cast<V>(x)
                                    : saturated_cast<V>(y);
  }
};

// This is just boilerplate that wraps the standard floating point arithmetic.
// A macro isn't the nicest solution, but it beats rewriting these repeatedly.
#define BASE_FLOAT_ARITHMETIC_OPS(NAME, OP)                              \
  template <typename T, typename U>                                      \
  struct Clamped##NAME##Op<                                              \
      T, U,                                                              \
      typename std::enable_if<std::is_floating_point<T>::value ||        \
                              std::is_floating_point<U>::value>::type> { \
    using result_type = typename MaxExponentPromotion<T, U>::type;       \
    template <typename V = result_type>                                  \
    static constexpr V Do(T x, U y) {                                    \
      return saturated_cast<V>(x OP y);                                  \
    }                                                                    \
  };

BASE_FLOAT_ARITHMETIC_OPS(Add, +)
BASE_FLOAT_ARITHMETIC_OPS(Sub, -)
BASE_FLOAT_ARITHMETIC_OPS(Mul, *)
BASE_FLOAT_ARITHMETIC_OPS(Div, /)

#undef BASE_FLOAT_ARITHMETIC_OPS

}  // namespace internal
}  // namespace base

#endif  // BASE_NUMERICS_CLAMPED_MATH_IMPL_H_
