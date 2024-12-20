// Copyright 2017 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef BASE_NUMERICS_CHECKED_MATH_H_
#define BASE_NUMERICS_CHECKED_MATH_H_

#include <cstddef>
#include <limits>
#include <type_traits>

#include "base/numerics/checked_math_impl.h"

namespace base {
namespace internal {

template <typename T>
class CheckedNumeric {
  static_assert(std::is_arithmetic<T>::value,
                "CheckedNumeric<T>: T must be a numeric type.");

 public:
  using type = T;

  constexpr CheckedNumeric() = default;

  // Copy constructor.
  template <typename Src>
  constexpr CheckedNumeric(const CheckedNumeric<Src>& rhs)
      : state_(rhs.state_.value(), rhs.IsValid()) {}

  template <typename Src>
  friend class CheckedNumeric;

  // This is not an explicit constructor because we implicitly upgrade regular
  // numerics to CheckedNumerics to make them easier to use.
  template <typename Src>
  constexpr CheckedNumeric(Src value)  // NOLINT(runtime/explicit)
      : state_(value) {
    static_assert(std::is_arithmetic<Src>::value, "Argument must be numeric.");
  }

  // This is not an explicit constructor because we want a seamless conversion
  // from StrictNumeric types.
  template <typename Src>
  constexpr CheckedNumeric(
      StrictNumeric<Src> value)  // NOLINT(runtime/explicit)
      : state_(static_cast<Src>(value)) {}

  // IsValid() - The public API to test if a CheckedNumeric is currently valid.
  // A range checked destination type can be supplied using the Dst template
  // parameter.
  template <typename Dst = T>
  constexpr bool IsValid() const {
    return state_.is_valid() &&
           IsValueInRangeForNumericType<Dst>(state_.value());
  }

  // AssignIfValid(Dst) - Assigns the underlying value if it is currently valid
  // and is within the range supported by the destination type. Returns true if
  // successful and false otherwise.
  template <typename Dst>
#if defined(__clang__) || defined(__GNUC__)
  __attribute__((warn_unused_result))
#elif defined(_MSC_VER)
  _Check_return_
#endif
  constexpr bool
  AssignIfValid(Dst* result) const {
    return BASE_NUMERICS_LIKELY(IsValid<Dst>())
               ? ((*result = static_cast<Dst>(state_.value())), true)
               : false;
  }

  // ValueOrDie() - The primary accessor for the underlying value. If the
  // current state is not valid it will CHECK and crash.
  // A range checked destination type can be supplied using the Dst template
  // parameter, which will trigger a CHECK if the value is not in bounds for
  // the destination.
  // The CHECK behavior can be overridden by supplying a handler as a
  // template parameter, for test code, etc. However, the handler cannot access
  // the underlying value, and it is not available through other means.
  template <typename Dst = T, class CheckHandler = CheckOnFailure>
  constexpr StrictNumeric<Dst> ValueOrDie() const {
    return BASE_NUMERICS_LIKELY(IsValid<Dst>())
               ? static_cast<Dst>(state_.value())
               : CheckHandler::template HandleFailure<Dst>();
  }

  // ValueOrDefault(T default_value) - A convenience method that returns the
  // current value if the state is valid, and the supplied default_value for
  // any other state.
  // A range checked destination type can be supplied using the Dst template
  // parameter. WARNING: This function may fail to compile or CHECK at runtime
  // if the supplied default_value is not within range of the destination type.
  template <typename Dst = T, typename Src>
  constexpr StrictNumeric<Dst> ValueOrDefault(const Src default_value) const {
    return BASE_NUMERICS_LIKELY(IsValid<Dst>())
               ? static_cast<Dst>(state_.value())
               : checked_cast<Dst>(default_value);
  }

  // Returns a checked numeric of the specified type, cast from the current
  // CheckedNumeric. If the current state is invalid or the destination cannot
  // represent the result then the returned CheckedNumeric will be invalid.
  template <typename Dst>
  constexpr CheckedNumeric<typename UnderlyingType<Dst>::type> Cast() const {
    return *this;
  }

  // This friend method is available solely for providing more detailed logging
  // in the tests. Do not implement it in production code, because the
  // underlying values may change at any time.
  template <typename U>
  friend U GetNumericValueForTest(const CheckedNumeric<U>& src);

  // Prototypes for the supported arithmetic operator overloads.
  template <typename Src>
  constexpr CheckedNumeric& operator+=(const Src rhs);
  template <typename Src>
  constexpr CheckedNumeric& operator-=(const Src rhs);
  template <typename Src>
  constexpr CheckedNumeric& operator*=(const Src rhs);
  template <typename Src>
  constexpr CheckedNumeric& operator/=(const Src rhs);
  template <typename Src>
  constexpr CheckedNumeric& operator%=(const Src rhs);
  template <typename Src>
  constexpr CheckedNumeric& operator<<=(const Src rhs);
  template <typename Src>
  constexpr CheckedNumeric& operator>>=(const Src rhs);
  template <typename Src>
  constexpr CheckedNumeric& operator&=(const Src rhs);
  template <typename Src>
  constexpr CheckedNumeric& operator|=(const Src rhs);
  template <typename Src>
  constexpr CheckedNumeric& operator^=(const Src rhs);

  constexpr CheckedNumeric operator-() const {
    // The negation of two's complement int min is int min, so we simply
    // check for that in the constexpr case.
    // We use an optimized code path for a known run-time variable.
    return MustTreatAsConstexpr(state_.value()) || !std::is_signed<T>::value ||
                   std::is_floating_point<T>::value
               ? CheckedNumeric<T>(
                     NegateWrapper(state_.value()),
                     IsValid() && (!std::is_signed<T>::value ||
                                   std::is_floating_point<T>::value ||
                                   NegateWrapper(state_.value()) !=
                                       std::numeric_limits<T>::lowest()))
               : FastRuntimeNegate();
  }

  constexpr CheckedNumeric operator~() const {
    return CheckedNumeric<decltype(InvertWrapper(T()))>(
        InvertWrapper(state_.value()), IsValid());
  }

  constexpr CheckedNumeric Abs() const {
    return !IsValueNegative(state_.value()) ? *this : -*this;
  }

  template <typename U>
  constexpr CheckedNumeric<typename MathWrapper<CheckedMaxOp, T, U>::type> Max(
      const U rhs) const {
    using R = typename UnderlyingType<U>::type;
    using result_type = typename MathWrapper<CheckedMaxOp, T, U>::type;
    // TODO(jschuh): This can be converted to the MathOp version and remain
    // constexpr once we have C++14 support.
    return CheckedNumeric<result_type>(
        static_cast<result_type>(
            IsGreater<T, R>::Test(state_.value(), Wrapper<U>::value(rhs))
                ? state_.value()
                : Wrapper<U>::value(rhs)),
        state_.is_valid() && Wrapper<U>::is_valid(rhs));
  }

  template <typename U>
  constexpr CheckedNumeric<typename MathWrapper<CheckedMinOp, T, U>::type> Min(
      const U rhs) const {
    using R = typename UnderlyingType<U>::type;
    using result_type = typename MathWrapper<CheckedMinOp, T, U>::type;
    // TODO(jschuh): This can be converted to the MathOp version and remain
    // constexpr once we have C++14 support.
    return CheckedNumeric<result_type>(
        static_cast<result_type>(
            IsLess<T, R>::Test(state_.value(), Wrapper<U>::value(rhs))
                ? state_.value()
                : Wrapper<U>::value(rhs)),
        state_.is_valid() && Wrapper<U>::is_valid(rhs));
  }

  // This function is available only for integral types. It returns an unsigned
  // integer of the same width as the source type, containing the absolute value
  // of the source, and properly handling signed min.
  constexpr CheckedNumeric<typename UnsignedOrFloatForSize<T>::type>
  UnsignedAbs() const {
    return CheckedNumeric<typename UnsignedOrFloatForSize<T>::type>(
        SafeUnsignedAbs(state_.value()), state_.is_valid());
  }

  constexpr CheckedNumeric& operator++() {
    *this += 1;
    return *this;
  }

  constexpr CheckedNumeric operator++(int) {
    CheckedNumeric value = *this;
    *this += 1;
    return value;
  }

  constexpr CheckedNumeric& operator--() {
    *this -= 1;
    return *this;
  }

  constexpr CheckedNumeric operator--(int) {
    CheckedNumeric value = *this;
    *this -= 1;
    return value;
  }

  // These perform the actual math operations on the CheckedNumerics.
  // Binary arithmetic operations.
  template <template <typename, typename, typename> class M,
            typename L,
            typename R>
  static constexpr CheckedNumeric MathOp(const L lhs, const R rhs) {
    using Math = typename MathWrapper<M, L, R>::math;
    T result = 0;
    bool is_valid =
        Wrapper<L>::is_valid(lhs) && Wrapper<R>::is_valid(rhs) &&
        Math::Do(Wrapper<L>::value(lhs), Wrapper<R>::value(rhs), &result);
    return CheckedNumeric<T>(result, is_valid);
  }

  // Assignment arithmetic operations.
  template <template <typename, typename, typename> class M, typename R>
  constexpr CheckedNumeric& MathOp(const R rhs) {
    using Math = typename MathWrapper<M, T, R>::math;
    T result = 0;  // Using T as the destination saves a range check.
    bool is_valid = state_.is_valid() && Wrapper<R>::is_valid(rhs) &&
                    Math::Do(state_.value(), Wrapper<R>::value(rhs), &result);
    *this = CheckedNumeric<T>(result, is_valid);
    return *this;
  }

 private:
  CheckedNumericState<T> state_;

  CheckedNumeric FastRuntimeNegate() const {
    T result;
    bool success = CheckedSubOp<T, T>::Do(T(0), state_.value(), &result);
    return CheckedNumeric<T>(result, IsValid() && success);
  }

  template <typename Src>
  constexpr CheckedNumeric(Src value, bool is_valid)
      : state_(value, is_valid) {}

  // These wrappers allow us to handle state the same way for both
  // CheckedNumeric and POD arithmetic types.
  template <typename Src>
  struct Wrapper {
    static constexpr bool is_valid(Src) { return true; }
    static constexpr Src value(Src value) { return value; }
  };

  template <typename Src>
  struct Wrapper<CheckedNumeric<Src>> {
    static constexpr bool is_valid(const CheckedNumeric<Src> v) {
      return v.IsValid();
    }
    static constexpr Src value(const CheckedNumeric<Src> v) {
      return v.state_.value();
    }
  };

  template <typename Src>
  struct Wrapper<StrictNumeric<Src>> {
    static constexpr bool is_valid(const StrictNumeric<Src>) { return true; }
    static constexpr Src value(const StrictNumeric<Src> v) {
      return static_cast<Src>(v);
    }
  };
};

// Convenience functions to avoid the ugly template disambiguator syntax.
template <typename Dst, typename Src>
constexpr bool IsValidForType(const CheckedNumeric<Src> value) {
  return value.template IsValid<Dst>();
}

template <typename Dst, typename Src>
constexpr StrictNumeric<Dst> ValueOrDieForType(
    const CheckedNumeric<Src> value) {
  return value.template ValueOrDie<Dst>();
}

template <typename Dst, typename Src, typename Default>
constexpr StrictNumeric<Dst> ValueOrDefaultForType(
    const CheckedNumeric<Src> value,
    const Default default_value) {
  return value.template ValueOrDefault<Dst>(default_value);
}

// Convience wrapper to return a new CheckedNumeric from the provided arithmetic
// or CheckedNumericType.
template <typename T>
constexpr CheckedNumeric<typename UnderlyingType<T>::type> MakeCheckedNum(
    const T value) {
  return value;
}

// These implement the variadic wrapper for the math operations.
template <template <typename, typename, typename> class M,
          typename L,
          typename R>
constexpr CheckedNumeric<typename MathWrapper<M, L, R>::type> CheckMathOp(
    const L lhs,
    const R rhs) {
  using Math = typename MathWrapper<M, L, R>::math;
  return CheckedNumeric<typename Math::result_type>::template MathOp<M>(lhs,
                                                                        rhs);
}

// General purpose wrapper template for arithmetic operations.
template <template <typename, typename, typename> class M,
          typename L,
          typename R,
          typename... Args>
constexpr CheckedNumeric<typename ResultType<M, L, R, Args...>::type>
CheckMathOp(const L lhs, const R rhs, const Args... args) {
  return CheckMathOp<M>(CheckMathOp<M>(lhs, rhs), args...);
}

BASE_NUMERIC_ARITHMETIC_OPERATORS(Checked, Check, Add, +, +=)
BASE_NUMERIC_ARITHMETIC_OPERATORS(Checked, Check, Sub, -, -=)
BASE_NUMERIC_ARITHMETIC_OPERATORS(Checked, Check, Mul, *, *=)
BASE_NUMERIC_ARITHMETIC_OPERATORS(Checked, Check, Div, /, /=)
BASE_NUMERIC_ARITHMETIC_OPERATORS(Checked, Check, Mod, %, %=)
BASE_NUMERIC_ARITHMETIC_OPERATORS(Checked, Check, Lsh, <<, <<=)
BASE_NUMERIC_ARITHMETIC_OPERATORS(Checked, Check, Rsh, >>, >>=)
BASE_NUMERIC_ARITHMETIC_OPERATORS(Checked, Check, And, &, &=)
BASE_NUMERIC_ARITHMETIC_OPERATORS(Checked, Check, Or, |, |=)
BASE_NUMERIC_ARITHMETIC_OPERATORS(Checked, Check, Xor, ^, ^=)
BASE_NUMERIC_ARITHMETIC_VARIADIC(Checked, Check, Max)
BASE_NUMERIC_ARITHMETIC_VARIADIC(Checked, Check, Min)

// These are some extra StrictNumeric operators to support simple pointer
// arithmetic with our result types. Since wrapping on a pointer is always
// bad, we trigger the CHECK condition here.
template <typename L, typename R>
L* operator+(L* lhs, const StrictNumeric<R> rhs) {
  uintptr_t result = CheckAdd(reinterpret_cast<uintptr_t>(lhs),
                              CheckMul(sizeof(L), static_cast<R>(rhs)))
                         .template ValueOrDie<uintptr_t>();
  return reinterpret_cast<L*>(result);
}

template <typename L, typename R>
L* operator-(L* lhs, const StrictNumeric<R> rhs) {
  uintptr_t result = CheckSub(reinterpret_cast<uintptr_t>(lhs),
                              CheckMul(sizeof(L), static_cast<R>(rhs)))
                         .template ValueOrDie<uintptr_t>();
  return reinterpret_cast<L*>(result);
}

}  // namespace internal

using internal::CheckAdd;
using internal::CheckAnd;
using internal::CheckDiv;
using internal::CheckedNumeric;
using internal::CheckLsh;
using internal::CheckMax;
using internal::CheckMin;
using internal::CheckMod;
using internal::CheckMul;
using internal::CheckOr;
using internal::CheckRsh;
using internal::CheckSub;
using internal::CheckXor;
using internal::IsValidForType;
using internal::MakeCheckedNum;
using internal::ValueOrDefaultForType;
using internal::ValueOrDieForType;

}  // namespace base

#endif  // BASE_NUMERICS_CHECKED_MATH_H_
