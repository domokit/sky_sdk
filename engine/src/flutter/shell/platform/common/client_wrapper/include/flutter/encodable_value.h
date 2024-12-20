// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_ENCODABLE_VALUE_H_
#define FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_ENCODABLE_VALUE_H_

#include <any>
#include <cassert>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <variant>
#include <vector>

// Unless overridden, attempt to detect the RTTI state from the compiler.
#ifndef FLUTTER_ENABLE_RTTI
#if defined(_MSC_VER)
#ifdef _CPPRTTI
#define FLUTTER_ENABLE_RTTI 1
#endif
#elif defined(__clang__)
#if __has_feature(cxx_rtti)
#define FLUTTER_ENABLE_RTTI 1
#endif
#elif defined(__GNUC__)
#ifdef __GXX_RTTI
#define FLUTTER_ENABLE_RTTI 1
#endif
#endif
#endif  // #ifndef FLUTTER_ENABLE_RTTI

namespace flutter {

static_assert(sizeof(double) == 8, "EncodableValue requires a 64-bit double");

// A container for arbitrary types in EncodableValue.
//
// This is used in conjunction with StandardCodecExtension to allow using other
// types with a StandardMethodCodec/StandardMessageCodec. It is implicitly
// convertible to EncodableValue, so constructing an EncodableValue from a
// custom type can generally be written as:
//   CustomEncodableValue(MyType(...))
// rather than:
//   EncodableValue(CustomEncodableValue(MyType(...)))
//
// For extracting received custom types, it is implicitly convertible to
// std::any. For example:
//   const MyType& my_type_value =
//        std::any_cast<MyType>(std::get<CustomEncodableValue>(value));
//
// If RTTI is enabled, different extension types can be checked with type():
//   if (custom_value->type() == typeid(SomeData)) { ... }
// Clients that wish to disable RTTI would need to decide on another approach
// for distinguishing types (e.g., in StandardCodecExtension::WriteValueOfType)
// if multiple custom types are needed. For instance, wrapping all of the
// extension types in an EncodableValue-style variant, and only ever storing
// that variant in CustomEncodableValue.
class CustomEncodableValue {
 public:
  explicit CustomEncodableValue(const std::any& value) : value_(value) {}
  ~CustomEncodableValue() = default;

  // Allow implicit conversion to std::any to allow direct use of any_cast.
  // NOLINTNEXTLINE(google-explicit-constructor)
  operator std::any&() { return value_; }
  // NOLINTNEXTLINE(google-explicit-constructor)
  operator const std::any&() const { return value_; }

#if defined(FLUTTER_ENABLE_RTTI) && FLUTTER_ENABLE_RTTI
  // Passthrough to std::any's type().
  const std::type_info& type() const noexcept { return value_.type(); }
#endif

  // This operator exists only to provide a stable ordering for use as a
  // std::map key, to satisfy the compiler requirements for EncodableValue.
  // It does not attempt to provide useful ordering semantics, and using a
  // custom value as a map key is not recommended.
  bool operator<(const CustomEncodableValue& other) const {
    return this < &other;
  }
  bool operator==(const CustomEncodableValue& other) const {
    return this == &other;
  }

 private:
  std::any value_;
};

class EncodableValue;

// Convenience type aliases.
using EncodableList = std::vector<EncodableValue>;
using EncodableMap = std::map<EncodableValue, EncodableValue>;

namespace internal {
// The base class for EncodableValue. Do not use this directly; it exists only
// for EncodableValue to inherit from.
//
// Do not change the order or indexes of the items here; see the comment on
// EncodableValue
using EncodableValueVariant = std::variant<std::monostate,
                                           bool,
                                           int32_t,
                                           int64_t,
                                           double,
                                           std::string,
                                           std::vector<uint8_t>,
                                           std::vector<int32_t>,
                                           std::vector<int64_t>,
                                           std::vector<double>,
                                           EncodableList,
                                           EncodableMap,
                                           CustomEncodableValue,
                                           std::vector<float>>;
}  // namespace internal

// An object that can contain any value or collection type supported by
// Flutter's standard method codec.
//
// For details, see:
// https://api.flutter.dev/flutter/services/StandardMessageCodec-class.html
//
// As an example, the following Dart structure:
//   {
//     'flag': true,
//     'name': 'Thing',
//     'values': [1, 2.0, 4],
//   }
// would correspond to:
//   EncodableValue(EncodableMap{
//       {EncodableValue("flag"), EncodableValue(true)},
//       {EncodableValue("name"), EncodableValue("Thing")},
//       {EncodableValue("values"), EncodableValue(EncodableList{
//                                      EncodableValue(1),
//                                      EncodableValue(2.0),
//                                      EncodableValue(4),
//                                  })},
//   })
//
// The primary API surface for this object is std::variant. For instance,
// getting a string value from an EncodableValue, with type checking:
//   if (std::holds_alternative<std::string>(value)) {
//     std::string some_string = std::get<std::string>(value);
//   }
//
// The order/indexes of the variant types is part of the API surface, and is
// guaranteed not to change.
//
// The variant types are mapped with Dart types in following ways:
// std::monostate       -> null
// bool                 -> bool
// int32_t              -> int
// int64_t              -> int
// double               -> double
// std::string          -> String
// std::vector<uint8_t> -> Uint8List
// std::vector<int32_t> -> Int32List
// std::vector<int64_t> -> Int64List
// std::vector<float>   -> Float32List
// std::vector<double>  -> Float64List
// EncodableList        -> List
// EncodableMap         -> Map
class EncodableValue : public internal::EncodableValueVariant {
 public:
  // Rely on std::variant for most of the constructors/operators.
  using super = internal::EncodableValueVariant;
  using super::super;
  using super::operator=;

  explicit EncodableValue() = default;

  // Avoid the C++17 pitfall of conversion from char* to bool. Should not be
  // needed for C++20.
  explicit EncodableValue(const char* string) : super(std::string(string)) {}
  EncodableValue& operator=(const char* other) {
    *this = std::string(other);
    return *this;
  }

  // Allow implicit conversion from CustomEncodableValue; the only reason to
  // make a CustomEncodableValue (which can only be constructed explicitly) is
  // to use it with EncodableValue, so the risk of unintended conversions is
  // minimal, and it avoids the need for the verbose:
  //   EncodableValue(CustomEncodableValue(...)).
  // NOLINTNEXTLINE(google-explicit-constructor)
  EncodableValue(const CustomEncodableValue& v) : super(v) {}

  // Override the conversion constructors from std::variant to make them
  // explicit, to avoid implicit conversion.
  //
  // While implicit conversion can be convenient in some cases, it can have very
  // surprising effects. E.g., calling a function that takes an EncodableValue
  // but accidentally passing an EncodableValue* would, instead of failing to
  // compile, go through a pointer->bool->EncodableValue(bool) chain and
  // silently call the function with a temp-constructed EncodableValue(true).
  template <class T>
  constexpr explicit EncodableValue(T&& t) noexcept : super(t) {}

  // Returns true if the value is null. Convenience wrapper since unlike the
  // other types, std::monostate uses aren't self-documenting.
  bool IsNull() const { return std::holds_alternative<std::monostate>(*this); }

  // Convenience method to simplify handling objects received from Flutter
  // where the values may be larger than 32-bit, since they have the same type
  // on the Dart side, but will be either 32-bit or 64-bit here depending on
  // the value.
  //
  // Calling this method if the value doesn't contain either an int32_t or an
  // int64_t will throw an exception.
  int64_t LongValue() const {
    if (std::holds_alternative<int32_t>(*this)) {
      return std::get<int32_t>(*this);
    }
    return std::get<int64_t>(*this);
  }

  // Explicitly provide operator<, delegating to std::variant's operator<.
  // There are issues with with the way the standard library-provided
  // < and <=> comparisons interact with classes derived from variant.
  friend bool operator<(const EncodableValue& lhs, const EncodableValue& rhs) {
    return static_cast<const super&>(lhs) < static_cast<const super&>(rhs);
  }
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_COMMON_CLIENT_WRAPPER_INCLUDE_FLUTTER_ENCODABLE_VALUE_H_
