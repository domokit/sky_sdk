// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_CONVERTER_TONIC_DART_CONVERTER_H_
#define LIB_CONVERTER_TONIC_DART_CONVERTER_H_

#include <string>
#include <type_traits>
#include <vector>

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/common/macros.h"
#include "tonic/logging/dart_error.h"

namespace tonic {

// DartConvert converts types back and forth from Sky to Dart. The template
// parameter |T| determines what kind of type conversion to perform.
template <typename T, typename Enable = void>
struct DartConverter {};

// This is to work around the fact that typedefs do not create new types. If you
// have a typedef, and want it to use a different converter, specialize this
// template and override the types here.
// Ex:
//   typedef int ColorType;  // Want to use a different converter.
//   class ColorConverterType {};  // Dummy type.
//   template<> struct DartConvertType<ColorConverterType> {
//     using ConverterType = ColorConverterType;
//     using ValueType = ColorType;
//   };
template <typename T>
struct DartConverterTypes {
  using ConverterType = T;
  using ValueType = T;
};

template <>
struct DartConverter<void> {
  using FfiType = void;
  static constexpr const char* kFfiRepresentation = "Void";
  static constexpr const char* kDartRepresentation = "void";
  static constexpr bool kAllowedInLeafCall = true;
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

////////////////////////////////////////////////////////////////////////////////
// Boolean

template <>
struct DartConverter<bool> {
  using NativeType = bool;
  using FfiType = bool;
  static constexpr const char* kFfiRepresentation = "Bool";
  static constexpr const char* kDartRepresentation = "bool";
  static constexpr bool kAllowedInLeafCall = true;

  static Dart_Handle ToDart(NativeType val) { return Dart_NewBoolean(val); }

  static void SetReturnValue(Dart_NativeArguments args, bool val) {
    Dart_SetBooleanReturnValue(args, val);
  }

  static NativeType FromDart(Dart_Handle handle) {
    bool result = 0;
    Dart_BooleanValue(handle, &result);
    return result;
  }

  static NativeType FromArguments(Dart_NativeArguments args,
                                  int index,
                                  Dart_Handle& exception) {
    bool result = false;
    Dart_GetNativeBooleanArgument(args, index, &result);
    return result;
  }

  static NativeType FromFfi(FfiType val) { return val; }
  static FfiType ToFfi(NativeType val) { return val; }
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

////////////////////////////////////////////////////////////////////////////////
// Numbers

template <typename T>
struct DartConverterInteger {
  using FfiType = T;
  static constexpr const char* kDartRepresentation = "int";
  static constexpr bool kAllowedInLeafCall = true;

  static Dart_Handle ToDart(T val) { return Dart_NewInteger(val); }

  static void SetReturnValue(Dart_NativeArguments args, T val) {
    Dart_SetIntegerReturnValue(args, val);
  }

  static T FromDart(Dart_Handle handle) {
    int64_t result = 0;
    Dart_IntegerToInt64(handle, &result);
    return static_cast<T>(result);
  }

  static T FromArguments(Dart_NativeArguments args,
                         int index,
                         Dart_Handle& exception) {
    int64_t result = 0;
    Dart_GetNativeIntegerArgument(args, index, &result);
    return static_cast<T>(result);
  }
  static T FromFfi(FfiType val) { return val; }
  static FfiType ToFfi(T val) { return val; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  // Note: Returns the correct bit-width for the host architecture.
  static const char* GetFfiRepresentation() {
    if (sizeof(T) == 4) {
      if (std::is_signed<T>()) {
        return "Int32";
      }
      return "Uint32";
    }
    TONIC_DCHECK(sizeof(T) == 8);
    if (std::is_signed<T>()) {
      return "Int64";
    }
    return "Uint64";
  }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

template <>
struct DartConverter<int> : public DartConverterInteger<int> {};

template <>
struct DartConverter<long int> : public DartConverterInteger<long int> {};

template <>
struct DartConverter<unsigned> : public DartConverterInteger<unsigned> {};

template <>
struct DartConverter<long long> : public DartConverterInteger<long long> {};

template <>
struct DartConverter<unsigned long>
    : public DartConverterInteger<unsigned long> {};

template <>
struct DartConverter<unsigned long long> {
  using FfiType = unsigned long long;
  static constexpr const char* kFfiRepresentation = "Uint64";
  static constexpr const char* kDartRepresentation = "int";
  static constexpr bool kAllowedInLeafCall = true;

  // TODO(abarth): The Dart VM API doesn't yet have an entry-point for
  // an unsigned 64-bit type. We will need to add a Dart API for
  // constructing an integer from uint64_t.
  //
  // (In the meantime, we have asserts below to check that we're never
  // converting values that have the 64th bit set.)

  static Dart_Handle ToDart(unsigned long long val) {
    TONIC_DCHECK(val <= 0x7fffffffffffffffLL);
    return Dart_NewInteger(static_cast<int64_t>(val));
  }

  static void SetReturnValue(Dart_NativeArguments args,
                             unsigned long long val) {
    TONIC_DCHECK(val <= 0x7fffffffffffffffLL);
    Dart_SetIntegerReturnValue(args, val);
  }

  static unsigned long long FromDart(Dart_Handle handle) {
    int64_t result = 0;
    Dart_IntegerToInt64(handle, &result);
    return result;
  }

  static unsigned long long FromArguments(Dart_NativeArguments args,
                                          int index,
                                          Dart_Handle& exception) {
    int64_t result = 0;
    Dart_GetNativeIntegerArgument(args, index, &result);
    return result;
  }

  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
  static FfiType FromFfi(FfiType val) {
    TONIC_DCHECK(val <= 0x7fffffffffffffffLL);
    return val;
  }
  // FFI does a bitwise conversion from uint64_t in C to int64 in Dart.
  static FfiType ToFfi(FfiType val) {
    TONIC_DCHECK(val <= 0x7fffffffffffffffLL);
    return val;
  }
};

// There is intentionally no DartConverter<float>, to avoid UB when Dart code
// gives us a double that is greater than the max float or less than -max float.
template <>
struct DartConverter<double> {
  using FfiType = double;
  static constexpr const char* kFfiRepresentation = "Double";
  static constexpr const char* kDartRepresentation = "double";
  static constexpr bool kAllowedInLeafCall = true;

  static Dart_Handle ToDart(double val) { return Dart_NewDouble(val); }

  static void SetReturnValue(Dart_NativeArguments args, double val) {
    Dart_SetDoubleReturnValue(args, val);
  }

  static double FromDart(Dart_Handle handle) {
    double result = 0;
    Dart_DoubleValue(handle, &result);
    return result;
  }

  static double FromArguments(Dart_NativeArguments args,
                              int index,
                              Dart_Handle& exception) {
    double result = 0;
    Dart_GetNativeDoubleArgument(args, index, &result);
    return result;
  }

  static double FromFfi(FfiType val) { return val; }
  static FfiType ToFfi(double val) { return val; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }

  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
};

////////////////////////////////////////////////////////////////////////////////
// Enum Classes

template <typename T>
struct DartConverter<T, typename std::enable_if<std::is_enum<T>::value>::type> {
  using FfiType = int32_t;
  static constexpr const char* kFfiRepresentation = "Int32";
  static constexpr const char* kDartRepresentation = "int";
  static constexpr bool kAllowedInLeafCall = true;

  static Dart_Handle ToDart(T val) {
    return Dart_NewInteger(
        static_cast<typename std::underlying_type<T>::type>(val));
  }

  static void SetReturnValue(Dart_NativeArguments args, T val) {
    Dart_SetIntegerReturnValue(
        args, static_cast<typename std::underlying_type<T>::type>(val));
  }

  static T FromDart(Dart_Handle handle) {
    int64_t result = 0;
    Dart_IntegerToInt64(handle, &result);
    return static_cast<T>(result);
  }

  static T FromArguments(Dart_NativeArguments args,
                         int index,
                         Dart_Handle& exception) {
    int64_t result = 0;
    Dart_GetNativeIntegerArgument(args, index, &result);
    return static_cast<T>(result);
  }

  static T FromFfi(FfiType val) { return static_cast<T>(val); }
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

////////////////////////////////////////////////////////////////////////////////
// Strings

template <>
struct DartConverter<std::string> {
  using NativeType = std::string;
  using FfiType = Dart_Handle;
  static constexpr const char* kFfiRepresentation = "Handle";
  static constexpr const char* kDartRepresentation = "String";
  static constexpr bool kAllowedInLeafCall = false;

  static Dart_Handle ToDart(const NativeType& val) {
    return Dart_NewStringFromUTF8(reinterpret_cast<const uint8_t*>(val.data()),
                                  val.length());
  }

  static void SetReturnValue(Dart_NativeArguments args, const NativeType& val) {
    Dart_SetReturnValue(args, ToDart(val));
  }

  static NativeType FromDart(Dart_Handle handle) {
    if (Dart_IsNull(handle)) {
      return std::string();
    }
    uint8_t* data = nullptr;
    intptr_t length = 0;
    if (Dart_IsError(Dart_StringToUTF8(handle, &data, &length)))
      return std::string();
    return std::string(reinterpret_cast<char*>(data), length);
  }

  static NativeType FromArguments(Dart_NativeArguments args,
                                  int index,
                                  Dart_Handle& exception) {
    return FromDart(Dart_GetNativeArgument(args, index));
  }

  static NativeType FromFfi(FfiType val) { return FromDart(val); }
  static FfiType ToFfi(NativeType val) { return ToDart(val); }
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

template <>
struct DartConverter<std::u16string> {
  using NativeType = std::u16string;
  using FfiType = Dart_Handle;
  static constexpr const char* kFfiRepresentation = "Handle";
  static constexpr const char* kDartRepresentation = "String";
  static constexpr bool kAllowedInLeafCall = false;

  static Dart_Handle ToDart(const NativeType& val) {
    return Dart_NewStringFromUTF16(
        reinterpret_cast<const uint16_t*>(val.data()), val.length());
  }

  static void SetReturnValue(Dart_NativeArguments args, const NativeType& val) {
    Dart_SetReturnValue(args, ToDart(val));
  }

  static NativeType FromDart(Dart_Handle handle) {
    if (Dart_IsNull(handle)) {
      return std::u16string();
    }
    intptr_t length = 0;
    Dart_StringLength(handle, &length);
    std::vector<uint16_t> data(length);
    Dart_StringToUTF16(handle, data.data(), &length);
    return std::u16string(reinterpret_cast<char16_t*>(data.data()), length);
  }

  static NativeType FromArguments(Dart_NativeArguments args,
                                  int index,
                                  Dart_Handle& exception) {
    return FromDart(Dart_GetNativeArgument(args, index));
  }

  static NativeType FromFfi(FfiType val) { return FromDart(val); }
  static FfiType ToFfi(NativeType val) { return ToDart(val); }
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

template <>
struct DartConverter<const char*> {
  static Dart_Handle ToDart(const char* val) {
    return Dart_NewStringFromCString(val);
  }

  static void SetReturnValue(Dart_NativeArguments args, const char* val) {
    Dart_SetReturnValue(args, ToDart(val));
  }

  static const char* FromDart(Dart_Handle handle) {
    if (Dart_IsNull(handle)) {
      return nullptr;
    }
    const char* result = nullptr;
    Dart_StringToCString(handle, &result);
    return result;
  }

  static const char* FromArguments(Dart_NativeArguments args,
                                   int index,
                                   Dart_Handle& exception) {
    return FromDart(Dart_GetNativeArgument(args, index));
  }
};

////////////////////////////////////////////////////////////////////////////////
// Collections

inline Dart_Handle LookupNonNullableType(const std::string& library_name,
                                         const std::string& type_name) {
  auto library =
      Dart_LookupLibrary(DartConverter<std::string>::ToDart(library_name));
  if (CheckAndHandleError(library)) {
    return library;
  }
  auto type_string = DartConverter<std::string>::ToDart(type_name);
  if (CheckAndHandleError(type_string)) {
    return type_string;
  }
  auto type = Dart_GetNonNullableType(library, type_string, 0, nullptr);
  if (CheckAndHandleError(type)) {
    return type;
  }
  return type;
}

template <typename T,
          std::enable_if_t<std::is_same<std::string, T>::value, int> = 0>
Dart_Handle ToDartTypeHandle() {
  return LookupNonNullableType("dart:core", "String");
}

template <typename T, std::enable_if_t<std::is_integral<T>::value, int> = 0>
Dart_Handle ToDartTypeHandle() {
  return LookupNonNullableType("dart:core", "int");
}

template <typename T,
          std::enable_if_t<std::is_floating_point<T>::value, int> = 0>
Dart_Handle ToDartTypeHandle() {
  return LookupNonNullableType("dart:core", "double");
}

template <typename T>
Dart_Handle CreateZeroInitializedDartObject(
    Dart_Handle type_handle_or_null = ::Dart_Null()) {
  if constexpr (std::is_same<std::string, T>::value) {
    return ::Dart_EmptyString();
  } else if constexpr (std::is_integral<T>::value) {
    return ::Dart_NewIntegerFromUint64(0u);
  } else if constexpr (std::is_floating_point<T>::value) {
    return ::Dart_NewDouble(0.0);
  } else {
    auto object = ::Dart_New(type_handle_or_null, ::Dart_Null(), 0, nullptr);
    CheckAndHandleError(object);
    return object;
  }
  return ::Dart_Null();
}

template <typename T, typename Enable = void>
struct DartListFactory {
  static Dart_Handle NewList(Dart_Handle type_handle, intptr_t length) {
    bool is_nullable = false;
    auto is_nullable_handle = ::Dart_IsNullableType(type_handle, &is_nullable);
    if (CheckAndHandleError(is_nullable_handle)) {
      return is_nullable_handle;
    }
    if (is_nullable) {
      auto list = ::Dart_NewListOfType(type_handle, length);
      CheckAndHandleError(list);
      return list;
    } else {
      auto sentinel = CreateZeroInitializedDartObject<T>(type_handle);
      if (CheckAndHandleError(sentinel)) {
        return sentinel;
      }
      auto list = ::Dart_NewListOfTypeFilled(type_handle, sentinel, length);
      CheckAndHandleError(list);
      return list;
    }
    return ::Dart_Null();
  }
};

template <typename T>
struct DartConverter<std::vector<T>> {
  using FfiType = Dart_Handle;
  static constexpr const char* kFfiRepresentation = "Handle";
  static constexpr const char* kDartRepresentation = "List";
  static constexpr bool kAllowedInLeafCall = false;

  using ValueType = typename DartConverterTypes<T>::ValueType;
  using ConverterType = typename DartConverterTypes<T>::ConverterType;

  static Dart_Handle ToDart(const std::vector<ValueType>& val) {
    Dart_Handle list = DartListFactory<ValueType>::NewList(
        ToDartTypeHandle<ValueType>(), val.size());
    if (Dart_IsError(list))
      return list;
    for (size_t i = 0; i < val.size(); i++) {
      Dart_Handle result =
          Dart_ListSetAt(list, i, DartConverter<ConverterType>::ToDart(val[i]));
      if (Dart_IsError(result))
        return result;
    }
    return list;
  }

  static void SetReturnValue(Dart_NativeArguments args,
                             const std::vector<ValueType>& val) {
    Dart_SetReturnValue(args, ToDart(val));
  }

  static std::vector<ValueType> FromDart(Dart_Handle handle) {
    std::vector<ValueType> result;

    if (!Dart_IsList(handle))
      return result;

    intptr_t length = 0;
    Dart_ListLength(handle, &length);

    if (length == 0)
      return result;

    result.reserve(length);

    std::vector<Dart_Handle> items(length);
    Dart_Handle items_result =
        Dart_ListGetRange(handle, 0, length, items.data());
    TONIC_DCHECK(!Dart_IsError(items_result));

    for (intptr_t i = 0; i < length; ++i) {
      TONIC_DCHECK(items[i]);
      result.push_back(DartConverter<ConverterType>::FromDart(items[i]));
    }
    return result;
  }

  static std::vector<ValueType> FromArguments(Dart_NativeArguments args,
                                              int index,
                                              Dart_Handle& exception) {
    return FromDart(Dart_GetNativeArgument(args, index));
  }

  static std::vector<ValueType> FromFfi(FfiType val) { return FromDart(val); }
  static FfiType ToFfi(std::vector<ValueType> val) { return ToDart(val); }
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

////////////////////////////////////////////////////////////////////////////////
// Dart_Handle

template <>
struct DartConverter<Dart_Handle> {
  using NativeType = Dart_Handle;
  using FfiType = Dart_Handle;
  static constexpr const char* kFfiRepresentation = "Handle";
  static constexpr const char* kDartRepresentation = "Object";
  static constexpr bool kAllowedInLeafCall = false;

  static Dart_Handle ToDart(NativeType val) { return val; }

  static void SetReturnValue(Dart_NativeArguments args, Dart_Handle val) {
    Dart_SetReturnValue(args, val);
  }

  static NativeType FromDart(Dart_Handle handle) { return handle; }

  static NativeType FromArguments(Dart_NativeArguments args,
                                  int index,
                                  Dart_Handle& exception) {
    Dart_Handle result = Dart_GetNativeArgument(args, index);
    TONIC_DCHECK(!Dart_IsError(result));
    return result;
  }

  static NativeType FromFfi(FfiType val) { return val; }
  static FfiType ToFfi(NativeType val) { return val; }
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

////////////////////////////////////////////////////////////////////////////////
// Convenience wrappers using type inference

template <typename T>
Dart_Handle ToDart(const T& object) {
  return DartConverter<T>::ToDart(object);
}

////////////////////////////////////////////////////////////////////////////////
// std::string support

inline Dart_Handle StdStringToDart(const std::string& val) {
  return DartConverter<std::string>::ToDart(val);
}

inline std::string StdStringFromDart(Dart_Handle handle) {
  return DartConverter<std::string>::FromDart(handle);
}

// Alias Dart_NewStringFromCString for less typing.
inline Dart_Handle ToDart(const char* val) {
  return Dart_NewStringFromCString(val);
}

}  // namespace tonic

#endif  // LIB_CONVERTER_TONIC_DART_CONVERTER_H_
