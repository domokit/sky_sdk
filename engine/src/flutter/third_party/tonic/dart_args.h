// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_DART_ARGS_H_
#define LIB_TONIC_DART_ARGS_H_

#include <iostream>
#include <sstream>
#include <type_traits>
#include <utility>

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/converter/dart_converter.h"
#include "tonic/dart_wrappable.h"

namespace tonic {

class DartArgIterator {
 public:
  explicit DartArgIterator(Dart_NativeArguments args, int start_index = 1)
      : args_(args), index_(start_index), had_exception_(false) {}

  template <typename T>
  T GetNext() {
    if (had_exception_)
      return T();
    Dart_Handle exception = nullptr;
    T arg = DartConverter<T>::FromArguments(args_, index_++, exception);
    if (exception) {
      had_exception_ = true;
      Dart_ThrowException(exception);
    }
    return arg;
  }

  bool had_exception() const { return had_exception_; }

  Dart_NativeArguments args() const { return args_; }

 private:
  Dart_NativeArguments args_;
  int index_;
  bool had_exception_;

  TONIC_DISALLOW_COPY_AND_ASSIGN(DartArgIterator);
};

// Classes for generating and storing an argument pack of integer indices
// (based on well-known "indices trick", see: http://goo.gl/bKKojn):
template <size_t... indices>
struct IndicesHolder {};

template <size_t requested_index, size_t... indices>
struct IndicesGenerator {
  using type = typename IndicesGenerator<requested_index - 1,
                                         requested_index - 1,
                                         indices...>::type;
};

template <size_t... indices>
struct IndicesGenerator<0, indices...> {
  using type = IndicesHolder<indices...>;
};

template <typename T>
class IndicesForSignature {};

template <typename ResultType, typename... ArgTypes>
struct IndicesForSignature<ResultType (*)(ArgTypes...)> {
  static const size_t count = sizeof...(ArgTypes);
  using type = typename IndicesGenerator<count>::type;
};

template <typename C, typename ResultType, typename... ArgTypes>
struct IndicesForSignature<ResultType (C::*)(ArgTypes...)> {
  static const size_t count = sizeof...(ArgTypes);
  using type = typename IndicesGenerator<count>::type;
};

template <typename C, typename ResultType, typename... ArgTypes>
struct IndicesForSignature<ResultType (C::*)(ArgTypes...) const> {
  static const size_t count = sizeof...(ArgTypes);
  using type = typename IndicesGenerator<count>::type;
};

template <size_t index, typename ArgType>
struct DartArgHolder {
  using ValueType = typename std::remove_const<
      typename std::remove_reference<ArgType>::type>::type;

  ValueType value;

  explicit DartArgHolder(DartArgIterator* it)
      : value(it->GetNext<ValueType>()) {}
};

template <typename T>
void DartReturn(T result, Dart_NativeArguments args) {
  DartConverter<T>::SetReturnValue(args, std::move(result));
}

template <typename IndicesType, typename T>
class DartDispatcher {};

// Match functions on the form:
// `void f(ArgTypes...)`
template <size_t... indices, typename... ArgTypes>
struct DartDispatcher<IndicesHolder<indices...>, void (*)(ArgTypes...)>
    : public DartArgHolder<indices, ArgTypes>... {
  using FunctionPtr = void (*)(ArgTypes...);

  DartArgIterator* it_;

  explicit DartDispatcher(DartArgIterator* it)
      : DartArgHolder<indices, ArgTypes>(it)..., it_(it) {}

  void Dispatch(FunctionPtr func) {
    (*func)(DartArgHolder<indices, ArgTypes>::value...);
  }
};

// Match functions on the form:
// `ResultType f(ArgTypes...)`
template <size_t... indices, typename ResultType, typename... ArgTypes>
struct DartDispatcher<IndicesHolder<indices...>, ResultType (*)(ArgTypes...)>
    : public DartArgHolder<indices, ArgTypes>... {
  using FunctionPtr = ResultType (*)(ArgTypes...);
  using CtorResultType = ResultType;

  DartArgIterator* it_;

  explicit DartDispatcher(DartArgIterator* it)
      : DartArgHolder<indices, ArgTypes>(it)..., it_(it) {}

  void Dispatch(FunctionPtr func) {
    DartReturn((*func)(DartArgHolder<indices, ArgTypes>::value...),
               it_->args());
  }

  ResultType DispatchCtor(FunctionPtr func) {
    return (*func)(DartArgHolder<indices, ArgTypes>::value...);
  }
};

// Match instance methods on the form:
// `void C::m(ArgTypes...)`
template <size_t... indices, typename C, typename... ArgTypes>
struct DartDispatcher<IndicesHolder<indices...>, void (C::*)(ArgTypes...)>
    : public DartArgHolder<indices, ArgTypes>... {
  using FunctionPtr = void (C::*)(ArgTypes...);

  DartArgIterator* it_;

  explicit DartDispatcher(DartArgIterator* it)
      : DartArgHolder<indices, ArgTypes>(it)..., it_(it) {}

  void Dispatch(FunctionPtr func) {
    (GetReceiver<C>(it_->args())->*func)(
        DartArgHolder<indices, ArgTypes>::value...);
  }
};

// Match instance methods on the form:
// `ReturnType (C::m)(ArgTypes...) const`
template <size_t... indices,
          typename C,
          typename ReturnType,
          typename... ArgTypes>
struct DartDispatcher<IndicesHolder<indices...>,
                      ReturnType (C::*)(ArgTypes...) const>
    : public DartArgHolder<indices, ArgTypes>... {
  using FunctionPtr = ReturnType (C::*)(ArgTypes...) const;

  DartArgIterator* it_;

  explicit DartDispatcher(DartArgIterator* it)
      : DartArgHolder<indices, ArgTypes>(it)..., it_(it) {}

  void Dispatch(FunctionPtr func) {
    DartReturn((GetReceiver<C>(it_->args())->*func)(
                   DartArgHolder<indices, ArgTypes>::value...),
               it_->args());
  }
};

// Match instance methods on the form:
// `ReturnType (C::m)(ArgTypes...)`
template <size_t... indices,
          typename C,
          typename ResultType,
          typename... ArgTypes>
struct DartDispatcher<IndicesHolder<indices...>, ResultType (C::*)(ArgTypes...)>
    : public DartArgHolder<indices, ArgTypes>... {
  using FunctionPtr = ResultType (C::*)(ArgTypes...);

  DartArgIterator* it_;

  explicit DartDispatcher(DartArgIterator* it)
      : DartArgHolder<indices, ArgTypes>(it)..., it_(it) {}

  void Dispatch(FunctionPtr func) {
    DartReturn((GetReceiver<C>(it_->args())->*func)(
                   DartArgHolder<indices, ArgTypes>::value...),
               it_->args());
  }
};

template <typename Sig>
void DartCall(Sig func, Dart_NativeArguments args) {
  DartArgIterator it(args);
  using Indices = typename IndicesForSignature<Sig>::type;
  DartDispatcher<Indices, Sig> decoder(&it);
  if (it.had_exception())
    return;
  decoder.Dispatch(func);
}

template <typename Sig>
void DartCallStatic(Sig func, Dart_NativeArguments args) {
  DartArgIterator it(args, 0);
  using Indices = typename IndicesForSignature<Sig>::type;
  DartDispatcher<Indices, Sig> decoder(&it);
  if (it.had_exception())
    return;
  decoder.Dispatch(func);
}

template <typename Sig>
void DartCallConstructor(Sig func, Dart_NativeArguments args) {
  DartArgIterator it(args);
  using Indices = typename IndicesForSignature<Sig>::type;
  using Wrappable = typename DartDispatcher<Indices, Sig>::CtorResultType;
  Wrappable wrappable;
  {
    DartDispatcher<Indices, Sig> decoder(&it);
    if (it.had_exception())
      return;
    wrappable = decoder.DispatchCtor(func);
  }

  Dart_Handle wrapper = Dart_GetNativeArgument(args, 0);
  TONIC_CHECK(!CheckAndHandleError(wrapper));

  intptr_t native_fields[DartWrappable::kNumberOfNativeFields];
  TONIC_CHECK(!CheckAndHandleError(Dart_GetNativeFieldsOfArgument(
      args, 0, DartWrappable::kNumberOfNativeFields, native_fields)));
  TONIC_CHECK(!native_fields[DartWrappable::kPeerIndex]);

  wrappable->AssociateWithDartWrapper(wrapper);
}

// Templates to automatically setup static entry points for FFI Native
// functions.
// Entry points for instance methods take the instance as the first argument and
// call the given method with the remaining arguments.
// Arguments will automatically get converted to and from their FFI
// representations with the DartConverter templates.
//
// @tparam C The type of the receiver. Or `void` if there is no receiver.
// @tparam Signature The signature of the function being dispatched to.
// @tparam function The function pointer being dispatched to.
template <typename C, typename Signature, Signature function>
struct FfiDispatcher;

// Concatenate the FFI representation of each argument to the stream,
// serialising them into a comma separated list.
// Example: "Handle, Bool, Uint64"
template <typename Arg, typename... Args>
void WriteFfiArguments(std::ostringstream* stream) {
  *stream << tonic::DartConverter<typename std::remove_const<
      typename std::remove_reference<Arg>::type>::type>::GetFfiRepresentation();
  if constexpr (sizeof...(Args) > 0) {
    *stream << ", ";
    WriteFfiArguments<Args...>(stream);
  }
}

// Concatenate the Dart representation of each argument to the stream,
// serialising them into a comma separated list.
// Example: "Object, bool, int"
template <typename Arg, typename... Args>
void WriteDartArguments(std::ostringstream* stream) {
  *stream << tonic::DartConverter<
      typename std::remove_const<typename std::remove_reference<Arg>::type>::
          type>::GetDartRepresentation();
  if constexpr (sizeof...(Args) > 0) {
    *stream << ", ";
    WriteDartArguments<Args...>(stream);
  }
}

// Logical 'and' together whether each argument is allowed in a leaf call.
template <typename Arg, typename... Args>
bool AllowedInLeafCall() {
  bool result = tonic::DartConverter<typename std::remove_const<
      typename std::remove_reference<Arg>::type>::type>::AllowedInLeafCall();
  if constexpr (sizeof...(Args) > 0) {
    result &= AllowedInLeafCall<Args...>();
  }
  return result;
}

// Match `Return function(...)`.
template <typename Return, typename... Args, Return (*function)(Args...)>
struct FfiDispatcher<void, Return (*)(Args...), function> {
  using FfiReturn = typename DartConverter<Return>::FfiType;
  static const size_t n_args = sizeof...(Args);

  // Static C entry-point with Dart FFI signature.
  static FfiReturn Call(
      typename DartConverter<typename std::remove_const<
          typename std::remove_reference<Args>::type>::type>::FfiType... args) {
    // Call C++ function, forwarding converted native arguments.
    return DartConverter<Return>::ToFfi(function(
        DartConverter<typename std::remove_const<typename std::remove_reference<
            Args>::type>::type>::FromFfi(args)...));
  }

  static bool AllowedAsLeafCall() {
    if constexpr (sizeof...(Args) > 0) {
      return AllowedInLeafCall<Return>() && AllowedInLeafCall<Args...>();
    }
    return AllowedInLeafCall<Return>();
  }

  static const char* GetReturnFfiRepresentation() {
    return tonic::DartConverter<Return>::GetFfiRepresentation();
  }

  static const char* GetReturnDartRepresentation() {
    return tonic::DartConverter<Return>::GetDartRepresentation();
  }

  static void WriteFfiArguments(std::ostringstream* stream) {
    if constexpr (sizeof...(Args) > 0) {
      ::tonic::WriteFfiArguments<Args...>(stream);
    }
  }

  static void WriteDartArguments(std::ostringstream* stream) {
    if constexpr (sizeof...(Args) > 0) {
      ::tonic::WriteDartArguments<Args...>(stream);
    }
  }
};

// Match `Return C::method(...)`.
template <typename C,
          typename Return,
          typename... Args,
          Return (C::*method)(Args...)>
struct FfiDispatcher<C, Return (C::*)(Args...), method> {
  using FfiReturn = typename DartConverter<Return>::FfiType;
  static const size_t n_args = sizeof...(Args);

  // Static C entry-point with Dart FFI signature.
  static FfiReturn Call(
      DartWrappable* receiver,
      typename DartConverter<typename std::remove_const<
          typename std::remove_reference<Args>::type>::type>::FfiType... args) {
    // Call C++ method on receiver, forwarding converted native arguments.
    return DartConverter<Return>::ToFfi((static_cast<C*>(receiver)->*method)(
        DartConverter<typename std::remove_const<typename std::remove_reference<
            Args>::type>::type>::FromFfi(args)...));
  }

  static bool AllowedAsLeafCall() {
    if constexpr (sizeof...(Args) > 0) {
      return AllowedInLeafCall<Return>() && AllowedInLeafCall<Args...>();
    }
    return AllowedInLeafCall<Return>();
  }

  static const char* GetReturnFfiRepresentation() {
    return tonic::DartConverter<Return>::GetFfiRepresentation();
  }

  static const char* GetReturnDartRepresentation() {
    return tonic::DartConverter<Return>::GetDartRepresentation();
  }

  static void WriteFfiArguments(std::ostringstream* stream) {
    *stream << tonic::DartConverter<C*>::GetFfiRepresentation();
    if constexpr (sizeof...(Args) > 0) {
      *stream << ", ";
      ::tonic::WriteFfiArguments<Args...>(stream);
    }
  }

  static void WriteDartArguments(std::ostringstream* stream) {
    *stream << tonic::DartConverter<C*>::GetDartRepresentation();
    if constexpr (sizeof...(Args) > 0) {
      *stream << ", ";
      ::tonic::WriteDartArguments<Args...>(stream);
    }
  }
};

// Match `Return C::method(...) const`.
template <typename C,
          typename Return,
          typename... Args,
          Return (C::*method)(Args...) const>
struct FfiDispatcher<C, Return (C::*)(Args...) const, method> {
  using FfiReturn = typename DartConverter<Return>::FfiType;
  static const size_t n_args = sizeof...(Args);

  // Static C entry-point with Dart FFI signature.
  static FfiReturn Call(
      DartWrappable* receiver,
      typename DartConverter<typename std::remove_const<
          typename std::remove_reference<Args>::type>::type>::FfiType... args) {
    // Call C++ method on receiver, forwarding converted native arguments.
    return DartConverter<Return>::ToFfi((static_cast<C*>(receiver)->*method)(
        DartConverter<typename std::remove_const<typename std::remove_reference<
            Args>::type>::type>::FromFfi(args)...));
  }

  static bool AllowedAsLeafCall() {
    if constexpr (sizeof...(Args) > 0) {
      return AllowedInLeafCall<Return>() && AllowedInLeafCall<Args...>();
    }
    return AllowedInLeafCall<Return>();
  }

  static const char* GetReturnFfiRepresentation() {
    return tonic::DartConverter<Return>::GetFfiRepresentation();
  }

  static const char* GetReturnDartRepresentation() {
    return tonic::DartConverter<Return>::GetDartRepresentation();
  }

  static void WriteFfiArguments(std::ostringstream* stream) {
    *stream << tonic::DartConverter<C*>::GetFfiRepresentation();
    if constexpr (sizeof...(Args) > 0) {
      *stream << ", ";
      ::tonic::WriteFfiArguments<Args...>(stream);
    }
  }

  static void WriteDartArguments(std::ostringstream* stream) {
    *stream << tonic::DartConverter<C*>::GetDartRepresentation();
    if constexpr (sizeof...(Args) > 0) {
      *stream << ", ";
      ::tonic::WriteDartArguments<Args...>(stream);
    }
  }
};

// `void` specialisation since we can't declare `ToFfi` to take void rvalues.
// Match `void function(...)`.
template <typename... Args, void (*function)(Args...)>
struct FfiDispatcher<void, void (*)(Args...), function> {
  static const size_t n_args = sizeof...(Args);

  // Static C entry-point with Dart FFI signature.
  static void Call(
      typename DartConverter<typename std::remove_const<
          typename std::remove_reference<Args>::type>::type>::FfiType... args) {
    // Call C++ function, forwarding converted native arguments.
    function(
        DartConverter<typename std::remove_const<typename std::remove_reference<
            Args>::type>::type>::FromFfi(args)...);
  }

  static bool AllowedAsLeafCall() {
    if constexpr (sizeof...(Args) > 0) {
      return AllowedInLeafCall<Args...>();
    }
    return true;
  }

  static const char* GetReturnFfiRepresentation() {
    return tonic::DartConverter<void>::GetFfiRepresentation();
  }

  static const char* GetReturnDartRepresentation() {
    return tonic::DartConverter<void>::GetDartRepresentation();
  }

  static void WriteFfiArguments(std::ostringstream* stream) {
    if constexpr (sizeof...(Args) > 0) {
      ::tonic::WriteFfiArguments<Args...>(stream);
    }
  }

  static void WriteDartArguments(std::ostringstream* stream) {
    if constexpr (sizeof...(Args) > 0) {
      ::tonic::WriteDartArguments<Args...>(stream);
    }
  }
};

// `void` specialisation since we can't declare `ToFfi` to take void rvalues.
// Match `void C::method(...)`.
template <typename C, typename... Args, void (C::*method)(Args...)>
struct FfiDispatcher<C, void (C::*)(Args...), method> {
  static const size_t n_args = sizeof...(Args);

  // Static C entry-point with Dart FFI signature.
  static void Call(
      DartWrappable* receiver,
      typename DartConverter<typename std::remove_const<
          typename std::remove_reference<Args>::type>::type>::FfiType... args) {
    // Call C++ method on receiver, forwarding converted native arguments.
    (static_cast<C*>(receiver)->*method)(
        DartConverter<typename std::remove_const<typename std::remove_reference<
            Args>::type>::type>::FromFfi(args)...);
  }

  static bool AllowedAsLeafCall() {
    if constexpr (sizeof...(Args) > 0) {
      return AllowedInLeafCall<Args...>();
    }
    return true;
  }

  static const char* GetReturnFfiRepresentation() {
    return tonic::DartConverter<void>::GetFfiRepresentation();
  }

  static const char* GetReturnDartRepresentation() {
    return tonic::DartConverter<void>::GetDartRepresentation();
  }

  static void WriteFfiArguments(std::ostringstream* stream) {
    *stream << tonic::DartConverter<C*>::GetFfiRepresentation();
    if constexpr (sizeof...(Args) > 0) {
      *stream << ", ";
      ::tonic::WriteFfiArguments<Args...>(stream);
    }
  }

  static void WriteDartArguments(std::ostringstream* stream) {
    *stream << tonic::DartConverter<C*>::GetDartRepresentation();
    if constexpr (sizeof...(Args) > 0) {
      *stream << ", ";
      ::tonic::WriteDartArguments<Args...>(stream);
    }
  }
};

}  // namespace tonic

#endif  // LIB_TONIC_DART_ARGS_H_
