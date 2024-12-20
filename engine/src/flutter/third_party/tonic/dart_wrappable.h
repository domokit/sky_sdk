// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_DART_WRAPPABLE_H_
#define LIB_TONIC_DART_WRAPPABLE_H_

#include "third_party/dart/runtime/include/dart_api.h"
#include "tonic/common/macros.h"
#include "tonic/converter/dart_converter.h"
#include "tonic/dart_state.h"
#include "tonic/dart_weak_persistent_value.h"
#include "tonic/dart_wrapper_info.h"
#include "tonic/logging/dart_error.h"

#include <type_traits>

namespace tonic {

// DartWrappable is a base class that you can inherit from in order to be
// exposed to Dart code as an interface.
class DartWrappable {
 public:
  enum DartNativeFields {
    kPeerIndex,  // Must be first to work with Dart_GetNativeReceiver.
    kNumberOfNativeFields,
  };

  DartWrappable() : dart_wrapper_(DartWeakPersistentValue()) {}

  // Subclasses that wish to expose a new interface must override this function
  // and provide information about their wrapper. There is no need to call your
  // base class's implementation of this function.
  // Implement using IMPLEMENT_WRAPPERTYPEINFO macro
  virtual const DartWrapperInfo& GetDartWrapperInfo() const = 0;

  virtual void RetainDartWrappableReference() const = 0;

  virtual void ReleaseDartWrappableReference() const = 0;

  // Use this method sparingly. It follows a slower path using Dart_New.
  // Prefer constructing the object in Dart code and using
  // AssociateWithDartWrapper.
  Dart_Handle CreateDartWrapper(DartState* dart_state);
  void AssociateWithDartWrapper(Dart_Handle wrappable);
  void ClearDartWrapper();  // Warning: Might delete this.
  Dart_WeakPersistentHandle dart_wrapper() const {
    return dart_wrapper_.value();
  }

 protected:
  virtual ~DartWrappable();

  static Dart_PersistentHandle GetTypeForWrapper(
      tonic::DartState* dart_state,
      const tonic::DartWrapperInfo& wrapper_info);

 private:
  static void FinalizeDartWrapper(void* isolate_callback_data, void* peer);

  DartWeakPersistentValue dart_wrapper_;

  TONIC_DISALLOW_COPY_AND_ASSIGN(DartWrappable);
};

#define DEFINE_WRAPPERTYPEINFO()                                           \
 public:                                                                   \
  const tonic::DartWrapperInfo& GetDartWrapperInfo() const override {      \
    return dart_wrapper_info_;                                             \
  }                                                                        \
  static Dart_PersistentHandle GetDartType(tonic::DartState* dart_state) { \
    return GetTypeForWrapper(dart_state, dart_wrapper_info_);              \
  }                                                                        \
                                                                           \
 private:                                                                  \
  static const tonic::DartWrapperInfo& dart_wrapper_info_

#define IMPLEMENT_WRAPPERTYPEINFO(LibraryName, ClassName)                   \
  static const tonic::DartWrapperInfo                                       \
      kDartWrapperInfo_##LibraryName_##ClassName(#LibraryName, #ClassName); \
  const tonic::DartWrapperInfo& ClassName::dart_wrapper_info_ =             \
      kDartWrapperInfo_##LibraryName_##ClassName;

struct DartConverterWrappable {
  static DartWrappable* FromDart(Dart_Handle handle);
  static DartWrappable* FromArguments(Dart_NativeArguments args,
                                      int index,
                                      Dart_Handle& exception);
};

template <typename T>
struct DartConverter<
    T*,
    typename std::enable_if<
        std::is_convertible<T*, const DartWrappable*>::value>::type> {
  using FfiType = DartWrappable*;
  static constexpr const char* kFfiRepresentation = "Pointer";
  static constexpr const char* kDartRepresentation = "Pointer";
  static constexpr bool kAllowedInLeafCall = true;

  static Dart_Handle ToDart(DartWrappable* val) {
    if (!val) {
      return Dart_Null();
    }
    if (Dart_WeakPersistentHandle wrapper = val->dart_wrapper()) {
      auto strong_handle = Dart_HandleFromWeakPersistent(wrapper);
      if (!Dart_IsNull(strong_handle)) {
        return strong_handle;
      }
      // After the weak referenced object has been GCed, the handle points to
      // Dart_Null(). Fall through create a new wrapper object.
    }
    return val->CreateDartWrapper(DartState::Current());
  }

  static void SetReturnValue(Dart_NativeArguments args,
                             DartWrappable* val,
                             bool auto_scope = true) {
    if (!val) {
      Dart_SetReturnValue(args, Dart_Null());
      return;
    } else if (Dart_WeakPersistentHandle wrapper = val->dart_wrapper()) {
      auto strong_handle = Dart_HandleFromWeakPersistent(wrapper);
      if (!Dart_IsNull(strong_handle)) {
        Dart_SetReturnValue(args, strong_handle);
        return;
      }
      // After the weak referenced object has been GCed, the handle points to
      // Dart_Null(). Fall through create a new wrapper object.
    }
    Dart_SetReturnValue(args, val->CreateDartWrapper(DartState::Current()));
  }

  static T* FromDart(Dart_Handle handle) {
    // TODO(abarth): We're missing a type check.
    return static_cast<T*>(DartConverterWrappable::FromDart(handle));
  }

  static T* FromArguments(Dart_NativeArguments args,
                          int index,
                          Dart_Handle& exception,
                          bool auto_scope = true) {
    // TODO(abarth): We're missing a type check.
    return static_cast<T*>(
        DartConverterWrappable::FromArguments(args, index, exception));
  }

  static T* FromFfi(FfiType val) { return static_cast<T*>(val); }
  static FfiType ToFfi(T* val) { return val; }
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

////////////////////////////////////////////////////////////////////////////////
// Support for generic smart pointers that have a "get" method that returns a
// pointer to a type that is Dart convertible as well as a constructor that
// adopts a raw pointer to that type.

template <template <typename T> class PTR, typename T>
struct DartConverter<PTR<T>> {
  using NativeType = PTR<T>;
  using FfiType = Dart_Handle;
  static constexpr const char* kFfiRepresentation = "Handle";
  static constexpr const char* kDartRepresentation = "Object";
  static constexpr bool kAllowedInLeafCall = false;

  static Dart_Handle ToDart(const NativeType& val) {
    return DartConverter<T*>::ToDart(val.get());
  }

  static NativeType FromDart(Dart_Handle handle) {
    return NativeType(DartConverter<T*>::FromDart(handle));
  }

  static NativeType FromArguments(Dart_NativeArguments args,
                                  int index,
                                  Dart_Handle& exception,
                                  bool auto_scope = true) {
    return NativeType(
        DartConverter<T*>::FromArguments(args, index, exception, auto_scope));
  }

  static void SetReturnValue(Dart_NativeArguments args,
                             const NativeType& val,
                             bool auto_scope = true) {
    DartConverter<T*>::SetReturnValue(args, val.get());
  }

  static NativeType FromFfi(FfiType val) { return FromDart(val); }
  static FfiType ToFfi(const NativeType& val) { return ToDart(val); }
  static const char* GetFfiRepresentation() { return kFfiRepresentation; }
  static const char* GetDartRepresentation() { return kDartRepresentation; }
  static bool AllowedInLeafCall() { return kAllowedInLeafCall; }
};

template <typename T>
inline T* GetReceiver(Dart_NativeArguments args) {
  intptr_t receiver;
  Dart_Handle result = Dart_GetNativeReceiver(args, &receiver);
  TONIC_DCHECK(!Dart_IsError(result));
  if (!receiver)
    Dart_ThrowException(ToDart("Object has been disposed."));
  return static_cast<T*>(reinterpret_cast<DartWrappable*>(receiver));
}

}  // namespace tonic

#endif  // LIB_TONIC_DART_WRAPPABLE_H_
