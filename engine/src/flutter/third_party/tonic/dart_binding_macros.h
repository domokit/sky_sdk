// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_DART_BINDING_MACROS_H_
#define LIB_TONIC_DART_BINDING_MACROS_H_

#include "tonic/dart_args.h"

#define DART_NATIVE_NO_UI_CHECK_CALLBACK(CLASS, METHOD)     \
  static void CLASS##_##METHOD(Dart_NativeArguments args) { \
    tonic::DartCall(&CLASS::METHOD, args);                  \
  }

#define DART_NATIVE_CALLBACK(CLASS, METHOD)                 \
  static void CLASS##_##METHOD(Dart_NativeArguments args) { \
    UIDartState::ThrowIfUIOperationsProhibited();           \
    tonic::DartCall(&CLASS::METHOD, args);                  \
  }

#define DART_NATIVE_CALLBACK_STATIC(CLASS, METHOD)          \
  static void CLASS##_##METHOD(Dart_NativeArguments args) { \
    tonic::DartCallStatic(&CLASS::METHOD, args);            \
  }

#define DART_REGISTER_NATIVE(CLASS, METHOD) \
  {#CLASS "_" #METHOD, CLASS##_##METHOD,    \
   tonic::IndicesForSignature<decltype(&CLASS::METHOD)>::count + 1, true},

#define DART_REGISTER_NATIVE_STATIC(CLASS, METHOD)                        \
  {                                                                       \
    #CLASS "_" #METHOD, CLASS##_##METHOD,                                 \
        tonic::IndicesForSignature<decltype(&CLASS::METHOD)>::count, true \
  }

#define DART_BIND_ALL(CLASS, FOR_EACH)                              \
  FOR_EACH(DART_NATIVE_CALLBACK)                                    \
  void CLASS::RegisterNatives(tonic::DartLibraryNatives* natives) { \
    natives->Register({FOR_EACH(DART_REGISTER_NATIVE)});            \
  }

#endif  // LIB_TONIC_DART_BINDING_MACROS_H_
