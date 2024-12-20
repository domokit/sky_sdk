// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "natives.h"

#include <zircon/syscalls.h>

#include <cstring>
#include <memory>
#include <vector>

#include "handle.h"
#include "handle_disposition.h"
#include "handle_waiter.h"
#include "system.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/tonic/dart_binding_macros.h"
#include "third_party/tonic/dart_class_library.h"
#include "third_party/tonic/dart_class_provider.h"
#include "third_party/tonic/dart_library_natives.h"
#include "third_party/tonic/dart_state.h"
#include "third_party/tonic/logging/dart_invoke.h"
#include "third_party/tonic/typed_data/typed_list.h"

using tonic::ToDart;

namespace zircon {
namespace dart {
namespace {

static tonic::DartLibraryNatives* g_natives;

tonic::DartLibraryNatives* InitNatives() {
  tonic::DartLibraryNatives* natives = new tonic::DartLibraryNatives();
  HandleDisposition::RegisterNatives(natives);
  HandleWaiter::RegisterNatives(natives);
  Handle::RegisterNatives(natives);
  System::RegisterNatives(natives);

  return natives;
}

Dart_NativeFunction NativeLookup(Dart_Handle name,
                                 int argument_count,
                                 bool* auto_setup_scope) {
  const char* function_name = nullptr;
  Dart_Handle result = Dart_StringToCString(name, &function_name);
  if (Dart_IsError(result)) {
    Dart_PropagateError(result);
  }
  FML_DCHECK(function_name != nullptr);
  FML_DCHECK(auto_setup_scope != nullptr);
  *auto_setup_scope = true;
  if (!g_natives)
    g_natives = InitNatives();
  return g_natives->GetNativeFunction(name, argument_count, auto_setup_scope);
}

const uint8_t* NativeSymbol(Dart_NativeFunction native_function) {
  if (!g_natives)
    g_natives = InitNatives();
  return g_natives->GetSymbol(native_function);
}

}  // namespace

void Initialize() {
  Dart_Handle library = Dart_LookupLibrary(ToDart("dart:zircon"));
  FML_CHECK(!tonic::CheckAndHandleError(library));
  Dart_Handle result = Dart_SetNativeResolver(
      library, zircon::dart::NativeLookup, zircon::dart::NativeSymbol);
  FML_CHECK(!tonic::CheckAndHandleError(result));

  auto dart_state = tonic::DartState::Current();
  std::unique_ptr<tonic::DartClassProvider> zircon_class_provider(
      new tonic::DartClassProvider(dart_state, "dart:zircon"));
  dart_state->class_library().add_provider("zircon",
                                           std::move(zircon_class_provider));
}

}  // namespace dart
}  // namespace zircon
