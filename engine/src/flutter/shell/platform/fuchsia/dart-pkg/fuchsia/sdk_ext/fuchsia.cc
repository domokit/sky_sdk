// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "dart-pkg/fuchsia/sdk_ext/fuchsia.h"

#include <zircon/syscalls.h>

#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#include "dart-pkg/zircon/sdk_ext/handle.h"
#include "dart-pkg/zircon/sdk_ext/natives.h"
#include "dart-pkg/zircon/sdk_ext/system.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/tonic/dart_binding_macros.h"
#include "third_party/tonic/dart_class_library.h"
#include "third_party/tonic/dart_class_provider.h"
#include "third_party/tonic/dart_library_natives.h"
#include "third_party/tonic/dart_state.h"
#include "third_party/tonic/logging/dart_invoke.h"
#include "third_party/tonic/typed_data/typed_list.h"

using tonic::ToDart;

namespace fuchsia {
namespace dart {
namespace {

static tonic::DartLibraryNatives* g_natives;

tonic::DartLibraryNatives* InitNatives() {
  tonic::DartLibraryNatives* natives = new tonic::DartLibraryNatives();

  return natives;
}

#define REGISTER_FUNCTION(name, count) {"" #name, name, count},
#define DECLARE_FUNCTION(name, count) \
  extern void name(Dart_NativeArguments args);

#define FIDL_NATIVE_LIST(V) V(SetReturnCode, 1)

FIDL_NATIVE_LIST(DECLARE_FUNCTION);

static struct NativeEntries {
  const char* name;
  Dart_NativeFunction function;
  int argument_count;
} Entries[] = {FIDL_NATIVE_LIST(REGISTER_FUNCTION)};

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
  size_t num_entries = std::size(Entries);
  for (size_t i = 0; i < num_entries; ++i) {
    const struct NativeEntries& entry = Entries[i];
    if (!strcmp(function_name, entry.name) &&
        (entry.argument_count == argument_count)) {
      return entry.function;
    }
  }
  if (!g_natives)
    g_natives = InitNatives();
  return g_natives->GetNativeFunction(name, argument_count, auto_setup_scope);
}

const uint8_t* NativeSymbol(Dart_NativeFunction native_function) {
  size_t num_entries = std::size(Entries);
  for (size_t i = 0; i < num_entries; ++i) {
    const struct NativeEntries& entry = Entries[i];
    if (entry.function == native_function) {
      return reinterpret_cast<const uint8_t*>(entry.name);
    }
  }
  if (!g_natives)
    g_natives = InitNatives();
  return g_natives->GetSymbol(native_function);
}

void SetReturnCode(Dart_NativeArguments arguments) {
  int64_t return_code;
  Dart_Handle status =
      Dart_GetNativeIntegerArgument(arguments, 0, &return_code);
  if (!tonic::CheckAndHandleError(status)) {
    tonic::DartState::Current()->SetReturnCode(return_code);
  }
}

}  // namespace

void Initialize(zx::channel directory_request,
                std::optional<zx::eventpair> view_ref) {
  zircon::dart::Initialize();

  Dart_Handle library = Dart_LookupLibrary(ToDart("dart:fuchsia"));
  FML_CHECK(!tonic::CheckAndHandleError(library));
  Dart_Handle result = Dart_SetNativeResolver(
      library, fuchsia::dart::NativeLookup, fuchsia::dart::NativeSymbol);
  FML_CHECK(!tonic::CheckAndHandleError(result));

  auto dart_state = tonic::DartState::Current();
  std::unique_ptr<tonic::DartClassProvider> fuchsia_class_provider(
      new tonic::DartClassProvider(dart_state, "dart:fuchsia"));
  dart_state->class_library().add_provider("fuchsia",
                                           std::move(fuchsia_class_provider));

  if (directory_request) {
    result = Dart_SetField(
        library, ToDart("_outgoingServices"),
        ToDart(zircon::dart::Handle::Create(std::move(directory_request))));
    FML_CHECK(!tonic::CheckAndHandleError(result));
  }

  if (view_ref) {
    result = Dart_SetField(
        library, ToDart("_viewRef"),
        ToDart(zircon::dart::Handle::Create((*view_ref).release())));
    FML_CHECK(!tonic::CheckAndHandleError(result));
  }
}

}  // namespace dart
}  // namespace fuchsia
