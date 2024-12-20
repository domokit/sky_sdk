// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "builtin_libraries.h"

#include <lib/fdio/namespace.h>
#include <lib/zx/channel.h>

#include <optional>

#include "dart-pkg/fuchsia/sdk_ext/fuchsia.h"
#include "flutter/fml/logging.h"
#include "runtime/dart/utils/inlines.h"
#include "third_party/dart/runtime/bin/io_natives.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_microtask_queue.h"
#include "third_party/tonic/logging/dart_error.h"

#include "logging.h"

using tonic::ToDart;

namespace dart_runner {
namespace {

#define REGISTER_FUNCTION(name, count) {#name, name, count},
#define DECLARE_FUNCTION(name, count) \
  extern void name(Dart_NativeArguments args);

#define BUILTIN_NATIVE_LIST(V) \
  V(Logger_PrintString, 1)     \
  V(ScheduleMicrotask, 1)

BUILTIN_NATIVE_LIST(DECLARE_FUNCTION);

const struct NativeEntry {
  const char* name;
  Dart_NativeFunction function;
  int argument_count;
} kBuiltinEntries[] = {BUILTIN_NATIVE_LIST(REGISTER_FUNCTION)};

Dart_NativeFunction BuiltinNativeLookup(Dart_Handle name,
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
  size_t num_entries = dart_utils::ArraySize(kBuiltinEntries);
  for (size_t i = 0; i < num_entries; i++) {
    const NativeEntry& entry = kBuiltinEntries[i];
    if (!strcmp(function_name, entry.name) &&
        (entry.argument_count == argument_count)) {
      return entry.function;
    }
  }
  return nullptr;
}

const uint8_t* BuiltinNativeSymbol(Dart_NativeFunction native_function) {
  size_t num_entries = dart_utils::ArraySize(kBuiltinEntries);
  for (size_t i = 0; i < num_entries; i++) {
    const NativeEntry& entry = kBuiltinEntries[i];
    if (entry.function == native_function)
      return reinterpret_cast<const uint8_t*>(entry.name);
  }
  return nullptr;
}

void Logger_PrintString(Dart_NativeArguments args) {
  intptr_t length = 0;
  uint8_t* chars = nullptr;
  Dart_Handle str = Dart_GetNativeArgument(args, 0);
  Dart_Handle result = Dart_StringToUTF8(str, &chars, &length);
  if (Dart_IsError(result)) {
    Dart_PropagateError(result);
  } else {
    fwrite(chars, 1, length, stdout);
    fputc('\n', stdout);
    fflush(stdout);
  }
}

void ScheduleMicrotask(Dart_NativeArguments args) {
  Dart_Handle closure = Dart_GetNativeArgument(args, 0);
  if (tonic::CheckAndHandleError(closure) || !Dart_IsClosure(closure))
    return;
  tonic::DartMicrotaskQueue::GetForCurrentThread()->ScheduleMicrotask(closure);
}

}  // namespace

void InitBuiltinLibrariesForIsolate(const std::string& script_uri,
                                    fdio_ns_t* namespc,
                                    int stdoutfd,
                                    int stderrfd,
                                    zx::channel directory_request,
                                    bool service_isolate) {
  // dart:fuchsia --------------------------------------------------------------
  // dart runner doesn't care about scenic view ref.
  if (!service_isolate) {
    fuchsia::dart::Initialize(std::move(directory_request), std::nullopt);
  }

  // dart:fuchsia.builtin ------------------------------------------------------

  Dart_Handle builtin_lib = Dart_LookupLibrary(ToDart("dart:fuchsia.builtin"));
  FML_CHECK(!tonic::CheckAndHandleError(builtin_lib));
  Dart_Handle result = Dart_SetNativeResolver(builtin_lib, BuiltinNativeLookup,
                                              BuiltinNativeSymbol);
  FML_CHECK(!tonic::CheckAndHandleError(result));

  // dart:io -------------------------------------------------------------------

  Dart_Handle io_lib = Dart_LookupLibrary(ToDart("dart:io"));
  FML_CHECK(!tonic::CheckAndHandleError(io_lib));
  result = Dart_SetNativeResolver(io_lib, dart::bin::IONativeLookup,
                                  dart::bin::IONativeSymbol);
  FML_CHECK(!tonic::CheckAndHandleError(result));

  // dart:zircon ---------------------------------------------------------------

  Dart_Handle zircon_lib = Dart_LookupLibrary(ToDart("dart:zircon"));
  FML_CHECK(!tonic::CheckAndHandleError(zircon_lib));
  // NativeResolver already set by fuchsia::dart::Initialize().

  // Core libraries ------------------------------------------------------------

  Dart_Handle async_lib = Dart_LookupLibrary(ToDart("dart:async"));
  FML_CHECK(!tonic::CheckAndHandleError(async_lib));

  Dart_Handle core_lib = Dart_LookupLibrary(ToDart("dart:core"));
  FML_CHECK(!tonic::CheckAndHandleError(core_lib));

  Dart_Handle internal_lib = Dart_LookupLibrary(ToDart("dart:_internal"));
  FML_CHECK(!tonic::CheckAndHandleError(internal_lib));

  Dart_Handle isolate_lib = Dart_LookupLibrary(ToDart("dart:isolate"));
  FML_CHECK(!tonic::CheckAndHandleError(isolate_lib));

#if !defined(AOT_RUNTIME)
  // AOT: These steps already happened at compile time in gen_snapshot.

  // We need to ensure that all the scripts loaded so far are finalized
  // as we are about to invoke some Dart code below to set up closures.
  result = Dart_FinalizeLoading(false);
  FML_CHECK(!tonic::CheckAndHandleError(result));
#endif

  // Setup the internal library's 'internalPrint' function.
  Dart_Handle print =
      Dart_Invoke(builtin_lib, ToDart("_getPrintClosure"), 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(print));

  result = Dart_SetField(internal_lib, ToDart("_printClosure"), print);
  FML_CHECK(!tonic::CheckAndHandleError(result));

  // Set up the 'scheduleImmediate' closure.
  Dart_Handle schedule_immediate_closure;
  if (service_isolate) {
    // Running on dart::ThreadPool.
    schedule_immediate_closure = Dart_Invoke(
        isolate_lib, ToDart("_getIsolateScheduleImmediateClosure"), 0, nullptr);
  } else {
    // Running on async::Loop.
    schedule_immediate_closure = Dart_Invoke(
        builtin_lib, ToDart("_getScheduleMicrotaskClosure"), 0, nullptr);
  }
  FML_CHECK(!tonic::CheckAndHandleError(schedule_immediate_closure));

  Dart_Handle schedule_args[1];
  schedule_args[0] = schedule_immediate_closure;
  result = Dart_Invoke(async_lib, ToDart("_setScheduleImmediateClosure"), 1,
                       schedule_args);
  FML_CHECK(!tonic::CheckAndHandleError(result));

  // Set up the namespace in dart:io.
  Dart_Handle namespace_type =
      Dart_GetNonNullableType(io_lib, ToDart("_Namespace"), 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(namespace_type));

  Dart_Handle namespace_args[1];
  namespace_args[0] = ToDart(reinterpret_cast<intptr_t>(namespc));
  result =
      Dart_Invoke(namespace_type, ToDart("_setupNamespace"), 1, namespace_args);
  FML_CHECK(!tonic::CheckAndHandleError(result));

  // Set up the namespace in dart:zircon.
  namespace_type =
      Dart_GetNonNullableType(zircon_lib, ToDart("_Namespace"), 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(namespace_type));

  result = Dart_SetField(namespace_type, ToDart("_namespace"),
                         ToDart(reinterpret_cast<intptr_t>(namespc)));
  FML_CHECK(!tonic::CheckAndHandleError(result));

  // Set up stdout and stderr.
  Dart_Handle stdio_args[3];
  stdio_args[0] = Dart_NewInteger(0);
  stdio_args[1] = Dart_NewInteger(stdoutfd);
  stdio_args[2] = Dart_NewInteger(stderrfd);
  result = Dart_Invoke(io_lib, ToDart("_setStdioFDs"), 3, stdio_args);
  FML_CHECK(!tonic::CheckAndHandleError(result));

  // Disable some dart:io operations.
  Dart_Handle embedder_config_type =
      Dart_GetNonNullableType(io_lib, ToDart("_EmbedderConfig"), 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(embedder_config_type));

  result =
      Dart_SetField(embedder_config_type, ToDart("_mayExit"), Dart_False());
  FML_CHECK(!tonic::CheckAndHandleError(result));

  // Set the script location.
  result = Dart_SetField(builtin_lib, ToDart("_rawScript"), ToDart(script_uri));
  FML_CHECK(!tonic::CheckAndHandleError(result));

  // Setup the uriBase with the base uri of the fidl app.
  Dart_Handle uri_base =
      Dart_Invoke(io_lib, ToDart("_getUriBaseClosure"), 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(uri_base));

  result = Dart_SetField(core_lib, ToDart("_uriBaseClosure"), uri_base);
  FML_CHECK(!tonic::CheckAndHandleError(result));

  Dart_Handle setup_hooks = ToDart("_setupHooks");
  result = Dart_Invoke(builtin_lib, setup_hooks, 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(result));
  result = Dart_Invoke(io_lib, setup_hooks, 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(result));
  result = Dart_Invoke(isolate_lib, setup_hooks, 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(result));
}

}  // namespace dart_runner
