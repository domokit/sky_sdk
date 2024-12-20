// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "isolate_configurator.h"

#include "dart-pkg/fuchsia/sdk_ext/fuchsia.h"
#include "dart-pkg/zircon/sdk_ext/handle.h"
#include "third_party/dart/runtime/include/dart_api.h"
#include "third_party/tonic/converter/dart_converter.h"
#include "third_party/tonic/dart_state.h"
#include "third_party/tonic/logging/dart_error.h"

namespace flutter_runner {

IsolateConfigurator::IsolateConfigurator(UniqueFDIONS fdio_ns,
                                         zx::channel directory_request,
                                         zx::eventpair view_ref)
    : fdio_ns_(std::move(fdio_ns)),
      directory_request_(std::move(directory_request)),
      view_ref_(std::move(view_ref)) {}

IsolateConfigurator::~IsolateConfigurator() = default;

bool IsolateConfigurator::ConfigureCurrentIsolate() {
  if (used_) {
    return false;
  }
  used_ = true;

  BindFuchsia();
  BindZircon();
  BindDartIO();

  // This is now owned by the Dart bindings. So relinquish our ownership of the
  // handle.
  (void)fdio_ns_.release();

  return true;
}

void IsolateConfigurator::BindFuchsia() {
  fuchsia::dart::Initialize(std::move(directory_request_),
                            std::move(view_ref_));
}

void IsolateConfigurator::BindZircon() {
  // Tell dart:zircon about the FDIO namespace configured for this instance.
  Dart_Handle zircon_lib = Dart_LookupLibrary(tonic::ToDart("dart:zircon"));
  FML_CHECK(!tonic::CheckAndHandleError(zircon_lib));

  Dart_Handle namespace_type = Dart_GetNonNullableType(
      zircon_lib, tonic::ToDart("_Namespace"), 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(namespace_type));

  Dart_Handle result =
      Dart_SetField(namespace_type,               //
                    tonic::ToDart("_namespace"),  //
                    tonic::ToDart(reinterpret_cast<intptr_t>(fdio_ns_.get())));
  FML_CHECK(!tonic::CheckAndHandleError(result));
}

void IsolateConfigurator::BindDartIO() {
  // Grab the dart:io lib.
  Dart_Handle io_lib = Dart_LookupLibrary(tonic::ToDart("dart:io"));
  FML_CHECK(!tonic::CheckAndHandleError(io_lib));

  // Disable dart:io exit()
  Dart_Handle embedder_config_type = Dart_GetNonNullableType(
      io_lib, tonic::ToDart("_EmbedderConfig"), 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(embedder_config_type));

  Dart_Handle result = Dart_SetField(embedder_config_type,
                                     tonic::ToDart("_mayExit"), Dart_False());
  FML_CHECK(!tonic::CheckAndHandleError(result));

  // Tell dart:io about the FDIO namespace configured for this instance.
  Dart_Handle namespace_type =
      Dart_GetNonNullableType(io_lib, tonic::ToDart("_Namespace"), 0, nullptr);
  FML_CHECK(!tonic::CheckAndHandleError(namespace_type));

  Dart_Handle namespace_args[] = {
      Dart_NewInteger(reinterpret_cast<intptr_t>(fdio_ns_.get())),  //
  };
  result = Dart_Invoke(namespace_type, tonic::ToDart("_setupNamespace"), 1,
                       namespace_args);
  FML_CHECK(!tonic::CheckAndHandleError(result));
}

}  // namespace flutter_runner
