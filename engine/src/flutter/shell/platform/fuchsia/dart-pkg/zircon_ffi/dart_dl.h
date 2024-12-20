#ifndef FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_DART_DL_H_
#define FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_DART_DL_H_

#include "macros.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize Dart API with dynamic linking.
//
// Must be called with `NativeApi.initializeApiDLData` from `dart:ffi`, before
// using other functions.
//
// Returns 1 on success.
ZIRCON_FFI_EXPORT int zircon_dart_dl_initialize(void* initialize_api_dl_data);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // FLUTTER_SHELL_PLATFORM_FUCHSIA_DART_PKG_ZIRCON_FFI_DART_DL_H_
