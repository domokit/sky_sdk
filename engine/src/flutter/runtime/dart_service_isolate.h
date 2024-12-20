// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_DART_SERVICE_ISOLATE_H_
#define FLUTTER_RUNTIME_DART_SERVICE_ISOLATE_H_

#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "third_party/dart/runtime/include/dart_api.h"

namespace flutter {

//------------------------------------------------------------------------------
/// @brief      Utility methods for interacting with the DartVM managed service
///             isolate present in debug and profile runtime modes.
///
class DartServiceIsolate {
 public:
  //----------------------------------------------------------------------------
  /// The handle used to refer to callbacks registered with the service isolate.
  ///
  using CallbackHandle = ptrdiff_t;

  //----------------------------------------------------------------------------
  /// A callback made by the Dart VM when the VM Service is ready. The argument
  /// indicates the VM Service URI.
  ///
  using DartVMServiceServerStateCallback =
      std::function<void(const std::string& vm_service_uri)>;

  //----------------------------------------------------------------------------
  /// @brief      Start the service isolate. This call may only be made in the
  ///             Dart VM initiated isolate creation callback. It is only valid
  ///             to make this call when the VM explicitly requests the creation
  ///             of the service isolate. The VM does this by specifying the
  ///             script URI to be `DART_VM_SERVICE_ISOLATE_NAME`. The isolate
  ///             to be designated as the service isolate must already be
  ///             created (but not running) when this call is made.
  ///
  /// @param[in]  server_ip                     The service protocol IP address.
  /// @param[in]  server_port                   The service protocol port.
  /// @param[in]  embedder_tag_handler          The library tag handler.
  /// @param[in]  disable_origin_check          If websocket origin checks must
  ///                                           be enabled.
  /// @param[in]  disable_service_auth_codes    If service auth codes must be
  ///                                           enabled.
  /// @param[in]  enable_service_port_fallback  If fallback to port 0 must be
  ///                                           enabled when the bind fails.
  /// @param      error                         The error when this method
  ///                                           returns false. This string must
  ///                                           be freed by the caller using
  ///                                           `free`.
  ///
  /// @return     If the startup was successful. Refer to the `error` for
  ///             details on failure.
  ///
  static bool Startup(const std::string& server_ip,
                      intptr_t server_port,
                      Dart_LibraryTagHandler embedder_tag_handler,
                      bool disable_origin_check,
                      bool disable_service_auth_codes,
                      bool enable_service_port_fallback,
                      char** error);

  //----------------------------------------------------------------------------
  /// @brief      Add a callback that will get invoked when the VM Service
  ///             starts up. If the VM Service has already started before this
  ///             call is made, the callback is invoked immediately.
  ///
  ///             This method is thread safe.
  ///
  /// @param[in]  callback  The callback with information about the VM Service.
  ///
  /// @return     A handle for the callback that can be used later in
  ///             `RemoveServerStatusCallback`.
  ///
  [[nodiscard]] static CallbackHandle AddServerStatusCallback(
      const DartVMServiceServerStateCallback& callback);

  //----------------------------------------------------------------------------
  /// @brief      Removed a callback previously registered via
  ///             `AddServiceStatusCallback`.
  ///
  ///             This method is thread safe.
  ///
  /// @param[in]  handle  The handle
  ///
  /// @return     If the callback was unregistered. This may fail if there was
  ///             no such callback with that handle.
  ///
  static bool RemoveServerStatusCallback(CallbackHandle handle);

 private:
  // Native entries.
  static void NotifyServerState(Dart_NativeArguments args);
  static void Shutdown(Dart_NativeArguments args);

  static std::mutex callbacks_mutex_;
  static std::set<std::unique_ptr<DartVMServiceServerStateCallback>> callbacks_;
};

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_DART_SERVICE_ISOLATE_H_
