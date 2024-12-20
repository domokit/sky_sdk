// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/isolate_name_server/isolate_name_server_natives.h"

#include <string>

#include "flutter/lib/ui/isolate_name_server/isolate_name_server.h"
#include "flutter/lib/ui/ui_dart_state.h"
#include "third_party/tonic/dart_binding_macros.h"
#include "third_party/tonic/dart_library_natives.h"

namespace flutter {

Dart_Handle IsolateNameServerNatives::LookupPortByName(
    const std::string& name) {
  auto name_server = UIDartState::Current()->GetIsolateNameServer();
  if (!name_server) {
    return Dart_Null();
  }
  Dart_PortEx port = name_server->LookupIsolatePortByName(name);
  if (port.port_id == ILLEGAL_PORT) {
    return Dart_Null();
  }
  return Dart_NewSendPortEx(port);
}

bool IsolateNameServerNatives::RegisterPortWithName(Dart_Handle port_handle,
                                                    const std::string& name) {
  auto name_server = UIDartState::Current()->GetIsolateNameServer();
  if (!name_server) {
    return false;
  }
  Dart_PortEx port;
  Dart_SendPortGetIdEx(port_handle, &port);
  if (!name_server->RegisterIsolatePortWithName(port, name)) {
    return false;
  }
  return true;
}

bool IsolateNameServerNatives::RemovePortNameMapping(const std::string& name) {
  auto name_server = UIDartState::Current()->GetIsolateNameServer();
  if (!name_server || !name_server->RemoveIsolateNameMapping(name)) {
    return false;
  }
  return true;
}

}  // namespace flutter
