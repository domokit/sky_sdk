// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_ISOLATE_NAME_SERVER_ISOLATE_NAME_SERVER_H_
#define FLUTTER_LIB_UI_ISOLATE_NAME_SERVER_ISOLATE_NAME_SERVER_H_

#include <map>
#include <mutex>
#include <string>

#include "flutter/fml/macros.h"
#include "third_party/dart/runtime/include/dart_api.h"

namespace flutter {

class IsolateNameServer {
 public:
  IsolateNameServer();

  ~IsolateNameServer();

  // Looks up the Dart_Port associated with a given name. Returns ILLEGAL_PORT
  // if the name does not exist.
  Dart_PortEx LookupIsolatePortByName(const std::string& name);

  // Registers a Dart_Port with a given name. Returns true if registration is
  // successful, false if the name entry already exists.
  bool RegisterIsolatePortWithName(Dart_PortEx port, const std::string& name);

  // Removes a name to Dart_Port mapping given a name. Returns true if the
  // mapping was successfully removed, false if the mapping does not exist.
  bool RemoveIsolateNameMapping(const std::string& name);

 private:
  Dart_PortEx LookupIsolatePortByNameUnprotected(const std::string& name);

  mutable std::mutex mutex_;
  std::map<std::string, Dart_PortEx> port_mapping_;

  FML_DISALLOW_COPY_AND_ASSIGN(IsolateNameServer);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_ISOLATE_NAME_SERVER_ISOLATE_NAME_SERVER_H_
