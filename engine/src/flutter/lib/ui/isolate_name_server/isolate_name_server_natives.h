// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_ISOLATE_NAME_SERVER_ISOLATE_NAME_SERVER_NATIVES_H_
#define FLUTTER_LIB_UI_ISOLATE_NAME_SERVER_ISOLATE_NAME_SERVER_NATIVES_H_

#include <string>

#include "third_party/dart/runtime/include/dart_api.h"

namespace flutter {

class IsolateNameServerNatives {
 public:
  static Dart_Handle LookupPortByName(const std::string& name);
  static bool RegisterPortWithName(Dart_Handle port_handle,
                                   const std::string& name);
  static bool RemovePortNameMapping(const std::string& name);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_ISOLATE_NAME_SERVER_ISOLATE_NAME_SERVER_NATIVES_H_
