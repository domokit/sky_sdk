// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/connection_collection.h"

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

FLUTTER_ASSERT_ARC

namespace flutter {
ConnectionCollection::Connection ConnectionCollection::AquireConnection(const std::string& name) {
  Connection nextConnection = ++counter_;
  connections_[name] = nextConnection;
  return nextConnection;
}

std::string ConnectionCollection::CleanupConnection(ConnectionCollection::Connection connection) {
  if (connection > 0) {
    std::string channel;
    for (auto& keyValue : connections_) {
      if (keyValue.second == connection) {
        channel = keyValue.first;
        break;
      }
    }
    if (channel.length() > 0) {
      connections_.erase(channel);
      return channel;
    }
  }
  return "";
}

bool ConnectionCollection::IsValidConnection(ConnectionCollection::Connection connection) {
  return connection > 0;
}

ConnectionCollection::Connection ConnectionCollection::MakeErrorConnection(int errCode) {
  if (errCode < 0) {
    return -1 * errCode;
  }
  return errCode;
}

}  // namespace flutter
