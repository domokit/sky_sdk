// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_CONNECTION_COLLECTION_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_CONNECTION_COLLECTION_H_

#include <cstdint>
#include <map>
#include <string>

namespace flutter {

/// Maintains a current integer assigned to a name (connections).
class ConnectionCollection {
 public:
  typedef int64_t Connection;
  static const Connection kInvalidConnection = 0;

  Connection AquireConnection(const std::string& name);
  ///\returns the name of the channel when cleanup is successful, otherwise
  ///         the empty string.
  std::string CleanupConnection(Connection connection);

  static bool IsValidConnection(Connection connection);

  static Connection MakeErrorConnection(int errCode);

 private:
  std::map<std::string, Connection> connections_;
  Connection counter_ = 0;
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_CONNECTION_COLLECTION_H_
