// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef LIB_TONIC_PARSERS_PACKAGES_MAP_H_
#define LIB_TONIC_PARSERS_PACKAGES_MAP_H_

#include <string>
#include <unordered_map>

namespace tonic {

class PackagesMap {
 public:
  PackagesMap();
  ~PackagesMap();

  bool Parse(const std::string& source, std::string* error);
  std::string Resolve(const std::string& package_name);

 private:
  std::unordered_map<std::string, std::string> map_;
};

}  // namespace tonic

#endif  // LIB_TONIC_PARSERS_PACKAGES_MAP_H_
