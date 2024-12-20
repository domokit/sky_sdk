// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_GOLDEN_TESTS_GOLDEN_DIGEST_H_
#define FLUTTER_IMPELLER_GOLDEN_TESTS_GOLDEN_DIGEST_H_

#include <map>
#include <string>
#include <vector>

#include "flutter/impeller/golden_tests/working_directory.h"

namespace impeller {
namespace testing {

/// Manages a global variable for tracking instances of golden images.
class GoldenDigest {
 public:
  static GoldenDigest* Instance();

  void AddDimension(const std::string& name, const std::string& value);

  void AddImage(const std::string& test_name,
                const std::string& filename,
                int32_t width,
                int32_t height);

  /// Writes a "digest.json" file to `working_directory`.
  ///
  /// Returns `true` on success.
  bool Write(WorkingDirectory* working_directory);

 private:
  GoldenDigest(const GoldenDigest&) = delete;

  GoldenDigest& operator=(const GoldenDigest&) = delete;
  GoldenDigest();
  struct Entry {
    std::string test_name;
    std::string filename;
    int32_t width;
    int32_t height;
    double max_diff_pixels_percent;
    int32_t max_color_delta;
  };

  static GoldenDigest* instance_;
  std::vector<Entry> entries_;
  std::map<std::string, std::string> dimensions_;
};
}  // namespace testing
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_GOLDEN_TESTS_GOLDEN_DIGEST_H_
