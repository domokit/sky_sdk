// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/golden_tests/golden_digest.h"

#include <fstream>
#include <sstream>

static const double kMaxDiffPixelsPercent = 0.01;
static const int32_t kMaxColorDelta = 8;

namespace impeller {
namespace testing {

GoldenDigest* GoldenDigest::instance_ = nullptr;

GoldenDigest* GoldenDigest::Instance() {
  if (!instance_) {
    instance_ = new GoldenDigest();
  }
  return instance_;
}

GoldenDigest::GoldenDigest() {}

void GoldenDigest::AddDimension(const std::string& name,
                                const std::string& value) {
  std::stringstream ss;
  ss << "\"" << value << "\"";
  dimensions_[name] = ss.str();
}

void GoldenDigest::AddImage(const std::string& test_name,
                            const std::string& filename,
                            int32_t width,
                            int32_t height) {
  entries_.push_back({test_name, filename, width, height, kMaxDiffPixelsPercent,
                      kMaxColorDelta});
}

bool GoldenDigest::Write(WorkingDirectory* working_directory) {
  std::ofstream fout;
  fout.open(working_directory->GetFilenamePath("digest.json"));
  if (!fout.good()) {
    return false;
  }

  fout << "{" << std::endl;
  fout << "  \"dimensions\": {" << std::endl;
  bool is_first = true;
  for (const auto& dimension : dimensions_) {
    if (!is_first) {
      fout << "," << std::endl;
    }
    is_first = false;
    fout << "    \"" << dimension.first << "\": " << dimension.second;
  }
  fout << std::endl << "  }," << std::endl;
  fout << "  \"entries\":" << std::endl;

  fout << "  [" << std::endl;
  is_first = true;
  for (const auto& entry : entries_) {
    if (!is_first) {
      fout << "," << std::endl;
    }
    is_first = false;

    fout << "    { " << "\"testName\" : \"" << entry.test_name << "\", "
         << "\"filename\" : \"" << entry.filename << "\", "
         << "\"width\" : " << entry.width << ", "
         << "\"height\" : " << entry.height << ", ";

    if (entry.max_diff_pixels_percent ==
        static_cast<int64_t>(entry.max_diff_pixels_percent)) {
      fout << "\"maxDiffPixelsPercent\" : " << entry.max_diff_pixels_percent
           << ".0, ";
    } else {
      fout << "\"maxDiffPixelsPercent\" : " << entry.max_diff_pixels_percent
           << ", ";
    }

    fout << "\"maxColorDelta\":" << entry.max_color_delta << " ";
    fout << "}";
  }
  fout << std::endl << "  ]" << std::endl;

  fout << "}" << std::endl;

  fout.close();
  return true;
}

}  // namespace testing
}  // namespace impeller
