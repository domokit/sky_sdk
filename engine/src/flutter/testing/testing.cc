// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "testing.h"

#include <utility>

#include "flutter/fml/file.h"
#include "flutter/fml/paths.h"

namespace flutter::testing {

std::string GetCurrentTestName() {
  return ::testing::UnitTest::GetInstance()->current_test_info()->name();
}

std::string GetDefaultKernelFilePath() {
  return fml::paths::JoinPaths({GetFixturesPath(), "kernel_blob.bin"});
}

fml::UniqueFD OpenFixturesDirectory() {
  auto fixtures_directory =
      OpenDirectory(GetFixturesPath(),          // path
                    false,                      // create
                    fml::FilePermission::kRead  // permission
      );

  if (!fixtures_directory.is_valid()) {
    FML_LOG(ERROR) << "Could not open fixtures directory.";
    return {};
  }
  return fixtures_directory;
}

fml::UniqueFD OpenFixture(const std::string& fixture_name) {
  if (fixture_name.size() == 0) {
    FML_LOG(ERROR) << "Invalid fixture name.";
    return {};
  }

  auto fixtures_directory = OpenFixturesDirectory();

  auto fixture_fd = fml::OpenFile(fixtures_directory,         // base directory
                                  fixture_name.c_str(),       // path
                                  false,                      // create
                                  fml::FilePermission::kRead  // permission
  );
  if (!fixture_fd.is_valid()) {
    FML_LOG(ERROR) << "Could not open fixture for path: " << GetFixturesPath()
                   << "/" << fixture_name << ".";
    return {};
  }

  return fixture_fd;
}

std::unique_ptr<fml::Mapping> OpenFixtureAsMapping(
    const std::string& fixture_name) {
  return fml::FileMapping::CreateReadOnly(OpenFixture(fixture_name));
}

sk_sp<SkData> OpenFixtureAsSkData(const std::string& fixture_name) {
  auto mapping = flutter::testing::OpenFixtureAsMapping(fixture_name);
  if (!mapping) {
    return nullptr;
  }
  auto data = SkData::MakeWithProc(
      mapping->GetMapping(), mapping->GetSize(),
      [](const void* ptr, void* context) {
        delete reinterpret_cast<fml::Mapping*>(context);
      },
      mapping.get());
  mapping.release();
  return data;
}

bool MemsetPatternSetOrCheck(uint8_t* buffer, size_t size, MemsetPatternOp op) {
  if (buffer == nullptr) {
    return false;
  }

  auto pattern = reinterpret_cast<const uint8_t*>("dErP");
  constexpr auto pattern_length = 4;

  uint8_t* start = buffer;
  uint8_t* p = buffer;

  while ((start + size) - p >= pattern_length) {
    switch (op) {
      case MemsetPatternOp::kMemsetPatternOpSetBuffer:
        memmove(p, pattern, pattern_length);
        break;
      case MemsetPatternOp::kMemsetPatternOpCheckBuffer:
        if (memcmp(pattern, p, pattern_length) != 0) {
          return false;
        }
        break;
    };
    p += pattern_length;
  }

  if ((start + size) - p != 0) {
    switch (op) {
      case MemsetPatternOp::kMemsetPatternOpSetBuffer:
        memmove(p, pattern, (start + size) - p);
        break;
      case MemsetPatternOp::kMemsetPatternOpCheckBuffer:
        if (memcmp(pattern, p, (start + size) - p) != 0) {
          return false;
        }
        break;
    }
  }

  return true;
}

bool MemsetPatternSetOrCheck(std::vector<uint8_t>& buffer, MemsetPatternOp op) {
  return MemsetPatternSetOrCheck(buffer.data(), buffer.size(), op);
}

}  // namespace flutter::testing
