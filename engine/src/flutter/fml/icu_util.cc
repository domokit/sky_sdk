// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/icu_util.h"

#include <memory>
#include <mutex>

#include "flutter/fml/build_config.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/native_library.h"
#include "flutter/fml/paths.h"
#include "third_party/icu/source/common/unicode/udata.h"

namespace fml {
namespace icu {

class ICUContext {
 public:
  explicit ICUContext(const std::string& icu_data_path) : valid_(false) {
    valid_ = SetupMapping(icu_data_path) && SetupICU();
  }

  explicit ICUContext(std::unique_ptr<Mapping> mapping)
      : mapping_(std::move(mapping)) {
    valid_ = SetupICU();
  }

  ~ICUContext() = default;

  bool SetupMapping(const std::string& icu_data_path) {
    // Check if the path exists and it readable directly.
    auto fd =
        fml::OpenFile(icu_data_path.c_str(), false, fml::FilePermission::kRead);

    // Check the path relative to the current executable.
    if (!fd.is_valid()) {
      auto directory = fml::paths::GetExecutableDirectoryPath();

      if (!directory.first) {
        return false;
      }

      std::string path_relative_to_executable =
          paths::JoinPaths({directory.second, icu_data_path});

      fd = fml::OpenFile(path_relative_to_executable.c_str(), false,
                         fml::FilePermission::kRead);
    }

    if (!fd.is_valid()) {
      return false;
    }

    std::initializer_list<FileMapping::Protection> protection = {
        fml::FileMapping::Protection::kRead};

    auto file_mapping = std::make_unique<FileMapping>(fd, protection);

    if (file_mapping->GetSize() != 0) {
      mapping_ = std::move(file_mapping);
      return true;
    }

    return false;
  }

  bool SetupICU() {
    if (GetSize() == 0) {
      return false;
    }

    UErrorCode err_code = U_ZERO_ERROR;
    udata_setCommonData(GetMapping(), &err_code);
    return (err_code == U_ZERO_ERROR);
  }

  const uint8_t* GetMapping() const {
    return mapping_ ? mapping_->GetMapping() : nullptr;
  }

  size_t GetSize() const { return mapping_ ? mapping_->GetSize() : 0; }

  bool IsValid() const { return valid_; }

 private:
  bool valid_;
  std::unique_ptr<Mapping> mapping_;

  FML_DISALLOW_COPY_AND_ASSIGN(ICUContext);
};

void InitializeICUOnce(const std::string& icu_data_path) {
  static ICUContext* context = new ICUContext(icu_data_path);
  FML_CHECK(context->IsValid())
      << "Must be able to initialize the ICU context. Tried: " << icu_data_path;
}

std::once_flag g_icu_init_flag;
void InitializeICU(const std::string& icu_data_path) {
  std::call_once(g_icu_init_flag,
                 [&icu_data_path]() { InitializeICUOnce(icu_data_path); });
}

void InitializeICUFromMappingOnce(std::unique_ptr<Mapping> mapping) {
  static ICUContext* context = new ICUContext(std::move(mapping));
  FML_CHECK(context->IsValid())
      << "Unable to initialize the ICU context from a mapping.";
}

void InitializeICUFromMapping(std::unique_ptr<Mapping> mapping) {
  std::call_once(g_icu_init_flag, [mapping = std::move(mapping)]() mutable {
    InitializeICUFromMappingOnce(std::move(mapping));
  });
}

}  // namespace icu
}  // namespace fml
