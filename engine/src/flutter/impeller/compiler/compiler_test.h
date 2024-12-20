// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_COMPILER_COMPILER_TEST_H_
#define FLUTTER_IMPELLER_COMPILER_COMPILER_TEST_H_

#include "flutter/testing/testing.h"
#include "impeller/base/validation.h"
#include "impeller/compiler/compiler.h"
#include "impeller/compiler/source_options.h"
#include "impeller/compiler/types.h"

namespace impeller {
namespace compiler {
namespace testing {

class CompilerTest : public ::testing::TestWithParam<TargetPlatform> {
 public:
  CompilerTest();

  ~CompilerTest();

  std::unique_ptr<fml::FileMapping> GetReflectionJson(
      const char* fixture_name) const;

  std::unique_ptr<fml::FileMapping> GetShaderFile(
      const char* fixture_name,
      TargetPlatform platform) const;

  bool CanCompileAndReflect(
      const char* fixture_name,
      SourceType source_type = SourceType::kUnknown,
      SourceLanguage source_language = SourceLanguage::kGLSL,
      const char* entry_point_name = "main") const;

 private:
  std::string intermediates_path_;
  fml::UniqueFD intermediates_directory_;

  CompilerTest(const CompilerTest&) = delete;

  CompilerTest& operator=(const CompilerTest&) = delete;
};

}  // namespace testing
}  // namespace compiler
}  // namespace impeller

#endif  // FLUTTER_IMPELLER_COMPILER_COMPILER_TEST_H_
