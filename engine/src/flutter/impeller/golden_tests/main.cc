// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <wordexp.h>

#include "flutter/fml/backtrace.h"
#include "flutter/fml/build_config.h"
#include "flutter/fml/command_line.h"
#include "flutter/fml/logging.h"
#include "flutter/impeller/base/validation.h"
#include "flutter/impeller/golden_tests/golden_digest.h"
#include "flutter/impeller/golden_tests/working_directory.h"
#include "gtest/gtest.h"

namespace {
void print_usage() {
  std::cout << "usage: impeller_golden_tests --working_dir=<working_dir>"
            << std::endl
            << std::endl;
  std::cout << "flags:" << std::endl;
  std::cout << "  working_dir: Where the golden images will be generated and "
               "uploaded to Skia Gold from."
            << std::endl;
}
}  // namespace

namespace impeller {
TEST(ValidationTest, IsFatal) {
  EXPECT_TRUE(ImpellerValidationErrorsAreFatal());
}
}  // namespace impeller

int main(int argc, char** argv) {
  impeller::ImpellerValidationErrorsSetFatal(true);
  fml::InstallCrashHandler();
  testing::InitGoogleTest(&argc, argv);
  fml::CommandLine cmd = fml::CommandLineFromPlatformOrArgcArgv(argc, argv);

  std::optional<std::string> working_dir;
  for (const auto& option : cmd.options()) {
    if (option.name == "working_dir") {
      wordexp_t wordexp_result;
      int code = wordexp(option.value.c_str(), &wordexp_result, 0);
      FML_CHECK(code == 0);
      FML_CHECK(wordexp_result.we_wordc != 0);
      working_dir = wordexp_result.we_wordv[0];
      wordfree(&wordexp_result);
    }
  }
  if (!working_dir) {
    std::cout << "required argument \"working_dir\" is missing." << std::endl
              << std::endl;
    print_usage();
    return 1;
  }

  impeller::testing::WorkingDirectory::Instance()->SetPath(working_dir.value());
  std::cout << "working directory: "
            << impeller::testing::WorkingDirectory::Instance()->GetPath()
            << std::endl;

  int return_code = RUN_ALL_TESTS();
  if (0 == return_code) {
    impeller::testing::GoldenDigest::Instance()->Write(
        impeller::testing::WorkingDirectory::Instance());
  }
  return return_code;
}
