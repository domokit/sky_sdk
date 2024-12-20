/*
 * Copyright 2017 Google, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "flutter/fml/command_line.h"
#include "flutter/fml/icu_util.h"
#include "flutter/fml/logging.h"
#include "flutter/testing/testing.h"
#include "flutter/third_party/txt/tests/txt_test_utils.h"
#include "third_party/benchmark/include/benchmark/benchmark.h"

// We will use a custom main to allow custom font directories for consistency.
int main(int argc, char** argv) {
  ::benchmark::Initialize(&argc, argv);
  fml::CommandLine cmd = fml::CommandLineFromPlatformOrArgcArgv(argc, argv);
  txt::SetFontDir(flutter::testing::GetFixturesPath());
  if (txt::GetFontDir().length() <= 0) {
    FML_LOG(ERROR) << "Font directory not set via txt::SetFontDir.";
    return EXIT_FAILURE;
  }
  FML_DCHECK(txt::GetFontDir().length() > 0);

  std::string icudtl_path =
      cmd.GetOptionValueWithDefault("icu-data-file-path", "icudtl.dat");
  fml::icu::InitializeICU(icudtl_path);

  ::benchmark::RunSpecifiedBenchmarks();
}
