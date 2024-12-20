// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "component_v2.h"

#include <gtest/gtest.h>
#include <optional>

namespace flutter_runner {
namespace {

TEST(ComponentV2, ParseProgramMetadataDefaultAssets) {
  // The assets_path defaults to the "data" value if unspecified.
  std::vector<fuchsia::data::DictionaryEntry> entries;

  fuchsia::data::DictionaryEntry data_entry;
  data_entry.key = "data";
  data_entry.value = std::make_unique<fuchsia::data::DictionaryValue>(
      fuchsia::data::DictionaryValue::WithStr("foobar"));
  entries.push_back(std::move(data_entry));

  fuchsia::data::Dictionary program_metadata;
  program_metadata.set_entries(std::move(entries));

  ProgramMetadata result = ComponentV2::ParseProgramMetadata(program_metadata);

  EXPECT_EQ(result.data_path, "pkg/foobar");
  EXPECT_EQ(result.assets_path, "pkg/foobar");
  EXPECT_EQ(result.old_gen_heap_size, std::nullopt);
}

TEST(ComponentV2, ParseProgramMetadataIgnoreInvalidKeys) {
  // Invalid keys are ignored.
  std::vector<fuchsia::data::DictionaryEntry> entries;

  fuchsia::data::DictionaryEntry not_data_entry;
  not_data_entry.key = "not_data";
  not_data_entry.value = std::make_unique<fuchsia::data::DictionaryValue>(
      fuchsia::data::DictionaryValue::WithStr("foo"));
  entries.push_back(std::move(not_data_entry));

  fuchsia::data::DictionaryEntry data_entry;
  data_entry.key = "data";
  data_entry.value = std::make_unique<fuchsia::data::DictionaryValue>(
      fuchsia::data::DictionaryValue::WithStr("bar"));
  entries.push_back(std::move(data_entry));

  fuchsia::data::DictionaryEntry assets_entry;
  assets_entry.key = "assets";
  assets_entry.value = std::make_unique<fuchsia::data::DictionaryValue>(
      fuchsia::data::DictionaryValue::WithStr("baz"));
  entries.push_back(std::move(assets_entry));

  fuchsia::data::Dictionary program_metadata;
  program_metadata.set_entries(std::move(entries));

  ProgramMetadata result = ComponentV2::ParseProgramMetadata(program_metadata);

  EXPECT_EQ(result.data_path, "pkg/bar");
  EXPECT_EQ(result.assets_path, "pkg/baz");
  EXPECT_EQ(result.old_gen_heap_size, std::nullopt);
}

TEST(ComponentV2, ParseProgramMetadataOldGenHeapSize) {
  // The old_gen_heap_size can be specified.
  std::vector<fuchsia::data::DictionaryEntry> entries;

  fuchsia::data::DictionaryEntry args_entry;
  args_entry.key = "args";
  args_entry.value = std::make_unique<fuchsia::data::DictionaryValue>(
      fuchsia::data::DictionaryValue::WithStrVec({"--old_gen_heap_size=100"}));
  entries.push_back(std::move(args_entry));

  fuchsia::data::Dictionary program_metadata;
  program_metadata.set_entries(std::move(entries));

  ProgramMetadata result = ComponentV2::ParseProgramMetadata(program_metadata);

  EXPECT_EQ(result.data_path, "");
  EXPECT_EQ(result.assets_path, "");
  EXPECT_EQ(result.old_gen_heap_size, 100);
}

TEST(ComponentV2, ParseProgramMetadataOldGenHeapSizeInvalid) {
  // Invalid old_gen_heap_sizes should be ignored.
  std::vector<fuchsia::data::DictionaryEntry> entries;

  fuchsia::data::DictionaryEntry args_entry;
  args_entry.key = "args";
  args_entry.value = std::make_unique<fuchsia::data::DictionaryValue>(
      fuchsia::data::DictionaryValue::WithStrVec(
          {"--old_gen_heap_size=asdf100"}));
  entries.push_back(std::move(args_entry));

  fuchsia::data::Dictionary program_metadata;
  program_metadata.set_entries(std::move(entries));

  ProgramMetadata result = ComponentV2::ParseProgramMetadata(program_metadata);

  EXPECT_EQ(result.data_path, "");
  EXPECT_EQ(result.assets_path, "");
  EXPECT_EQ(result.old_gen_heap_size, std::nullopt);
}

TEST(ComponentV2, ParseProgramMetadataNoExposeDirs) {
  // Should always have a valid expose_dirs entry
  std::vector<fuchsia::data::DictionaryEntry> entries;

  fuchsia::data::Dictionary program_metadata;
  program_metadata.set_entries(std::move(entries));

  ProgramMetadata result = ComponentV2::ParseProgramMetadata(program_metadata);

  EXPECT_EQ(result.expose_dirs.empty(), true);
}

TEST(ComponentV2, ParseProgramMetadataWithSingleExposeDirs) {
  // Can parse a single exposed dir
  std::vector<fuchsia::data::DictionaryEntry> entries;

  fuchsia::data::DictionaryEntry args_entry;
  args_entry.key = "args";
  args_entry.value = std::make_unique<fuchsia::data::DictionaryValue>(
      fuchsia::data::DictionaryValue::WithStrVec({"--expose_dirs=foo"}));
  entries.push_back(std::move(args_entry));

  fuchsia::data::Dictionary program_metadata;
  program_metadata.set_entries(std::move(entries));

  ProgramMetadata result = ComponentV2::ParseProgramMetadata(program_metadata);

  ASSERT_EQ(result.expose_dirs.size(), 1u);
  EXPECT_EQ(result.expose_dirs[0], "foo");
}

TEST(ComponentV2, ParseProgramMetadataWithMultipleExposeDirs) {
  // Can parse a multiple exposed dirs
  std::vector<fuchsia::data::DictionaryEntry> entries;

  fuchsia::data::DictionaryEntry args_entry;
  args_entry.key = "args";
  args_entry.value = std::make_unique<fuchsia::data::DictionaryValue>(
      fuchsia::data::DictionaryValue::WithStrVec(
          {"--expose_dirs=foo,bar,baz"}));
  entries.push_back(std::move(args_entry));

  fuchsia::data::Dictionary program_metadata;
  program_metadata.set_entries(std::move(entries));

  ProgramMetadata result = ComponentV2::ParseProgramMetadata(program_metadata);

  ASSERT_EQ(result.expose_dirs.size(), 3u);
  EXPECT_EQ(result.expose_dirs[0], "foo");
  EXPECT_EQ(result.expose_dirs[1], "bar");
  EXPECT_EQ(result.expose_dirs[2], "baz");
}

TEST(ComponentV2, ParseProgramMetadataWithSingleExposeDirsAndOtherArgs) {
  // Can parse a single exposed dir when other args are present
  std::vector<fuchsia::data::DictionaryEntry> entries;

  fuchsia::data::DictionaryEntry args_entry;
  args_entry.key = "args";
  args_entry.value = std::make_unique<fuchsia::data::DictionaryValue>(
      fuchsia::data::DictionaryValue::WithStrVec(
          {"--expose_dirs=foo", "--foo=bar"}));
  entries.push_back(std::move(args_entry));

  fuchsia::data::Dictionary program_metadata;
  program_metadata.set_entries(std::move(entries));

  ProgramMetadata result = ComponentV2::ParseProgramMetadata(program_metadata);

  ASSERT_EQ(result.expose_dirs.size(), 1u);
  EXPECT_EQ(result.expose_dirs[0], "foo");
}

}  // anonymous namespace
}  // namespace flutter_runner
