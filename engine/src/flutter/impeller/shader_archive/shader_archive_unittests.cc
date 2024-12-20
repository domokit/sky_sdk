// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <string>

#include "flutter/fml/mapping.h"
#include "flutter/testing/testing.h"
#include "impeller/shader_archive/shader_archive.h"
#include "impeller/shader_archive/shader_archive_writer.h"

namespace impeller {
namespace testing {

static std::shared_ptr<fml::Mapping> CreateMappingFromString(
    std::string p_string) {
  auto string = std::make_shared<std::string>(std::move(p_string));
  return std::make_shared<fml::NonOwnedMapping>(
      reinterpret_cast<const uint8_t*>(string->data()), string->size(),
      [string](auto, auto) {});
}

const std::string CreateStringFromMapping(const fml::Mapping& mapping) {
  return std::string{reinterpret_cast<const char*>(mapping.GetMapping()),
                     mapping.GetSize()};
}

TEST(ShaderArchiveTest, CanReadAndWriteBlobs) {
  ShaderArchiveWriter writer;
  ASSERT_TRUE(writer.AddShader(ArchiveShaderType::kVertex, "Hello",
                               CreateMappingFromString("World")));
  ASSERT_TRUE(writer.AddShader(ArchiveShaderType::kFragment, "Foo",
                               CreateMappingFromString("Bar")));
  ASSERT_TRUE(writer.AddShader(ArchiveShaderType::kVertex, "Baz",
                               CreateMappingFromString("Bang")));
  ASSERT_TRUE(writer.AddShader(ArchiveShaderType::kVertex, "Ping",
                               CreateMappingFromString("Pong")));
  ASSERT_TRUE(writer.AddShader(ArchiveShaderType::kFragment, "Pang",
                               CreateMappingFromString("World")));

  auto mapping = writer.CreateMapping();
  ASSERT_NE(mapping, nullptr);

  ShaderArchive library(mapping);
  ASSERT_TRUE(library.IsValid());
  ASSERT_EQ(library.GetShaderCount(), 5u);

  // Wrong type.
  ASSERT_EQ(library.GetMapping(ArchiveShaderType::kFragment, "Hello"), nullptr);

  auto hello_vtx = library.GetMapping(ArchiveShaderType::kVertex, "Hello");
  ASSERT_NE(hello_vtx, nullptr);
  ASSERT_EQ(CreateStringFromMapping(*hello_vtx), "World");
}

}  // namespace testing
}  // namespace impeller
