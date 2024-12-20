// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_RUNTIME_EMBEDDER_RESOURCES_H_
#define FLUTTER_RUNTIME_EMBEDDER_RESOURCES_H_

namespace flutter {
namespace runtime {

struct ResourcesEntry {
  const char* path_;
  const char* resource_;
  int length_;
};

}  // namespace runtime
}  // namespace flutter

namespace flutter {

class EmbedderResources {
 public:
  explicit EmbedderResources(runtime::ResourcesEntry* resources_table);

  static const int kNoSuchInstance;

  int ResourceLookup(const char* path, const char** resource);
  const char* Path(int idx);

 private:
  runtime::ResourcesEntry* At(int idx);

  runtime::ResourcesEntry* resources_table_;
};

}  // namespace flutter

#endif  // FLUTTER_RUNTIME_EMBEDDER_RESOURCES_H_
