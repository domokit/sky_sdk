// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/runtime/embedder_resources.h"

#include <cstring>

#include "flutter/fml/logging.h"

namespace flutter {

using runtime::ResourcesEntry;

EmbedderResources::EmbedderResources(ResourcesEntry* resources_table)
    : resources_table_(resources_table) {}

const int EmbedderResources::kNoSuchInstance = -1;

int EmbedderResources::ResourceLookup(const char* path, const char** resource) {
  for (int i = 0; resources_table_[i].path_ != nullptr; i++) {
    const ResourcesEntry& entry = resources_table_[i];
    if (strcmp(path, entry.path_) == 0) {
      *resource = entry.resource_;
      FML_DCHECK(entry.length_ > 0);
      return entry.length_;
    }
  }
  return kNoSuchInstance;
}

const char* EmbedderResources::Path(int idx) {
  FML_DCHECK(idx >= 0);
  ResourcesEntry* entry = At(idx);
  if (entry == nullptr) {
    return nullptr;
  }
  FML_DCHECK(entry->path_ != nullptr);
  return entry->path_;
}

ResourcesEntry* EmbedderResources::At(int idx) {
  FML_DCHECK(idx >= 0);
  for (int i = 0; resources_table_[i].path_ != nullptr; i++) {
    if (idx == i) {
      return &resources_table_[i];
    }
  }
  return nullptr;
}

}  // namespace flutter
