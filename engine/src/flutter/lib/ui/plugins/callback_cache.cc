// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/plugins/callback_cache.h"

#include <fstream>
#include <iterator>

#include "flutter/fml/build_config.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/paths.h"
#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"
#include "third_party/tonic/converter/dart_converter.h"

using rapidjson::Document;
using rapidjson::StringBuffer;
using rapidjson::Writer;
using tonic::ToDart;

namespace flutter {

static const char* kHandleKey = "handle";
static const char* kRepresentationKey = "representation";
static const char* kNameKey = "name";
static const char* kClassNameKey = "class_name";
static const char* kLibraryPathKey = "library_path";
static const char* kCacheName = "flutter_callback_cache.json";
std::mutex DartCallbackCache::mutex_;
std::string DartCallbackCache::cache_path_;
std::map<int64_t, DartCallbackRepresentation> DartCallbackCache::cache_;

void DartCallbackCache::SetCachePath(const std::string& path) {
  cache_path_ = fml::paths::JoinPaths({path, kCacheName});
}

Dart_Handle DartCallbackCache::GetCallback(int64_t handle) {
  std::scoped_lock lock(mutex_);
  auto iterator = cache_.find(handle);
  if (iterator != cache_.end()) {
    DartCallbackRepresentation cb = iterator->second;
    return LookupDartClosure(cb.name, cb.class_name, cb.library_path);
  }
  return Dart_Null();
}

int64_t DartCallbackCache::GetCallbackHandle(const std::string& name,
                                             const std::string& class_name,
                                             const std::string& library_path) {
  std::scoped_lock lock(mutex_);
  std::hash<std::string> hasher;
  int64_t hash = hasher(name);
  hash += hasher(class_name);
  hash += hasher(library_path);

  if (cache_.find(hash) == cache_.end()) {
    cache_[hash] = {name, class_name, library_path};
    SaveCacheToDisk();
  }
  return hash;
}

std::unique_ptr<DartCallbackRepresentation>
DartCallbackCache::GetCallbackInformation(int64_t handle) {
  std::scoped_lock lock(mutex_);
  auto iterator = cache_.find(handle);
  if (iterator != cache_.end()) {
    return std::make_unique<DartCallbackRepresentation>(iterator->second);
  }
  return nullptr;
}

void DartCallbackCache::SaveCacheToDisk() {
  // Cache JSON format
  // [
  //   {
  //      "hash": 42,
  //      "representation": {
  //          "name": "...",
  //          "class_name": "...",
  //          "library_path": "..."
  //      }
  //   },
  //   {
  //   ...
  //   }
  // ]
  StringBuffer s;
  Writer<StringBuffer> writer(s);
  writer.StartArray();
  for (auto iterator = cache_.begin(); iterator != cache_.end(); ++iterator) {
    int64_t hash = iterator->first;
    DartCallbackRepresentation cb = iterator->second;
    writer.StartObject();
    writer.Key(kHandleKey);
    writer.Int64(hash);
    writer.Key(kRepresentationKey);
    writer.StartObject();
    writer.Key(kNameKey);
    writer.String(cb.name.c_str());
    writer.Key(kClassNameKey);
    writer.String(cb.class_name.c_str());
    writer.Key(kLibraryPathKey);
    writer.String(cb.library_path.c_str());
    writer.EndObject();
    writer.EndObject();
  }
  writer.EndArray();

  std::ofstream output(cache_path_);
  output << s.GetString();
  output.close();
}

void DartCallbackCache::LoadCacheFromDisk() {
  std::scoped_lock lock(mutex_);

  // Don't reload the cache if it's already populated.
  if (!cache_.empty()) {
    return;
  }
  std::ifstream input(cache_path_);
  if (!input) {
    return;
  }
  std::string cache_contents{std::istreambuf_iterator<char>(input),
                             std::istreambuf_iterator<char>()};
  Document d;
  d.Parse(cache_contents.c_str());
  if (d.HasParseError() || !d.IsArray()) {
    // Could not parse callback cache, aborting restore.
    return;
  }
  const auto entries = d.GetArray();
  for (auto* it = entries.begin(); it != entries.end(); ++it) {
    const auto root_obj = it->GetObject();
    const auto representation = root_obj[kRepresentationKey].GetObject();

    const int64_t hash = root_obj[kHandleKey].GetInt64();
    DartCallbackRepresentation cb;
    cb.name = representation[kNameKey].GetString();
    cb.class_name = representation[kClassNameKey].GetString();
    cb.library_path = representation[kLibraryPathKey].GetString();
    cache_[hash] = cb;
  }
}

Dart_Handle DartCallbackCache::LookupDartClosure(
    const std::string& name,
    const std::string& class_name,
    const std::string& library_path) {
  Dart_Handle closure_name = ToDart(name);
  if (Dart_IsError(closure_name)) {
    return closure_name;
  }
  Dart_Handle library_name =
      library_path.empty() ? Dart_Null() : ToDart(library_path);
  if (Dart_IsError(library_name)) {
    return library_name;
  }
  Dart_Handle cls_name = class_name.empty() ? Dart_Null() : ToDart(class_name);
  if (Dart_IsError(cls_name)) {
    return cls_name;
  }

  Dart_Handle library;
  if (library_name == Dart_Null()) {
    library = Dart_RootLibrary();
  } else {
    library = Dart_LookupLibrary(library_name);
  }
  if (Dart_IsError(library)) {
    return library;
  }

  Dart_Handle closure;
  if (Dart_IsNull(cls_name)) {
    closure = Dart_GetField(library, closure_name);
  } else {
    Dart_Handle cls = Dart_GetClass(library, cls_name);
    if (Dart_IsError(cls)) {
      return cls;
    }
    if (Dart_IsNull(cls)) {
      closure = Dart_Null();
    } else {
      closure = Dart_GetStaticMethodClosure(library, cls, closure_name);
    }
  }
  return closure;
}

}  // namespace flutter
