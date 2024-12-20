// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "tonic/file_loader/file_loader.h"

#include <iostream>
#include <memory>
#include <utility>

#include "tonic/common/macros.h"
#include "tonic/converter/dart_converter.h"
#include "tonic/filesystem/filesystem/file.h"
#include "tonic/filesystem/filesystem/path.h"
#include "tonic/filesystem/filesystem/portable_unistd.h"
#include "tonic/parsers/packages_map.h"

namespace tonic {
namespace {

constexpr char kDartScheme[] = "dart:";

constexpr char kFileScheme[] = "file:";
constexpr size_t kFileSchemeLength = sizeof(kFileScheme) - 1;

constexpr char kPackageScheme[] = "package:";
constexpr size_t kPackageSchemeLength = sizeof(kPackageScheme) - 1;

// Extract the scheme prefix ('package:' or 'file:' from )
std::string ExtractSchemePrefix(std::string url) {
  if (url.find(kPackageScheme) == 0u)
    return kPackageScheme;
  if (url.find(kFileScheme) == 0u)
    return kFileScheme;
  return std::string();
}

// Extract the path from a package: or file: url.
std::string ExtractPath(std::string url) {
  if (url.find(kPackageScheme) == 0u)
    return url.substr(kPackageSchemeLength);
  if (url.find(kFileScheme) == 0u)
    return url.substr(kFileSchemeLength);
  return url;
}

}  // namespace

FileLoader::FileLoader(int dirfd) : dirfd_(dirfd) {}

FileLoader::~FileLoader() {
  for (auto kernel_buffer : kernel_buffers_)
    free(kernel_buffer);

  if (dirfd_ >= 0)
    close(dirfd_);
}

std::string FileLoader::SanitizeURIEscapedCharacters(const std::string& str) {
  std::string result;
  result.reserve(str.size());
  for (std::string::size_type i = 0; i < str.size(); ++i) {
    if (str[i] == '%') {
      if (i > str.size() - 3 || !isxdigit(str[i + 1]) || !isxdigit(str[i + 2]))
        return "";
      const std::string hex = str.substr(i + 1, 2);
      const unsigned char c = strtoul(hex.c_str(), nullptr, 16);
      if (!c)
        return "";
      result += c;
      i += 2;
    } else {
      result += str[i];
    }
  }
  return result;
}

bool FileLoader::LoadPackagesMap(const std::string& packages) {
  packages_ = packages;
  std::string packages_source;
  if (!ReadFileToString(packages_, &packages_source)) {
    tonic::Log("error: Unable to load .packages file '%s'.", packages_.c_str());
    return false;
  }
  packages_map_.reset(new PackagesMap());
  std::string error;
  if (!packages_map_->Parse(packages_source, &error)) {
    tonic::Log("error: Unable to parse .packages file '%s'. %s",
               packages_.c_str(), error.c_str());
    return false;
  }
  return true;
}

std::string FileLoader::GetFilePathForPackageURL(std::string url) {
  if (!packages_map_)
    return std::string();
  TONIC_DCHECK(url.find(kPackageScheme) == 0u);
  url = url.substr(kPackageSchemeLength);

  size_t slash = url.find(FileLoader::kPathSeparator);
  if (slash == std::string::npos)
    return std::string();
  std::string package = url.substr(0, slash);
  std::string library_path = url.substr(slash + 1);
  std::string package_path = packages_map_->Resolve(package);
  if (package_path.empty())
    return std::string();
  if (package_path.find(FileLoader::kFileURLPrefix) == 0u)
    return SanitizePath(package_path.substr(FileLoader::kFileURLPrefixLength) +
                        library_path);
  return filesystem::GetDirectoryName(filesystem::AbsolutePath(packages_)) +
         FileLoader::kPathSeparator + package_path +
         FileLoader::kPathSeparator + library_path;
}

Dart_Handle FileLoader::HandleLibraryTag(Dart_LibraryTag tag,
                                         Dart_Handle library,
                                         Dart_Handle url) {
  TONIC_DCHECK(Dart_IsNull(library) || Dart_IsLibrary(library) ||
               Dart_IsString(library));
  TONIC_DCHECK(Dart_IsString(url));
  if (tag == Dart_kCanonicalizeUrl)
    return CanonicalizeURL(library, url);
  if (tag == Dart_kKernelTag)
    return Kernel(url);
  if (tag == Dart_kImportTag)
    return Import(url);
  return Dart_NewApiError("Unknown library tag.");
}

Dart_Handle FileLoader::CanonicalizeURL(Dart_Handle library, Dart_Handle url) {
  std::string string = StdStringFromDart(url);
  if (string.find(kDartScheme) == 0u)
    return url;
  if (string.find(kPackageScheme) == 0u)
    return StdStringToDart(SanitizePath(string));

  if (string.find(kFileScheme) == 0u) {
    return StdStringToDart(SanitizeURIEscapedCharacters(string));
  }

  std::string library_url = StdStringFromDart(Dart_LibraryUrl(library));
  std::string prefix = ExtractSchemePrefix(library_url);
  std::string base_path = ExtractPath(library_url);
  std::string simplified_path =
      filesystem::SimplifyPath(filesystem::GetDirectoryName(base_path) +
                               FileLoader::kPathSeparator + string);
  return StdStringToDart(SanitizePath(prefix + simplified_path));
}

std::string FileLoader::GetFilePathForURL(std::string url) {
  if (url.find(kPackageScheme) == 0u)
    return GetFilePathForPackageURL(std::move(url));
  if (url.find(kFileScheme) == 0u)
    return GetFilePathForFileURL(std::move(url));
  return url;
}

Dart_Handle FileLoader::FetchBytes(const std::string& url,
                                   uint8_t*& buffer,
                                   intptr_t& buffer_size) {
  buffer = nullptr;
  buffer_size = -1;

  std::string path = filesystem::SimplifyPath(GetFilePathForURL(url));
  if (path.empty()) {
    std::string error_message = "error: Unable to read '" + url + "'.";
    return Dart_NewUnhandledExceptionError(
        Dart_NewStringFromCString(error_message.c_str()));
  }
  std::string absolute_path = filesystem::GetAbsoluteFilePath(path);
  auto result = filesystem::ReadFileToBytes(absolute_path);
  if (result.first == nullptr) {
    std::string error_message =
        "error: Unable to read '" + absolute_path + "'.";
    return Dart_NewUnhandledExceptionError(
        Dart_NewStringFromCString(error_message.c_str()));
  }
  buffer = result.first;
  buffer_size = result.second;
  return Dart_True();
}

Dart_Handle FileLoader::Import(Dart_Handle url) {
  std::string url_string = StdStringFromDart(url);
  uint8_t* buffer = nullptr;
  intptr_t buffer_size = -1;
  Dart_Handle result = FetchBytes(url_string, buffer, buffer_size);
  if (Dart_IsError(result)) {
    return result;
  }
  // The embedder must keep the buffer alive until isolate shutdown.
  kernel_buffers_.push_back(buffer);
  return Dart_LoadLibraryFromKernel(buffer, buffer_size);
}

namespace {
void MallocFinalizer(void* isolate_callback_data, void* peer) {
  free(peer);
}
}  // namespace

Dart_Handle FileLoader::Kernel(Dart_Handle url) {
  std::string url_string = StdStringFromDart(url);
  uint8_t* buffer = nullptr;
  intptr_t buffer_size = -1;
  Dart_Handle result = FetchBytes(url_string, buffer, buffer_size);
  if (Dart_IsError(result)) {
    return result;
  }
  result =
      Dart_NewExternalTypedData(Dart_TypedData_kUint8, buffer, buffer_size);
  Dart_NewFinalizableHandle(result, buffer, buffer_size, MallocFinalizer);
  return result;
}

// This is invoked upon a reload request.
void FileLoader::SetPackagesUrl(Dart_Handle url) {
  if (url == Dart_Null()) {
    // No packages url specified.
    LoadPackagesMap(packages());
    return;
  }
  const std::string& packages_url = StdStringFromDart(url);
  LoadPackagesMap(packages_url);
}

std::string FileLoader::GetFilePathForFileURL(std::string url) {
  TONIC_DCHECK(url.find(FileLoader::kFileURLPrefix) == 0u);
  return SanitizePath(url.substr(FileLoader::kFileURLPrefixLength));
}

std::string FileLoader::GetFileURLForPath(const std::string& path) {
  return std::string(FileLoader::kFileURLPrefix) + path;
}

}  // namespace tonic
