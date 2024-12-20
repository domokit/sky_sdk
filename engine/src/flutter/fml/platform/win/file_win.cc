// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/file.h"

#include <Fileapi.h>
#include <Shlwapi.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/stat.h>
#include <string>

#include <algorithm>
#include <climits>
#include <cstring>
#include <optional>
#include <sstream>

#include "flutter/fml/build_config.h"
#include "flutter/fml/mapping.h"
#include "flutter/fml/platform/win/errors_win.h"
#include "flutter/fml/platform/win/wstring_conversion.h"

namespace fml {

static std::string GetFullHandlePath(const fml::UniqueFD& handle) {
  wchar_t buffer[MAX_PATH] = {0};
  const DWORD buffer_size = ::GetFinalPathNameByHandle(
      handle.get(), buffer, MAX_PATH, FILE_NAME_NORMALIZED);
  if (buffer_size == 0) {
    return {};
  }
  return WideStringToUtf8({buffer, buffer_size});
}

static bool IsAbsolutePath(const char* path) {
  if (path == nullptr || strlen(path) == 0) {
    return false;
  }

  auto wpath = Utf8ToWideString({path});
  if (wpath.empty()) {
    return false;
  }

  return ::PathIsRelative(wpath.c_str()) == FALSE;
}

static std::string GetAbsolutePath(const fml::UniqueFD& base_directory,
                                   const char* subpath) {
  std::string path;
  if (IsAbsolutePath(subpath)) {
    path = subpath;
  } else {
    std::stringstream stream;
    stream << GetFullHandlePath(base_directory) << "\\" << subpath;
    path = stream.str();
  }
  std::replace(path.begin(), path.end(), '/', '\\');
  return path;
}

static std::wstring GetTemporaryDirectoryPath() {
  wchar_t wchar_path[MAX_PATH];
  auto result_size = ::GetTempPath(MAX_PATH, wchar_path);
  if (result_size > 0) {
    return {wchar_path, result_size};
  }
  return {};
}

static DWORD GetDesiredAccessFlags(FilePermission permission) {
  switch (permission) {
    case FilePermission::kRead:
      return GENERIC_READ;
    case FilePermission::kWrite:
      return GENERIC_WRITE;
    case FilePermission::kReadWrite:
      return GENERIC_READ | GENERIC_WRITE;
  }
  return GENERIC_READ;
}

static DWORD GetShareFlags(FilePermission permission) {
  switch (permission) {
    case FilePermission::kRead:
      return FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE;
    case FilePermission::kWrite:
      return 0;
    case FilePermission::kReadWrite:
      return 0;
  }
  return FILE_SHARE_READ;
}

static DWORD GetFileAttributesForUtf8Path(const char* absolute_path) {
  return ::GetFileAttributes(Utf8ToWideString(absolute_path).c_str());
}

static DWORD GetFileAttributesForUtf8Path(const fml::UniqueFD& base_directory,
                                          const char* path) {
  std::string full_path = GetAbsolutePath(base_directory, path);
  return GetFileAttributesForUtf8Path(full_path.c_str());
}

std::string CreateTemporaryDirectory() {
  // Get the system temporary directory.
  auto temp_dir_container = GetTemporaryDirectoryPath();
  if (temp_dir_container.empty()) {
    FML_DLOG(ERROR) << "Could not get system temporary directory.";
    return {};
  }

  // Create a UUID.
  UUID uuid;
  RPC_STATUS status = UuidCreateSequential(&uuid);
  if (status != RPC_S_OK && status != RPC_S_UUID_LOCAL_ONLY) {
    FML_DLOG(ERROR) << "Could not create UUID for temporary directory.";
    return {};
  }

  RPC_WSTR uuid_string;
  status = UuidToString(&uuid, &uuid_string);
  if (status != RPC_S_OK) {
    FML_DLOG(ERROR) << "Could not map UUID to string for temporary directory.";
    return {};
  }

  std::wstring uuid_str(reinterpret_cast<wchar_t*>(uuid_string));
  RpcStringFree(&uuid_string);

  // Join the two and create a path to the new temporary directory.

  std::wstringstream stream;
  stream << temp_dir_container << "\\" << uuid_str;
  auto temp_dir = stream.str();

  auto dir_fd = OpenDirectory(WideStringToUtf8(temp_dir).c_str(), true,
                              FilePermission::kReadWrite);
  if (!dir_fd.is_valid()) {
    FML_DLOG(ERROR) << "Could not get temporary directory file descriptor. "
                    << GetLastErrorMessage();
    return {};
  }

  return WideStringToUtf8(std::move(temp_dir));
}

fml::UniqueFD OpenFile(const fml::UniqueFD& base_directory,
                       const char* path,
                       bool create_if_necessary,
                       FilePermission permission) {
  return OpenFile(GetAbsolutePath(base_directory, path).c_str(),
                  create_if_necessary, permission);
}

fml::UniqueFD OpenFile(const char* path,
                       bool create_if_necessary,
                       FilePermission permission) {
  if (path == nullptr || strlen(path) == 0) {
    return {};
  }

  auto file_name = Utf8ToWideString({path});

  if (file_name.empty()) {
    return {};
  }

  const DWORD creation_disposition =
      create_if_necessary ? OPEN_ALWAYS : OPEN_EXISTING;

  const DWORD flags = FILE_ATTRIBUTE_NORMAL;

  auto handle =
      CreateFile(file_name.c_str(),                  // lpFileName
                 GetDesiredAccessFlags(permission),  // dwDesiredAccess
                 GetShareFlags(permission),          // dwShareMode
                 nullptr,                            // lpSecurityAttributes  //
                 creation_disposition,               // dwCreationDisposition //
                 flags,   // dwFlagsAndAttributes                  //
                 nullptr  // hTemplateFile                         //
      );

  if (handle == INVALID_HANDLE_VALUE) {
    return {};
  }

  return fml::UniqueFD{handle};
}

fml::UniqueFD OpenDirectory(const fml::UniqueFD& base_directory,
                            const char* path,
                            bool create_if_necessary,
                            FilePermission permission) {
  return OpenDirectory(GetAbsolutePath(base_directory, path).c_str(),
                       create_if_necessary, permission);
}

fml::UniqueFD OpenDirectory(const char* path,
                            bool create_if_necessary,
                            FilePermission permission) {
  if (path == nullptr || strlen(path) == 0) {
    return {};
  }

  auto file_name = Utf8ToWideString({path});

  if (file_name.empty()) {
    return {};
  }

  if (create_if_necessary) {
    if (!::CreateDirectory(file_name.c_str(), nullptr)) {
      if (GetLastError() != ERROR_ALREADY_EXISTS) {
        FML_DLOG(ERROR) << "Could not create directory. "
                        << GetLastErrorMessage();
        return {};
      }
    }
  }

  const DWORD creation_disposition = OPEN_EXISTING;

  const DWORD flags = FILE_ATTRIBUTE_NORMAL | FILE_FLAG_BACKUP_SEMANTICS;

  auto handle =
      CreateFile(file_name.c_str(),                  // lpFileName
                 GetDesiredAccessFlags(permission),  // dwDesiredAccess
                 GetShareFlags(permission),          // dwShareMode
                 nullptr,                            // lpSecurityAttributes  //
                 creation_disposition,               // dwCreationDisposition //
                 flags,   // dwFlagsAndAttributes                  //
                 nullptr  // hTemplateFile                         //
      );

  if (handle == INVALID_HANDLE_VALUE) {
    return {};
  }

  return fml::UniqueFD{handle};
}

fml::UniqueFD Duplicate(fml::UniqueFD::element_type descriptor) {
  if (descriptor == INVALID_HANDLE_VALUE) {
    return fml::UniqueFD{};
  }

  HANDLE duplicated = INVALID_HANDLE_VALUE;

  if (!::DuplicateHandle(
          GetCurrentProcess(),  // source process
          descriptor,           // source handle
          GetCurrentProcess(),  // target process
          &duplicated,          // target handle
          0,      // desired access (ignored because DUPLICATE_SAME_ACCESS)
          FALSE,  // inheritable
          DUPLICATE_SAME_ACCESS)  // options
  ) {
    return fml::UniqueFD{};
  }

  return fml::UniqueFD{duplicated};
}

bool IsDirectory(const fml::UniqueFD& directory) {
  BY_HANDLE_FILE_INFORMATION info;
  if (!::GetFileInformationByHandle(directory.get(), &info)) {
    return false;
  }
  return info.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY;
}

bool IsDirectory(const fml::UniqueFD& base_directory, const char* path) {
  return GetFileAttributesForUtf8Path(base_directory, path) &
         FILE_ATTRIBUTE_DIRECTORY;
}

bool IsFile(const std::string& path) {
  DWORD attributes = GetFileAttributesForUtf8Path(path.c_str());
  if (attributes == INVALID_FILE_ATTRIBUTES) {
    return false;
  }
  return !(attributes &
           (FILE_ATTRIBUTE_DIRECTORY | FILE_ATTRIBUTE_REPARSE_POINT));
}

bool UnlinkDirectory(const char* path) {
  if (!::RemoveDirectory(Utf8ToWideString(path).c_str())) {
    FML_DLOG(ERROR) << "Could not remove directory: '" << path << "'. "
                    << GetLastErrorMessage();
    return false;
  }
  return true;
}

bool UnlinkDirectory(const fml::UniqueFD& base_directory, const char* path) {
  if (!::RemoveDirectory(
          Utf8ToWideString(GetAbsolutePath(base_directory, path)).c_str())) {
    FML_DLOG(ERROR) << "Could not remove directory: '" << path << "'. "
                    << GetLastErrorMessage();
    return false;
  }
  return true;
}

bool UnlinkFile(const char* path) {
  if (!::DeleteFile(Utf8ToWideString(path).c_str())) {
    FML_DLOG(ERROR) << "Could not remove file: '" << path << "'. "
                    << GetLastErrorMessage();
    return false;
  }
  return true;
}

bool UnlinkFile(const fml::UniqueFD& base_directory, const char* path) {
  if (!::DeleteFile(
          Utf8ToWideString(GetAbsolutePath(base_directory, path)).c_str())) {
    FML_DLOG(ERROR) << "Could not remove file: '" << path << "'. "
                    << GetLastErrorMessage();
    return false;
  }
  return true;
}

bool TruncateFile(const fml::UniqueFD& file, size_t size) {
  LARGE_INTEGER large_size;
  large_size.QuadPart = size;
  large_size.LowPart = SetFilePointer(file.get(), large_size.LowPart,
                                      &large_size.HighPart, FILE_BEGIN);
  if (large_size.LowPart == INVALID_SET_FILE_POINTER &&
      GetLastError() != NO_ERROR) {
    FML_DLOG(ERROR) << "Could not update file size. " << GetLastErrorMessage();
    return false;
  }

  if (!::SetEndOfFile(file.get())) {
    FML_DLOG(ERROR) << "Could not commit file size update. "
                    << GetLastErrorMessage();
    return false;
  }
  return true;
}

bool FileExists(const fml::UniqueFD& base_directory, const char* path) {
  return GetFileAttributesForUtf8Path(base_directory, path) !=
         INVALID_FILE_ATTRIBUTES;
}

// TODO(jonahwilliams): https://github.com/flutter/flutter/issues/102838 this is
// not atomic on Windows.
bool WriteAtomically(const fml::UniqueFD& base_directory,
                     const char* file_name,
                     const Mapping& mapping) {
  if (file_name == nullptr) {
    return false;
  }

  auto file_path = GetAbsolutePath(base_directory, file_name);
  auto temp_file =
      OpenFile(file_path.c_str(), true, FilePermission::kReadWrite);

  if (!temp_file.is_valid()) {
    FML_DLOG(ERROR) << "Could not create file: " << file_path.c_str() << " "
                    << GetLastError() << " " << GetLastErrorMessage();
    return false;
  }

  if (!TruncateFile(temp_file, mapping.GetSize())) {
    FML_DLOG(ERROR) << "Could not truncate the file to the correct size. "
                    << GetLastErrorMessage();
    return false;
  }

  {
    FileMapping temp_file_mapping(temp_file, {FileMapping::Protection::kRead,
                                              FileMapping::Protection::kWrite});
    if (temp_file_mapping.GetSize() != mapping.GetSize()) {
      FML_DLOG(ERROR) << "Temporary file mapping size was incorrect. Is "
                      << temp_file_mapping.GetSize() << ". Should be "
                      << mapping.GetSize() << ".";
      return false;
    }

    if (temp_file_mapping.GetMutableMapping() == nullptr) {
      FML_DLOG(ERROR) << "Temporary file does not have a mutable mapping.";
      return false;
    }

    ::memcpy(temp_file_mapping.GetMutableMapping(), mapping.GetMapping(),
             mapping.GetSize());

    if (!::FlushViewOfFile(temp_file_mapping.GetMutableMapping(),
                           mapping.GetSize())) {
      FML_DLOG(ERROR) << "Could not flush file view. " << GetLastErrorMessage();
      return false;
    }

    if (!::FlushFileBuffers(temp_file.get())) {
      FML_DLOG(ERROR) << "Could not flush file buffers. "
                      << GetLastErrorMessage();
      return false;
    }

    // File mapping is detroyed.
  }

  temp_file.reset();

  return true;
}

bool VisitFiles(const fml::UniqueFD& directory, const FileVisitor& visitor) {
  std::string search_pattern = GetFullHandlePath(directory) + "\\*";
  WIN32_FIND_DATA find_file_data;
  HANDLE find_handle = ::FindFirstFile(Utf8ToWideString(search_pattern).c_str(),
                                       &find_file_data);

  if (find_handle == INVALID_HANDLE_VALUE) {
    FML_DLOG(ERROR) << "Can't open the directory. Error: "
                    << GetLastErrorMessage();
    return true;  // continue to visit other files
  }

  do {
    std::string filename = WideStringToUtf8(find_file_data.cFileName);
    if (filename != "." && filename != "..") {
      if (!visitor(directory, filename)) {
        ::FindClose(find_handle);
        return false;
      }
    }
  } while (::FindNextFile(find_handle, &find_file_data));
  ::FindClose(find_handle);
  return true;
}

}  // namespace fml
