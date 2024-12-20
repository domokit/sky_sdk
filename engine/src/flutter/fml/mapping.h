// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_MAPPING_H_
#define FLUTTER_FML_MAPPING_H_

#include <initializer_list>
#include <memory>
#include <string>
#include <vector>

#include "flutter/fml/build_config.h"
#include "flutter/fml/file.h"
#include "flutter/fml/macros.h"
#include "flutter/fml/native_library.h"
#include "flutter/fml/unique_fd.h"

namespace fml {

class Mapping {
 public:
  Mapping();

  virtual ~Mapping();

  virtual size_t GetSize() const = 0;

  virtual const uint8_t* GetMapping() const = 0;

  // Whether calling madvise(DONTNEED) on the mapping is non-destructive.
  // Generally true for file-mapped memory and false for anonymous memory.
  virtual bool IsDontNeedSafe() const = 0;

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(Mapping);
};

class FileMapping final : public Mapping {
 public:
  enum class Protection {
    kRead,
    kWrite,
    kExecute,
  };

  explicit FileMapping(const fml::UniqueFD& fd,
                       std::initializer_list<Protection> protection = {
                           Protection::kRead});

  ~FileMapping() override;

  static std::unique_ptr<FileMapping> CreateReadOnly(const std::string& path);

  static std::unique_ptr<FileMapping> CreateReadOnly(
      const fml::UniqueFD& base_fd,
      const std::string& sub_path = "");

  static std::unique_ptr<FileMapping> CreateReadExecute(
      const std::string& path);

  static std::unique_ptr<FileMapping> CreateReadExecute(
      const fml::UniqueFD& base_fd,
      const std::string& sub_path = "");

  // |Mapping|
  size_t GetSize() const override;

  // |Mapping|
  const uint8_t* GetMapping() const override;

  // |Mapping|
  bool IsDontNeedSafe() const override;

  uint8_t* GetMutableMapping();

  bool IsValid() const;

 private:
  bool valid_ = false;
  size_t size_ = 0;
  uint8_t* mapping_ = nullptr;
  uint8_t* mutable_mapping_ = nullptr;

#if FML_OS_WIN
  fml::UniqueFD mapping_handle_;
#endif

  FML_DISALLOW_COPY_AND_ASSIGN(FileMapping);
};

class DataMapping final : public Mapping {
 public:
  explicit DataMapping(std::vector<uint8_t> data);

  explicit DataMapping(const std::string& string);

  ~DataMapping() override;

  // |Mapping|
  size_t GetSize() const override;

  // |Mapping|
  const uint8_t* GetMapping() const override;

  // |Mapping|
  bool IsDontNeedSafe() const override;

 private:
  std::vector<uint8_t> data_;

  FML_DISALLOW_COPY_AND_ASSIGN(DataMapping);
};

class NonOwnedMapping final : public Mapping {
 public:
  using ReleaseProc = std::function<void(const uint8_t* data, size_t size)>;
  NonOwnedMapping(const uint8_t* data,
                  size_t size,
                  const ReleaseProc& release_proc = nullptr,
                  bool dontneed_safe = false);

  ~NonOwnedMapping() override;

  // |Mapping|
  size_t GetSize() const override;

  // |Mapping|
  const uint8_t* GetMapping() const override;

  // |Mapping|
  bool IsDontNeedSafe() const override;

 private:
  const uint8_t* const data_;
  const size_t size_;
  const ReleaseProc release_proc_;
  const bool dontneed_safe_;

  FML_DISALLOW_COPY_AND_ASSIGN(NonOwnedMapping);
};

/// A Mapping like NonOwnedMapping, but uses Free as its release proc.
class MallocMapping final : public Mapping {
 public:
  MallocMapping();

  /// Creates a MallocMapping for a region of memory (without copying it).
  /// The function will `abort()` if the malloc fails.
  /// @param data The starting address of the mapping.
  /// @param size The size of the mapping in bytes.
  MallocMapping(uint8_t* data, size_t size);

  MallocMapping(fml::MallocMapping&& mapping);

  ~MallocMapping() override;

  /// Copies the data from `begin` to `end`.
  /// It's templated since void* arithemetic isn't allowed and we want support
  /// for `uint8_t` and `char`.
  template <typename T>
  static MallocMapping Copy(const T* begin, const T* end) {
    FML_DCHECK(end >= begin);
    size_t length = end - begin;
    return Copy(begin, length);
  }

  /// Copies a region of memory into a MallocMapping.
  /// The function will `abort()` if the malloc fails.
  /// @param begin The starting address of where we will copy.
  /// @param length The length of the region to copy in bytes.
  static MallocMapping Copy(const void* begin, size_t length);

  // |Mapping|
  size_t GetSize() const override;

  // |Mapping|
  const uint8_t* GetMapping() const override;

  // |Mapping|
  bool IsDontNeedSafe() const override;

  /// Removes ownership of the data buffer.
  /// After this is called; the mapping will point to nullptr.
  [[nodiscard]] uint8_t* Release();

 private:
  uint8_t* data_;
  size_t size_;

  FML_DISALLOW_COPY_AND_ASSIGN(MallocMapping);
};

class SymbolMapping final : public Mapping {
 public:
  SymbolMapping(fml::RefPtr<fml::NativeLibrary> native_library,
                const char* symbol_name);

  ~SymbolMapping() override;

  // |Mapping|
  size_t GetSize() const override;

  // |Mapping|
  const uint8_t* GetMapping() const override;

  // |Mapping|
  bool IsDontNeedSafe() const override;

 private:
  fml::RefPtr<fml::NativeLibrary> native_library_;
  const uint8_t* mapping_ = nullptr;

  FML_DISALLOW_COPY_AND_ASSIGN(SymbolMapping);
};

}  // namespace fml

#endif  // FLUTTER_FML_MAPPING_H_
