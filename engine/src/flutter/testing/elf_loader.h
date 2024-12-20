// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_ELF_LOADER_H_
#define FLUTTER_TESTING_ELF_LOADER_H_

#include <memory>

#include "flutter/common/settings.h"
#include "flutter/fml/macros.h"
#include "third_party/dart/runtime/bin/elf_loader.h"

namespace flutter::testing {

inline constexpr const char* kDefaultAOTAppELFFileName = "app_elf_snapshot.so";

// This file name is what gen_snapshot defaults to. It is based off of the
// name of the base file, with the `2` indicating that this split corresponds
// to loading unit id of 2. The base module id is 1 and is omitted as it is not
// considered a split. If dart changes the naming convention, this should be
// changed to match, however, this is considered unlikely to happen.
inline constexpr const char* kDefaultAOTAppELFSplitFileName =
    "app_elf_snapshot.so-2.part.so";

struct LoadedELFDeleter {
  void operator()(Dart_LoadedElf* elf) { ::Dart_UnloadELF(elf); }
};

using UniqueLoadedELF = std::unique_ptr<Dart_LoadedElf, LoadedELFDeleter>;

struct ELFAOTSymbols {
  UniqueLoadedELF loaded_elf;
  const uint8_t* vm_snapshot_data = nullptr;
  const uint8_t* vm_snapshot_instrs = nullptr;
  const uint8_t* vm_isolate_data = nullptr;
  const uint8_t* vm_isolate_instrs = nullptr;
};

//------------------------------------------------------------------------------
/// @brief      Attempts to resolve AOT symbols from the portable ELF loader.
///             This location is automatically resolved from the fixtures
///             generator. This only returns valid symbols when the VM is
///             configured for AOT.
///
/// @param[in]  elf_filename  The AOT ELF filename from the fixtures generator.
///
/// @return     The loaded ELF symbols.
///
ELFAOTSymbols LoadELFSymbolFromFixturesIfNeccessary(std::string elf_filename);

//------------------------------------------------------------------------------
/// @brief      Attempts to resolve split loading unit AOT symbols from the
///             portable ELF loader. If the dart code does not make use of
///             deferred libraries, then there will be no split .so to load.
///             This only returns valid symbols when the VM is configured for
///             AOT.
///
/// @param[in]  elf_split_filename  The split AOT ELF filename from the fixtures
/// generator.
///
/// @return     The loaded ELF symbols.
///
ELFAOTSymbols LoadELFSplitSymbolFromFixturesIfNeccessary(
    std::string elf_split_filename);

//------------------------------------------------------------------------------
/// @brief      Prepare the settings objects various AOT mappings resolvers with
///             the symbols already loaded. This method does nothing in non-AOT
///             runtime modes.
///
/// @warning    The symbols must not be collected till all shell instantiations
///             made using the settings object are collected.
///
/// @param[in/out] settings  The settings whose AOT resolvers to populate.
/// @param[in]     symbols   The symbols used to populate the settings object.
///
/// @return     If the settings object was correctly updated.
///
bool PrepareSettingsForAOTWithSymbols(Settings& settings,
                                      const ELFAOTSymbols& symbols);

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_ELF_LOADER_H_
