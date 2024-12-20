// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>

#include "flutter/fml/native_library.h"
#include "flutter/fml/paths.h"
#include "flutter/shell/version/version.h"

// Include once for the default enum definition.
#include "flutter/shell/common/switches.h"

#undef FLUTTER_SHELL_COMMON_SWITCHES_H_

struct SwitchDesc {
  flutter::Switch sw;
  const std::string_view flag;
  const char* help;
};

#undef DEF_SWITCHES_START
#undef DEF_SWITCH
#undef DEF_SWITCHES_END

// clang-format off
#define DEF_SWITCHES_START static const struct SwitchDesc gSwitchDescs[] = {
#define DEF_SWITCH(p_swtch, p_flag, p_help) \
  { flutter::Switch:: p_swtch, p_flag, p_help },
#define DEF_SWITCHES_END };
// clang-format on

// List of common and safe VM flags to allow to be passed directly to the VM.
#if FLUTTER_RELEASE

// clang-format off
static const std::string kAllowedDartFlags[] = {
    "--enable-isolate-groups",
    "--no-enable-isolate-groups",
};
// clang-format on

#else

// clang-format off
static const std::string kAllowedDartFlags[] = {
    "--enable-isolate-groups",
    "--no-enable-isolate-groups",
    "--enable_mirrors",
    "--enable-service-port-fallback",
    "--max_profile_depth",
    "--profile_period",
    "--random_seed",
    "--sample-buffer-duration",
    "--trace-kernel",
    "--trace-reload",
    "--trace-reload-verbose",
    "--write-service-info",
    "--max_subtype_cache_entries",
    "--enable-asserts",
};
// clang-format on

#endif  // FLUTTER_RELEASE

// Include again for struct definition.
#include "flutter/shell/common/switches.h"

// Define symbols for the ICU data that is linked into the Flutter library on
// Android.  This is a workaround for crashes seen when doing dynamic lookups
// of the engine's own symbols on some older versions of Android.
#if FML_OS_ANDROID
extern uint8_t _binary_icudtl_dat_start[];
extern size_t _binary_icudtl_dat_size;

static std::unique_ptr<fml::Mapping> GetICUStaticMapping() {
  return std::make_unique<fml::NonOwnedMapping>(_binary_icudtl_dat_start,
                                                _binary_icudtl_dat_size);
}
#endif

namespace flutter {

void PrintUsage(const std::string& executable_name) {
  std::cerr << std::endl << "  " << executable_name << std::endl << std::endl;

  std::cerr << "Versions: " << std::endl << std::endl;

  std::cerr << "Flutter Engine Version: " << GetFlutterEngineVersion()
            << std::endl;
  std::cerr << "Skia Version: " << GetSkiaVersion() << std::endl;

  std::cerr << "Dart Version: " << GetDartVersion() << std::endl << std::endl;

  std::cerr << "Available Flags:" << std::endl;

  const uint32_t column_width = 80;

  const uint32_t flags_count = static_cast<uint32_t>(Switch::Sentinel);

  uint32_t max_width = 2;
  for (uint32_t i = 0; i < flags_count; i++) {
    auto desc = gSwitchDescs[i];
    max_width = std::max<uint32_t>(desc.flag.size() + 2, max_width);
  }

  const uint32_t help_width = column_width - max_width - 3;

  std::cerr << std::string(column_width, '-') << std::endl;
  for (uint32_t i = 0; i < flags_count; i++) {
    auto desc = gSwitchDescs[i];

    std::cerr << std::setw(max_width)
              << std::string("--") +
                     std::string{desc.flag.data(), desc.flag.size()}
              << " : ";

    std::istringstream stream(desc.help);
    int32_t remaining = help_width;

    std::string word;
    while (stream >> word && remaining > 0) {
      remaining -= (word.size() + 1);
      if (remaining <= 0) {
        std::cerr << std::endl
                  << std::string(max_width, ' ') << "   " << word << " ";
        remaining = help_width;
      } else {
        std::cerr << word << " ";
      }
    }

    std::cerr << std::endl;
  }
  std::cerr << std::string(column_width, '-') << std::endl;
}

const std::string_view FlagForSwitch(Switch swtch) {
  for (uint32_t i = 0; i < static_cast<uint32_t>(Switch::Sentinel); i++) {
    if (gSwitchDescs[i].sw == swtch) {
      return gSwitchDescs[i].flag;
    }
  }
  return std::string_view();
}

static std::vector<std::string> ParseCommaDelimited(const std::string& input) {
  std::istringstream ss(input);
  std::vector<std::string> result;
  std::string token;
  while (std::getline(ss, token, ',')) {
    result.push_back(token);
  }
  return result;
}

static bool IsAllowedDartVMFlag(const std::string& flag) {
  for (uint32_t i = 0; i < std::size(kAllowedDartFlags); ++i) {
    const std::string& allowed = kAllowedDartFlags[i];
    // Check that the prefix of the flag matches one of the allowed flags. This
    // is to handle cases where flags take arguments, such as in
    // "--max_profile_depth 1".
    //
    // We don't need to worry about cases like "--safe --sneaky_dangerous" as
    // the VM will discard these as a single unrecognized flag.
    if (flag.length() >= allowed.length() &&
        std::equal(allowed.begin(), allowed.end(), flag.begin())) {
      return true;
    }
  }
  return false;
}

template <typename T>
static bool GetSwitchValue(const fml::CommandLine& command_line,
                           Switch sw,
                           T* result) {
  std::string switch_string;

  if (!command_line.GetOptionValue(FlagForSwitch(sw), &switch_string)) {
    return false;
  }

  std::stringstream stream(switch_string);
  T value = 0;
  if (stream >> value) {
    *result = value;
    return true;
  }

  return false;
}

std::unique_ptr<fml::Mapping> GetSymbolMapping(
    const std::string& symbol_prefix,
    const std::string& native_lib_path) {
  const uint8_t* mapping = nullptr;
  intptr_t size;

  auto lookup_symbol = [&mapping, &size, symbol_prefix](
                           const fml::RefPtr<fml::NativeLibrary>& library) {
    mapping = library->ResolveSymbol((symbol_prefix + "_start").c_str());
    size = reinterpret_cast<intptr_t>(
        library->ResolveSymbol((symbol_prefix + "_size").c_str()));
  };

  fml::RefPtr<fml::NativeLibrary> library =
      fml::NativeLibrary::CreateForCurrentProcess();
  lookup_symbol(library);

  if (!(mapping && size)) {
    // Symbol lookup for the current process fails on some devices.  As a
    // fallback, try doing the lookup based on the path to the Flutter library.
    library = fml::NativeLibrary::Create(native_lib_path.c_str());
    lookup_symbol(library);
  }

  FML_CHECK(mapping && size) << "Unable to resolve symbols: " << symbol_prefix;
  return std::make_unique<fml::NonOwnedMapping>(mapping, size);
}

Settings SettingsFromCommandLine(const fml::CommandLine& command_line) {
  Settings settings = {};

  // Set executable name.
  if (command_line.has_argv0()) {
    settings.executable_name = command_line.argv0();
  }

  // Enable the VM Service
  settings.enable_vm_service =
      !command_line.HasOption(FlagForSwitch(Switch::DisableVMService)) &&
      // TODO(bkonyi): remove once flutter_tools no longer uses this option.
      // See https://github.com/dart-lang/sdk/issues/50233
      !command_line.HasOption(FlagForSwitch(Switch::DisableObservatory));

  // Enable mDNS VM Service Publication
  settings.enable_vm_service_publication =
      !command_line.HasOption(
          FlagForSwitch(Switch::DisableVMServicePublication)) &&
      !command_line.HasOption(
          FlagForSwitch(Switch::DisableObservatoryPublication));

  // Set VM Service Host
  if (command_line.HasOption(FlagForSwitch(Switch::DeviceVMServiceHost))) {
    command_line.GetOptionValue(FlagForSwitch(Switch::DeviceVMServiceHost),
                                &settings.vm_service_host);
  } else if (command_line.HasOption(
                 FlagForSwitch(Switch::DeviceObservatoryHost))) {
    // TODO(bkonyi): remove once flutter_tools no longer uses this option.
    // See https://github.com/dart-lang/sdk/issues/50233
    command_line.GetOptionValue(FlagForSwitch(Switch::DeviceObservatoryHost),
                                &settings.vm_service_host);
  }
  // Default the VM Service port based on --ipv6 if not set.
  if (settings.vm_service_host.empty()) {
    settings.vm_service_host =
        command_line.HasOption(FlagForSwitch(Switch::IPv6)) ? "::1"
                                                            : "127.0.0.1";
  }

  // Set VM Service Port
  if (command_line.HasOption(FlagForSwitch(Switch::DeviceVMServicePort))) {
    if (!GetSwitchValue(command_line, Switch::DeviceVMServicePort,
                        &settings.vm_service_port)) {
      FML_LOG(INFO)
          << "VM Service port specified was malformed. Will default to "
          << settings.vm_service_port;
    }
  } else if (command_line.HasOption(
                 FlagForSwitch(Switch::DeviceObservatoryPort))) {
    // TODO(bkonyi): remove once flutter_tools no longer uses this option.
    // See https://github.com/dart-lang/sdk/issues/50233
    if (!GetSwitchValue(command_line, Switch::DeviceObservatoryPort,
                        &settings.vm_service_port)) {
      FML_LOG(INFO)
          << "VM Service port specified was malformed. Will default to "
          << settings.vm_service_port;
    }
  }

  settings.may_insecurely_connect_to_all_domains = !command_line.HasOption(
      FlagForSwitch(Switch::DisallowInsecureConnections));

  command_line.GetOptionValue(FlagForSwitch(Switch::DomainNetworkPolicy),
                              &settings.domain_network_policy);

  // Disable need for authentication codes for VM service communication, if
  // specified.
  settings.disable_service_auth_codes =
      command_line.HasOption(FlagForSwitch(Switch::DisableServiceAuthCodes));

  // Allow fallback to automatic port selection if binding to a specified port
  // fails.
  settings.enable_service_port_fallback =
      command_line.HasOption(FlagForSwitch(Switch::EnableServicePortFallback));

  // Checked mode overrides.
  settings.disable_dart_asserts =
      command_line.HasOption(FlagForSwitch(Switch::DisableDartAsserts));

  settings.start_paused =
      command_line.HasOption(FlagForSwitch(Switch::StartPaused));

  settings.enable_checked_mode =
      command_line.HasOption(FlagForSwitch(Switch::EnableCheckedMode));

  settings.enable_dart_profiling =
      command_line.HasOption(FlagForSwitch(Switch::EnableDartProfiling));

  settings.enable_software_rendering =
      command_line.HasOption(FlagForSwitch(Switch::EnableSoftwareRendering));

  settings.endless_trace_buffer =
      command_line.HasOption(FlagForSwitch(Switch::EndlessTraceBuffer));

  settings.trace_startup =
      command_line.HasOption(FlagForSwitch(Switch::TraceStartup));

  settings.enable_serial_gc =
      command_line.HasOption(FlagForSwitch(Switch::EnableSerialGC));

#if !FLUTTER_RELEASE
  settings.trace_skia = true;

  if (command_line.HasOption(FlagForSwitch(Switch::TraceSkia))) {
    // If --trace-skia is specified, then log all Skia events.
    settings.trace_skia_allowlist.reset();
  } else {
    std::string trace_skia_allowlist;
    command_line.GetOptionValue(FlagForSwitch(Switch::TraceSkiaAllowlist),
                                &trace_skia_allowlist);
    if (trace_skia_allowlist.size()) {
      settings.trace_skia_allowlist = ParseCommaDelimited(trace_skia_allowlist);
    } else {
      settings.trace_skia_allowlist = {"skia.shaders"};
    }
  }
#endif  // !FLUTTER_RELEASE

  std::string trace_allowlist;
  command_line.GetOptionValue(FlagForSwitch(Switch::TraceAllowlist),
                              &trace_allowlist);
  settings.trace_allowlist = ParseCommaDelimited(trace_allowlist);

  settings.trace_systrace =
      command_line.HasOption(FlagForSwitch(Switch::TraceSystrace));

  command_line.GetOptionValue(FlagForSwitch(Switch::TraceToFile),
                              &settings.trace_to_file);

  settings.skia_deterministic_rendering_on_cpu =
      command_line.HasOption(FlagForSwitch(Switch::SkiaDeterministicRendering));

  settings.verbose_logging =
      command_line.HasOption(FlagForSwitch(Switch::VerboseLogging));

  command_line.GetOptionValue(FlagForSwitch(Switch::FlutterAssetsDir),
                              &settings.assets_path);

  std::vector<std::string_view> aot_shared_library_name =
      command_line.GetOptionValues(FlagForSwitch(Switch::AotSharedLibraryName));

  std::vector<std::string_view> vmservice_shared_library_name =
      command_line.GetOptionValues(
          FlagForSwitch(Switch::AotVMServiceSharedLibraryName));
  for (auto path : vmservice_shared_library_name) {
    settings.vmservice_snapshot_library_path.emplace_back(path);
  }

  std::string snapshot_asset_path;
  command_line.GetOptionValue(FlagForSwitch(Switch::SnapshotAssetPath),
                              &snapshot_asset_path);

  std::string vm_snapshot_data_filename;
  command_line.GetOptionValue(FlagForSwitch(Switch::VmSnapshotData),
                              &vm_snapshot_data_filename);

  command_line.GetOptionValue(FlagForSwitch(Switch::Route), &settings.route);

  std::string vm_snapshot_instr_filename;
  command_line.GetOptionValue(FlagForSwitch(Switch::VmSnapshotInstructions),
                              &vm_snapshot_instr_filename);

  std::string isolate_snapshot_data_filename;
  command_line.GetOptionValue(FlagForSwitch(Switch::IsolateSnapshotData),
                              &isolate_snapshot_data_filename);

  std::string isolate_snapshot_instr_filename;
  command_line.GetOptionValue(
      FlagForSwitch(Switch::IsolateSnapshotInstructions),
      &isolate_snapshot_instr_filename);

  if (!aot_shared_library_name.empty()) {
    for (std::string_view name : aot_shared_library_name) {
      settings.application_library_path.emplace_back(name);
    }
  } else if (!snapshot_asset_path.empty()) {
    settings.vm_snapshot_data_path =
        fml::paths::JoinPaths({snapshot_asset_path, vm_snapshot_data_filename});
    settings.vm_snapshot_instr_path = fml::paths::JoinPaths(
        {snapshot_asset_path, vm_snapshot_instr_filename});
    settings.isolate_snapshot_data_path = fml::paths::JoinPaths(
        {snapshot_asset_path, isolate_snapshot_data_filename});
    settings.isolate_snapshot_instr_path = fml::paths::JoinPaths(
        {snapshot_asset_path, isolate_snapshot_instr_filename});
  }

  command_line.GetOptionValue(FlagForSwitch(Switch::CacheDirPath),
                              &settings.temp_directory_path);

  bool leak_vm = "true" == command_line.GetOptionValueWithDefault(
                               FlagForSwitch(Switch::LeakVM), "true");
  settings.leak_vm = leak_vm;

  if (settings.icu_initialization_required) {
    command_line.GetOptionValue(FlagForSwitch(Switch::ICUDataFilePath),
                                &settings.icu_data_path);
    if (command_line.HasOption(FlagForSwitch(Switch::ICUSymbolPrefix))) {
      std::string icu_symbol_prefix, native_lib_path;
      command_line.GetOptionValue(FlagForSwitch(Switch::ICUSymbolPrefix),
                                  &icu_symbol_prefix);
      command_line.GetOptionValue(FlagForSwitch(Switch::ICUNativeLibPath),
                                  &native_lib_path);

#if FML_OS_ANDROID
      settings.icu_mapper = GetICUStaticMapping;
#else
      settings.icu_mapper = [icu_symbol_prefix, native_lib_path] {
        return GetSymbolMapping(icu_symbol_prefix, native_lib_path);
      };
#endif
    }
  }

  settings.use_test_fonts =
      command_line.HasOption(FlagForSwitch(Switch::UseTestFonts));
  settings.use_asset_fonts =
      !command_line.HasOption(FlagForSwitch(Switch::DisableAssetFonts));

#if FML_OS_IOS || FML_OS_IOS_SIMULATOR
// On these configurations, the Impeller flags are completely ignored with the
// default taking hold.
#else   // FML_OS_IOS && !FML_OS_IOS_SIMULATOR
  {
    std::string enable_impeller_value;
    if (command_line.GetOptionValue(FlagForSwitch(Switch::EnableImpeller),
                                    &enable_impeller_value)) {
      settings.enable_impeller =
          enable_impeller_value.empty() || "true" == enable_impeller_value;
    }
  }
#endif  // FML_OS_IOS && !FML_OS_IOS_SIMULATOR

  {
    std::string impeller_backend_value;
    if (command_line.GetOptionValue(FlagForSwitch(Switch::ImpellerBackend),
                                    &impeller_backend_value)) {
      if (!impeller_backend_value.empty()) {
        settings.requested_rendering_backend = impeller_backend_value;
      }
    }
  }

  settings.enable_vulkan_validation =
      command_line.HasOption(FlagForSwitch(Switch::EnableVulkanValidation));
  settings.enable_opengl_gpu_tracing =
      command_line.HasOption(FlagForSwitch(Switch::EnableOpenGLGPUTracing));
  settings.enable_vulkan_gpu_tracing =
      command_line.HasOption(FlagForSwitch(Switch::EnableVulkanGPUTracing));

  settings.enable_embedder_api =
      command_line.HasOption(FlagForSwitch(Switch::EnableEmbedderAPI));

  settings.prefetched_default_font_manager = command_line.HasOption(
      FlagForSwitch(Switch::PrefetchedDefaultFontManager));

  std::string all_dart_flags;
  if (command_line.GetOptionValue(FlagForSwitch(Switch::DartFlags),
                                  &all_dart_flags)) {
    // Assume that individual flags are comma separated.
    std::vector<std::string> flags = ParseCommaDelimited(all_dart_flags);
    for (const auto& flag : flags) {
      if (!IsAllowedDartVMFlag(flag)) {
        FML_LOG(FATAL) << "Encountered disallowed Dart VM flag: " << flag;
      }
      settings.dart_flags.push_back(flag);
    }
  }

#if !FLUTTER_RELEASE
  command_line.GetOptionValue(FlagForSwitch(Switch::LogTag), &settings.log_tag);
#endif

  settings.dump_skp_on_shader_compilation =
      command_line.HasOption(FlagForSwitch(Switch::DumpSkpOnShaderCompilation));

  settings.cache_sksl =
      command_line.HasOption(FlagForSwitch(Switch::CacheSkSL));

  settings.purge_persistent_cache =
      command_line.HasOption(FlagForSwitch(Switch::PurgePersistentCache));

  if (command_line.HasOption(FlagForSwitch(Switch::OldGenHeapSize))) {
    std::string old_gen_heap_size;
    command_line.GetOptionValue(FlagForSwitch(Switch::OldGenHeapSize),
                                &old_gen_heap_size);
    settings.old_gen_heap_size = std::stoi(old_gen_heap_size);
  }

  if (command_line.HasOption(
          FlagForSwitch(Switch::ResourceCacheMaxBytesThreshold))) {
    std::string resource_cache_max_bytes_threshold;
    command_line.GetOptionValue(
        FlagForSwitch(Switch::ResourceCacheMaxBytesThreshold),
        &resource_cache_max_bytes_threshold);
    settings.resource_cache_max_bytes_threshold =
        std::stoi(resource_cache_max_bytes_threshold);
  }

  settings.enable_platform_isolates =
      command_line.HasOption(FlagForSwitch(Switch::EnablePlatformIsolates));

  settings.disable_surface_control = command_line.HasOption(
      FlagForSwitch(Switch::DisableAndroidSurfaceControl));

  settings.merged_platform_ui_thread = !command_line.HasOption(
      FlagForSwitch(Switch::DisableMergedPlatformUIThread));

  return settings;
}

}  // namespace flutter
