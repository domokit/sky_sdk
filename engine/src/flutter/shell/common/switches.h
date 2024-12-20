// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <string_view>

#include "flutter/common/settings.h"
#include "flutter/fml/command_line.h"

#ifndef FLUTTER_SHELL_COMMON_SWITCHES_H_
#define FLUTTER_SHELL_COMMON_SWITCHES_H_

namespace flutter {

// clang-format off
#ifndef DEF_SWITCHES_START
#define DEF_SWITCHES_START enum class Switch {
#endif
#ifndef DEF_SWITCH
#define DEF_SWITCH(swtch, flag, help) swtch,
#endif
#ifndef DEF_SWITCHES_END
#define DEF_SWITCHES_END Sentinel, } ;
#endif
// clang-format on

DEF_SWITCHES_START
DEF_SWITCH(AotSharedLibraryName,
           "aot-shared-library-name",
           "Name of the *.so containing AOT compiled Dart assets.")
DEF_SWITCH(AotVMServiceSharedLibraryName,
           "aot-vmservice-shared-library-name",
           "Name of the *.so containing AOT compiled Dart assets for "
           "launching the service isolate.")
DEF_SWITCH(SnapshotAssetPath,
           "snapshot-asset-path",
           "Path to the directory containing the four files specified by "
           "VmSnapshotData, VmSnapshotInstructions, "
           "VmSnapshotInstructions and IsolateSnapshotInstructions.")
DEF_SWITCH(VmSnapshotData,
           "vm-snapshot-data",
           "The VM snapshot data that will be memory mapped as read-only. "
           "SnapshotAssetPath must be present.")
DEF_SWITCH(VmSnapshotInstructions,
           "vm-snapshot-instr",
           "The VM instructions snapshot that will be memory mapped as read "
           "and executable. SnapshotAssetPath must be present.")
DEF_SWITCH(IsolateSnapshotData,
           "isolate-snapshot-data",
           "The isolate snapshot data that will be memory mapped as read-only. "
           "SnapshotAssetPath must be present.")
DEF_SWITCH(IsolateSnapshotInstructions,
           "isolate-snapshot-instr",
           "The isolate instructions snapshot that will be memory mapped as "
           "read and executable. SnapshotAssetPath must be present.")
DEF_SWITCH(CacheDirPath,
           "cache-dir-path",
           "Path to the cache directory. "
           "This is different from the persistent_cache_path in embedder.h, "
           "which is used for Skia shader cache.")
DEF_SWITCH(ICUDataFilePath, "icu-data-file-path", "Path to the ICU data file.")
DEF_SWITCH(ICUSymbolPrefix,
           "icu-symbol-prefix",
           "Prefix for the symbols representing ICU data linked into the "
           "Flutter library.")
DEF_SWITCH(ICUNativeLibPath,
           "icu-native-lib-path",
           "Path to the library file that exports the ICU data.")
DEF_SWITCH(DartFlags,
           "dart-flags",
           "Flags passed directly to the Dart VM without being interpreted "
           "by the Flutter shell.")
DEF_SWITCH(DeviceVMServiceHost,
           "vm-service-host",
           "The hostname/IP address on which the Dart VM Service should "
           "be served. If not set, defaults to 127.0.0.1 or ::1 depending on "
           "whether --ipv6 is specified.")
// TODO(bkonyi): remove once flutter_tools no longer uses this option.
// See https://github.com/dart-lang/sdk/issues/50233
DEF_SWITCH(
    DeviceObservatoryHost,
    "observatory-host",
    "(deprecated) The hostname/IP address on which the Dart VM Service should "
    "be served. If not set, defaults to 127.0.0.1 or ::1 depending on "
    "whether --ipv6 is specified.")
DEF_SWITCH(DeviceVMServicePort,
           "vm-service-port",
           "A custom Dart VM Service port. The default is to pick a randomly "
           "available open port.")
// TODO(bkonyi): remove once flutter_tools no longer uses this option.
// See https://github.com/dart-lang/sdk/issues/50233
DEF_SWITCH(DeviceObservatoryPort,
           "observatory-port",
           "(deprecated) A custom Dart VM Service port. The default is to pick "
           "a randomly "
           "available open port.")
DEF_SWITCH(
    DisableVMService,
    "disable-vm-service",
    "Disable the Dart VM Service. The Dart VM Service is never available "
    "in release mode.")
// TODO(bkonyi): remove once flutter_tools no longer uses this option.
// See https://github.com/dart-lang/sdk/issues/50233
DEF_SWITCH(DisableObservatory,
           "disable-observatory",
           "(deprecated) Disable the Dart VM Service. The Dart VM Service is "
           "never available "
           "in release mode.")
DEF_SWITCH(DisableVMServicePublication,
           "disable-vm-service-publication",
           "Disable mDNS Dart VM Service publication.")
// TODO(bkonyi): remove once flutter_tools no longer uses this option.
// See https://github.com/dart-lang/sdk/issues/50233
DEF_SWITCH(DisableObservatoryPublication,
           "disable-observatory-publication",
           "(deprecated) Disable mDNS Dart VM Service publication.")
DEF_SWITCH(IPv6,
           "ipv6",
           "Bind to the IPv6 localhost address for the Dart VM Service. "
           "Ignored if --vm-service-host is set.")
DEF_SWITCH(EnableDartProfiling,
           "enable-dart-profiling",
           "Enable Dart profiling. Profiling information can be viewed from "
           "Dart / Flutter DevTools.")
DEF_SWITCH(EndlessTraceBuffer,
           "endless-trace-buffer",
           "Enable an endless trace buffer. The default is a ring buffer. "
           "This is useful when very old events need to viewed. For example, "
           "during application launch. Memory usage will continue to grow "
           "indefinitely however.")
DEF_SWITCH(EnableSoftwareRendering,
           "enable-software-rendering",
           "Enable rendering using the Skia software backend. This is useful "
           "when testing Flutter on emulators. By default, Flutter will "
           "attempt to either use OpenGL, Metal, or Vulkan.")
DEF_SWITCH(Route,
           "route",
           "Start app with an specific route defined on the framework")
DEF_SWITCH(SkiaDeterministicRendering,
           "skia-deterministic-rendering",
           "Skips the call to SkGraphics::Init(), thus avoiding swapping out "
           "some Skia function pointers based on available CPU features. This "
           "is used to obtain 100% deterministic behavior in Skia rendering.")
DEF_SWITCH(FlutterAssetsDir,
           "flutter-assets-dir",
           "Path to the Flutter assets directory.")
DEF_SWITCH(Help, "help", "Display this help text.")
DEF_SWITCH(LogTag, "log-tag", "Tag associated with log messages.")
DEF_SWITCH(DisableServiceAuthCodes,
           "disable-service-auth-codes",
           "Disable the requirement for authentication codes for communicating"
           " with the VM service.")
DEF_SWITCH(EnableServicePortFallback,
           "enable-service-port-fallback",
           "Allow the VM service to fallback to automatic port selection if"
           " binding to a specified port fails.")
DEF_SWITCH(StartPaused,
           "start-paused",
           "Start the application paused in the Dart debugger.")
DEF_SWITCH(EnableCheckedMode, "enable-checked-mode", "Enable checked mode.")
DEF_SWITCH(TraceStartup,
           "trace-startup",
           "Trace early application lifecycle. Automatically switches to an "
           "endless trace buffer.")
DEF_SWITCH(TraceSkia,
           "trace-skia",
           "Trace Skia calls. This is useful when debugging the GPU threed."
           "By default, Skia tracing is not enabled to reduce the number of "
           "traced events")
DEF_SWITCH(TraceSkiaAllowlist,
           "trace-skia-allowlist",
           "Filters out all Skia trace event categories except those that are "
           "specified in this comma separated list.")
DEF_SWITCH(
    TraceAllowlist,
    "trace-allowlist",
    "Filters out all trace events except those that are specified in this "
    "comma separated list of allowed prefixes.")
DEF_SWITCH(DumpSkpOnShaderCompilation,
           "dump-skp-on-shader-compilation",
           "Automatically dump the skp that triggers new shader compilations. "
           "This is useful for writing custom ShaderWarmUp to reduce jank. "
           "By default, this is not enabled to reduce the overhead. ")
DEF_SWITCH(CacheSkSL,
           "cache-sksl",
           "Only cache the shader in SkSL instead of binary or GLSL. This "
           "should only be used during development phases. The generated SkSLs "
           "can later be used in the release build for shader precompilation "
           "at launch in order to eliminate the shader-compile jank.")
DEF_SWITCH(PurgePersistentCache,
           "purge-persistent-cache",
           "Remove all existing persistent cache. This is mainly for debugging "
           "purposes such as reproducing the shader compilation jank.")
DEF_SWITCH(
    TraceSystrace,
    "trace-systrace",
    "Trace to the system tracer (instead of the timeline) on platforms where "
    "such a tracer is available. Currently only supported on Android and "
    "Fuchsia.")
DEF_SWITCH(TraceToFile,
           "trace-to-file",
           "Write the timeline trace to a file at the specified path. The file "
           "will be in Perfetto's proto format; it will be possible to load "
           "the file into Perfetto's trace viewer.")
DEF_SWITCH(UseTestFonts,
           "use-test-fonts",
           "Running tests that layout and measure text will not yield "
           "consistent results across various platforms. Enabling this option "
           "will make font resolution default to the Ahem test font on all "
           "platforms (See https://www.w3.org/Style/CSS/Test/Fonts/Ahem/). "
           "This option is only available on the desktop test shells.")
DEF_SWITCH(DisableAssetFonts,
           "disable-asset-fonts",
           "Prevents usage of any non-test fonts unless they were explicitly "
           "Loaded via dart:ui font APIs. This option is only available on the "
           "desktop test shells.")
DEF_SWITCH(PrefetchedDefaultFontManager,
           "prefetched-default-font-manager",
           "Indicates whether the embedding started a prefetch of the "
           "default font manager before creating the engine.")
DEF_SWITCH(VerboseLogging,
           "verbose-logging",
           "By default, only errors are logged. This flag enabled logging at "
           "all severity levels. This is NOT a per shell flag and affect log "
           "levels for all shells in the process.")
DEF_SWITCH(RunForever,
           "run-forever",
           "In non-interactive mode, keep the shell running after the Dart "
           "script has completed.")
DEF_SWITCH(DisableDartAsserts,
           "disable-dart-asserts",
           "Dart code runs with assertions enabled when the runtime mode is "
           "debug. In profile and release product modes, assertions are "
           "disabled. This flag may be specified if the user wishes to run "
           "with assertions disabled in the debug product mode (i.e. with JIT "
           "or DBC).")
DEF_SWITCH(EnableSerialGC,
           "enable-serial-gc",
           "On low power devices with low core counts, running concurrent "
           "GC tasks on threads can cause them to contend with the UI thread "
           "which could potentially lead to jank. This option turns off all "
           "concurrent GC activities")
DEF_SWITCH(DisallowInsecureConnections,
           "disallow-insecure-connections",
           "By default, dart:io allows all socket connections. If this switch "
           "is set, all insecure connections are rejected.")
DEF_SWITCH(DomainNetworkPolicy,
           "domain-network-policy",
           "JSON encoded network policy per domain. This overrides the "
           "DisallowInsecureConnections switch. Embedder can specify whether "
           "to allow or disallow insecure connections at a domain level.")
DEF_SWITCH(
    ForceMultithreading,
    "force-multithreading",
    "Uses separate threads for the platform, UI, GPU and IO task runners. "
    "By default, a single thread is used for all task runners. Only available "
    "in the flutter_tester.")
DEF_SWITCH(OldGenHeapSize,
           "old-gen-heap-size",
           "The size limit in megabytes for the Dart VM old gen heap space.")

DEF_SWITCH(ResourceCacheMaxBytesThreshold,
           "resource-cache-max-bytes-threshold",
           "The max bytes threshold of resource cache, or 0 for unlimited.")
DEF_SWITCH(EnableImpeller,
           "enable-impeller",
           "Enable the Impeller renderer on supported platforms. Ignored if "
           "Impeller is not supported on the platform.")
DEF_SWITCH(ImpellerBackend,
           "impeller-backend",
           "Requests a particular Impeller backend on platforms that support "
           "multiple backends. (ex `opengles` or `vulkan`)")
DEF_SWITCH(EnableVulkanValidation,
           "enable-vulkan-validation",
           "Enable loading Vulkan validation layers. The layers must be "
           "available to the application and loadable. On non-Vulkan backends, "
           "this flag does nothing.")
DEF_SWITCH(EnableOpenGLGPUTracing,
           "enable-opengl-gpu-tracing",
           "Enable tracing of GPU execution time when using the Impeller "
           "OpenGLES backend.")
DEF_SWITCH(EnableVulkanGPUTracing,
           "enable-vulkan-gpu-tracing",
           "Enable tracing of GPU execution time when using the Impeller "
           "Vulkan backend.")
DEF_SWITCH(LeakVM,
           "leak-vm",
           "When the last shell shuts down, the shared VM is leaked by default "
           "(the leak_vm in VM settings is true). To clean up the leak VM, set "
           "this value to false.")
DEF_SWITCH(EnableEmbedderAPI,
           "enable-embedder-api",
           "Enable the embedder api. Defaults to false. iOS only.")
DEF_SWITCH(EnablePlatformIsolates,
           "enable-platform-isolates",
           "Enable support for isolates that run on the platform thread.")
DEF_SWITCH(DisableMergedPlatformUIThread,
           "no-enable-merged-platform-ui-thread",
           "Merge the ui thread and platform thread.")
DEF_SWITCH(DisableAndroidSurfaceControl,
           "disable-surface-control",
           "Disable the SurfaceControl backed swapchain even when supported.")
DEF_SWITCHES_END

void PrintUsage(const std::string& executable_name);

const std::string_view FlagForSwitch(Switch swtch);

Settings SettingsFromCommandLine(const fml::CommandLine& command_line);

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_SWITCHES_H_
