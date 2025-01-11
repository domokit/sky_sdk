// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:unified_analytics/unified_analytics.dart' as analytics;
import 'package:vm_service/vm_service.dart';

import '../android/android_device.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../daemon.dart';
import '../device.dart';
import '../features.dart';
import '../globals.dart' as globals;
import '../ios/devices.dart';
import '../macos/macos_ipad_device.dart';
import '../project.dart';
import '../reporting/reporting.dart';
import '../resident_runner.dart';
import '../run_cold.dart';
import '../run_hot.dart';
import '../runner/flutter_command.dart';
import '../runner/flutter_command_runner.dart';
import '../tracing.dart';
import '../web/compile.dart';
import '../web/web_constants.dart';
import '../web/web_runner.dart';
import 'daemon.dart';

/// Shared logic between `flutter run` and `flutter drive` commands.
abstract class RunCommandBase extends FlutterCommand with DeviceBasedDevelopmentArtifacts {
  RunCommandBase({required bool verboseHelp}) {
    addBuildModeFlags(verboseHelp: verboseHelp, defaultToRelease: false);
    usesDartDefineOption();
    usesFlavorOption();
    usesWebResourcesCdnFlag();
    addNativeNullAssertions(hide: !verboseHelp);
    addBundleSkSLPathOption(hide: !verboseHelp);
    usesApplicationBinaryOption();
    argParser
      ..addFlag(
        'trace-startup',
        negatable: false,
        help:
            'Trace application startup, then exit, saving the trace to a file. '
            'By default, this will be saved in the "build" directory. If the '
            'FLUTTER_TEST_OUTPUTS_DIR environment variable is set, the file '
            'will be written there instead.',
      )
      ..addFlag(
        'cache-startup-profile',
        help:
            'Caches the CPU profile collected before the first frame for startup '
            'analysis.',
      )
      ..addFlag(
        'verbose-system-logs',
        negatable: false,
        help: 'Include verbose logging from the Flutter engine.',
      )
      ..addFlag(
        'cache-sksl',
        negatable: false,
        help: 'Cache the shader in the SkSL format instead of in binary or GLSL formats.',
      )
      ..addFlag(
        'dump-skp-on-shader-compilation',
        negatable: false,
        help:
            'Automatically dump the skp that triggers new shader compilations. '
            'This is useful for writing custom ShaderWarmUp to reduce jank. '
            'By default, this is not enabled as it introduces significant overhead. '
            'This is only available in profile or debug builds.',
      )
      ..addFlag(
        'purge-persistent-cache',
        negatable: false,
        help:
            'Removes all existing persistent caches. This allows reproducing '
            'shader compilation jank that normally only happens the first time '
            'an app is run, or for reliable testing of compilation jank fixes '
            '(e.g. shader warm-up).',
      )
      ..addOption('route', help: 'Which route to load when running the app.')
      ..addOption(
        'vmservice-out-file',
        help:
            'A file to write the attached vmservice URL to after an '
            'application is started.',
        valueHelp: 'project/example/out.txt',
        hide: !verboseHelp,
      )
      ..addFlag(
        'disable-service-auth-codes',
        negatable: false,
        hide: !verboseHelp,
        help:
            '(deprecated) Allow connections to the VM service without using authentication codes. '
            '(Not recommended! This can open your device to remote code execution attacks!)',
      )
      ..addFlag(
        'start-paused',
        defaultsTo: startPausedDefault,
        help: 'Start in a paused mode and wait for a debugger to connect.',
      )
      ..addOption(
        'dart-flags',
        hide: !verboseHelp,
        help:
            'Pass a list of comma separated flags to the Dart instance at '
            'application startup. Flags passed through this option must be '
            'present on the allowlist defined within the Flutter engine. If '
            'a disallowed flag is encountered, the process will be '
            'terminated immediately.\n\n'
            'This flag is not available on the stable channel and is only '
            'applied in debug and profile modes. This option should only '
            'be used for experiments and should not be used by typical users.',
      )
      ..addFlag(
        'endless-trace-buffer',
        negatable: false,
        help:
            'Enable tracing to an infinite buffer, instead of a ring buffer. '
            'This is useful when recording large traces. To use an endless buffer to '
            'record startup traces, combine this with "--trace-startup".',
      )
      ..addFlag(
        'trace-systrace',
        negatable: false,
        help:
            'Enable tracing to the system tracer. This is only useful on '
            'platforms where such a tracer is available (Android, iOS, '
            'macOS and Fuchsia).',
      )
      ..addOption(
        'trace-to-file',
        help:
            'Write the timeline trace to a file at the specified path. The '
            "file will be in Perfetto's proto format; it will be possible to "
            "load the file into Perfetto's trace viewer.",
        valueHelp: 'path/to/trace.binpb',
      )
      ..addFlag(
        'trace-skia',
        negatable: false,
        help:
            'Enable tracing of Skia code. This is useful when debugging '
            'the raster thread (formerly known as the GPU thread). '
            'By default, Flutter will not log Skia code, as it introduces significant '
            'overhead that may affect recorded performance metrics in a misleading way.',
      )
      ..addOption(
        'trace-allowlist',
        hide: !verboseHelp,
        help:
            'Filters out all trace events except those that are specified in '
            'this comma separated list of allowed prefixes.',
        valueHelp: 'foo,bar',
      )
      ..addOption(
        'trace-skia-allowlist',
        hide: !verboseHelp,
        help:
            'Filters out all Skia trace events except those that are specified in '
            'this comma separated list of allowed prefixes.',
        valueHelp: 'skia.gpu,skia.shaders',
      )
      ..addFlag(
        'enable-dart-profiling',
        defaultsTo: true,
        help:
            'Whether the Dart VM sampling CPU profiler is enabled. This flag '
            'is only meaningful in debug and profile builds.',
      )
      ..addFlag(
        'enable-software-rendering',
        negatable: false,
        help:
            '(deprecated) Enable rendering using the Skia software backend. '
            'This is useful when testing Flutter on emulators. By default, '
            'Flutter will attempt to either use OpenGL or Vulkan and fall back '
            'to software when neither is available. This option is not supported '
            'when using the Impeller rendering engine.',
        hide: !verboseHelp,
      )
      ..addFlag(
        'skia-deterministic-rendering',
        negatable: false,
        help:
            '(deprecated) When combined with "--enable-software-rendering", this should provide completely '
            'deterministic (i.e. reproducible) Skia rendering. This is useful for testing purposes '
            '(e.g. when comparing screenshots). This option is not supported '
            'when using the Impeller rendering engine.',
        hide: !verboseHelp,
      )
      ..addMultiOption(
        'dart-entrypoint-args',
        abbr: 'a',
        help:
            'Pass a list of arguments to the Dart entrypoint at application '
            'startup. By default this is main(List<String> args). Specify '
            'this option multiple times each with one argument to pass '
            'multiple arguments to the Dart entrypoint. Currently this is '
            'only supported on desktop platforms.',
      )
      ..addFlag(
        'uninstall-first',
        hide: !verboseHelp,
        help:
            'Uninstall previous versions of the app on the device '
            'before reinstalling. Currently only supported on iOS.',
      )
      ..addFlag(
        FlutterOptions.kWebWasmFlag,
        help: 'Compile to WebAssembly rather than JavaScript.\n$kWasmMoreInfo',
        negatable: false,
      );
    usesWebOptions(verboseHelp: verboseHelp);
    usesTargetOption();
    usesPortOptions(verboseHelp: verboseHelp);
    usesIpv6Flag(verboseHelp: verboseHelp);
    usesPubOption();
    usesTrackWidgetCreation(verboseHelp: verboseHelp);
    addNullSafetyModeOptions(hide: !verboseHelp);
    usesDeviceUserOption();
    usesDeviceTimeoutOption();
    usesDeviceConnectionOption();
    addDdsOptions(verboseHelp: verboseHelp);
    addDevToolsOptions(verboseHelp: verboseHelp);
    addServeObservatoryOptions(verboseHelp: verboseHelp);
    addAndroidSpecificBuildOptions(hide: !verboseHelp);
    usesFatalWarningsOption(verboseHelp: verboseHelp);
    addEnableImpellerFlag(verboseHelp: verboseHelp);
    addEnableVulkanValidationFlag(verboseHelp: verboseHelp);
    addEnableEmbedderApiFlag(verboseHelp: verboseHelp);
  }

  bool get traceStartup => boolArg('trace-startup');
  bool get enableDartProfiling => boolArg('enable-dart-profiling');
  bool get cacheSkSL => boolArg('cache-sksl');
  bool get dumpSkpOnShaderCompilation => boolArg('dump-skp-on-shader-compilation');
  bool get purgePersistentCache => boolArg('purge-persistent-cache');
  bool get disableServiceAuthCodes => boolArg('disable-service-auth-codes');
  bool get cacheStartupProfile => boolArg('cache-startup-profile');
  bool get runningWithPrebuiltApplication =>
      argResults![FlutterOptions.kUseApplicationBinary] != null;
  bool get trackWidgetCreation => boolArg('track-widget-creation');
  ImpellerStatus get enableImpeller =>
      ImpellerStatus.fromBool(argResults!['enable-impeller'] as bool?);
  bool get enableVulkanValidation => boolArg('enable-vulkan-validation');
  bool get uninstallFirst => boolArg('uninstall-first');
  bool get enableEmbedderApi => boolArg('enable-embedder-api');

  @override
  bool get refreshWirelessDevices => true;

  @override
  bool get reportNullSafety => true;

  /// Whether to start the application paused by default.
  bool get startPausedDefault;

  String? get route => stringArg('route');

  String? get traceAllowlist => stringArg('trace-allowlist');

  bool get useWasm => boolArg(FlutterOptions.kWebWasmFlag);

  // Keep in sync with the [TestCommand.webRenderer] getter.
  WebRendererMode get webRenderer {
    final List<String> dartDefines = extractDartDefines(
      defineConfigJsonMap: extractDartDefineConfigJsonMap(),
    );
    return WebRendererMode.fromDartDefines(dartDefines, useWasm: useWasm);
  }

  /// Create a debugging options instance for the current `run` or `drive` invocation.
  @visibleForTesting
  @protected
  Future<DebuggingOptions> createDebuggingOptions(bool webMode) async {
    final BuildInfo buildInfo = await getBuildInfo();
    final int? webBrowserDebugPort =
        featureFlags.isWebEnabled && argResults!.wasParsed('web-browser-debug-port')
            ? int.parse(stringArg('web-browser-debug-port')!)
            : null;
    final List<String> webBrowserFlags =
        featureFlags.isWebEnabled ? stringsArg(FlutterOptions.kWebBrowserFlag) : const <String>[];

    final Map<String, String> webHeaders =
        featureFlags.isWebEnabled ? extractWebHeaders() : const <String, String>{};

    if (buildInfo.mode.isRelease) {
      return DebuggingOptions.disabled(
        buildInfo,
        dartEntrypointArgs: stringsArg('dart-entrypoint-args'),
        hostname: featureFlags.isWebEnabled ? stringArg('web-hostname') : '',
        port: featureFlags.isWebEnabled ? stringArg('web-port') : '',
        tlsCertPath: featureFlags.isWebEnabled ? stringArg('web-tls-cert-path') : null,
        tlsCertKeyPath: featureFlags.isWebEnabled ? stringArg('web-tls-cert-key-path') : null,
        webUseSseForDebugProxy:
            featureFlags.isWebEnabled && stringArg('web-server-debug-protocol') == 'sse',
        webUseSseForDebugBackend:
            featureFlags.isWebEnabled && stringArg('web-server-debug-backend-protocol') == 'sse',
        webUseSseForInjectedClient:
            featureFlags.isWebEnabled &&
            stringArg('web-server-debug-injected-client-protocol') == 'sse',
        webEnableExposeUrl: featureFlags.isWebEnabled && boolArg('web-allow-expose-url'),
        webRunHeadless: featureFlags.isWebEnabled && boolArg('web-run-headless'),
        webBrowserDebugPort: webBrowserDebugPort,
        webBrowserFlags: webBrowserFlags,
        webHeaders: webHeaders,
        webRenderer: webRenderer,
        webUseWasm: useWasm,
        enableImpeller: enableImpeller,
        enableVulkanValidation: enableVulkanValidation,
        uninstallFirst: uninstallFirst,
        enableDartProfiling: enableDartProfiling,
        enableEmbedderApi: enableEmbedderApi,
        usingCISystem: usingCISystem,
        debugLogsDirectoryPath: debugLogsDirectoryPath,
      );
    } else {
      return DebuggingOptions.enabled(
        buildInfo,
        startPaused: boolArg('start-paused'),
        disableServiceAuthCodes: boolArg('disable-service-auth-codes'),
        cacheStartupProfile: cacheStartupProfile,
        enableDds: enableDds,
        dartEntrypointArgs: stringsArg('dart-entrypoint-args'),
        dartFlags: stringArg('dart-flags') ?? '',
        useTestFonts: argParser.options.containsKey('use-test-fonts') && boolArg('use-test-fonts'),
        enableSoftwareRendering:
            argParser.options.containsKey('enable-software-rendering') &&
            boolArg('enable-software-rendering'),
        skiaDeterministicRendering:
            argParser.options.containsKey('skia-deterministic-rendering') &&
            boolArg('skia-deterministic-rendering'),
        traceSkia: boolArg('trace-skia'),
        traceAllowlist: traceAllowlist,
        traceSkiaAllowlist: stringArg('trace-skia-allowlist'),
        traceSystrace: boolArg('trace-systrace'),
        traceToFile: stringArg('trace-to-file'),
        endlessTraceBuffer: boolArg('endless-trace-buffer'),
        dumpSkpOnShaderCompilation: dumpSkpOnShaderCompilation,
        cacheSkSL: cacheSkSL,
        purgePersistentCache: purgePersistentCache,
        deviceVmServicePort: deviceVmservicePort,
        hostVmServicePort: hostVmservicePort,
        disablePortPublication: await disablePortPublication,
        ddsPort: ddsPort,
        devToolsServerAddress: devToolsServerAddress,
        verboseSystemLogs: boolArg('verbose-system-logs'),
        hostname: featureFlags.isWebEnabled ? stringArg('web-hostname') : '',
        port: featureFlags.isWebEnabled ? stringArg('web-port') : '',
        tlsCertPath: featureFlags.isWebEnabled ? stringArg('web-tls-cert-path') : null,
        tlsCertKeyPath: featureFlags.isWebEnabled ? stringArg('web-tls-cert-key-path') : null,
        webUseSseForDebugProxy:
            featureFlags.isWebEnabled && stringArg('web-server-debug-protocol') == 'sse',
        webUseSseForDebugBackend:
            featureFlags.isWebEnabled && stringArg('web-server-debug-backend-protocol') == 'sse',
        webUseSseForInjectedClient:
            featureFlags.isWebEnabled &&
            stringArg('web-server-debug-injected-client-protocol') == 'sse',
        webEnableExposeUrl: featureFlags.isWebEnabled && boolArg('web-allow-expose-url'),
        webRunHeadless: featureFlags.isWebEnabled && boolArg('web-run-headless'),
        webBrowserDebugPort: webBrowserDebugPort,
        webBrowserFlags: webBrowserFlags,
        webEnableExpressionEvaluation:
            featureFlags.isWebEnabled && boolArg('web-enable-expression-evaluation'),
        webLaunchUrl: featureFlags.isWebEnabled ? stringArg('web-launch-url') : null,
        webHeaders: webHeaders,
        webRenderer: webRenderer,
        webUseWasm: useWasm,
        vmserviceOutFile: stringArg('vmservice-out-file'),
        fastStart:
            argParser.options.containsKey('fast-start') &&
            boolArg('fast-start') &&
            !runningWithPrebuiltApplication,
        nullAssertions: boolArg('null-assertions'),
        nativeNullAssertions: boolArg('native-null-assertions'),
        enableImpeller: enableImpeller,
        enableVulkanValidation: enableVulkanValidation,
        uninstallFirst: uninstallFirst,
        serveObservatory: boolArg('serve-observatory'),
        enableDartProfiling: enableDartProfiling,
        enableEmbedderApi: enableEmbedderApi,
        usingCISystem: usingCISystem,
        debugLogsDirectoryPath: debugLogsDirectoryPath,
        enableDevTools: boolArg(FlutterCommand.kEnableDevTools),
        ipv6: boolArg(FlutterCommand.ipv6Flag),
        printDtd: boolArg(FlutterGlobalOptions.kPrintDtd, global: true),
      );
    }
  }
}

class RunCommand extends RunCommandBase {
  RunCommand({bool verboseHelp = false}) : super(verboseHelp: verboseHelp) {
    requiresPubspecYaml();
    usesFilesystemOptions(hide: !verboseHelp);
    usesExtraDartFlagOptions(verboseHelp: verboseHelp);
    usesFrontendServerStarterPathOption(verboseHelp: verboseHelp);
    addEnableExperimentation(hide: !verboseHelp);
    usesInitializeFromDillOption(hide: !verboseHelp);
    usesNativeAssetsOption(hide: !verboseHelp);

    // By default, the app should to publish the VM service port over mDNS.
    // This will allow subsequent "flutter attach" commands to connect to the VM
    // without needing to know the port.
    addPublishPort(verboseHelp: verboseHelp);
    addIgnoreDeprecationOption();
    argParser
      ..addFlag(
        'await-first-frame-when-tracing',
        defaultsTo: true,
        help:
            'Whether to wait for the first frame when tracing startup ("--trace-startup"), '
            'or just dump the trace as soon as the application is running. The first frame '
            'is detected by looking for a Timeline event with the name '
            '"${Tracing.firstUsefulFrameEventName}". '
            "By default, the widgets library's binding takes care of sending this event.",
      )
      ..addFlag(
        'use-test-fonts',
        help:
            'Enable (and default to) the "Ahem" font. This is a special font '
            'used in tests to remove any dependencies on the font metrics. It '
            'is enabled when you use "flutter test". Set this flag when running '
            'a test using "flutter run" for debugging purposes. This flag is '
            'only available when running in debug mode.',
      )
      ..addFlag('build', defaultsTo: true, help: 'If necessary, build the app before running.')
      ..addOption('project-root', hide: !verboseHelp, help: 'Specify the project root directory.')
      ..addFlag(
        'machine',
        hide: !verboseHelp,
        negatable: false,
        help:
            'Handle machine structured JSON command input and provide output '
            'and progress in machine friendly format.',
      )
      ..addFlag(
        'hot',
        defaultsTo: kHotReloadDefault,
        help:
            'Run with support for hot reloading. Only available for debug mode. Not available with "--trace-startup".',
      )
      ..addFlag(
        'resident',
        defaultsTo: true,
        hide: !verboseHelp,
        help:
            'Stay resident after launching the application. Not available with "--trace-startup".',
      )
      ..addOption(
        'pid-file',
        help:
            'Specify a file to write the process ID to. '
            'You can send SIGUSR1 to trigger a hot reload '
            'and SIGUSR2 to trigger a hot restart. '
            'The file is created when the signal handlers '
            'are hooked and deleted when they are removed.',
      )
      ..addFlag(
        'report-ready',
        help:
            'Print "ready" to the console after handling a keyboard command.\n'
            'This is primarily useful for tests and other automation, but consider '
            'using "--machine" instead.',
        hide: !verboseHelp,
      )
      ..addFlag(
        'benchmark',
        negatable: false,
        hide: !verboseHelp,
        help:
            'Enable a benchmarking mode. This will run the given application, '
            'measure the startup time and the app restart time, write the '
            'results out to "refresh_benchmark.json", and exit. This flag is '
            'intended for use in generating automated flutter benchmarks.',
      )
      // TODO(zanderso): Off by default with investigating whether this
      // is slower for certain use cases.
      // See: https://github.com/flutter/flutter/issues/49499
      ..addFlag(
        'fast-start',
        help:
            'Whether to quickly bootstrap applications with a minimal app. '
            'Currently this is only supported on Android devices. This option '
            'cannot be paired with "--${FlutterOptions.kUseApplicationBinary}".',
        hide: !verboseHelp,
      );
  }

  @override
  final String name = 'run';

  @override
  DeprecationBehavior get deprecationBehavior =>
      boolArg('ignore-deprecation') ? DeprecationBehavior.ignore : _deviceDeprecationBehavior;
  DeprecationBehavior _deviceDeprecationBehavior = DeprecationBehavior.none;

  @override
  final String description = 'Run your Flutter app on an attached device.';

  @override
  String get category => FlutterCommandCategory.project;

  List<Device>? devices;
  bool webMode = false;

  String? get userIdentifier => stringArg(FlutterOptions.kDeviceUser);

  @override
  bool get startPausedDefault => false;

  @override
  Future<String?> get usagePath async {
    final String? command = await super.usagePath;

    if (devices == null) {
      return command;
    }
    if (devices!.length > 1) {
      return '$command/all';
    }
    return '$command/${getNameForTargetPlatform(await devices![0].targetPlatform)}';
  }

  @override
  Future<CustomDimensions> get usageValues async {
    final AnalyticsUsageValuesRecord record = await _sharedAnalyticsUsageValues;

    return CustomDimensions(
      commandRunIsEmulator: record.runIsEmulator,
      commandRunTargetName: record.runTargetName,
      commandRunTargetOsVersion: record.runTargetOsVersion,
      commandRunModeName: record.runModeName,
      commandRunProjectModule: record.runProjectModule,
      commandRunProjectHostLanguage: record.runProjectHostLanguage,
      commandRunAndroidEmbeddingVersion: record.runAndroidEmbeddingVersion,
      commandRunEnableImpeller: record.runEnableImpeller,
      commandRunIOSInterfaceType: record.runIOSInterfaceType,
      commandRunIsTest: record.runIsTest,
    );
  }

  @override
  Future<analytics.Event> unifiedAnalyticsUsageValues(String commandPath) async {
    final AnalyticsUsageValuesRecord record = await _sharedAnalyticsUsageValues;

    return analytics.Event.commandUsageValues(
      workflow: commandPath,
      commandHasTerminal: hasTerminal,
      runIsEmulator: record.runIsEmulator,
      runTargetName: record.runTargetName,
      runTargetOsVersion: record.runTargetOsVersion,
      runModeName: record.runModeName,
      runProjectModule: record.runProjectModule,
      runProjectHostLanguage: record.runProjectHostLanguage,
      runAndroidEmbeddingVersion: record.runAndroidEmbeddingVersion,
      runEnableImpeller: record.runEnableImpeller,
      runIOSInterfaceType: record.runIOSInterfaceType,
      runIsTest: record.runIsTest,
    );
  }

  late final Future<AnalyticsUsageValuesRecord> _sharedAnalyticsUsageValues =
      (() async {
        String deviceType, deviceOsVersion;
        bool isEmulator;
        bool anyAndroidDevices = false;
        bool anyIOSDevices = false;
        bool anyWirelessIOSDevices = false;

        if (devices == null || devices!.isEmpty) {
          deviceType = 'none';
          deviceOsVersion = 'none';
          isEmulator = false;
        } else if (devices!.length == 1) {
          final Device device = devices![0];
          final TargetPlatform platform = await device.targetPlatform;
          anyAndroidDevices = platform == TargetPlatform.android;
          anyIOSDevices = platform == TargetPlatform.ios;
          if (device is IOSDevice && device.isWirelesslyConnected) {
            anyWirelessIOSDevices = true;
          }
          deviceType = getNameForTargetPlatform(platform);
          deviceOsVersion = await device.sdkNameAndVersion;
          isEmulator = await device.isLocalEmulator;
        } else {
          deviceType = 'multiple';
          deviceOsVersion = 'multiple';
          isEmulator = false;
          for (final Device device in devices!) {
            final TargetPlatform platform = await device.targetPlatform;
            anyAndroidDevices = anyAndroidDevices || (platform == TargetPlatform.android);
            anyIOSDevices = anyIOSDevices || (platform == TargetPlatform.ios);
            if (device is IOSDevice && device.isWirelesslyConnected) {
              anyWirelessIOSDevices = true;
            }
            if (anyAndroidDevices && anyIOSDevices) {
              break;
            }
          }
        }

        String? iOSInterfaceType;
        if (anyIOSDevices) {
          iOSInterfaceType = anyWirelessIOSDevices ? 'wireless' : 'usb';
        }

        String? androidEmbeddingVersion;
        final List<String> hostLanguage = <String>[];
        if (anyAndroidDevices) {
          final AndroidProject androidProject = FlutterProject.current().android;
          if (androidProject.existsSync()) {
            hostLanguage.add(androidProject.isKotlin ? 'kotlin' : 'java');
            androidEmbeddingVersion =
                androidProject.getEmbeddingVersion().toString().split('.').last;
          }
        }
        if (anyIOSDevices) {
          final IosProject iosProject = FlutterProject.current().ios;
          if (iosProject.exists) {
            final Iterable<File> swiftFiles = iosProject.hostAppRoot
                .listSync(recursive: true, followLinks: false)
                .whereType<File>()
                .where((File file) => globals.fs.path.extension(file.path) == '.swift');
            hostLanguage.add(swiftFiles.isNotEmpty ? 'swift' : 'objc');
          }
        }

        final BuildInfo buildInfo = await getBuildInfo();
        final String modeName = buildInfo.modeName;
        return (
          runIsEmulator: isEmulator,
          runTargetName: deviceType,
          runTargetOsVersion: deviceOsVersion,
          runModeName: modeName,
          runProjectModule: project.isModule,
          runProjectHostLanguage: hostLanguage.join(','),
          runAndroidEmbeddingVersion: androidEmbeddingVersion,
          runEnableImpeller: enableImpeller.asBool,
          runIOSInterfaceType: iOSInterfaceType,
          runIsTest: targetFile.endsWith('_test.dart'),
        );
      })();

  @override
  bool get shouldRunPub {
    // If we are running with a prebuilt application, do not run pub.
    if (runningWithPrebuiltApplication) {
      return false;
    }

    return super.shouldRunPub;
  }

  bool shouldUseHotMode(BuildInfo buildInfo) {
    final bool hotArg = boolArg('hot');
    final bool shouldUseHotMode = hotArg && !traceStartup;
    return buildInfo.isDebug && shouldUseHotMode;
  }

  bool get stayResident => boolArg('resident');
  bool get awaitFirstFrameWhenTracing => boolArg('await-first-frame-when-tracing');

  @override
  Future<void> validateCommand() async {
    // When running with a prebuilt application, no command validation is
    // necessary.
    if (!runningWithPrebuiltApplication) {
      await super.validateCommand();
    }

    devices = await findAllTargetDevices();
    if (devices == null) {
      throwToolExit(null);
    }

    if (devices!.length == 1 && devices!.first is MacOSDesignedForIPadDevice) {
      throwToolExit('Mac Designed for iPad is currently not supported for flutter run -d.');
    }

    if (globals.deviceManager!.hasSpecifiedAllDevices) {
      devices?.removeWhere((Device device) => device is MacOSDesignedForIPadDevice);
    }

    if (globals.deviceManager!.hasSpecifiedAllDevices && runningWithPrebuiltApplication) {
      throwToolExit(
        'Using "-d all" with "--${FlutterOptions.kUseApplicationBinary}" is not supported',
      );
    }

    if (userIdentifier != null &&
        devices!.every((Device device) => device.platformType != PlatformType.android)) {
      throwToolExit(
        '--${FlutterOptions.kDeviceUser} is only supported for Android. At least one Android device is required.',
      );
    }

    if (devices!.any((Device device) => device is AndroidDevice)) {
      _deviceDeprecationBehavior = DeprecationBehavior.exit;
    }

    // Only support "web mode" with a single web device due to resident runner
    // refactoring required otherwise.
    webMode =
        featureFlags.isWebEnabled &&
        devices!.length == 1 &&
        await devices!.single.targetPlatform == TargetPlatform.web_javascript;

    if (useWasm && !webMode) {
      throwToolExit('--wasm is only supported on the web platform');
    }

    if (webRenderer.isDeprecated) {
      globals.logger.printWarning(webRenderer.deprecationWarning);
    }

    if (webRenderer == WebRendererMode.skwasm && !useWasm) {
      throwToolExit('Skwasm renderer requires --wasm');
    }

    final String? flavor = stringArg('flavor');
    final bool flavorsSupportedOnEveryDevice = devices!.every(
      (Device device) => device.supportsFlavors,
    );
    if (flavor != null && !flavorsSupportedOnEveryDevice) {
      globals.printWarning(
        '--flavor is only supported for Android, macOS, and iOS devices. '
        'Flavor-related features may not function properly and could '
        'behave differently in a future release.',
      );
    }
  }

  @visibleForTesting
  Future<ResidentRunner> createRunner({
    required bool hotMode,
    required List<FlutterDevice> flutterDevices,
    required String? applicationBinaryPath,
    required FlutterProject flutterProject,
  }) async {
    if (hotMode && !webMode) {
      return HotRunner(
        flutterDevices,
        target: targetFile,
        debuggingOptions: await createDebuggingOptions(webMode),
        benchmarkMode: boolArg('benchmark'),
        applicationBinary:
            applicationBinaryPath == null ? null : globals.fs.file(applicationBinaryPath),
        projectRootPath: stringArg('project-root'),
        dillOutputPath: stringArg('output-dill'),
        stayResident: stayResident,
        analytics: globals.analytics,
        nativeAssetsYamlFile: stringArg(FlutterOptions.kNativeAssetsYamlFile),
      );
    } else if (webMode) {
      return webRunnerFactory!.createWebRunner(
        flutterDevices.single,
        target: targetFile,
        flutterProject: flutterProject,
        debuggingOptions: await createDebuggingOptions(webMode),
        stayResident: stayResident,
        fileSystem: globals.fs,
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        logger: globals.logger,
        systemClock: globals.systemClock,
      );
    }
    return ColdRunner(
      flutterDevices,
      target: targetFile,
      debuggingOptions: await createDebuggingOptions(webMode),
      traceStartup: traceStartup,
      awaitFirstFrameWhenTracing: awaitFirstFrameWhenTracing,
      applicationBinary:
          applicationBinaryPath == null ? null : globals.fs.file(applicationBinaryPath),
      stayResident: stayResident,
    );
  }

  @visibleForTesting
  Daemon createMachineDaemon() {
    final Daemon daemon = Daemon(
      DaemonConnection(
        daemonStreams: DaemonStreams.fromStdio(globals.stdio, logger: globals.logger),
        logger: globals.logger,
      ),
      notifyingLogger:
          (globals.logger is NotifyingLogger)
              ? globals.logger as NotifyingLogger
              : NotifyingLogger(verbose: globals.logger.isVerbose, parent: globals.logger),
      logToStdout: true,
    );
    return daemon;
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    final BuildInfo buildInfo = await getBuildInfo();
    // Enable hot mode by default if `--no-hot` was not passed and we are in
    // debug mode.
    final bool hotMode = shouldUseHotMode(buildInfo);
    final String? applicationBinaryPath = stringArg(FlutterOptions.kUseApplicationBinary);

    if (boolArg('machine')) {
      if (devices!.length > 1) {
        throwToolExit('"--machine" does not support "-d all".');
      }
      final Daemon daemon = createMachineDaemon();
      late AppInstance app;
      try {
        app = await daemon.appDomain.startApp(
          devices!.first,
          globals.fs.currentDirectory.path,
          targetFile,
          route,
          await createDebuggingOptions(webMode),
          hotMode,
          applicationBinary:
              applicationBinaryPath == null ? null : globals.fs.file(applicationBinaryPath),
          trackWidgetCreation: trackWidgetCreation,
          projectRootPath: stringArg('project-root'),
          packagesFilePath: globalResults![FlutterGlobalOptions.kPackagesOption] as String?,
          dillOutputPath: stringArg('output-dill'),
          userIdentifier: userIdentifier,
        );
      } on Exception catch (error) {
        throwToolExit(error.toString());
      }
      final DateTime appStartedTime = globals.systemClock.now();
      final int result = await app.runner.waitForAppToFinish();
      if (result != 0) {
        throwToolExit(null, exitCode: result);
      }
      return FlutterCommandResult(
        ExitStatus.success,
        timingLabelParts: <String>['daemon'],
        endTimeOverride: appStartedTime,
      );
    }
    globals.terminal.usesTerminalUi = true;

    final BuildMode buildMode = getBuildMode();
    for (final Device device in devices!) {
      if (!await device.supportsRuntimeMode(buildMode)) {
        throwToolExit(
          '${sentenceCase(getFriendlyModeName(buildMode))} '
          'mode is not supported by ${device.displayName}.',
        );
      }
      if (hotMode) {
        if (!device.supportsHotReload) {
          throwToolExit(
            'Hot reload is not supported by ${device.displayName}. '
            'Run with "--no-hot".',
          );
        }
      }
    }

    List<String>? expFlags;
    if (argParser.options.containsKey(FlutterOptions.kEnableExperiment) &&
        stringsArg(FlutterOptions.kEnableExperiment).isNotEmpty) {
      expFlags = stringsArg(FlutterOptions.kEnableExperiment);
    }
    final List<FlutterDevice> flutterDevices = <FlutterDevice>[
      for (final Device device in devices!)
        await FlutterDevice.create(
          device,
          experimentalFlags: expFlags,
          target: targetFile,
          buildInfo: buildInfo,
          userIdentifier: userIdentifier,
          platform: globals.platform,
        ),
    ];

    final ResidentRunner runner = await createRunner(
      applicationBinaryPath: applicationBinaryPath,
      flutterDevices: flutterDevices,
      flutterProject: project,
      hotMode: hotMode,
    );

    DateTime? appStartedTime;
    // Sync completer so the completing agent attaching to the resident doesn't
    // need to know about analytics.
    //
    // Do not add more operations to the future.
    final Completer<void> appStartedTimeRecorder = Completer<void>.sync();

    TerminalHandler? handler;
    // This callback can't throw.
    unawaited(
      appStartedTimeRecorder.future.then<void>((_) {
        appStartedTime = globals.systemClock.now();
        if (stayResident) {
          handler =
              TerminalHandler(
                  runner,
                  logger: globals.logger,
                  terminal: globals.terminal,
                  signals: globals.signals,
                  processInfo: globals.processInfo,
                  reportReady: boolArg('report-ready'),
                  pidFile: stringArg('pid-file'),
                )
                ..registerSignalHandlers()
                ..setupTerminal();
        }
      }),
    );
    try {
      final int? result = await runner.run(
        appStartedCompleter: appStartedTimeRecorder,
        route: route,
      );
      handler?.stop();
      if (result != 0) {
        throwToolExit(null, exitCode: result);
      }
    } on RPCError catch (error) {
      if (error.code == RPCErrorKind.kServiceDisappeared.code ||
          error.message.contains('Service connection disposed')) {
        throwToolExit('Lost connection to device.');
      }
      rethrow;
    } finally {
      // However we exited from the runner, ensure the terminal has line mode
      // and echo mode enabled before we return the user to the shell.
      try {
        globals.terminal.singleCharMode = false;
      } on StdinException {
        // Do nothing, if the STDIN handle is no longer available, there is nothing actionable for us to do at this point
      }
    }
    return FlutterCommandResult(
      ExitStatus.success,
      timingLabelParts: <String?>[
        if (hotMode) 'hot' else 'cold',
        getBuildMode().cliName,
        if (devices!.length == 1)
          getNameForTargetPlatform(await devices![0].targetPlatform)
        else
          'multiple',
        if (devices!.length == 1 && await devices![0].isLocalEmulator) 'emulator' else null,
      ],
      endTimeOverride: appStartedTime,
    );
  }
}

/// Schema for the usage values to send for analytics reporting.
typedef AnalyticsUsageValuesRecord =
    ({
      String? runAndroidEmbeddingVersion,
      bool? runEnableImpeller,
      String? runIOSInterfaceType,
      bool runIsEmulator,
      bool runIsTest,
      String runModeName,
      String runProjectHostLanguage,
      bool runProjectModule,
      String runTargetName,
      String runTargetOsVersion,
    });
