// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:process/process.dart';
import 'package:unified_analytics/unified_analytics.dart';

import 'android/android_sdk.dart';
import 'android/android_studio.dart';
import 'android/gradle_utils.dart';
import 'android/java.dart';
import 'artifacts.dart';
import 'base/bot_detector.dart';
import 'base/config.dart';
import 'base/context.dart';
import 'base/error_handling_io.dart';
import 'base/file_system.dart';
import 'base/io.dart';
import 'base/logger.dart';
import 'base/net.dart';
import 'base/os.dart';
import 'base/platform.dart';
import 'base/process.dart';
import 'base/signals.dart';
import 'base/template.dart';
import 'base/terminal.dart';
import 'base/time.dart';
import 'base/user_messages.dart';
import 'build_system/build_system.dart';
import 'build_system/build_targets.dart';
import 'cache.dart';
import 'custom_devices/custom_devices_config.dart';
import 'device.dart';
import 'doctor.dart';
import 'ios/ios_workflow.dart';
import 'ios/plist_parser.dart';
import 'ios/simulators.dart';
import 'ios/xcodeproj.dart';
import 'macos/cocoapods.dart';
import 'macos/cocoapods_validator.dart';
import 'macos/xcdevice.dart';
import 'macos/xcode.dart';
import 'native_assets.dart';
import 'persistent_tool_state.dart';
import 'pre_run_validator.dart';
import 'project.dart';
import 'reporting/crash_reporting.dart';
import 'reporting/reporting.dart';
import 'runner/flutter_command.dart';
import 'runner/local_engine.dart';
import 'version.dart';

// TODO(ianh): We should remove all the global variables and replace them with
// arguments (to constructors, methods, etc, as appropriate).

Artifacts? get artifacts => context.get<Artifacts>();
BuildSystem get buildSystem => context.get<BuildSystem>()!;
BuildTargets get buildTargets => context.get<BuildTargets>()!;
Cache get cache => context.get<Cache>()!;
CocoaPodsValidator? get cocoapodsValidator => context.get<CocoaPodsValidator>();
Config get config => context.get<Config>()!;
CrashReporter? get crashReporter => context.get<CrashReporter>();
DeviceManager? get deviceManager => context.get<DeviceManager>();
Doctor? get doctor => context.get<Doctor>();
HttpClientFactory? get httpClientFactory => context.get<HttpClientFactory>();
IOSSimulatorUtils? get iosSimulatorUtils => context.get<IOSSimulatorUtils>();
Logger get logger => context.get<Logger>()!;
OperatingSystemUtils get os => context.get<OperatingSystemUtils>()!;
Signals get signals => context.get<Signals>() ?? LocalSignals.instance;
AndroidStudio? get androidStudio => context.get<AndroidStudio>();
AndroidSdk? get androidSdk => context.get<AndroidSdk>();
FlutterVersion get flutterVersion => context.get<FlutterVersion>()!;
Usage get flutterUsage => context.get<Usage>()!;
XcodeProjectInterpreter? get xcodeProjectInterpreter => context.get<XcodeProjectInterpreter>();
XCDevice? get xcdevice => context.get<XCDevice>();
Xcode? get xcode => context.get<Xcode>();
IOSWorkflow? get iosWorkflow => context.get<IOSWorkflow>();
LocalEngineLocator? get localEngineLocator => context.get<LocalEngineLocator>();

PersistentToolState? get persistentToolState => PersistentToolState.instance;

BotDetector get botDetector => context.get<BotDetector>() ?? _defaultBotDetector;
final BotDetector _defaultBotDetector = BotDetector(
  httpClientFactory: context.get<HttpClientFactory>() ?? () => HttpClient(),
  platform: platform,
  persistentToolState:
      persistentToolState ??
      PersistentToolState(fileSystem: fs, logger: logger, platform: platform),
);
Future<bool> get isRunningOnBot => botDetector.isRunningOnBot;

// Analytics instance for package:unified_analytics for analytics
// reporting for all Flutter and Dart related tooling
Analytics get analytics => context.get<Analytics>()!;

/// Currently active implementation of the file system.
///
/// By default it uses local disk-based implementation. Override this in tests
/// with [MemoryFileSystem].
FileSystem get fs => ErrorHandlingFileSystem(
  delegate: context.get<FileSystem>() ?? localFileSystem,
  platform: platform,
);

FileSystemUtils get fsUtils =>
    context.get<FileSystemUtils>() ?? FileSystemUtils(fileSystem: fs, platform: platform);

const ProcessManager _kLocalProcessManager = LocalProcessManager();

/// The active process manager.
ProcessManager get processManager => context.get<ProcessManager>() ?? _kLocalProcessManager;
ProcessUtils get processUtils => context.get<ProcessUtils>()!;

const Platform _kLocalPlatform = LocalPlatform();
Platform get platform => context.get<Platform>() ?? _kLocalPlatform;

UserMessages get userMessages => context.get<UserMessages>()!;

final OutputPreferences _default = OutputPreferences(
  wrapText: stdio.hasTerminal,
  showColor: platform.stdoutSupportsAnsi,
  stdio: stdio,
);
OutputPreferences get outputPreferences => context.get<OutputPreferences>() ?? _default;

/// The current system clock instance.
SystemClock get systemClock => context.get<SystemClock>() ?? _systemClock;
SystemClock _systemClock = const SystemClock();

ProcessInfo get processInfo => context.get<ProcessInfo>()!;

/// Display an error level message to the user. Commands should use this if they
/// fail in some way.
///
/// Set [emphasis] to true to make the output bold if it's supported.
/// Set [color] to a [TerminalColor] to color the output, if the logger
/// supports it. The [color] defaults to [TerminalColor.red].
void printError(
  String message, {
  StackTrace? stackTrace,
  bool? emphasis,
  TerminalColor? color,
  int? indent,
  int? hangingIndent,
  bool? wrap,
}) {
  logger.printError(
    message,
    stackTrace: stackTrace,
    emphasis: emphasis ?? false,
    color: color,
    indent: indent,
    hangingIndent: hangingIndent,
    wrap: wrap,
  );
}

/// Display a warning level message to the user. Commands should use this if they
/// have important warnings to convey that aren't fatal.
///
/// Set [emphasis] to true to make the output bold if it's supported.
/// Set [color] to a [TerminalColor] to color the output, if the logger
/// supports it. The [color] defaults to [TerminalColor.cyan].
void printWarning(
  String message, {
  bool? emphasis,
  TerminalColor? color,
  int? indent,
  int? hangingIndent,
  bool? wrap,
}) {
  logger.printWarning(
    message,
    emphasis: emphasis ?? false,
    color: color,
    indent: indent,
    hangingIndent: hangingIndent,
    wrap: wrap,
  );
}

/// Display normal output of the command. This should be used for things like
/// progress messages, success messages, or just normal command output.
///
/// Set `emphasis` to true to make the output bold if it's supported.
///
/// Set `newline` to false to skip the trailing linefeed.
///
/// If `indent` is provided, each line of the message will be prepended by the
/// specified number of whitespaces.
void printStatus(
  String message, {
  bool? emphasis,
  bool? newline,
  TerminalColor? color,
  int? indent,
  int? hangingIndent,
  bool? wrap,
}) {
  logger.printStatus(
    message,
    emphasis: emphasis ?? false,
    color: color,
    newline: newline ?? true,
    indent: indent,
    hangingIndent: hangingIndent,
    wrap: wrap,
  );
}

/// Display the [message] inside a box.
///
/// For example, this is the generated output:
///
///   ┌─ [title] ─┐
///   │ [message] │
///   └───────────┘
///
/// If a terminal is attached, the lines in [message] are automatically wrapped based on
/// the available columns.
void printBox(String message, {String? title}) {
  logger.printBox(message, title: title);
}

/// Use this for verbose tracing output. Users can turn this output on in order
/// to help diagnose issues with the toolchain or with their setup.
void printTrace(String message) => logger.printTrace(message);

AnsiTerminal get terminal {
  return context.get<AnsiTerminal>() ?? _defaultAnsiTerminal;
}

final AnsiTerminal _defaultAnsiTerminal = AnsiTerminal(
  stdio: stdio,
  platform: platform,
  now: DateTime.now(),
  shutdownHooks: shutdownHooks,
);

/// The global Stdio wrapper.
Stdio get stdio => context.get<Stdio>() ?? (_stdioInstance ??= Stdio());
Stdio? _stdioInstance;

PlistParser get plistParser =>
    context.get<PlistParser>() ??
    (_plistInstance ??= PlistParser(
      fileSystem: fs,
      processManager: processManager,
      logger: logger,
    ));
PlistParser? _plistInstance;

/// The global template renderer.
TemplateRenderer get templateRenderer => context.get<TemplateRenderer>()!;

/// Global [ShutdownHooks] that should be run before the tool process exits.
///
/// This is depended on by [localFileSystem] which is called before any
/// [Context] is set up, and thus this cannot be a Context getter.
final ShutdownHooks shutdownHooks = ShutdownHooks();

// Unless we're in a test of this class's signal handling features, we must
// have only one instance created with the singleton LocalSignals instance
// and the catchable signals it considers to be fatal.
LocalFileSystem? _instance;
LocalFileSystem get localFileSystem =>
    _instance ??= LocalFileSystem(LocalSignals.instance, Signals.defaultExitSignals, shutdownHooks);

/// Gradle utils in the current [AppContext].
GradleUtils? get gradleUtils => context.get<GradleUtils>();

CocoaPods? get cocoaPods => context.get<CocoaPods>();

FlutterProjectFactory get projectFactory {
  return context.get<FlutterProjectFactory>() ??
      FlutterProjectFactory(logger: logger, fileSystem: fs);
}

CustomDevicesConfig get customDevicesConfig => context.get<CustomDevicesConfig>()!;

PreRunValidator get preRunValidator =>
    context.get<PreRunValidator>() ?? const NoOpPreRunValidator();

// Used to build RegExp instances which can detect the VM service message.
final RegExp kVMServiceMessageRegExp = RegExp(
  r'The Dart VM service is listening on ((http|//)[a-zA-Z0-9:/=_\-\.\[\]]+)',
);

// The official tool no longer allows non-null safe builds. This can be
// overridden in other clients.
NonNullSafeBuilds get nonNullSafeBuilds =>
    context.get<NonNullSafeBuilds>() ?? NonNullSafeBuilds.notAllowed;

/// Contains information about the JRE/JDK to use for Java-dependent operations.
///
/// A value of [null] indicates that no installation of java could be found on
/// the host machine.
Java? get java => context.get<Java>();

TestCompilerNativeAssetsBuilder? get nativeAssetsBuilder =>
    context.get<TestCompilerNativeAssetsBuilder>();
