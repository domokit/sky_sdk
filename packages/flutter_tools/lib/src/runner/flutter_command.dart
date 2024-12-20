// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/args.dart';
import 'package:args/command_runner.dart';
import 'package:file/file.dart';
import 'package:meta/meta.dart';
import 'package:package_config/package_config_types.dart';
import 'package:unified_analytics/unified_analytics.dart';

import '../application_package.dart';
import '../base/common.dart';
import '../base/context.dart';
import '../base/io.dart' as io;
import '../base/io.dart';
import '../base/os.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../build_system/build_system.dart';
import '../bundle.dart' as bundle;
import '../cache.dart';
import '../convert.dart';
import '../dart/generate_synthetic_packages.dart';
import '../dart/package_map.dart';
import '../dart/pub.dart';
import '../device.dart';
import '../features.dart';
import '../globals.dart' as globals;
import '../preview_device.dart';
import '../project.dart';
import '../reporting/reporting.dart';
import '../reporting/unified_analytics.dart';
import 'flutter_command_runner.dart';
import 'target_devices.dart';

export '../cache.dart' show DevelopmentArtifact;

abstract class DotEnvRegex {
  // Dot env multi-line block value regex
  static final RegExp multiLineBlock = RegExp(r'^\s*([a-zA-Z_]+[a-zA-Z0-9_]*)\s*=\s*"""\s*(.*)$');

  // Dot env full line value regex (eg FOO=bar)
  // Entire line will be matched including key and value
  static final RegExp keyValue = RegExp(r'^\s*([a-zA-Z_]+[a-zA-Z0-9_]*)\s*=\s*(.*)?$');

  // Dot env value wrapped in double quotes regex (eg FOO="bar")
  // Value between double quotes will be matched (eg only bar in "bar")
  static final RegExp doubleQuotedValue = RegExp(r'^"(.*)"\s*(\#\s*.*)?$');

  // Dot env value wrapped in single quotes regex (eg FOO='bar')
  // Value between single quotes will be matched (eg only bar in 'bar')
  static final RegExp singleQuotedValue = RegExp(r"^'(.*)'\s*(\#\s*.*)?$");

  // Dot env value wrapped in back quotes regex (eg FOO=`bar`)
  // Value between back quotes will be matched (eg only bar in `bar`)
  static final RegExp backQuotedValue = RegExp(r'^`(.*)`\s*(\#\s*.*)?$');

  // Dot env value without quotes regex (eg FOO=bar)
  // Value without quotes will be matched (eg full value after the equals sign)
  static final RegExp unquotedValue = RegExp(r'^([^#\n\s]*)\s*(?:\s*#\s*(.*))?$');
}

abstract class _HttpRegex {
  // https://datatracker.ietf.org/doc/html/rfc7230#section-3.2
  static const String _vchar = r'\x21-\x7E';
  static const String _spaceOrTab = r'\x20\x09';
  static const String _nonDelimiterVchar =
      r'\x21\x23-\x27\x2A\x2B\x2D\x2E\x30-\x39\x41-\x5A\x5E-\x7A\x7C\x7E';

  // --web-header is provided as key=value for consistency with --dart-define
  static final RegExp httpHeader = RegExp(
    '^([$_nonDelimiterVchar]+)'
    r'\s*=\s*'
    '([$_vchar$_spaceOrTab]+)'
    r'$',
  );
}

enum ExitStatus { success, warning, fail, killed }

/// [FlutterCommand]s' subclasses' [FlutterCommand.runCommand] can optionally
/// provide a [FlutterCommandResult] to furnish additional information for
/// analytics.
class FlutterCommandResult {
  const FlutterCommandResult(this.exitStatus, {this.timingLabelParts, this.endTimeOverride});

  /// A command that succeeded. It is used to log the result of a command invocation.
  factory FlutterCommandResult.success() {
    return const FlutterCommandResult(ExitStatus.success);
  }

  /// A command that exited with a warning. It is used to log the result of a command invocation.
  factory FlutterCommandResult.warning() {
    return const FlutterCommandResult(ExitStatus.warning);
  }

  /// A command that failed. It is used to log the result of a command invocation.
  factory FlutterCommandResult.fail() {
    return const FlutterCommandResult(ExitStatus.fail);
  }

  final ExitStatus exitStatus;

  /// Optional data that can be appended to the timing event.
  /// https://developers.google.com/analytics/devguides/collection/analyticsjs/field-reference#timingLabel
  /// Do not add PII.
  final List<String?>? timingLabelParts;

  /// Optional epoch time when the command's non-interactive wait time is
  /// complete during the command's execution. Use to measure user perceivable
  /// latency without measuring user interaction time.
  ///
  /// [FlutterCommand] will automatically measure and report the command's
  /// complete time if not overridden.
  final DateTime? endTimeOverride;

  @override
  String toString() => exitStatus.name;
}

/// Common flutter command line options.
abstract final class FlutterOptions {
  static const String kFrontendServerStarterPath = 'frontend-server-starter-path';
  static const String kExtraFrontEndOptions = 'extra-front-end-options';
  static const String kExtraGenSnapshotOptions = 'extra-gen-snapshot-options';
  static const String kEnableExperiment = 'enable-experiment';
  static const String kFileSystemRoot = 'filesystem-root';
  static const String kFileSystemScheme = 'filesystem-scheme';
  static const String kSplitDebugInfoOption = 'split-debug-info';
  static const String kDartObfuscationOption = 'obfuscate';
  static const String kDartDefinesOption = 'dart-define';
  static const String kDartDefineFromFileOption = 'dart-define-from-file';
  static const String kBundleSkSLPathOption = 'bundle-sksl-path';
  static const String kPerformanceMeasurementFile = 'performance-measurement-file';
  static const String kNullSafety = 'sound-null-safety';
  static const String kDeviceUser = 'device-user';
  static const String kDeviceTimeout = 'device-timeout';
  static const String kDeviceConnection = 'device-connection';
  static const String kAnalyzeSize = 'analyze-size';
  static const String kCodeSizeDirectory = 'code-size-directory';
  static const String kNullAssertions = 'null-assertions';
  static const String kAndroidGradleDaemon = 'android-gradle-daemon';
  static const String kDeferredComponents = 'deferred-components';
  static const String kAndroidProjectArgs = 'android-project-arg';
  static const String kAndroidSkipBuildDependencyValidation =
      'android-skip-build-dependency-validation';
  static const String kInitializeFromDill = 'initialize-from-dill';
  static const String kAssumeInitializeFromDillUpToDate = 'assume-initialize-from-dill-up-to-date';
  static const String kNativeAssetsYamlFile = 'native-assets-yaml-file';
  static const String kFatalWarnings = 'fatal-warnings';
  static const String kUseApplicationBinary = 'use-application-binary';
  static const String kWebBrowserFlag = 'web-browser-flag';
  static const String kWebResourcesCdnFlag = 'web-resources-cdn';
  static const String kWebWasmFlag = 'wasm';
}

/// flutter command categories for usage.
abstract final class FlutterCommandCategory {
  static const String sdk = 'Flutter SDK';
  static const String project = 'Project';
  static const String tools = 'Tools & Devices';
}

abstract class FlutterCommand extends Command<void> {
  /// The currently executing command (or sub-command).
  ///
  /// Will be `null` until the top-most command has begun execution.
  static FlutterCommand? get current => context.get<FlutterCommand>();

  /// The option name for a custom VM Service port.
  static const String vmServicePortOption = 'vm-service-port';

  /// The option name for a custom VM Service port.
  static const String observatoryPortOption = 'observatory-port';

  /// The option name for a custom DevTools server address.
  static const String kDevToolsServerAddress = 'devtools-server-address';

  /// The flag name for whether to launch the DevTools or not.
  static const String kEnableDevTools = 'devtools';

  /// The flag name for whether or not to use ipv6.
  static const String ipv6Flag = 'ipv6';

  @override
  ArgParser get argParser => _argParser;
  final ArgParser _argParser = ArgParser(
    usageLineLength:
        globals.outputPreferences.wrapText ? globals.outputPreferences.wrapColumn : null,
  );

  @override
  FlutterCommandRunner? get runner => super.runner as FlutterCommandRunner?;

  bool _requiresPubspecYaml = false;

  /// Whether this command uses the 'target' option.
  bool _usesTargetOption = false;

  bool _usesPubOption = false;

  bool _usesPortOption = false;

  bool _usesIpv6Flag = false;

  bool _usesFatalWarnings = false;

  DeprecationBehavior get deprecationBehavior => DeprecationBehavior.none;

  bool get shouldRunPub => _usesPubOption && boolArg('pub');

  bool get shouldUpdateCache => true;

  bool get deprecated => false;

  ProcessInfo get processInfo => globals.processInfo;

  /// When the command runs and this is true, trigger an async process to
  /// discover devices from discoverers that support wireless devices for an
  /// extended amount of time and refresh the device cache with the results.
  bool get refreshWirelessDevices => false;

  @override
  bool get hidden => deprecated;

  bool _excludeDebug = false;
  bool _excludeRelease = false;

  /// Grabs the [Analytics] instance from the global context. It is defined
  /// at the [FlutterCommand] level to enable any classes that extend it to
  /// easily reference it or overwrite as necessary.
  Analytics get analytics => globals.analytics;

  void requiresPubspecYaml() {
    _requiresPubspecYaml = true;
  }

  void usesWebOptions({required bool verboseHelp}) {
    argParser.addMultiOption(
      'web-header',
      help:
          'Additional key-value pairs that will added by the web server '
          'as headers to all responses. Multiple headers can be passed by '
          'repeating "--web-header" multiple times.',
      valueHelp: 'X-Custom-Header=header-value',
      splitCommas: false,
      hide: !verboseHelp,
    );
    argParser.addOption(
      'web-hostname',
      defaultsTo: 'localhost',
      help:
          'The hostname that the web server will use to resolve an IP to serve '
          'from. The unresolved hostname is used to launch Chrome when using '
          'the chrome Device. The name "any" may also be used to serve on any '
          'IPV4 for either the Chrome or web-server device.',
      hide: !verboseHelp,
    );
    argParser.addOption(
      'web-port',
      help:
          'The host port to serve the web application from. If not provided, the tool '
          'will select a random open port on the host.',
      hide: !verboseHelp,
    );
    argParser.addOption(
      'web-tls-cert-path',
      help:
          'The certificate that host will use to serve using TLS connection. '
          'If not provided, the tool will use default http scheme.',
    );
    argParser.addOption(
      'web-tls-cert-key-path',
      help:
          'The certificate key that host will use to authenticate cert. '
          'If not provided, the tool will use default http scheme.',
    );
    argParser.addOption(
      'web-server-debug-protocol',
      allowed: <String>['sse', 'ws'],
      defaultsTo: 'ws',
      help:
          'The protocol (SSE or WebSockets) to use for the debug service proxy '
          'when using the Web Server device and Dart Debug extension. '
          'This is useful for editors/debug adapters that do not support debugging '
          'over SSE (the default protocol for Web Server/Dart Debugger extension).',
      hide: !verboseHelp,
    );
    argParser.addOption(
      'web-server-debug-backend-protocol',
      allowed: <String>['sse', 'ws'],
      defaultsTo: 'ws',
      help:
          'The protocol (SSE or WebSockets) to use for the Dart Debug Extension '
          'backend service when using the Web Server device. '
          'Using WebSockets can improve performance but may fail when connecting through '
          'some proxy servers.',
      hide: !verboseHelp,
    );
    argParser.addOption(
      'web-server-debug-injected-client-protocol',
      allowed: <String>['sse', 'ws'],
      defaultsTo: 'ws',
      help:
          'The protocol (SSE or WebSockets) to use for the injected client '
          'when using the Web Server device. '
          'Using WebSockets can improve performance but may fail when connecting through '
          'some proxy servers.',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'web-allow-expose-url',
      help:
          'Enables daemon-to-editor requests (app.exposeUrl) for exposing URLs '
          'when running on remote machines.',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'web-run-headless',
      help:
          'Launches the browser in headless mode. Currently only Chrome '
          'supports this option.',
      hide: !verboseHelp,
    );
    argParser.addOption(
      'web-browser-debug-port',
      help:
          'The debug port the browser should use. If not specified, a '
          'random port is selected. Currently only Chrome supports this option. '
          'It serves the Chrome DevTools Protocol '
          '(https://chromedevtools.github.io/devtools-protocol/).',
      hide: !verboseHelp,
    );
    argParser.addFlag(
      'web-enable-expression-evaluation',
      defaultsTo: true,
      help: 'Enables expression evaluation in the debugger.',
      hide: !verboseHelp,
    );
    argParser.addOption(
      'web-launch-url',
      help:
          'The URL to provide to the browser. Defaults to an HTTP URL with the host '
          'name of "--web-hostname", the port of "--web-port", and the path set to "/".',
    );
    argParser.addMultiOption(
      FlutterOptions.kWebBrowserFlag,
      help:
          'Additional flag to pass to a browser instance at startup.\n'
          'Chrome: https://www.chromium.org/developers/how-tos/run-chromium-with-flags/\n'
          'Firefox: https://wiki.mozilla.org/Firefox/CommandLineOptions\n'
          'Multiple flags can be passed by repeating "--${FlutterOptions.kWebBrowserFlag}" multiple times.',
      valueHelp: '--foo=bar',
      hide: !verboseHelp,
    );
  }

  void usesTargetOption() {
    argParser.addOption(
      'target',
      abbr: 't',
      defaultsTo: bundle.defaultMainPath,
      help:
          'The main entry-point file of the application, as run on the device.\n'
          'If the "--target" option is omitted, but a file name is provided on '
          'the command line, then that is used instead.',
      valueHelp: 'path',
    );
    _usesTargetOption = true;
  }

  void usesFatalWarningsOption({required bool verboseHelp}) {
    argParser.addFlag(
      FlutterOptions.kFatalWarnings,
      hide: !verboseHelp,
      help:
          'Causes the command to fail if warnings are sent to the console '
          'during its execution.',
    );
    _usesFatalWarnings = true;
  }

  String get targetFile {
    if (argResults?.wasParsed('target') ?? false) {
      return stringArg('target')!;
    }
    final List<String>? rest = argResults?.rest;
    if (rest != null && rest.isNotEmpty) {
      return rest.first;
    }
    return bundle.defaultMainPath;
  }

  /// Indicates if the current command running has a terminal attached.
  bool get hasTerminal => globals.stdio.hasTerminal;

  /// Path to the Dart's package config file.
  ///
  /// This can be overridden by some of its subclasses.
  String? get packagesPath => stringArg(FlutterGlobalOptions.kPackagesOption, global: true);

  /// Whether flutter is being run from our CI.
  ///
  /// This is true if `--ci` is passed to the command or if environment
  /// variable `LUCI_CI` is `True`.
  bool get usingCISystem {
    return boolArg(FlutterGlobalOptions.kContinuousIntegrationFlag, global: true) ||
        globals.platform.environment['LUCI_CI'] == 'True';
  }

  String? get debugLogsDirectoryPath =>
      stringArg(FlutterGlobalOptions.kDebugLogsDirectoryFlag, global: true);

  /// The value of the `--filesystem-scheme` argument.
  ///
  /// This can be overridden by some of its subclasses.
  String? get fileSystemScheme =>
      argParser.options.containsKey(FlutterOptions.kFileSystemScheme)
          ? stringArg(FlutterOptions.kFileSystemScheme)
          : null;

  /// The values of the `--filesystem-root` argument.
  ///
  /// This can be overridden by some of its subclasses.
  List<String>? get fileSystemRoots =>
      argParser.options.containsKey(FlutterOptions.kFileSystemRoot)
          ? stringsArg(FlutterOptions.kFileSystemRoot)
          : null;

  void usesPubOption({bool hide = false}) {
    argParser.addFlag(
      'pub',
      defaultsTo: true,
      hide: hide,
      help: 'Whether to run "flutter pub get" before executing this command.',
    );
    _usesPubOption = true;
  }

  /// Adds flags for using a specific filesystem root and scheme.
  ///
  /// The `hide` argument indicates whether or not to hide these options when
  /// the user asks for help.
  void usesFilesystemOptions({required bool hide}) {
    argParser
      ..addOption(
        'output-dill',
        hide: hide,
        help: 'Specify the path to frontend server output kernel file.',
      )
      ..addMultiOption(
        FlutterOptions.kFileSystemRoot,
        hide: hide,
        help:
            'Specify the path that is used as the root of a virtual file system '
            'during compilation. The input file name should be specified as a URL '
            'using the scheme given in "--${FlutterOptions.kFileSystemScheme}".\n'
            'Requires the "--output-dill" option to be explicitly specified.',
      )
      ..addOption(
        FlutterOptions.kFileSystemScheme,
        defaultsTo: 'org-dartlang-root',
        hide: hide,
        help:
            'Specify the scheme that is used for virtual file system used in '
            'compilation. See also the "--${FlutterOptions.kFileSystemRoot}" option.',
      );
  }

  /// Adds options for connecting to the Dart VM Service port.
  void usesPortOptions({required bool verboseHelp}) {
    argParser.addOption(
      vmServicePortOption,
      help:
          '(deprecated; use host-vmservice-port instead) '
          'Listen to the given port for a Dart VM Service connection.\n'
          'Specifying port 0 (the default) will find a random free port.\n '
          'if the Dart Development Service (DDS) is enabled, this will not be the port '
          'of the VmService instance advertised on the command line.',
      hide: !verboseHelp,
    );
    argParser.addOption(
      observatoryPortOption,
      help:
          '(deprecated; use host-vmservice-port instead) '
          'Listen to the given port for a Dart VM Service connection.\n'
          'Specifying port 0 (the default) will find a random free port.\n '
          'if the Dart Development Service (DDS) is enabled, this will not be the port '
          'of the VmService instance advertised on the command line.',
      hide: !verboseHelp,
    );
    argParser.addOption(
      'device-vmservice-port',
      help:
          'Look for vmservice connections only from the specified port.\n'
          'Specifying port 0 (the default) will accept the first vmservice '
          'discovered.',
    );
    argParser.addOption(
      'host-vmservice-port',
      help:
          'When a device-side vmservice port is forwarded to a host-side '
          'port, use this value as the host port.\nSpecifying port 0 '
          '(the default) will find a random free host port.',
    );
    _usesPortOption = true;
  }

  /// Add option values for output directory of artifacts
  void usesOutputDir() {
    // TODO(eliasyishak): this feature has been added to [BuildWebCommand] and
    //  [BuildAarCommand]
    argParser.addOption(
      'output',
      abbr: 'o',
      aliases: <String>['output-dir'],
      help:
          'The absolute path to the directory where the repository is generated. '
          'By default, this is <current-directory>/build/<target-platform>.\n'
          'Currently supported for subcommands: aar, web.',
    );
  }

  void addDevToolsOptions({required bool verboseHelp}) {
    argParser.addFlag(
      kEnableDevTools,
      hide: !verboseHelp,
      defaultsTo: true,
      help:
          'Enable (or disable, with "--no-$kEnableDevTools") the launching of the '
          'Flutter DevTools debugger and profiler. '
          'If "--no-$kEnableDevTools" is specified, "--$kDevToolsServerAddress" is ignored.',
    );
    argParser.addOption(
      kDevToolsServerAddress,
      hide: !verboseHelp,
      help:
          'When this value is provided, the Flutter tool will not spin up a '
          'new DevTools server instance, and will instead use the one provided '
          'at the given address. Ignored if "--no-$kEnableDevTools" is specified.',
    );
  }

  void addDdsOptions({required bool verboseHelp}) {
    argParser.addOption(
      'dds-port',
      help:
          'When this value is provided, the Dart Development Service (DDS) will be '
          'bound to the provided port.\n'
          'Specifying port 0 (the default) will find a random free port.',
    );
    argParser.addFlag(
      'dds',
      defaultsTo: true,
      help:
          'Enable the Dart Developer Service (DDS).\n'
          'It may be necessary to disable this when attaching to an application with '
          'an existing DDS instance (e.g., attaching to an application currently '
          'connected to by "flutter run"), or when running certain tests.\n'
          'Disabling this feature may degrade IDE functionality if a DDS instance is '
          'not already connected to the target application.',
    );
    argParser.addFlag(
      'disable-dds',
      hide: !verboseHelp,
      help:
          '(deprecated; use "--no-dds" instead) '
          'Disable the Dart Developer Service (DDS).',
    );
  }

  void addServeObservatoryOptions({required bool verboseHelp}) {
    argParser.addFlag(
      'serve-observatory',
      hide: !verboseHelp,
      help: 'Serve the legacy Observatory developer tooling through the VM service.',
    );
  }

  late final bool enableDds = () {
    bool ddsEnabled = false;
    if (argResults?.wasParsed('disable-dds') ?? false) {
      if (argResults?.wasParsed('dds') ?? false) {
        throwToolExit(
          'The "--[no-]dds" and "--[no-]disable-dds" arguments are mutually exclusive. Only specify "--[no-]dds".',
        );
      }
      ddsEnabled = !boolArg('disable-dds');
      // TODO(ianh): enable the following code once google3 is migrated away from --disable-dds (and add test to flutter_command_test.dart)
      // ignore: dead_code, literal_only_boolean_expressions
      if (false) {
        if (ddsEnabled) {
          globals.printWarning(
            '${globals.logger.terminal.warningMark} The "--no-disable-dds" argument is deprecated and redundant, and should be omitted.',
          );
        } else {
          globals.printWarning(
            '${globals.logger.terminal.warningMark} The "--disable-dds" argument is deprecated. Use "--no-dds" instead.',
          );
        }
      }
    } else {
      ddsEnabled = boolArg('dds');
    }
    return ddsEnabled;
  }();

  bool get _hostVmServicePortProvided =>
      (argResults?.wasParsed(vmServicePortOption) ?? false) ||
      (argResults?.wasParsed(observatoryPortOption) ?? false) ||
      (argResults?.wasParsed('host-vmservice-port') ?? false);

  int _tryParseHostVmservicePort() {
    final String? vmServicePort =
        stringArg(vmServicePortOption) ?? stringArg(observatoryPortOption);
    final String? hostPort = stringArg('host-vmservice-port');
    if (vmServicePort == null && hostPort == null) {
      throwToolExit('Invalid port for `--vm-service-port/--host-vmservice-port`');
    }
    try {
      return int.parse((vmServicePort ?? hostPort)!);
    } on FormatException catch (error) {
      throwToolExit('Invalid port for `--vm-service-port/--host-vmservice-port`: $error');
    }
  }

  int get ddsPort {
    if (argResults?.wasParsed('dds-port') != true && _hostVmServicePortProvided) {
      // If an explicit DDS port is _not_ provided, use the host-vmservice-port for DDS.
      return _tryParseHostVmservicePort();
    } else if (argResults?.wasParsed('dds-port') ?? false) {
      // If an explicit DDS port is provided, use dds-port for DDS.
      return int.tryParse(stringArg('dds-port')!) ?? 0;
    }
    // Otherwise, DDS can bind to a random port.
    return 0;
  }

  Uri? get devToolsServerAddress {
    if (argResults?.wasParsed(kDevToolsServerAddress) ?? false) {
      final Uri? uri = Uri.tryParse(stringArg(kDevToolsServerAddress)!);
      if (uri != null && uri.host.isNotEmpty && uri.port != 0) {
        return uri;
      }
    }
    return null;
  }

  /// Gets the vmservice port provided to in the 'vm-service-port' or
  /// 'host-vmservice-port option.
  ///
  /// Only one of "host-vmservice-port" and "vm-service-port" may be
  /// specified.
  ///
  /// If no port is set, returns null.
  int? get hostVmservicePort {
    if (!_usesPortOption || !_hostVmServicePortProvided) {
      return null;
    }
    if ((argResults?.wasParsed(vmServicePortOption) ?? false) &&
        (argResults?.wasParsed(observatoryPortOption) ?? false) &&
        (argResults?.wasParsed('host-vmservice-port') ?? false)) {
      throwToolExit(
        'Only one of "--vm-service-port" and '
        '"--host-vmservice-port" may be specified.',
      );
    }
    // If DDS is enabled and no explicit DDS port is provided, use the
    // host-vmservice-port for DDS instead and bind the VM service to a random
    // port.
    if (enableDds && argResults?.wasParsed('dds-port') != true) {
      return null;
    }
    return _tryParseHostVmservicePort();
  }

  /// Gets the vmservice port provided to in the 'device-vmservice-port' option.
  ///
  /// If no port is set, returns null.
  int? get deviceVmservicePort {
    final String? devicePort = stringArg('device-vmservice-port');
    if (!_usesPortOption || devicePort == null) {
      return null;
    }
    try {
      return int.parse(devicePort);
    } on FormatException catch (error) {
      throwToolExit('Invalid port for `--device-vmservice-port`: $error');
    }
  }

  void addPublishPort({bool enabledByDefault = true, bool verboseHelp = false}) {
    argParser.addFlag(
      'publish-port',
      hide: !verboseHelp,
      help:
          'Publish the VM service port over mDNS. Disable to prevent the '
          'local network permission app dialog in debug and profile build modes (iOS devices only).',
      defaultsTo: enabledByDefault,
    );
  }

  Future<bool> get disablePortPublication async => !boolArg('publish-port');

  void usesIpv6Flag({required bool verboseHelp}) {
    argParser.addFlag(
      ipv6Flag,
      negatable: false,
      help:
          'Binds to IPv6 localhost instead of IPv4 when the flutter tool '
          'forwards the host port to a device port.',
      hide: !verboseHelp,
    );
    _usesIpv6Flag = true;
  }

  bool? get ipv6 => _usesIpv6Flag ? boolArg('ipv6') : null;

  void usesBuildNumberOption() {
    argParser.addOption(
      'build-number',
      help:
          'An identifier used as an internal version number.\n'
          'Each build must have a unique identifier to differentiate it from previous builds.\n'
          'It is used to determine whether one build is more recent than another, with higher numbers indicating more recent build.\n'
          'On Android it is used as "versionCode".\n'
          'On Xcode builds it is used as "CFBundleVersion".\n'
          'On Windows it is used as the build suffix for the product and file versions.',
    );
  }

  void usesBuildNameOption() {
    argParser.addOption(
      'build-name',
      help:
          'A "x.y.z" string used as the version number shown to users.\n'
          'For each new version of your app, you will provide a version number to differentiate it from previous versions.\n'
          'On Android it is used as "versionName".\n'
          'On Xcode builds it is used as "CFBundleShortVersionString".\n'
          'On Windows it is used as the major, minor, and patch parts of the product and file versions.',
      valueHelp: 'x.y.z',
    );
  }

  void usesDartDefineOption() {
    argParser.addMultiOption(
      FlutterOptions.kDartDefinesOption,
      aliases: <String>[kDartDefines], // supported for historical reasons
      help:
          'Additional key-value pairs that will be available as constants '
          'from the String.fromEnvironment, bool.fromEnvironment, and int.fromEnvironment '
          'constructors.\n'
          'Multiple defines can be passed by repeating "--${FlutterOptions.kDartDefinesOption}" multiple times.',
      valueHelp: 'foo=bar',
      splitCommas: false,
    );
    _usesDartDefineFromFileOption();
  }

  void _usesDartDefineFromFileOption() {
    argParser.addMultiOption(
      FlutterOptions.kDartDefineFromFileOption,
      help:
          'The path of a .json or .env file containing key-value pairs that will be available as environment variables.\n'
          'These can be accessed using the String.fromEnvironment, bool.fromEnvironment, and int.fromEnvironment constructors.\n'
          'Multiple defines can be passed by repeating "--${FlutterOptions.kDartDefineFromFileOption}" multiple times.\n'
          'Entries from "--${FlutterOptions.kDartDefinesOption}" with identical keys take precedence over entries from these files.',
      valueHelp: 'use-define-config.json|.env',
      splitCommas: false,
    );
  }

  void usesWebResourcesCdnFlag() {
    argParser.addFlag(
      FlutterOptions.kWebResourcesCdnFlag,
      defaultsTo: true,
      help: 'Use Web static resources hosted on a CDN.',
    );
  }

  void usesDeviceUserOption() {
    argParser.addOption(
      FlutterOptions.kDeviceUser,
      help:
          'Identifier number for a user or work profile on Android only. Run "adb shell pm list users" for available identifiers.',
      valueHelp: '10',
    );
  }

  void usesDeviceTimeoutOption() {
    argParser.addOption(
      FlutterOptions.kDeviceTimeout,
      help:
          'Time in seconds to wait for devices to attach. Longer timeouts may be necessary for networked devices.',
      valueHelp: '10',
    );
  }

  void usesDeviceConnectionOption() {
    argParser.addOption(
      FlutterOptions.kDeviceConnection,
      defaultsTo: 'both',
      help: 'Discover devices based on connection type.',
      allowed: <String>['attached', 'wireless', 'both'],
      allowedHelp: <String, String>{
        'both': 'Searches for both attached and wireless devices.',
        'attached':
            'Only searches for devices connected by USB or built-in (such as simulators/emulators, MacOS/Windows, Chrome)',
        'wireless':
            'Only searches for devices connected wirelessly. Discovering wireless devices may take longer.',
      },
    );
  }

  void usesApplicationBinaryOption() {
    argParser.addOption(
      FlutterOptions.kUseApplicationBinary,
      help:
          'Specify a pre-built application binary to use when running. For Android applications, '
          'this must be the path to an APK. For iOS applications, the path to an IPA. Other device types '
          'do not yet support prebuilt application binaries.',
      valueHelp: 'path/to/app.apk',
    );
  }

  /// Whether it is safe for this command to use a cached pub invocation.
  bool get cachePubGet => true;

  /// Whether this command should report null safety analytics.
  bool get reportNullSafety => false;

  late final Duration? deviceDiscoveryTimeout = () {
    if ((argResults?.options.contains(FlutterOptions.kDeviceTimeout) ?? false) &&
        (argResults?.wasParsed(FlutterOptions.kDeviceTimeout) ?? false)) {
      final int? timeoutSeconds = int.tryParse(stringArg(FlutterOptions.kDeviceTimeout)!);
      if (timeoutSeconds == null) {
        throwToolExit(
          'Could not parse "--${FlutterOptions.kDeviceTimeout}" argument. It must be an integer.',
        );
      }
      return Duration(seconds: timeoutSeconds);
    }
    return null;
  }();

  DeviceConnectionInterface? get deviceConnectionInterface {
    if ((argResults?.options.contains(FlutterOptions.kDeviceConnection) ?? false) &&
        (argResults?.wasParsed(FlutterOptions.kDeviceConnection) ?? false)) {
      return switch (stringArg(FlutterOptions.kDeviceConnection)) {
        'attached' => DeviceConnectionInterface.attached,
        'wireless' => DeviceConnectionInterface.wireless,
        _ => null,
      };
    }
    return null;
  }

  late final TargetDevices _targetDevices = TargetDevices(
    platform: globals.platform,
    deviceManager: globals.deviceManager!,
    logger: globals.logger,
    deviceConnectionInterface: deviceConnectionInterface,
  );

  void addBuildModeFlags({
    required bool verboseHelp,
    bool defaultToRelease = true,
    bool excludeDebug = false,
    bool excludeRelease = false,
  }) {
    // A release build must be the default if a debug build is not possible.
    assert(defaultToRelease || !excludeDebug);
    _excludeDebug = excludeDebug;
    _excludeRelease = excludeRelease;
    defaultBuildMode = defaultToRelease ? BuildMode.release : BuildMode.debug;

    if (!excludeDebug) {
      argParser.addFlag(
        'debug',
        negatable: false,
        help: 'Build a debug version of your app${defaultToRelease ? '' : ' (default mode)'}.',
      );
    }
    argParser.addFlag(
      'profile',
      negatable: false,
      help: 'Build a version of your app specialized for performance profiling.',
    );
    if (!excludeRelease) {
      argParser.addFlag(
        'release',
        negatable: false,
        help: 'Build a release version of your app${defaultToRelease ? ' (default mode)' : ''}.',
      );
      argParser.addFlag(
        'jit-release',
        negatable: false,
        hide: !verboseHelp,
        help:
            'Build a JIT release version of your app${defaultToRelease ? ' (default mode)' : ''}.',
      );
    }
  }

  void addSplitDebugInfoOption() {
    argParser.addOption(
      FlutterOptions.kSplitDebugInfoOption,
      help:
          'In a release build, this flag reduces application size by storing '
          'Dart program symbols in a separate file on the host rather than in the '
          'application. The value of the flag should be a directory where program '
          'symbol files can be stored for later use. These symbol files contain '
          'the information needed to symbolize Dart stack traces. For an app built '
          'with this flag, the "flutter symbolize" command with the right program '
          'symbol file is required to obtain a human readable stack trace.\n'
          'This flag cannot be combined with "--${FlutterOptions.kAnalyzeSize}".',
      valueHelp: 'v1.2.3/',
    );
  }

  void addDartObfuscationOption() {
    argParser.addFlag(
      FlutterOptions.kDartObfuscationOption,
      help:
          'In a release build, this flag removes identifiers and replaces them '
          'with randomized values for the purposes of source code obfuscation. This '
          'flag must always be combined with "--${FlutterOptions.kSplitDebugInfoOption}" option, the '
          'mapping between the values and the original identifiers is stored in the '
          'symbol map created in the specified directory. For an app built with this '
          'flag, the "flutter symbolize" command with the right program '
          'symbol file is required to obtain a human readable stack trace.\n'
          '\n'
          'Because all identifiers are renamed, methods like Object.runtimeType, '
          'Type.toString, Enum.toString, Stacktrace.toString, Symbol.toString '
          '(for constant symbols or those generated by runtime system) will '
          'return obfuscated results. Any code or tests that rely on exact names '
          'will break.',
    );
  }

  void addBundleSkSLPathOption({required bool hide}) {
    argParser.addOption(
      FlutterOptions.kBundleSkSLPathOption,
      help:
          'A path to a file containing precompiled SkSL shaders generated '
          'during "flutter run". These can be included in an application to '
          'improve the first frame render times.',
      hide: hide,
      valueHelp: 'flutter_1.sksl',
    );
  }

  void addTreeShakeIconsFlag({bool? enabledByDefault}) {
    argParser.addFlag(
      'tree-shake-icons',
      defaultsTo: enabledByDefault ?? kIconTreeShakerEnabledDefault,
      help: 'Tree shake icon fonts so that only glyphs used by the application remain.',
    );
  }

  void addShrinkingFlag({required bool verboseHelp}) {
    argParser.addFlag(
      'shrink',
      hide: !verboseHelp,
      help:
          'This flag has no effect. Code shrinking is always enabled in release builds. '
          'To learn more, see: https://developer.android.com/studio/build/shrink-code',
    );
  }

  void addNullSafetyModeOptions({required bool hide}) {
    argParser.addFlag(
      FlutterOptions.kNullSafety,
      help: 'This flag is deprecated as only null-safe code is supported.',
      defaultsTo: true,
      hide: true,
    );
    argParser.addFlag(
      FlutterOptions.kNullAssertions,
      help: 'This flag is deprecated as only null-safe code is supported.',
      hide: true,
    );
  }

  void usesFrontendServerStarterPathOption({required bool verboseHelp}) {
    argParser.addOption(
      FlutterOptions.kFrontendServerStarterPath,
      help:
          'When this value is provided, the frontend server will be started '
          'in JIT mode from the specified file, instead of from the AOT '
          'snapshot shipped with the Dart SDK. The specified file can either '
          'be a Dart source file, or an AppJIT snapshot. This option does '
          'not affect web builds.',
      hide: !verboseHelp,
    );
  }

  /// Enables support for the hidden options --extra-front-end-options and
  /// --extra-gen-snapshot-options.
  void usesExtraDartFlagOptions({required bool verboseHelp}) {
    argParser.addMultiOption(
      FlutterOptions.kExtraFrontEndOptions,
      aliases: <String>[kExtraFrontEndOptions], // supported for historical reasons
      help:
          'A comma-separated list of additional command line arguments that will be passed directly to the Dart front end. '
          'For example, "--${FlutterOptions.kExtraFrontEndOptions}=--enable-experiment=nonfunction-type-aliases".',
      valueHelp: '--foo,--bar',
      hide: !verboseHelp,
    );
    argParser.addMultiOption(
      FlutterOptions.kExtraGenSnapshotOptions,
      aliases: <String>[kExtraGenSnapshotOptions], // supported for historical reasons
      help:
          'A comma-separated list of additional command line arguments that will be passed directly to the Dart native compiler. '
          '(Only used in "--profile" or "--release" builds.) '
          'For example, "--${FlutterOptions.kExtraGenSnapshotOptions}=--no-strip".',
      valueHelp: '--foo,--bar',
      hide: !verboseHelp,
    );
  }

  void usesFuchsiaOptions({bool hide = false}) {
    argParser.addOption(
      'target-model',
      help: 'Target model that determines what core libraries are available.',
      defaultsTo: 'flutter',
      hide: hide,
      allowed: const <String>['flutter', 'flutter_runner'],
    );
    argParser.addOption(
      'module',
      abbr: 'm',
      hide: hide,
      help: 'The name of the module (required if attaching to a fuchsia device).',
      valueHelp: 'module-name',
    );
  }

  void addEnableExperimentation({required bool hide}) {
    argParser.addMultiOption(
      FlutterOptions.kEnableExperiment,
      help:
          'The name of an experimental Dart feature to enable. For more information see: '
          'https://github.com/dart-lang/sdk/blob/main/docs/process/experimental-flags.md',
      hide: hide,
    );
  }

  void addBuildPerformanceFile({bool hide = false}) {
    argParser.addOption(
      FlutterOptions.kPerformanceMeasurementFile,
      help:
          'The name of a file where flutter assemble performance and '
          'cached-ness information will be written in a JSON format.',
      hide: hide,
    );
  }

  void addAndroidSpecificBuildOptions({bool hide = false}) {
    argParser.addFlag(
      FlutterOptions.kAndroidGradleDaemon,
      help:
          'Whether to enable the Gradle daemon when performing an Android build. '
          'Starting the daemon is the default behavior of the gradle wrapper script created '
          'in a Flutter project. Setting this flag to false corresponds to passing '
          '"--no-daemon" to the gradle wrapper script. This flag will cause the daemon '
          'process to terminate after the build is completed.',
      defaultsTo: true,
      hide: hide,
    );
    argParser.addFlag(
      FlutterOptions.kAndroidSkipBuildDependencyValidation,
      help:
          'Whether to skip version checking for Java, Gradle, '
          'the Android Gradle Plugin (AGP), and the Kotlin Gradle Plugin (KGP)'
          ' during Android builds.',
    );
    argParser.addMultiOption(
      FlutterOptions.kAndroidProjectArgs,
      help:
          'Additional arguments specified as key=value that are passed directly to the gradle '
          'project via the -P flag. These can be accessed in build.gradle via the "project.property" API.',
      splitCommas: false,
      abbr: 'P',
    );
  }

  void addNativeNullAssertions({bool hide = false}) {
    argParser.addFlag(
      'native-null-assertions',
      defaultsTo: true,
      hide: hide,
      help:
          'Enables additional runtime null checks in web applications to ensure '
          'the correct nullability of native (such as in dart:html) and external '
          '(such as with JS interop) types. This is enabled by default but only takes '
          'effect in sound mode. To report an issue with a null assertion failure in '
          'dart:html or the other dart web libraries, please file a bug at: '
          'https://github.com/dart-lang/sdk/issues/labels/web-libraries',
    );
  }

  void usesInitializeFromDillOption({required bool hide}) {
    argParser.addOption(
      FlutterOptions.kInitializeFromDill,
      help:
          'Initializes the resident compiler with a specific kernel file instead of '
          'the default cached location.',
      hide: hide,
    );
    argParser.addFlag(
      FlutterOptions.kAssumeInitializeFromDillUpToDate,
      help:
          'If set, assumes that the file passed in initialize-from-dill is up '
          'to date and skip the check and potential invalidation of files.',
      hide: hide,
    );
  }

  void usesNativeAssetsOption({required bool hide}) {
    argParser.addOption(
      FlutterOptions.kNativeAssetsYamlFile,
      help:
          'Initializes the resident compiler with a custom native assets '
          'yaml file instead of the default cached location.',
      hide: hide,
    );
  }

  void addIgnoreDeprecationOption({bool hide = false}) {
    argParser.addFlag(
      'ignore-deprecation',
      negatable: false,
      help:
          'Indicates that the app should ignore deprecation warnings and continue to build '
          'using deprecated APIs. Use of this flag may cause your app to fail to build when '
          'deprecated APIs are removed.',
    );
  }

  /// Adds build options common to all of the desktop build commands.
  void addCommonDesktopBuildOptions({required bool verboseHelp}) {
    addBuildModeFlags(verboseHelp: verboseHelp);
    addBuildPerformanceFile(hide: !verboseHelp);
    addBundleSkSLPathOption(hide: !verboseHelp);
    addDartObfuscationOption();
    addEnableExperimentation(hide: !verboseHelp);
    addNullSafetyModeOptions(hide: !verboseHelp);
    addSplitDebugInfoOption();
    addTreeShakeIconsFlag();
    usesAnalyzeSizeFlag();
    usesDartDefineOption();
    usesExtraDartFlagOptions(verboseHelp: verboseHelp);
    usesPubOption();
    usesTargetOption();
    usesTrackWidgetCreation(verboseHelp: verboseHelp);
    usesBuildNumberOption();
    usesBuildNameOption();
  }

  /// The build mode that this command will use if no build mode is
  /// explicitly specified.
  ///
  /// Use [getBuildMode] to obtain the actual effective build mode.
  BuildMode defaultBuildMode = BuildMode.debug;

  BuildMode getBuildMode() {
    // No debug when _excludeDebug is true. If debug is not excluded, then take
    // the command line flag (if such exists for this command).
    bool argIfDefined(String flagName, bool ifNotDefined) {
      return argParser.options.containsKey(flagName) ? boolArg(flagName) : ifNotDefined;
    }

    final bool debugResult = !_excludeDebug && argIfDefined('debug', false);
    final bool jitReleaseResult = !_excludeRelease && argIfDefined('jit-release', false);
    final bool releaseResult = !_excludeRelease && argIfDefined('release', false);
    final bool profileResult = argIfDefined('profile', false);
    final List<bool> modeFlags = <bool>[
      debugResult,
      profileResult,
      jitReleaseResult,
      releaseResult,
    ];
    if (modeFlags.where((bool flag) => flag).length > 1) {
      throw UsageException(
        'Only one of "--debug", "--profile", "--jit-release", '
            'or "--release" can be specified.',
        '',
      );
    }
    if (debugResult) {
      return BuildMode.debug;
    }
    if (profileResult) {
      return BuildMode.profile;
    }
    if (releaseResult) {
      return BuildMode.release;
    }
    if (jitReleaseResult) {
      return BuildMode.jitRelease;
    }
    return defaultBuildMode;
  }

  void usesFlavorOption() {
    argParser.addOption(
      'flavor',
      help:
          'Build a custom app flavor as defined by platform-specific build setup.\n'
          'Supports the use of product flavors in Android Gradle scripts, and '
          'the use of custom Xcode schemes.\n'
          'Overrides the value of the "default-flavor" entry in the flutter pubspec.',
    );
  }

  void usesTrackWidgetCreation({bool hasEffect = true, required bool verboseHelp}) {
    argParser.addFlag(
      'track-widget-creation',
      hide: !hasEffect && !verboseHelp,
      defaultsTo: true,
      help:
          'Track widget creation locations. This enables features such as the widget inspector. '
          'This parameter is only functional in debug mode (i.e. when compiling JIT, not AOT).',
    );
  }

  void usesAnalyzeSizeFlag() {
    argParser.addFlag(
      FlutterOptions.kAnalyzeSize,
      help:
          'Whether to produce additional profile information for artifact output size. '
          'This flag is only supported on "--release" builds. When building for Android, a single '
          'ABI must be specified at a time with the "--target-platform" flag. When building for iOS, '
          'only the symbols from the arm64 architecture are used to analyze code size.\n'
          'By default, the intermediate output files will be placed in a transient directory in the '
          'build directory. This can be overridden with the "--${FlutterOptions.kCodeSizeDirectory}" option.\n'
          'This flag cannot be combined with "--${FlutterOptions.kSplitDebugInfoOption}".',
    );

    argParser.addOption(
      FlutterOptions.kCodeSizeDirectory,
      help:
          'The location to write code size analysis files. If this is not specified, files '
          'are written to a temporary directory under the build directory.',
    );
  }

  void addEnableImpellerFlag({required bool verboseHelp}) {
    argParser.addFlag(
      'enable-impeller',
      hide: !verboseHelp,
      defaultsTo: null,
      help:
          'Whether to enable the Impeller rendering engine. '
          'Impeller is the default renderer on iOS. On Android, Impeller '
          'is available but not the default. This flag will cause Impeller '
          'to be used on Android. On other platforms, this flag will be '
          'ignored.',
    );
  }

  void addEnableVulkanValidationFlag({required bool verboseHelp}) {
    argParser.addFlag(
      'enable-vulkan-validation',
      hide: !verboseHelp,
      help:
          'Enable vulkan validation on the Impeller rendering backend if '
          'Vulkan is in use and the validation layers are available to the '
          'application.',
    );
  }

  void addEnableEmbedderApiFlag({required bool verboseHelp}) {
    argParser.addFlag(
      'enable-embedder-api',
      hide: !verboseHelp,
      help: 'Whether to enable the experimental embedder API on iOS.',
    );
  }

  /// Returns a [FlutterProject] view of the current directory or a ToolExit error,
  /// if `pubspec.yaml` or `example/pubspec.yaml` is invalid.
  FlutterProject get project => FlutterProject.current();

  /// The path to the package config for the current project.
  ///
  /// If an explicit argument is given, that is returned. Otherwise the file
  /// system is searched for the package config. For projects in pub workspaces
  /// the package config might be located in a parent directory.
  ///
  /// If none is found `.dart_tool/package_config.json` is returned.
  String packageConfigPath() {
    final String? packagesPath = this.packagesPath;
    return packagesPath ?? findPackageConfigFileOrDefault(project.directory).path;
  }

  /// Compute the [BuildInfo] for the current flutter command.
  ///
  /// Commands that build multiple build modes can pass in a [forcedBuildMode]
  /// to be used instead of parsing flags.
  ///
  /// Throws a [ToolExit] if the current set of options is not compatible with
  /// each other.
  Future<BuildInfo> getBuildInfo({
    BuildMode? forcedBuildMode,
    File? forcedTargetFile,
    bool? forcedUseLocalCanvasKit,
  }) async {
    final bool trackWidgetCreation =
        argParser.options.containsKey('track-widget-creation') && boolArg('track-widget-creation');

    final String? buildNumber =
        argParser.options.containsKey('build-number') ? stringArg('build-number') : null;

    final File packageConfigFile = globals.fs.file(packageConfigPath());

    final PackageConfig packageConfig = await loadPackageConfigWithLogging(
      packageConfigFile,
      logger: globals.logger,
      throwOnError: false,
    );

    final List<String> experiments =
        argParser.options.containsKey(FlutterOptions.kEnableExperiment)
            ? stringsArg(FlutterOptions.kEnableExperiment).toList()
            : <String>[];
    final List<String> extraGenSnapshotOptions =
        argParser.options.containsKey(FlutterOptions.kExtraGenSnapshotOptions)
            ? stringsArg(FlutterOptions.kExtraGenSnapshotOptions).toList()
            : <String>[];
    final List<String> extraFrontEndOptions =
        argParser.options.containsKey(FlutterOptions.kExtraFrontEndOptions)
            ? stringsArg(FlutterOptions.kExtraFrontEndOptions).toList()
            : <String>[];

    if (experiments.isNotEmpty) {
      for (final String expFlag in experiments) {
        final String flag = '--enable-experiment=$expFlag';
        extraFrontEndOptions.add(flag);
        extraGenSnapshotOptions.add(flag);
      }
    }

    String? codeSizeDirectory;
    if (argParser.options.containsKey(FlutterOptions.kAnalyzeSize) &&
        boolArg(FlutterOptions.kAnalyzeSize)) {
      Directory directory = globals.fsUtils.getUniqueDirectory(
        globals.fs.directory(getBuildDirectory()),
        'flutter_size',
      );
      if (argParser.options.containsKey(FlutterOptions.kCodeSizeDirectory) &&
          stringArg(FlutterOptions.kCodeSizeDirectory) != null) {
        directory = globals.fs.directory(stringArg(FlutterOptions.kCodeSizeDirectory));
      }
      directory.createSync(recursive: true);
      codeSizeDirectory = directory.path;
    }

    NullSafetyMode nullSafetyMode = NullSafetyMode.sound;
    if (argParser.options.containsKey(FlutterOptions.kNullSafety)) {
      final bool wasNullSafetyFlagParsed =
          argResults?.wasParsed(FlutterOptions.kNullSafety) ?? false;
      // Extra frontend options are only provided if explicitly
      // requested.
      if (wasNullSafetyFlagParsed) {
        if (boolArg(FlutterOptions.kNullSafety)) {
          nullSafetyMode = NullSafetyMode.sound;
          extraFrontEndOptions.add('--sound-null-safety');
        } else {
          nullSafetyMode = NullSafetyMode.unsound;
          extraFrontEndOptions.add('--no-sound-null-safety');
        }
      }
    }

    final bool dartObfuscation =
        argParser.options.containsKey(FlutterOptions.kDartObfuscationOption) &&
        boolArg(FlutterOptions.kDartObfuscationOption);

    final String? splitDebugInfoPath =
        argParser.options.containsKey(FlutterOptions.kSplitDebugInfoOption)
            ? stringArg(FlutterOptions.kSplitDebugInfoOption)
            : null;

    final bool androidGradleDaemon =
        !argParser.options.containsKey(FlutterOptions.kAndroidGradleDaemon) ||
        boolArg(FlutterOptions.kAndroidGradleDaemon);

    final bool androidSkipBuildDependencyValidation =
        !argParser.options.containsKey(FlutterOptions.kAndroidSkipBuildDependencyValidation) ||
        boolArg(FlutterOptions.kAndroidSkipBuildDependencyValidation);

    final List<String> androidProjectArgs =
        argParser.options.containsKey(FlutterOptions.kAndroidProjectArgs)
            ? stringsArg(FlutterOptions.kAndroidProjectArgs)
            : <String>[];

    if (dartObfuscation && (splitDebugInfoPath == null || splitDebugInfoPath.isEmpty)) {
      throwToolExit(
        '"--${FlutterOptions.kDartObfuscationOption}" can only be used in '
        'combination with "--${FlutterOptions.kSplitDebugInfoOption}"',
      );
    }
    final BuildMode buildMode = forcedBuildMode ?? getBuildMode();
    if (buildMode != BuildMode.release && codeSizeDirectory != null) {
      throwToolExit('"--${FlutterOptions.kAnalyzeSize}" can only be used on release builds.');
    }
    if (codeSizeDirectory != null && splitDebugInfoPath != null) {
      throwToolExit(
        '"--${FlutterOptions.kAnalyzeSize}" cannot be combined with "--${FlutterOptions.kSplitDebugInfoOption}".',
      );
    }

    final bool treeShakeIcons =
        argParser.options.containsKey('tree-shake-icons') &&
        buildMode.isPrecompiled &&
        boolArg('tree-shake-icons');

    final String? bundleSkSLPath =
        argParser.options.containsKey(FlutterOptions.kBundleSkSLPathOption)
            ? stringArg(FlutterOptions.kBundleSkSLPathOption)
            : null;

    if (bundleSkSLPath != null && !globals.fs.isFileSync(bundleSkSLPath)) {
      throwToolExit('No SkSL shader bundle found at $bundleSkSLPath.');
    }

    final String? performanceMeasurementFile =
        argParser.options.containsKey(FlutterOptions.kPerformanceMeasurementFile)
            ? stringArg(FlutterOptions.kPerformanceMeasurementFile)
            : null;

    final Map<String, Object?> defineConfigJsonMap = extractDartDefineConfigJsonMap();
    final List<String> dartDefines = extractDartDefines(defineConfigJsonMap: defineConfigJsonMap);

    final bool useCdn =
        !argParser.options.containsKey(FlutterOptions.kWebResourcesCdnFlag) ||
        boolArg(FlutterOptions.kWebResourcesCdnFlag);
    bool useLocalWebSdk = false;
    if (globalResults?.wasParsed(FlutterGlobalOptions.kLocalWebSDKOption) ?? false) {
      useLocalWebSdk = stringArg(FlutterGlobalOptions.kLocalWebSDKOption, global: true) != null;
    }
    final bool useLocalCanvasKit = forcedUseLocalCanvasKit ?? (!useCdn || useLocalWebSdk);

    final String? defaultFlavor = project.manifest.defaultFlavor;
    final String? cliFlavor = argParser.options.containsKey('flavor') ? stringArg('flavor') : null;
    final String? flavor = cliFlavor ?? defaultFlavor;
    if (flavor != null) {
      if (globals.platform.environment['FLUTTER_APP_FLAVOR'] != null) {
        throwToolExit(
          'FLUTTER_APP_FLAVOR is used by the framework and cannot be set in the environment.',
        );
      }
      if (dartDefines.any((String define) => define.startsWith('FLUTTER_APP_FLAVOR'))) {
        throwToolExit(
          'FLUTTER_APP_FLAVOR is used by the framework and cannot be '
          'set using --${FlutterOptions.kDartDefinesOption} or --${FlutterOptions.kDartDefineFromFileOption}',
        );
      }
      dartDefines.add('FLUTTER_APP_FLAVOR=$flavor');
    }

    return BuildInfo(
      buildMode,
      flavor,
      trackWidgetCreation: trackWidgetCreation,
      frontendServerStarterPath:
          argParser.options.containsKey(FlutterOptions.kFrontendServerStarterPath)
              ? stringArg(FlutterOptions.kFrontendServerStarterPath)
              : null,
      extraFrontEndOptions: extraFrontEndOptions.isNotEmpty ? extraFrontEndOptions : null,
      extraGenSnapshotOptions: extraGenSnapshotOptions.isNotEmpty ? extraGenSnapshotOptions : null,
      fileSystemRoots: fileSystemRoots,
      fileSystemScheme: fileSystemScheme,
      buildNumber: buildNumber,
      buildName: argParser.options.containsKey('build-name') ? stringArg('build-name') : null,
      treeShakeIcons: treeShakeIcons,
      splitDebugInfoPath: splitDebugInfoPath,
      dartObfuscation: dartObfuscation,
      dartDefines: dartDefines,
      bundleSkSLPath: bundleSkSLPath,
      dartExperiments: experiments,
      performanceMeasurementFile: performanceMeasurementFile,
      packageConfigPath: packagesPath ?? packageConfigFile.path,
      nullSafetyMode: nullSafetyMode,
      codeSizeDirectory: codeSizeDirectory,
      androidGradleDaemon: androidGradleDaemon,
      androidSkipBuildDependencyValidation: androidSkipBuildDependencyValidation,
      packageConfig: packageConfig,
      androidProjectArgs: androidProjectArgs,
      initializeFromDill:
          argParser.options.containsKey(FlutterOptions.kInitializeFromDill)
              ? stringArg(FlutterOptions.kInitializeFromDill)
              : null,
      assumeInitializeFromDillUpToDate:
          argParser.options.containsKey(FlutterOptions.kAssumeInitializeFromDillUpToDate) &&
          boolArg(FlutterOptions.kAssumeInitializeFromDillUpToDate),
      useLocalCanvasKit: useLocalCanvasKit,
    );
  }

  void setupApplicationPackages() {
    applicationPackages ??= ApplicationPackageFactory.instance;
  }

  /// The path to send to Google Analytics. Return null here to disable
  /// tracking of the command.
  Future<String?> get usagePath async {
    if (parent is FlutterCommand) {
      final FlutterCommand? commandParent = parent as FlutterCommand?;
      final String? path = await commandParent?.usagePath;
      // Don't report for parents that return null for usagePath.
      return path == null ? null : '$path/$name';
    } else {
      return name;
    }
  }

  /// Additional usage values to be sent with the usage ping.
  Future<CustomDimensions> get usageValues async => const CustomDimensions();

  /// Additional usage values to be sent with the usage ping for
  /// package:unified_analytics.
  ///
  /// Implementations of [FlutterCommand] can override this getter in order
  /// to add additional parameters in the [Event.commandUsageValues] constructor.
  Future<Event> unifiedAnalyticsUsageValues(String commandPath) async =>
      Event.commandUsageValues(workflow: commandPath, commandHasTerminal: hasTerminal);

  /// Runs this command.
  ///
  /// Rather than overriding this method, subclasses should override
  /// [verifyThenRunCommand] to perform any verification
  /// and [runCommand] to execute the command
  /// so that this method can record and report the overall time to analytics.
  @override
  Future<void> run() {
    final DateTime startTime = globals.systemClock.now();

    return context.run<void>(
      name: 'command',
      overrides: <Type, Generator>{FlutterCommand: () => this},
      body: () async {
        if (_usesFatalWarnings) {
          globals.logger.fatalWarnings = boolArg(FlutterOptions.kFatalWarnings);
        }
        // Prints the welcome message if needed.
        globals.flutterUsage.printWelcome();
        _printDeprecationWarning();
        final String? commandPath = await usagePath;
        if (commandPath != null) {
          _registerSignalHandlers(commandPath, startTime);
        }
        FlutterCommandResult commandResult = FlutterCommandResult.fail();
        try {
          commandResult = await verifyThenRunCommand(commandPath);
        } finally {
          final DateTime endTime = globals.systemClock.now();
          globals.printTrace(
            globals.userMessages.flutterElapsedTime(
              name,
              getElapsedAsMilliseconds(endTime.difference(startTime)),
            ),
          );
          if (commandPath != null) {
            _sendPostUsage(commandPath, commandResult, startTime, endTime);
          }
          if (_usesFatalWarnings) {
            globals.logger.checkForFatalLogs();
          }
        }
      },
    );
  }

  @visibleForOverriding
  String get deprecationWarning {
    return '${globals.logger.terminal.warningMark} The "$name" command is '
        'deprecated and will be removed in a future version of Flutter. '
        'See https://flutter.dev/to/previous-releases '
        'for previous releases of Flutter.\n';
  }

  void _printDeprecationWarning() {
    if (deprecated) {
      globals.printWarning(deprecationWarning);
    }
  }

  List<String> extractDartDefines({required Map<String, Object?> defineConfigJsonMap}) {
    final List<String> dartDefines = <String>[];

    defineConfigJsonMap.forEach((String key, Object? value) {
      dartDefines.add('$key=$value');
    });

    if (argParser.options.containsKey(FlutterOptions.kDartDefinesOption)) {
      dartDefines.addAll(stringsArg(FlutterOptions.kDartDefinesOption));
    }

    return dartDefines;
  }

  Map<String, Object?> extractDartDefineConfigJsonMap() {
    final Map<String, Object?> dartDefineConfigJsonMap = <String, Object?>{};

    if (argParser.options.containsKey(FlutterOptions.kDartDefineFromFileOption)) {
      final List<String> configFilePaths = stringsArg(FlutterOptions.kDartDefineFromFileOption);

      for (final String path in configFilePaths) {
        if (!globals.fs.isFileSync(path)) {
          throwToolExit(
            'Did not find the file passed to "--${FlutterOptions.kDartDefineFromFileOption}". Path: $path',
          );
        }

        final String configRaw = globals.fs.file(path).readAsStringSync();

        // Determine whether the file content is JSON or .env format.
        String configJsonRaw;
        if (configRaw.trim().startsWith('{')) {
          configJsonRaw = configRaw;
        } else {
          // Convert env file to JSON.
          configJsonRaw = convertEnvFileToJsonRaw(configRaw);
        }

        try {
          // Fix json convert Object value :type '_InternalLinkedHashMap<String, dynamic>' is not a subtype of type 'Map<String, Object>' in type cast
          (json.decode(configJsonRaw) as Map<String, dynamic>).forEach((String key, Object? value) {
            dartDefineConfigJsonMap[key] = value;
          });
        } on FormatException catch (err) {
          throwToolExit(
            'Unable to parse the file at path "$path" due to a formatting error. '
            'Ensure that the file contains valid JSON.\n'
            'Error details: $err',
          );
        }
      }
    }

    return dartDefineConfigJsonMap;
  }

  /// Parse a property line from an env file.
  /// Supposed property structure should be:
  ///   key=value
  ///
  /// Where: key is a string without spaces and value is a string.
  /// Value can also contain '=' char.
  ///
  /// Returns a record of key and value as strings.
  MapEntry<String, String> _parseProperty(String line) {
    if (DotEnvRegex.multiLineBlock.hasMatch(line)) {
      throwToolExit('Multi-line value is not supported: $line');
    }

    final Match? keyValueMatch = DotEnvRegex.keyValue.firstMatch(line);
    if (keyValueMatch == null) {
      throwToolExit(
        'Unable to parse file provided for '
        '--${FlutterOptions.kDartDefineFromFileOption}.\n'
        'Invalid property line: $line',
      );
    }

    final String key = keyValueMatch.group(1)!;
    final String value = keyValueMatch.group(2) ?? '';

    // Remove wrapping quotes and trailing line comment.
    final Match? doubleQuotedValueMatch = DotEnvRegex.doubleQuotedValue.firstMatch(value);
    if (doubleQuotedValueMatch != null) {
      return MapEntry<String, String>(key, doubleQuotedValueMatch.group(1)!);
    }

    final Match? singleQuotedValueMatch = DotEnvRegex.singleQuotedValue.firstMatch(value);
    if (singleQuotedValueMatch != null) {
      return MapEntry<String, String>(key, singleQuotedValueMatch.group(1)!);
    }

    final Match? backQuotedValueMatch = DotEnvRegex.backQuotedValue.firstMatch(value);
    if (backQuotedValueMatch != null) {
      return MapEntry<String, String>(key, backQuotedValueMatch.group(1)!);
    }

    final Match? unquotedValueMatch = DotEnvRegex.unquotedValue.firstMatch(value);
    if (unquotedValueMatch != null) {
      return MapEntry<String, String>(key, unquotedValueMatch.group(1)!);
    }

    return MapEntry<String, String>(key, value);
  }

  /// Converts an .env file string to its equivalent JSON string.
  ///
  /// For example, the .env file string
  ///   key=value # comment
  ///   complexKey="foo#bar=baz"
  /// would be converted to a JSON string equivalent to:
  ///   {
  ///     "key": "value",
  ///     "complexKey": "foo#bar=baz"
  ///   }
  ///
  /// Multiline values are not supported.
  String convertEnvFileToJsonRaw(String configRaw) {
    final List<String> lines =
        configRaw
            .split('\n')
            .map((String line) => line.trim())
            .where((String line) => line.isNotEmpty)
            .where((String line) => !line.startsWith('#')) // Remove comment lines.
            .toList();

    final Map<String, String> propertyMap = <String, String>{};
    for (final String line in lines) {
      final MapEntry<String, String> property = _parseProperty(line);
      propertyMap[property.key] = property.value;
    }

    return jsonEncode(propertyMap);
  }

  Map<String, String> extractWebHeaders() {
    final Map<String, String> webHeaders = <String, String>{};

    if (argParser.options.containsKey('web-header')) {
      final List<String> candidates = stringsArg('web-header');
      final List<String> invalidHeaders = <String>[];
      for (final String candidate in candidates) {
        final Match? keyValueMatch = _HttpRegex.httpHeader.firstMatch(candidate);
        if (keyValueMatch == null) {
          invalidHeaders.add(candidate);
          continue;
        }

        webHeaders[keyValueMatch.group(1)!] = keyValueMatch.group(2)!;
      }

      if (invalidHeaders.isNotEmpty) {
        throwToolExit('Invalid web headers: ${invalidHeaders.join(', ')}');
      }
    }

    return webHeaders;
  }

  void _registerSignalHandlers(String commandPath, DateTime startTime) {
    void handler(io.ProcessSignal s) {
      globals.cache.releaseLock();
      _sendPostUsage(
        commandPath,
        const FlutterCommandResult(ExitStatus.killed),
        startTime,
        globals.systemClock.now(),
      );
    }

    globals.signals.addHandler(io.ProcessSignal.sigterm, handler);
    globals.signals.addHandler(io.ProcessSignal.sigint, handler);
  }

  /// Logs data about this command.
  ///
  /// For example, the command path (e.g. `build/apk`) and the result,
  /// as well as the time spent running it.
  void _sendPostUsage(
    String commandPath,
    FlutterCommandResult commandResult,
    DateTime startTime,
    DateTime endTime,
  ) {
    // Send command result.
    final int? maxRss = getMaxRss(processInfo);
    CommandResultEvent(commandPath, commandResult.toString(), maxRss).send();
    analytics.send(
      Event.flutterCommandResult(
        commandPath: commandPath,
        result: commandResult.toString(),
        maxRss: maxRss,
        commandHasTerminal: hasTerminal,
      ),
    );

    // Send timing.
    final List<String?> labels = <String?>[
      commandResult.exitStatus.name,
      if (commandResult.timingLabelParts?.isNotEmpty ?? false) ...?commandResult.timingLabelParts,
    ];

    final String label = labels
        .where((String? label) => label != null && !_isBlank(label))
        .join('-');

    // If the command provides its own end time, use it. Otherwise report
    // the duration of the entire execution.
    final Duration elapsedDuration = (commandResult.endTimeOverride ?? endTime).difference(
      startTime,
    );
    globals.flutterUsage.sendTiming(
      'flutter',
      name,
      elapsedDuration,
      // Report in the form of `success-[parameter1-parameter2]`, all of which
      // can be null if the command doesn't provide a FlutterCommandResult.
      label: label == '' ? null : label,
    );
    analytics.send(
      Event.timing(
        workflow: 'flutter',
        variableName: name,
        elapsedMilliseconds: elapsedDuration.inMilliseconds,
        // Report in the form of `success-[parameter1-parameter2]`, all of which
        // can be null if the command doesn't provide a FlutterCommandResult.
        label: label == '' ? null : label,
      ),
    );
  }

  /// Perform validation then call [runCommand] to execute the command.
  /// Return a [Future] that completes with an exit code
  /// indicating whether execution was successful.
  ///
  /// Subclasses should override this method to perform verification
  /// then call this method to execute the command
  /// rather than calling [runCommand] directly.
  @mustCallSuper
  Future<FlutterCommandResult> verifyThenRunCommand(String? commandPath) async {
    if (argParser.options.containsKey(FlutterOptions.kNullSafety) &&
        argResults![FlutterOptions.kNullSafety] == false &&
        globals.nonNullSafeBuilds == NonNullSafeBuilds.notAllowed) {
      throwToolExit('''
Could not find an option named "no-${FlutterOptions.kNullSafety}".

Run 'flutter -h' (or 'flutter <command> -h') for available flutter commands and options.
''');
    }

    globals.preRunValidator.validate();

    if (refreshWirelessDevices) {
      // Loading wireless devices takes longer so start it early.
      _targetDevices.startExtendedWirelessDeviceDiscovery(
        deviceDiscoveryTimeout: deviceDiscoveryTimeout,
      );
    }

    // Populate the cache. We call this before pub get below so that the
    // sky_engine package is available in the flutter cache for pub to find.
    if (shouldUpdateCache) {
      // First always update universal artifacts, as some of these (e.g.
      // ios-deploy on macOS) are required to determine `requiredArtifacts`.
      final bool offline;
      if (argParser.options.containsKey('offline')) {
        offline = boolArg('offline');
      } else {
        offline = false;
      }
      await globals.cache.updateAll(<DevelopmentArtifact>{
        DevelopmentArtifact.universal,
      }, offline: offline);
      await globals.cache.updateAll(await requiredArtifacts, offline: offline);
    }
    globals.cache.releaseLock();

    await validateCommand();

    final FlutterProject project = FlutterProject.current();
    project.checkForDeprecation(deprecationBehavior: deprecationBehavior);

    if (shouldRunPub) {
      final Environment environment = Environment(
        artifacts: globals.artifacts!,
        logger: globals.logger,
        cacheDir: globals.cache.getRoot(),
        engineVersion: globals.flutterVersion.engineRevision,
        fileSystem: globals.fs,
        flutterRootDir: globals.fs.directory(Cache.flutterRoot),
        outputDir: globals.fs.directory(getBuildDirectory()),
        processManager: globals.processManager,
        platform: globals.platform,
        usage: globals.flutterUsage,
        analytics: analytics,
        projectDir: project.directory,
        packageConfigPath: packageConfigPath(),
        generateDartPluginRegistry: true,
      );

      await pub.get(
        context: PubContext.getVerifyContext(name),
        project: project,
        checkUpToDate: cachePubGet,
      );

      await generateLocalizationsSyntheticPackage(
        environment: environment,
        buildSystem: globals.buildSystem,
        buildTargets: globals.buildTargets,
      );

      // null implicitly means all plugins are allowed
      List<String>? allowedPlugins;
      if (stringArg(FlutterGlobalOptions.kDeviceIdOption, global: true) == 'preview') {
        // The preview device does not currently support any plugins.
        allowedPlugins = PreviewDevice.supportedPubPlugins;
      }
      await project.regeneratePlatformSpecificTooling(allowedPlugins: allowedPlugins);
      if (reportNullSafety) {
        await _sendNullSafetyAnalyticsEvents(project);
      }
    }

    setupApplicationPackages();

    if (commandPath != null) {
      // Until the GA4 migration is complete, we will continue to send to the GA3 instance
      // as well as GA4. Once migration is complete, we will only make a call for GA4 values
      final List<Object> pairOfUsageValues = await Future.wait<Object>(<Future<Object>>[
        usageValues,
        unifiedAnalyticsUsageValues(commandPath),
      ]);

      Usage.command(
        commandPath,
        parameters: CustomDimensions(
          commandHasTerminal: hasTerminal,
        ).merge(pairOfUsageValues[0] as CustomDimensions),
      );
      analytics.send(pairOfUsageValues[1] as Event);
    }

    return runCommand();
  }

  Future<void> _sendNullSafetyAnalyticsEvents(FlutterProject project) async {
    final BuildInfo buildInfo = await getBuildInfo();
    NullSafetyAnalysisEvent(
      buildInfo.packageConfig,
      buildInfo.nullSafetyMode,
      project.manifest.appName,
      globals.flutterUsage,
    ).send();
  }

  /// The set of development artifacts required for this command.
  ///
  /// Defaults to an empty set. Including [DevelopmentArtifact.universal] is
  /// not required as it is always updated.
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async => const <DevelopmentArtifact>{};

  /// Subclasses must implement this to execute the command.
  /// Optionally provide a [FlutterCommandResult] to send more details about the
  /// execution for analytics.
  Future<FlutterCommandResult> runCommand();

  /// Find and return all target [Device]s based upon currently connected
  /// devices and criteria entered by the user on the command line.
  /// If no device can be found that meets specified criteria,
  /// then print an error message and return null.
  Future<List<Device>?> findAllTargetDevices({
    bool includeDevicesUnsupportedByProject = false,
  }) async {
    return _targetDevices.findAllTargetDevices(
      deviceDiscoveryTimeout: deviceDiscoveryTimeout,
      includeDevicesUnsupportedByProject: includeDevicesUnsupportedByProject,
    );
  }

  /// Find and return the target [Device] based upon currently connected
  /// devices and criteria entered by the user on the command line.
  /// If a device cannot be found that meets specified criteria,
  /// then print an error message and return null.
  ///
  /// If [includeDevicesUnsupportedByProject] is true, the tool does not filter
  /// the list by the current project support list.
  Future<Device?> findTargetDevice({bool includeDevicesUnsupportedByProject = false}) async {
    List<Device>? deviceList = await findAllTargetDevices(
      includeDevicesUnsupportedByProject: includeDevicesUnsupportedByProject,
    );
    if (deviceList == null) {
      return null;
    }
    if (deviceList.length > 1) {
      globals.printStatus(globals.userMessages.flutterSpecifyDevice);
      deviceList = await globals.deviceManager!.getAllDevices();
      globals.printStatus('');
      await Device.printDevices(deviceList, globals.logger);
      return null;
    }
    return deviceList.single;
  }

  @protected
  @mustCallSuper
  Future<void> validateCommand() async {
    if (_requiresPubspecYaml && globalResults?.wasParsed('packages') != true) {
      // Don't expect a pubspec.yaml file if the user passed in an explicit .packages file path.

      // If there is no pubspec in the current directory, look in the parent
      // until one can be found.
      final String? path = findProjectRoot(globals.fs, globals.fs.currentDirectory.path);
      if (path == null) {
        throwToolExit(globals.userMessages.flutterNoPubspec);
      }
      if (path != globals.fs.currentDirectory.path) {
        globals.fs.currentDirectory = path;
        globals.printStatus(
          'Changing current working directory to: ${globals.fs.currentDirectory.path}',
        );
      }
    }

    if (_usesTargetOption) {
      final String targetPath = targetFile;
      if (!globals.fs.isFileSync(targetPath)) {
        throwToolExit(globals.userMessages.flutterTargetFileMissing(targetPath));
      }
    }
  }

  @override
  String get usage {
    final String usageWithoutDescription = super.usage.substring(
      // The description plus two newlines.
      description.length + 2,
    );
    final String help = <String>[
      if (deprecated)
        '${globals.logger.terminal.warningMark} Deprecated. This command will be removed in a future version of Flutter.',
      description,
      '',
      'Global options:',
      '${runner?.argParser.usage}',
      '',
      usageWithoutDescription,
    ].join('\n');
    return help;
  }

  ApplicationPackageFactory? applicationPackages;

  /// Gets the parsed command-line flag named [name] as a `bool`.
  ///
  /// If no flag named [name] was added to the [ArgParser], an [ArgumentError]
  /// will be thrown.
  bool boolArg(String name, {bool global = false}) {
    return (global ? globalResults : argResults)!.flag(name);
  }

  /// Gets the parsed command-line option named [name] as a `String`.
  ///
  /// If no option named [name] was added to the [ArgParser], an [ArgumentError]
  /// will be thrown.
  String? stringArg(String name, {bool global = false}) {
    return (global ? globalResults : argResults)!.option(name);
  }

  /// Gets the parsed command-line option named [name] as `List<String>`.
  List<String> stringsArg(String name, {bool global = false}) {
    return (global ? globalResults : argResults)!.multiOption(name);
  }
}

/// A mixin which applies an implementation of [requiredArtifacts] that only
/// downloads artifacts corresponding to potentially connected devices.
mixin DeviceBasedDevelopmentArtifacts on FlutterCommand {
  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async {
    // If there are no devices, use the default configuration.
    // Otherwise, only add development artifacts corresponding to
    // potentially connected devices. We might not be able to determine if a
    // device is connected yet, so include it in case it becomes connected.
    final List<Device> devices = await globals.deviceManager!.getDevices(
      filter: DeviceDiscoveryFilter(excludeDisconnected: false),
    );
    if (devices.isEmpty) {
      return super.requiredArtifacts;
    }
    final Set<DevelopmentArtifact> artifacts = <DevelopmentArtifact>{DevelopmentArtifact.universal};
    for (final Device device in devices) {
      final TargetPlatform targetPlatform = await device.targetPlatform;
      final DevelopmentArtifact? developmentArtifact = artifactFromTargetPlatform(targetPlatform);
      if (developmentArtifact != null) {
        artifacts.add(developmentArtifact);
      }
    }
    return artifacts;
  }
}

// Returns the development artifact for the target platform, or null
// if none is supported
@protected
DevelopmentArtifact? artifactFromTargetPlatform(TargetPlatform targetPlatform) {
  switch (targetPlatform) {
    case TargetPlatform.android:
    case TargetPlatform.android_arm:
    case TargetPlatform.android_arm64:
    case TargetPlatform.android_x64:
    case TargetPlatform.android_x86:
      return DevelopmentArtifact.androidGenSnapshot;
    case TargetPlatform.web_javascript:
      return DevelopmentArtifact.web;
    case TargetPlatform.ios:
      return DevelopmentArtifact.iOS;
    case TargetPlatform.darwin:
      if (featureFlags.isMacOSEnabled) {
        return DevelopmentArtifact.macOS;
      }
      return null;
    case TargetPlatform.windows_x64:
    case TargetPlatform.windows_arm64:
      if (featureFlags.isWindowsEnabled) {
        return DevelopmentArtifact.windows;
      }
      return null;
    case TargetPlatform.linux_x64:
    case TargetPlatform.linux_arm64:
      if (featureFlags.isLinuxEnabled) {
        return DevelopmentArtifact.linux;
      }
      return null;
    case TargetPlatform.fuchsia_arm64:
    case TargetPlatform.fuchsia_x64:
    case TargetPlatform.tester:
      return null;
  }
}

/// Returns true if s is either null, empty or is solely made of whitespace characters (as defined by String.trim).
bool _isBlank(String s) => s.trim().isEmpty;

/// Whether the tool should allow non-null safe builds.
///
/// The Dart SDK no longer supports non-null safe builds, so this value in the
/// tool's context should always be [NonNullSafeBuilds.notAllowed].
enum NonNullSafeBuilds { allowed, notAllowed }
