// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:unified_analytics/unified_analytics.dart';
import 'package:vm_service/vm_service.dart';

import '../android/android_device.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/signals.dart';
import '../base/terminal.dart';
import '../build_info.dart';
import '../commands/daemon.dart';
import '../compile.dart';
import '../daemon.dart';
import '../device.dart';
import '../device_port_forwarder.dart';
import '../device_vm_service_discovery_for_attach.dart';
import '../ios/devices.dart';
import '../macos/macos_ipad_device.dart';
import '../mdns_discovery.dart';
import '../project.dart';
import '../resident_runner.dart';
import '../run_cold.dart';
import '../run_hot.dart';
import '../runner/flutter_command.dart';
import '../runner/flutter_command_runner.dart';

/// A Flutter-command that attaches to applications that have been launched
/// without `flutter run`.
///
/// With an application already running, a HotRunner can be attached to it
/// with:
/// ```bash
/// $ flutter attach --debug-url http://127.0.0.1:12345/QqL7EFEDNG0=/
/// ```
///
/// If `--disable-service-auth-codes` was provided to the application at startup
/// time, a HotRunner can be attached with just a port:
/// ```bash
/// $ flutter attach --debug-port 12345
/// ```
///
/// Alternatively, the attach command can start listening and scan for new
/// programs that become active:
/// ```bash
/// $ flutter attach
/// ```
/// As soon as a new VM Service is detected the command attaches to it and
/// enables hot reloading.
///
/// To attach to a flutter mod running on a fuchsia device, `--module` must
/// also be provided.
class AttachCommand extends FlutterCommand {
  AttachCommand({
    bool verboseHelp = false,
    HotRunnerFactory? hotRunnerFactory,
    required Stdio stdio,
    required Logger logger,
    required Terminal terminal,
    required Signals signals,
    required Platform platform,
    required ProcessInfo processInfo,
    required FileSystem fileSystem,
  }) : _hotRunnerFactory = hotRunnerFactory ?? HotRunnerFactory(),
       _stdio = stdio,
       _logger = logger,
       _terminal = terminal,
       _signals = signals,
       _platform = platform,
       _processInfo = processInfo,
       _fileSystem = fileSystem {
    addBuildModeFlags(verboseHelp: verboseHelp, defaultToRelease: false, excludeRelease: true);
    usesTargetOption();
    usesPortOptions(verboseHelp: verboseHelp);
    usesIpv6Flag(verboseHelp: verboseHelp);
    usesFilesystemOptions(hide: !verboseHelp);
    usesFuchsiaOptions(hide: !verboseHelp);
    usesDartDefineOption();
    usesDeviceUserOption();
    addEnableExperimentation(hide: !verboseHelp);
    addNullSafetyModeOptions(hide: !verboseHelp);
    usesInitializeFromDillOption(hide: !verboseHelp);
    usesNativeAssetsOption(hide: !verboseHelp);
    argParser
      ..addOption(
        'debug-port',
        hide: !verboseHelp,
        help:
            '(deprecated) Device port where the Dart VM Service is listening. Requires '
            '"--disable-service-auth-codes" to also be provided to the Flutter '
            'application at launch, otherwise this command will fail to connect to '
            'the application. In general, "--debug-url" should be used instead.',
      )
      ..addOption(
        'debug-url',
        aliases: <String>['debug-uri'], // supported for historical reasons
        help: 'The URL at which the Dart VM Service is listening.',
      )
      ..addOption(
        'app-id',
        help:
            'The package name (Android) or bundle identifier (iOS) for the app. '
            'This can be specified to avoid being prompted if multiple Dart VM Service ports '
            'are advertised.\n'
            'If you have multiple devices or emulators running, you should include the '
            'device hostname as well, e.g. "com.example.myApp@my-iphone".\n'
            'This parameter is case-insensitive.',
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
      ..addOption('project-root', hide: !verboseHelp, help: 'Normally used only in run target.')
      ..addFlag(
        'machine',
        hide: !verboseHelp,
        negatable: false,
        help:
            'Handle machine structured JSON command input and provide output '
            'and progress in machine-friendly format.',
      );
    usesTrackWidgetCreation(verboseHelp: verboseHelp);
    addDdsOptions(verboseHelp: verboseHelp);
    addDevToolsOptions(verboseHelp: verboseHelp);
    addServeObservatoryOptions(verboseHelp: verboseHelp);
    usesDeviceTimeoutOption();
    usesDeviceConnectionOption();
  }

  final HotRunnerFactory _hotRunnerFactory;
  final Stdio _stdio;
  final Logger _logger;
  final Terminal _terminal;
  final Signals _signals;
  final Platform _platform;
  final ProcessInfo _processInfo;
  final FileSystem _fileSystem;

  @override
  final String name = 'attach';

  @override
  final String description = r'''
Attach to a running app.

For attaching to Android or iOS devices, simply using `flutter attach` is
usually sufficient. The tool will search for a running Flutter app or module,
if available. Otherwise, the tool will wait for the next Flutter app or module
to launch before attaching.

For Fuchsia, the module name must be provided, e.g. `$flutter attach
--module=mod_name`. This can be called either before or after the application
is started.

If the app or module is already running and the specific vmService port is
known, it can be explicitly provided to attach via the command-line, e.g.
`$ flutter attach --debug-port 12345`''';

  @override
  final String category = FlutterCommandCategory.tools;

  @override
  bool get refreshWirelessDevices => true;

  int? get debugPort {
    if (argResults!['debug-port'] == null) {
      return null;
    }
    try {
      return int.parse(stringArg('debug-port')!);
    } on Exception catch (error) {
      throwToolExit('Invalid port for `--debug-port`: $error');
    }
  }

  Uri? get debugUri {
    final String? debugUrl = stringArg('debug-url');
    if (debugUrl == null) {
      return null;
    }
    final Uri? uri = Uri.tryParse(debugUrl);
    if (uri == null) {
      throwToolExit('Invalid `--debug-url`: $debugUrl');
    }
    if (!uri.hasPort) {
      throwToolExit('Port not specified for `--debug-url`: $uri');
    }
    return uri;
  }

  bool get serveObservatory => boolArg('serve-observatory');

  String? get appId {
    return stringArg('app-id');
  }

  String? get userIdentifier => stringArg(FlutterOptions.kDeviceUser);

  @override
  Future<void> validateCommand() async {
    await super.validateCommand();

    final Device? targetDevice = await findTargetDevice();
    if (targetDevice == null) {
      throwToolExit(null);
    }

    debugPort;
    // Allow --ipv6 for iOS devices even if --debug-port and --debug-url
    // are unknown.
    if (!_isIOSDevice(targetDevice) &&
        debugPort == null &&
        debugUri == null &&
        argResults!.wasParsed(FlutterCommand.ipv6Flag)) {
      throwToolExit(
        'When the --debug-port or --debug-url is unknown, this command determines '
        'the value of --ipv6 on its own.',
      );
    }
    if (debugPort == null &&
        debugUri == null &&
        argResults!.wasParsed(FlutterCommand.vmServicePortOption)) {
      throwToolExit(
        'When the --debug-port or --debug-url is unknown, this command does not use '
        'the value of --vm-service-port.',
      );
    }
    if (debugPort != null && debugUri != null) {
      throwToolExit('Either --debug-port or --debug-url can be provided, not both.');
    }

    if (userIdentifier != null) {
      final Device? device = await findTargetDevice();
      if (device is! AndroidDevice) {
        throwToolExit('--${FlutterOptions.kDeviceUser} is only supported for Android');
      }
    }
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    await _validateArguments();

    final Device? device = await findTargetDevice();

    if (device == null) {
      throwToolExit('Did not find any valid target devices.');
    }

    await _attachToDevice(device);

    return FlutterCommandResult.success();
  }

  Future<void> _attachToDevice(Device device) async {
    final FlutterProject flutterProject = FlutterProject.current();

    final Daemon? daemon =
        boolArg('machine')
            ? Daemon(
              DaemonConnection(
                daemonStreams: DaemonStreams.fromStdio(_stdio, logger: _logger),
                logger: _logger,
              ),
              notifyingLogger:
                  (_logger is NotifyingLogger)
                      ? _logger
                      : NotifyingLogger(verbose: _logger.isVerbose, parent: _logger),
              logToStdout: true,
            )
            : null;

    Stream<Uri>? vmServiceUri;
    final bool usesIpv6 = ipv6!;
    final String ipv6Loopback = InternetAddress.loopbackIPv6.address;
    final String ipv4Loopback = InternetAddress.loopbackIPv4.address;
    final String hostname = usesIpv6 ? ipv6Loopback : ipv4Loopback;
    final bool isWirelessIOSDevice = (device is IOSDevice) && device.isWirelesslyConnected;

    if ((debugPort == null && debugUri == null) || isWirelessIOSDevice) {
      // The device port we expect to have the debug port be listening
      final int? devicePort = debugPort ?? debugUri?.port ?? deviceVmservicePort;

      final VMServiceDiscoveryForAttach vmServiceDiscovery = device.getVMServiceDiscoveryForAttach(
        appId: appId,
        fuchsiaModule: stringArg('module'),
        filterDevicePort: devicePort,
        expectedHostPort: hostVmservicePort,
        ipv6: usesIpv6,
        logger: _logger,
      );

      _logger.printStatus('Waiting for a connection from Flutter on ${device.name}...');
      final Status discoveryStatus = _logger.startSpinner(
        timeout: const Duration(seconds: 30),
        slowWarningCallback: () {
          // On iOS we rely on mDNS to find Dart VM Service. Remind the user to allow local network permissions on the device.
          if (_isIOSDevice(device)) {
            return 'The Dart VM Service was not discovered after 30 seconds. This is taking much longer than expected...\n\n'
                'Click "Allow" to the prompt on your device asking if you would like to find and connect devices on your local network. '
                'If you selected "Don\'t Allow", you can turn it on in Settings > Your App Name > Local Network. '
                "If you don't see your app in the Settings, uninstall the app and rerun to see the prompt again.\n";
          }

          return 'The Dart VM Service was not discovered after 30 seconds. This is taking much longer than expected...\n';
        },
      );

      vmServiceUri = vmServiceDiscovery.uris;

      // Stop the timer once we receive the first uri.
      vmServiceUri = vmServiceUri.map((Uri uri) {
        discoveryStatus.stop();
        return uri;
      });
    } else {
      vmServiceUri =
          Stream<Uri>.fromFuture(
            buildVMServiceUri(
              device,
              debugUri?.host ?? hostname,
              debugPort ?? debugUri!.port,
              hostVmservicePort,
              debugUri?.path,
            ),
          ).asBroadcastStream();
    }

    _terminal.usesTerminalUi = daemon == null;

    try {
      int? result;
      if (daemon != null) {
        final ResidentRunner runner = await createResidentRunner(
          vmServiceUris: vmServiceUri,
          device: device,
          flutterProject: flutterProject,
          usesIpv6: usesIpv6,
        );
        late AppInstance app;
        try {
          app = await daemon.appDomain.launch(
            runner,
            ({
              Completer<DebugConnectionInfo>? connectionInfoCompleter,
              Completer<void>? appStartedCompleter,
            }) {
              return runner.attach(
                connectionInfoCompleter: connectionInfoCompleter,
                appStartedCompleter: appStartedCompleter,
                allowExistingDdsInstance: true,
              );
            },
            device,
            null,
            true,
            _fileSystem.currentDirectory,
            LaunchMode.attach,
            _logger as AppRunLogger,
          );
        } on Exception catch (error) {
          throwToolExit(error.toString());
        }
        result = await app.runner.waitForAppToFinish();
        return;
      }
      while (true) {
        final ResidentRunner runner = await createResidentRunner(
          vmServiceUris: vmServiceUri,
          device: device,
          flutterProject: flutterProject,
          usesIpv6: usesIpv6,
        );
        final Completer<void> onAppStart = Completer<void>.sync();
        TerminalHandler? terminalHandler;
        unawaited(
          onAppStart.future.whenComplete(() {
            terminalHandler =
                TerminalHandler(
                    runner,
                    logger: _logger,
                    terminal: _terminal,
                    signals: _signals,
                    processInfo: _processInfo,
                    reportReady: boolArg('report-ready'),
                    pidFile: stringArg('pid-file'),
                  )
                  ..registerSignalHandlers()
                  ..setupTerminal();
          }),
        );
        result = await runner.attach(
          appStartedCompleter: onAppStart,
          allowExistingDdsInstance: true,
        );
        if (result != 0) {
          throwToolExit(null, exitCode: result);
        }
        terminalHandler?.stop();
        assert(result != null);
        if (runner.exited || !runner.isWaitingForVmService) {
          break;
        }
        _logger.printStatus('Waiting for a new connection from Flutter on ${device.name}...');
      }
    } on RPCError catch (err) {
      if (err.code == RPCErrorKind.kServiceDisappeared.code ||
          err.message.contains('Service connection disposed')) {
        throwToolExit('Lost connection to device.');
      }
      rethrow;
    } finally {
      final List<ForwardedPort> ports = device.portForwarder!.forwardedPorts.toList();
      for (final ForwardedPort port in ports) {
        await device.portForwarder!.unforward(port);
      }
      // However we exited from the runner, ensure the terminal has line mode
      // and echo mode enabled before we return the user to the shell.
      try {
        _terminal.singleCharMode = false;
      } on StdinException {
        // Do nothing, if the STDIN handle is no longer available, there is nothing actionable for us to do at this point
      }
    }
  }

  Future<ResidentRunner> createResidentRunner({
    required Stream<Uri> vmServiceUris,
    required Device device,
    required FlutterProject flutterProject,
    required bool usesIpv6,
  }) async {
    final BuildInfo buildInfo = await getBuildInfo();

    final FlutterDevice flutterDevice = await FlutterDevice.create(
      device,
      target: targetFile,
      targetModel: TargetModel(stringArg('target-model')!),
      buildInfo: buildInfo,
      userIdentifier: userIdentifier,
      platform: _platform,
    );
    flutterDevice.vmServiceUris = vmServiceUris;
    final List<FlutterDevice> flutterDevices = <FlutterDevice>[flutterDevice];
    final DebuggingOptions debuggingOptions = DebuggingOptions.enabled(
      buildInfo,
      enableDds: enableDds,
      ddsPort: ddsPort,
      devToolsServerAddress: devToolsServerAddress,
      serveObservatory: serveObservatory,
      usingCISystem: usingCISystem,
      debugLogsDirectoryPath: debugLogsDirectoryPath,
      enableDevTools: boolArg(FlutterCommand.kEnableDevTools),
      ipv6: usesIpv6,
      printDtd: boolArg(FlutterGlobalOptions.kPrintDtd, global: true),
    );

    return buildInfo.isDebug
        ? _hotRunnerFactory.build(
          flutterDevices,
          target: targetFile,
          debuggingOptions: debuggingOptions,
          packagesFilePath: globalResults![FlutterGlobalOptions.kPackagesOption] as String?,
          projectRootPath: stringArg('project-root'),
          dillOutputPath: stringArg('output-dill'),
          flutterProject: flutterProject,
          nativeAssetsYamlFile: stringArg(FlutterOptions.kNativeAssetsYamlFile),
          analytics: analytics,
        )
        : ColdRunner(flutterDevices, target: targetFile, debuggingOptions: debuggingOptions);
  }

  Future<void> _validateArguments() async {}

  bool _isIOSDevice(Device device) {
    return (device.platformType == PlatformType.ios) || (device is MacOSDesignedForIPadDevice);
  }
}

class HotRunnerFactory {
  HotRunner build(
    List<FlutterDevice> devices, {
    required String target,
    required DebuggingOptions debuggingOptions,
    bool benchmarkMode = false,
    File? applicationBinary,
    bool hostIsIde = false,
    String? projectRootPath,
    String? packagesFilePath,
    String? dillOutputPath,
    bool stayResident = true,
    FlutterProject? flutterProject,
    String? nativeAssetsYamlFile,
    required Analytics analytics,
  }) => HotRunner(
    devices,
    target: target,
    debuggingOptions: debuggingOptions,
    benchmarkMode: benchmarkMode,
    applicationBinary: applicationBinary,
    hostIsIde: hostIsIde,
    projectRootPath: projectRootPath,
    dillOutputPath: dillOutputPath,
    stayResident: stayResident,
    nativeAssetsYamlFile: nativeAssetsYamlFile,
    analytics: analytics,
  );
}
