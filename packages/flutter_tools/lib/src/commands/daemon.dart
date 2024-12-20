// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:async/async.dart';
import 'package:meta/meta.dart';
import 'package:uuid/uuid.dart';

import '../android/android_workflow.dart';
import '../application_package.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/terminal.dart';
import '../base/utils.dart';
import '../build_info.dart';
import '../convert.dart';
import '../daemon.dart';
import '../device.dart';
import '../device_port_forwarder.dart';
import '../device_vm_service_discovery_for_attach.dart';
import '../emulator.dart';
import '../features.dart';
import '../globals.dart' as globals;
import '../project.dart';
import '../proxied_devices/debounce_data_stream.dart';
import '../proxied_devices/file_transfer.dart';
import '../resident_runner.dart';
import '../run_cold.dart';
import '../run_hot.dart';
import '../runner/flutter_command.dart';
import '../vmservice.dart';
import '../web/web_runner.dart';

const String protocolVersion = '0.6.1';

/// A server process command. This command will start up a long-lived server.
/// It reads JSON-RPC based commands from stdin, executes them, and returns
/// JSON-RPC based responses and events to stdout.
///
/// It can be shutdown with a `daemon.shutdown` command (or by killing the
/// process).
class DaemonCommand extends FlutterCommand {
  DaemonCommand({this.hidden = false}) {
    argParser.addOption(
      'listen-on-tcp-port',
      help:
          'If specified, the daemon will be listening for commands on the specified port instead of stdio.',
      valueHelp: 'port',
    );
  }

  @override
  final String name = 'daemon';

  @override
  final String description = 'Run a persistent, JSON-RPC based server to communicate with devices.';

  @override
  final String category = FlutterCommandCategory.tools;

  @override
  final bool hidden;

  @override
  Future<FlutterCommandResult> runCommand() async {
    if (argResults!['listen-on-tcp-port'] != null) {
      int? port;
      try {
        port = int.parse(stringArg('listen-on-tcp-port')!);
      } on FormatException catch (error) {
        throwToolExit('Invalid port for `--listen-on-tcp-port`: $error');
      }

      await DaemonServer(
        port: port,
        logger: StdoutLogger(
          terminal: globals.terminal,
          stdio: globals.stdio,
          outputPreferences: globals.outputPreferences,
        ),
        notifyingLogger: asLogger<NotifyingLogger>(globals.logger),
      ).run();
      return FlutterCommandResult.success();
    }
    globals.printStatus('Starting device daemon...');
    final Daemon daemon = Daemon(
      DaemonConnection(
        daemonStreams: DaemonStreams.fromStdio(globals.stdio, logger: globals.logger),
        logger: globals.logger,
      ),
      notifyingLogger: asLogger<NotifyingLogger>(globals.logger),
    );
    final int code = await daemon.onExit;
    if (code != 0) {
      throwToolExit('Daemon exited with non-zero exit code: $code', exitCode: code);
    }
    return FlutterCommandResult.success();
  }
}

@visibleForTesting
class DaemonServer {
  DaemonServer({
    this.port,
    required this.logger,
    this.notifyingLogger,
    @visibleForTesting
    Future<ServerSocket> Function(InternetAddress address, int port) bind = ServerSocket.bind,
  }) : _bind = bind;

  final int? port;

  /// Stdout logger used to print general server-related errors.
  final Logger logger;

  // Logger that sends the message to the other end of daemon connection.
  final NotifyingLogger? notifyingLogger;

  final Future<ServerSocket> Function(InternetAddress address, int port) _bind;

  Future<void> run() async {
    ServerSocket? serverSocket;
    try {
      serverSocket = await _bind(InternetAddress.loopbackIPv4, port!);
    } on SocketException {
      logger.printTrace('Bind on $port failed with IPv4, retrying on IPv6');
    }

    // If binding on IPv4 failed, try binding on IPv6.
    // Omit try catch here, let the failure fallthrough.
    serverSocket ??= await _bind(InternetAddress.loopbackIPv6, port!);

    logger.printStatus('Daemon server listening on ${serverSocket.port}');

    final StreamSubscription<Socket> subscription = serverSocket.listen((Socket socket) async {
      // We have to listen to socket.done. Otherwise when the connection is
      // reset, we will receive an uncatchable exception.
      // https://github.com/dart-lang/sdk/issues/25518
      final Future<void> socketDone = socket.done.then<void>(
        (_) {},
        onError: (Object error, StackTrace stackTrace) {
          logger.printError('Socket error: $error');
          logger.printTrace('$stackTrace');
        },
      );
      final Daemon daemon = Daemon(
        DaemonConnection(
          daemonStreams: DaemonStreams.fromSocket(socket, logger: logger),
          logger: logger,
        ),
        notifyingLogger: notifyingLogger,
      );
      await daemon.onExit;
      await socketDone;
    });

    // Wait indefinitely until the server closes.
    await subscription.asFuture<void>();
    await subscription.cancel();
  }
}

typedef CommandHandler = Future<Object?>? Function(Map<String, Object?> args);
typedef CommandHandlerWithBinary =
    Future<Object?> Function(Map<String, Object?> args, Stream<List<int>>? binary);

class Daemon {
  Daemon(
    this.connection, {
    this.notifyingLogger,
    this.logToStdout = false,
    FileTransfer fileTransfer = const FileTransfer(),
  }) {
    // Set up domains.
    registerDomain(daemonDomain = DaemonDomain(this));
    registerDomain(appDomain = AppDomain(this));
    registerDomain(deviceDomain = DeviceDomain(this));
    registerDomain(emulatorDomain = EmulatorDomain(this));
    registerDomain(devToolsDomain = DevToolsDomain(this));
    registerDomain(proxyDomain = ProxyDomain(this, fileTransfer: fileTransfer));

    // Start listening.
    _commandSubscription = connection.incomingCommands.listen(
      _handleRequest,
      onDone: () {
        shutdown();
        if (!_onExitCompleter.isCompleted) {
          _onExitCompleter.complete(0);
        }
      },
    );
  }

  final DaemonConnection connection;

  late DaemonDomain daemonDomain;
  late AppDomain appDomain;
  late DeviceDomain deviceDomain;
  EmulatorDomain? emulatorDomain;
  DevToolsDomain? devToolsDomain;
  late ProxyDomain proxyDomain;
  StreamSubscription<DaemonMessage>? _commandSubscription;

  final NotifyingLogger? notifyingLogger;
  final bool logToStdout;

  final Completer<int> _onExitCompleter = Completer<int>();
  final Map<String, Domain> _domainMap = <String, Domain>{};

  @visibleForTesting
  void registerDomain(Domain domain) {
    _domainMap[domain.name] = domain;
  }

  Future<int> get onExit => _onExitCompleter.future;

  void _handleRequest(DaemonMessage request) {
    // {id, method, params}

    // [id] is an opaque type to us.
    final Object? id = request.data['id'];

    if (id == null) {
      globals.stdio.stderrWrite('no id for request: $request\n');
      return;
    }

    try {
      final String method = request.data['method']! as String;
      if (!method.contains('.')) {
        throw DaemonException('method not understood: $method');
      }

      final String prefix = method.substring(0, method.indexOf('.'));
      final String name = method.substring(method.indexOf('.') + 1);
      if (_domainMap[prefix] == null) {
        throw DaemonException('no domain for method: $method');
      }

      _domainMap[prefix]!.handleCommand(
        name,
        id,
        castStringKeyedMap(request.data['params']) ?? const <String, Object?>{},
        request.binary,
      );
    } on Exception catch (error, trace) {
      connection.sendErrorResponse(id, _toJsonable(error), trace);
    }
  }

  Future<void> shutdown({Object? error}) async {
    await devToolsDomain?.dispose();
    await _commandSubscription?.cancel();
    await connection.dispose();
    for (final Domain domain in _domainMap.values) {
      await domain.dispose();
    }
    if (!_onExitCompleter.isCompleted) {
      if (error == null) {
        _onExitCompleter.complete(0);
      } else {
        _onExitCompleter.completeError(error);
      }
    }
  }
}

abstract class Domain {
  Domain(this.daemon, this.name);

  final Daemon daemon;
  final String name;
  final Map<String, CommandHandler> _handlers = <String, CommandHandler>{};
  final Map<String, CommandHandlerWithBinary> _handlersWithBinary =
      <String, CommandHandlerWithBinary>{};

  void registerHandler(String name, CommandHandler handler) {
    assert(!_handlers.containsKey(name));
    assert(!_handlersWithBinary.containsKey(name));
    _handlers[name] = handler;
  }

  void registerHandlerWithBinary(String name, CommandHandlerWithBinary handler) {
    assert(!_handlers.containsKey(name));
    assert(!_handlersWithBinary.containsKey(name));
    _handlersWithBinary[name] = handler;
  }

  @override
  String toString() => name;

  void handleCommand(
    String command,
    Object id,
    Map<String, Object?> args,
    Stream<List<int>>? binary,
  ) {
    Future<Object?>.sync(() {
      if (_handlers.containsKey(command)) {
        return _handlers[command]!(args);
      } else if (_handlersWithBinary.containsKey(command)) {
        return _handlersWithBinary[command]!(args, binary);
      }
      throw DaemonException('command not understood: $name.$command');
    }).then<Object?>(
      (Object? result) {
        daemon.connection.sendResponse(id, _toJsonable(result));
        return null;
      },
      onError: (Object error, StackTrace stackTrace) {
        daemon.connection.sendErrorResponse(id, _toJsonable(error), stackTrace);
        return null;
      },
    );
  }

  void sendEvent(String name, [Object? args, List<int>? binary]) {
    daemon.connection.sendEvent(name, _toJsonable(args), binary);
  }

  String? _getStringArg(Map<String, Object?> args, String name, {bool required = false}) {
    if (required && !args.containsKey(name)) {
      throw DaemonException('$name is required');
    }
    final Object? val = args[name];
    if (val != null && val is! String) {
      throw DaemonException('$name is not a String');
    }
    return val as String?;
  }

  bool? _getBoolArg(Map<String, Object?> args, String name, {bool required = false}) {
    if (required && !args.containsKey(name)) {
      throw DaemonException('$name is required');
    }
    final Object? val = args[name];
    if (val != null && val is! bool) {
      throw DaemonException('$name is not a bool');
    }
    return val as bool?;
  }

  int? _getIntArg(Map<String, Object?> args, String name, {bool required = false}) {
    if (required && !args.containsKey(name)) {
      throw DaemonException('$name is required');
    }
    final Object? val = args[name];
    if (val != null && val is! int) {
      throw DaemonException('$name is not an int');
    }
    return val as int?;
  }

  Future<void> dispose() async {}
}

/// This domain responds to methods like [version] and [shutdown].
///
/// This domain fires the `daemon.logMessage` event.
class DaemonDomain extends Domain {
  DaemonDomain(Daemon daemon) : super(daemon, 'daemon') {
    registerHandler('version', version);
    registerHandler('shutdown', shutdown);
    registerHandler('getSupportedPlatforms', getSupportedPlatforms);
    registerHandler('setNotifyVerbose', setNotifyVerbose);

    sendEvent('daemon.connected', <String, Object?>{'version': protocolVersion, 'pid': pid});

    _subscription = daemon.notifyingLogger!.onMessage.listen((LogMessage message) {
      if (daemon.logToStdout) {
        if (message.level == 'status' || message.level == 'trace') {
          // We use `print()` here instead of `stdout.writeln()` in order to
          // capture the print output for testing.
          // ignore: avoid_print
          print(message.message);
        } else if (message.level == 'error' || message.level == 'warning') {
          globals.stdio.stderrWrite('${message.message}\n');
          if (message.stackTrace != null) {
            globals.stdio.stderrWrite('${message.stackTrace.toString().trimRight()}\n');
          }
        }
      } else {
        if (message.stackTrace != null) {
          sendEvent('daemon.logMessage', <String, Object?>{
            'level': message.level,
            'message': message.message,
            'stackTrace': message.stackTrace.toString(),
          });
        } else {
          sendEvent('daemon.logMessage', <String, Object?>{
            'level': message.level,
            'message': message.message,
          });
        }
      }
    });
  }

  StreamSubscription<LogMessage>? _subscription;

  Future<String> version(Map<String, Object?> args) {
    return Future<String>.value(protocolVersion);
  }

  /// Sends a request back to the client asking it to expose/tunnel a URL.
  ///
  /// This method should only be called if the client opted-in with the
  /// --web-allow-expose-url switch. The client may return the same URL back if
  /// tunnelling is not required for a given URL.
  Future<String> exposeUrl(String url) async {
    final Object? res = await daemon.connection.sendRequest('app.exposeUrl', <String, String>{
      'url': url,
    });
    if (res is Map<String, Object?> && res['url'] is String) {
      return res['url']! as String;
    } else {
      globals.printError(
        'Invalid response to exposeUrl - params should include a String url field',
      );
      return url;
    }
  }

  Future<void> shutdown(Map<String, Object?> args) {
    Timer.run(daemon.shutdown);
    return Future<void>.value();
  }

  @override
  Future<void> dispose() async {
    await _subscription?.cancel();
  }

  /// Enumerates the platforms supported by the provided project.
  ///
  /// This does not filter based on the current workflow restrictions, such
  /// as whether command line tools are installed or whether the host platform
  /// is correct.
  Future<Map<String, Object>> getSupportedPlatforms(Map<String, Object?> args) async {
    final String? projectRoot = _getStringArg(args, 'projectRoot', required: true);
    final List<String> platformTypes = <String>[];
    final Map<String, Object> platformTypesMap = <String, Object>{};
    try {
      final FlutterProject flutterProject = FlutterProject.fromDirectory(
        globals.fs.directory(projectRoot),
      );
      final Set<SupportedPlatform> supportedPlatforms =
          flutterProject.getSupportedPlatforms().toSet();

      void handlePlatformType(PlatformType platform) {
        final List<Map<String, Object>> reasons = <Map<String, Object>>[];
        switch (platform) {
          case PlatformType.linux:
            if (!featureFlags.isLinuxEnabled) {
              reasons.add(<String, Object>{
                'reasonText': 'the Linux feature is not enabled',
                'fixText': 'Run "flutter config --enable-linux-desktop"',
                'fixCode': _ReasonCode.config.name,
              });
            }
            if (!supportedPlatforms.contains(SupportedPlatform.linux)) {
              reasons.add(<String, Object>{
                'reasonText': 'the Linux platform is not enabled for this project',
                'fixText': 'Run "flutter create --platforms=linux ." in your application directory',
                'fixCode': _ReasonCode.create.name,
              });
            }
          case PlatformType.macos:
            if (!featureFlags.isMacOSEnabled) {
              reasons.add(<String, Object>{
                'reasonText': 'the macOS feature is not enabled',
                'fixText': 'Run "flutter config --enable-macos-desktop"',
                'fixCode': _ReasonCode.config.name,
              });
            }
            if (!supportedPlatforms.contains(SupportedPlatform.macos)) {
              reasons.add(<String, Object>{
                'reasonText': 'the macOS platform is not enabled for this project',
                'fixText': 'Run "flutter create --platforms=macos ." in your application directory',
                'fixCode': _ReasonCode.create.name,
              });
            }
          case PlatformType.windows:
            if (!featureFlags.isWindowsEnabled) {
              reasons.add(<String, Object>{
                'reasonText': 'the Windows feature is not enabled',
                'fixText': 'Run "flutter config --enable-windows-desktop"',
                'fixCode': _ReasonCode.config.name,
              });
            }
            if (!supportedPlatforms.contains(SupportedPlatform.windows)) {
              reasons.add(<String, Object>{
                'reasonText': 'the Windows platform is not enabled for this project',
                'fixText':
                    'Run "flutter create --platforms=windows ." in your application directory',
                'fixCode': _ReasonCode.create.name,
              });
            }
          case PlatformType.ios:
            if (!featureFlags.isIOSEnabled) {
              reasons.add(<String, Object>{
                'reasonText': 'the iOS feature is not enabled',
                'fixText': 'Run "flutter config --enable-ios"',
                'fixCode': _ReasonCode.config.name,
              });
            }
            if (!supportedPlatforms.contains(SupportedPlatform.ios)) {
              reasons.add(<String, Object>{
                'reasonText': 'the iOS platform is not enabled for this project',
                'fixText': 'Run "flutter create --platforms=ios ." in your application directory',
                'fixCode': _ReasonCode.create.name,
              });
            }
          case PlatformType.android:
            if (!featureFlags.isAndroidEnabled) {
              reasons.add(<String, Object>{
                'reasonText': 'the Android feature is not enabled',
                'fixText': 'Run "flutter config --enable-android"',
                'fixCode': _ReasonCode.config.name,
              });
            }
            if (!supportedPlatforms.contains(SupportedPlatform.android)) {
              reasons.add(<String, Object>{
                'reasonText': 'the Android platform is not enabled for this project',
                'fixText':
                    'Run "flutter create --platforms=android ." in your application directory',
                'fixCode': _ReasonCode.create.name,
              });
            }
          case PlatformType.web:
            if (!featureFlags.isWebEnabled) {
              reasons.add(<String, Object>{
                'reasonText': 'the Web feature is not enabled',
                'fixText': 'Run "flutter config --enable-web"',
                'fixCode': _ReasonCode.config.name,
              });
            }
            if (!supportedPlatforms.contains(SupportedPlatform.web)) {
              reasons.add(<String, Object>{
                'reasonText': 'the Web platform is not enabled for this project',
                'fixText': 'Run "flutter create --platforms=web ." in your application directory',
                'fixCode': _ReasonCode.create.name,
              });
            }
          case PlatformType.fuchsia:
            if (!featureFlags.isFuchsiaEnabled) {
              reasons.add(<String, Object>{
                'reasonText': 'the Fuchsia feature is not enabled',
                'fixText': 'Run "flutter config --enable-fuchsia"',
                'fixCode': _ReasonCode.config.name,
              });
            }
            if (!supportedPlatforms.contains(SupportedPlatform.fuchsia)) {
              reasons.add(<String, Object>{
                'reasonText': 'the Fuchsia platform is not enabled for this project',
                'fixText':
                    'Run "flutter create --platforms=fuchsia ." in your application directory',
                'fixCode': _ReasonCode.create.name,
              });
            }
          case PlatformType.custom:
            if (!featureFlags.areCustomDevicesEnabled) {
              reasons.add(<String, Object>{
                'reasonText': 'the custom devices feature is not enabled',
                'fixText': 'Run "flutter config --enable-custom-devices"',
                'fixCode': _ReasonCode.config.name,
              });
            }
          case PlatformType.windowsPreview:
            // TODO(fujino): detect if there any plugins with native code
            if (!featureFlags.isPreviewDeviceEnabled) {
              reasons.add(<String, Object>{
                'reasonText': 'the Preview Device feature is not enabled',
                'fixText': 'Run "flutter config --enable-flutter-preview',
                'fixCode': _ReasonCode.config.name,
              });
            }
            if (!supportedPlatforms.contains(SupportedPlatform.windows)) {
              reasons.add(<String, Object>{
                'reasonText': 'the Windows platform is not enabled for this project',
                'fixText':
                    'Run "flutter create --platforms=windows ." in your application directory',
                'fixCode': _ReasonCode.create.name,
              });
            }
        }

        if (reasons.isEmpty) {
          platformTypes.add(platform.name);
          platformTypesMap[platform.name] = const <String, Object>{'isSupported': true};
        } else {
          platformTypesMap[platform.name] = <String, Object>{
            'isSupported': false,
            'reasons': reasons,
          };
        }
      }

      PlatformType.values.forEach(handlePlatformType);

      return <String, Object>{
        // TODO(fujino): delete this key https://github.com/flutter/flutter/issues/140473
        'platforms': platformTypes,
        'platformTypes': platformTypesMap,
      };
    } on Exception catch (err, stackTrace) {
      sendEvent('log', <String, Object?>{
        'log': 'Failed to parse project metadata',
        'stackTrace': stackTrace.toString(),
        'error': true,
      });
      // On any sort of failure, fall back to Android and iOS for backwards
      // compatibility.
      return const <String, Object>{
        'platforms': <String>['android', 'ios'],
        'platformTypes': <String, Object>{
          'android': <String, Object>{'isSupported': true},
          'ios': <String, Object>{'isSupported': true},
        },
      };
    }
  }

  /// If notifyVerbose is set, the daemon will forward all verbose logs.
  Future<void> setNotifyVerbose(Map<String, Object?> args) async {
    daemon.notifyingLogger?.notifyVerbose = _getBoolArg(args, 'verbose') ?? true;
  }
}

/// The reason a [PlatformType] is not currently supported.
///
/// The [name] of this value will be sent as a response to daemon client.
enum _ReasonCode { create, config }

typedef RunOrAttach =
    Future<void> Function({
      Completer<DebugConnectionInfo>? connectionInfoCompleter,
      Completer<void>? appStartedCompleter,
    });

/// This domain responds to methods like [start] and [stop].
///
/// It fires events for application start, stop, and stdout and stderr.
class AppDomain extends Domain {
  AppDomain(Daemon daemon) : super(daemon, 'app') {
    registerHandler('restart', restart);
    registerHandler('callServiceExtension', callServiceExtension);
    registerHandler('stop', stop);
    registerHandler('detach', detach);
  }

  static const Uuid _uuidGenerator = Uuid();

  static String _getNewAppId() => _uuidGenerator.v4();

  final List<AppInstance> _apps = <AppInstance>[];

  final DebounceOperationQueue<OperationResult, OperationType> operationQueue =
      DebounceOperationQueue<OperationResult, OperationType>();

  Future<AppInstance> startApp(
    Device device,
    String projectDirectory,
    String target,
    String? route,
    DebuggingOptions options,
    bool enableHotReload, {
    File? applicationBinary,
    required bool trackWidgetCreation,
    String? projectRootPath,
    String? packagesFilePath,
    String? dillOutputPath,
    String? isolateFilter,
    bool machine = true,
    String? userIdentifier,
  }) async {
    if (!await device.supportsRuntimeMode(options.buildInfo.mode)) {
      throw Exception(
        '${sentenceCase(options.buildInfo.friendlyModeName)} '
        'mode is not supported for ${device.name}.',
      );
    }

    // We change the current working directory for the duration of the `start` command.
    final Directory cwd = globals.fs.currentDirectory;
    globals.fs.currentDirectory = globals.fs.directory(projectDirectory);
    final FlutterProject flutterProject = FlutterProject.current();

    final FlutterDevice flutterDevice = await FlutterDevice.create(
      device,
      target: target,
      buildInfo: options.buildInfo,
      platform: globals.platform,
      userIdentifier: userIdentifier,
    );

    ResidentRunner runner;

    if (await device.targetPlatform == TargetPlatform.web_javascript) {
      runner = webRunnerFactory!.createWebRunner(
        flutterDevice,
        flutterProject: flutterProject,
        target: target,
        debuggingOptions: options,
        stayResident: true,
        urlTunneller: options.webEnableExposeUrl! ? daemon.daemonDomain.exposeUrl : null,
        machine: machine,
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
        logger: globals.logger,
        fileSystem: globals.fs,
      );
    } else if (enableHotReload) {
      runner = HotRunner(
        <FlutterDevice>[flutterDevice],
        target: target,
        debuggingOptions: options,
        applicationBinary: applicationBinary,
        projectRootPath: projectRootPath,
        dillOutputPath: dillOutputPath,
        hostIsIde: true,
        machine: machine,
        analytics: globals.analytics,
      );
    } else {
      runner = ColdRunner(
        <FlutterDevice>[flutterDevice],
        target: target,
        debuggingOptions: options,
        applicationBinary: applicationBinary,
        machine: machine,
      );
    }

    return launch(
      runner,
      ({
        Completer<DebugConnectionInfo>? connectionInfoCompleter,
        Completer<void>? appStartedCompleter,
      }) {
        return runner.run(
          connectionInfoCompleter: connectionInfoCompleter,
          appStartedCompleter: appStartedCompleter,
          route: route,
        );
      },
      device,
      projectDirectory,
      enableHotReload,
      cwd,
      LaunchMode.run,
      asLogger<AppRunLogger>(globals.logger),
    );
  }

  Future<AppInstance> launch(
    ResidentRunner runner,
    RunOrAttach runOrAttach,
    Device device,
    String? projectDirectory,
    bool enableHotReload,
    Directory cwd,
    LaunchMode launchMode,
    AppRunLogger logger,
  ) async {
    final AppInstance app = AppInstance(
      _getNewAppId(),
      runner: runner,
      logToStdout: daemon.logToStdout,
      logger: logger,
    );
    _apps.add(app);

    // Set the domain and app for the given AppRunLogger. This allows the logger
    // to log messages containing the app ID to the host.
    logger.domain = this;
    logger.app = app;

    _sendAppEvent(app, 'start', <String, Object?>{
      'deviceId': device.id,
      'directory': projectDirectory,
      'supportsRestart': isRestartSupported(enableHotReload, device),
      'launchMode': launchMode.toString(),
      'mode': runner.debuggingOptions.buildInfo.modeName,
    });

    Completer<DebugConnectionInfo>? connectionInfoCompleter;

    if (runner.debuggingEnabled) {
      connectionInfoCompleter = Completer<DebugConnectionInfo>();
      // We don't want to wait for this future to complete and callbacks won't fail.
      // As it just writes to stdout.
      unawaited(
        connectionInfoCompleter.future.then<void>((DebugConnectionInfo info) {
          final Map<String, Object?> params = <String, Object?>{
            // The web vmservice proxy does not have an http address.
            'port': info.httpUri?.port ?? info.wsUri!.port,
            'wsUri': info.wsUri.toString(),
          };
          if (info.baseUri != null) {
            params['baseUri'] = info.baseUri;
          }
          _sendAppEvent(app, 'debugPort', params);
        }),
      );
    }
    final Completer<void> appStartedCompleter = Completer<void>();
    // We don't want to wait for this future to complete, and callbacks won't fail,
    // as it just writes to stdout.
    unawaited(
      appStartedCompleter.future.then<void>((void value) {
        _sendAppEvent(app, 'started');
      }),
    );

    await app._runInZone<void>(this, () async {
      try {
        await runOrAttach(
          connectionInfoCompleter: connectionInfoCompleter,
          appStartedCompleter: appStartedCompleter,
        );
        _sendAppEvent(app, 'stop');
      } on Exception catch (error, trace) {
        _sendAppEvent(app, 'stop', <String, Object?>{
          'error': _toJsonable(error),
          'trace': '$trace',
        });
      } finally {
        // If the full directory is used instead of the path then this causes
        // a TypeError with the ErrorHandlingFileSystem.
        globals.fs.currentDirectory = cwd.path;
        _apps.remove(app);
      }
    });
    return app;
  }

  bool isRestartSupported(bool enableHotReload, Device device) =>
      enableHotReload && device.supportsHotRestart;

  final int _hotReloadDebounceDurationMs = 50;

  Future<OperationResult>? restart(Map<String, Object?> args) async {
    final String? appId = _getStringArg(args, 'appId', required: true);
    final bool fullRestart = _getBoolArg(args, 'fullRestart') ?? false;
    final bool pauseAfterRestart = _getBoolArg(args, 'pause') ?? false;
    final String? restartReason = _getStringArg(args, 'reason');
    final bool debounce = _getBoolArg(args, 'debounce') ?? false;
    // This is an undocumented parameter used for integration tests.
    final int? debounceDurationOverrideMs = _getIntArg(args, 'debounceDurationOverrideMs');

    final AppInstance? app = _getApp(appId);
    if (app == null) {
      throw DaemonException("app '$appId' not found");
    }

    return _queueAndDebounceReloadAction(
      app,
      fullRestart ? OperationType.restart : OperationType.reload,
      debounce,
      debounceDurationOverrideMs,
      () {
        return app.restart(
          fullRestart: fullRestart,
          pause: pauseAfterRestart,
          reason: restartReason,
        );
      },
    )!;
  }

  /// Debounce and queue reload actions.
  ///
  /// Only one reload action will run at a time. Actions requested in quick
  /// succession (within [_hotReloadDebounceDuration]) will be merged together
  /// and all return the same result. If an action is requested after an identical
  /// action has already started, it will be queued and run again once the first
  /// action completes.
  Future<OperationResult>? _queueAndDebounceReloadAction(
    AppInstance app,
    OperationType operationType,
    bool debounce,
    int? debounceDurationOverrideMs,
    Future<OperationResult> Function() action,
  ) {
    final Duration debounceDuration =
        debounce
            ? Duration(milliseconds: debounceDurationOverrideMs ?? _hotReloadDebounceDurationMs)
            : Duration.zero;

    return operationQueue.queueAndDebounce(
      operationType,
      debounceDuration,
      () => app._runInZone<OperationResult>(this, action),
    );
  }

  /// Returns an error, or the service extension result (a map with two fixed
  /// keys, `type` and `method`). The result may have one or more additional keys,
  /// depending on the specific service extension end-point. For example:
  ///
  ///     {
  ///       "value":"android",
  ///       "type":"_extensionType",
  ///       "method":"ext.flutter.platformOverride"
  ///     }
  Future<Map<String, Object?>> callServiceExtension(Map<String, Object?> args) async {
    final String? appId = _getStringArg(args, 'appId', required: true);
    final String methodName = _getStringArg(args, 'methodName')!;
    final Map<String, Object?>? params =
        args['params'] == null ? <String, Object?>{} : castStringKeyedMap(args['params']);

    final AppInstance? app = _getApp(appId);
    if (app == null) {
      throw DaemonException("app '$appId' not found");
    }
    final FlutterDevice device = app.runner.flutterDevices.first;
    final List<FlutterView> views = await device.vmService!.getFlutterViews();
    final Map<String, Object?>? result = await device.vmService!.invokeFlutterExtensionRpcRaw(
      methodName,
      args: params,
      isolateId: views.first.uiIsolate!.id!,
    );
    if (result == null) {
      throw DaemonException('method not available: $methodName');
    }

    if (result.containsKey('error')) {
      // ignore: only_throw_errors
      throw result['error']!;
    }

    return result;
  }

  Future<bool> stop(Map<String, Object?> args) async {
    final String? appId = _getStringArg(args, 'appId', required: true);

    final AppInstance? app = _getApp(appId);
    if (app == null) {
      throw DaemonException("app '$appId' not found");
    }

    return app.stop().then<bool>(
      (void value) => true,
      onError: (Object? error, StackTrace stack) {
        _sendAppEvent(app, 'log', <String, Object?>{'log': '$error', 'error': true});
        app.closeLogger();
        _apps.remove(app);
        return false;
      },
    );
  }

  Future<bool> detach(Map<String, Object?> args) async {
    final String? appId = _getStringArg(args, 'appId', required: true);

    final AppInstance? app = _getApp(appId);
    if (app == null) {
      throw DaemonException("app '$appId' not found");
    }

    return app.detach().then<bool>(
      (void value) => true,
      onError: (Object? error, StackTrace stack) {
        _sendAppEvent(app, 'log', <String, Object?>{'log': '$error', 'error': true});
        app.closeLogger();
        _apps.remove(app);
        return false;
      },
    );
  }

  AppInstance? _getApp(String? id) {
    for (final AppInstance app in _apps) {
      if (app.id == id) {
        return app;
      }
    }
    return null;
  }

  void _sendAppEvent(AppInstance app, String name, [Map<String, Object?>? args]) {
    sendEvent('app.$name', <String, Object?>{'appId': app.id, ...?args});
  }
}

typedef _DeviceEventHandler = void Function(Device device);

/// This domain lets callers list and monitor connected devices.
///
/// It exports a `getDevices()` call, as well as firing `device.added` and
/// `device.removed` events.
class DeviceDomain extends Domain {
  DeviceDomain(Daemon daemon) : super(daemon, 'device') {
    registerHandler('getDevices', getDevices);
    registerHandler('discoverDevices', discoverDevices);
    registerHandler('enable', enable);
    registerHandler('disable', disable);
    registerHandler('forward', forward);
    registerHandler('unforward', unforward);
    registerHandler('supportsRuntimeMode', supportsRuntimeMode);
    registerHandler('uploadApplicationPackage', uploadApplicationPackage);
    registerHandler('logReader.start', startLogReader);
    registerHandler('logReader.stop', stopLogReader);
    registerHandler('startApp', startApp);
    registerHandler('stopApp', stopApp);
    registerHandler('takeScreenshot', takeScreenshot);
    registerHandler('startDartDevelopmentService', startDartDevelopmentService);
    registerHandler('shutdownDartDevelopmentService', shutdownDartDevelopmentService);
    registerHandler('getDiagnostics', getDiagnostics);
    registerHandler('startVMServiceDiscoveryForAttach', startVMServiceDiscoveryForAttach);
    registerHandler('stopVMServiceDiscoveryForAttach', stopVMServiceDiscoveryForAttach);

    // Use the device manager discovery so that client provided device types
    // are usable via the daemon protocol.
    globals.deviceManager!.deviceDiscoverers.forEach(addDeviceDiscoverer);
  }

  /// An incrementing number used to generate unique ids.
  int _id = 0;
  final Map<String, ApplicationPackage?> _applicationPackages = <String, ApplicationPackage?>{};
  final Map<String, DeviceLogReader> _logReaders = <String, DeviceLogReader>{};

  void addDeviceDiscoverer(DeviceDiscovery discoverer) {
    if (!discoverer.supportsPlatform) {
      return;
    }

    if (discoverer is PollingDeviceDiscovery) {
      _discoverers.add(discoverer);
      discoverer.onAdded.listen(_onDeviceEvent('device.added'));
      discoverer.onRemoved.listen(_onDeviceEvent('device.removed'));
    }
  }

  Future<void> _serializeDeviceEvents = Future<void>.value();

  _DeviceEventHandler _onDeviceEvent(String eventName) {
    return (Device device) {
      _serializeDeviceEvents = _serializeDeviceEvents.then<void>((_) async {
        try {
          final Map<String, Object?> response = await _deviceToMap(device);
          sendEvent(eventName, response);
        } on Exception catch (err) {
          globals.printError('$err');
        }
      });
    };
  }

  final List<PollingDeviceDiscovery> _discoverers = <PollingDeviceDiscovery>[];

  /// Return a list of the currently connected devices, with each device
  /// represented as a map of properties (id, name, platform, ...).
  Future<List<Map<String, Object?>>> getDevices([Map<String, Object?>? args]) async {
    return <Map<String, Object?>>[
      for (final PollingDeviceDiscovery discoverer in _discoverers)
        for (final Device device in await discoverer.devices(filter: DeviceDiscoveryFilter()))
          await _deviceToMap(device),
    ];
  }

  /// Return a list of the current devices, discarding existing cache of devices.
  Future<List<Map<String, Object?>>> discoverDevices(Map<String, Object?> args) async {
    final int? timeoutInMilliseconds = _getIntArg(args, 'timeoutInMilliseconds');
    final Duration? timeout =
        timeoutInMilliseconds != null ? Duration(milliseconds: timeoutInMilliseconds) : null;

    // Calling `discoverDevices()` and `_deviceToMap()` in parallel for better performance.
    final List<List<Device>> devicesListList = await Future.wait(<Future<List<Device>>>[
      for (final PollingDeviceDiscovery discoverer in _discoverers)
        discoverer.discoverDevices(timeout: timeout),
    ]);

    final List<Device> devices = <Device>[
      for (final List<Device> devicesList in devicesListList) ...devicesList,
    ];
    return Future.wait(<Future<Map<String, Object?>>>[
      for (final Device device in devices) _deviceToMap(device),
    ]);
  }

  /// Enable device events.
  Future<void> enable(Map<String, Object?> args) async {
    for (final PollingDeviceDiscovery discoverer in _discoverers) {
      discoverer.startPolling();
    }
  }

  /// Disable device events.
  Future<void> disable(Map<String, Object?> args) async {
    for (final PollingDeviceDiscovery discoverer in _discoverers) {
      discoverer.stopPolling();
    }
  }

  /// Forward a host port to a device port.
  Future<Map<String, Object?>> forward(Map<String, Object?> args) async {
    final String? deviceId = _getStringArg(args, 'deviceId', required: true);
    final int devicePort = _getIntArg(args, 'devicePort', required: true)!;
    int? hostPort = _getIntArg(args, 'hostPort');

    final Device? device = await daemon.deviceDomain._getDevice(deviceId);
    if (device == null) {
      throw DaemonException("device '$deviceId' not found");
    }

    hostPort = await device.portForwarder!.forward(devicePort, hostPort: hostPort);

    return <String, Object?>{'hostPort': hostPort};
  }

  /// Removes a forwarded port.
  Future<void> unforward(Map<String, Object?> args) async {
    final String? deviceId = _getStringArg(args, 'deviceId', required: true);
    final int devicePort = _getIntArg(args, 'devicePort', required: true)!;
    final int hostPort = _getIntArg(args, 'hostPort', required: true)!;

    final Device? device = await daemon.deviceDomain._getDevice(deviceId);
    if (device == null) {
      throw DaemonException("device '$deviceId' not found");
    }

    return device.portForwarder!.unforward(ForwardedPort(hostPort, devicePort));
  }

  /// Returns whether a device supports runtime mode.
  Future<bool> supportsRuntimeMode(Map<String, Object?> args) async {
    final String? deviceId = _getStringArg(args, 'deviceId', required: true);
    final Device? device = await daemon.deviceDomain._getDevice(deviceId);
    if (device == null) {
      throw DaemonException("device '$deviceId' not found");
    }
    final String buildMode = _getStringArg(args, 'buildMode', required: true)!;
    return await device.supportsRuntimeMode(BuildMode.fromCliName(buildMode));
  }

  /// Creates an application package from a file in the temp directory.
  Future<String> uploadApplicationPackage(Map<String, Object?> args) async {
    final TargetPlatform targetPlatform = getTargetPlatformForName(
      _getStringArg(args, 'targetPlatform', required: true)!,
    );
    final File applicationBinary = daemon.proxyDomain.tempDirectory.childFile(
      _getStringArg(args, 'applicationBinary', required: true)!,
    );
    final ApplicationPackage? applicationPackage = await ApplicationPackageFactory.instance!
        .getPackageForPlatform(targetPlatform, applicationBinary: applicationBinary);
    final String id = 'application_package_${_id++}';
    _applicationPackages[id] = applicationPackage;
    return id;
  }

  /// Starts the log reader on the device.
  Future<String> startLogReader(Map<String, Object?> args) async {
    final String? deviceId = _getStringArg(args, 'deviceId', required: true);
    final Device? device = await daemon.deviceDomain._getDevice(deviceId);
    if (device == null) {
      throw DaemonException("device '$deviceId' not found");
    }
    final String? applicationPackageId = _getStringArg(args, 'applicationPackageId');
    final ApplicationPackage? applicationPackage =
        applicationPackageId != null ? _applicationPackages[applicationPackageId] : null;
    final String id = '${deviceId}_${_id++}';

    final DeviceLogReader logReader = await device.getLogReader(app: applicationPackage);
    logReader.logLines.listen((String log) => sendEvent('device.logReader.logLines.$id', log));

    _logReaders[id] = logReader;

    return id;
  }

  /// Stops a log reader that was previously started.
  Future<void> stopLogReader(Map<String, Object?> args) async {
    final String? id = _getStringArg(args, 'id', required: true);
    _logReaders.remove(id)?.dispose();
  }

  /// Starts an app on a device.
  Future<Map<String, Object?>> startApp(Map<String, Object?> args) async {
    final String? deviceId = _getStringArg(args, 'deviceId', required: true);
    final Device? device = await daemon.deviceDomain._getDevice(deviceId);
    if (device == null) {
      throw DaemonException("device '$deviceId' not found");
    }
    final String? applicationPackageId = _getStringArg(
      args,
      'applicationPackageId',
      required: true,
    );
    final ApplicationPackage applicationPackage = _applicationPackages[applicationPackageId!]!;

    final LaunchResult result = await device.startApp(
      applicationPackage,
      debuggingOptions: DebuggingOptions.fromJson(
        castStringKeyedMap(args['debuggingOptions'])!,
        // We are using prebuilts, build info does not matter here.
        BuildInfo.dummy,
      ),
      mainPath: _getStringArg(args, 'mainPath'),
      route: _getStringArg(args, 'route'),
      platformArgs: castStringKeyedMap(args['platformArgs']) ?? const <String, Object>{},
      prebuiltApplication: _getBoolArg(args, 'prebuiltApplication') ?? false,
      userIdentifier: _getStringArg(args, 'userIdentifier'),
    );
    return <String, Object?>{
      'started': result.started,
      'vmServiceUri': result.vmServiceUri?.toString(),
      // TODO(bkonyi): remove once clients have migrated to relying on vmServiceUri.
      'observatoryUri': result.vmServiceUri?.toString(),
    };
  }

  /// Stops an app.
  Future<bool> stopApp(Map<String, Object?> args) async {
    final String? deviceId = _getStringArg(args, 'deviceId', required: true);
    final Device? device = await daemon.deviceDomain._getDevice(deviceId);
    if (device == null) {
      throw DaemonException("device '$deviceId' not found");
    }
    final String? applicationPackageId = _getStringArg(args, 'applicationPackageId');
    ApplicationPackage? applicationPackage;
    if (applicationPackageId != null) {
      applicationPackage = _applicationPackages[applicationPackageId];
    }
    return device.stopApp(
      applicationPackage,
      userIdentifier: _getStringArg(args, 'userIdentifier'),
    );
  }

  /// Takes a screenshot.
  Future<String?> takeScreenshot(Map<String, Object?> args) async {
    final String? deviceId = _getStringArg(args, 'deviceId', required: true);
    final Device? device = await daemon.deviceDomain._getDevice(deviceId);
    if (device == null) {
      throw DaemonException("device '$deviceId' not found");
    }
    final String tempFileName = 'screenshot_${_id++}';
    final File tempFile = daemon.proxyDomain.tempDirectory.childFile(tempFileName);
    await device.takeScreenshot(tempFile);
    if (await tempFile.exists()) {
      final String imageBase64 = base64.encode(await tempFile.readAsBytes());
      return imageBase64;
    } else {
      return null;
    }
  }

  /// Starts DDS for the device.
  Future<Map<String, Object?>> startDartDevelopmentService(Map<String, Object?> args) async {
    final String? deviceId = _getStringArg(args, 'deviceId', required: true);
    final bool? disableServiceAuthCodes = _getBoolArg(args, 'disableServiceAuthCodes');
    final String vmServiceUriStr = _getStringArg(args, 'vmServiceUri', required: true)!;
    final bool enableDevTools = _getBoolArg(args, 'enableDevTools') ?? false;
    final String? devToolsServerAddressStr = _getStringArg(args, 'devToolsServerAddress');

    final Device? device = await daemon.deviceDomain._getDevice(deviceId);
    if (device == null) {
      throw DaemonException("device '$deviceId' not found");
    }

    Uri? devToolsServerAddress;
    if (devToolsServerAddressStr != null) {
      devToolsServerAddress = Uri.parse(devToolsServerAddressStr);
    }

    await device.dds.startDartDevelopmentService(
      Uri.parse(vmServiceUriStr),
      disableServiceAuthCodes: disableServiceAuthCodes,
      enableDevTools: enableDevTools,
      devToolsServerAddress: devToolsServerAddress,
    );
    unawaited(device.dds.done.whenComplete(() => sendEvent('device.dds.done.$deviceId')));
    return <String, Object?>{'ddsUri': device.dds.uri?.toString()};
  }

  /// Starts DDS for the device.
  Future<void> shutdownDartDevelopmentService(Map<String, Object?> args) async {
    final String? deviceId = _getStringArg(args, 'deviceId', required: true);

    final Device? device = await daemon.deviceDomain._getDevice(deviceId);
    if (device == null) {
      throw DaemonException("device '$deviceId' not found");
    }

    device.dds.shutdown();
  }

  @override
  Future<void> dispose() {
    for (final PollingDeviceDiscovery discoverer in _discoverers) {
      discoverer.dispose();
    }
    return Future<void>.value();
  }

  /// Return the connected device matching the deviceId field in the args.
  Future<Device?> _getDevice(String? deviceId) async {
    for (final PollingDeviceDiscovery discoverer in _discoverers) {
      final List<Device> devices = await discoverer.devices(filter: DeviceDiscoveryFilter());
      Device? device;
      for (final Device localDevice in devices) {
        if (localDevice.id == deviceId) {
          device = localDevice;
        }
      }
      if (device != null) {
        return device;
      }
    }
    return null;
  }

  /// Gets a list of diagnostic messages pertaining to issues with any connected
  /// devices.
  Future<List<String>> getDiagnostics(Map<String, Object?> args) async {
    // Call `getDiagnostics()` in parallel to improve performance.
    final List<List<String>> diagnosticsLists = await Future.wait(<Future<List<String>>>[
      for (final PollingDeviceDiscovery discoverer in _discoverers) discoverer.getDiagnostics(),
    ]);

    return <String>[for (final List<String> diagnostics in diagnosticsLists) ...diagnostics];
  }

  final Map<String, StreamSubscription<Uri>> _vmServiceDiscoverySubscriptions =
      <String, StreamSubscription<Uri>>{};

  Future<String> startVMServiceDiscoveryForAttach(Map<String, Object?> args) async {
    final String? deviceId = _getStringArg(args, 'deviceId', required: true);
    final String? appId = _getStringArg(args, 'appId');
    final String? fuchsiaModule = _getStringArg(args, 'fuchsiaModule');
    final int? filterDevicePort = _getIntArg(args, 'filterDevicePort');
    final bool? ipv6 = _getBoolArg(args, 'ipv6');

    final Device? device = await daemon.deviceDomain._getDevice(deviceId);
    if (device == null) {
      throw DaemonException("device '$deviceId' not found");
    }

    final String id = '${_id++}';

    final VMServiceDiscoveryForAttach discovery = device.getVMServiceDiscoveryForAttach(
      appId: appId,
      fuchsiaModule: fuchsiaModule,
      filterDevicePort: filterDevicePort,
      ipv6: ipv6 ?? false,
      logger: globals.logger,
    );
    _vmServiceDiscoverySubscriptions[id] = discovery.uris.listen(
      (Uri uri) => sendEvent('device.VMServiceDiscoveryForAttach.$id', uri.toString()),
    );

    return id;
  }

  Future<void> stopVMServiceDiscoveryForAttach(Map<String, Object?> args) async {
    final String? id = _getStringArg(args, 'id', required: true);
    await _vmServiceDiscoverySubscriptions.remove(id)?.cancel();
  }
}

class DevToolsDomain extends Domain {
  DevToolsDomain(Daemon daemon) : super(daemon, 'devtools') {
    registerHandler('serve', serve);
  }

  DevtoolsLauncher? _devtoolsLauncher;

  Future<Map<String, Object?>> serve([Map<String, Object?>? args]) async {
    _devtoolsLauncher ??= DevtoolsLauncher.instance;
    final DevToolsServerAddress? server = await _devtoolsLauncher?.serve();
    return <String, Object?>{'host': server?.host, 'port': server?.port};
  }

  @override
  Future<void> dispose() async {
    await _devtoolsLauncher?.close();
  }
}

Future<Map<String, Object?>> _deviceToMap(Device device) async {
  return <String, Object?>{
    'id': device.id,
    'name': device.name,
    'platform': getNameForTargetPlatform(await device.targetPlatform),
    'emulator': await device.isLocalEmulator,
    'category': device.category?.toString(),
    'platformType': device.platformType?.toString(),
    'ephemeral': device.ephemeral,
    'emulatorId': await device.emulatorId,
    'sdk': await device.sdkNameAndVersion,
    'isConnected': device.isConnected,
    'connectionInterface': getNameForDeviceConnectionInterface(device.connectionInterface),
    'capabilities': <String, Object>{
      'hotReload': device.supportsHotReload,
      'hotRestart': device.supportsHotRestart,
      'screenshot': device.supportsScreenshot,
      'fastStart': device.supportsFastStart,
      'flutterExit': device.supportsFlutterExit,
      'hardwareRendering': await device.supportsHardwareRendering,
      'startPaused': device.supportsStartPaused,
    },
  };
}

Map<String, Object?> _emulatorToMap(Emulator emulator) {
  return <String, Object?>{
    'id': emulator.id,
    'name': emulator.name,
    'category': emulator.category.toString(),
    'platformType': emulator.platformType.toString(),
  };
}

Map<String, Object?> _operationResultToMap(OperationResult result) {
  return <String, Object?>{'code': result.code, 'message': result.message};
}

Object? _toJsonable(Object? obj) {
  return switch (obj) {
    String() || int() || bool() || Map<Object?, Object?>() || List<Object?>() || null => obj,
    OperationResult() => _operationResultToMap(obj),
    ToolExit() => obj.message,
    _ => obj.toString(),
  };
}

class NotifyingLogger extends DelegatingLogger {
  NotifyingLogger({required this.verbose, required Logger parent, this.notifyVerbose = false})
    : super(parent) {
    _messageController = StreamController<LogMessage>.broadcast(onListen: _onListen);
  }

  final bool verbose;
  final List<LogMessage> messageBuffer = <LogMessage>[];
  late StreamController<LogMessage> _messageController;

  bool notifyVerbose = false;

  void _onListen() {
    if (messageBuffer.isNotEmpty) {
      messageBuffer.forEach(_messageController.add);
      messageBuffer.clear();
    }
  }

  Stream<LogMessage> get onMessage => _messageController.stream;

  @override
  void printError(
    String message, {
    StackTrace? stackTrace,
    bool? emphasis = false,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    _sendMessage(LogMessage('error', message, stackTrace));
  }

  @override
  void printWarning(
    String message, {
    bool? emphasis = false,
    TerminalColor? color,
    int? indent,
    int? hangingIndent,
    bool? wrap,
    bool fatal = true,
  }) {
    _sendMessage(LogMessage('warning', message));
  }

  @override
  void printStatus(
    String message, {
    bool? emphasis = false,
    TerminalColor? color,
    bool? newline = true,
    int? indent,
    int? hangingIndent,
    bool? wrap,
  }) {
    _sendMessage(LogMessage('status', message));
  }

  @override
  void printBox(String message, {String? title}) {
    _sendMessage(LogMessage('status', title == null ? message : '$title: $message'));
  }

  @override
  void printTrace(String message) {
    if (notifyVerbose) {
      _sendMessage(LogMessage('trace', message));
      return;
    }
    if (!verbose) {
      return;
    }
    super.printError(message);
  }

  @override
  Status startProgress(
    String message, {
    Duration? timeout,
    String? progressId,
    bool multilineOutput = false,
    bool includeTiming = true,
    int progressIndicatorPadding = kDefaultStatusPadding,
  }) {
    assert(timeout != null);
    printStatus(message);
    return SilentStatus(stopwatch: Stopwatch());
  }

  void _sendMessage(LogMessage logMessage) {
    if (_messageController.hasListener) {
      return _messageController.add(logMessage);
    }
    messageBuffer.add(logMessage);
  }

  void dispose() {
    _messageController.close();
  }

  @override
  void sendEvent(String name, [Map<String, Object?>? args]) {}

  @override
  bool get supportsColor => false;

  @override
  bool get hasTerminal => false;

  // This method is only relevant for terminals.
  @override
  void clear() {}
}

/// A running application, started by this daemon.
class AppInstance {
  AppInstance(
    this.id, {
    required this.runner,
    this.logToStdout = false,
    required AppRunLogger logger,
  }) : _logger = logger;

  final String id;
  final ResidentRunner runner;
  final bool logToStdout;
  final AppRunLogger _logger;

  Future<OperationResult> restart({bool fullRestart = false, bool pause = false, String? reason}) {
    return runner.restart(fullRestart: fullRestart, pause: pause, reason: reason);
  }

  Future<void> stop() => runner.exit();
  Future<void> detach() => runner.detach();

  void closeLogger() {
    _logger.close();
  }

  Future<T> _runInZone<T>(AppDomain domain, FutureOr<T> Function() method) async {
    return method();
  }
}

/// This domain responds to methods like [getEmulators] and [launch].
class EmulatorDomain extends Domain {
  EmulatorDomain(Daemon daemon) : super(daemon, 'emulator') {
    registerHandler('getEmulators', getEmulators);
    registerHandler('launch', launch);
    registerHandler('create', create);
  }

  EmulatorManager emulators = EmulatorManager(
    fileSystem: globals.fs,
    logger: globals.logger,
    java: globals.java,
    androidSdk: globals.androidSdk,
    processManager: globals.processManager,
    androidWorkflow: androidWorkflow!,
  );

  Future<List<Map<String, Object?>>> getEmulators([Map<String, Object?>? args]) async {
    final List<Emulator> list = await emulators.getAllAvailableEmulators();
    return list.map<Map<String, Object?>>(_emulatorToMap).toList();
  }

  Future<void> launch(Map<String, Object?> args) async {
    final String emulatorId = _getStringArg(args, 'emulatorId', required: true)!;
    final bool coldBoot = _getBoolArg(args, 'coldBoot') ?? false;
    final List<Emulator> matches = await emulators.getEmulatorsMatching(emulatorId);
    if (matches.isEmpty) {
      throw DaemonException("emulator '$emulatorId' not found");
    } else if (matches.length > 1) {
      throw DaemonException("multiple emulators match '$emulatorId'");
    } else {
      await matches.first.launch(coldBoot: coldBoot);
    }
  }

  Future<Map<String, Object?>> create(Map<String, Object?> args) async {
    final String? name = _getStringArg(args, 'name');
    final CreateEmulatorResult res = await emulators.createEmulator(name: name);
    return <String, Object?>{
      'success': res.success,
      'emulatorName': res.emulatorName,
      'error': res.error,
    };
  }
}

class ProxyDomain extends Domain {
  ProxyDomain(Daemon daemon, {required FileTransfer fileTransfer})
    : _fileTransfer = fileTransfer,
      super(daemon, 'proxy') {
    registerHandlerWithBinary('writeTempFile', writeTempFile);
    registerHandler('calculateFileHashes', calculateFileHashes);
    registerHandlerWithBinary('updateFile', updateFile);
    registerHandler('connect', connect);
    registerHandler('disconnect', disconnect);
    registerHandlerWithBinary('write', write);
  }

  final FileTransfer _fileTransfer;

  final Map<String, Socket> _forwardedConnections = <String, Socket>{};
  int _id = 0;

  /// Writes to a file in a local temporary directory.
  Future<void> writeTempFile(Map<String, Object?> args, Stream<List<int>>? binary) async {
    final String path = _getStringArg(args, 'path', required: true)!;
    final File file = tempDirectory.childFile(path);
    await file.parent.create(recursive: true);
    await file.openWrite().addStream(binary!);
  }

  /// Calculate rolling hashes for a file in the local temporary directory.
  Future<Map<String, Object?>?> calculateFileHashes(Map<String, Object?> args) async {
    final String path = _getStringArg(args, 'path', required: true)!;
    final bool cacheResult = _getBoolArg(args, 'cacheResult') ?? false;
    final File file = tempDirectory.childFile(path);
    if (!await file.exists()) {
      return null;
    }
    final File hashFile = file.parent.childFile('${file.basename}.hashes');
    if (hashFile.existsSync() && hashFile.statSync().modified.isAfter(file.statSync().modified)) {
      // If the cached hash file is newer than the file, assume that the cached
      // is up to date. Return the cached result directly.
      final String cachedJson = await hashFile.readAsString();
      return json.decode(cachedJson) as Map<String, Object?>;
    }
    final BlockHashes result = await _fileTransfer.calculateBlockHashesOfFile(file);
    final Map<String, Object?> resultObject = result.toJson();

    if (cacheResult) {
      await hashFile.writeAsString(json.encode(resultObject));
    }

    return resultObject;
  }

  Future<bool?> updateFile(Map<String, Object?> args, Stream<List<int>>? binary) async {
    final String path = _getStringArg(args, 'path', required: true)!;
    final File file = tempDirectory.childFile(path);
    if (!await file.exists()) {
      return null;
    }
    final List<Map<String, Object?>> deltaJson =
        (args['delta']! as List<Object?>).cast<Map<String, Object?>>();
    final List<FileDeltaBlock> delta = FileDeltaBlock.fromJsonList(deltaJson);
    final bool result = await _fileTransfer.rebuildFile(file, delta, binary!);
    return result;
  }

  /// Opens a connection to a local port, and returns the connection id.
  Future<String> connect(Map<String, Object?> args) async {
    final int targetPort = _getIntArg(args, 'port', required: true)!;
    final String id = 'portForwarder_${targetPort}_${_id++}';

    Socket? socket;

    try {
      socket = await Socket.connect(InternetAddress.loopbackIPv4, targetPort);
    } on SocketException {
      globals.logger.printTrace('Connecting to localhost:$targetPort failed with IPv4');
    }

    try {
      // If connecting to IPv4 loopback interface fails, try IPv6.
      socket ??= await Socket.connect(InternetAddress.loopbackIPv6, targetPort);
    } on SocketException {
      globals.logger.printError('Connecting to localhost:$targetPort failed');
    }

    if (socket == null) {
      throw Exception('Failed to connect to the port');
    }

    _forwardedConnections[id] = socket;
    debounceDataStream(socket).listen(
      (List<int> data) {
        sendEvent('proxy.data.$id', null, data);
      },
      onError: (Object error, StackTrace stackTrace) {
        // Socket error, probably disconnected.
        globals.logger.printTrace('Socket error: $error, $stackTrace');
      },
    );

    unawaited(
      socket.done
          .then<Object?>(
            (Object? obj) => obj,
            onError: (Object error, StackTrace stackTrace) {
              // Socket error, probably disconnected.
              globals.logger.printTrace('Socket error: $error, $stackTrace');
            },
          )
          .then((Object? _) {
            sendEvent('proxy.disconnected.$id');
          }),
    );
    return id;
  }

  /// Disconnects from a previously established connection.
  Future<bool> disconnect(Map<String, Object?> args) async {
    final String? id = _getStringArg(args, 'id', required: true);
    if (_forwardedConnections.containsKey(id)) {
      await _forwardedConnections.remove(id)?.close();
      return true;
    }
    return false;
  }

  /// Writes to a previously established connection.
  Future<bool> write(Map<String, Object?> args, Stream<List<int>>? binary) async {
    final String? id = _getStringArg(args, 'id', required: true);
    if (_forwardedConnections.containsKey(id)) {
      final StreamSubscription<List<int>> subscription = binary!.listen(
        _forwardedConnections[id!]!.add,
      );
      await subscription.asFuture<void>();
      await subscription.cancel();
      return true;
    }
    return false;
  }

  @override
  Future<void> dispose() async {
    for (final Socket connection in _forwardedConnections.values) {
      connection.destroy();
    }
    // We deliberately not clean up the tempDirectory here. The application package files that
    // are transferred into this directory through ProxiedDevices are left in the directory
    // to be reused on any subsequent runs.
  }

  Directory? _tempDirectory;
  Directory get tempDirectory =>
      _tempDirectory ??= globals.fs.systemTempDirectory.childDirectory('flutter_tool_daemon')
        ..createSync();
}

/// A [Logger] which sends log messages to a listening daemon client.
///
/// This class can either:
///   1) Send stdout messages and progress events to the client IDE
///   1) Log messages to stdout and send progress events to the client IDE
//
// TODO(devoncarew): To simplify this code a bit, we could choose to specialize
// this class into two, one for each of the above use cases.
class AppRunLogger extends DelegatingLogger {
  AppRunLogger({required Logger parent}) : super(parent);

  AppDomain? domain;
  late AppInstance app;
  int _nextProgressId = 0;

  Status? _status;

  @override
  Status startProgress(
    String message, {
    Duration? timeout,
    String? progressId,
    bool multilineOutput = false,
    bool includeTiming = true,
    int progressIndicatorPadding = kDefaultStatusPadding,
  }) {
    final int id = _nextProgressId++;

    _sendProgressEvent(eventId: id.toString(), eventType: progressId, message: message);

    _status = SilentStatus(
      onFinish: () {
        _status = null;
        _sendProgressEvent(eventId: id.toString(), eventType: progressId, finished: true);
      },
      stopwatch: Stopwatch(),
    )..start();
    return _status!;
  }

  void close() {
    domain = null;
  }

  void _sendProgressEvent({
    required String eventId,
    required String? eventType,
    bool finished = false,
    String? message,
  }) {
    if (domain == null) {
      // If we're sending progress events before an app has started, send the
      // progress messages as plain status messages.
      if (message != null) {
        printStatus(message);
      }
    } else {
      final Map<String, Object?> event = <String, Object?>{
        'id': eventId,
        'progressId': eventType,
        if (message != null) 'message': message,
        'finished': finished,
      };

      domain!._sendAppEvent(app, 'progress', event);
    }
  }

  @override
  void sendEvent(String name, [Map<String, Object?>? args, List<int>? binary]) {
    if (domain == null) {
      printStatus('event sent after app closed: $name');
    } else {
      domain!.sendEvent(name, args, binary);
    }
  }

  @override
  bool get supportsColor => false;

  @override
  bool get hasTerminal => false;

  // This method is only relevant for terminals.
  @override
  void clear() {}
}

class LogMessage {
  LogMessage(this.level, this.message, [this.stackTrace]);

  final String level;
  final String message;
  final StackTrace? stackTrace;
}

/// The method by which the Flutter app was launched.
enum LaunchMode {
  run._('run'),
  attach._('attach');

  const LaunchMode._(this._value);

  final String _value;

  @override
  String toString() => _value;
}

enum OperationType { reload, restart }

/// A queue that debounces operations for a period and merges operations of the same type.
/// Only one action (or any type) will run at a time. Actions of the same type requested
/// in quick succession will be merged together and all return the same result. If an action
/// is requested after an identical action has already started, it will be queued
/// and run again once the first action completes.
class DebounceOperationQueue<T, K> {
  final Map<K, RestartableTimer> _debounceTimers = <K, RestartableTimer>{};
  final Map<K, Future<T>> _operationQueue = <K, Future<T>>{};
  Future<void>? _inProgressAction;

  Future<T> queueAndDebounce(
    K operationType,
    Duration debounceDuration,
    Future<T> Function() action,
  ) {
    // If there is already an operation of this type waiting to run, reset its
    // debounce timer and return its future.
    if (_operationQueue[operationType] != null) {
      _debounceTimers[operationType]?.reset();
      return _operationQueue[operationType]!;
    }

    // Otherwise, put one in the queue with a timer.
    final Completer<T> completer = Completer<T>();
    _operationQueue[operationType] = completer.future;
    _debounceTimers[operationType] = RestartableTimer(debounceDuration, () async {
      // Remove us from the queue so we can't be reset now we've started.
      unawaited(_operationQueue.remove(operationType));
      _debounceTimers.remove(operationType);

      // No operations should be allowed to run concurrently even if they're
      // different types.
      while (_inProgressAction != null) {
        await _inProgressAction;
      }

      _inProgressAction = action()
          .then(completer.complete, onError: completer.completeError)
          .whenComplete(() => _inProgressAction = null);
    });

    return completer.future;
  }
}

/// Specialized exception for returning errors to the daemon client.
class DaemonException implements Exception {
  DaemonException(this.message);

  final String message;

  @override
  String toString() => message;
}
