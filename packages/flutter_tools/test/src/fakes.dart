// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io show IOSink, ProcessSignal, Stdout, StdoutException;

import 'package:dds/dds_launcher.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/android/android_studio.dart';
import 'package:flutter_tools/src/android/java.dart';
import 'package:flutter_tools/src/base/bot_detector.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/time.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/convert.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/ios/plist_parser.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/version.dart';
import 'package:test/fake.dart';

/// Environment with DYLD_LIBRARY_PATH=/path/to/libraries
class FakeDyldEnvironmentArtifact extends ArtifactSet {
  FakeDyldEnvironmentArtifact() : super(DevelopmentArtifact.iOS);
  @override
  Map<String, String> get environment => <String, String>{
    'DYLD_LIBRARY_PATH': '/path/to/libraries',
  };

  @override
  Future<bool> isUpToDate(FileSystem fileSystem) => Future<bool>.value(true);

  @override
  String get name => 'fake';

  @override
  Future<void> update(
    ArtifactUpdater artifactUpdater,
    Logger logger,
    FileSystem fileSystem,
    OperatingSystemUtils operatingSystemUtils, {
    bool offline = false,
  }) async {}
}

/// A fake process implementation which can be provided all necessary values.
class FakeProcess implements Process {
  FakeProcess({
    this.pid = 1,
    Future<int>? exitCode,
    IOSink? stdin,
    this.stdout = const Stream<List<int>>.empty(),
    this.stderr = const Stream<List<int>>.empty(),
  }) : exitCode = exitCode ?? Future<int>.value(0),
       stdin = stdin ?? MemoryIOSink();

  @override
  final int pid;

  @override
  final Future<int> exitCode;

  @override
  final io.IOSink stdin;

  @override
  final Stream<List<int>> stdout;

  @override
  final Stream<List<int>> stderr;

  @override
  bool kill([io.ProcessSignal signal = io.ProcessSignal.sigterm]) {
    return true;
  }
}

/// An IOSink that completes a future with the first line written to it.
class CompleterIOSink extends MemoryIOSink {
  CompleterIOSink({this.throwOnAdd = false});

  final bool throwOnAdd;

  final Completer<List<int>> _completer = Completer<List<int>>();

  Future<List<int>> get future => _completer.future;

  @override
  void add(List<int> data) {
    if (!_completer.isCompleted) {
      // When throwOnAdd is true, complete with empty so any expected output
      // doesn't appear.
      _completer.complete(throwOnAdd ? <int>[] : data);
    }
    if (throwOnAdd) {
      throw Exception('CompleterIOSink Error');
    }
    super.add(data);
  }
}

/// An IOSink that collects whatever is written to it.
class MemoryIOSink implements IOSink {
  @override
  Encoding encoding = utf8;

  final List<List<int>> writes = <List<int>>[];

  @override
  void add(List<int> data) {
    writes.add(data);
  }

  @override
  Future<void> addStream(Stream<List<int>> stream) {
    final Completer<void> completer = Completer<void>();
    late StreamSubscription<List<int>> sub;
    sub = stream.listen(
      (List<int> data) {
        try {
          add(data);
          // Catches all exceptions to propagate them to the completer.
        } catch (err, stack) {
          sub.cancel();
          completer.completeError(err, stack);
        }
      },
      onError: completer.completeError,
      onDone: completer.complete,
      cancelOnError: true,
    );
    return completer.future;
  }

  @override
  void writeCharCode(int charCode) {
    add(<int>[charCode]);
  }

  @override
  void write(Object? obj) {
    add(encoding.encode('$obj'));
  }

  @override
  void writeln([Object? obj = '']) {
    add(encoding.encode('$obj\n'));
  }

  @override
  void writeAll(Iterable<dynamic> objects, [String separator = '']) {
    bool addSeparator = false;
    for (final dynamic object in objects) {
      if (addSeparator) {
        write(separator);
      }
      write(object);
      addSeparator = true;
    }
  }

  @override
  void addError(dynamic error, [StackTrace? stackTrace]) {
    throw UnimplementedError();
  }

  @override
  Future<void> get done => close();

  @override
  Future<void> close() async {}

  @override
  Future<void> flush() async {}

  void clear() {
    writes.clear();
  }

  String getAndClear() {
    final String result = utf8.decode(writes.expand((List<int> l) => l).toList());
    clear();
    return result;
  }
}

class MemoryStdout extends MemoryIOSink implements io.Stdout {
  @override
  bool get hasTerminal => _hasTerminal;
  set hasTerminal(bool value) {
    _hasTerminal = value;
  }

  bool _hasTerminal = true;

  @override
  String get lineTerminator => '\n';
  @override
  set lineTerminator(String value) {
    throw UnimplementedError('Setting the line terminator is not supported');
  }

  @override
  io.IOSink get nonBlocking => this;

  @override
  bool get supportsAnsiEscapes => _supportsAnsiEscapes;
  set supportsAnsiEscapes(bool value) {
    _supportsAnsiEscapes = value;
  }

  bool _supportsAnsiEscapes = true;

  @override
  int get terminalColumns {
    if (_terminalColumns != null) {
      return _terminalColumns!;
    }
    throw const io.StdoutException('unspecified mock value');
  }

  set terminalColumns(int value) => _terminalColumns = value;
  int? _terminalColumns;

  @override
  int get terminalLines {
    if (_terminalLines != null) {
      return _terminalLines!;
    }
    throw const io.StdoutException('unspecified mock value');
  }

  set terminalLines(int value) => _terminalLines = value;
  int? _terminalLines;
}

/// A Stdio that collects stdout and supports simulated stdin.
class FakeStdio extends Stdio {
  final MemoryStdout _stdout = MemoryStdout()..terminalColumns = 80;
  final MemoryIOSink _stderr = MemoryIOSink();
  final FakeStdin _stdin = FakeStdin();

  @override
  MemoryStdout get stdout => _stdout;

  @override
  MemoryIOSink get stderr => _stderr;

  @override
  Stream<List<int>> get stdin => _stdin;

  void simulateStdin(String line) {
    _stdin.controller.add(utf8.encode('$line\n'));
  }

  @override
  bool hasTerminal = false;

  List<String> get writtenToStdout => _stdout.writes.map<String>(_stdout.encoding.decode).toList();
  List<String> get writtenToStderr => _stderr.writes.map<String>(_stderr.encoding.decode).toList();
}

class FakeStdin extends Fake implements Stdin {
  final StreamController<List<int>> controller = StreamController<List<int>>();

  void Function(bool mode)? echoModeCallback;

  bool _echoMode = true;

  @override
  bool get echoMode => _echoMode;

  @override
  set echoMode(bool mode) {
    _echoMode = mode;
    if (echoModeCallback != null) {
      echoModeCallback!(mode);
    }
  }

  @override
  bool lineMode = true;

  @override
  bool hasTerminal = false;

  @override
  Stream<S> transform<S>(StreamTransformer<List<int>, S> transformer) {
    return controller.stream.transform(transformer);
  }

  @override
  StreamSubscription<List<int>> listen(
    void Function(List<int> event)? onData, {
    Function? onError,
    void Function()? onDone,
    bool? cancelOnError,
  }) {
    return controller.stream.listen(
      onData,
      onError: onError,
      onDone: onDone,
      cancelOnError: cancelOnError,
    );
  }
}

class FakePlistParser implements PlistParser {
  FakePlistParser([Map<String, Object>? underlyingValues])
    : _underlyingValues = underlyingValues ?? <String, Object>{};

  final Map<String, Object> _underlyingValues;

  void setProperty(String key, Object value) {
    _underlyingValues[key] = value;
  }

  @override
  String? plistXmlContent(String plistFilePath) => throw UnimplementedError();

  @override
  String? plistJsonContent(String filePath, {bool sorted = false}) {
    throw UnimplementedError();
  }

  @override
  Map<String, Object> parseFile(String plistFilePath) {
    return _underlyingValues;
  }

  @override
  T? getValueFromFile<T>(String plistFilePath, String key) {
    return _underlyingValues[key] as T?;
  }

  @override
  bool replaceKey(String plistFilePath, {required String key, String? value}) {
    if (value == null) {
      _underlyingValues.remove(key);
      return true;
    }
    setProperty(key, value);
    return true;
  }
}

class FakeBotDetector implements BotDetector {
  const FakeBotDetector(bool isRunningOnBot) : _isRunningOnBot = isRunningOnBot;

  @override
  Future<bool> get isRunningOnBot async => _isRunningOnBot;

  final bool _isRunningOnBot;
}

class FakeFlutterVersion implements FlutterVersion {
  FakeFlutterVersion({
    this.branch = 'master',
    this.dartSdkVersion = '12',
    this.devToolsVersion = '2.8.0',
    this.engineRevision = 'abcdefghijklmnopqrstuvwxyz',
    this.engineRevisionShort = 'abcde',
    this.engineAge = '0 hours ago',
    this.engineCommitDate = '12/01/01',
    this.repositoryUrl = 'https://github.com/flutter/flutter.git',
    this.frameworkVersion = '0.0.0',
    this.frameworkRevision = '11111111111111111111',
    this.frameworkRevisionShort = '11111',
    this.frameworkAge = '0 hours ago',
    this.frameworkCommitDate = '12/01/01',
    this.gitTagVersion = const GitTagVersion.unknown(),
    this.flutterRoot = '/path/to/flutter',
    this.nextFlutterVersion,
  });

  final String branch;

  bool get didFetchTagsAndUpdate => _didFetchTagsAndUpdate;
  bool _didFetchTagsAndUpdate = false;

  /// Will be returned by [fetchTagsAndGetVersion] if not null.
  final FlutterVersion? nextFlutterVersion;

  @override
  FlutterVersion fetchTagsAndGetVersion({SystemClock clock = const SystemClock()}) {
    _didFetchTagsAndUpdate = true;
    return nextFlutterVersion ?? this;
  }

  bool get didCheckFlutterVersionFreshness => _didCheckFlutterVersionFreshness;
  bool _didCheckFlutterVersionFreshness = false;

  @override
  String get channel {
    if (kOfficialChannels.contains(branch) || kObsoleteBranches.containsKey(branch)) {
      return branch;
    }
    return kUserBranch;
  }

  @override
  final String flutterRoot;

  @override
  final String devToolsVersion;

  @override
  final String dartSdkVersion;

  @override
  final String engineRevision;

  @override
  final String engineRevisionShort;

  @override
  final String? engineCommitDate;

  @override
  final String engineAge;

  @override
  final String? repositoryUrl;

  @override
  final String frameworkVersion;

  @override
  final String frameworkRevision;

  @override
  final String frameworkRevisionShort;

  @override
  final String frameworkAge;

  @override
  final String frameworkCommitDate;

  @override
  final GitTagVersion gitTagVersion;

  @override
  FileSystem get fs => throw UnimplementedError('FakeFlutterVersion.fs is not implemented');

  @override
  Future<void> checkFlutterVersionFreshness() async {
    _didCheckFlutterVersionFreshness = true;
  }

  @override
  Future<void> ensureVersionFile() async {}

  @override
  String getBranchName({bool redactUnknownBranches = false}) {
    if (!redactUnknownBranches ||
        kOfficialChannels.contains(branch) ||
        kObsoleteBranches.containsKey(branch)) {
      return branch;
    }
    return kUserBranch;
  }

  @override
  String getVersionString({bool redactUnknownBranches = false}) {
    return '${getBranchName(redactUnknownBranches: redactUnknownBranches)}/$frameworkRevision';
  }

  @override
  Map<String, Object> toJson() {
    return <String, Object>{};
  }
}

// A test implementation of [FeatureFlags] that allows enabling without reading
// config. If not otherwise specified, all values default to false.
class TestFeatureFlags implements FeatureFlags {
  TestFeatureFlags({
    this.isLinuxEnabled = false,
    this.isMacOSEnabled = false,
    this.isWebEnabled = false,
    this.isWindowsEnabled = false,
    this.isAndroidEnabled = true,
    this.isIOSEnabled = true,
    this.isFuchsiaEnabled = false,
    this.areCustomDevicesEnabled = false,
    this.isCliAnimationEnabled = true,
    this.isNativeAssetsEnabled = false,
    this.isSwiftPackageManagerEnabled = false,
    this.isExplicitPackageDependenciesEnabled = false,
  });

  @override
  final bool isLinuxEnabled;

  @override
  final bool isMacOSEnabled;

  @override
  final bool isWebEnabled;

  @override
  final bool isWindowsEnabled;

  @override
  final bool isAndroidEnabled;

  @override
  final bool isIOSEnabled;

  @override
  final bool isFuchsiaEnabled;

  @override
  final bool areCustomDevicesEnabled;

  @override
  final bool isCliAnimationEnabled;

  @override
  final bool isNativeAssetsEnabled;

  @override
  final bool isSwiftPackageManagerEnabled;

  @override
  final bool isExplicitPackageDependenciesEnabled;

  @override
  bool isEnabled(Feature feature) {
    return switch (feature) {
      flutterWebFeature => isWebEnabled,
      flutterLinuxDesktopFeature => isLinuxEnabled,
      flutterMacOSDesktopFeature => isMacOSEnabled,
      flutterWindowsDesktopFeature => isWindowsEnabled,
      flutterAndroidFeature => isAndroidEnabled,
      flutterIOSFeature => isIOSEnabled,
      flutterFuchsiaFeature => isFuchsiaEnabled,
      flutterCustomDevicesFeature => areCustomDevicesEnabled,
      cliAnimation => isCliAnimationEnabled,
      nativeAssets => isNativeAssetsEnabled,
      explicitPackageDependencies => isExplicitPackageDependenciesEnabled,
      _ => false,
    };
  }
}

class FakeOperatingSystemUtils extends Fake implements OperatingSystemUtils {
  FakeOperatingSystemUtils({this.hostPlatform = HostPlatform.linux_x64});

  final List<List<String>> chmods = <List<String>>[];

  @override
  void makeExecutable(File file) {}

  @override
  HostPlatform hostPlatform = HostPlatform.linux_x64;

  @override
  void chmod(FileSystemEntity entity, String mode) {
    chmods.add(<String>[entity.path, mode]);
  }

  @override
  File? which(String execName) => null;

  @override
  List<File> whichAll(String execName) => <File>[];

  @override
  int? getDirectorySize(Directory directory) => 10000000; // 10 MB / 9.5 MiB

  @override
  void unzip(File file, Directory targetDirectory) {}

  @override
  void unpack(File gzippedTarFile, Directory targetDirectory) {}

  @override
  Stream<List<int>> gzipLevel1Stream(Stream<List<int>> stream) => stream;

  @override
  String get name => 'fake OS name and version';

  @override
  String get pathVarSeparator => ';';

  @override
  Future<int> findFreePort({bool ipv6 = false}) async => 12345;
}

class FakeStopwatch implements Stopwatch {
  @override
  bool get isRunning => _isRunning;
  bool _isRunning = false;

  @override
  void start() => _isRunning = true;

  @override
  void stop() => _isRunning = false;

  @override
  Duration elapsed = Duration.zero;

  @override
  int get elapsedMicroseconds => elapsed.inMicroseconds;

  @override
  int get elapsedMilliseconds => elapsed.inMilliseconds;

  @override
  int get elapsedTicks => elapsed.inMilliseconds;

  @override
  int get frequency => 1000;

  @override
  void reset() {
    _isRunning = false;
    elapsed = Duration.zero;
  }

  @override
  String toString() => '$runtimeType $elapsed $isRunning';
}

class FakeStopwatchFactory implements StopwatchFactory {
  FakeStopwatchFactory({Stopwatch? stopwatch, Map<String, Stopwatch>? stopwatches})
    : stopwatches = <String, Stopwatch>{
        if (stopwatches != null) ...stopwatches,
        if (stopwatch != null) '': stopwatch,
      };

  Map<String, Stopwatch> stopwatches;

  @override
  Stopwatch createStopwatch([String name = '']) {
    return stopwatches[name] ?? FakeStopwatch();
  }
}

class FakeFlutterProjectFactory implements FlutterProjectFactory {
  @override
  FlutterProject fromDirectory(Directory directory) {
    return FlutterProject.fromDirectoryTest(directory);
  }

  @override
  Map<String, FlutterProject> get projects => throw UnimplementedError();
}

class FakeAndroidSdk extends Fake implements AndroidSdk {
  @override
  late bool platformToolsAvailable;

  @override
  late bool licensesAvailable;

  @override
  AndroidSdkVersion? latestVersion;
}

class FakeAndroidStudio extends Fake implements AndroidStudio {
  @override
  String get javaPath => 'java';
}

class FakeJava extends Fake implements Java {
  FakeJava({
    this.javaHome = '/android-studio/jbr',
    this.javaSource = JavaSource.androidStudio,
    String binary = '/android-studio/jbr/bin/java',
    Version? version,
    bool canRun = true,
  }) : binaryPath = binary,
       version = version ?? const Version.withText(19, 0, 2, 'openjdk 19.0.2 2023-01-17'),
       _environment = <String, String>{
         if (javaHome != null) Java.javaHomeEnvironmentVariable: javaHome,
         'PATH': '/android-studio/jbr/bin',
       },
       _canRun = canRun;

  @override
  String? javaHome;

  @override
  String binaryPath;

  @override
  JavaSource javaSource;

  final Map<String, String> _environment;
  final bool _canRun;

  @override
  Map<String, String> get environment => _environment;

  @override
  Version? version;

  @override
  bool canRun() {
    return _canRun;
  }
}

class FakeDartDevelopmentServiceLauncher extends Fake implements DartDevelopmentServiceLauncher {
  FakeDartDevelopmentServiceLauncher({required this.uri, this.devToolsUri, this.dtdUri});

  @override
  final Uri uri;

  @override
  final Uri? devToolsUri;

  @override
  final Uri? dtdUri;

  @override
  Future<void> get done => _completer.future;

  @override
  Future<void> shutdown() async => _completer.complete();

  final Completer<void> _completer = Completer<void>();
}

class FakeDevtoolsLauncher extends Fake implements DevtoolsLauncher {
  FakeDevtoolsLauncher({DevToolsServerAddress? serverAddress}) : _serverAddress = serverAddress;

  @override
  Future<void> get processStart => _processStarted.future;

  final Completer<void> _processStarted = Completer<void>();

  @override
  Future<void> get ready => readyCompleter.future;

  Completer<void> readyCompleter = Completer<void>()..complete();

  @override
  DevToolsServerAddress? activeDevToolsServer;

  @override
  Uri? devToolsUrl;

  @override
  Uri? dtdUri;

  @override
  bool printDtdUri = false;

  final DevToolsServerAddress? _serverAddress;

  @override
  Future<DevToolsServerAddress?> serve() async => _serverAddress;

  @override
  Future<void> launch(Uri vmServiceUri, {List<String>? additionalArguments}) {
    _processStarted.complete();
    return Completer<void>().future;
  }

  bool closed = false;

  @override
  Future<void> close() async {
    closed = true;
  }
}

/// A fake [Logger] that throws the [Invocation] for any method call.
class FakeLogger implements Logger {
  @override
  dynamic noSuchMethod(Invocation invocation) => throw invocation; // ignore: only_throw_errors
}

class ClosedStdinController extends Fake implements StreamSink<List<int>> {
  @override
  Future<Object?> addStream(Stream<List<int>> stream) async =>
      throw const SocketException('Bad pipe');

  @override
  Future<Object?> close() async {
    return null;
  }
}
