// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io show Platform, stdin, stdout;

/// Provides API parity with the `Platform` class in `dart:io`, but using
/// instance properties rather than static properties. This difference enables
/// the use of these APIs in tests, where you can provide mock implementations.
abstract class Platform {
  /// Creates a new [Platform].
  const Platform();

  /// The number of processors of the machine.
  int get numberOfProcessors;

  /// The path separator used by the operating system to separate
  /// components in file paths.
  String get pathSeparator;

  /// A string (`linux`, `macos`, `windows`, `android`, `ios`, or `fuchsia`)
  /// representing the operating system.
  String get operatingSystem;

  /// A string representing the version of the operating system or platform.
  String get operatingSystemVersion;

  /// Get the local hostname for the system.
  String get localHostname;

  /// True if the operating system is Linux.
  bool get isLinux => operatingSystem == 'linux';

  /// True if the operating system is OS X.
  bool get isMacOS => operatingSystem == 'macos';

  /// True if the operating system is Windows.
  bool get isWindows => operatingSystem == 'windows';

  /// True if the operating system is Android.
  bool get isAndroid => operatingSystem == 'android';

  /// True if the operating system is iOS.
  bool get isIOS => operatingSystem == 'ios';

  /// True if the operating system is Fuchsia.
  bool get isFuchsia => operatingSystem == 'fuchsia';

  /// The environment for this process.
  ///
  /// The returned environment is an unmodifiable map whose content is
  /// retrieved from the operating system on its first use.
  ///
  /// Environment variables on Windows are case-insensitive. The map
  /// returned on Windows is therefore case-insensitive and will convert
  /// all keys to upper case. On other platforms the returned map is
  /// a standard case-sensitive map.
  Map<String, String> get environment;

  /// The path of the executable used to run the script in this isolate.
  ///
  /// The path returned is the literal path used to run the script. This
  /// path might be relative or just be a name from which the executable
  /// was found by searching the `PATH`.
  ///
  /// To get the absolute path to the resolved executable use
  /// [resolvedExecutable].
  String get executable;

  /// The path of the executable used to run the script in this
  /// isolate after it has been resolved by the OS.
  ///
  /// This is the absolute path, with all symlinks resolved, to the
  /// executable used to run the script.
  String get resolvedExecutable;

  /// The absolute URI of the script being run in this
  /// isolate.
  ///
  /// If the script argument on the command line is relative,
  /// it is resolved to an absolute URI before fetching the script, and
  /// this absolute URI is returned.
  ///
  /// URI resolution only does string manipulation on the script path, and this
  /// may be different from the file system's path resolution behavior. For
  /// example, a symbolic link immediately followed by '..' will not be
  /// looked up.
  ///
  /// If the executable environment does not support [script] an empty
  /// [Uri] is returned.
  Uri get script;

  /// The flags passed to the executable used to run the script in this
  /// isolate. These are the command-line flags between the executable name
  /// and the script name. Each fetch of `executableArguments` returns a new
  /// list containing the flags passed to the executable.
  List<String> get executableArguments;

  /// The value of the `--packages` flag passed to the executable
  /// used to run the script in this isolate. This is the configuration which
  /// specifies how Dart packages are looked up.
  ///
  /// If there is no `--packages` flag, `null` is returned.
  String? get packageConfig;

  /// The version of the current Dart runtime.
  ///
  /// The returned `String` is formatted as the [semver](http://semver.org)
  /// version string of the current dart runtime, possibly followed by
  /// whitespace and other version and build details.
  String get version;

  /// When stdin is connected to a terminal, whether ANSI codes are supported.
  bool get stdinSupportsAnsi;

  /// When stdout is connected to a terminal, whether ANSI codes are supported.
  bool get stdoutSupportsAnsi;

  /// Get the name of the current locale.
  String get localeName;
}

/// `Platform` implementation that delegates directly to `dart:io`.
class LocalPlatform extends Platform {
  /// Creates a new [LocalPlatform].
  const LocalPlatform();

  @override
  int get numberOfProcessors => io.Platform.numberOfProcessors;

  @override
  String get pathSeparator => io.Platform.pathSeparator;

  @override
  String get operatingSystem => io.Platform.operatingSystem;

  @override
  String get operatingSystemVersion => io.Platform.operatingSystemVersion;

  @override
  String get localHostname => io.Platform.localHostname;

  @override
  Map<String, String> get environment => io.Platform.environment;

  @override
  String get executable => io.Platform.executable;

  @override
  String get resolvedExecutable => io.Platform.resolvedExecutable;

  @override
  Uri get script => io.Platform.script;

  @override
  List<String> get executableArguments => io.Platform.executableArguments;

  @override
  String? get packageConfig => io.Platform.packageConfig;

  @override
  String get version => io.Platform.version;

  @override
  bool get stdinSupportsAnsi => io.stdin.supportsAnsiEscapes;

  @override
  bool get stdoutSupportsAnsi => io.stdout.supportsAnsiEscapes;

  @override
  String get localeName => io.Platform.localeName;
}

final Uri _empty = Uri.parse('');

/// Provides a mutable implementation of the [Platform] interface.
class FakePlatform extends Platform {
  /// Creates a new [FakePlatform] with the specified properties.
  ///
  /// Unspecified properties will default to a 'linux' OS.
  FakePlatform({
    this.numberOfProcessors = 1,
    this.pathSeparator = '/',
    this.operatingSystem = 'linux',
    this.operatingSystemVersion = '',
    this.localHostname = '',
    this.environment = const <String, String>{},
    this.executable = '',
    this.resolvedExecutable = '',
    Uri? script,
    this.executableArguments = const <String>[],
    this.packageConfig,
    this.version = '',
    this.stdinSupportsAnsi = false,
    this.stdoutSupportsAnsi = false,
    this.localeName = '',
  }) : script = script ?? _empty;

  /// Creates a new [FakePlatform] with properties whose initial values mirror
  /// the specified [platform].
  FakePlatform.fromPlatform(Platform platform)
    : numberOfProcessors = platform.numberOfProcessors,
      pathSeparator = platform.pathSeparator,
      operatingSystem = platform.operatingSystem,
      operatingSystemVersion = platform.operatingSystemVersion,
      localHostname = platform.localHostname,
      environment = Map<String, String>.from(platform.environment),
      executable = platform.executable,
      resolvedExecutable = platform.resolvedExecutable,
      script = platform.script,
      executableArguments = List<String>.from(platform.executableArguments),
      packageConfig = platform.packageConfig,
      version = platform.version,
      stdinSupportsAnsi = platform.stdinSupportsAnsi,
      stdoutSupportsAnsi = platform.stdoutSupportsAnsi,
      localeName = platform.localeName;

  @override
  int numberOfProcessors;

  @override
  String pathSeparator;

  @override
  String operatingSystem;

  @override
  String operatingSystemVersion;

  @override
  String localHostname;

  @override
  Map<String, String> environment;

  @override
  String executable;

  @override
  String resolvedExecutable;

  @override
  Uri script;

  @override
  List<String> executableArguments;

  @override
  String? packageConfig;

  @override
  String version;

  @override
  bool stdinSupportsAnsi;

  @override
  bool stdoutSupportsAnsi;

  @override
  String localeName;
}
