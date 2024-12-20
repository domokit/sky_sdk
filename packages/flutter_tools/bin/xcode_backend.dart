// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

void main(List<String> arguments) {
  File? scriptOutputStreamFile;
  final String? scriptOutputStreamFileEnv = Platform.environment['SCRIPT_OUTPUT_STREAM_FILE'];
  if (scriptOutputStreamFileEnv != null && scriptOutputStreamFileEnv.isNotEmpty) {
    scriptOutputStreamFile = File(scriptOutputStreamFileEnv);
  }
  Context(
    arguments: arguments,
    environment: Platform.environment,
    scriptOutputStreamFile: scriptOutputStreamFile,
  ).run();
}

/// Container for script arguments and environment variables.
///
/// All interactions with the platform are broken into individual methods that
/// can be overridden in tests.
class Context {
  Context({required this.arguments, required this.environment, File? scriptOutputStreamFile}) {
    if (scriptOutputStreamFile != null) {
      scriptOutputStream = scriptOutputStreamFile.openSync(mode: FileMode.write);
    }
  }

  final Map<String, String> environment;
  final List<String> arguments;
  RandomAccessFile? scriptOutputStream;

  void run() {
    if (arguments.isEmpty) {
      // Named entry points were introduced in Flutter v0.0.7.
      stderr.write(
        'error: Your Xcode project is incompatible with this version of Flutter. '
        'Run "rm -rf ios/Runner.xcodeproj" and "flutter create ." to regenerate.\n',
      );
      exit(-1);
    }

    final String subCommand = arguments.first;
    switch (subCommand) {
      case 'build':
        buildApp();
      case 'prepare':
        prepare();
      case 'thin':
        // No-op, thinning is handled during the bundle asset assemble build target.
        break;
      case 'embed':
        embedFlutterFrameworks();
      case 'embed_and_thin':
        // Thinning is handled during the bundle asset assemble build target, so just embed.
        embedFlutterFrameworks();
      case 'test_vm_service_bonjour_service':
        // Exposed for integration testing only.
        addVmServiceBonjourService();
    }
  }

  bool existsFile(String path) {
    final File file = File(path);
    return file.existsSync();
  }

  /// Run given command in a synchronous subprocess.
  ///
  /// Will throw [Exception] if the exit code is not 0.
  ProcessResult runSync(
    String bin,
    List<String> args, {
    bool verbose = false,
    bool allowFail = false,
    String? workingDirectory,
  }) {
    if (verbose) {
      print('♦ $bin ${args.join(' ')}');
    }
    final ProcessResult result = Process.runSync(bin, args, workingDirectory: workingDirectory);
    if (verbose) {
      print((result.stdout as String).trim());
    }
    final String resultStderr = result.stderr.toString().trim();
    if (resultStderr.isNotEmpty) {
      final StringBuffer errorOutput = StringBuffer();
      if (result.exitCode != 0) {
        // "error:" prefix makes this show up as an Xcode compilation error.
        errorOutput.write('error: ');
      }
      errorOutput.write(resultStderr);
      echoError(errorOutput.toString());
    }
    if (!allowFail && result.exitCode != 0) {
      throw Exception('Command "$bin ${args.join(' ')}" exited with code ${result.exitCode}');
    }
    return result;
  }

  /// Log message to stderr.
  void echoError(String message) {
    stderr.writeln(message);
  }

  /// Log message to stdout.
  void echo(String message) {
    stdout.write(message);
  }

  /// Exit the application with the given exit code.
  ///
  /// Exists to allow overriding in tests.
  Never exitApp(int code) {
    exit(code);
  }

  /// Return value from environment if it exists, else throw [Exception].
  String environmentEnsure(String key) {
    final String? value = environment[key];
    if (value == null) {
      throw Exception('Expected the environment variable "$key" to exist, but it was not found');
    }
    return value;
  }

  // When provided with a pipe by the host Flutter build process, output to the
  // pipe goes to stdout of the Flutter build process directly.
  void streamOutput(String output) {
    scriptOutputStream?.writeStringSync('$output\n');
  }

  String parseFlutterBuildMode() {
    // Use FLUTTER_BUILD_MODE if it's set, otherwise use the Xcode build configuration name
    // This means that if someone wants to use an Xcode build config other than Debug/Profile/Release,
    // they _must_ set FLUTTER_BUILD_MODE so we know what type of artifact to build.
    final String? buildMode =
        (environment['FLUTTER_BUILD_MODE'] ?? environment['CONFIGURATION'])?.toLowerCase();

    if (buildMode != null) {
      if (buildMode.contains('release')) {
        return 'release';
      }
      if (buildMode.contains('profile')) {
        return 'profile';
      }
      if (buildMode.contains('debug')) {
        return 'debug';
      }
    }
    echoError('========================================================================');
    echoError('ERROR: Unknown FLUTTER_BUILD_MODE: $buildMode.');
    echoError("Valid values are 'Debug', 'Profile', or 'Release' (case insensitive).");
    echoError('This is controlled by the FLUTTER_BUILD_MODE environment variable.');
    echoError('If that is not set, the CONFIGURATION environment variable is used.');
    echoError('');
    echoError('You can fix this by either adding an appropriately named build');
    echoError('configuration, or adding an appropriate value for FLUTTER_BUILD_MODE to the');
    echoError(
      '.xcconfig file for the current build configuration (${environment['CONFIGURATION']}).',
    );
    echoError('========================================================================');
    exitApp(-1);
  }

  /// Copies all files from [source] to [destination].
  ///
  /// Does not copy `.DS_Store`.
  ///
  /// If [delete], delete extraneous files from [destination].
  void runRsync(
    String source,
    String destination, {
    List<String> extraArgs = const <String>[],
    bool delete = false,
  }) {
    runSync('rsync', <String>[
      '-8', // Avoid mangling filenames with encodings that do not match the current locale.
      '-av',
      if (delete) '--delete',
      '--filter',
      '- .DS_Store',
      ...extraArgs,
      source,
      destination,
    ]);
  }

  // Adds the App.framework as an embedded binary and the flutter_assets as
  // resources.
  void embedFlutterFrameworks() {
    // Embed App.framework from Flutter into the app (after creating the Frameworks directory
    // if it doesn't already exist).
    final String xcodeFrameworksDir =
        '${environment['TARGET_BUILD_DIR']}/${environment['FRAMEWORKS_FOLDER_PATH']}';
    runSync('mkdir', <String>['-p', '--', xcodeFrameworksDir]);
    runRsync(
      delete: true,
      '${environment['BUILT_PRODUCTS_DIR']}/App.framework',
      xcodeFrameworksDir,
    );

    // Embed the actual Flutter.framework that the Flutter app expects to run against,
    // which could be a local build or an arch/type specific build.
    runRsync(
      delete: true,
      '${environment['BUILT_PRODUCTS_DIR']}/Flutter.framework',
      '$xcodeFrameworksDir/',
    );

    // Copy the native assets. These do not have to be codesigned here because,
    // they are already codesigned in buildNativeAssetsMacOS.
    final String sourceRoot = environment['SOURCE_ROOT'] ?? '';
    String projectPath = '$sourceRoot/..';
    if (environment['FLUTTER_APPLICATION_PATH'] != null) {
      projectPath = environment['FLUTTER_APPLICATION_PATH']!;
    }
    final String flutterBuildDir = environment['FLUTTER_BUILD_DIR']!;
    final String nativeAssetsPath = '$projectPath/$flutterBuildDir/native_assets/ios/';
    final bool verbose = (environment['VERBOSE_SCRIPT_LOGGING'] ?? '').isNotEmpty;
    if (Directory(nativeAssetsPath).existsSync()) {
      if (verbose) {
        print('♦ Copying native assets from $nativeAssetsPath.');
      }
      runRsync(
        extraArgs: <String>['--filter', '- native_assets.yaml', '--filter', '- native_assets.json'],
        nativeAssetsPath,
        xcodeFrameworksDir,
      );
    } else if (verbose) {
      print("♦ No native assets to bundle. $nativeAssetsPath doesn't exist.");
    }

    addVmServiceBonjourService();
  }

  // Add the vmService publisher Bonjour service to the produced app bundle Info.plist.
  void addVmServiceBonjourService() {
    // Skip adding Bonjour service settings when DISABLE_PORT_PUBLICATION is YES.
    // These settings are not needed if port publication is disabled.
    if (environment['DISABLE_PORT_PUBLICATION'] == 'YES') {
      return;
    }

    final String buildMode = parseFlutterBuildMode();

    // Debug and profile only.
    if (buildMode == 'release') {
      return;
    }

    final String builtProductsPlist =
        '${environment['BUILT_PRODUCTS_DIR'] ?? ''}/${environment['INFOPLIST_PATH'] ?? ''}';

    if (!existsFile(builtProductsPlist)) {
      // Very occasionally Xcode hasn't created an Info.plist when this runs.
      // The file will be present on re-run.
      echo(
        '${environment['INFOPLIST_PATH'] ?? ''} does not exist. Skipping '
        '_dartVmService._tcp NSBonjourServices insertion. Try re-building to '
        'enable "flutter attach".',
      );
      return;
    }

    // If there are already NSBonjourServices specified by the app (uncommon),
    // insert the vmService service name to the existing list.
    ProcessResult result = runSync('plutil', <String>[
      '-extract',
      'NSBonjourServices',
      'xml1',
      '-o',
      '-',
      builtProductsPlist,
    ], allowFail: true);
    if (result.exitCode == 0) {
      runSync('plutil', <String>[
        '-insert',
        'NSBonjourServices.0',
        '-string',
        '_dartVmService._tcp',
        builtProductsPlist,
      ]);
    } else {
      // Otherwise, add the NSBonjourServices key and vmService service name.
      runSync('plutil', <String>[
        '-insert',
        'NSBonjourServices',
        '-json',
        '["_dartVmService._tcp"]',
        builtProductsPlist,
      ]);
      //fi
    }

    // Don't override the local network description the Flutter app developer
    // specified (uncommon). This text will appear below the "Your app would
    // like to find and connect to devices on your local network" permissions
    // popup.
    result = runSync('plutil', <String>[
      '-extract',
      'NSLocalNetworkUsageDescription',
      'xml1',
      '-o',
      '-',
      builtProductsPlist,
    ], allowFail: true);
    if (result.exitCode != 0) {
      runSync('plutil', <String>[
        '-insert',
        'NSLocalNetworkUsageDescription',
        '-string',
        'Allow Flutter tools on your computer to connect and debug your application. This prompt will not appear on release builds.',
        builtProductsPlist,
      ]);
    }
  }

  void prepare() {
    // The "prepare" command runs in a pre-action script, which also runs when
    // using the Xcode/xcodebuild clean command. Skip if cleaning.
    if (environment['ACTION'] == 'clean') {
      return;
    }
    final bool verbose = (environment['VERBOSE_SCRIPT_LOGGING'] ?? '').isNotEmpty;
    final String sourceRoot = environment['SOURCE_ROOT'] ?? '';
    final String projectPath = environment['FLUTTER_APPLICATION_PATH'] ?? '$sourceRoot/..';

    final String buildMode = parseFlutterBuildMode();

    final List<String> flutterArgs = _generateFlutterArgsForAssemble('prepare', buildMode, verbose);

    // The "prepare" command only targets the UnpackIOS target, which copies the
    // Flutter framework to the BUILT_PRODUCTS_DIR.
    flutterArgs.add('${buildMode}_unpack_ios');

    final ProcessResult result = runSync(
      '${environmentEnsure('FLUTTER_ROOT')}/bin/flutter',
      flutterArgs,
      verbose: verbose,
      allowFail: true,
      workingDirectory: projectPath, // equivalent of RunCommand pushd "${project_path}"
    );

    if (result.exitCode != 0) {
      echoError('Failed to copy Flutter framework.');
      exitApp(-1);
    }
  }

  void buildApp() {
    final bool verbose = (environment['VERBOSE_SCRIPT_LOGGING'] ?? '').isNotEmpty;
    final String sourceRoot = environment['SOURCE_ROOT'] ?? '';
    final String projectPath = environment['FLUTTER_APPLICATION_PATH'] ?? '$sourceRoot/..';

    final String buildMode = parseFlutterBuildMode();

    final List<String> flutterArgs = _generateFlutterArgsForAssemble('build', buildMode, verbose);

    flutterArgs.add('${buildMode}_ios_bundle_flutter_assets');

    final ProcessResult result = runSync(
      '${environmentEnsure('FLUTTER_ROOT')}/bin/flutter',
      flutterArgs,
      verbose: verbose,
      allowFail: true,
      workingDirectory: projectPath, // equivalent of RunCommand pushd "${project_path}"
    );

    if (result.exitCode != 0) {
      echoError('Failed to package $projectPath.');
      exitApp(-1);
    }

    streamOutput('done');
    streamOutput(' └─Compiling, linking and signing...');

    echo('Project $projectPath built and packaged successfully.');
  }

  List<String> _generateFlutterArgsForAssemble(String command, String buildMode, bool verbose) {
    String targetPath = 'lib/main.dart';
    if (environment['FLUTTER_TARGET'] != null) {
      targetPath = environment['FLUTTER_TARGET']!;
    }

    // Warn the user if not archiving (ACTION=install) in release mode.
    final String? action = environment['ACTION'];
    if (action == 'install' && buildMode != 'release') {
      echo(
        'warning: Flutter archive not built in Release mode. Ensure '
        'FLUTTER_BUILD_MODE is set to release or run "flutter build ios '
        '--release", then re-run Archive from Xcode.',
      );
    }

    final List<String> flutterArgs = <String>[];

    if (verbose) {
      flutterArgs.add('--verbose');
    }

    if (environment['FLUTTER_ENGINE'] != null && environment['FLUTTER_ENGINE']!.isNotEmpty) {
      flutterArgs.add('--local-engine-src-path=${environment['FLUTTER_ENGINE']}');
    }

    if (environment['LOCAL_ENGINE'] != null && environment['LOCAL_ENGINE']!.isNotEmpty) {
      flutterArgs.add('--local-engine=${environment['LOCAL_ENGINE']}');
    }

    if (environment['LOCAL_ENGINE_HOST'] != null && environment['LOCAL_ENGINE_HOST']!.isNotEmpty) {
      flutterArgs.add('--local-engine-host=${environment['LOCAL_ENGINE_HOST']}');
    }

    // The "prepare" command runs in a pre-action script, which doesn't always
    // filter the "ARCHS" build setting. Attempt to filter the architecture
    // to improve caching. If this filter is incorrect, it will later be
    // corrected by the "build" command.
    String archs = environment['ARCHS'] ?? '';
    if (command == 'prepare' && archs.contains(' ')) {
      // If "ONLY_ACTIVE_ARCH" is "YES", the product includes only code for the
      // native architecture ("NATIVE_ARCH").
      final String? nativeArch = environment['NATIVE_ARCH'];
      if (environment['ONLY_ACTIVE_ARCH'] == 'YES' && nativeArch != null) {
        if (nativeArch.contains('arm64') && archs.contains('arm64')) {
          archs = 'arm64';
        } else if (nativeArch.contains('x86_64') && archs.contains('x86_64')) {
          archs = 'x86_64';
        }
      }
    }

    flutterArgs.addAll(<String>[
      'assemble',
      '--no-version-check',
      '--output=${environment['BUILT_PRODUCTS_DIR'] ?? ''}/',
      '-dTargetPlatform=ios',
      '-dTargetFile=$targetPath',
      '-dBuildMode=$buildMode',
      if (environment['FLAVOR'] != null) '-dFlavor=${environment['FLAVOR']}',
      '-dIosArchs=$archs',
      '-dSdkRoot=${environment['SDKROOT'] ?? ''}',
      '-dSplitDebugInfo=${environment['SPLIT_DEBUG_INFO'] ?? ''}',
      '-dTreeShakeIcons=${environment['TREE_SHAKE_ICONS'] ?? ''}',
      '-dTrackWidgetCreation=${environment['TRACK_WIDGET_CREATION'] ?? ''}',
      '-dDartObfuscation=${environment['DART_OBFUSCATION'] ?? ''}',
      '-dAction=${environment['ACTION'] ?? ''}',
      '-dFrontendServerStarterPath=${environment['FRONTEND_SERVER_STARTER_PATH'] ?? ''}',
      '--ExtraGenSnapshotOptions=${environment['EXTRA_GEN_SNAPSHOT_OPTIONS'] ?? ''}',
      '--DartDefines=${environment['DART_DEFINES'] ?? ''}',
      '--ExtraFrontEndOptions=${environment['EXTRA_FRONT_END_OPTIONS'] ?? ''}',
    ]);

    if (command == 'prepare') {
      // Use the PreBuildAction define flag to force the tool to use a different
      // filecache file for the "prepare" command. This will make the environment
      // buildPrefix for the "prepare" command unique from the "build" command.
      // This will improve caching since the "build" command has more target dependencies.
      flutterArgs.add('-dPreBuildAction=PrepareFramework');
    }

    if (environment['PERFORMANCE_MEASUREMENT_FILE'] != null &&
        environment['PERFORMANCE_MEASUREMENT_FILE']!.isNotEmpty) {
      flutterArgs.add(
        '--performance-measurement-file=${environment['PERFORMANCE_MEASUREMENT_FILE']}',
      );
    }

    final String? expandedCodeSignIdentity = environment['EXPANDED_CODE_SIGN_IDENTITY'];
    if (expandedCodeSignIdentity != null &&
        expandedCodeSignIdentity.isNotEmpty &&
        environment['CODE_SIGNING_REQUIRED'] != 'NO') {
      flutterArgs.add('-dCodesignIdentity=$expandedCodeSignIdentity');
    }

    if (environment['BUNDLE_SKSL_PATH'] != null && environment['BUNDLE_SKSL_PATH']!.isNotEmpty) {
      flutterArgs.add('-dBundleSkSLPath=${environment['BUNDLE_SKSL_PATH']}');
    }

    if (environment['CODE_SIZE_DIRECTORY'] != null &&
        environment['CODE_SIZE_DIRECTORY']!.isNotEmpty) {
      flutterArgs.add('-dCodeSizeDirectory=${environment['CODE_SIZE_DIRECTORY']}');
    }

    return flutterArgs;
  }
}
