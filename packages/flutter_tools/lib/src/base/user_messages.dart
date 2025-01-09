// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'platform.dart';

/// Class containing some message strings that can be produced by Flutter tools.
//
// This allows partial reimplementations of the flutter tool to override
// certain messages.
// TODO(andrewkolos): It is unclear if this is worth keeping. See
// https://github.com/flutter/flutter/issues/125155.
class UserMessages {
  // Messages used in multiple components.
  String get flutterToolBugInstructions =>
      'Please report a bug at https://github.com/flutter/flutter/issues.';

  // Messages used in FlutterValidator
  String flutterStatusInfo(String? channel, String? version, String os, String locale) =>
      'Channel ${channel ?? 'unknown'}, ${version ?? 'unknown version'}, on $os, locale $locale';
  String flutterVersion(String version, String channel, String flutterRoot) =>
      'Flutter version $version on channel $channel at $flutterRoot';
  String get flutterUnknownChannel =>
      'Currently on an unknown channel. Run `flutter channel` to switch to an official channel.\n'
      "If that doesn't fix the issue, reinstall Flutter by following instructions at https://flutter.dev/setup.";
  String get flutterUnknownVersion =>
      'Cannot resolve current version, possibly due to local changes.\n'
      'Reinstall Flutter by following instructions at https://flutter.dev/setup.';
  String flutterRevision(String revision, String age, String date) =>
      'Framework revision $revision ($age), $date';
  String flutterUpstreamRepositoryUrl(String url) => 'Upstream repository $url';
  String get flutterUpstreamRepositoryUnknown =>
      'Unknown upstream repository.\n'
      'Reinstall Flutter by following instructions at https://flutter.dev/setup.';
  String flutterUpstreamRepositoryUrlEnvMismatch(String url) =>
      'Upstream repository $url is not the same as FLUTTER_GIT_URL';
  String flutterUpstreamRepositoryUrlNonStandard(String url) =>
      'Upstream repository $url is not a standard remote.\n'
      'Set environment variable "FLUTTER_GIT_URL" to $url to dismiss this error.';
  String flutterGitUrl(String url) => 'FLUTTER_GIT_URL = $url';
  String engineRevision(String revision) => 'Engine revision $revision';
  String dartRevision(String revision) => 'Dart version $revision';
  String devToolsVersion(String version) => 'DevTools version $version';
  String pubMirrorURL(String url) => 'Pub download mirror $url';
  String flutterMirrorURL(String url) => 'Flutter download mirror $url';
  String get flutterBinariesDoNotRun =>
      'Downloaded executables cannot execute on host.\n'
      'See https://github.com/flutter/flutter/issues/6207 for more information.';
  String get flutterBinariesLinuxRepairCommands =>
      'On Debian/Ubuntu/Mint: sudo apt-get install lib32stdc++6\n'
      'On Fedora: dnf install libstdc++.i686\n'
      'On Arch: pacman -S lib32-gcc-libs';
  String get flutterValidatorErrorIntentional =>
      'If those were intentional, you can disregard the above warnings; however it is '
      'recommended to use "git" directly to perform update checks and upgrades.';

  // Messages used in NoIdeValidator
  String get noIdeStatusInfo => 'No supported IDEs installed';
  List<String> get noIdeInstallationInfo => <String>[
    'IntelliJ - https://www.jetbrains.com/idea/',
    'Android Studio - https://developer.android.com/studio/',
    'VS Code - https://code.visualstudio.com/',
  ];

  // Messages used in IntellijValidator
  String intellijStatusInfo(String version) => 'version $version';
  String get intellijPluginInfo =>
      'For information about installing plugins, see\n'
      'https://flutter.dev/to/intellij-setup';
  String intellijMinimumVersion(String minVersion) =>
      'This install is older than the minimum recommended version of $minVersion.';
  String intellijLocation(String installPath) => 'IntelliJ at $installPath';

  // Message used in IntellijValidatorOnMac
  String get intellijMacUnknownResult => 'Cannot determine if IntelliJ is installed';

  // Messages used in DeviceValidator
  String get devicesMissing => 'No devices available';
  String devicesAvailable(int devices) => '$devices available';

  // Messages used in AndroidValidator
  String androidCantRunJavaBinary(String javaBinary) =>
      'Cannot execute $javaBinary to determine the version';
  String get androidUnknownJavaVersion => 'Could not determine java version';
  String androidJavaVersion(String javaVersion) => 'Java version $javaVersion';
  String androidJavaMinimumVersion(String javaVersion) =>
      'Java version $javaVersion is older than the minimum recommended version of 1.8';
  String androidSdkLicenseOnly(String envKey) =>
      'Android SDK contains licenses only.\n'
      'Your first build of an Android application will take longer than usual, '
      'while gradle downloads the missing components. This functionality will '
      'only work if the licenses in the licenses folder in $envKey are valid.\n'
      'If the Android SDK has been installed to another location, set $envKey to that location.\n'
      'You may also want to add it to your PATH environment variable.\n\n'
      'Certain features, such as `flutter emulators` and `flutter devices`, will '
      'not work without the currently missing SDK components.';
  String androidBadSdkDir(String envKey, String homeDir) =>
      '$envKey = $homeDir\n'
      'but Android SDK not found at this location.';
  String androidMissingSdkInstructions(Platform platform) =>
      'Unable to locate Android SDK.\n'
      'Install Android Studio from: https://developer.android.com/studio/index.html\n'
      'On first launch it will assist you in installing the Android SDK components.\n'
      '(or visit ${androidSdkInstallUrl(platform)} for detailed instructions).\n'
      'If the Android SDK has been installed to a custom location, please use\n'
      '`flutter config --android-sdk` to update to that location.\n';
  String androidSdkLocation(String directory) => 'Android SDK at $directory';
  String androidSdkPlatformToolsVersion(String platform, String tools) =>
      'Platform $platform, build-tools $tools';
  String androidSdkInstallHelp(Platform platform) =>
      'Try re-installing or updating your Android SDK,\n'
      'visit ${androidSdkInstallUrl(platform)} for detailed instructions.';
  // Also occurs in AndroidLicenseValidator
  String androidStatusInfo(String version) => 'Android SDK version $version';

  // Messages used in AndroidLicenseValidator
  String get androidMissingJdk =>
      'No Java Development Kit (JDK) found; You must have the environment '
      'variable JAVA_HOME set and the java binary in your PATH. '
      'You can download the JDK from https://www.oracle.com/technetwork/java/javase/downloads/.';
  String get androidLicensesAll => 'All Android licenses accepted.';
  String get androidLicensesSome =>
      'Some Android licenses not accepted. To resolve this, run: flutter doctor --android-licenses';
  String get androidLicensesNone =>
      'Android licenses not accepted. To resolve this, run: flutter doctor --android-licenses';
  String androidLicensesUnknown(Platform platform) =>
      'Android license status unknown.\n'
      'Run `flutter doctor --android-licenses` to accept the SDK licenses.\n'
      'See ${androidSdkInstallUrl(platform)} for more details.';
  String androidSdkManagerOutdated(String managerPath) =>
      'A newer version of the Android SDK is required. To update, run:\n'
      '$managerPath --update\n';
  String androidLicensesTimeout(String managerPath) => 'Intentionally killing $managerPath';
  String get androidSdkShort => 'Unable to locate Android SDK.';
  String androidMissingSdkManager(String sdkManagerPath, Platform platform) =>
      'Android sdkmanager tool not found ($sdkManagerPath).\n'
      'Try re-installing or updating your Android SDK,\n'
      'visit ${androidSdkInstallUrl(platform)} for detailed instructions.';
  String androidCannotRunSdkManager(String sdkManagerPath, String error, Platform platform) =>
      'Android sdkmanager tool was found, but failed to run ($sdkManagerPath): "$error".\n'
      'Try re-installing or updating your Android SDK,\n'
      'visit ${androidSdkInstallUrl(platform)} for detailed instructions.';
  String androidSdkBuildToolsOutdated(
    int sdkMinVersion,
    String buildToolsMinVersion,
    Platform platform,
  ) =>
      'Flutter requires Android SDK $sdkMinVersion and the Android BuildTools $buildToolsMinVersion\n'
      'To update the Android SDK visit ${androidSdkInstallUrl(platform)} for detailed instructions.';
  String get androidMissingCmdTools =>
      'cmdline-tools component is missing\n'
      'Run `path/to/sdkmanager --install "cmdline-tools;latest"`\n'
      'See https://developer.android.com/studio/command-line for more details.';

  // Messages used in AndroidStudioValidator
  String androidStudioVersion(String version) => 'version $version';
  String androidStudioLocation(String location) => 'Android Studio at $location';
  String get androidStudioNeedsUpdate => 'Try updating or re-installing Android Studio.';
  String get androidStudioResetDir =>
      'Consider removing your android-studio-dir setting by running:\n'
      'flutter config --android-studio-dir=';
  String get aaptNotFound =>
      'Could not locate aapt. Please ensure you have the Android buildtools installed.';

  // Messages used in NoAndroidStudioValidator
  String androidStudioMissing(String location) =>
      'android-studio-dir = $location\n'
      'but Android Studio not found at this location.';
  String androidStudioInstallation(Platform platform) =>
      'Android Studio not found; download from https://developer.android.com/studio/index.html\n'
      '(or visit ${androidSdkInstallUrl(platform)} for detailed instructions).';

  // Messages used in XcodeValidator
  String xcodeLocation(String location) => 'Xcode at $location';

  String xcodeOutdated(String requiredVersion) =>
      'Flutter requires Xcode $requiredVersion or higher.\n'
      'Download the latest version or update via the Mac App Store.';

  String xcodeRecommended(String recommendedVersion) =>
      'Flutter recommends a minimum Xcode version of $recommendedVersion.\n'
      'Download the latest version or update via the Mac App Store.';

  String get xcodeEula =>
      "Xcode end user license agreement not signed; open Xcode or run the command 'sudo xcodebuild -license'.";
  String get xcodeMissingSimct =>
      'Xcode requires additional components to be installed in order to run.\n'
      'Launch Xcode and install additional required components when prompted or run:\n'
      '  sudo xcodebuild -runFirstLaunch';
  String get xcodeMissing =>
      'Xcode not installed; this is necessary for iOS and macOS development.\n'
      'Download at https://developer.apple.com/xcode/.';
  String get xcodeIncomplete =>
      'Xcode installation is incomplete; a full installation is necessary for iOS and macOS development.\n'
      'Download at: https://developer.apple.com/xcode/\n'
      'Or install Xcode via the App Store.\n'
      'Once installed, run:\n'
      '  sudo xcode-select --switch /Applications/Xcode.app/Contents/Developer\n'
      '  sudo xcodebuild -runFirstLaunch';

  // Messages used in CocoaPodsValidator
  String cocoaPodsVersion(String version) => 'CocoaPods version $version';
  String cocoaPodsMissing(String consequence, String installInstructions) =>
      'CocoaPods not installed.\n'
      '$consequence\n'
      'For installation instructions, $installInstructions';
  String cocoaPodsUnknownVersion(String consequence, String upgradeInstructions) =>
      'Unknown CocoaPods version installed.\n'
      '$consequence\n'
      'To update CocoaPods, $upgradeInstructions';
  String cocoaPodsOutdated(
    String currentVersion,
    String recVersion,
    String consequence,
    String upgradeInstructions,
  ) =>
      'CocoaPods $currentVersion out of date ($recVersion is recommended).\n'
      '$consequence\n'
      'To update CocoaPods, $upgradeInstructions';
  String cocoaPodsBrokenInstall(String consequence, String reinstallInstructions) =>
      'CocoaPods installed but not working.\n'
      '$consequence\n'
      'For re-installation instructions, $reinstallInstructions';

  // Messages used in VisualStudioValidator
  String visualStudioVersion(String name, String version) => '$name version $version';
  String visualStudioLocation(String location) => 'Visual Studio at $location';
  String windows10SdkVersion(String version) => 'Windows 10 SDK version $version';
  String visualStudioMissingComponents(String workload, List<String> components) =>
      'Visual Studio is missing necessary components. Please re-run the '
      'Visual Studio installer for the "$workload" workload, and include these components:\n'
      '  ${components.join('\n  ')}\n'
      '  Windows 10 SDK';
  String get windows10SdkNotFound =>
      'Unable to locate a Windows 10 SDK. If building fails, install the Windows 10 SDK in Visual Studio.';
  String visualStudioMissing(String workload) =>
      'Visual Studio not installed; this is necessary to develop Windows apps.\n'
      'Download at https://visualstudio.microsoft.com/downloads/.\n'
      'Please install the "$workload" workload, including all of its default components';
  String visualStudioTooOld(String minimumVersion, String workload) =>
      'Visual Studio $minimumVersion or later is required.\n'
      'Download at https://visualstudio.microsoft.com/downloads/.\n'
      'Please install the "$workload" workload, including all of its default components';
  String get visualStudioIsPrerelease =>
      'The current Visual Studio installation is a pre-release version. It may not be '
      'supported by Flutter yet.';
  String get visualStudioNotLaunchable =>
      'The current Visual Studio installation is not launchable. Please reinstall Visual Studio.';
  String get visualStudioIsIncomplete =>
      'The current Visual Studio installation is incomplete.\n'
      'Please use Visual Studio Installer to complete the installation or reinstall Visual Studio.';
  String get visualStudioRebootRequired =>
      'Visual Studio requires a reboot of your system to complete installation.';

  // Messages used in LinuxDoctorValidator
  String get clangMissing =>
      'clang++ is required for Linux development.\n'
      'It is likely available from your distribution (e.g.: apt install clang), or '
      'can be downloaded from https://releases.llvm.org/';
  String clangTooOld(String minimumVersion) => 'clang++ $minimumVersion or later is required.';
  String get cmakeMissing =>
      'CMake is required for Linux development.\n'
      'It is likely available from your distribution (e.g.: apt install cmake), or '
      'can be downloaded from https://cmake.org/download/';
  String cmakeTooOld(String minimumVersion) => 'cmake $minimumVersion or later is required.';
  String ninjaVersion(String version) => 'ninja version $version';
  String get ninjaMissing =>
      'ninja is required for Linux development.\n'
      'It is likely available from your distribution (e.g.: apt install ninja-build), or '
      'can be downloaded from https://github.com/ninja-build/ninja/releases';
  String ninjaTooOld(String minimumVersion) => 'ninja $minimumVersion or later is required.';
  String pkgConfigVersion(String version) => 'pkg-config version $version';
  String get pkgConfigMissing =>
      'pkg-config is required for Linux development.\n'
      'It is likely available from your distribution (e.g.: apt install pkg-config), or '
      'can be downloaded from https://www.freedesktop.org/wiki/Software/pkg-config/';
  String pkgConfigTooOld(String minimumVersion) =>
      'pkg-config $minimumVersion or later is required.';
  String get gtkLibrariesMissing =>
      'GTK 3.0 development libraries are required for Linux development.\n'
      'They are likely available from your distribution (e.g.: apt install libgtk-3-dev)';

  // Messages used in FlutterCommand
  String flutterElapsedTime(String name, String elapsedTime) =>
      '"flutter $name" took $elapsedTime.';
  String get flutterNoDevelopmentDevice =>
      "Unable to locate a development device; please run 'flutter doctor' "
      'for information about installing additional components.';
  String get flutterNoSupportedDevices => 'No supported devices connected.';
  String flutterMissPlatformProjects(List<String> unsupportedDevicesType) =>
      'If you would like your app to run on ${unsupportedDevicesType.join(' or ')}, consider running `flutter create .` to generate projects for these platforms.';
  String get flutterSpecifyDeviceWithAllOption =>
      'More than one device connected; please specify a device with '
      "the '-d <deviceId>' flag, or use '-d all' to act on all devices.";
  String get flutterSpecifyDevice =>
      'More than one device connected; please specify a device with '
      "the '-d <deviceId>' flag.";
  String get flutterNoPubspec =>
      'Error: No pubspec.yaml file found.\n'
      'This command should be run from the root of your Flutter project.';
  String flutterTargetFileMissing(String path) => 'Target file "$path" not found.';
  String get flutterBasePatchFlagsExclusive => 'Error: Only one of --baseline, --patch is allowed.';
  String get flutterBaselineRequiresTraceFile =>
      'Error: --baseline requires --compilation-trace-file to be specified.';
  String get flutterPatchRequiresTraceFile =>
      'Error: --patch requires --compilation-trace-file to be specified.';

  // Messages used in FlutterCommandRunner
  String runnerNoRoot(String error) => 'Unable to locate flutter root: $error';
  String runnerWrapColumnInvalid(dynamic value) =>
      'Argument to --wrap-column must be a positive integer. You supplied $value.';
  String runnerWrapColumnParseError(dynamic value) =>
      'Unable to parse argument --wrap-column=$value. Must be a positive integer.';
  String runnerBugReportFinished(String zipFileName) =>
      'Bug report written to $zipFileName.\n'
      'Warning: this bug report contains local paths, device identifiers, and log snippets.';
  String get runnerNoRecordTo => 'record-to location not specified';
  String get runnerNoReplayFrom => 'replay-from location not specified';
  String runnerNoEngineSrcDir(String enginePackageName, String engineEnvVar) =>
      'Unable to detect local Flutter engine src directory.\n'
      'Either specify a dependency_override for the $enginePackageName package in your pubspec.yaml and '
      'ensure --package-root is set if necessary, or set the \$$engineEnvVar environment variable, or '
      'use --local-engine-src-path to specify the path to the root of your flutter/engine repository.';
  String runnerNoEngineBuildDirInPath(String engineSourcePath) =>
      'Unable to detect a Flutter engine build directory in $engineSourcePath.\n'
      "Please ensure that $engineSourcePath is a Flutter engine 'src' directory and that "
      "you have compiled the engine in that directory, which should produce an 'out' directory";
  String get runnerLocalEngineOrWebSdkRequired =>
      'You must specify --local-engine or --local-web-sdk if you are using a locally built engine or web sdk.';
  String get runnerLocalEngineRequiresHostEngine =>
      'You are using a locally built engine (--local-engine) but have not specified --local-engine-host.\n'
      'You may be building with a different engine than the one you are running with. '
      'See https://github.com/flutter/flutter/issues/132245 for details.';
  String runnerNoEngineBuild(String engineBuildPath) =>
      'No Flutter engine build found at $engineBuildPath.';
  String runnerNoWebSdk(String webSdkPath) => 'No Flutter web sdk found at $webSdkPath.';
  String runnerWrongFlutterInstance(String flutterRoot, String currentDir) =>
      "Warning: the 'flutter' tool you are currently running is not the one from the current directory:\n"
      '  running Flutter  : $flutterRoot\n'
      '  current directory: $currentDir\n'
      'This can happen when you have multiple copies of flutter installed. Please check your system path to verify '
      "that you're running the expected version (run 'flutter --version' to see which flutter is on your path).\n";
  String runnerRemovedFlutterRepo(String flutterRoot, String flutterPath) =>
      'Warning! This package referenced a Flutter repository via the .packages file that is '
      "no longer available. The repository from which the 'flutter' tool is currently "
      'executing will be used instead.\n'
      '  running Flutter tool: $flutterRoot\n'
      '  previous reference  : $flutterPath\n'
      'This can happen if you deleted or moved your copy of the Flutter repository, or '
      'if it was on a volume that is no longer mounted or has been mounted at a '
      'different location. Please check your system path to verify that you are running '
      "the expected version (run 'flutter --version' to see which flutter is on your path).\n";
  String runnerChangedFlutterRepo(String flutterRoot, String flutterPath) =>
      "Warning! The 'flutter' tool you are currently running is from a different Flutter "
      'repository than the one last used by this package. The repository from which the '
      "'flutter' tool is currently executing will be used instead.\n"
      '  running Flutter tool: $flutterRoot\n'
      '  previous reference  : $flutterPath\n'
      'This can happen when you have multiple copies of flutter installed. Please check '
      'your system path to verify that you are running the expected version (run '
      "'flutter --version' to see which flutter is on your path).\n";
  String invalidVersionSettingHintMessage(String invalidVersion) =>
      'Invalid version $invalidVersion found, default value will be used.\n'
      'In pubspec.yaml, a valid version should look like: build-name+build-number.\n'
      'In Android, build-name is used as versionName while build-number used as versionCode.\n'
      'Read more about Android versioning at https://developer.android.com/studio/publish/versioning\n'
      'In iOS, build-name is used as CFBundleShortVersionString while build-number used as CFBundleVersion.\n'
      'Read more about iOS versioning at\n'
      'https://developer.apple.com/library/archive/documentation/General/Reference/InfoPlistKeyReference/Articles/CoreFoundationKeys.html\n';

  String androidSdkInstallUrl(Platform platform) {
    const String baseUrl = 'https://flutter.dev/to/';
    if (platform.isMacOS) {
      return '${baseUrl}macos-android-setup';
    } else if (platform.isLinux) {
      return '${baseUrl}linux-android-setup';
    } else if (platform.isWindows) {
      return '${baseUrl}windows-android-setup';
    } else {
      return '${baseUrl}android-setup';
    }
  }
}
