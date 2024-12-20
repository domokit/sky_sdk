// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../../src/android/android_sdk.dart';
import '../../src/android/android_studio.dart';
import '../android/java.dart';
import '../base/common.dart';
import '../convert.dart';
import '../features.dart';
import '../globals.dart' as globals;
import '../reporting/reporting.dart';
import '../runner/flutter_command.dart';
import '../runner/flutter_command_runner.dart';

class ConfigCommand extends FlutterCommand {
  ConfigCommand({bool verboseHelp = false}) {
    argParser.addFlag(
      'list',
      help: 'List all settings and their current values.',
      negatable: false,
    );
    argParser.addFlag(
      'analytics',
      hide: !verboseHelp,
      help:
          'Enable or disable reporting anonymously tool usage statistics and crash reports.\n'
          '(An alias for "--${FlutterGlobalOptions.kEnableAnalyticsFlag}" '
          'and "--${FlutterGlobalOptions.kDisableAnalyticsFlag}" top level flags.)',
    );
    argParser.addFlag(
      'clear-ios-signing-cert',
      negatable: false,
      help:
          'Clear the saved development certificate choice used to sign apps for iOS device deployment.',
    );
    argParser.addOption('android-sdk', help: 'The Android SDK directory.');
    argParser.addOption(
      'android-studio-dir',
      help:
          'The Android Studio installation directory. If unset, flutter will search for valid installations at well-known locations.',
    );
    argParser.addOption(
      'jdk-dir',
      help:
          'The Java Development Kit (JDK) installation directory. '
          'If unset, flutter will search for one in the following order:\n'
          '    1) the JDK bundled with the latest installation of Android Studio,\n'
          '    2) the JDK found at the directory found in the JAVA_HOME environment variable, and\n'
          "    3) the directory containing the java binary found in the user's path.",
    );
    argParser.addOption(
      'build-dir',
      help: 'The relative path to override a projects build directory.',
      valueHelp: 'out/',
    );
    argParser.addFlag(
      'machine',
      negatable: false,
      hide: !verboseHelp,
      help: 'Print config values as json.',
    );
    for (final Feature feature in allFeatures) {
      final String? configSetting = feature.configSetting;
      if (configSetting == null) {
        continue;
      }
      argParser.addFlag(configSetting, help: feature.generateHelpMessage());
    }
    argParser.addFlag(
      'clear-features',
      help: 'Remove all configured features and restore them to the default values.',
      negatable: false,
    );
  }

  @override
  final String name = 'config';

  @override
  final String description =
      'Configure Flutter settings.\n\n'
      'To remove a setting, configure it to an empty string.\n\n'
      'The Flutter tool anonymously reports feature usage statistics and basic crash reports to help improve '
      "Flutter tools over time. See Google's privacy policy: https://www.google.com/intl/en/policies/privacy/";

  @override
  final String category = FlutterCommandCategory.sdk;

  @override
  final List<String> aliases = <String>['configure'];

  @override
  bool get shouldUpdateCache => false;

  @override
  String get usageFooter => '\n$analyticsUsage';

  /// Return null to disable analytics recording of the `config` command.
  @override
  Future<String?> get usagePath async => null;

  @override
  Future<FlutterCommandResult> runCommand() async {
    final List<String> rest = argResults!.rest;
    if (rest.isNotEmpty) {
      throwToolExit(
        exitCode: 2,
        'error: flutter config: Too many arguments.\n'
        '\n'
        'If a value has a space in it, enclose in quotes on the command line\n'
        'to make a single argument.  For example:\n'
        '    flutter config --android-studio-dir "/opt/Android Studio"',
      );
    }

    if (boolArg('list')) {
      globals.printStatus(settingsText);
      return FlutterCommandResult.success();
    }

    if (boolArg('machine')) {
      await handleMachine();
      return FlutterCommandResult.success();
    }

    if (boolArg('clear-features')) {
      for (final Feature feature in allFeatures) {
        final String? configSetting = feature.configSetting;
        if (configSetting != null) {
          globals.config.removeValue(configSetting);
        }
      }
      globals.printStatus(requireReloadTipText);
      return FlutterCommandResult.success();
    }

    if (argResults!.wasParsed('analytics')) {
      final bool value = boolArg('analytics');
      // The tool sends the analytics event *before* toggling the flag
      // intentionally to be sure that opt-out events are sent correctly.
      AnalyticsConfigEvent(enabled: value).send();
      if (!value) {
        // Normally, the tool waits for the analytics to all send before the
        // tool exits, but only when analytics are enabled. When reporting that
        // analytics have been disable, the wait must be done here instead.
        await globals.flutterUsage.ensureAnalyticsSent();
      }
      globals.flutterUsage.enabled = value;
      globals.printStatus('Analytics reporting ${value ? 'enabled' : 'disabled'}.');

      // TODO(eliasyishak): Set the telemetry for the unified_analytics
      //  package as well, the above will be removed once we have
      //  fully transitioned to using the new package,
      //  https://github.com/flutter/flutter/issues/128251
      await globals.analytics.setTelemetry(value);
    }

    if (argResults!.wasParsed('android-sdk')) {
      _updateConfig('android-sdk', stringArg('android-sdk')!);
    }

    if (argResults!.wasParsed('android-studio-dir')) {
      _updateConfig('android-studio-dir', stringArg('android-studio-dir')!);
    }

    if (argResults!.wasParsed('jdk-dir')) {
      _updateConfig('jdk-dir', stringArg('jdk-dir')!);
    }

    if (argResults!.wasParsed('clear-ios-signing-cert')) {
      _updateConfig('ios-signing-cert', '');
    }

    if (argResults!.wasParsed('build-dir')) {
      final String buildDir = stringArg('build-dir')!;
      if (globals.fs.path.isAbsolute(buildDir)) {
        throwToolExit('build-dir should be a relative path');
      }
      _updateConfig('build-dir', buildDir);
    }

    for (final Feature feature in allFeatures) {
      final String? configSetting = feature.configSetting;
      if (configSetting == null) {
        continue;
      }
      if (argResults!.wasParsed(configSetting)) {
        final bool keyValue = boolArg(configSetting);
        globals.config.setValue(configSetting, keyValue);
        globals.printStatus('Setting "$configSetting" value to "$keyValue".');
      }
    }

    if (argResults == null || argResults!.arguments.isEmpty) {
      globals.printStatus(usage);
    } else {
      globals.printStatus('\n$requireReloadTipText');
    }

    return FlutterCommandResult.success();
  }

  Future<void> handleMachine() async {
    // Get all the current values.
    final Map<String, Object?> results = <String, Object?>{};
    for (final String key in globals.config.keys) {
      results[key] = globals.config.getValue(key);
    }

    // Ensure we send any calculated ones, if overrides don't exist.
    final AndroidStudio? androidStudio = globals.androidStudio;
    if (results['android-studio-dir'] == null && androidStudio != null) {
      results['android-studio-dir'] = androidStudio.directory;
    }
    final AndroidSdk? androidSdk = globals.androidSdk;
    if (results['android-sdk'] == null && androidSdk != null) {
      results['android-sdk'] = androidSdk.directory.path;
    }
    final Java? java = globals.java;
    if (results['jdk-dir'] == null && java != null) {
      results['jdk-dir'] = java.javaHome;
    }

    globals.printStatus(const JsonEncoder.withIndent('  ').convert(results));
  }

  void _updateConfig(String keyName, String keyValue) {
    if (keyValue.isEmpty) {
      globals.config.removeValue(keyName);
      globals.printStatus('Removing "$keyName" value.');
    } else {
      globals.config.setValue(keyName, keyValue);
      globals.printStatus('Setting "$keyName" value to "$keyValue".');
    }
  }

  /// List all config settings. for feature flags, include whether they are available.
  String get settingsText {
    final Map<String, Feature> featuresByName = <String, Feature>{};
    final String channel = globals.flutterVersion.channel;
    for (final Feature feature in allFeatures) {
      final String? configSetting = feature.configSetting;
      if (configSetting != null) {
        featuresByName[configSetting] = feature;
      }
    }
    final Set<String> keys = <String>{
      ...allFeatures.map((Feature e) => e.configSetting).whereType<String>(),
      ...globals.config.keys,
    };
    final Iterable<String> settings = keys.map<String>((String key) {
      Object? value = globals.config.getValue(key);
      value ??= '(Not set)';
      final StringBuffer buffer = StringBuffer('  $key: $value');
      if (featuresByName.containsKey(key)) {
        final FeatureChannelSetting setting = featuresByName[key]!.getSettingForChannel(channel);
        if (!setting.available) {
          buffer.write(' (Unavailable)');
        }
      }
      return buffer.toString();
    });
    final StringBuffer buffer = StringBuffer();
    buffer.writeln('All Settings:');
    if (settings.isEmpty) {
      buffer.writeln('  No configs have been configured.');
    } else {
      buffer.writeln(settings.join('\n'));
    }
    return buffer.toString();
  }

  /// List the status of the analytics reporting.
  String get analyticsUsage {
    final bool analyticsEnabled =
        globals.flutterUsage.enabled && !globals.flutterUsage.suppressAnalytics;
    return 'Analytics reporting is currently ${analyticsEnabled ? 'enabled' : 'disabled'}.';
  }

  /// Raising the reload tip for setting changes.
  final String requireReloadTipText =
      'You may need to restart any open editors for them to read new settings.';
}
