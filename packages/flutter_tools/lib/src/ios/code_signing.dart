// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:process/process.dart';

import '../base/common.dart';
import '../base/config.dart';
import '../base/io.dart';
import '../base/logger.dart';
import '../base/platform.dart';
import '../base/process.dart';
import '../base/terminal.dart';
import '../convert.dart' show utf8;

const String _developmentTeamBuildSettingName = 'DEVELOPMENT_TEAM';

/// User message when no development certificates are found in the keychain.
///
/// The user likely never did any iOS development.
const String noCertificatesInstruction = '''
════════════════════════════════════════════════════════════════════════════════
No valid code signing certificates were found
You can connect to your Apple Developer account by signing in with your Apple ID
in Xcode and create an iOS Development Certificate as well as a Provisioning\u0020
Profile for your project by:
$fixWithDevelopmentTeamInstruction
  5- Trust your newly created Development Certificate on your iOS device
     via Settings > General > Device Management > [your new certificate] > Trust

For more information, please visit:
  https://developer.apple.com/library/content/documentation/IDEs/Conceptual/
  AppDistributionGuide/MaintainingCertificates/MaintainingCertificates.html

Or run on an iOS simulator without code signing
════════════════════════════════════════════════════════════════════════════════''';

/// User message when there are no provisioning profile for the current app bundle identifier.
///
/// The user did iOS development but never on this project and/or device.
const String noProvisioningProfileInstruction = '''
════════════════════════════════════════════════════════════════════════════════
No Provisioning Profile was found for your project's Bundle Identifier or your\u0020
device. You can create a new Provisioning Profile for your project in Xcode for\u0020
your team by:
$fixWithDevelopmentTeamInstruction

It's also possible that a previously installed app with the same Bundle\u0020
Identifier was signed with a different certificate.

For more information, please visit:
  https://flutter.dev/to/ios-app-signing

Or run on an iOS simulator without code signing
════════════════════════════════════════════════════════════════════════════════''';

/// Fallback error message for signing issues.
///
/// Couldn't auto sign the app but can likely solved by retracing the signing flow in Xcode.
const String noDevelopmentTeamInstruction = '''
════════════════════════════════════════════════════════════════════════════════
Building a deployable iOS app requires a selected Development Team with a\u0020
Provisioning Profile. Please ensure that a Development Team is selected by:
$fixWithDevelopmentTeamInstruction

For more information, please visit:
  https://flutter.dev/to/ios-development-team

Or run on an iOS simulator without code signing
════════════════════════════════════════════════════════════════════════════════''';
const String fixWithDevelopmentTeamInstruction = '''
  1- Open the Flutter project's Xcode target with
       open ios/Runner.xcworkspace
  2- Select the 'Runner' project in the navigator then the 'Runner' target
     in the project settings
  3- Make sure a 'Development Team' is selected under Signing & Capabilities > Team.\u0020
     You may need to:
         - Log in with your Apple ID in Xcode first
         - Ensure you have a valid unique Bundle ID
         - Register your device with your Apple Developer Account
         - Let Xcode automatically provision a profile for your app
  4- Build or run your project again''';

final RegExp _securityFindIdentityDeveloperIdentityExtractionPattern = RegExp(
  r'^\s*\d+\).+"(.+Develop(ment|er).+)"$',
);
final RegExp _securityFindIdentityCertificateCnExtractionPattern = RegExp(r'.*\(([a-zA-Z0-9]+)\)');
final RegExp _certificateOrganizationalUnitExtractionPattern = RegExp(r'OU=([a-zA-Z0-9]+)');

/// Given a [BuildableIOSApp], this will try to find valid development code
/// signing identities in the user's keychain prompting a choice if multiple
/// are found.
///
/// Returns a set of build configuration settings that uses the selected
/// signing identities.
///
/// Will return null if none are found, if the user cancels or if the Xcode
/// project has a development team set in the project's build settings.
Future<Map<String, String>?> getCodeSigningIdentityDevelopmentTeamBuildSetting({
  required Map<String, String> buildSettings,
  required ProcessManager processManager,
  required Platform platform,
  required Logger logger,
  required Config config,
  required Terminal terminal,
}) async {
  // If the user already has it set in the project build settings itself,
  // continue with that.
  if (_isNotEmpty(buildSettings[_developmentTeamBuildSettingName])) {
    logger.printStatus(
      'Automatically signing iOS for device deployment using specified development '
      'team in Xcode project: ${buildSettings[_developmentTeamBuildSettingName]}',
    );
    return null;
  }

  if (_isNotEmpty(buildSettings['PROVISIONING_PROFILE'])) {
    return null;
  }

  final String? developmentTeam = await _getCodeSigningIdentityDevelopmentTeam(
    processManager: processManager,
    platform: platform,
    logger: logger,
    config: config,
    terminal: terminal,
    shouldExitOnNoCerts: true,
  );

  if (developmentTeam == null) {
    return null;
  }

  return <String, String>{_developmentTeamBuildSettingName: developmentTeam};
}

Future<String?> getCodeSigningIdentityDevelopmentTeam({
  required ProcessManager processManager,
  required Platform platform,
  required Logger logger,
  required Config config,
  required Terminal terminal,
}) async => _getCodeSigningIdentityDevelopmentTeam(
  processManager: processManager,
  platform: platform,
  logger: logger,
  config: config,
  terminal: terminal,
);

/// Set [shouldExitOnNoCerts] to show instructions for how to add a cert when none are found, then [toolExit].
Future<String?> _getCodeSigningIdentityDevelopmentTeam({
  required ProcessManager processManager,
  required Platform platform,
  required Logger logger,
  required Config config,
  required Terminal terminal,
  bool shouldExitOnNoCerts = false,
}) async {
  if (!platform.isMacOS) {
    return null;
  }

  // If the user's environment is missing the tools needed to find and read
  // certificates, abandon. Tools should be pre-equipped on macOS.
  final ProcessUtils processUtils = ProcessUtils(processManager: processManager, logger: logger);
  if (!await processUtils.exitsHappy(const <String>['which', 'security']) ||
      !await processUtils.exitsHappy(const <String>['which', 'openssl'])) {
    return null;
  }

  const List<String> findIdentityCommand = <String>[
    'security',
    'find-identity',
    '-p',
    'codesigning',
    '-v',
  ];

  String findIdentityStdout;
  try {
    findIdentityStdout =
        (await processUtils.run(findIdentityCommand, throwOnError: true)).stdout.trim();
  } on ProcessException catch (error) {
    logger.printTrace('Unexpected failure from find-identity: $error.');
    return null;
  }

  final List<String> validCodeSigningIdentities =
      findIdentityStdout
          .split('\n')
          .map<String?>((String outputLine) {
            return _securityFindIdentityDeveloperIdentityExtractionPattern
                .firstMatch(outputLine)
                ?.group(1);
          })
          .where(_isNotEmpty)
          .whereType<String>()
          .toSet() // Unique.
          .toList();

  final String? signingIdentity = await _chooseSigningIdentity(
    validCodeSigningIdentities,
    logger,
    config,
    terminal,
    shouldExitOnNoCerts,
  );

  // If none are chosen, return null.
  if (signingIdentity == null) {
    return null;
  }

  logger.printStatus('Developer identity "$signingIdentity" selected for iOS code signing');

  final String? signingCertificateId = _securityFindIdentityCertificateCnExtractionPattern
      .firstMatch(signingIdentity)
      ?.group(1);

  // If `security`'s output format changes, we'd have to update the above regex.
  if (signingCertificateId == null) {
    return null;
  }

  String signingCertificateStdout;
  try {
    signingCertificateStdout =
        (await processUtils.run(<String>[
          'security',
          'find-certificate',
          '-c',
          signingCertificateId,
          '-p',
        ], throwOnError: true)).stdout.trim();
  } on ProcessException catch (error) {
    logger.printTrace("Couldn't find the certificate: $error.");
    return null;
  }

  final Process opensslProcess = await processUtils.start(const <String>[
    'openssl',
    'x509',
    '-subject',
  ]);

  await ProcessUtils.writeToStdinGuarded(
    stdin: opensslProcess.stdin,
    content: signingCertificateStdout,
    onError: (Object? error, _) {
      throw Exception('Unexpected error when writing to openssl: $error');
    },
  );
  await opensslProcess.stdin.close();

  final String opensslOutput = await utf8.decodeStream(opensslProcess.stdout);
  // Fire and forget discard of the stderr stream so we don't hold onto resources.
  // Don't care about the result.
  unawaited(opensslProcess.stderr.drain<String?>());

  if (await opensslProcess.exitCode != 0) {
    return null;
  }

  return _certificateOrganizationalUnitExtractionPattern.firstMatch(opensslOutput)?.group(1);
}

/// Set [shouldExitOnNoCerts] to show instructions for how to add a cert when none are found, then [toolExit].
Future<String?> _chooseSigningIdentity(
  List<String> validCodeSigningIdentities,
  Logger logger,
  Config config,
  Terminal terminal,
  bool shouldExitOnNoCerts,
) async {
  // The user has no valid code signing identities.
  if (validCodeSigningIdentities.isEmpty) {
    if (shouldExitOnNoCerts) {
      logger.printError(noCertificatesInstruction, emphasis: true);
      throwToolExit('No development certificates available to code sign app for device deployment');
    } else {
      return null;
    }
  }

  if (validCodeSigningIdentities.length == 1) {
    return validCodeSigningIdentities.first;
  }

  if (validCodeSigningIdentities.length > 1) {
    final String? savedCertChoice = config.getValue('ios-signing-cert') as String?;

    if (savedCertChoice != null) {
      if (validCodeSigningIdentities.contains(savedCertChoice)) {
        logger.printStatus(
          'Found saved certificate choice "$savedCertChoice". To clear, use "flutter config".',
        );
        return savedCertChoice;
      } else {
        logger.printError(
          'Saved signing certificate "$savedCertChoice" is not a valid development certificate',
        );
      }
    }

    // If terminal UI can't be used, just attempt with the first valid certificate
    // since we can't ask the user.
    if (!terminal.usesTerminalUi) {
      return validCodeSigningIdentities.first;
    }

    final int count = validCodeSigningIdentities.length;
    logger.printStatus(
      'Multiple valid development certificates available (your choice will be saved):',
      emphasis: true,
    );
    for (int i = 0; i < count; i++) {
      logger.printStatus('  ${i + 1}) ${validCodeSigningIdentities[i]}', emphasis: true);
    }
    logger.printStatus('  a) Abort', emphasis: true);

    final String choice = await terminal.promptForCharInput(
      List<String>.generate(count, (int number) => '${number + 1}')..add('a'),
      prompt: 'Please select a certificate for code signing',
      defaultChoiceIndex: 0, // Just pressing enter chooses the first one.
      logger: logger,
    );

    if (choice == 'a') {
      throwToolExit('Aborted. Code signing is required to build a deployable iOS app.');
    } else {
      final String selectedCert = validCodeSigningIdentities[int.parse(choice) - 1];
      logger.printStatus('Certificate choice "$selectedCert" saved');
      config.setValue('ios-signing-cert', selectedCert);
      return selectedCert;
    }
  }

  return null;
}

/// Returns true if s is a not empty string.
bool _isNotEmpty(String? s) => s != null && s.isNotEmpty;
