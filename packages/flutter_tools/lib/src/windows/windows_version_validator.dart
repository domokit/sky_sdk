// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:process/process.dart';

import '../base/io.dart';
import '../base/logger.dart';
import '../base/os.dart';
import '../doctor_validator.dart';

/// Flutter only supports development on Windows host machines version 10 and greater.
const List<String> kUnsupportedVersions = <String>['6', '7', '8'];

/// Regex pattern for identifying line from systeminfo stdout with windows version
/// (ie. 10.0.22631.4037)
const String kWindowsOSVersionSemVerPattern = r'([0-9]+)\.([0-9]+)\.([0-9]+)\.?([0-9\.]+)?';

/// Regex pattern for identifying a running instance of the Topaz OFD process.
/// This is a known process that interferes with the build toolchain.
/// See https://github.com/flutter/flutter/issues/121366
const String kCoreProcessPattern = r'Topaz\s+OFD\\Warsaw\\core\.exe';

/// Validator for supported Windows host machine operating system version.
class WindowsVersionValidator extends DoctorValidator {
  WindowsVersionValidator({
    required OperatingSystemUtils operatingSystemUtils,
    required ProcessLister processLister,
    required WindowsVersionExtractor versionExtractor,
  }) : _operatingSystemUtils = operatingSystemUtils,
       _processLister = processLister,
       _versionExtractor = versionExtractor,
       super('Windows Version');

  // See https://learn.microsoft.com/en-us/windows/release-health/windows11-release-information
  static const int _lowestWindows11BuildNumber = 22000;

  final OperatingSystemUtils _operatingSystemUtils;
  final ProcessLister _processLister;
  final WindowsVersionExtractor _versionExtractor;

  Future<ValidationResult> _topazScan() async {
    if (!_processLister.canRunPowershell()) {
      return ValidationResult(ValidationType.missing, const <ValidationMessage>[
        ValidationMessage.hint(
          'Failed to find ${ProcessLister.powershell} or ${ProcessLister.pwsh} on PATH',
        ),
      ]);
    }
    final ProcessResult getProcessesResult = await _processLister.getProcessesWithPath();
    if (getProcessesResult.exitCode != 0) {
      return ValidationResult(ValidationType.missing, const <ValidationMessage>[
        ValidationMessage.hint('Get-Process failed to complete'),
      ]);
    }
    final RegExp topazRegex = RegExp(kCoreProcessPattern, caseSensitive: false, multiLine: true);
    final String processes = getProcessesResult.stdout as String;
    final bool topazFound = topazRegex.hasMatch(processes);
    if (topazFound) {
      return ValidationResult(ValidationType.missing, const <ValidationMessage>[
        ValidationMessage.hint(
          'The Topaz OFD Security Module was detected on your machine. '
          'You may need to disable it to build Flutter applications.',
        ),
      ]);
    }
    return ValidationResult(ValidationType.success, const <ValidationMessage>[]);
  }

  @override
  Future<ValidationResult> validateImpl() async {
    final RegExp regex = RegExp(kWindowsOSVersionSemVerPattern, multiLine: true);
    final String commandResult = _operatingSystemUtils.name;
    final Iterable<RegExpMatch> matches = regex.allMatches(commandResult);

    // Use the string split method to extract the major version
    // and check against the [kUnsupportedVersions] list
    ValidationType windowsVersionStatus;
    final List<ValidationMessage> messages = <ValidationMessage>[];
    String statusInfo;
    if (matches.length == 1 && !kUnsupportedVersions.contains(matches.elementAt(0).group(1))) {
      windowsVersionStatus = ValidationType.success;
      final WindowsVersionExtractionResult details = await _versionExtractor.getDetails();
      String? caption = details.caption;
      if (caption == null || caption.isEmpty) {
        final bool isWindows11 =
            int.parse(matches.elementAt(0).group(3)!) > _lowestWindows11BuildNumber;
        if (isWindows11) {
          caption = 'Windows 11 or higher';
        } else {
          caption = 'Windows 10';
        }
      }
      statusInfo = <String?>[
        caption,
        details.displayVersion,
        details.releaseId,
      ].where((String? part) => part != null).join(', ');

      // Check if the Topaz OFD security module is running, and warn the user if it is.
      // See https://github.com/flutter/flutter/issues/121366
      final List<ValidationResult> subResults = <ValidationResult>[await _topazScan()];
      for (final ValidationResult subResult in subResults) {
        if (subResult.type != ValidationType.success) {
          statusInfo = 'Problem detected with Windows installation';
          windowsVersionStatus = ValidationType.partial;
          messages.addAll(subResult.messages);
        }
      }
    } else {
      windowsVersionStatus = ValidationType.missing;
      statusInfo = 'Unable to determine Windows version (command `ver` returned $commandResult)';
    }

    return ValidationResult(windowsVersionStatus, messages, statusInfo: statusInfo);
  }
}

class ProcessLister {
  ProcessLister(this.processManager);

  final ProcessManager processManager;

  static const String powershell = 'powershell';
  static const String pwsh = 'pwsh';

  bool canRunPowershell() {
    return processManager.canRun(powershell) || processManager.canRun(pwsh);
  }

  Future<ProcessResult> getProcessesWithPath() async {
    const String argument = 'Get-Process | Format-List Path';
    const List<String> psArgs = <String>['-command', argument];
    if (processManager.canRun(powershell)) {
      return processManager.run(<String>[powershell, ...psArgs]);
    }
    if (processManager.canRun(pwsh)) {
      return processManager.run(<String>[pwsh, ...psArgs]);
    }
    throw StateError('Failed to find $powershell or $pwsh on PATH');
  }
}

/// This helper class takes the Windows edition and processor architecture from the Windows management interface (WMI)
/// with the wmic command. The Windows ReleaseId and DisplayVersion are taken from the registry key
/// HKLM\SOFTWARE\Microsoft\Windows NT\CurrentVersion via the reg command. The extracted data are caption (consists of
/// the edition and the processor architecture), releaseId and displayVersion and are returned via the
/// [WindowsVersionExtractionResult] class.
class WindowsVersionExtractor {
  WindowsVersionExtractor({required ProcessManager processManager, required Logger logger})
    : _logger = logger,
      _processManager = processManager;

  final ProcessManager _processManager;
  final Logger _logger;

  Future<WindowsVersionExtractionResult> getDetails() async {
    String? caption, releaseId, displayVersion;
    try {
      final ProcessResult captionResult = await _processManager.run(<String>[
        'wmic',
        'os',
        'get',
        'Caption,OSArchitecture',
      ]);

      if (captionResult.exitCode == 0) {
        final String? output = captionResult.stdout as String?;
        if (output != null) {
          final List<String> parts = output.split('\n');
          if (parts.length >= 2) {
            caption = parts[1].replaceAll('Microsoft Windows', '').replaceAll('  ', ' ').trim();
          }
        }
      }
    } on ProcessException catch (e) {
      _logger.printTrace('Failed to get Caption and OSArchitecture from WMI: $e');
    }

    try {
      final ProcessResult osDetails = await _processManager.run(<String>[
        'reg',
        'query',
        r'HKLM\SOFTWARE\Microsoft\Windows NT\CurrentVersion',
        '/t',
        'REG_SZ',
      ]);

      if (osDetails.exitCode == 0) {
        final String? output = osDetails.stdout as String?;
        if (output != null) {
          final Map<String, String> data = Map<String, String>.fromEntries(
            output.split('\n').where((String line) => line.contains('REG_SZ')).map((String line) {
              final List<String> parts = line.split('REG_SZ');
              return MapEntry<String, String>(parts.first.trim(), parts.last.trim());
            }),
          );
          releaseId = data['ReleaseId'];
          displayVersion = data['DisplayVersion'];
        }
      }
    } on ProcessException catch (e) {
      _logger.printTrace('Failed to get ReleaseId and DisplayVersion from registry: $e');
    }

    return WindowsVersionExtractionResult(
      caption: caption,
      releaseId: releaseId,
      displayVersion: displayVersion,
    );
  }
}

/// The result of the Windows version extraction. Typically values would be for [caption] e.g. "11 Pro 64-bit", for
/// [releaseId] e.g. "2009" and for [displayVersion] e.g. "22H2". All values could be null when the extraction fails.
final class WindowsVersionExtractionResult {
  WindowsVersionExtractionResult({
    required this.caption,
    required this.releaseId,
    required this.displayVersion,
  });

  factory WindowsVersionExtractionResult.empty() {
    return WindowsVersionExtractionResult(caption: null, releaseId: null, displayVersion: null);
  }

  final String? caption;
  final String? releaseId;
  final String? displayVersion;
}
