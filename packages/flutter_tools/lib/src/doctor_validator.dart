// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import 'base/async_guard.dart';
import 'base/terminal.dart';
import 'globals.dart' as globals;

class ValidatorTask {
  ValidatorTask(this.validator, this.result);
  final DoctorValidator validator;
  final Future<ValidationResult> result;
}

/// A series of tools and required install steps for a target platform (iOS or Android).
abstract class Workflow {
  const Workflow();

  /// Whether the workflow applies to this platform (as in, should we ever try and use it).
  bool get appliesToHostPlatform;

  /// Are we functional enough to list devices?
  bool get canListDevices;

  /// Could this thing launch *something*? It may still have minor issues.
  bool get canLaunchDevices;

  /// Are we functional enough to list emulators?
  bool get canListEmulators;
}

enum ValidationType { crash, missing, partial, notAvailable, success }

enum ValidationMessageType { error, hint, information }

abstract class DoctorValidator {
  const DoctorValidator(this.title);

  /// This is displayed in the CLI.
  final String title;

  String get slowWarning => 'This is taking an unexpectedly long time...';

  static const Duration _slowWarningDuration = Duration(seconds: 10);

  /// Duration before the spinner should display [slowWarning].
  Duration get slowWarningDuration => _slowWarningDuration;

  Future<ValidationResult> validate();
}

/// A validator that runs other [DoctorValidator]s and combines their output
/// into a single [ValidationResult]. It uses the title of the first validator
/// passed to the constructor and reports the statusInfo of the first validator
/// that provides one. Other titles and statusInfo strings are discarded.
class GroupedValidator extends DoctorValidator {
  GroupedValidator(this.subValidators) : super(subValidators[0].title);

  final List<DoctorValidator> subValidators;

  List<ValidationResult> _subResults = <ValidationResult>[];

  /// Sub-validator results.
  ///
  /// To avoid losing information when results are merged, the sub-results are
  /// cached on this field when they are available. The results are in the same
  /// order as the sub-validator list.
  List<ValidationResult> get subResults => _subResults;

  @override
  String get slowWarning => _currentSlowWarning;
  String _currentSlowWarning = 'Initializing...';

  @override
  Future<ValidationResult> validate() async {
    final List<ValidatorTask> tasks = <ValidatorTask>[
      for (final DoctorValidator validator in subValidators)
        ValidatorTask(validator, asyncGuard<ValidationResult>(() => validator.validate())),
    ];

    final List<ValidationResult> results = <ValidationResult>[];
    for (final ValidatorTask subValidator in tasks) {
      _currentSlowWarning = subValidator.validator.slowWarning;
      try {
        results.add(await subValidator.result);
      } on Exception catch (exception, stackTrace) {
        results.add(ValidationResult.crash(exception, stackTrace));
      }
    }
    _currentSlowWarning = 'Merging results...';
    return _mergeValidationResults(results);
  }

  ValidationResult _mergeValidationResults(List<ValidationResult> results) {
    assert(results.isNotEmpty, 'Validation results should not be empty');
    _subResults = results;
    ValidationType mergedType = results[0].type;
    final List<ValidationMessage> mergedMessages = <ValidationMessage>[];
    String? statusInfo;

    for (final ValidationResult result in results) {
      statusInfo ??= result.statusInfo;
      switch (result.type) {
        case ValidationType.success:
          if (mergedType == ValidationType.missing) {
            mergedType = ValidationType.partial;
          }
        case ValidationType.notAvailable:
        case ValidationType.partial:
          mergedType = ValidationType.partial;
        case ValidationType.crash:
        case ValidationType.missing:
          if (mergedType == ValidationType.success) {
            mergedType = ValidationType.partial;
          }
      }
      mergedMessages.addAll(result.messages);
    }

    return ValidationResult(mergedType, mergedMessages, statusInfo: statusInfo);
  }
}

@immutable
class ValidationResult {
  /// [ValidationResult.type] should only equal [ValidationResult.success]
  /// if no [messages] are hints or errors.
  const ValidationResult(this.type, this.messages, {this.statusInfo});

  factory ValidationResult.crash(Object error, [StackTrace? stackTrace]) {
    return ValidationResult(ValidationType.crash, <ValidationMessage>[
      const ValidationMessage.error(
        'Due to an error, the doctor check did not complete. '
        'If the error message below is not helpful, '
        'please let us know about this issue at https://github.com/flutter/flutter/issues.',
      ),
      ValidationMessage.error('$error'),
      if (stackTrace != null)
        // Stacktrace is informational. Printed in verbose mode only.
        ValidationMessage('$stackTrace'),
    ], statusInfo: 'the doctor check crashed');
  }

  final ValidationType type;
  // A short message about the status.
  final String? statusInfo;
  final List<ValidationMessage> messages;

  String get leadingBox => switch (type) {
    ValidationType.crash => '[☠]',
    ValidationType.missing => '[✗]',
    ValidationType.success => '[✓]',
    ValidationType.notAvailable || ValidationType.partial => '[!]',
  };

  String get coloredLeadingBox {
    return globals.terminal.color(leadingBox, switch (type) {
      ValidationType.success => TerminalColor.green,
      ValidationType.crash || ValidationType.missing => TerminalColor.red,
      ValidationType.notAvailable || ValidationType.partial => TerminalColor.yellow,
    });
  }

  /// The string representation of the type.
  String get typeStr => switch (type) {
    ValidationType.crash => 'crash',
    ValidationType.missing => 'missing',
    ValidationType.success => 'installed',
    ValidationType.notAvailable => 'notAvailable',
    ValidationType.partial => 'partial',
  };

  @override
  String toString() {
    return '$runtimeType($type, $messages, $statusInfo)';
  }
}

/// A status line for the flutter doctor validation to display.
///
/// The [message] is required and represents either an informational statement
/// about the particular doctor validation that passed, or more context
/// on the cause and/or solution to the validation failure.
@immutable
class ValidationMessage {
  /// Create a validation message with information for a passing validator.
  ///
  /// By default this is not displayed unless the doctor is run in
  /// verbose mode.
  ///
  /// The [contextUrl] may be supplied to link to external resources. This
  /// is displayed after the informative message in verbose modes.
  const ValidationMessage(this.message, {this.contextUrl, String? piiStrippedMessage})
    : type = ValidationMessageType.information,
      piiStrippedMessage = piiStrippedMessage ?? message;

  /// Create a validation message with information for a failing validator.
  const ValidationMessage.error(this.message, {String? piiStrippedMessage})
    : type = ValidationMessageType.error,
      piiStrippedMessage = piiStrippedMessage ?? message,
      contextUrl = null;

  /// Create a validation message with information for a partially failing
  /// validator.
  const ValidationMessage.hint(this.message, {String? piiStrippedMessage})
    : type = ValidationMessageType.hint,
      piiStrippedMessage = piiStrippedMessage ?? message,
      contextUrl = null;

  final ValidationMessageType type;
  final String? contextUrl;
  final String message;

  /// Optional message with PII stripped, to show instead of [message].
  final String piiStrippedMessage;

  bool get isError => type == ValidationMessageType.error;

  bool get isHint => type == ValidationMessageType.hint;

  bool get isInformation => type == ValidationMessageType.information;

  String get indicator => switch (type) {
    ValidationMessageType.error => '✗',
    ValidationMessageType.hint => '!',
    ValidationMessageType.information => '•',
  };

  String get coloredIndicator {
    return globals.terminal.color(indicator, switch (type) {
      ValidationMessageType.error => TerminalColor.red,
      ValidationMessageType.hint => TerminalColor.yellow,
      ValidationMessageType.information => TerminalColor.green,
    });
  }

  @override
  String toString() => message;

  @override
  bool operator ==(Object other) {
    return other is ValidationMessage &&
        other.message == message &&
        other.type == type &&
        other.contextUrl == contextUrl;
  }

  @override
  int get hashCode => Object.hash(type, message, contextUrl);
}

class NoIdeValidator extends DoctorValidator {
  NoIdeValidator() : super('Flutter IDE Support');

  @override
  Future<ValidationResult> validate() async {
    return ValidationResult(
      // Info hint to user they do not have a supported IDE installed
      ValidationType.notAvailable,
      globals.userMessages.noIdeInstallationInfo
          .map((String ideInfo) => ValidationMessage(ideInfo))
          .toList(),
      statusInfo: globals.userMessages.noIdeStatusInfo,
    );
  }
}

class ValidatorWithResult extends DoctorValidator {
  ValidatorWithResult(super.title, this.result);

  final ValidationResult result;

  @override
  Future<ValidationResult> validate() async => result;
}
