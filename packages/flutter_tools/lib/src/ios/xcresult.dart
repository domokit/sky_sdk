// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

import '../../src/base/process.dart';
import '../../src/convert.dart' show json;
import '../../src/macos/xcode.dart';
import '../base/version.dart';
import '../convert.dart';

/// The generator of xcresults.
///
/// Call [generate] after an iOS/MacOS build will generate a [XCResult].
/// This only works when the `-resultBundleVersion` is set to 3.
/// * See also: [XCResult].
class XCResultGenerator {
  /// Construct the [XCResultGenerator].
  XCResultGenerator({required this.resultPath, required this.xcode, required this.processUtils});

  /// The file path that used to store the xcrun result.
  ///
  /// There's usually a `resultPath.xcresult` file in the same folder.
  final String resultPath;

  /// The [ProcessUtils] to run commands.
  final ProcessUtils processUtils;

  /// [Xcode] object used to run xcode command.
  final Xcode xcode;

  /// Generates the XCResult.
  ///
  /// Calls `xcrun xcresulttool get --legacy --path <resultPath> --format json`,
  /// then stores the useful information the json into an [XCResult] object.
  ///
  /// A`issueDiscarders` can be passed to discard any issues that matches the description of any [XCResultIssueDiscarder] in the list.
  Future<XCResult> generate({
    List<XCResultIssueDiscarder> issueDiscarders = const <XCResultIssueDiscarder>[],
  }) async {
    final Version? xcodeVersion = xcode.currentVersion;
    final RunResult result = await processUtils.run(<String>[
      ...xcode.xcrunCommand(),
      'xcresulttool',
      'get',
      // See https://github.com/flutter/flutter/issues/151502
      if (xcodeVersion != null && xcodeVersion >= Version(16, 0, 0)) '--legacy',
      '--path',
      resultPath,
      '--format',
      'json',
    ]);
    if (result.exitCode != 0) {
      return XCResult.failed(errorMessage: result.stderr);
    }
    if (result.stdout.isEmpty) {
      return XCResult.failed(errorMessage: 'xcresult parser: Unrecognized top level json format.');
    }
    final Object? resultJson = json.decode(result.stdout);
    if (resultJson == null || resultJson is! Map<String, Object?>) {
      // If json parsing failed, indicate such error.
      // This also includes the top level json object is an array, which indicates
      // the structure of the json is changed and this parser class possibly needs to update for this change.
      return XCResult.failed(errorMessage: 'xcresult parser: Unrecognized top level json format.');
    }
    return XCResult(resultJson: resultJson, issueDiscarders: issueDiscarders);
  }
}

/// The xcresult of an `xcodebuild` command.
///
/// This is the result from an `xcrun xcresulttool get --legacy --path <resultPath> --format json` run.
/// The result contains useful information such as build errors and warnings.
class XCResult {
  /// Parse the `resultJson` and stores useful information in the returned `XCResult`.
  factory XCResult({
    required Map<String, Object?> resultJson,
    List<XCResultIssueDiscarder> issueDiscarders = const <XCResultIssueDiscarder>[],
  }) {
    final List<XCResultIssue> issues = <XCResultIssue>[];

    final Object? issuesMap = resultJson['issues'];
    if (issuesMap == null || issuesMap is! Map<String, Object?>) {
      return XCResult.failed(errorMessage: 'xcresult parser: Failed to parse the issues map.');
    }

    final Object? errorSummaries = issuesMap['errorSummaries'];
    if (errorSummaries is Map<String, Object?>) {
      issues.addAll(
        _parseIssuesFromIssueSummariesJson(
          type: XCResultIssueType.error,
          issueSummariesJson: errorSummaries,
          issueDiscarder: issueDiscarders,
        ),
      );
    }

    final Object? warningSummaries = issuesMap['warningSummaries'];
    if (warningSummaries is Map<String, Object?>) {
      issues.addAll(
        _parseIssuesFromIssueSummariesJson(
          type: XCResultIssueType.warning,
          issueSummariesJson: warningSummaries,
          issueDiscarder: issueDiscarders,
        ),
      );
    }

    final Object? actionsMap = resultJson['actions'];
    if (actionsMap is Map<String, Object?>) {
      final List<XCResultIssue> actionIssues = _parseActionIssues(
        actionsMap,
        issueDiscarders: issueDiscarders,
      );
      issues.addAll(actionIssues);
    }

    return XCResult._(issues: issues);
  }

  factory XCResult.failed({required String errorMessage}) {
    return XCResult._(parseSuccess: false, parsingErrorMessage: errorMessage);
  }

  /// Create a [XCResult] with constructed [XCResultIssue]s for testing.
  @visibleForTesting
  factory XCResult.test({
    List<XCResultIssue>? issues,
    bool? parseSuccess,
    String? parsingErrorMessage,
  }) {
    return XCResult._(
      issues: issues ?? const <XCResultIssue>[],
      parseSuccess: parseSuccess ?? true,
      parsingErrorMessage: parsingErrorMessage,
    );
  }

  XCResult._({
    this.issues = const <XCResultIssue>[],
    this.parseSuccess = true,
    this.parsingErrorMessage,
  });

  /// The issues in the xcresult file.
  final List<XCResultIssue> issues;

  /// Indicate if the xcresult was successfully parsed.
  ///
  /// See also: [parsingErrorMessage] for the error message if the parsing was unsuccessful.
  final bool parseSuccess;

  /// The error message describes why the parse if unsuccessful.
  ///
  /// This is `null` if [parseSuccess] is `true`.
  final String? parsingErrorMessage;
}

/// An issue object in the XCResult
class XCResultIssue {
  /// Construct an `XCResultIssue` object from `issueJson`.
  ///
  /// `issueJson` is the object at xcresultJson[['actions']['_values'][0]['buildResult']['issues']['errorSummaries'/'warningSummaries']['_values'].
  factory XCResultIssue({
    required XCResultIssueType type,
    required Map<String, Object?> issueJson,
  }) {
    // Parse type.
    final Object? issueSubTypeMap = issueJson['issueType'];
    String? subType;
    if (issueSubTypeMap is Map<String, Object?>) {
      final Object? subTypeValue = issueSubTypeMap['_value'];
      if (subTypeValue is String) {
        subType = subTypeValue;
      }
    }

    // Parse message.
    String? message;
    final Object? messageMap = issueJson['message'];
    if (messageMap is Map<String, Object?>) {
      final Object? messageValue = messageMap['_value'];
      if (messageValue is String) {
        message = messageValue;
      }
    }

    final List<String> warnings = <String>[];
    // Parse url and convert it to a location String.
    String? location;
    final Object? documentLocationInCreatingWorkspaceMap =
        issueJson['documentLocationInCreatingWorkspace'];
    if (documentLocationInCreatingWorkspaceMap is Map<String, Object?>) {
      final Object? urlMap = documentLocationInCreatingWorkspaceMap['url'];
      if (urlMap is Map<String, Object?>) {
        final Object? urlValue = urlMap['_value'];
        if (urlValue is String) {
          location = _convertUrlToLocationString(urlValue);
          if (location == null) {
            warnings.add(
              '(XCResult) The `url` exists but it was failed to be parsed. url: $urlValue',
            );
          }
        }
      }
    }

    return XCResultIssue._(
      type: type,
      subType: subType,
      message: message,
      location: location,
      warnings: warnings,
    );
  }

  /// Create a [XCResultIssue] without JSON parsing for testing.
  @visibleForTesting
  factory XCResultIssue.test({
    XCResultIssueType type = XCResultIssueType.error,
    String? subType,
    String? message,
    String? location,
    List<String> warnings = const <String>[],
  }) {
    return XCResultIssue._(
      type: type,
      subType: subType,
      message: message,
      location: location,
      warnings: warnings,
    );
  }

  XCResultIssue._({
    required this.type,
    required this.subType,
    required this.message,
    required this.location,
    required this.warnings,
  });

  /// The type of the issue.
  final XCResultIssueType type;

  /// The sub type of the issue.
  ///
  /// This is a more detailed category about the issue.
  /// The possible values are `Warning`, `Semantic Issue'` etc.
  final String? subType;

  /// Human readable message for the issue.
  ///
  /// This can be displayed to user for their information.
  final String? message;

  /// The location where the issue occurs.
  ///
  /// This is a re-formatted version of the "url" value in the json.
  /// The format looks like <FileLocation>:<StartingLineNumber>:<StartingColumnNumber>.
  final String? location;

  /// Warnings when constructing the issue object.
  final List<String> warnings;
}

/// The type of an `XCResultIssue`.
enum XCResultIssueType {
  /// The issue is an warning.
  ///
  /// This is for all the issues under the `warningSummaries` key in the xcresult.
  warning,

  /// The issue is an warning.
  ///
  /// This is for all the issues under the `errorSummaries` key in the xcresult.
  error,
}

/// Discards the [XCResultIssue] that matches any of the matchers.
class XCResultIssueDiscarder {
  XCResultIssueDiscarder({
    this.typeMatcher,
    this.subTypeMatcher,
    this.messageMatcher,
    this.locationMatcher,
  }) : assert(
         typeMatcher != null ||
             subTypeMatcher != null ||
             messageMatcher != null ||
             locationMatcher != null,
       );

  /// The type of the discarder.
  ///
  /// A [XCResultIssue] should be discarded if its `type` equals to this.
  final XCResultIssueType? typeMatcher;

  /// The subType of the discarder.
  ///
  /// A [XCResultIssue] should be discarded if its `subType` matches the RegExp.
  final RegExp? subTypeMatcher;

  /// The message of the discarder.
  ///
  /// A [XCResultIssue] should be discarded if its `message` matches the RegExp.
  final RegExp? messageMatcher;

  /// The location of the discarder.
  ///
  /// A [XCResultIssue] should be discarded if its `location` matches the RegExp.
  final RegExp? locationMatcher;
}

// A typical location url string looks like file:///foo.swift#CharacterRangeLen=0&EndingColumnNumber=82&EndingLineNumber=7&StartingColumnNumber=82&StartingLineNumber=7.
//
// This function converts it to something like: /foo.swift:<StartingLineNumber>:<StartingColumnNumber>.
String? _convertUrlToLocationString(String url) {
  final Uri? fragmentLocation = Uri.tryParse(url);
  if (fragmentLocation == null) {
    return null;
  }
  // Parse the fragment as a query of key-values:
  final Uri fileLocation = Uri(path: fragmentLocation.path, query: fragmentLocation.fragment);
  String startingLineNumber = fileLocation.queryParameters['StartingLineNumber'] ?? '';
  if (startingLineNumber.isNotEmpty) {
    startingLineNumber = ':$startingLineNumber';
  }
  String startingColumnNumber = fileLocation.queryParameters['StartingColumnNumber'] ?? '';
  if (startingColumnNumber.isNotEmpty) {
    startingColumnNumber = ':$startingColumnNumber';
  }
  return '${fileLocation.path}$startingLineNumber$startingColumnNumber';
}

// Determine if an `issue` should be discarded based on the `discarder`.
bool _shouldDiscardIssue({
  required XCResultIssue issue,
  required XCResultIssueDiscarder discarder,
}) {
  if (issue.type == discarder.typeMatcher) {
    return true;
  }
  if (issue.subType != null &&
      discarder.subTypeMatcher != null &&
      discarder.subTypeMatcher!.hasMatch(issue.subType!)) {
    return true;
  }
  if (issue.message != null &&
      discarder.messageMatcher != null &&
      discarder.messageMatcher!.hasMatch(issue.message!)) {
    return true;
  }
  if (issue.location != null &&
      discarder.locationMatcher != null &&
      discarder.locationMatcher!.hasMatch(issue.location!)) {
    return true;
  }

  return false;
}

List<XCResultIssue> _parseIssuesFromIssueSummariesJson({
  required XCResultIssueType type,
  required Map<String, Object?> issueSummariesJson,
  required List<XCResultIssueDiscarder> issueDiscarder,
}) {
  final List<XCResultIssue> issues = <XCResultIssue>[];
  final Object? errorsList = issueSummariesJson['_values'];
  if (errorsList is List<Object?>) {
    for (final Object? issueJson in errorsList) {
      if (issueJson == null || issueJson is! Map<String, Object?>) {
        continue;
      }
      final XCResultIssue resultIssue = XCResultIssue(type: type, issueJson: issueJson);
      bool discard = false;
      for (final XCResultIssueDiscarder discarder in issueDiscarder) {
        if (_shouldDiscardIssue(issue: resultIssue, discarder: discarder)) {
          discard = true;
          break;
        }
      }
      if (discard) {
        continue;
      }
      issues.add(resultIssue);
    }
  }
  return issues;
}

List<XCResultIssue> _parseActionIssues(
  Map<String, Object?> actionsMap, {
  required List<XCResultIssueDiscarder> issueDiscarders,
}) {
  // Example of json:
  // {
  //   "actions" : {
  //     "_values" : [
  //       {
  //         "actionResult" : {
  //           "_type" : {
  //             "_name" : "ActionResult"
  //           },
  //           "issues" : {
  //             "_type" : {
  //               "_name" : "ResultIssueSummaries"
  //             },
  //             "testFailureSummaries" : {
  //               "_type" : {
  //                 "_name" : "Array"
  //               },
  //               "_values" : [
  //                 {
  //                   "_type" : {
  //                     "_name" : "TestFailureIssueSummary",
  //                     "_supertype" : {
  //                       "_name" : "IssueSummary"
  //                     }
  //                   },
  //                   "issueType" : {
  //                     "_type" : {
  //                       "_name" : "String"
  //                     },
  //                     "_value" : "Uncategorized"
  //                   },
  //                   "message" : {
  //                     "_type" : {
  //                       "_name" : "String"
  //                     },
  //                     "_value" : "Unable to find a destination matching the provided destination specifier:\n\t\t{ id:1234D567-890C-1DA2-34E5-F6789A0123C4 }\n\n\tIneligible destinations for the \"Runner\" scheme:\n\t\t{ platform:iOS, id:dvtdevice-DVTiPhonePlaceholder-iphoneos:placeholder, name:Any iOS Device, error:iOS 17.0 is not installed. To use with Xcode, first download and install the platform }"
  //                   }
  //                 }
  //               ]
  //             }
  //           }
  //         }
  //       }
  //     ]
  //   }
  // }
  final List<XCResultIssue> issues = <XCResultIssue>[];
  final Object? actionsValues = actionsMap['_values'];
  if (actionsValues is! List<Object?>) {
    return issues;
  }

  for (final Object? actionValue in actionsValues) {
    if (actionValue is! Map<String, Object?>) {
      continue;
    }
    final Object? actionResult = actionValue['actionResult'];
    if (actionResult is! Map<String, Object?>) {
      continue;
    }
    final Object? actionResultIssues = actionResult['issues'];
    if (actionResultIssues is! Map<String, Object?>) {
      continue;
    }
    final Object? testFailureSummaries = actionResultIssues['testFailureSummaries'];
    if (testFailureSummaries is Map<String, Object?>) {
      issues.addAll(
        _parseIssuesFromIssueSummariesJson(
          type: XCResultIssueType.error,
          issueSummariesJson: testFailureSummaries,
          issueDiscarder: issueDiscarders,
        ),
      );
    }
  }

  return issues;
}
