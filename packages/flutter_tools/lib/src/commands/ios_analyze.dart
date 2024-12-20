// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import '../base/logger.dart';
import '../convert.dart';
import '../ios/xcodeproj.dart';
import '../project.dart';

/// The type of analysis to perform.
enum IOSAnalyzeOption {
  /// Prints out available build variants of the iOS Xcode sub-project.
  ///
  /// An example output:
  ///
  /// {"configurations":["Debug","Release","Profile"],"targets":["Runner","RunnerTests"]}
  listBuildOptions,

  /// Outputs universal link settings of the iOS Xcode sub-project into a file.
  ///
  /// The file path will be printed after the command is run successfully.
  outputUniversalLinkSettings,
}

/// Analyze the iOS Xcode sub-project of a Flutter project.
///
/// The [userPath] must be point to a flutter project.
class IOSAnalyze {
  IOSAnalyze({
    required this.project,
    required this.option,
    this.configuration,
    this.target,
    required this.logger,
  }) : assert(
         option == IOSAnalyzeOption.listBuildOptions || (configuration != null && target != null),
       );

  final FlutterProject project;
  final IOSAnalyzeOption option;
  final String? configuration;
  final String? target;
  final Logger logger;

  Future<void> analyze() async {
    switch (option) {
      case IOSAnalyzeOption.listBuildOptions:
        final XcodeProjectInfo? info = await project.ios.projectInfo();
        final Map<String, List<String>> result;
        if (info == null) {
          result = const <String, List<String>>{};
        } else {
          result = <String, List<String>>{
            'configurations': info.buildConfigurations,
            'targets': info.targets,
          };
        }
        logger.printStatus(jsonEncode(result));
      case IOSAnalyzeOption.outputUniversalLinkSettings:
        final String filePath = await project.ios.outputsUniversalLinkSettings(
          configuration: configuration!,
          target: target!,
        );
        logger.printStatus('result saved in $filePath');
    }
  }
}
