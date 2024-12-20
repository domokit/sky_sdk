// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import '../base/file_system.dart';
import '../base/logger.dart';
import '../convert.dart';
import '../project.dart';

/// The type of analysis to perform.
enum AndroidAnalyzeOption {
  /// Prints out available build variants of the Android sub-project.
  ///
  /// An example output:
  /// ["debug", "profile", "release"]
  listBuildVariant,

  /// Outputs app link settings of the Android sub-project into a file.
  ///
  /// The file path will be printed after the command is run successfully.
  outputAppLinkSettings,
}

/// Analyze the Android sub-project of a Flutter project.
///
/// The [userPath] must be point to a flutter project.
class AndroidAnalyze {
  AndroidAnalyze({
    required this.fileSystem,
    required this.option,
    required this.userPath,
    this.buildVariant,
    required this.logger,
  }) : assert(option == AndroidAnalyzeOption.listBuildVariant || buildVariant != null);

  final FileSystem fileSystem;
  final AndroidAnalyzeOption option;
  final String? buildVariant;
  final String userPath;
  final Logger logger;

  Future<void> analyze() async {
    final FlutterProject project = FlutterProject.fromDirectory(fileSystem.directory(userPath));
    switch (option) {
      case AndroidAnalyzeOption.listBuildVariant:
        logger.printStatus(jsonEncode(await project.android.getBuildVariants()));
      case AndroidAnalyzeOption.outputAppLinkSettings:
        assert(buildVariant != null);
        final String filePath = await project.android.outputsAppLinkSettings(
          variant: buildVariant!,
        );
        logger.printStatus('result saved in $filePath');
    }
  }
}
