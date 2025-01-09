// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

/// An overridable collection of values provided by the environment.
@immutable
final class Environment {
  /// Creates a new environment from the given values.
  const Environment({required this.isCi, required this.showVerbose, required this.logsDir});

  /// Whether the current program is running on a CI environment.
  ///
  /// Useful for determining if certain features should be enabled or disabled
  /// based on the environment, or to add safety checks (for example, using
  /// confusing or ambiguous flags).
  final bool isCi;

  /// Whether the user has requested verbose logging and program output.
  final bool showVerbose;

  /// What directory to store logs and screenshots in.
  final String? logsDir;

  @override
  bool operator ==(Object o) {
    return o is Environment &&
        o.isCi == isCi &&
        o.showVerbose == showVerbose &&
        o.logsDir == logsDir;
  }

  @override
  int get hashCode => Object.hash(isCi, showVerbose, logsDir);

  @override
  String toString() {
    return 'Environment(isCi: $isCi, showVerbose: $showVerbose, logsDir: $logsDir)';
  }
}
