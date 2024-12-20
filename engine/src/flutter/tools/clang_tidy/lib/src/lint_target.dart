// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Describes what should be linted by the Clang Tidy tool.
sealed class LintTarget {
  /// Creates a new [LintTarget].
  const LintTarget();
}

/// Lints all files in the project.
final class LintAll extends LintTarget {
  /// Defines a lint target that lints all files in the project.
  const LintAll();
}

/// Lint all files that have changed since the last commit.
///
/// This considers only the last commit, not all commits in the current branch.
final class LintChanged extends LintTarget {
  /// Defines a lint target of files that have changed since the last commit.
  const LintChanged();
}

/// Lint all files that have changed compared to HEAD.
///
/// This considers _all_ commits in the current branch, not just the last one.
final class LintHead extends LintTarget {
  /// Defines a lint target of files that have changed compared to HEAD.
  const LintHead();
}

/// Lint all files whose paths match the given regex.
final class LintRegex extends LintTarget {
  /// Creates a new [LintRegex] with the given [regex].
  const LintRegex(this.regex);

  /// The regular expression to match against file paths.
  final String regex;
}
