// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:path/path.dart' show Context;

/// Possible states of the Flutter repo checkout after Gradle lockfile
/// regeneration.
sealed class CheckoutStatePostGradleRegeneration {
  factory CheckoutStatePostGradleRegeneration(String gitStatusOutput, Context context) {
    gitStatusOutput = gitStatusOutput.trim();
    if (gitStatusOutput.isEmpty) {
      return const NoDiff();
    }

    final List<String> changes = gitStatusOutput.split('\n');
    final List<String> changedPaths = <String>[];
    for (final String line in changes) {
      final RegExpMatch? match = pattern.firstMatch(line);
      if (match == null) {
        return MalformedLine(line);
      }
      changedPaths.add(match.group(1)!);
    }

    final List<String> nonLockfileDiffs =
        changedPaths.where((String path) {
          final String extension = context.extension(path);
          return extension != '.lockfile';
        }).toList();

    if (nonLockfileDiffs.isNotEmpty) {
      return NonLockfileChanges(nonLockfileDiffs);
    }

    return const OnlyLockfileChanges();
  }

  /// Output format for `git status --porcelain` and `git status --short`.
  ///
  /// The first capture group is the path to the file or directory changed,
  /// relative to the root of the repository.
  ///
  /// See `man git-status` for more reference.
  static final RegExp pattern = RegExp(r'[ACDMRTU ]{1,2} (\S+)');
}

/// No files were changed, no commit needed.
final class NoDiff implements CheckoutStatePostGradleRegeneration {
  const NoDiff();
}

/// Only files ending in *.lockfile were changed; changes can be committed.
final class OnlyLockfileChanges implements CheckoutStatePostGradleRegeneration {
  const OnlyLockfileChanges();
}

/// There are changed files that do not end in *.lockfile; fail the script.
///
/// Because the script to regenerate Gradle lockfiles triggers a Gradle build,
/// and because the packages_autoroller can have its PRs merged without a
/// human review, we are conservative about what changes we commit.
final class NonLockfileChanges implements CheckoutStatePostGradleRegeneration {
  const NonLockfileChanges(this.changes);

  final List<String> changes;
}

/// A line in the output of `git status` does not match the expected pattern;
/// fail the script.
///
/// This likely means there is a bug in the regular expression, and it needs
/// to be updated.
final class MalformedLine implements CheckoutStatePostGradleRegeneration {
  const MalformedLine(this.line);

  final String line;
}
