// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@internal
library;

import 'package:meta/meta.dart';

/// Merges and returns the result of adding [extraArgs] to original [buildArgs].
///
/// A builder, for example `macos/ios_debug` might look like this (truncated):
/// ```json
/// {
///   "gn": [
///     "--ios",
///     "--runtime-mode",
///     "debug",
///     "--no-stripped",
///     "--no-lte"
///   ]
/// }
/// ```
///
/// Before asking GN to generate a build from these arguments, [extraArgs] are
/// added by ways of merging; that is, by assuming that arguments provided by
/// [extraArgs] are strictly boolean flags in the format "--foo" or "--no-foo",
/// and either are appended or replace an existing argument specified in
/// [buildArgs].
///
/// [extraArgs] that are not boolean arguments are not supported.
///
/// ## Example
///
/// ```dart
/// print(mergeGnArgs(
///   buildArgs: [],
///   extraArgs: ["--foo"]
/// ));
/// ```
///
/// ... prints "[--foo]".
///
/// ```dart
/// print(mergeGnArgs(
///   buildArgs: ["--foo"],
///   extraArgs: ["--bar"]
/// ));
/// ```
///
/// ... prints "[--foo, --bar]".
///
/// ```dart
/// print(mergeGnArgs(
///   buildArgs: ["--foo", "--no-bar", "--baz"],
///   extraArgs: ["--no-foo", "--bar"]
/// ));
/// ```
///
/// ... prints "[--no-foo, --bar, --baz]".
List<String> mergeGnArgs({required List<String> buildArgs, required List<String> extraArgs}) {
  // Make a copy of buildArgs so replacements can be made.
  final newBuildArgs = List.of(buildArgs);

  // Index "extraArgs" as map of "flag-without-no" => true/false.
  final indexedExtra = <String, bool>{};
  for (final extraArg in extraArgs) {
    if (!_isFlag(extraArg)) {
      throw ArgumentError.value(
        extraArgs,
        'extraArgs',
        'Each argument must be in the form "--flag" or "--no-flag".',
      );
    }
    final (name, value) = _extractRawFlag(extraArg);
    indexedExtra[name] = value;
  }

  // Iterate over newBuildArgs and replace if applicable.
  for (var i = 0; i < newBuildArgs.length; i++) {
    // It is valid to have non-flags (i.e. --runtime-mode=debug) here. Skip.
    final buildArg = newBuildArgs[i];
    if (!_isFlag(buildArg)) {
      continue;
    }

    // If there is no repalcement value, leave as-is.
    final (name, value) = _extractRawFlag(buildArg);
    final replaceWith = indexedExtra.remove(name);
    if (replaceWith == null) {
      continue;
    }

    // Replace (i.e. --foo with --no-foo or --no-foo with --foo).
    newBuildArgs[i] = _toFlag(name, replaceWith);
  }

  // Append arguments that were not replaced above.
  for (final MapEntry(key: name, value: value) in indexedExtra.entries) {
    newBuildArgs.add(_toFlag(name, value));
  }

  return newBuildArgs;
}

bool _isFlag(String arg) {
  return arg.startsWith('--') && !arg.contains('=') && !arg.contains(' ');
}

(String, bool) _extractRawFlag(String flagArgument) {
  var rawFlag = flagArgument.substring(2);
  var flagValue = true;
  if (rawFlag.startsWith('no-')) {
    rawFlag = rawFlag.substring(3);
    flagValue = false;
  }
  return (rawFlag, flagValue);
}

String _toFlag(String name, bool value) {
  return "--${!value ? 'no-' : ''}$name";
}
