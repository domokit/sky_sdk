// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart' show File;

import 'globals.dart';
import 'proto/conductor_state.pb.dart' as pb;
import 'repository.dart';
import 'state.dart';
import 'stdio.dart' show Stdio;

/// Interface for shared functionality across all sub-commands.
///
/// Different frontends (e.g. CLI vs desktop) can share [Context]s, although
/// methods for capturing user interaction may be overridden.
abstract class Context {
  const Context({required this.checkouts, required this.stateFile});

  final Checkouts checkouts;
  final File stateFile;
  Stdio get stdio => checkouts.stdio;

  /// Confirm an action with the user before proceeding.
  ///
  /// The default implementation reads from STDIN. This can be overridden in UI
  /// implementations that capture user interaction differently.
  Future<bool> prompt(String message) async {
    stdio.write('${message.trim()} (y/n) ');
    final String response = stdio.readLineSync().trim();
    final String firstChar = response[0].toUpperCase();
    if (firstChar == 'Y') {
      return true;
    }
    if (firstChar == 'N') {
      return false;
    }
    throw ConductorException('Unknown user input (expected "y" or "n"): $response');
  }

  /// Save the release's [state].
  ///
  /// This can be overridden by frontends that may not persist the state to
  /// disk, and/or may need to call additional update hooks each time the state
  /// is updated.
  void updateState(pb.ConductorState state, [List<String> logs = const <String>[]]) {
    writeStateToFile(stateFile, state, logs);
  }
}
