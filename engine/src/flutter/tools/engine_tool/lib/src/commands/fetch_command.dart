// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../dependencies.dart';
import 'command.dart';

/// The root 'fetch' command.
final class FetchCommand extends CommandBase {
  /// Constructs the 'fetch' command.
  FetchCommand({required super.environment, super.usageLineLength});

  @override
  String get name => 'fetch';

  @override
  String get description => "Download the Flutter engine's dependencies";

  @override
  List<String> get aliases => const <String>['sync'];

  @override
  Future<int> run() {
    return fetchDependencies(environment);
  }
}
