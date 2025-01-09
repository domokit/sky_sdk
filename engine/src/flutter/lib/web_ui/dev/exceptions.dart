// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

class BrowserInstallerException implements Exception {
  BrowserInstallerException(this.message);

  final String message;

  @override
  String toString() => message;
}

/// Throw this exception in felt command to exit felt with a message and a
/// non-zero exit code.
class ToolExit implements Exception {
  ToolExit(this.message, {this.exitCode = 1});

  final String message;
  final int exitCode;

  @override
  String toString() => message;
}
