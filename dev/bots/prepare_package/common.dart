// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' hide Platform;

const String gobMirror = 'https://flutter.googlesource.com/mirrors/flutter';
const String githubRepo = 'https://github.com/flutter/flutter.git';
const String mingitForWindowsUrl =
    'https://storage.googleapis.com/flutter_infra_release/mingit/'
    '603511c649b00bbef0a6122a827ac419b656bc19/mingit.zip';
const String releaseFolder = '/releases';
const String gsBase = 'gs://flutter_infra_release';
const String gsReleaseFolder = '$gsBase$releaseFolder';
const String baseUrl = 'https://storage.googleapis.com/flutter_infra_release';
const int shortCacheSeconds = 60;
const String frameworkVersionTag = 'frameworkVersionFromGit';
const String dartVersionTag = 'dartSdkVersion';
const String dartTargetArchTag = 'dartTargetArch';

enum Branch { beta, stable, master, main }

/// Exception class for when a process fails to run, so we can catch
/// it and provide something more readable than a stack trace.
class PreparePackageException implements Exception {
  PreparePackageException(this.message, [this.result]);

  final String message;
  final ProcessResult? result;
  int get exitCode => result?.exitCode ?? -1;

  @override
  String toString() {
    String output = runtimeType.toString();
    output += ': $message';
    final String stderr = result?.stderr as String? ?? '';
    if (stderr.isNotEmpty) {
      output += ':\n$stderr';
    }
    return output;
  }
}
