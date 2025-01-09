// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'base/context.dart';
import 'base/file_system.dart';
import 'build_info.dart';

abstract class ApplicationPackageFactory {
  static ApplicationPackageFactory? get instance => context.get<ApplicationPackageFactory>();

  /// Create an [ApplicationPackage] for the given platform.
  Future<ApplicationPackage?> getPackageForPlatform(
    TargetPlatform platform, {
    BuildInfo? buildInfo,
    File? applicationBinary,
  });
}

abstract class ApplicationPackage {
  ApplicationPackage({required this.id});

  /// Package ID from the Android Manifest or equivalent.
  final String id;

  String? get name;

  String? get displayName => name;

  @override
  String toString() => displayName ?? id;
}

/// An interface for application package that is created from prebuilt binary.
abstract class PrebuiltApplicationPackage implements ApplicationPackage {
  /// The application bundle of the prebuilt application.
  ///
  /// The same ApplicationPackage should be able to be recreated by passing
  /// the file to [FlutterApplicationPackageFactory.getPackageForPlatform].
  FileSystemEntity get applicationPackage;
}
