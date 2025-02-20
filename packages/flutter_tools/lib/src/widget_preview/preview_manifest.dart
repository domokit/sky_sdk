// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:crypto/crypto.dart';
import 'package:meta/meta.dart';

import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/version.dart';
import '../cache.dart';
import '../convert.dart';
import '../project.dart';

typedef PreviewManifestContents = Map<String, Object?>;

class PreviewManifest {
  PreviewManifest({
    required this.logger,
    required this.rootProject,
    required this.fs,
    required this.cache,
  });

  static const String previewManifestPath = 'preview_manifest.json';
  static final Version previewManifestVersion = Version(0, 0, 1);
  static const String kManifestVersion = 'version';
  static const String kSdkVersion = 'sdk-version';
  static const String kPubspecHash = 'pubspec-hash';

  final Logger logger;
  final FlutterProject rootProject;
  final FileSystem fs;
  final Cache cache;

  Directory get widgetPreviewScaffold => rootProject.widgetPreviewScaffold;
  String get _manifestPath => fs.path.join(widgetPreviewScaffold.path, previewManifestPath);
  File get _manifest => fs.file(_manifestPath);

  PreviewManifestContents? _tryLoadManifest() {
    final File manifest = fs.file(_manifestPath);
    if (!manifest.existsSync()) {
      return null;
    }
    return json.decode(manifest.readAsStringSync()) as PreviewManifestContents;
  }

  void generate() {
    logger.printStatus('Creating the Widget Preview Scaffold manifest at ${_manifest.path}');
    assert(!_manifest.existsSync());
    _manifest.createSync(recursive: true);
    final PreviewManifestContents manifestContents = <String, Object?>{
      kManifestVersion: previewManifestVersion.toString(),
      kSdkVersion: cache.dartSdkVersion,
      kPubspecHash: _calculatePubspecHash(),
    };
    _updateManifest(manifestContents);
  }

  void _updateManifest(PreviewManifestContents contents) {
    _manifest.writeAsStringSync(json.encode(contents));
  }

  String _calculatePubspecHash() {
    return md5.convert(rootProject.manifest.toYaml().toString().codeUnits).toString();
  }

  bool shouldGenerateProject() {
    if (!widgetPreviewScaffold.existsSync()) {
      return true;
    }
    final PreviewManifestContents? manifest = _tryLoadManifest();
    // If the manifest doesn't exist or the SDK version isn't present, the widget preview scaffold
    // should be regenerated and rebuilt.
    if (manifest == null ||
        !manifest.containsKey(kManifestVersion) ||
        !manifest.containsKey(kSdkVersion)) {
      logger.printWarning(
        'Invalid Widget Preview Scaffold manifest at ${_manifest.path}. Regenerating Widget '
        'Preview Scaffold.',
      );
      return true;
    }
    final Version? manifestVersion = Version.parse(manifest[kManifestVersion]! as String);
    // If the manifest version in the scaffold doesn't match the current preview manifest spec,
    // we should regenerate it.
    // TODO(bkonyi): is this actually what we want to do, or do we want to just update the manifest?
    if (manifestVersion == null || manifestVersion != previewManifestVersion) {
      logger.printStatus(
        'The existing Widget Preview Scaffold manifest version ($manifestVersion) '
        'is older than the currently supported version ($previewManifestVersion). Regenerating '
        'Widget Preview Scaffold.',
      );
      return true;
    }
    // If the SDK version of the widget preview scaffold doesn't match the current SDK version
    // the widget preview scaffold should also be regenerated to pick up any new functionality and
    // avoid possible binary compatibility issues.
    final bool sdkVersionMismatch = manifest[kSdkVersion] != cache.dartSdkVersion;
    if (sdkVersionMismatch) {
      logger.printStatus(
        'The existing Widget Preview Scaffold was generated with Dart SDK '
        'version ${manifest[kSdkVersion]}, which does not match the current Dart SDK version '
        '(${cache.dartSdkVersion}). Regenerating Widget Preview Scaffold.',
      );
    }
    return sdkVersionMismatch;
  }

  bool shouldRegeneratePubspec() {
    final PreviewManifestContents manifest = _tryLoadManifest()!;
    if (!manifest.containsKey(kPubspecHash)) {
      logger.printWarning(
        'The Widget Preview Scaffold manifest does not include the last known state of the root '
        "project's pubspec.yaml.",
      );
      return true;
    }
    return manifest[kPubspecHash] != _calculatePubspecHash();
  }

  void updatePubspecHash() {
    final PreviewManifestContents manifest = _tryLoadManifest()!;
    manifest[kPubspecHash] = _calculatePubspecHash();
    _updateManifest(manifest);
  }

  @visibleForTesting
  PreviewManifest copyWith({Cache? cache}) {
    return PreviewManifest(
      logger: logger,
      rootProject: rootProject,
      fs: fs,
      cache: cache ?? this.cache,
    );
  }
}
