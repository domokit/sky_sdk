// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:convert/convert.dart';
import 'package:crypto/crypto.dart';
import 'package:file/file.dart';
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart' show LocalPlatform, Platform;
import 'package:process/process.dart';

import 'common.dart';
import 'process_runner.dart';

class ArchivePublisher {
  ArchivePublisher(
    this.tempDir,
    this.revision,
    this.branch,
    this.version,
    this.outputFile,
    this.dryRun, {
    ProcessManager? processManager,
    bool subprocessOutput = true,
    required this.fs,
    this.platform = const LocalPlatform(),
  }) : assert(revision.length == 40),
       platformName = platform.operatingSystem.toLowerCase(),
       metadataGsPath = '$gsReleaseFolder/${getMetadataFilename(platform)}',
       _processRunner = ProcessRunner(
         processManager: processManager,
         subprocessOutput: subprocessOutput,
       );

  final Platform platform;
  final FileSystem fs;
  final String platformName;
  final String metadataGsPath;
  final Branch branch;
  final String revision;
  final Map<String, String> version;
  final Directory tempDir;
  final File outputFile;
  final ProcessRunner _processRunner;
  final bool dryRun;
  String get destinationArchivePath =>
      '${branch.name}/$platformName/${path.basename(outputFile.path)}';
  static String getMetadataFilename(Platform platform) =>
      'releases_${platform.operatingSystem.toLowerCase()}.json';

  Future<String> _getChecksum(File archiveFile) async {
    final AccumulatorSink<Digest> digestSink = AccumulatorSink<Digest>();
    final ByteConversionSink sink = sha256.startChunkedConversion(digestSink);

    final Stream<List<int>> stream = archiveFile.openRead();
    await stream.forEach((List<int> chunk) {
      sink.add(chunk);
    });
    sink.close();
    return digestSink.events.single.toString();
  }

  /// Publish the archive to Google Storage.
  ///
  /// This method will throw if the target archive already exists on cloud
  /// storage.
  Future<void> publishArchive([bool forceUpload = false]) async {
    final String destGsPath = '$gsReleaseFolder/$destinationArchivePath';
    if (!forceUpload) {
      if (await _cloudPathExists(destGsPath) && !dryRun) {
        throw PreparePackageException('File $destGsPath already exists on cloud storage!');
      }
    }
    await _cloudCopy(src: outputFile.absolute.path, dest: destGsPath);
    assert(tempDir.existsSync());
    final String gcsPath = '$gsReleaseFolder/${getMetadataFilename(platform)}';
    await _publishMetadata(gcsPath);
  }

  /// Downloads and updates the metadata file without publishing it.
  Future<void> generateLocalMetadata() async {
    await _updateMetadata('$gsReleaseFolder/${getMetadataFilename(platform)}');
  }

  Future<Map<String, dynamic>> _addRelease(Map<String, dynamic> jsonData) async {
    jsonData['base_url'] = '$baseUrl$releaseFolder';
    if (!jsonData.containsKey('current_release')) {
      jsonData['current_release'] = <String, String>{};
    }
    (jsonData['current_release'] as Map<String, dynamic>)[branch.name] = revision;
    if (!jsonData.containsKey('releases')) {
      jsonData['releases'] = <Map<String, dynamic>>[];
    }

    final Map<String, dynamic> newEntry = <String, dynamic>{};
    newEntry['hash'] = revision;
    newEntry['channel'] = branch.name;
    newEntry['version'] = version[frameworkVersionTag];
    newEntry['dart_sdk_version'] = version[dartVersionTag];
    newEntry['dart_sdk_arch'] = version[dartTargetArchTag];
    newEntry['release_date'] = DateTime.now().toUtc().toIso8601String();
    newEntry['archive'] = destinationArchivePath;
    newEntry['sha256'] = await _getChecksum(outputFile);

    // Search for any entries with the same hash and channel and remove them.
    final List<dynamic> releases = jsonData['releases'] as List<dynamic>;
    jsonData['releases'] = <Map<String, dynamic>>[
      for (final Map<String, dynamic> entry in releases.cast<Map<String, dynamic>>())
        if (entry['hash'] != newEntry['hash'] ||
            entry['channel'] != newEntry['channel'] ||
            entry['dart_sdk_arch'] != newEntry['dart_sdk_arch'])
          entry,
      newEntry,
    ]..sort((Map<String, dynamic> a, Map<String, dynamic> b) {
      final DateTime aDate = DateTime.parse(a['release_date'] as String);
      final DateTime bDate = DateTime.parse(b['release_date'] as String);
      return bDate.compareTo(aDate);
    });
    return jsonData;
  }

  Future<void> _updateMetadata(String gsPath) async {
    // We can't just cat the metadata from the server with 'gsutil cat', because
    // Windows wants to echo the commands that execute in gsutil.bat to the
    // stdout when we do that. So, we copy the file locally and then read it
    // back in.
    final File metadataFile = fs.file(
      path.join(tempDir.absolute.path, getMetadataFilename(platform)),
    );
    await _runGsUtil(<String>['cp', gsPath, metadataFile.absolute.path]);
    Map<String, dynamic> jsonData = <String, dynamic>{};
    if (!dryRun) {
      final String currentMetadata = metadataFile.readAsStringSync();
      if (currentMetadata.isEmpty) {
        throw PreparePackageException('Empty metadata received from server');
      }
      try {
        jsonData = json.decode(currentMetadata) as Map<String, dynamic>;
      } on FormatException catch (e) {
        throw PreparePackageException('Unable to parse JSON metadata received from cloud: $e');
      }
    }
    // Run _addRelease, even on a dry run, so we can inspect the metadata on a
    // dry run. On a dry run, the only thing in the metadata file be the new
    // release.
    jsonData = await _addRelease(jsonData);

    const JsonEncoder encoder = JsonEncoder.withIndent('  ');
    metadataFile.writeAsStringSync(encoder.convert(jsonData));
  }

  /// Publishes the metadata file to GCS.
  Future<void> _publishMetadata(String gsPath) async {
    final File metadataFile = fs.file(
      path.join(tempDir.absolute.path, getMetadataFilename(platform)),
    );
    await _cloudCopy(
      src: metadataFile.absolute.path,
      dest: gsPath,
      // This metadata file is used by the website, so we don't want a long
      // latency between publishing a release and it being available on the
      // site.
      cacheSeconds: shortCacheSeconds,
    );
  }

  Future<String> _runGsUtil(
    List<String> args, {
    Directory? workingDirectory,
    bool failOk = false,
  }) async {
    if (dryRun) {
      print('gsutil.py -- $args');
      return '';
    }
    return _processRunner.runProcess(
      <String>[
        'python3',
        path.join(platform.environment['DEPOT_TOOLS']!, 'gsutil.py'),
        '--',
        ...args,
      ],
      workingDirectory: workingDirectory,
      failOk: failOk,
    );
  }

  /// Determine if a file exists at a given [cloudPath].
  Future<bool> _cloudPathExists(String cloudPath) async {
    try {
      await _runGsUtil(<String>['stat', cloudPath]);
    } on PreparePackageException {
      // `gsutil stat gs://path/to/file` will exit with 1 if file does not exist
      return false;
    }
    return true;
  }

  Future<String> _cloudCopy({required String src, required String dest, int? cacheSeconds}) async {
    // We often don't have permission to overwrite, but
    // we have permission to remove, so that's what we do.
    await _runGsUtil(<String>['rm', dest], failOk: true);
    String? mimeType;
    if (dest.endsWith('.tar.xz')) {
      mimeType = 'application/x-gtar';
    }
    if (dest.endsWith('.zip')) {
      mimeType = 'application/zip';
    }
    if (dest.endsWith('.json')) {
      mimeType = 'application/json';
    }
    return _runGsUtil(<String>[
      // Use our preferred MIME type for the files we care about
      // and let gsutil figure it out for anything else.
      if (mimeType != null) ...<String>['-h', 'Content-Type:$mimeType'],
      if (cacheSeconds != null) ...<String>['-h', 'Cache-Control:max-age=$cacheSeconds'],
      'cp',
      src,
      dest,
    ]);
  }
}
