// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// This script removes published archives from the cloud storage and the
/// corresponding JSON metadata file that the website uses to determine what
/// releases are available.
///
/// If asked to remove a release that is currently the release on that channel,
/// it will replace that release with the next most recent release on that
/// channel.
library;

import 'dart:async';
import 'dart:convert';
import 'dart:io' hide Platform;

import 'package:args/args.dart';
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart' show LocalPlatform, Platform;
import 'package:process/process.dart';

const String gsBase = 'gs://flutter_infra_release';
const String releaseFolder = '/releases';
const String gsReleaseFolder = '$gsBase$releaseFolder';
const String baseUrl = 'https://storage.googleapis.com/flutter_infra_release';

/// Exception class for when a process fails to run, so we can catch
/// it and provide something more readable than a stack trace.
class UnpublishException implements Exception {
  UnpublishException(this.message, [this.result]);

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

enum Channel { dev, beta, stable }

String getChannelName(Channel channel) {
  return switch (channel) {
    Channel.beta => 'beta',
    Channel.dev => 'dev',
    Channel.stable => 'stable',
  };
}

Channel fromChannelName(String? name) {
  return switch (name) {
    'beta' => Channel.beta,
    'dev' => Channel.dev,
    'stable' => Channel.stable,
    _ => throw ArgumentError('Invalid channel name.'),
  };
}

enum PublishedPlatform { linux, macos, windows }

String getPublishedPlatform(PublishedPlatform platform) {
  return switch (platform) {
    PublishedPlatform.linux => 'linux',
    PublishedPlatform.macos => 'macos',
    PublishedPlatform.windows => 'windows',
  };
}

PublishedPlatform fromPublishedPlatform(String name) {
  return switch (name) {
    'linux' => PublishedPlatform.linux,
    'macos' => PublishedPlatform.macos,
    'windows' => PublishedPlatform.windows,
    _ => throw ArgumentError('Invalid published platform name.'),
  };
}

/// A helper class for classes that want to run a process, optionally have the
/// stderr and stdout reported as the process runs, and capture the stdout
/// properly without dropping any.
class ProcessRunner {
  /// Creates a [ProcessRunner].
  ///
  /// The [processManager], [subprocessOutput], and [platform] arguments must
  /// not be null.
  ProcessRunner({
    this.processManager = const LocalProcessManager(),
    this.subprocessOutput = true,
    this.defaultWorkingDirectory,
    this.platform = const LocalPlatform(),
  }) {
    environment = Map<String, String>.from(platform.environment);
  }

  /// The platform to use for a starting environment.
  final Platform platform;

  /// Set [subprocessOutput] to show output as processes run. Stdout from the
  /// process will be printed to stdout, and stderr printed to stderr.
  final bool subprocessOutput;

  /// Set the [processManager] in order to inject a test instance to perform
  /// testing.
  final ProcessManager processManager;

  /// Sets the default directory used when `workingDirectory` is not specified
  /// to [runProcess].
  final Directory? defaultWorkingDirectory;

  /// The environment to run processes with.
  late Map<String, String> environment;

  /// Run the command and arguments in `commandLine` as a sub-process from
  /// `workingDirectory` if set, or the [defaultWorkingDirectory] if not. Uses
  /// [Directory.current] if [defaultWorkingDirectory] is not set.
  ///
  /// Set `failOk` if [runProcess] should not throw an exception when the
  /// command completes with a non-zero exit code.
  Future<String> runProcess(
    List<String> commandLine, {
    Directory? workingDirectory,
    bool failOk = false,
  }) async {
    workingDirectory ??= defaultWorkingDirectory ?? Directory.current;
    if (subprocessOutput) {
      stderr.write('Running "${commandLine.join(' ')}" in ${workingDirectory.path}.\n');
    }
    final List<int> output = <int>[];
    final Completer<void> stdoutComplete = Completer<void>();
    final Completer<void> stderrComplete = Completer<void>();
    late Process process;
    Future<int> allComplete() async {
      await stderrComplete.future;
      await stdoutComplete.future;
      return process.exitCode;
    }

    try {
      process = await processManager.start(
        commandLine,
        workingDirectory: workingDirectory.absolute.path,
        environment: environment,
      );
      process.stdout.listen((List<int> event) {
        output.addAll(event);
        if (subprocessOutput) {
          stdout.add(event);
        }
      }, onDone: () async => stdoutComplete.complete());
      if (subprocessOutput) {
        process.stderr.listen((List<int> event) {
          stderr.add(event);
        }, onDone: () async => stderrComplete.complete());
      } else {
        stderrComplete.complete();
      }
    } on ProcessException catch (e) {
      final String message =
          'Running "${commandLine.join(' ')}" in ${workingDirectory.path} '
          'failed with:\n$e';
      throw UnpublishException(message);
    } on ArgumentError catch (e) {
      final String message =
          'Running "${commandLine.join(' ')}" in ${workingDirectory.path} '
          'failed with:\n$e';
      throw UnpublishException(message);
    }

    final int exitCode = await allComplete();
    if (exitCode != 0 && !failOk) {
      final String message =
          'Running "${commandLine.join(' ')}" in ${workingDirectory.path} failed';
      throw UnpublishException(message, ProcessResult(0, exitCode, null, 'returned $exitCode'));
    }
    return utf8.decoder.convert(output).trim();
  }
}

class ArchiveUnpublisher {
  ArchiveUnpublisher(
    this.tempDir,
    this.revisionsBeingRemoved,
    this.channels,
    this.platform, {
    this.confirmed = false,
    ProcessManager? processManager,
    bool subprocessOutput = true,
  }) : assert(revisionsBeingRemoved.length == 40),
       metadataGsPath = '$gsReleaseFolder/${getMetadataFilename(platform)}',
       _processRunner = ProcessRunner(
         processManager: processManager ?? const LocalProcessManager(),
         subprocessOutput: subprocessOutput,
       );

  final PublishedPlatform platform;
  final String metadataGsPath;
  final Set<Channel> channels;
  final Set<String> revisionsBeingRemoved;
  final bool confirmed;
  final Directory tempDir;
  final ProcessRunner _processRunner;
  static String getMetadataFilename(PublishedPlatform platform) =>
      'releases_${getPublishedPlatform(platform)}.json';

  /// Remove the archive from Google Storage.
  Future<void> unpublishArchive() async {
    final Map<String, dynamic> jsonData = await _loadMetadata();
    final List<Map<String, String>> releases =
        (jsonData['releases'] as List<dynamic>).map<Map<String, String>>((dynamic entry) {
          final Map<String, dynamic> mapEntry = entry as Map<String, dynamic>;
          return mapEntry.cast<String, String>();
        }).toList();
    final Map<Channel, Map<String, String>> paths = await _getArchivePaths(releases);
    releases.removeWhere(
      (Map<String, String> value) =>
          revisionsBeingRemoved.contains(value['hash']) &&
          channels.contains(fromChannelName(value['channel'])),
    );
    releases.sort((Map<String, String> a, Map<String, String> b) {
      final DateTime aDate = DateTime.parse(a['release_date']!);
      final DateTime bDate = DateTime.parse(b['release_date']!);
      return bDate.compareTo(aDate);
    });
    jsonData['releases'] = releases;
    for (final Channel channel in channels) {
      if (!revisionsBeingRemoved.contains(
        (jsonData['current_release'] as Map<String, dynamic>)[getChannelName(channel)],
      )) {
        // Don't replace the current release if it's not one of the revisions we're removing.
        continue;
      }
      final Map<String, String> replacementRelease = releases.firstWhere(
        (Map<String, String> value) => value['channel'] == getChannelName(channel),
      );
      (jsonData['current_release'] as Map<String, dynamic>)[getChannelName(channel)] =
          replacementRelease['hash'];
      print(
        '${confirmed ? 'Reverting' : 'Would revert'} current ${getChannelName(channel)} '
        '${getPublishedPlatform(platform)} release to ${replacementRelease['hash']} (version ${replacementRelease['version']}).',
      );
    }
    await _cloudRemoveArchive(paths);
    await _updateMetadata(jsonData);
  }

  Future<Map<Channel, Map<String, String>>> _getArchivePaths(
    List<Map<String, String>> releases,
  ) async {
    final Set<String> hashes = <String>{};
    final Map<Channel, Map<String, String>> paths = <Channel, Map<String, String>>{};
    for (final Map<String, String> revision in releases) {
      final String hash = revision['hash']!;
      final Channel channel = fromChannelName(revision['channel']);
      hashes.add(hash);
      if (revisionsBeingRemoved.contains(hash) && channels.contains(channel)) {
        paths[channel] ??= <String, String>{};
        paths[channel]![hash] = revision['archive']!;
      }
    }
    final Set<String> missingRevisions = revisionsBeingRemoved.difference(
      hashes.intersection(revisionsBeingRemoved),
    );
    if (missingRevisions.isNotEmpty) {
      final bool plural = missingRevisions.length > 1;
      throw UnpublishException(
        'Revision${plural ? 's' : ''} $missingRevisions ${plural ? 'are' : 'is'} not present in the server metadata.',
      );
    }
    return paths;
  }

  Future<Map<String, dynamic>> _loadMetadata() async {
    final File metadataFile = File(path.join(tempDir.absolute.path, getMetadataFilename(platform)));
    // Always run this, even in dry runs.
    await _runGsUtil(<String>['cp', metadataGsPath, metadataFile.absolute.path], confirm: true);
    final String currentMetadata = metadataFile.readAsStringSync();
    if (currentMetadata.isEmpty) {
      throw UnpublishException('Empty metadata received from server');
    }

    Map<String, dynamic> jsonData;
    try {
      jsonData = json.decode(currentMetadata) as Map<String, dynamic>;
    } on FormatException catch (e) {
      throw UnpublishException('Unable to parse JSON metadata received from cloud: $e');
    }

    return jsonData;
  }

  Future<void> _updateMetadata(Map<String, dynamic> jsonData) async {
    // We can't just cat the metadata from the server with 'gsutil cat', because
    // Windows wants to echo the commands that execute in gsutil.bat to the
    // stdout when we do that. So, we copy the file locally and then read it
    // back in.
    final File metadataFile = File(path.join(tempDir.absolute.path, getMetadataFilename(platform)));
    const JsonEncoder encoder = JsonEncoder.withIndent('  ');
    metadataFile.writeAsStringSync(encoder.convert(jsonData));
    print(
      '${confirmed ? 'Overwriting' : 'Would overwrite'} $metadataGsPath with contents of ${metadataFile.absolute.path}',
    );
    await _cloudReplaceDest(metadataFile.absolute.path, metadataGsPath);
  }

  Future<String> _runGsUtil(
    List<String> args, {
    Directory? workingDirectory,
    bool failOk = false,
    bool confirm = false,
  }) async {
    final List<String> command = <String>['gsutil', '--', ...args];
    if (confirm) {
      return _processRunner.runProcess(command, workingDirectory: workingDirectory, failOk: failOk);
    } else {
      print('Would run: ${command.join(' ')}');
      return '';
    }
  }

  Future<void> _cloudRemoveArchive(Map<Channel, Map<String, String>> paths) async {
    final List<String> files = <String>[];
    print('${confirmed ? 'Removing' : 'Would remove'} the following release archives:');
    for (final Channel channel in paths.keys) {
      final Map<String, String> hashes = paths[channel]!;
      for (final String hash in hashes.keys) {
        final String file = '$gsReleaseFolder/${hashes[hash]}';
        files.add(file);
        print('  $file');
      }
    }
    await _runGsUtil(<String>['rm', ...files], failOk: true, confirm: confirmed);
  }

  Future<String> _cloudReplaceDest(String src, String dest) async {
    assert(dest.startsWith('gs:'), '_cloudReplaceDest must have a destination in cloud storage.');
    assert(!src.startsWith('gs:'), '_cloudReplaceDest must have a local source file.');
    // We often don't have permission to overwrite, but
    // we have permission to remove, so that's what we do first.
    await _runGsUtil(<String>['rm', dest], failOk: true, confirm: confirmed);
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
    final List<String> args = <String>[
      // Use our preferred MIME type for the files we care about
      // and let gsutil figure it out for anything else.
      if (mimeType != null) ...<String>['-h', 'Content-Type:$mimeType'],
      ...<String>['cp', src, dest],
    ];
    return _runGsUtil(args, confirm: confirmed);
  }
}

void _printBanner(String message) {
  final String banner = '*** $message ***';
  print('\n');
  print('*' * banner.length);
  print(banner);
  print('*' * banner.length);
  print('\n');
}

/// Prepares a flutter git repo to be removed from the published cloud storage.
Future<void> main(List<String> rawArguments) async {
  final List<String> allowedChannelValues =
      Channel.values.map<String>((Channel channel) => getChannelName(channel)).toList();
  final List<String> allowedPlatformNames =
      PublishedPlatform.values
          .map<String>((PublishedPlatform platform) => getPublishedPlatform(platform))
          .toList();
  final ArgParser argParser = ArgParser();
  argParser.addOption(
    'temp_dir',
    help:
        'A location where temporary files may be written. Defaults to a '
        'directory in the system temp folder. If a temp_dir is not '
        'specified, then by default a generated temporary directory will be '
        'created, used, and removed automatically when the script exits.',
  );
  argParser.addMultiOption(
    'revision',
    help:
        'The Flutter git repo revisions to remove from the published site. '
        'Must be full 40-character hashes. More than one may be specified, '
        'either by giving the option more than once, or by giving a comma '
        'separated list. Required.',
  );
  argParser.addMultiOption(
    'channel',
    allowed: allowedChannelValues,
    help:
        'The Flutter channels to remove the archives corresponding to the '
        'revisions given with --revision. More than one may be specified, '
        'either by giving the option more than once, or by giving a '
        'comma separated list. If not specified, then the archives from all '
        'channels that a revision appears in will be removed.',
  );
  argParser.addMultiOption(
    'platform',
    allowed: allowedPlatformNames,
    help:
        'The Flutter platforms to remove the archive from. May specify more '
        'than one, either by giving the option more than once, or by giving a '
        'comma separated list. If not specified, then the archives from all '
        'platforms that a revision appears in will be removed.',
  );
  argParser.addFlag(
    'confirm',
    help:
        'If set, will actually remove the archive from Google Cloud Storage '
        'upon successful execution of this script. Published archives will be '
        'removed from this directory: $baseUrl$releaseFolder. This option '
        'must be set to perform any action on the server, otherwise only a dry '
        'run is performed.',
  );
  argParser.addFlag('help', negatable: false, help: 'Print help for this command.');

  final ArgResults parsedArguments = argParser.parse(rawArguments);

  if (parsedArguments['help'] as bool) {
    print(argParser.usage);
    exit(0);
  }

  void errorExit(String message, {int exitCode = -1}) {
    stderr.write('Error: $message\n\n');
    stderr.write('${argParser.usage}\n');
    exit(exitCode);
  }

  final List<String> revisions = parsedArguments['revision'] as List<String>;
  if (revisions.isEmpty) {
    errorExit('Invalid argument: at least one --revision must be specified.');
  }
  for (final String revision in revisions) {
    if (revision.length != 40) {
      errorExit(
        'Invalid argument: --revision "$revision" must be the entire hash, not just a prefix.',
      );
    }
    if (revision.contains(RegExp(r'[^a-fA-F0-9]'))) {
      errorExit('Invalid argument: --revision "$revision" contains non-hex characters.');
    }
  }

  final String tempDirArg = parsedArguments['temp_dir'] as String;
  Directory tempDir;
  bool removeTempDir = false;
  if (tempDirArg.isEmpty) {
    tempDir = Directory.systemTemp.createTempSync('flutter_package.');
    removeTempDir = true;
  } else {
    tempDir = Directory(tempDirArg);
    if (!tempDir.existsSync()) {
      errorExit("Temporary directory $tempDirArg doesn't exist.");
    }
  }

  if (!(parsedArguments['confirm'] as bool)) {
    _printBanner(
      'This will be just a dry run. To actually perform the changes below, re-run with --confirm argument.',
    );
  }

  final List<String> channelArg = parsedArguments['channel'] as List<String>;
  final List<String> channelOptions = channelArg.isNotEmpty ? channelArg : allowedChannelValues;
  final Set<Channel> channels =
      channelOptions.map<Channel>((String value) => fromChannelName(value)).toSet();
  final List<String> platformArg = parsedArguments['platform'] as List<String>;
  final List<String> platformOptions = platformArg.isNotEmpty ? platformArg : allowedPlatformNames;
  final List<PublishedPlatform> platforms =
      platformOptions
          .map<PublishedPlatform>((String value) => fromPublishedPlatform(value))
          .toList();
  int exitCode = 0;
  late String message;
  late String stack;
  try {
    for (final PublishedPlatform platform in platforms) {
      final ArchiveUnpublisher publisher = ArchiveUnpublisher(
        tempDir,
        revisions.toSet(),
        channels,
        platform,
        confirmed: parsedArguments['confirm'] as bool,
      );
      await publisher.unpublishArchive();
    }
  } on UnpublishException catch (e, s) {
    exitCode = e.exitCode;
    message = e.message;
    stack = s.toString();
  } catch (e, s) {
    exitCode = -1;
    message = e.toString();
    stack = s.toString();
  } finally {
    if (removeTempDir) {
      tempDir.deleteSync(recursive: true);
    }
    if (exitCode != 0) {
      errorExit('$message\n$stack', exitCode: exitCode);
    }
    if (!(parsedArguments['confirm'] as bool)) {
      _printBanner(
        'This was just a dry run. To actually perform the above changes, re-run with --confirm argument.',
      );
    }
    exit(0);
  }
}
