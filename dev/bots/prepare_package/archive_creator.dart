// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' show stderr;
import 'dart:typed_data';

import 'package:convert/convert.dart';
import 'package:crypto/crypto.dart';
import 'package:file/file.dart';
import 'package:http/http.dart' as http;
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart' show LocalPlatform, Platform;
import 'package:pool/pool.dart';
import 'package:process/process.dart';

import 'common.dart';
import 'process_runner.dart';

typedef HttpReader = Future<Uint8List> Function(Uri url, {Map<String, String> headers});

/// Creates a pre-populated Flutter archive from a git repo.
class ArchiveCreator {
  /// [tempDir] is the directory to use for creating the archive. The script
  /// will place several GiB of data there, so it should have available space.
  ///
  /// The processManager argument is used to inject a mock of [ProcessManager] for
  /// testing purposes.
  ///
  /// If subprocessOutput is true, then output from processes invoked during
  /// archive creation is echoed to stderr and stdout.
  factory ArchiveCreator(
    Directory tempDir,
    Directory outputDir,
    String revision,
    Branch branch, {
    required FileSystem fs,
    HttpReader? httpReader,
    Platform platform = const LocalPlatform(),
    ProcessManager? processManager,
    bool strict = true,
    bool subprocessOutput = true,
  }) {
    final Directory flutterRoot = fs.directory(path.join(tempDir.path, 'flutter'));
    final ProcessRunner processRunner = ProcessRunner(
      processManager: processManager,
      subprocessOutput: subprocessOutput,
      platform: platform,
    )..environment['PUB_CACHE'] = path.join(tempDir.path, '.pub-cache');
    final String flutterExecutable = path.join(flutterRoot.absolute.path, 'bin', 'flutter');
    final String dartExecutable = path.join(
      flutterRoot.absolute.path,
      'bin',
      'cache',
      'dart-sdk',
      'bin',
      'dart',
    );

    return ArchiveCreator._(
      tempDir: tempDir,
      platform: platform,
      flutterRoot: flutterRoot,
      fs: fs,
      outputDir: outputDir,
      revision: revision,
      branch: branch,
      strict: strict,
      processRunner: processRunner,
      httpReader: httpReader ?? http.readBytes,
      flutterExecutable: flutterExecutable,
      dartExecutable: dartExecutable,
    );
  }

  ArchiveCreator._({
    required this.branch,
    required String dartExecutable,
    required this.fs,
    required String flutterExecutable,
    required this.flutterRoot,
    required this.httpReader,
    required this.outputDir,
    required this.platform,
    required ProcessRunner processRunner,
    required this.revision,
    required this.strict,
    required this.tempDir,
  }) : assert(revision.length == 40),
       _processRunner = processRunner,
       _flutter = flutterExecutable,
       _dart = dartExecutable;

  /// The platform to use for the environment and determining which
  /// platform we're running on.
  final Platform platform;

  /// The branch to build the archive for. The branch must contain [revision].
  final Branch branch;

  /// The git revision hash to build the archive for. This revision has
  /// to be available in the [branch], although it doesn't have to be
  /// at HEAD, since we clone the branch and then reset to this revision
  /// to create the archive.
  final String revision;

  /// The flutter root directory in the [tempDir].
  final Directory flutterRoot;

  /// The temporary directory used to build the archive in.
  final Directory tempDir;

  /// The directory to write the output file to.
  final Directory outputDir;

  final FileSystem fs;

  /// True if the creator should be strict about checking requirements or not.
  ///
  /// In strict mode, will insist that the [revision] be a tagged revision.
  final bool strict;

  final Uri _minGitUri = Uri.parse(mingitForWindowsUrl);
  final ProcessRunner _processRunner;

  /// Used to tell the [ArchiveCreator] which function to use for reading
  /// bytes from a URL. Used in tests to inject a fake reader. Defaults to
  /// [http.readBytes].
  final HttpReader httpReader;

  final Map<String, String> _version = <String, String>{};
  late String _flutter;
  late String _dart;

  late final Future<String> _dartArch =
      (() async {
        // Parse 'arch' out of a string like '... "os_arch"\n'.
        return (await _runDart(<String>[
          '--version',
        ])).trim().split(' ').last.replaceAll('"', '').split('_')[1];
      })();

  /// Returns a default archive name when given a Git revision.
  /// Used when an output filename is not given.
  Future<String> get _archiveName async {
    final String os = platform.operatingSystem.toLowerCase();
    // Include the intended host architecture in the file name for non-x64.
    final String arch = await _dartArch == 'x64' ? '' : '${await _dartArch}_';
    // We don't use .tar.xz on Mac because although it can unpack them
    // on the command line (with tar), the "Archive Utility" that runs
    // when you double-click on them just does some crazy behavior (it
    // converts it to a compressed cpio archive, and when you double
    // click on that, it converts it back to .tar.xz, without ever
    // unpacking it!) So, we use .zip for Mac, and the files are about
    // 220MB larger than they need to be. :-(
    final String suffix = platform.isLinux ? 'tar.xz' : 'zip';
    final String package = '${os}_$arch${_version[frameworkVersionTag]}';
    return 'flutter_$package-${branch.name}.$suffix';
  }

  /// Checks out the flutter repo and prepares it for other operations.
  ///
  /// Returns the version for this release as obtained from the git tags, and
  /// the dart version as obtained from `flutter --version`.
  Future<Map<String, String>> initializeRepo() async {
    await _checkoutFlutter();
    if (_version.isEmpty) {
      _version.addAll(await _getVersion());
    }
    return _version;
  }

  /// Performs all of the steps needed to create an archive.
  Future<File> createArchive() async {
    assert(_version.isNotEmpty, 'Must run initializeRepo before createArchive');
    final File outputFile = fs.file(path.join(outputDir.absolute.path, await _archiveName));
    await _installMinGitIfNeeded();
    await _populateCaches();
    await _validate();
    await _archiveFiles(outputFile);
    return outputFile;
  }

  /// Validates the integrity of the release package.
  ///
  /// Currently only checks that macOS binaries are codesigned. Will throw a
  /// [PreparePackageException] if the test fails.
  Future<void> _validate() async {
    // Only validate in strict mode, which means `--publish`
    if (!strict || !platform.isMacOS) {
      return;
    }
    // Validate that the dart binary is codesigned
    try {
      // TODO(fujino): Use the conductor https://github.com/flutter/flutter/issues/81701
      await _processRunner.runProcess(<String>[
        'codesign',
        '-vvvv',
        '--check-notarization',
        _dart,
      ], workingDirectory: flutterRoot);
    } on PreparePackageException catch (e) {
      throw PreparePackageException('The binary $_dart was not codesigned!\n${e.message}');
    }
  }

  /// Returns the version map of this release, according the to tags in the
  /// repo and the output of `flutter --version --machine`.
  ///
  /// This looks for the tag attached to [revision] and, if it doesn't find one,
  /// git will give an error.
  ///
  /// If [strict] is true, the exact [revision] must be tagged to return the
  /// version. If [strict] is not true, will look backwards in time starting at
  /// [revision] to find the most recent version tag.
  ///
  /// The version found as a git tag is added to the information given by
  /// `flutter --version --machine` with the `frameworkVersionFromGit` tag, and
  /// returned.
  Future<Map<String, String>> _getVersion() async {
    String gitVersion;
    if (strict) {
      try {
        gitVersion = await _runGit(<String>['describe', '--tags', '--exact-match', revision]);
      } on PreparePackageException catch (exception) {
        throw PreparePackageException(
          'Git error when checking for a version tag attached to revision $revision.\n'
          'Perhaps there is no tag at that revision?:\n'
          '$exception',
        );
      }
    } else {
      gitVersion = await _runGit(<String>['describe', '--tags', '--abbrev=0', revision]);
    }
    // Run flutter command twice, once to make sure the flutter command is built
    // and ready (and thus won't output any junk on stdout the second time), and
    // once to capture theJSON output. The second run should be fast.
    await _runFlutter(<String>['--version', '--machine']);
    final String versionJson = await _runFlutter(<String>['--version', '--machine']);
    final Map<String, String> versionMap = <String, String>{};
    final Map<String, dynamic> result = json.decode(versionJson) as Map<String, dynamic>;
    result.forEach((String key, dynamic value) => versionMap[key] = value.toString());
    versionMap[frameworkVersionTag] = gitVersion;
    versionMap[dartTargetArchTag] = await _dartArch;
    return versionMap;
  }

  /// Clone the Flutter repo and make sure that the git environment is sane
  /// for when the user will unpack it.
  Future<void> _checkoutFlutter() async {
    // We want the user to start out the in the specified branch instead of a
    // detached head. To do that, we need to make sure the branch points at the
    // desired revision.
    await _runGit(<String>['clone', '-b', branch.name, gobMirror], workingDirectory: tempDir);
    await _runGit(<String>['reset', '--hard', revision]);

    // Make the origin point to github instead of the chromium mirror.
    await _runGit(<String>['remote', 'set-url', 'origin', githubRepo]);

    // Minify `.git` footprint (saving about ~100 MB as of Oct 2022)
    await _runGit(<String>['gc', '--prune=now', '--aggressive']);
  }

  /// Retrieve the MinGit executable from storage and unpack it.
  Future<void> _installMinGitIfNeeded() async {
    if (!platform.isWindows) {
      return;
    }
    final Uint8List data = await httpReader(_minGitUri);
    final File gitFile = fs.file(path.join(tempDir.absolute.path, 'mingit.zip'));
    await gitFile.writeAsBytes(data, flush: true);

    final Directory minGitPath = fs.directory(
      path.join(flutterRoot.absolute.path, 'bin', 'mingit'),
    );
    await minGitPath.create(recursive: true);
    await _unzipArchive(gitFile, workingDirectory: minGitPath);
  }

  /// Downloads an archive of every package that is present in the temporary
  /// pub-cache from pub.dev. Stores the archives in
  /// $flutterRoot/.pub-preload-cache.
  ///
  /// These archives will be installed in the user-level cache on first
  /// following flutter command that accesses the cache.
  ///
  /// Precondition: all packages currently in the PUB_CACHE of [_processRunner]
  /// are installed from pub.dev.
  Future<void> _downloadPubPackageArchives() async {
    final Pool pool = Pool(10); // Number of simultaneous downloads.
    final http.Client client = http.Client();
    final Directory preloadCache = fs.directory(path.join(flutterRoot.path, '.pub-preload-cache'));
    preloadCache.createSync(recursive: true);

    /// Fetch a single package.
    Future<void> fetchPackageArchive(String name, String version) async {
      await pool.withResource(() async {
        stderr.write('Fetching package archive for $name-$version.\n');
        int retries = 7;
        while (true) {
          retries -= 1;
          try {
            final Uri packageListingUrl = Uri.parse('https://pub.dev/api/packages/$name');
            // Fetch the package listing to obtain the package download url.
            final http.Response packageListingResponse = await client.get(packageListingUrl);
            if (packageListingResponse.statusCode != 200) {
              throw Exception(
                'Downloading $packageListingUrl failed. Status code ${packageListingResponse.statusCode}.',
              );
            }
            final dynamic decodedPackageListing = json.decode(packageListingResponse.body);
            if (decodedPackageListing is! Map) {
              throw const FormatException('Package listing should be a map');
            }
            final dynamic versions = decodedPackageListing['versions'];
            if (versions is! List) {
              throw const FormatException('.versions should be a list');
            }
            final Map<String, dynamic> versionDescription =
                versions.firstWhere(
                      (dynamic description) {
                        if (description is! Map) {
                          throw const FormatException('.versions elements should be maps');
                        }
                        return description['version'] == version;
                      },
                      orElse:
                          () =>
                              throw FormatException(
                                'Could not find $name-$version in package listing',
                              ),
                    )
                    as Map<String, dynamic>;
            final dynamic downloadUrl = versionDescription['archive_url'];
            if (downloadUrl is! String) {
              throw const FormatException('archive_url should be a string');
            }
            final dynamic archiveSha256 = versionDescription['archive_sha256'];
            if (archiveSha256 is! String) {
              throw const FormatException('archive_sha256 should be a string');
            }
            final http.Request request = http.Request('get', Uri.parse(downloadUrl));
            final http.StreamedResponse response = await client.send(request);
            if (response.statusCode != 200) {
              throw Exception(
                'Downloading ${request.url} failed. Status code ${response.statusCode}.',
              );
            }
            final File archiveFile = fs.file(path.join(preloadCache.path, '$name-$version.tar.gz'));
            await response.stream.pipe(archiveFile.openWrite());
            final Stream<List<int>> archiveStream = archiveFile.openRead();
            final Digest r = await sha256.bind(archiveStream).first;
            if (hex.encode(r.bytes) != archiveSha256) {
              throw Exception('Hash mismatch of downloaded archive');
            }
          } on Exception catch (e) {
            stderr.write('Failed downloading $name-$version. $e\n');
            if (retries > 0) {
              stderr.write('Retrying download of $name-$version...');
              // Retry.
              continue;
            } else {
              rethrow;
            }
          }
          break;
        }
      });
    }

    final Map<String, dynamic> cacheDescription =
        json.decode(await _runFlutter(<String>['pub', 'cache', 'list'])) as Map<String, dynamic>;
    final Map<String, dynamic> packages = cacheDescription['packages'] as Map<String, dynamic>;
    final List<Future<void>> downloads = <Future<void>>[];
    for (final MapEntry<String, dynamic> package in packages.entries) {
      final String name = package.key;
      final Map<String, dynamic> versions = package.value as Map<String, dynamic>;
      for (final String version in versions.keys) {
        downloads.add(fetchPackageArchive(name, version));
      }
    }
    await Future.wait(downloads);
    client.close();
  }

  /// Prepare the archive repo so that it has all of the caches warmed up and
  /// is configured for the user to begin working.
  Future<void> _populateCaches() async {
    await _runFlutter(<String>['doctor']);
    await _runFlutter(<String>['update-packages']);
    await _runFlutter(<String>['precache']);
    await _runFlutter(<String>['ide-config']);

    // Create each of the templates, since they will call 'pub get' on
    // themselves when created, and this will warm the cache with their
    // dependencies too.
    for (final String template in <String>['app', 'package', 'plugin']) {
      final String createName = path.join(tempDir.path, 'create_$template');
      await _runFlutter(
        <String>['create', '--template=$template', createName],
        // Run it outside the cloned Flutter repo to not nest git repos, since
        // they'll be git repos themselves too.
        workingDirectory: tempDir,
      );
    }
    await _downloadPubPackageArchives();
    // Yes, we could just skip all .packages files when constructing
    // the archive, but some are checked in, and we don't want to skip
    // those.
    await _runGit(<String>[
      'clean',
      '-f',
      // Do not -X as it could lead to entire bin/cache getting cleaned
      '-x',
      '--',
      '**/.packages',
    ]);

    /// Remove package_config files and any contents in .dart_tool
    await _runGit(<String>['clean', '-f', '-x', '--', '**/.dart_tool/']);

    // Ensure the above commands do not clean out the cache
    final Directory flutterCache = fs.directory(
      path.join(flutterRoot.absolute.path, 'bin', 'cache'),
    );
    if (!flutterCache.existsSync()) {
      throw Exception('The flutter cache was not found at ${flutterCache.path}!');
    }

    /// Remove git subfolder from .pub-cache, this contains the flutter goldens
    /// and new flutter_gallery.
    final Directory gitCache = fs.directory(
      path.join(flutterRoot.absolute.path, '.pub-cache', 'git'),
    );
    if (gitCache.existsSync()) {
      gitCache.deleteSync(recursive: true);
    }
  }

  /// Write the archive to the given output file.
  Future<void> _archiveFiles(File outputFile) async {
    if (outputFile.path.toLowerCase().endsWith('.zip')) {
      await _createZipArchive(outputFile, flutterRoot);
    } else if (outputFile.path.toLowerCase().endsWith('.tar.xz')) {
      await _createTarArchive(outputFile, flutterRoot);
    }
  }

  Future<String> _runDart(List<String> args, {Directory? workingDirectory}) {
    return _processRunner.runProcess(<String>[
      _dart,
      ...args,
    ], workingDirectory: workingDirectory ?? flutterRoot);
  }

  Future<String> _runFlutter(List<String> args, {Directory? workingDirectory}) {
    return _processRunner.runProcess(<String>[
      _flutter,
      ...args,
    ], workingDirectory: workingDirectory ?? flutterRoot);
  }

  Future<String> _runGit(List<String> args, {Directory? workingDirectory}) {
    return _processRunner.runProcess(<String>[
      'git',
      ...args,
    ], workingDirectory: workingDirectory ?? flutterRoot);
  }

  /// Unpacks the given zip file into the currentDirectory (if set), or the
  /// same directory as the archive.
  Future<String> _unzipArchive(File archive, {Directory? workingDirectory}) {
    workingDirectory ??= fs.directory(path.dirname(archive.absolute.path));
    List<String> commandLine;
    if (platform.isWindows) {
      commandLine = <String>['7za', 'x', archive.absolute.path];
    } else {
      commandLine = <String>['unzip', archive.absolute.path];
    }
    return _processRunner.runProcess(commandLine, workingDirectory: workingDirectory);
  }

  /// Create a zip archive from the directory source.
  Future<String> _createZipArchive(File output, Directory source) async {
    List<String> commandLine;
    if (platform.isWindows) {
      // Unhide the .git folder, https://docs.microsoft.com/en-us/windows-server/administration/windows-commands/attrib.
      await _processRunner.runProcess(<String>[
        'attrib',
        '-h',
        '.git',
      ], workingDirectory: fs.directory(source.absolute.path));
      commandLine = <String>[
        '7za',
        'a',
        '-tzip',
        '-mx=9',
        output.absolute.path,
        path.basename(source.path),
      ];
    } else {
      commandLine = <String>[
        'zip',
        '-r',
        '-9',
        '--symlinks',
        output.absolute.path,
        path.basename(source.path),
      ];
    }
    return _processRunner.runProcess(
      commandLine,
      workingDirectory: fs.directory(path.dirname(source.absolute.path)),
    );
  }

  /// Create a tar archive from the directory source.
  Future<String> _createTarArchive(File output, Directory source) {
    return _processRunner.runProcess(<String>[
      'tar',
      'cJf',
      output.absolute.path,
      // Print out input files as they get added, to debug hangs
      '--verbose',
      path.basename(source.absolute.path),
    ], workingDirectory: fs.directory(path.dirname(source.absolute.path)));
  }
}
