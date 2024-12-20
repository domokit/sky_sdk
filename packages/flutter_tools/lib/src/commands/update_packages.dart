// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:collection';

import 'package:meta/meta.dart';

import '../base/common.dart';
import '../base/context.dart';
import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/net.dart';
import '../base/task_queue.dart';
import '../cache.dart';
import '../dart/pub.dart';
import '../globals.dart' as globals;
import '../project.dart';
import '../runner/flutter_command.dart';
import '../update_packages_pins.dart';
import '../version.dart';

// Pub packages are rolled automatically by the flutter-pub-roller-bot
// by using the `flutter update-packages --force-upgrade`.
// For the latest status, see:
//   https://github.com/pulls?q=author%3Aflutter-pub-roller-bot

class UpdatePackagesCommand extends FlutterCommand {
  UpdatePackagesCommand() {
    argParser
      ..addFlag(
        'force-upgrade',
        help:
            'Attempt to update all the dependencies to their latest versions.\n'
            'This will actually modify the pubspec.yaml files in your checkout.',
        negatable: false,
      )
      ..addOption(
        'cherry-pick-package',
        help:
            'Attempt to update only the specified package. The "--cherry-pick-version" version must be specified also.',
      )
      ..addOption(
        'cherry-pick-version',
        help:
            'Attempt to update the package to the specified version. The "--cherry-pick-package" option must be specified also.',
      )
      ..addFlag(
        'paths',
        help:
            'Finds paths in the dependency chain leading from package specified '
            'in "--from" to package specified in "--to".',
        negatable: false,
      )
      ..addOption(
        'from',
        help:
            'Used with "--dependency-path". Specifies the package to begin '
            'searching dependency path from.',
      )
      ..addOption(
        'to',
        help:
            'Used with "--dependency-path". Specifies the package that the '
            'sought-after dependency path leads to.',
      )
      ..addFlag(
        'transitive-closure',
        help:
            'Prints the dependency graph that is the transitive closure of '
            'packages the Flutter SDK depends on.',
        negatable: false,
      )
      ..addFlag(
        'consumer-only',
        help:
            'Only prints the dependency graph that is the transitive closure '
            'that a consumer of the Flutter SDK will observe (when combined '
            'with "--transitive-closure").',
        negatable: false,
      )
      ..addFlag(
        'verify-only',
        help: 'Verifies the package checksum without changing or updating deps.',
        negatable: false,
      )
      ..addFlag(
        'offline',
        help: 'Use cached packages instead of accessing the network.',
        negatable: false,
      )
      ..addFlag(
        'crash',
        help: 'For Flutter CLI testing only, forces this command to throw an unhandled exception.',
        negatable: false,
      )
      ..addOption(
        'jobs',
        abbr: 'j',
        help:
            'Causes the "pub get" runs to happen concurrently on this many '
            'CPUs. Defaults to the number of CPUs that this machine has.',
      )
      ..addOption(
        'synthetic-package-path',
        help:
            'Write the synthetic monolithic pub package generated to do '
            'version solving to a persistent path. By default, a temporary '
            'directory that is deleted before the command exits. By '
            'providing this path, a Flutter maintainer can inspect further '
            'exactly how version solving was achieved.',
      );
  }

  @override
  final String name = 'update-packages';

  @override
  final String description =
      'Update the packages inside the Flutter repo. '
      'This is intended for CI and repo maintainers. '
      'Normal Flutter developers should not have to '
      'use this command.';

  @override
  final List<String> aliases = <String>['upgrade-packages'];

  @override
  final bool hidden = true;

  // Lazy-initialize the net utilities with values from the context.
  late final Net _net = Net(
    httpClientFactory: context.get<HttpClientFactory>(),
    logger: globals.logger,
    platform: globals.platform,
  );

  Future<void> _downloadCoverageData() async {
    final String urlBase =
        globals.platform.environment[kFlutterStorageBaseUrl] ?? 'https://storage.googleapis.com';
    final Uri coverageUri = Uri.parse('$urlBase/flutter_infra_release/flutter/coverage/lcov.info');
    final List<int>? data = await _net.fetchUrl(coverageUri, maxAttempts: 3);
    if (data == null) {
      throwToolExit('Failed to fetch coverage data from $coverageUri');
    }
    final String coverageDir = globals.fs.path.join(
      Cache.flutterRoot!,
      'packages/flutter/coverage',
    );
    globals.fs.file(globals.fs.path.join(coverageDir, 'lcov.base.info'))
      ..createSync(recursive: true)
      ..writeAsBytesSync(data, flush: true);
    globals.fs.file(globals.fs.path.join(coverageDir, 'lcov.info'))
      ..createSync(recursive: true)
      ..writeAsBytesSync(data, flush: true);
  }

  late final Directory _syntheticPackageDir =
      (() {
        final String? optionPath = stringArg('synthetic-package-path');
        if (optionPath == null) {
          return globals.fs.systemTempDirectory.createTempSync('flutter_update_packages.');
        }
        final Directory syntheticPackageDir = globals.fs.directory(optionPath);
        if (!syntheticPackageDir.existsSync()) {
          syntheticPackageDir.createSync(recursive: true);
        }
        globals.printStatus(
          'The synthetic package with all pub dependencies across the repo will '
          'be written to ${syntheticPackageDir.absolute.path}.',
        );
        return syntheticPackageDir;
      })();

  @override
  Future<FlutterCommandResult> runCommand() async {
    final List<Directory> packages = runner!.getRepoPackages();

    final bool forceUpgrade = boolArg('force-upgrade');
    final bool isPrintPaths = boolArg('paths');
    final bool isPrintTransitiveClosure = boolArg('transitive-closure');
    final bool isVerifyOnly = boolArg('verify-only');
    final bool isConsumerOnly = boolArg('consumer-only');
    final bool offline = boolArg('offline');
    final String? cherryPickPackage = stringArg('cherry-pick-package');
    final String? cherryPickVersion = stringArg('cherry-pick-version');

    if (boolArg('crash')) {
      throw StateError('test crash please ignore.');
    }

    if (forceUpgrade && offline) {
      throwToolExit('--force-upgrade cannot be used with the --offline flag');
    }

    if (forceUpgrade && cherryPickPackage != null) {
      throwToolExit('--force-upgrade cannot be used with the --cherry-pick-package flag');
    }

    if (forceUpgrade && isPrintPaths) {
      throwToolExit('--force-upgrade cannot be used with the --paths flag');
    }

    if (forceUpgrade && isPrintTransitiveClosure) {
      throwToolExit('--force-upgrade cannot be used with the --transitive-closure flag');
    }

    if (cherryPickPackage != null && offline) {
      throwToolExit('--cherry-pick-package cannot be used with the --offline flag');
    }

    if (cherryPickPackage != null && cherryPickVersion == null) {
      throwToolExit('--cherry-pick-version is required when using --cherry-pick-package flag');
    }

    if (isPrintPaths && (stringArg('from') == null || stringArg('to') == null)) {
      throwToolExit('The --from and --to flags are required when using the --paths flag');
    }

    if (!isPrintPaths && (stringArg('from') != null || stringArg('to') != null)) {
      throwToolExit('The --from and --to flags are only allowed when using the --paths flag');
    }

    if (isPrintTransitiveClosure && isPrintPaths) {
      throwToolExit('The --transitive-closure flag cannot be used with the --paths flag');
    }

    // "consumer" packages are those that constitute our public API (e.g. flutter, flutter_test, flutter_driver, flutter_localizations, integration_test).
    if (isConsumerOnly) {
      if (!isPrintTransitiveClosure) {
        throwToolExit('--consumer-only can only be used with the --transitive-closure flag');
      }
      // Only retain flutter, flutter_test, flutter_driver, and flutter_localizations.
      const List<String> consumerPackages = <String>[
        'flutter',
        'flutter_test',
        'flutter_driver',
        'flutter_localizations',
        'integration_test',
      ];
      // ensure we only get flutter/packages
      packages.retainWhere((Directory directory) {
        return consumerPackages.any((String package) {
          return directory.path.endsWith('packages${globals.fs.path.separator}$package');
        });
      });
    }

    if (isVerifyOnly) {
      _verifyPubspecs(packages);
      return FlutterCommandResult.success();
    }

    if (forceUpgrade) {
      // This feature attempts to collect all the packages used across all the
      // pubspec.yamls in the repo (including via transitive dependencies), and
      // find the latest version of each that can be used while keeping each
      // such package fixed at a single version across all the pubspec.yamls.
      globals.printStatus('Upgrading packages...');
    }

    // First, collect the dependencies:
    final List<PubspecYaml> pubspecs = <PubspecYaml>[];
    final Map<String, PubspecDependency> explicitDependencies = <String, PubspecDependency>{};
    final Map<String, PubspecDependency> allDependencies = <String, PubspecDependency>{};
    final Set<String> specialDependencies = <String>{};
    _collectDependencies(
      packages: packages,
      pubspecs: pubspecs,
      explicitDependencies: explicitDependencies,
      allDependencies: allDependencies,
      specialDependencies: specialDependencies,
      printPaths:
          forceUpgrade || isPrintPaths || isPrintTransitiveClosure || cherryPickPackage != null,
    );

    final Iterable<PubspecDependency> baseDependencies;
    if (cherryPickPackage != null) {
      if (!allDependencies.containsKey(cherryPickPackage)) {
        throwToolExit(
          'Package "$cherryPickPackage" is not currently a dependency, and therefore cannot be upgraded.',
        );
      }
      if (cherryPickVersion != null) {
        globals.printStatus(
          'Pinning package "$cherryPickPackage" to version "$cherryPickVersion"...',
        );
      } else {
        globals.printStatus('Upgrading package "$cherryPickPackage"...');
      }
      final List<PubspecDependency> adjustedDependencies = <PubspecDependency>[];
      for (final String package in allDependencies.keys) {
        if (package == cherryPickPackage) {
          assert(cherryPickVersion != null);
          final PubspecDependency pubspec = allDependencies[cherryPickPackage]!;
          adjustedDependencies.add(pubspec.copyWith(version: cherryPickVersion));
        } else {
          adjustedDependencies.add(allDependencies[package]!);
        }
      }
      baseDependencies = adjustedDependencies;
    } else if (forceUpgrade) {
      baseDependencies = explicitDependencies.values;
    } else {
      baseDependencies = allDependencies.values;
    }

    // Now that we have all the dependencies we care about, we are going to
    // create a fake package and then run either "pub upgrade", if requested,
    // followed by "pub get" on it. If upgrading, the pub tool will attempt to
    // bring these dependencies up to the most recent possible versions while
    // honoring all their constraints. If not upgrading the pub tool will only
    // attempt to download any necessary package versions to the pub cache to
    // warm the cache.
    final PubDependencyTree tree = PubDependencyTree(); // object to collect results
    await _pubGetAllDependencies(
      tempDir: _syntheticPackageDir,
      dependencies: baseDependencies,
      pubspecs: pubspecs,
      tree: tree,
      doUpgrade: forceUpgrade,
      isolateEnvironment:
          forceUpgrade || isPrintPaths || isPrintTransitiveClosure || cherryPickPackage != null,
      reportDependenciesToTree:
          forceUpgrade || isPrintPaths || isPrintTransitiveClosure || cherryPickPackage != null,
    );

    // Only delete the synthetic package if it was done in a temp directory
    if (stringArg('synthetic-package-path') == null) {
      _syntheticPackageDir.deleteSync(recursive: true);
    }

    if (forceUpgrade || isPrintTransitiveClosure || isPrintPaths || cherryPickPackage != null) {
      _processPubspecs(tree: tree, pubspecs: pubspecs, specialDependencies: specialDependencies);

      if (isPrintTransitiveClosure) {
        tree._dependencyTree.forEach((String from, Set<String> to) {
          globals.printStatus('$from -> $to');
        });
        return FlutterCommandResult.success();
      }

      if (isPrintPaths) {
        showDependencyPaths(from: stringArg('from')!, to: stringArg('to')!, tree: tree);
        return FlutterCommandResult.success();
      }

      globals.printStatus('Updating workspace...');
      _updatePubspecs(tree: tree, pubspecs: pubspecs, specialDependencies: specialDependencies);
    }

    await _runPubGetOnPackages(packages);

    return FlutterCommandResult.success();
  }

  void _verifyPubspecs(List<Directory> packages) {
    bool needsUpdate = false;
    globals.printStatus('Verifying pubspecs...');
    for (final Directory directory in packages) {
      final PubspecYaml pubspec = PubspecYaml(directory);
      globals.printTrace('Reading pubspec.yaml from ${directory.path}');
      if (pubspec.checksum.value == null) {
        // If the checksum is invalid or missing, we can just ask them run to run
        // upgrade again to compute it.
        globals.printWarning(
          'Warning: pubspec in ${directory.path} has out of date dependencies. '
          'Please run "flutter update-packages --force-upgrade" to update them correctly.',
        );
        needsUpdate = true;
      }
      // all dependencies in the pubspec sorted lexically.
      final Map<String, String> checksumDependencies = <String, String>{};
      for (final PubspecLine data in pubspec.inputData) {
        if (data is PubspecDependency && data.kind == DependencyKind.normal) {
          checksumDependencies[data.name] = data.version;
        }
      }
      final String checksum = _computeChecksum(
        checksumDependencies.keys,
        (String name) => checksumDependencies[name]!,
      );
      if (checksum != pubspec.checksum.value) {
        // If the checksum doesn't match, they may have added or removed some dependencies.
        // we need to run update-packages to recapture the transitive deps.
        globals.printWarning(
          'Warning: pubspec in ${directory.path} has updated or new dependencies. '
          'Please run "flutter update-packages --force-upgrade" to update them correctly.',
          // DO NOT PRINT THE CHECKSUM HERE.
          // It causes people to ignore the requirement to actually run the script.
        );
        needsUpdate = true;
      } else {
        // everything is correct in the pubspec.
        globals.printTrace('pubspec in ${directory.path} is up to date!');
      }
    }
    if (needsUpdate) {
      throwToolExit(
        'Warning: one or more pubspecs have invalid dependencies. '
        'Please run "flutter update-packages --force-upgrade" to update them correctly.',
        exitCode: 1,
      );
    }
    globals.printStatus('All pubspecs were up to date.');
  }

  void _collectDependencies({
    required List<Directory> packages,
    required List<PubspecYaml> pubspecs,
    required Set<String> specialDependencies,
    required Map<String, PubspecDependency> explicitDependencies,
    required Map<String, PubspecDependency> allDependencies,
    required bool printPaths,
  }) {
    // Visit all the directories with pubspec.yamls we care about.
    for (final Directory directory in packages) {
      if (printPaths) {
        globals.printTrace('Reading pubspec.yaml from: ${directory.path}');
      }
      final PubspecYaml pubspec = PubspecYaml(directory); // this parses the pubspec.yaml
      pubspecs.add(pubspec); // remember it for later
      for (final PubspecDependency dependency in pubspec.allDependencies) {
        if (allDependencies.containsKey(dependency.name)) {
          // If we've seen the dependency before, make sure that we are
          // importing it the same way. There's several ways to import a
          // dependency. Hosted (from pub via version number), by path (e.g.
          // pointing at the version of a package we get from the Dart SDK
          // that we download with Flutter), by SDK (e.g. the "flutter"
          // package is explicitly from "sdk: flutter").
          //
          // This makes sure that we don't import a package in two different
          // ways, e.g. by saying "sdk: flutter" in one pubspec.yaml and
          // saying "path: ../../..." in another.
          final PubspecDependency previous = allDependencies[dependency.name]!;
          if (dependency.kind != previous.kind || dependency._lockTarget != previous._lockTarget) {
            throwToolExit(
              'Inconsistent requirements around ${dependency.name}; '
              'saw ${dependency.kind} (${dependency._lockTarget}) in "${dependency.sourcePath}" '
              'and ${previous.kind} (${previous._lockTarget}) in "${previous.sourcePath}".',
            );
          }
          if (dependency.version != previous.version) {
            globals.printError(
              'Requiring multiple versions: multiple versions required by ${dependency.name}; '
              'saw ${dependency.version} in "${dependency.sourcePath}" '
              'and ${previous.version} in "${previous.sourcePath}".',
            );
          }
        }
        allDependencies[dependency.name] = dependency;
      }
      for (final PubspecDependency dependency in pubspec.allExplicitDependencies) {
        if (explicitDependencies.containsKey(dependency.name)) {
          // If we've seen the dependency before, make sure that we are
          // importing it the same way. There's several ways to import a
          // dependency. Hosted (from pub via version number), by path (e.g.
          // pointing at the version of a package we get from the Dart SDK
          // that we download with Flutter), by SDK (e.g. the "flutter"
          // package is explicitly from "sdk: flutter").
          //
          // This makes sure that we don't import a package in two different
          // ways, e.g. by saying "sdk: flutter" in one pubspec.yaml and
          // saying "path: ../../..." in another.
          final PubspecDependency previous = explicitDependencies[dependency.name]!;
          if (dependency.kind != previous.kind || dependency._lockTarget != previous._lockTarget) {
            throwToolExit(
              'Inconsistent requirements around ${dependency.name}; '
              'saw ${dependency.kind} (${dependency._lockTarget}) in "${dependency.sourcePath}" '
              'and ${previous.kind} (${previous._lockTarget}) in "${previous.sourcePath}".',
            );
          }
        }
        // Remember this dependency by name so we can look it up again.
        explicitDependencies[dependency.name] = dependency;
        // Normal dependencies are those we get from pub. The others we
        // already implicitly pin since we pull down one version of the
        // Flutter and Dart SDKs, so we track which those are here so that we
        // can omit them from our list of pinned dependencies later.
        if (dependency.kind != DependencyKind.normal) {
          specialDependencies.add(dependency.name);
        }
      }
    }
  }

  Future<void> _pubGetAllDependencies({
    required Directory tempDir,
    required Iterable<PubspecDependency> dependencies,
    required List<PubspecYaml> pubspecs,
    required PubDependencyTree tree,
    required bool doUpgrade,
    required bool isolateEnvironment,
    required bool reportDependenciesToTree,
  }) async {
    Directory? temporaryFlutterSdk;
    final Directory syntheticPackageDir = tempDir.childDirectory('synthetic_package');
    final File fakePackage = _pubspecFor(syntheticPackageDir);
    fakePackage.createSync(recursive: true);
    fakePackage.writeAsStringSync(generateFakePubspec(dependencies, doUpgrade: doUpgrade));

    if (isolateEnvironment) {
      // Create a synthetic flutter SDK so that transitive flutter SDK
      // constraints are not affected by this upgrade.
      temporaryFlutterSdk = createTemporaryFlutterSdk(
        globals.logger,
        globals.fs,
        globals.fs.directory(Cache.flutterRoot),
        pubspecs,
        tempDir,
      );
    }

    // Run "pub get" on it in order to force the download of any
    // needed packages to the pub cache, upgrading if requested.
    await pub.get(
      context: PubContext.updatePackages,
      project: FlutterProject.fromDirectory(syntheticPackageDir),
      upgrade: doUpgrade,
      offline: boolArg('offline'),
      flutterRootOverride: temporaryFlutterSdk?.path,
      outputMode: PubOutputMode.failuresOnly,
    );

    if (reportDependenciesToTree) {
      // Run "pub deps --style=compact" on the result.
      // We pipe all the output to tree.fill(), which parses it so that it can
      // create a graph of all the dependencies so that we can figure out the
      // transitive dependencies later. It also remembers which version was
      // selected for each package.
      await pub.batch(
        <String>['deps', '--style=compact'],
        context: PubContext.updatePackages,
        directory: syntheticPackageDir.path,
        filter: tree.fill,
      );
    }
  }

  void _processPubspecs({
    required PubDependencyTree tree,
    required List<PubspecYaml> pubspecs,
    required Set<String> specialDependencies,
  }) {
    for (final PubspecYaml pubspec in pubspecs) {
      final String package = pubspec.name;
      specialDependencies.add(package);
      tree._versions[package] = pubspec.version;
      for (final PubspecDependency dependency in pubspec.dependencies) {
        if (dependency.kind == DependencyKind.normal) {
          tree._dependencyTree[package] ??= <String>{};
          tree._dependencyTree[package]!.add(dependency.name);
        }
      }
    }
  }

  bool _updatePubspecs({
    required PubDependencyTree tree,
    required List<PubspecYaml> pubspecs,
    required Set<String> specialDependencies,
  }) {
    // Now that we have collected all the data, we can apply our dependency
    // versions to each pubspec.yaml that we collected. This mutates the
    // pubspec.yaml files.
    //
    // The specialDependencies argument is the set of package names to not pin
    // to specific versions because they are explicitly pinned by their
    // constraints. Here we list the names we earlier established we didn't
    // need to pin because they come from the Dart or Flutter SDKs.
    for (final PubspecYaml pubspec in pubspecs) {
      pubspec.apply(tree, specialDependencies);
    }
    return false;
  }

  Future<void> _runPubGetOnPackages(List<Directory> packages) async {
    final Stopwatch timer = Stopwatch()..start();
    int count = 0;

    // Now we run pub get on each of the affected packages to update their
    // pubspec.lock files with the right transitive dependencies.
    //
    // This can be expensive, so we run them in parallel. If we hadn't already
    // warmed the cache above, running them in parallel could be dangerous due
    // to contention when unpacking downloaded dependencies, but since we have
    // downloaded all that we need, it is safe to run them in parallel.
    final Status status = globals.logger.startProgress(
      'Running "flutter pub get" in affected packages...',
    );
    try {
      // int.tryParse will not accept null, but will convert empty string to null
      final int? maxJobs = int.tryParse(stringArg('jobs') ?? '');
      final TaskQueue<void> queue = TaskQueue<void>(maxJobs: maxJobs);
      for (final Directory dir in packages) {
        unawaited(
          queue.add(() async {
            final Stopwatch stopwatch = Stopwatch();
            stopwatch.start();
            await pub.get(
              context: PubContext.updatePackages,
              project: FlutterProject.fromDirectory(dir),
              // All dependencies should already have been downloaded by the fake
              // package, so the concurrent checks can all happen offline.
              offline: true,
              outputMode: PubOutputMode.failuresOnly,
            );
            stopwatch.stop();
            final double seconds = stopwatch.elapsedMilliseconds / 1000.0;
            final String relativeDir = globals.fs.path.relative(dir.path, from: Cache.flutterRoot);
            globals.printStatus('Ran pub get in $relativeDir in ${seconds.toStringAsFixed(1)}s...');
          }),
        );
        count += 1;
      }
      unawaited(
        queue.add(() async {
          final Stopwatch stopwatch = Stopwatch();
          await _downloadCoverageData();
          stopwatch.stop();
          final double seconds = stopwatch.elapsedMilliseconds / 1000.0;
          globals.printStatus(
            'Downloaded lcov data for package:flutter in ${seconds.toStringAsFixed(1)}s...',
          );
        }),
      );
      await queue.tasksComplete;
      status.stop();
      // The exception is rethrown, so don't catch only Exceptions.
    } catch (exception) {
      // ignore: avoid_catches_without_on_clauses
      status.cancel();
      rethrow;
    }

    final double seconds = timer.elapsedMilliseconds / 1000.0;
    globals.printStatus(
      "\nRan 'pub get' $count time${count == 1 ? "" : "s"} and fetched coverage data in ${seconds.toStringAsFixed(1)}s.",
    );
  }

  void showDependencyPaths({
    required String from,
    required String to,
    required PubDependencyTree tree,
  }) {
    if (!tree.contains(from)) {
      throwToolExit('Package $from not found in the dependency tree.');
    }
    if (!tree.contains(to)) {
      throwToolExit('Package $to not found in the dependency tree.');
    }

    final Queue<_DependencyLink> traversalQueue = Queue<_DependencyLink>();
    final Set<String> visited = <String>{};
    final List<_DependencyLink> paths = <_DependencyLink>[];

    traversalQueue.addFirst(_DependencyLink(from: null, to: from));
    while (traversalQueue.isNotEmpty) {
      final _DependencyLink link = traversalQueue.removeLast();
      if (link.to == to) {
        paths.add(link);
      }
      if (link.from != null) {
        visited.add(link.from!.to);
      }
      for (final String dependency in tree._dependencyTree[link.to]!) {
        if (!visited.contains(dependency)) {
          traversalQueue.addFirst(_DependencyLink(from: link, to: dependency));
        }
      }
    }

    for (_DependencyLink? path in paths) {
      final StringBuffer buf = StringBuffer();
      while (path != null) {
        buf.write(path.to);
        path = path.from;
        if (path != null) {
          buf.write(' <- ');
        }
      }
      globals.printStatus(buf.toString(), wrap: false);
    }

    if (paths.isEmpty) {
      globals.printStatus('No paths found from $from to $to');
    }
  }
}

class _DependencyLink {
  _DependencyLink({required this.from, required this.to});

  final _DependencyLink? from;
  final String to;

  @override
  String toString() => '${from?.to} -> $to';
}

/// The various sections of a pubspec.yaml file.
///
/// We care about the "dependencies", "dev_dependencies", and
/// "dependency_overrides" sections, as well as the "name" and "version" fields
/// in the pubspec header bucketed into [header]. The others are all bucketed
/// into [other].
enum Section { header, dependencies, devDependencies, dependencyOverrides, builders, other }

/// The various kinds of dependencies we know and care about.
enum DependencyKind {
  // Dependencies that will be path or sdk dependencies but
  // for which we haven't yet parsed the data.
  unknown,

  // Regular dependencies with a specified version range.
  normal,

  // Dependency that uses an explicit path, e.g. into the Dart SDK.
  path,

  // Dependency defined as coming from an SDK (typically "sdk: flutter").
  sdk,

  // A dependency that was "normal", but for which we later found a "path" or
  // "sdk" dependency in the dependency_overrides section.
  overridden,

  // A dependency that uses git.
  git,
}

/// This is the string we output next to each of our autogenerated transitive
/// dependencies so that we can ignore them the next time we parse the
/// pubspec.yaml file.
const String kTransitiveMagicString =
    '# THIS LINE IS AUTOGENERATED - TO UPDATE USE "flutter update-packages --force-upgrade"';

/// This is the string output before a checksum of the packages used.
const String kDependencyChecksum = '# PUBSPEC CHECKSUM: ';

/// This class represents a pubspec.yaml file for the purposes of upgrading the
/// dependencies as done by this file.
class PubspecYaml {
  /// You create one of these by providing a directory, from which we obtain the
  /// pubspec.yaml and parse it into a line-by-line form.
  factory PubspecYaml(Directory directory) {
    final File file = _pubspecFor(directory);
    return _parse(file, file.readAsLinesSync());
  }

  PubspecYaml._(this.file, this.name, this.version, this.inputData, this.checksum);

  final File file; // The actual pubspec.yaml file.

  /// The package name.
  final String name;

  /// The package version.
  final String? version;

  final List<PubspecLine> inputData; // Each line of the pubspec.yaml file, parsed(ish).

  /// The package checksum.
  ///
  /// If this was not found in the pubspec, a synthetic checksum is created
  /// with a value of `-1`.
  final PubspecChecksum checksum;

  /// This parses each line of a pubspec.yaml file (a list of lines) into
  /// slightly more structured data (in the form of a list of PubspecLine
  /// objects). We don't just use a YAML parser because we care about comments
  /// and also because we can just define the style of pubspec.yaml files we care
  /// about (since they're all under our control).
  static PubspecYaml _parse(File file, List<String> lines) {
    final String filename = file.path;
    String? packageName;
    String? packageVersion;
    PubspecChecksum?
    checksum; // the checksum value used to verify that dependencies haven't changed.
    final List<PubspecLine> result = <PubspecLine>[]; // The output buffer.
    Section section = Section.other; // Which section we're currently reading from.
    bool seenMain = false; // Whether we've seen the "dependencies:" section.
    bool seenDev = false; // Whether we've seen the "dev_dependencies:" section.
    // The masterDependencies map is used to keep track of the objects
    // representing actual dependencies we've seen so far in this file so that
    // if we see dependency overrides we can update the actual dependency so it
    // knows that it's not really a dependency.
    final Map<String, PubspecDependency> masterDependencies = <String, PubspecDependency>{};
    // The "special" dependencies (the ones that use git: or path: or sdk: or
    // whatnot) have the style of having extra data after the line that declares
    // the dependency. So we track what is the "current" (or "last") dependency
    // that we are dealing with using this variable.
    PubspecDependency? lastDependency;
    for (int index = 0; index < lines.length; index += 1) {
      String line = lines[index];
      if (lastDependency == null) {
        // First we look to see if we're transitioning to a new top-level section.
        // The PubspecHeader.parse static method can recognize those headers.
        final PubspecHeader? header = PubspecHeader.parse(line); // See if it's a header.
        if (header != null) {
          // It is!
          section = header.section; // The parser determined what kind of section it is.
          if (section == Section.header) {
            if (header.name == 'name') {
              packageName = header.value;
            } else if (header.name == 'version') {
              packageVersion = header.value;
            }
          } else if (section == Section.dependencies) {
            // If we're entering the "dependencies" section, we want to make sure that
            // it's the first section (of those we care about) that we've seen so far.
            if (seenMain) {
              throwToolExit(
                'Two dependencies sections found in $filename. There should only be one.',
              );
            }
            if (seenDev) {
              throwToolExit(
                'The dependencies section was after the dev_dependencies section in $filename. '
                'To enable one-pass processing, the dependencies section must come before the '
                'dev_dependencies section.',
              );
            }
            seenMain = true;
          } else if (section == Section.devDependencies) {
            // Similarly, if we're entering the dev_dependencies section, we should verify
            // that we've not seen one already.
            if (seenDev) {
              throwToolExit(
                'Two dev_dependencies sections found in $filename. There should only be one.',
              );
            }
            seenDev = true;
          }
          result.add(header);
        } else if (section == Section.builders) {
          // Do nothing.
          // This line isn't a section header, and we're not in a section we care about.
          // We just stick the line into the output unmodified.
          result.add(PubspecLine(line));
        } else if (section == Section.other) {
          if (line.contains(kDependencyChecksum)) {
            // This is the pubspec checksum. After computing it, we remove it from the output data
            // since it will be recomputed later.
            checksum = PubspecChecksum.parse(line);
          } else {
            // This line isn't a section header, and we're not in a section we care about.
            // We just stick the line into the output unmodified.
            result.add(PubspecLine(line));
          }
        } else {
          // We're in a section we care about. Try to parse out the dependency:
          final PubspecDependency? dependency = PubspecDependency.parse(
            line,
            filename: filename,
            isDevDependency: seenDev,
          );
          if (dependency != null) {
            // We got one!
            result.add(dependency);
            if (dependency.kind == DependencyKind.unknown) {
              // If we didn't get a version number, then we need to be ready to
              // read the next line as part of this dependency, so keep track of
              // this dependency object.
              lastDependency = dependency;
            }
            if (section != Section.dependencyOverrides) {
              // If we're not in the overrides section, then just remember the
              // dependency, in case it comes up again later in the overrides
              // section.
              //
              // First, make sure it's a unique dependency. Listing dependencies
              // twice doesn't make sense.
              if (masterDependencies.containsKey(dependency.name)) {
                throwToolExit('$filename contains two dependencies on ${dependency.name}.');
              }
              masterDependencies[dependency.name] = dependency;
            } else {
              // If we _are_ in the overrides section, then go tell the version
              // we saw earlier (if any -- there might not be, we might be
              // overriding a transitive dependency) that we have overridden it,
              // so that later when we output the dependencies we can leave
              // the line unmodified.
              masterDependencies[dependency.name]?.markOverridden(dependency);
            }
          } else if (line.contains(kDependencyChecksum)) {
            // This is the pubspec checksum. After computing it, we remove it from the output data
            // since it will be recomputed later.
            checksum = PubspecChecksum.parse(line);
          } else {
            // We're in a section we care about but got a line we didn't
            // recognize. Maybe it's a comment or a blank line or something.
            // Just pass it through.
            result.add(PubspecLine(line));
          }
        }
      } else {
        // If we're here it means the last line was a dependency that needed
        // extra information to be parsed from the next line.
        //
        // Try to parse the line by giving it to the last PubspecDependency
        // object we created. If parseLock fails to recognize the line, it will
        // throw. If it does recognize the line and needs the following lines in
        // its lockLine, it'll return false.
        // Otherwise it returns true.
        //
        // If it returns true, then it will have updated itself internally to
        // store the information from this line.
        if (!lastDependency.parseLock(
          line,
          filename,
          lockIsOverride: section == Section.dependencyOverrides,
        )) {
          // Ok we're dealing with some "git:" dependency. Consume lines until
          // we are out of the git dependency, and stuff them into the lock
          // line.
          lastDependency._lockLine = line;
          lastDependency._lockIsOverride = section == Section.dependencyOverrides;
          do {
            index += 1;
            if (index == lines.length) {
              throw StateError(
                'Invalid pubspec.yaml: a "git" dependency section terminated early.',
              );
            }
            line = lines[index];
            lastDependency._lockLine = '${lastDependency._lockLine}\n$line';
          } while (line.startsWith('   '));
        }
        // We're done with this special dependency, so reset back to null so
        // we'll go in the top section next time instead.
        lastDependency = null;
      }
    }
    return PubspecYaml._(
      file,
      packageName!,
      packageVersion,
      result,
      checksum ?? PubspecChecksum(null, ''),
    );
  }

  /// This returns all the explicit dependencies that this pubspec.yaml lists under dependencies.
  Iterable<PubspecDependency> get dependencies {
    // It works by iterating over the parsed data from _parse above, collecting
    // all the dependencies that were found, ignoring any that are flagged as as
    // overridden by subsequent entries in the same file and any that have the
    // magic comment flagging them as auto-generated transitive dependencies
    // that we added in a previous run.
    return inputData.whereType<PubspecDependency>().where(
      (PubspecDependency data) =>
          data.kind != DependencyKind.overridden && !data.isTransitive && !data.isDevDependency,
    );
  }

  /// This returns all regular dependencies and all dev dependencies.
  Iterable<PubspecDependency> get allExplicitDependencies {
    return inputData.whereType<PubspecDependency>().where(
      (PubspecDependency data) => data.kind != DependencyKind.overridden && !data.isTransitive,
    );
  }

  /// This returns all dependencies.
  Iterable<PubspecDependency> get allDependencies {
    return inputData.whereType<PubspecDependency>();
  }

  /// Take a dependency graph with explicit version numbers, and apply them to
  /// the pubspec.yaml, ignoring any that we know are special dependencies (those
  /// that depend on the Flutter or Dart SDK directly and are thus automatically
  /// pinned).
  void apply(PubDependencyTree versions, Set<String> specialDependencies) {
    final List<String> output = <String>[]; // the string data to output to the file, line by line
    final Set<String> directDependencies =
        <String>{}; // packages this pubspec directly depends on (i.e. not transitive)
    final Set<String> devDependencies = <String>{};
    Section section = Section.other; // the section we're currently handling

    // the line number where we're going to insert the transitive dependencies.
    int? endOfDirectDependencies;
    // The line number where we're going to insert the transitive dev dependencies.
    int? endOfDevDependencies;
    // Walk the pre-parsed input file, outputting it unmodified except for
    // updating version numbers, removing the old transitive dependencies lines,
    // and adding our new transitive dependencies lines. We also do a little
    // cleanup, removing trailing spaces, removing double-blank lines, leading
    // blank lines, and trailing blank lines, and ensuring the file ends with a
    // newline. This cleanup lets us be a little more aggressive while building
    // the output.
    for (final PubspecLine data in inputData) {
      if (data is PubspecHeader) {
        // This line was a header of some sort.
        //
        // If we're leaving one of the sections in which we can list transitive
        // dependencies, then remember this as the current last known valid
        // place to insert our transitive dependencies.
        if (section == Section.dependencies) {
          endOfDirectDependencies = output.length;
        }
        if (section == Section.devDependencies) {
          endOfDevDependencies = output.length;
        }
        section = data.section; // track which section we're now in.
        output.add(data.line); // insert the header into the output
      } else if (data is PubspecDependency) {
        // This was a dependency of some sort.
        // How we handle this depends on the section.
        switch (section) {
          case Section.devDependencies:
          case Section.dependencies:
            // For the dependencies and dev_dependencies sections, we reinsert
            // the dependency if it wasn't one of our autogenerated transitive
            // dependency lines.
            if (!data.isTransitive) {
              // Assert that we haven't seen it in this file already.
              assert(
                !directDependencies.contains(data.name) && !devDependencies.contains(data.name),
              );
              if (data.kind == DependencyKind.normal) {
                // This is a regular dependency, so we need to update the
                // version number.
                //
                // We output data that matches the format that
                // PubspecDependency.parse can handle. The data.suffix is any
                // previously-specified trailing comment.
                assert(versions.contains(data.name), "versions doesn't contain ${data.name}");
                output.add('  ${data.name}: ${versions.versionFor(data.name)}${data.suffix}');
              } else {
                // If it wasn't a regular dependency, then we output the line
                // unmodified. If there was an additional line (e.g. an "sdk:
                // flutter" line) then we output that too.
                output.add(data.line);
                if (data.lockLine != null) {
                  output.add(data.lockLine!);
                }
              }
              // Remember that we've dealt with this dependency so we don't
              // mention it again when doing the transitive dependencies.
              if (section == Section.dependencies) {
                directDependencies.add(data.name);
              } else {
                devDependencies.add(data.name);
              }
            }
            // Since we're in one of the places where we can list dependencies,
            // remember this as the current last known valid place to insert our
            // transitive dev dependencies. If the section is for regular dependencies,
            // then also remember the line for the end of direct dependencies.
            if (section == Section.dependencies) {
              endOfDirectDependencies = output.length;
            }
            endOfDevDependencies = output.length;
          case Section.builders:
          case Section.dependencyOverrides:
          case Section.header:
          case Section.other:
            // In other sections, pass everything through in its original form.
            output.add(data.line);
            if (data.lockLine != null) {
              output.add(data.lockLine!);
            }
        }
      } else {
        // Not a header, not a dependency, just pass that through unmodified.
        output.add(data.line);
      }
    }

    // If there are no dependencies or dev_dependencies sections, these will be
    // null. We have such files in our tests, so account for them here.
    endOfDirectDependencies ??= output.length;
    endOfDevDependencies ??= output.length;

    // Now include all the transitive dependencies and transitive dev dependencies.
    // The blocks of text to insert for each dependency section.
    final List<String> transitiveDependencyOutput = <String>[];
    final List<String> transitiveDevDependencyOutput = <String>[];

    // Which dependencies we need to handle for the transitive and dev dependency sections.
    final Set<String> transitiveDependencies = <String>{};
    final Set<String> transitiveDevDependencies = <String>{};

    // Merge the lists of dependencies we've seen in this file from dependencies, dev dependencies,
    // and the dependencies we know this file mentions that are already pinned
    // (and which didn't get special processing above).
    final Set<String> implied = <String>{
      ...directDependencies,
      ...specialDependencies,
      ...devDependencies,
      ...kExplicitlyExcludedPackages,
    };

    // Create a new set to hold the list of packages we've already processed, so
    // that we don't redundantly process them multiple times.
    final Set<String> done = <String>{};
    for (final String package in directDependencies) {
      transitiveDependencies.addAll(
        versions.getTransitiveDependenciesFor(package, seen: done, exclude: implied),
      );
    }
    for (final String package in devDependencies) {
      transitiveDevDependencies.addAll(
        versions.getTransitiveDependenciesFor(package, seen: done, exclude: implied),
      );
    }

    // Sort each dependency block lexically so that we don't get noisy diffs when upgrading.
    final List<String> transitiveDependenciesAsList = transitiveDependencies.toList()..sort();
    final List<String> transitiveDevDependenciesAsList = transitiveDevDependencies.toList()..sort();

    String computeTransitiveDependencyLineFor(String package) {
      return '  $package: ${versions.versionFor(package)} $kTransitiveMagicString';
    }

    // Add a line for each transitive dependency and transitive dev dependency using our magic string to recognize them later.
    for (final String package in transitiveDependenciesAsList) {
      transitiveDependencyOutput.add(computeTransitiveDependencyLineFor(package));
    }
    for (final String package in transitiveDevDependenciesAsList) {
      transitiveDevDependencyOutput.add(computeTransitiveDependencyLineFor(package));
    }

    // Build a sorted list of all dependencies for the checksum.
    final Set<String> checksumDependencies = <String>{
      ...directDependencies,
      ...devDependencies,
      ...transitiveDependenciesAsList,
      ...transitiveDevDependenciesAsList,
    }..removeAll(specialDependencies);

    // Add a blank line before and after each section to keep the resulting output clean.
    transitiveDependencyOutput
      ..insert(0, '')
      ..add('');
    transitiveDevDependencyOutput
      ..insert(0, '')
      ..add('');

    // Compute a new checksum from all sorted dependencies and their version and convert to a hex string.
    final String checksumString = _computeChecksum(checksumDependencies, versions.versionFor);

    // Insert the block of transitive dependency declarations into the output after [endOfDirectDependencies],
    // and the blocks of transitive dev dependency declarations into the output after [lastPossiblePlace]. Finally,
    // insert the [checksumString] at the very end.
    output
      ..insertAll(endOfDevDependencies, transitiveDevDependencyOutput)
      ..insertAll(endOfDirectDependencies, transitiveDependencyOutput)
      ..add('')
      ..add('$kDependencyChecksum$checksumString');

    // Remove trailing lines.
    while (output.last.isEmpty) {
      output.removeLast();
    }

    // Output the result to the pubspec.yaml file, skipping leading and
    // duplicate blank lines and removing trailing spaces.
    final StringBuffer contents = StringBuffer();
    bool hadBlankLine = true;
    for (String line in output) {
      line = line.trimRight();
      if (line == '') {
        if (!hadBlankLine) {
          contents.writeln();
        }
        hadBlankLine = true;
      } else {
        contents.writeln(line);
        hadBlankLine = false;
      }
    }
    file.writeAsStringSync(contents.toString());
  }
}

/// This is the base class for the objects that represent lines in the
/// pubspec.yaml files.
class PubspecLine {
  PubspecLine(this.line);

  /// The raw line as we saw it in the original file. This is used so that we can
  /// output the same line unmodified for the majority of lines.
  final String line;
}

/// A checksum of the non autogenerated dependencies.
class PubspecChecksum extends PubspecLine {
  PubspecChecksum(this.value, String line) : super(line);

  /// The checksum value, computed using [Object.hash] over the direct, dev,
  /// and special dependencies sorted lexically.
  ///
  /// If the line cannot be parsed, [value] will be null.
  final String? value;

  /// Parses a [PubspecChecksum] from a line.
  ///
  /// The returned PubspecChecksum will have a null [value] if no checksum could
  /// be found on this line. This is a value that [_computeChecksum] cannot return.
  static PubspecChecksum parse(String line) {
    final List<String> tokens = line.split(kDependencyChecksum);
    if (tokens.length != 2) {
      return PubspecChecksum(null, line);
    }
    return PubspecChecksum(tokens.last.trim(), line);
  }
}

/// A header, e.g. "dependencies:".
class PubspecHeader extends PubspecLine {
  PubspecHeader(super.line, this.section, {this.name, this.value});

  /// The section of the pubspec where the parse [line] appears.
  final Section section;

  /// The name in the pubspec line providing a name/value pair, such as "name"
  /// and "version".
  ///
  /// Example:
  ///
  /// The value of this field extracted from the following line is "version".
  ///
  /// ```none
  /// version: 0.16.5
  /// ```
  final String? name;

  /// The value in the pubspec line providing a name/value pair, such as "name"
  /// and "version".
  ///
  /// Example:
  ///
  /// The value of this field extracted from the following line is "0.16.5".
  ///
  /// ```none
  /// version: 0.16.5
  /// ```
  final String? value;

  static PubspecHeader? parse(String line) {
    // We recognize any line that:
    //  * doesn't start with a space (i.e. is aligned on the left edge)
    //  * ignoring trailing spaces and comments, ends with a colon
    //  * has contents before the colon
    // We also try to recognize which of the kinds of Sections it is
    // by comparing those contents against known strings.
    if (line.startsWith(' ')) {
      return null;
    }
    final String strippedLine = _stripComments(line);
    if (!strippedLine.contains(':') || strippedLine.length <= 1) {
      return null;
    }
    final List<String> parts = strippedLine.split(':');
    final String sectionName = parts.first;
    final String value = parts.last.trim();
    return switch (sectionName) {
      'dependencies' => PubspecHeader(line, Section.dependencies),
      'dev_dependencies' => PubspecHeader(line, Section.devDependencies),
      'dependency_overrides' => PubspecHeader(line, Section.dependencyOverrides),
      'builders' => PubspecHeader(line, Section.builders),
      'name' || 'version' => PubspecHeader(line, Section.header, name: sectionName, value: value),
      _ => PubspecHeader(line, Section.other),
    };
  }

  /// Returns the input after removing trailing spaces and anything after the
  /// first "#".
  static String _stripComments(String line) {
    final int hashIndex = line.indexOf('#');
    if (hashIndex < 0) {
      return line.trimRight();
    }
    return line.substring(0, hashIndex).trimRight();
  }
}

/// A dependency, as represented by a line (or two) from a pubspec.yaml file.
class PubspecDependency extends PubspecLine {
  PubspecDependency(
    super.line,
    this.name,
    this.suffix, {
    required this.isTransitive,
    required DependencyKind kind,
    required this.version,
    required this.sourcePath,
    required this.isDevDependency,
  }) : _kind = kind;

  static PubspecDependency? parse(
    String line, {
    required String filename,
    required bool isDevDependency,
  }) {
    // We recognize any line that:
    //  * starts with exactly two spaces, no more or less
    //  * has some content, then a colon
    //
    // If we recognize the line, then we look to see if there's anything after
    // the colon, ignoring comments. If there is, then this is a normal
    // dependency, otherwise it's an unknown one.
    //
    // We also try and save the version string, if any. This is used to verify
    // the checksum of package deps.
    //
    // We also look at the trailing comment, if any, to see if it is the magic
    // string that identifies the line as a transitive dependency that we
    // previously pinned, so we can ignore it.
    //
    // We remember the trailing comment, if any, so that we can reconstruct the
    // line later. We forget the specified version range, if any.
    if (line.length < 4 || line.startsWith('   ') || !line.startsWith('  ')) {
      return null;
    }
    final int colonIndex = line.indexOf(':');
    final int hashIndex = line.indexOf('#');
    if (colonIndex < 3) {
      // two spaces at 0 and 1, a character at 2
      return null;
    }
    if (hashIndex >= 0 && hashIndex < colonIndex) {
      return null;
    }
    final String package = line.substring(2, colonIndex).trimRight();
    assert(package.isNotEmpty);
    assert(line.startsWith('  $package'));
    String suffix = '';
    bool isTransitive = false;
    String stripped;
    String version = '';
    if (hashIndex >= 0) {
      assert(hashIndex > colonIndex);
      final String trailingComment = line.substring(hashIndex, line.length);
      assert(line.endsWith(trailingComment));
      isTransitive = trailingComment == kTransitiveMagicString;
      suffix = ' $trailingComment';
      stripped = line.substring(colonIndex + 1, hashIndex).trimRight();
    } else {
      stripped = line.substring(colonIndex + 1, line.length).trimRight();
    }
    if (colonIndex != -1) {
      version = line.substring(colonIndex + 1, hashIndex != -1 ? hashIndex : line.length).trim();
    }
    return PubspecDependency(
      line,
      package,
      suffix,
      isTransitive: isTransitive,
      version: version,
      kind: stripped.isEmpty ? DependencyKind.unknown : DependencyKind.normal,
      sourcePath: filename,
      isDevDependency: isDevDependency,
    );
  }

  final String name; // the package name
  final String suffix; // any trailing comment we found
  final String version; // the version string if found, or blank.
  final bool isTransitive; // whether the suffix matched kTransitiveMagicString
  final String sourcePath; // the filename of the pubspec.yaml file, for error messages
  final bool isDevDependency; // Whether this dependency is under the `dev dependencies` section.

  DependencyKind get kind => _kind;
  DependencyKind _kind = DependencyKind.normal;

  /// If we're a path or sdk dependency, the path or sdk in question.
  String? _lockTarget;

  /// If we were a two-line dependency, the second line (see the inherited [line]
  /// for the first).
  String? get lockLine => _lockLine;
  String? _lockLine;

  /// If we're a path or sdk dependency, whether we were found in a
  /// dependencies/dev_dependencies section, or a dependency_overrides section.
  /// We track this so that we can put ourselves in the right section when
  /// generating the fake pubspec.yaml.
  bool _lockIsOverride = false;

  static const String _pathPrefix = '    path: ';
  static const String _sdkPrefix = '    sdk: ';
  static const String _gitPrefix = '    git:';

  PubspecDependency copyWith({
    String? line,
    String? name,
    String? suffix,
    bool? isTransitive,
    DependencyKind? kind,
    String? version,
    String? sourcePath,
    bool? isDevDependency,
  }) {
    return PubspecDependency(
      line ?? this.line,
      name ?? this.name,
      suffix ?? this.suffix,
      isTransitive: isTransitive ?? this.isTransitive,
      kind: kind ?? this.kind,
      version: version ?? this.version,
      sourcePath: sourcePath ?? this.sourcePath,
      isDevDependency: isDevDependency ?? this.isDevDependency,
    );
  }

  /// Whether the dependency points to a package in the Flutter SDK.
  ///
  /// There are two ways one can point to a Flutter package:
  ///
  /// - Using a "sdk: flutter" dependency.
  /// - Using a "path" dependency that points somewhere in the Flutter
  ///   repository other than the "bin" directory.
  bool get pointsToSdk {
    if (_kind == DependencyKind.sdk) {
      return true;
    }

    final String? lockTarget = _lockTarget;
    if (_kind == DependencyKind.path &&
        lockTarget != null &&
        !globals.fs.path.isWithin(globals.fs.path.join(Cache.flutterRoot!, 'bin'), lockTarget) &&
        globals.fs.path.isWithin(Cache.flutterRoot!, lockTarget)) {
      return true;
    }

    return false;
  }

  /// If parse decided we were a two-line dependency, this is called to parse the second line.
  /// We throw if we couldn't parse this line.
  /// We return true if we parsed it and stored the line in lockLine.
  /// We return false if we parsed it and it's a git dependency that needs the next few lines.
  bool parseLock(String line, String pubspecPath, {required bool lockIsOverride}) {
    assert(kind == DependencyKind.unknown);
    if (line.startsWith(_pathPrefix)) {
      // We're a path dependency; remember the (absolute) path.
      _lockTarget = globals.fs.path.canonicalize(
        globals.fs.path.absolute(
          globals.fs.path.dirname(pubspecPath),
          line.substring(_pathPrefix.length, line.length),
        ),
      );
      _kind = DependencyKind.path;
    } else if (line.startsWith(_sdkPrefix)) {
      // We're an SDK dependency.
      _lockTarget = line.substring(_sdkPrefix.length, line.length);
      _kind = DependencyKind.sdk;
    } else if (line.startsWith(_gitPrefix)) {
      // We're a git: dependency. We'll have to get the next few lines.
      _kind = DependencyKind.git;
      return false;
    } else {
      throwToolExit('Could not parse additional details for dependency $name; line was: "$line"');
    }
    _lockIsOverride = lockIsOverride;
    _lockLine = line;
    return true;
  }

  void markOverridden(PubspecDependency sibling) {
    // This is called when we find a dependency is mentioned a second time,
    // first in dependencies/dev_dependencies, and then in dependency_overrides.
    // It is called on the one found in dependencies/dev_dependencies, so that
    // we'll later know to report our version as "any" in the fake pubspec.yaml
    // and unmodified in the official pubspec.yamls.
    assert(sibling.name == name);
    assert(sibling.sourcePath == sourcePath);
    assert(sibling.kind != DependencyKind.normal);
    _kind = DependencyKind.overridden;
  }

  /// This generates the entry for this dependency for the pubspec.yaml for the
  /// fake package that we'll use to get the version numbers figured out.
  ///
  /// When called with [allowUpgrade] as [true], the version constrains will be set
  /// to >= whatever the previous version was. If [allowUpgrade] is [false], then
  /// the previous version is used again as an exact pin.
  void describeForFakePubspec(
    StringBuffer dependencies,
    StringBuffer overrides, {
    bool allowUpgrade = true,
  }) {
    final String versionToUse;
    // This should only happen when manually adding new dependencies; otherwise
    // versions should always be pinned exactly
    if (version.isEmpty || version == 'any') {
      versionToUse = 'any';
    } else if (allowUpgrade) {
      // Must wrap in quotes for Yaml parsing
      versionToUse = "'>= $version'";
    } else {
      versionToUse = version;
    }
    switch (kind) {
      case DependencyKind.unknown:
      case DependencyKind.overridden:
        assert(kind != DependencyKind.unknown);
      case DependencyKind.normal:
        if (!kManuallyPinnedDependencies.containsKey(name)) {
          dependencies.writeln('  $name: $versionToUse');
        }
      case DependencyKind.path:
        if (_lockIsOverride) {
          dependencies.writeln('  $name: $versionToUse');
          overrides.writeln('  $name:');
          overrides.writeln('    path: $_lockTarget');
        } else {
          dependencies.writeln('  $name:');
          dependencies.writeln('    path: $_lockTarget');
        }
      case DependencyKind.sdk:
        if (_lockIsOverride) {
          dependencies.writeln('  $name: $versionToUse');
          overrides.writeln('  $name:');
          overrides.writeln('    sdk: $_lockTarget');
        } else {
          dependencies.writeln('  $name:');
          dependencies.writeln('    sdk: $_lockTarget');
        }
      case DependencyKind.git:
        if (_lockIsOverride) {
          dependencies.writeln('  $name: $versionToUse');
          overrides.writeln('  $name:');
          overrides.writeln(lockLine);
        } else {
          dependencies.writeln('  $name:');
          dependencies.writeln(lockLine);
        }
    }
  }

  @override
  String toString() {
    return '$name: $version';
  }
}

/// Generates the File object for the pubspec.yaml file of a given Directory.
File _pubspecFor(Directory directory) {
  return directory.fileSystem.file(directory.fileSystem.path.join(directory.path, 'pubspec.yaml'));
}

/// Generates the source of a fake pubspec.yaml file given a list of
/// dependencies.
@visibleForTesting
String generateFakePubspec(Iterable<PubspecDependency> dependencies, {bool doUpgrade = false}) {
  final StringBuffer result = StringBuffer();
  final StringBuffer overrides = StringBuffer();
  final bool verbose = doUpgrade;
  result.writeln('name: flutter_update_packages');
  result.writeln('environment:');
  result.writeln("  sdk: '>=2.12.0 <4.0.0'");
  result.writeln('dependencies:');
  overrides.writeln('dependency_overrides:');
  if (kManuallyPinnedDependencies.isNotEmpty) {
    if (verbose) {
      globals.printStatus('WARNING: the following packages use hard-coded version constraints:');
    }
    final Set<String> allTransitive = <String>{
      for (final PubspecDependency dependency in dependencies) dependency.name,
    };
    kManuallyPinnedDependencies.forEach((String package, String version) {
      // Don't add pinned dependency if it is not in the set of all transitive dependencies.
      if (!allTransitive.contains(package)) {
        if (verbose) {
          globals.printStatus(
            '  - $package: $version (skipped because it was not a transitive dependency)',
          );
        }
        return;
      }
      result.writeln('  $package: $version');
      if (verbose) {
        globals.printStatus('  - $package: $version');
      }
    });
  }
  if (verbose && kExplicitlyExcludedPackages.isNotEmpty) {
    globals.printStatus(
      'WARNING: the following packages are explicitly excluded from version pinning',
    );
    for (final String package in kExplicitlyExcludedPackages) {
      globals.printStatus('  - $package');
    }
  }
  for (final PubspecDependency dependency in dependencies) {
    if (!dependency.pointsToSdk) {
      dependency.describeForFakePubspec(result, overrides, allowUpgrade: doUpgrade);
    }
  }
  result.write(overrides.toString());
  return result.toString();
}

/// This object tracks the output of a call to "pub deps --style=compact".
///
/// It ends up holding the full graph of dependencies, and the version number for
/// each one.
class PubDependencyTree {
  final Map<String, String?> _versions = <String, String?>{};
  final Map<String, Set<String>> _dependencyTree = <String, Set<String>>{};

  /// Handles the output from "pub deps --style=compact".
  ///
  /// That output is of this form:
  ///
  /// ```none
  /// package_name 0.0.0
  ///
  /// dependencies:
  /// - analyzer 0.31.0-alpha.0 [watcher args package_config collection]
  /// - archive 1.0.31 [crypto args path]
  /// - args 0.13.7
  /// - cli_util 0.1.2+1 [path]
  ///
  /// dev dependencies:
  /// - async 1.13.3 [collection]
  /// - barback 0.15.2+11 [stack_trace source_span pool async collection path]
  ///
  /// dependency overrides:
  /// - analyzer 0.31.0-alpha.0 [watcher args package_config collection]
  /// ```
  ///
  /// We ignore all the lines that don't start with a hyphen. For each other
  /// line, we ignore any line that mentions a package we've already seen (this
  /// happens when the overrides section mentions something that was in the
  /// dependencies section). We ignore if something is a dependency or
  /// dev_dependency (pub won't use different versions for those two).
  ///
  /// We then parse out the package name, version number, and sub-dependencies for
  /// each entry, and store than in our _versions and _dependencyTree fields
  /// above.
  String? fill(String message) {
    if (message.startsWith('- ')) {
      final int space2 = message.indexOf(' ', 2);
      int space3 = message.indexOf(' ', space2 + 1);
      if (space3 < 0) {
        space3 = message.length;
      }
      final String package = message.substring(2, space2);
      if (!contains(package)) {
        // Some packages get listed in the dependency overrides section too.
        // We just ignore those. The data is the same either way.
        final String version = message.substring(space2 + 1, space3);
        List<String> dependencies;
        if (space3 < message.length) {
          assert(message[space3 + 1] == '[');
          assert(message[message.length - 1] == ']');
          final String allDependencies = message.substring(space3 + 2, message.length - 1);
          dependencies = allDependencies.split(' ');
        } else {
          dependencies = const <String>[];
        }
        _versions[package] = version;
        _dependencyTree[package] = Set<String>.of(dependencies);
      }
    }
    return null;
  }

  /// Whether we know about this package.
  bool contains(String package) {
    return _versions.containsKey(package);
  }

  /// The transitive closure of all the dependencies for the given package,
  /// excluding any listed in `seen`.
  Iterable<String> getTransitiveDependenciesFor(
    String package, {
    required Set<String> seen,
    required Set<String> exclude,
    List<String>? result,
  }) {
    result ??= <String>[];
    final Set<String>? dependencies = _dependencyTree[package];
    if (dependencies == null) {
      // We have no transitive dependencies extracted for flutter_sdk packages
      // because they were omitted from pubspec.yaml used for 'pub upgrade' run.
      return result;
    }
    for (final String dependency in dependencies) {
      if (!seen.contains(dependency)) {
        if (!exclude.contains(dependency)) {
          result.add(dependency);
        }
        seen.add(dependency);
        getTransitiveDependenciesFor(dependency, seen: seen, exclude: exclude, result: result);
      }
    }
    return result;
  }

  /// The version that a particular package ended up with.
  String versionFor(String package) {
    return _versions[package]!;
  }
}

// Produces a 16-bit checksum from the codePoints of the package name and
// version strings using Fletcher's algorithm.
String _computeChecksum(Iterable<String> names, String Function(String name) getVersion) {
  int lowerCheck = 0;
  int upperCheck = 0;
  final List<String> sortedNames = names.toList()..sort();
  for (final String name in sortedNames) {
    final String version = getVersion(name);
    final String value = '$name: $version';
    // Each code unit is 16 bits.
    for (final int codeUnit in value.codeUnits) {
      final int upper = codeUnit >> 8;
      final int lower = codeUnit & 0xFF;
      lowerCheck = (lowerCheck + upper) % 255;
      upperCheck = (upperCheck + lowerCheck) % 255;
      lowerCheck = (lowerCheck + lower) % 255;
      upperCheck = (upperCheck + lowerCheck) % 255;
    }
  }
  return ((upperCheck << 8) | lowerCheck).toRadixString(16).padLeft(4, '0');
}

/// Create a synthetic Flutter SDK so that pub version solving does not get
/// stuck on the old versions.
@visibleForTesting
Directory createTemporaryFlutterSdk(
  Logger logger,
  FileSystem fileSystem,
  Directory realFlutter,
  List<PubspecYaml> pubspecs,
  Directory tempDir,
) {
  final Set<String> currentPackages = <String>{};
  for (final FileSystemEntity entity in realFlutter.childDirectory('packages').listSync()) {
    // Verify that a pubspec.yaml exists to ensure this isn't a left over directory.
    if (entity is Directory && entity.childFile('pubspec.yaml').existsSync()) {
      currentPackages.add(fileSystem.path.basename(entity.path));
    }
  }

  final Map<String, PubspecYaml> pubspecsByName = <String, PubspecYaml>{};
  for (final PubspecYaml pubspec in pubspecs) {
    pubspecsByName[pubspec.name] = pubspec;
  }

  final Directory directory = tempDir.childDirectory('flutter_upgrade_sdk')..createSync();
  // Fill in version info.
  realFlutter.childFile('version').copySync(directory.childFile('version').path);
  final File versionJson = FlutterVersion.getVersionFile(realFlutter.fileSystem, realFlutter.path);
  final Directory binCacheDirectory = directory.childDirectory('bin').childDirectory('cache');
  binCacheDirectory.createSync(recursive: true);
  versionJson.copySync(binCacheDirectory.childFile('flutter.version.json').path);

  // Directory structure should mirror the current Flutter SDK
  final Directory packages = directory.childDirectory('packages');
  for (final String flutterPackage in currentPackages) {
    final File pubspecFile = packages.childDirectory(flutterPackage).childFile('pubspec.yaml')
      ..createSync(recursive: true);
    final PubspecYaml? pubspecYaml = pubspecsByName[flutterPackage];
    if (pubspecYaml == null) {
      logger.printWarning("Unexpected package '$flutterPackage' found in packages directory");
      continue;
    }
    final StringBuffer output = StringBuffer('name: $flutterPackage\n');

    // Fill in SDK dependency constraint.
    output.write('''
environment:
  sdk: ^3.7.0-0
''');

    output.writeln('dependencies:');
    for (final PubspecDependency dependency in pubspecYaml.dependencies) {
      if (dependency.isTransitive || dependency.isDevDependency) {
        continue;
      }
      if (dependency.kind == DependencyKind.sdk) {
        output.writeln('  ${dependency.name}:\n    sdk: flutter');
        continue;
      }
      output.writeln('  ${dependency.name}: any');
    }
    pubspecFile.writeAsStringSync(output.toString());
  }

  // Create the sky engine pubspec.yaml
  directory
      .childDirectory('bin')
      .childDirectory('cache')
      .childDirectory('pkg')
      .childDirectory('sky_engine')
      .childFile('pubspec.yaml')
    ..createSync(recursive: true)
    ..writeAsStringSync('''
name: sky_engine
version: 0.0.99
description: Dart SDK extensions for dart:ui
homepage: http://flutter.io
# sky_engine requires sdk_ext support in the analyzer which was added in 1.11.x
environment:
  sdk: ^3.7.0-0
''');

  return directory;
}
