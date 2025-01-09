// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';
import 'dart:math' as math;

import 'package:archive/archive_io.dart';
import 'package:args/args.dart';
import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:http/http.dart' as http;
import 'package:intl/intl.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart';
import 'package:process/process.dart';
import 'package:pub_semver/pub_semver.dart';

import 'dartdoc_checker.dart';

const String kDummyPackageName = 'Flutter';
const String kPlatformIntegrationPackageName = 'platform_integration';

/// Additional package dependencies that we want to have in the docs,
/// but not actually depend on them.
const Map<String, (String path, String version)> kFakeDependencies = <String, (String, String)>{
  'flutter_gpu': ('flutter_gpu/gpu.dart', '\n    sdk: flutter'),
};

class PlatformDocsSection {
  const PlatformDocsSection({
    required this.zipName,
    required this.sectionName,
    required this.checkFile,
    required this.subdir,
  });
  final String zipName;
  final String sectionName;
  final String checkFile;
  final String subdir;
}

const Map<String, PlatformDocsSection> kPlatformDocs = <String, PlatformDocsSection>{
  'android': PlatformDocsSection(
    zipName: 'android-javadoc.zip',
    sectionName: 'Android',
    checkFile: 'io/flutter/embedding/android/FlutterView.html',
    subdir: 'javadoc',
  ),
  'ios': PlatformDocsSection(
    zipName: 'ios-docs.zip',
    sectionName: 'iOS',
    checkFile: 'interface_flutter_view.html',
    subdir: 'ios-embedder',
  ),
  'macos': PlatformDocsSection(
    zipName: 'macos-docs.zip',
    sectionName: 'macOS',
    checkFile: 'interface_flutter_view.html',
    subdir: 'macos-embedder',
  ),
  'linux': PlatformDocsSection(
    zipName: 'linux-docs.zip',
    sectionName: 'Linux',
    checkFile: 'struct___fl_view.html',
    subdir: 'linux-embedder',
  ),
  'windows': PlatformDocsSection(
    zipName: 'windows-docs.zip',
    sectionName: 'Windows',
    checkFile: 'classflutter_1_1_flutter_view.html',
    subdir: 'windows-embedder',
  ),
  'impeller': PlatformDocsSection(
    zipName: 'impeller-docs.zip',
    sectionName: 'Impeller',
    checkFile: 'classimpeller_1_1_canvas.html',
    subdir: 'impeller',
  ),
};

/// This script will generate documentation for the packages in `packages/` and
/// write the documentation to the output directory specified on the command
/// line.
///
/// This script also updates the index.html file so that it can be placed at the
/// root of api.flutter.dev. The files are kept inside of
/// api.flutter.dev/flutter, so we need to manipulate paths a bit. See
/// https://github.com/flutter/flutter/issues/3900 for more info.
///
/// This will only work on UNIX systems, not Windows. It requires that 'git',
/// 'zip', and 'tar' be in the PATH. It requires that 'flutter' has been run
/// previously. It uses the version of Dart downloaded by the 'flutter' tool in
/// this repository and will fail if that is absent.
Future<void> main(List<String> arguments) async {
  const FileSystem filesystem = LocalFileSystem();
  const ProcessManager processManager = LocalProcessManager();
  const Platform platform = LocalPlatform();

  // The place to find customization files and configuration files for docs
  // generation.
  final Directory docsRoot =
      FlutterInformation.instance
          .getFlutterRoot()
          .childDirectory('dev')
          .childDirectory('docs')
          .absolute;
  final ArgParser argParser = _createArgsParser(
    publishDefault: docsRoot.childDirectory('doc').path,
  );
  final ArgResults args = argParser.parse(arguments);
  if (args['help'] as bool) {
    print('Usage:');
    print(argParser.usage);
    exit(0);
  }

  final Directory publishRoot = filesystem.directory(args['output-dir']! as String).absolute;
  final Directory packageRoot = publishRoot.parent;
  if (!filesystem.directory(packageRoot).existsSync()) {
    filesystem.directory(packageRoot).createSync(recursive: true);
  }

  if (!filesystem.directory(publishRoot).existsSync()) {
    filesystem.directory(publishRoot).createSync(recursive: true);
  }

  final Configurator configurator = Configurator(
    publishRoot: publishRoot,
    packageRoot: packageRoot,
    docsRoot: docsRoot,
    filesystem: filesystem,
    processManager: processManager,
    platform: platform,
  );
  configurator.generateConfiguration();

  final PlatformDocGenerator platformGenerator = PlatformDocGenerator(
    outputDir: publishRoot,
    filesystem: filesystem,
  );
  await platformGenerator.generatePlatformDocs();

  final DartdocGenerator dartdocGenerator = DartdocGenerator(
    publishRoot: publishRoot,
    packageRoot: packageRoot,
    docsRoot: docsRoot,
    filesystem: filesystem,
    processManager: processManager,
    useJson: args['json'] as bool? ?? true,
    validateLinks: args['validate-links']! as bool,
    verbose: args['verbose'] as bool? ?? false,
  );

  await dartdocGenerator.generateDartdoc();
  await configurator.generateOfflineAssetsIfNeeded();
}

ArgParser _createArgsParser({required String publishDefault}) {
  final ArgParser parser = ArgParser();
  parser.addFlag('help', abbr: 'h', negatable: false, help: 'Show command help.');
  parser.addFlag(
    'verbose',
    defaultsTo: true,
    help:
        'Whether to report all error messages (on) or attempt to '
        'filter out some known false positives (off). Shut this off '
        'locally if you want to address Flutter-specific issues.',
  );
  parser.addFlag(
    'json',
    help: 'Display json-formatted output from dartdoc and skip stdout/stderr prefixing.',
  );
  parser.addFlag(
    'validate-links',
    help: 'Display warnings for broken links generated by dartdoc (slow)',
  );
  parser.addOption(
    'output-dir',
    defaultsTo: publishDefault,
    help: 'Sets the output directory for the documentation.',
  );
  return parser;
}

/// A class used to configure the staging area for building the docs in.
///
/// The [generateConfiguration] function generates a dummy package with a
/// pubspec. It copies any assets and customization files from the framework
/// repo. It creates a metadata file for searches.
///
/// Once the docs have been generated, [generateOfflineAssetsIfNeeded] will
/// create offline assets like Dash/Zeal docsets and an offline ZIP file of the
/// site if the build is a CI build that is not a presubmit build.
class Configurator {
  Configurator({
    required this.docsRoot,
    required this.publishRoot,
    required this.packageRoot,
    required this.filesystem,
    required this.processManager,
    required this.platform,
  });

  /// The root of the directory in the Flutter repo where configuration data is
  /// stored.
  final Directory docsRoot;

  /// The root of the output area for the dartdoc docs.
  ///
  /// Typically this is a "doc" subdirectory under the [packageRoot].
  final Directory publishRoot;

  /// The root of the staging area for creating docs.
  final Directory packageRoot;

  /// The [FileSystem] object used to create [File] and [Directory] objects.
  final FileSystem filesystem;

  /// The [ProcessManager] object used to invoke external processes.
  ///
  /// Can be replaced by tests to have a fake process manager.
  final ProcessManager processManager;

  /// The [Platform] to use for this run.
  ///
  /// Can be replaced by tests to test behavior on different platforms.
  final Platform platform;

  void generateConfiguration() {
    final Version version = FlutterInformation.instance.getFlutterVersion();
    _createDummyPubspec();
    _createDummyLibrary();
    _createPageFooter(packageRoot, version);
    _copyCustomizations();
    _createSearchMetadata(
      docsRoot.childDirectory('lib').childFile('opensearch.xml'),
      publishRoot.childFile('opensearch.xml'),
    );
  }

  Future<void> generateOfflineAssetsIfNeeded() async {
    // Only create the offline docs if we're running in a non-presubmit build:
    // it takes too long otherwise.
    if (platform.environment.containsKey('LUCI_CI') &&
        (platform.environment['LUCI_PR'] ?? '').isEmpty) {
      _createOfflineZipFile();
      await _createDocset();
      _moveOfflineIntoPlace();
      _createRobotsTxt();
    }
  }

  /// Returns import or on-disk paths for all libraries in the Flutter SDK.
  Iterable<String> _libraryRefs() sync* {
    for (final Directory dir in findPackages(filesystem)) {
      final String dirName = dir.basename;
      for (final FileSystemEntity file in dir.childDirectory('lib').listSync()) {
        if (file is File && file.path.endsWith('.dart')) {
          yield '$dirName/${file.basename}';
        }
      }
    }

    // Add a fake references for libraries that we don't actually depend on so
    // that they will be included in the docs.
    for (final String package in kFakeDependencies.keys) {
      yield kFakeDependencies[package]!.$1;
    }

    // Add a fake package for platform integration APIs.
    yield '$kPlatformIntegrationPackageName/android.dart';
    yield '$kPlatformIntegrationPackageName/ios.dart';
    yield '$kPlatformIntegrationPackageName/macos.dart';
    yield '$kPlatformIntegrationPackageName/linux.dart';
    yield '$kPlatformIntegrationPackageName/windows.dart';
  }

  void _createDummyPubspec() {
    // Create the pubspec.yaml file.
    final List<String> pubspec = <String>[
      'name: $kDummyPackageName',
      'homepage: https://flutter.dev',
      'version: 0.0.0',
      'environment:',
      "  sdk: '>=3.2.0-0 <4.0.0'",
      'dependencies:',
      for (final String package in findPackageNames(filesystem)) '  $package:\n    sdk: flutter',
      '  $kPlatformIntegrationPackageName: 0.0.1',
      for (final String package in kFakeDependencies.keys)
        '  $package: ${kFakeDependencies[package]!.$2}',
      'dependency_overrides:',
      '  $kPlatformIntegrationPackageName:',
      '    path: ${docsRoot.childDirectory(kPlatformIntegrationPackageName).path}',
    ];

    packageRoot.childFile('pubspec.yaml').writeAsStringSync(pubspec.join('\n'));
  }

  void _createDummyLibrary() {
    final Directory libDir = packageRoot.childDirectory('lib');
    libDir.createSync();

    final StringBuffer contents = StringBuffer('library temp_doc;\n\n');
    for (final String libraryRef in _libraryRefs()) {
      contents.writeln("import 'package:$libraryRef';");
    }
    packageRoot.childDirectory('lib')
      ..createSync(recursive: true)
      ..childFile('temp_doc.dart').writeAsStringSync(contents.toString());
  }

  void _createPageFooter(Directory footerPath, Version version) {
    final String timestamp = DateFormat('yyyy-MM-dd HH:mm').format(DateTime.now());
    String channel = FlutterInformation.instance.getBranchName();
    // Backward compatibility: Still support running on "master", but pretend it is "main".
    if (channel == 'master') {
      channel = 'main';
    }
    final String gitRevision = FlutterInformation.instance.getFlutterRevision();
    final String channelOut = channel.isEmpty ? '' : '• $channel';
    footerPath.childFile('footer.html').writeAsStringSync('<script src="footer.js"></script>');
    publishRoot.childDirectory('flutter').childFile('footer.js')
      ..createSync(recursive: true)
      ..writeAsStringSync('''
(function() {
  var span = document.querySelector('footer>span');
  if (span) {
    span.innerText = 'Flutter $version • $timestamp • $gitRevision $channelOut';
  }
  var sourceLink = document.querySelector('a.source-link');
  if (sourceLink) {
    sourceLink.href = sourceLink.href.replace('/main/', '/$gitRevision/');
  }
})();
''');
  }

  void _copyCustomizations() {
    final List<String> files = <String>[
      'README.md',
      'analysis_options.yaml',
      'dartdoc_options.yaml',
    ];
    for (final String file in files) {
      final File source = docsRoot.childFile(file);
      final File destination = packageRoot.childFile(file);
      // Have to canonicalize because otherwise things like /foo/bar/baz and
      // /foo/../foo/bar/baz won't compare as identical.
      if (path.canonicalize(source.absolute.path) != path.canonicalize(destination.absolute.path)) {
        source.copySync(destination.path);
        print(
          'Copied ${path.canonicalize(source.absolute.path)} to ${path.canonicalize(destination.absolute.path)}',
        );
      }
    }
    final Directory assetsDir = filesystem.directory(publishRoot.childDirectory('assets'));
    final Directory assetSource = docsRoot.childDirectory('assets');
    if (path.canonicalize(assetSource.absolute.path) ==
        path.canonicalize(assetsDir.absolute.path)) {
      // Don't try and copy the directory over itself.
      return;
    }
    if (assetsDir.existsSync()) {
      assetsDir.deleteSync(recursive: true);
    }
    if (assetSource.existsSync()) {
      copyDirectorySync(
        assetSource,
        assetsDir,
        onFileCopied: (File src, File dest) {
          print(
            'Copied ${path.canonicalize(src.absolute.path)} to ${path.canonicalize(dest.absolute.path)}',
          );
        },
        filesystem: filesystem,
      );
    }
  }

  /// Generates an OpenSearch XML description that can be used to add a custom
  /// search for Flutter API docs to the browser. Unfortunately, it has to know
  /// the URL to which site to search, so we customize it here based upon the
  /// branch name.
  void _createSearchMetadata(File templatePath, File metadataPath) {
    final String template = templatePath.readAsStringSync();
    final String branch = FlutterInformation.instance.getBranchName();
    final String metadata = template.replaceAll(
      '{SITE_URL}',
      branch == 'stable' ? 'https://api.flutter.dev/' : 'https://main-api.flutter.dev/',
    );
    metadataPath.parent.create(recursive: true);
    metadataPath.writeAsStringSync(metadata);
  }

  Future<void> _createDocset() async {
    // Must have dashing installed: go get -u github.com/technosophos/dashing
    // Dashing produces a LOT of log output (~30MB), so we collect it, and just
    // show the end of it if there was a problem.
    print('${DateTime.now().toUtc()}: Building Flutter docset.');

    // If dashing gets stuck, Cirrus will time out the build after an hour, and we
    // never get to see the logs. Thus, we run it in the background and tail the
    // logs only if it fails.
    final ProcessWrapper result = ProcessWrapper(
      await processManager.start(<String>[
        'dashing',
        'build',
        '--source',
        publishRoot.path,
        '--config',
        docsRoot.childFile('dashing.json').path,
      ], workingDirectory: packageRoot.path),
    );
    final List<int> buffer = <int>[];
    result.stdout.listen(buffer.addAll);
    result.stderr.listen(buffer.addAll);
    // If the dashing process exited with an error, print the last 200 lines of stderr and exit.
    final int exitCode = await result.done;
    if (exitCode != 0) {
      print('Dashing docset generation failed with code $exitCode');
      final List<String> output = systemEncoding.decode(buffer).split('\n');
      print(output.sublist(math.max(output.length - 200, 0)).join('\n'));
      exit(exitCode);
    }
    buffer.clear();

    // Copy the favicon file to the output directory.
    final File faviconFile = publishRoot
        .childDirectory('flutter')
        .childDirectory('static-assets')
        .childFile('favicon.png');
    final File iconFile = packageRoot.childDirectory('flutter.docset').childFile('icon.png');
    faviconFile
      ..createSync(recursive: true)
      ..copySync(iconFile.path);

    // Post-process the dashing output.
    final File infoPlist = packageRoot
        .childDirectory('flutter.docset')
        .childDirectory('Contents')
        .childFile('Info.plist');
    String contents = infoPlist.readAsStringSync();

    // Since I didn't want to add the XML package as a dependency just for this,
    // I just used a regular expression to make this simple change.
    final RegExp findRe = RegExp(
      r'(\s*<key>DocSetPlatformFamily</key>\s*<string>)[^<]+(</string>)',
      multiLine: true,
    );
    contents = contents.replaceAllMapped(findRe, (Match match) {
      return '${match.group(1)}dartlang${match.group(2)}';
    });
    infoPlist.writeAsStringSync(contents);
    final Directory offlineDir = publishRoot.childDirectory('offline');
    if (!offlineDir.existsSync()) {
      offlineDir.createSync(recursive: true);
    }
    tarDirectory(
      packageRoot,
      offlineDir.childFile('flutter.docset.tar.gz'),
      processManager: processManager,
    );

    // Write the Dash/Zeal XML feed file.
    final bool isStable = platform.environment['LUCI_BRANCH'] == 'stable';
    offlineDir
        .childFile('flutter.xml')
        .writeAsStringSync(
          '<entry>\n'
          '  <version>${FlutterInformation.instance.getFlutterVersion()}</version>\n'
          '  <url>https://${isStable ? '' : 'main-'}api.flutter.dev/offline/flutter.docset.tar.gz</url>\n'
          '</entry>\n',
        );
  }

  // Creates the offline ZIP file containing all of the website HTML files.
  void _createOfflineZipFile() {
    print('${DateTime.now().toLocal()}: Creating offline docs archive.');
    zipDirectory(
      publishRoot,
      packageRoot.childFile('flutter.docs.zip'),
      processManager: processManager,
    );
  }

  // Moves the generated offline archives into the publish directory so that
  // they can be included in the output ZIP file.
  void _moveOfflineIntoPlace() {
    print('${DateTime.now().toUtc()}: Moving offline docs into place.');
    final Directory offlineDir = publishRoot.childDirectory('offline')..createSync(recursive: true);
    packageRoot
        .childFile('flutter.docs.zip')
        .renameSync(offlineDir.childFile('flutter.docs.zip').path);
  }

  // Creates a robots.txt file that disallows indexing unless the branch is the
  // stable branch.
  void _createRobotsTxt() {
    final File robotsTxt = publishRoot.childFile('robots.txt');
    if (FlutterInformation.instance.getBranchName() == 'stable') {
      robotsTxt.writeAsStringSync('# All robots welcome!');
    } else {
      robotsTxt.writeAsStringSync('User-agent: *\nDisallow: /');
    }
  }
}

/// Runs Dartdoc inside of the given pre-prepared staging area, prepared by
/// [Configurator.generateConfiguration].
///
/// Performs a sanity check of the output once the generation is complete.
class DartdocGenerator {
  DartdocGenerator({
    required this.docsRoot,
    required this.publishRoot,
    required this.packageRoot,
    required this.filesystem,
    required this.processManager,
    this.useJson = true,
    this.validateLinks = true,
    this.verbose = false,
  });

  /// The root of the directory in the Flutter repo where configuration data is
  /// stored.
  final Directory docsRoot;

  /// The root of the output area for the dartdoc docs.
  ///
  /// Typically this is a "doc" subdirectory under the [packageRoot].
  final Directory publishRoot;

  /// The root of the staging area for creating docs.
  final Directory packageRoot;

  /// The [FileSystem] object used to create [File] and [Directory] objects.
  final FileSystem filesystem;

  /// The [ProcessManager] object used to invoke external processes.
  ///
  /// Can be replaced by tests to have a fake process manager.
  final ProcessManager processManager;

  /// Whether or not dartdoc should output an index.json file of the
  /// documentation.
  final bool useJson;

  // Whether or not to have dartdoc validate its own links.
  final bool validateLinks;

  /// Whether or not to filter overly verbose log output from dartdoc.
  final bool verbose;

  Future<void> generateDartdoc() async {
    final Directory flutterRoot = FlutterInformation.instance.getFlutterRoot();
    final Map<String, String> pubEnvironment = <String, String>{
      'FLUTTER_ROOT': flutterRoot.absolute.path,
    };

    // If there's a .pub-cache dir in the Flutter root, use that.
    final File pubCache = flutterRoot.childFile('.pub-cache');
    if (pubCache.existsSync()) {
      pubEnvironment['PUB_CACHE'] = pubCache.path;
    }

    // Run pub.
    ProcessWrapper process = ProcessWrapper(
      await runPubProcess(
        arguments: <String>['get'],
        workingDirectory: packageRoot,
        environment: pubEnvironment,
        filesystem: filesystem,
        processManager: processManager,
      ),
    );
    printStream(process.stdout, prefix: 'pub:stdout: ');
    printStream(process.stderr, prefix: 'pub:stderr: ');
    final int code = await process.done;
    if (code != 0) {
      exit(code);
    }

    final Version version = FlutterInformation.instance.getFlutterVersion();

    // Verify which version of the global activated packages we're using.
    final ProcessResult versionResults = processManager.runSync(
      <String>[FlutterInformation.instance.getFlutterBinaryPath().path, 'pub', 'global', 'list'],
      workingDirectory: packageRoot.path,
      environment: pubEnvironment,
      stdoutEncoding: utf8,
    );
    print('');
    final Iterable<RegExpMatch> versionMatches = RegExp(
      r'^(?<name>dartdoc) (?<version>[^\s]+)',
      multiLine: true,
    ).allMatches(versionResults.stdout as String);
    for (final RegExpMatch match in versionMatches) {
      print('${match.namedGroup('name')} version: ${match.namedGroup('version')}');
    }

    print('flutter version: $version\n');

    // Dartdoc warnings and errors in these packages are considered fatal.
    // All packages owned by flutter should be in the list.
    final List<String> flutterPackages = <String>[
      kDummyPackageName,
      kPlatformIntegrationPackageName,
      ...findPackageNames(filesystem),
      // TODO(goderbauer): Figure out how to only include `dart:ui` of
      // `sky_engine` below, https://github.com/dart-lang/dartdoc/issues/2278.
      // 'sky_engine',
    ];

    // Generate the documentation. We don't need to exclude flutter_tools in
    // this list because it's not in the recursive dependencies of the package
    // defined at packageRoot
    final List<String> dartdocArgs = <String>[
      'global',
      'run',
      '--enable-asserts',
      'dartdoc',
      '--output',
      publishRoot.childDirectory('flutter').path,
      '--allow-tools',
      if (useJson) '--json',
      if (validateLinks) '--validate-links' else '--no-validate-links',
      '--link-to-source-excludes',
      flutterRoot.childDirectory('bin').childDirectory('cache').path,
      '--link-to-source-root',
      flutterRoot.path,
      '--link-to-source-uri-template',
      'https://github.com/flutter/flutter/blob/main/%f%#L%l%',
      '--inject-html',
      '--use-base-href',
      '--header',
      docsRoot.childFile('styles.html').path,
      '--header',
      docsRoot.childFile('analytics-header.html').path,
      '--header',
      docsRoot.childFile('survey.html').path,
      '--header',
      docsRoot.childFile('snippets.html').path,
      '--header',
      docsRoot.childFile('opensearch.html').path,
      '--footer',
      docsRoot.childFile('analytics-footer.html').path,
      '--footer-text',
      packageRoot.childFile('footer.html').path,
      '--allow-warnings-in-packages',
      flutterPackages.join(','),
      '--exclude-packages',
      <String>[
        'analyzer',
        'args',
        'barback',
        'csslib',
        'flutter_goldens',
        'flutter_goldens_client',
        'front_end',
        'fuchsia_remote_debug_protocol',
        'glob',
        'html',
        'http_multi_server',
        'io',
        'isolate',
        'js',
        'kernel',
        'logging',
        'mime',
        'mockito',
        'node_preamble',
        'plugin',
        'shelf',
        'shelf_packages_handler',
        'shelf_static',
        'shelf_web_socket',
        'utf',
        'watcher',
        'yaml',
      ].join(','),
      '--exclude',
      <String>[
        'dart:io/network_policy.dart', // dart-lang/dartdoc#2437
        'package:Flutter/temp_doc.dart',
        'package:http/browser_client.dart',
        'package:intl/intl_browser.dart',
        'package:matcher/mirror_matchers.dart',
        'package:quiver/io.dart',
        'package:quiver/mirrors.dart',
        'package:vm_service_client/vm_service_client.dart',
        'package:web_socket_channel/html.dart',
      ].join(','),
      '--favicon',
      docsRoot.childFile('favicon.ico').absolute.path,
      '--package-order',
      'flutter,Dart,$kPlatformIntegrationPackageName,flutter_test,flutter_driver',
      '--auto-include-dependencies',
    ];

    String quote(String arg) => arg.contains(' ') ? "'$arg'" : arg;
    print(
      'Executing: (cd "${packageRoot.path}" ; '
      '${FlutterInformation.instance.getFlutterBinaryPath().path} '
      'pub '
      '${dartdocArgs.map<String>(quote).join(' ')})',
    );

    process = ProcessWrapper(
      await runPubProcess(
        arguments: dartdocArgs,
        workingDirectory: packageRoot,
        environment: pubEnvironment,
        processManager: processManager,
      ),
    );
    printStream(
      process.stdout,
      prefix: useJson ? '' : 'dartdoc:stdout: ',
      filter: <Pattern>[
        if (!verbose) RegExp(r'^Generating docs for library '), // Unnecessary verbosity
      ],
    );
    printStream(
      process.stderr,
      prefix: useJson ? '' : 'dartdoc:stderr: ',
      filter: <Pattern>[
        if (!verbose)
          RegExp(
            // Remove warnings from packages outside our control
            r'^ warning: .+: \(.+[\\/]\.pub-cache[\\/]hosted[\\/]pub.dartlang.org[\\/].+\)',
          ),
      ],
    );
    final int exitCode = await process.done;

    if (exitCode != 0) {
      exit(exitCode);
    }

    _sanityCheckDocs();
    checkForUnresolvedDirectives(publishRoot.childDirectory('flutter'));

    _createIndexAndCleanup();

    print('Documentation written to ${publishRoot.path}');
  }

  void _sanityCheckExample(String fileString, String regExpString) {
    final File file = filesystem.file(fileString);
    if (file.existsSync()) {
      final RegExp regExp = RegExp(regExpString, dotAll: true);
      final String contents = file.readAsStringSync();
      if (!regExp.hasMatch(contents)) {
        throw Exception("Missing example code matching '$regExpString' in ${file.path}.");
      }
    } else {
      throw Exception(
        "Missing example code sanity test file ${file.path}. Either it didn't get published, or you might have to update the test to look at a different file.",
      );
    }
  }

  /// A subset of all generated doc files for [_sanityCheckDocs].
  @visibleForTesting
  List<File> get canaries {
    final Directory flutterDirectory = publishRoot.childDirectory('flutter');
    final Directory widgetsDirectory = flutterDirectory.childDirectory('widgets');

    return <File>[
      publishRoot.childDirectory('assets').childFile('overrides.css'),
      flutterDirectory.childDirectory('dart-io').childFile('File-class.html'),
      flutterDirectory.childDirectory('dart-ui').childFile('Canvas-class.html'),
      flutterDirectory
          .childDirectory('dart-ui')
          .childDirectory('Canvas')
          .childFile('drawRect.html'),
      flutterDirectory
          .childDirectory('flutter_driver')
          .childDirectory('FlutterDriver')
          .childFile('FlutterDriver.connectedTo.html'),
      flutterDirectory.childDirectory('flutter_gpu').childFile('flutter_gpu-library.html'),
      flutterDirectory
          .childDirectory('flutter_test')
          .childDirectory('WidgetTester')
          .childFile('pumpWidget.html'),
      flutterDirectory.childDirectory('material').childFile('Material-class.html'),
      flutterDirectory.childDirectory('material').childFile('Tooltip-class.html'),
      widgetsDirectory.childFile('Widget-class.html'),
      widgetsDirectory.childFile('Listener-class.html'),
    ];
  }

  /// Runs a sanity check by running a test.
  void _sanityCheckDocs([Platform platform = const LocalPlatform()]) {
    for (final File canary in canaries) {
      if (!canary.existsSync()) {
        throw Exception(
          'Missing "${canary.path}", which probably means the documentation failed to build correctly.',
        );
      }
    }
    // Make sure at least one example of each kind includes source code.
    final Directory widgetsDirectory = publishRoot
        .childDirectory('flutter')
        .childDirectory('widgets');

    // Check a "sample" example, any one will do.
    _sanityCheckExample(
      widgetsDirectory.childFile('showGeneralDialog.html').path,
      r'\s*<pre\s+id="longSnippet1".*<code\s+class="language-dart">\s*import &#39;package:flutter&#47;material.dart&#39;;',
    );

    // Check a "snippet" example, any one will do.
    _sanityCheckExample(
      widgetsDirectory.childDirectory('ModalRoute').childFile('barrierColor.html').path,
      r'\s*<pre.*id="sample-code">.*Color\s+get\s+barrierColor.*</pre>',
    );

    // Check a "dartpad" example, any one will do, and check for the correct URL
    // arguments.
    // Just use "main" for any branch other than "stable", just like it is done
    // in the snippet generator at https://github.com/flutter/assets-for-api-docs/blob/cc56972b8f03552fc5f9f9f1ef309efc6c93d7bc/packages/snippets/lib/src/snippet_generator.dart#L104.
    final String? luciBranch = platform.environment['LUCI_BRANCH']?.trim();
    final String expectedChannel = luciBranch == 'stable' ? 'stable' : 'main';
    final List<String> argumentRegExps = <String>[
      r'split=\d+',
      r'run=true',
      r'sample_id=widgets\.Listener\.\d+',
      'channel=$expectedChannel',
    ];
    for (final String argumentRegExp in argumentRegExps) {
      _sanityCheckExample(
        widgetsDirectory.childFile('Listener-class.html').path,
        r'\s*<iframe\s+class="snippet-dartpad"\s+src="'
        r'https:\/\/dartpad.dev\/embed-flutter.html\?.*?\b'
        '$argumentRegExp'
        r'\b.*">\s*<\/iframe>',
      );
    }
  }

  /// Creates a custom index.html because we try to maintain old
  /// paths. Cleanup unused index.html files no longer needed.
  void _createIndexAndCleanup() {
    print('\nCreating a custom index.html in ${publishRoot.childFile('index.html').path}');
    _copyIndexToRootOfDocs();
    _addHtmlBaseToIndex();
    _changePackageToSdkInTitlebar();
    _putRedirectInOldIndexLocation();
    _writeSnippetsIndexFile();
    print('\nDocs ready to go!');
  }

  void _copyIndexToRootOfDocs() {
    publishRoot
        .childDirectory('flutter')
        .childFile('index.html')
        .copySync(publishRoot.childFile('index.html').path);
  }

  void _changePackageToSdkInTitlebar() {
    final File indexFile = publishRoot.childFile('index.html');
    String indexContents = indexFile.readAsStringSync();
    indexContents = indexContents.replaceFirst(
      '<li><a href="https://flutter.dev">Flutter package</a></li>',
      '<li><a href="https://flutter.dev">Flutter SDK</a></li>',
    );

    indexFile.writeAsStringSync(indexContents);
  }

  void _addHtmlBaseToIndex() {
    final File indexFile = publishRoot.childFile('index.html');
    String indexContents = indexFile.readAsStringSync();
    indexContents = indexContents.replaceFirst(
      '</title>\n',
      '</title>\n  <base href="./flutter/">\n',
    );

    for (final String platform in kPlatformDocs.keys) {
      final String sectionName = kPlatformDocs[platform]!.sectionName;
      final String subdir = kPlatformDocs[platform]!.subdir;
      indexContents = indexContents.replaceAll(
        'href="$sectionName/$sectionName-library.html"',
        'href="../$subdir/index.html"',
      );
    }

    indexFile.writeAsStringSync(indexContents);
  }

  void _putRedirectInOldIndexLocation() {
    const String metaTag = '<meta http-equiv="refresh" content="0;URL=../index.html">';
    publishRoot.childDirectory('flutter').childFile('index.html').writeAsStringSync(metaTag);
  }

  void _writeSnippetsIndexFile() {
    final Directory snippetsDir = publishRoot.childDirectory('snippets');
    if (snippetsDir.existsSync()) {
      const JsonEncoder jsonEncoder = JsonEncoder.withIndent('    ');
      final Iterable<File> files = snippetsDir.listSync().whereType<File>().where(
        (File file) => path.extension(file.path) == '.json',
      );
      // Combine all the metadata into a single JSON array.
      final Iterable<String> fileContents = files.map((File file) => file.readAsStringSync());
      final List<dynamic> metadataObjects = fileContents.map<dynamic>(json.decode).toList();
      final String jsonArray = jsonEncoder.convert(metadataObjects);
      snippetsDir.childFile('index.json').writeAsStringSync(jsonArray);
    }
  }
}

/// Downloads and unpacks the platform specific documentation generated by the
/// engine build.
///
/// Unpacks and massages the data so that it can be properly included in the
/// output archive.
class PlatformDocGenerator {
  PlatformDocGenerator({required this.outputDir, required this.filesystem});

  final FileSystem filesystem;
  final Directory outputDir;
  final String engineRevision = FlutterInformation.instance.getEngineRevision();
  final String engineRealm = FlutterInformation.instance.getEngineRealm();

  /// This downloads an archive of platform docs for the engine from the artifact
  /// store and extracts them to the location used for Dartdoc.
  Future<void> generatePlatformDocs() async {
    final String realm = engineRealm.isNotEmpty ? '$engineRealm/' : '';

    for (final String platform in kPlatformDocs.keys) {
      final String zipFile = kPlatformDocs[platform]!.zipName;
      final String url =
          'https://storage.googleapis.com/${realm}flutter_infra_release/flutter/$engineRevision/$zipFile';
      await _extractDocs(url, platform, kPlatformDocs[platform]!, outputDir);
    }
  }

  /// Fetches the zip archive at the specified url.
  ///
  /// Returns null if the archive fails to download after [maxTries] attempts.
  Future<Archive?> _fetchArchive(String url, int maxTries) async {
    List<int>? responseBytes;
    for (int i = 0; i < maxTries; i++) {
      final http.Response response = await http.get(Uri.parse(url));
      if (response.statusCode == 200) {
        responseBytes = response.bodyBytes;
        break;
      }
      stderr.writeln('Failed attempt ${i + 1} to fetch $url.');

      // On failure print a short snipped from the body in case it's helpful.
      final int bodyLength = math.min(1024, response.body.length);
      stderr.writeln(
        'Response status code ${response.statusCode}. Body: ${response.body.substring(0, bodyLength)}',
      );
      sleep(const Duration(seconds: 1));
    }
    return responseBytes == null ? null : ZipDecoder().decodeBytes(responseBytes);
  }

  Future<void> _extractDocs(
    String url,
    String name,
    PlatformDocsSection platform,
    Directory outputDir,
  ) async {
    const int maxTries = 5;
    final Archive? archive = await _fetchArchive(url, maxTries);
    if (archive == null) {
      stderr.writeln('Failed to fetch zip archive from: $url after $maxTries attempts. Giving up.');
      exit(1);
    }

    final Directory output = outputDir.childDirectory(platform.subdir);
    print('Extracting ${platform.zipName} to ${output.path}');
    output.createSync(recursive: true);

    for (final ArchiveFile af in archive) {
      if (!af.name.endsWith('/')) {
        final File file = filesystem.file('${output.path}/${af.name}');
        file.createSync(recursive: true);
        file.writeAsBytesSync(af.content as List<int>);
      }
    }

    final File testFile = output.childFile(platform.checkFile);
    if (!testFile.existsSync()) {
      print('Expected file ${testFile.path} not found');
      exit(1);
    }
    print('${platform.sectionName} ready to go!');
  }
}

/// Recursively copies `srcDir` to `destDir`, invoking [onFileCopied], if
/// specified, for each source/destination file pair.
///
/// Creates `destDir` if needed.
void copyDirectorySync(
  Directory srcDir,
  Directory destDir, {
  void Function(File srcFile, File destFile)? onFileCopied,
  required FileSystem filesystem,
}) {
  if (!srcDir.existsSync()) {
    throw Exception('Source directory "${srcDir.path}" does not exist, nothing to copy');
  }

  if (!destDir.existsSync()) {
    destDir.createSync(recursive: true);
  }

  for (final FileSystemEntity entity in srcDir.listSync()) {
    final String newPath = path.join(destDir.path, path.basename(entity.path));
    if (entity is File) {
      final File newFile = filesystem.file(newPath);
      entity.copySync(newPath);
      onFileCopied?.call(entity, newFile);
    } else if (entity is Directory) {
      copyDirectorySync(entity, filesystem.directory(newPath), filesystem: filesystem);
    } else {
      throw Exception('${entity.path} is neither File nor Directory');
    }
  }
}

void printStream(
  Stream<List<int>> stream, {
  String prefix = '',
  List<Pattern> filter = const <Pattern>[],
}) {
  stream.transform<String>(utf8.decoder).transform<String>(const LineSplitter()).listen((
    String line,
  ) {
    if (!filter.any((Pattern pattern) => line.contains(pattern))) {
      print('$prefix$line'.trim());
    }
  });
}

void zipDirectory(Directory src, File output, {required ProcessManager processManager}) {
  // We would use the archive package to do this in one line, but it
  // is a lot slower, and doesn't do compression nearly as well.
  final ProcessResult zipProcess = processManager.runSync(<String>[
    'zip',
    '-r',
    '-9',
    '-q',
    output.path,
    '.',
  ], workingDirectory: src.path);

  if (zipProcess.exitCode != 0) {
    print('Creating offline ZIP archive ${output.path} failed:');
    print(zipProcess.stderr);
    exit(1);
  }
}

void tarDirectory(Directory src, File output, {required ProcessManager processManager}) {
  // We would use the archive package to do this in one line, but it
  // is a lot slower, and doesn't do compression nearly as well.
  final ProcessResult tarProcess = processManager.runSync(<String>[
    'tar',
    'cf',
    output.path,
    '--use-compress-program',
    'gzip --best',
    'flutter.docset',
  ], workingDirectory: src.path);

  if (tarProcess.exitCode != 0) {
    print('Creating a tarball ${output.path} failed:');
    print(tarProcess.stderr);
    exit(1);
  }
}

Future<Process> runPubProcess({
  required List<String> arguments,
  Directory? workingDirectory,
  Map<String, String>? environment,
  @visibleForTesting ProcessManager processManager = const LocalProcessManager(),
  @visibleForTesting FileSystem filesystem = const LocalFileSystem(),
}) {
  return processManager.start(
    <Object>[FlutterInformation.instance.getFlutterBinaryPath().path, 'pub', ...arguments],
    workingDirectory: (workingDirectory ?? filesystem.currentDirectory).path,
    environment: environment,
  );
}

List<String> findPackageNames(FileSystem filesystem) {
  return findPackages(
    filesystem,
  ).map<String>((FileSystemEntity file) => path.basename(file.path)).toList();
}

/// Finds all packages in the Flutter SDK
List<Directory> findPackages(FileSystem filesystem) {
  return FlutterInformation.instance
      .getFlutterRoot()
      .childDirectory('packages')
      .listSync()
      .where((FileSystemEntity entity) {
        if (entity is! Directory) {
          return false;
        }
        final File pubspec = entity.childFile('pubspec.yaml');
        if (!pubspec.existsSync()) {
          print("Unexpected package '${entity.path}' found in packages directory");
          return false;
        }
        // Would be nice to use a real YAML parser here, but we don't want to
        // depend on a whole package for it, and this is sufficient.
        return !pubspec.readAsStringSync().contains('nodoc: true');
      })
      .cast<Directory>()
      .toList();
}

/// An exception class used to indicate problems when collecting information.
class FlutterInformationException implements Exception {
  FlutterInformationException(this.message);
  final String message;

  @override
  String toString() {
    return '$runtimeType: $message';
  }
}

/// A singleton used to consolidate the way in which information about the
/// Flutter repo and environment is collected.
///
/// Collects the information once, and caches it for any later access.
///
/// The singleton instance can be overridden by tests by setting [instance].
class FlutterInformation {
  FlutterInformation({
    this.platform = const LocalPlatform(),
    this.processManager = const LocalProcessManager(),
    this.filesystem = const LocalFileSystem(),
  });

  final Platform platform;
  final ProcessManager processManager;
  final FileSystem filesystem;

  static FlutterInformation? _instance;

  static FlutterInformation get instance => _instance ??= FlutterInformation();

  @visibleForTesting
  static set instance(FlutterInformation? value) => _instance = value;

  /// The path to the Dart binary in the Flutter repo.
  ///
  /// This is probably a shell script.
  File getFlutterBinaryPath() {
    return getFlutterRoot().childDirectory('bin').childFile('flutter');
  }

  /// The path to the Flutter repo root directory.
  ///
  /// If the environment variable `FLUTTER_ROOT` is set, will use that instead
  /// of looking for it.
  ///
  /// Otherwise, uses the output of `flutter --version --machine` to find the
  /// Flutter root.
  Directory getFlutterRoot() {
    if (platform.environment['FLUTTER_ROOT'] != null) {
      return filesystem.directory(platform.environment['FLUTTER_ROOT']);
    }
    return getFlutterInformation()['flutterRoot']! as Directory;
  }

  /// Gets the semver version of the Flutter framework in the repo.
  Version getFlutterVersion() => getFlutterInformation()['frameworkVersion']! as Version;

  /// Gets the git hash of the engine used by the Flutter framework in the repo.
  String getEngineRevision() => getFlutterInformation()['engineRevision']! as String;

  /// Gets the value stored in bin/internal/engine.realm used by the Flutter
  /// framework repo.
  String getEngineRealm() => getFlutterInformation()['engineRealm']! as String;

  /// Gets the git hash of the Flutter framework in the repo.
  String getFlutterRevision() => getFlutterInformation()['flutterGitRevision']! as String;

  /// Gets the name of the current branch in the Flutter framework in the repo.
  String getBranchName() => getFlutterInformation()['branchName']! as String;

  Map<String, Object>? _cachedFlutterInformation;

  /// Gets a Map of various kinds of information about the Flutter repo.
  Map<String, Object> getFlutterInformation() {
    if (_cachedFlutterInformation != null) {
      return _cachedFlutterInformation!;
    }

    String flutterVersionJson;
    if (platform.environment['FLUTTER_VERSION'] != null) {
      flutterVersionJson = platform.environment['FLUTTER_VERSION']!;
    } else {
      // Determine which flutter command to run, which will determine which
      // flutter root is eventually used. If the FLUTTER_ROOT is set, then use
      // that flutter command, otherwise use the first one in the PATH.
      String flutterCommand;
      if (platform.environment['FLUTTER_ROOT'] != null) {
        flutterCommand =
            filesystem
                .directory(platform.environment['FLUTTER_ROOT'])
                .childDirectory('bin')
                .childFile('flutter')
                .absolute
                .path;
      } else {
        flutterCommand = 'flutter';
      }
      ProcessResult result;
      try {
        result = processManager.runSync(<String>[
          flutterCommand,
          '--version',
          '--machine',
        ], stdoutEncoding: utf8);
      } on ProcessException catch (e) {
        throw FlutterInformationException(
          'Unable to determine Flutter information. Either set FLUTTER_ROOT, or place the '
          'flutter command in your PATH.\n$e',
        );
      }
      if (result.exitCode != 0) {
        throw FlutterInformationException(
          'Unable to determine Flutter information, because of abnormal exit of flutter command.',
        );
      }
      // Strip out any non-JSON that might be printed along with the command
      // output.
      flutterVersionJson = (result.stdout as String).replaceAll(
        'Waiting for another flutter command to release the startup lock...',
        '',
      );
    }

    final Map<String, dynamic> flutterVersion =
        json.decode(flutterVersionJson) as Map<String, dynamic>;
    if (flutterVersion['flutterRoot'] == null ||
        flutterVersion['frameworkVersion'] == null ||
        flutterVersion['dartSdkVersion'] == null) {
      throw FlutterInformationException(
        'Flutter command output has unexpected format, unable to determine flutter root location.',
      );
    }

    final Map<String, Object> info = <String, Object>{};
    final Directory flutterRoot = filesystem.directory(flutterVersion['flutterRoot']! as String);
    info['flutterRoot'] = flutterRoot;
    info['frameworkVersion'] = Version.parse(flutterVersion['frameworkVersion'] as String);
    info['engineRevision'] = flutterVersion['engineRevision'] as String;
    final File engineRealm = flutterRoot
        .childDirectory('bin')
        .childDirectory('internal')
        .childFile('engine.realm');
    info['engineRealm'] = engineRealm.existsSync() ? engineRealm.readAsStringSync().trim() : '';

    final RegExpMatch? dartVersionRegex = RegExp(
      r'(?<base>[\d.]+)(?:\s+\(build (?<detail>[-.\w]+)\))?',
    ).firstMatch(flutterVersion['dartSdkVersion'] as String);
    if (dartVersionRegex == null) {
      throw FlutterInformationException(
        'Flutter command output has unexpected format, unable to parse dart SDK version ${flutterVersion['dartSdkVersion']}.',
      );
    }
    info['dartSdkVersion'] = Version.parse(
      dartVersionRegex.namedGroup('detail') ?? dartVersionRegex.namedGroup('base')!,
    );

    info['branchName'] = _getBranchName();
    info['flutterGitRevision'] = _getFlutterGitRevision();
    _cachedFlutterInformation = info;

    return info;
  }

  // Get the name of the release branch.
  //
  // On LUCI builds, the git HEAD is detached, so first check for the env
  // variable "LUCI_BRANCH"; if it is not set, fall back to calling git.
  String _getBranchName() {
    final String? luciBranch = platform.environment['LUCI_BRANCH'];
    if (luciBranch != null && luciBranch.trim().isNotEmpty) {
      return luciBranch.trim();
    }
    final ProcessResult gitResult = processManager.runSync(<String>[
      'git',
      'status',
      '-b',
      '--porcelain',
    ]);
    if (gitResult.exitCode != 0) {
      throw 'git status exit with non-zero exit code: ${gitResult.exitCode}';
    }
    final RegExp gitBranchRegexp = RegExp(r'^## (.*)');
    final RegExpMatch? gitBranchMatch = gitBranchRegexp.firstMatch(
      (gitResult.stdout as String).trim().split('\n').first,
    );
    return gitBranchMatch == null ? '' : gitBranchMatch.group(1)!.split('...').first;
  }

  // Get the git revision for the repo.
  String _getFlutterGitRevision() {
    const int kGitRevisionLength = 10;

    final ProcessResult gitResult = processManager.runSync(<String>['git', 'rev-parse', 'HEAD']);
    if (gitResult.exitCode != 0) {
      throw 'git rev-parse exit with non-zero exit code: ${gitResult.exitCode}';
    }
    final String gitRevision = (gitResult.stdout as String).trim();

    return gitRevision.length > kGitRevisionLength
        ? gitRevision.substring(0, kGitRevisionLength)
        : gitRevision;
  }
}
