// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:meta/meta.dart';
import 'package:package_config/package_config.dart';
import 'package:standard_message_codec/standard_message_codec.dart';

import 'base/common.dart';
import 'base/context.dart';
import 'base/deferred_component.dart';
import 'base/file_system.dart';
import 'base/logger.dart';
import 'base/platform.dart';
import 'base/utils.dart';
import 'build_info.dart';
import 'cache.dart';
import 'convert.dart';
import 'dart/package_map.dart';
import 'devfs.dart';
import 'flutter_manifest.dart';
import 'license_collector.dart';
import 'project.dart';

const String defaultManifestPath = 'pubspec.yaml';

const String kFontManifestJson = 'FontManifest.json';

// Should match '2x', '/1x', '1.5x', etc.
final RegExp _assetVariantDirectoryRegExp = RegExp(r'/?(\d+(\.\d*)?)x$');

/// The effect of adding `uses-material-design: true` to the pubspec is to insert
/// the following snippet into the asset manifest:
///
/// ```yaml
/// material:
///   - family: MaterialIcons
///     fonts:
///       - asset: fonts/MaterialIcons-Regular.otf
/// ```
const List<Map<String, Object>> kMaterialFonts = <Map<String, Object>>[
  <String, Object>{
    'family': 'MaterialIcons',
    'fonts': <Map<String, String>>[
      <String, String>{'asset': 'fonts/MaterialIcons-Regular.otf'},
    ],
  },
];

const List<String> kMaterialShaders = <String>['shaders/ink_sparkle.frag'];

/// Injected factory class for spawning [AssetBundle] instances.
abstract class AssetBundleFactory {
  /// The singleton instance, pulled from the [AppContext].
  static AssetBundleFactory get instance => context.get<AssetBundleFactory>()!;

  static AssetBundleFactory defaultInstance({
    required Logger logger,
    required FileSystem fileSystem,
    required Platform platform,
    bool splitDeferredAssets = false,
  }) => _ManifestAssetBundleFactory(
    logger: logger,
    fileSystem: fileSystem,
    platform: platform,
    splitDeferredAssets: splitDeferredAssets,
  );

  /// Creates a new [AssetBundle].
  AssetBundle createBundle();
}

enum AssetKind { regular, font, shader, model }

/// Contains all information about an asset needed by tool the to prepare and
/// copy an asset file to the build output.
final class AssetBundleEntry {
  AssetBundleEntry(this.content, {required this.kind, required this.transformers});

  final DevFSContent content;
  final AssetKind kind;
  final List<AssetTransformerEntry> transformers;

  Future<List<int>> contentsAsBytes() => content.contentsAsBytes();

  bool hasEquivalentConfigurationWith(AssetBundleEntry other) {
    return listEquals(transformers, other.transformers);
  }
}

abstract class AssetBundle {
  /// The files that were specified under the `assets` section in the pubspec,
  /// indexed by asset key.
  Map<String, AssetBundleEntry> get entries;

  /// The files that were specified under the deferred components assets sections
  /// in a pubspec, indexed by component name and asset key.
  Map<String, Map<String, AssetBundleEntry>> get deferredComponentsEntries;

  /// Additional files that this bundle depends on that are not included in the
  /// output result.
  List<File> get additionalDependencies;

  /// Input files used to build this asset bundle.
  List<File> get inputFiles;

  bool wasBuiltOnce();

  bool needsBuild({String manifestPath = defaultManifestPath});

  /// Returns 0 for success; non-zero for failure.
  Future<int> build({
    String manifestPath = defaultManifestPath,
    required String packageConfigPath,
    bool deferredComponentsEnabled = false,
    TargetPlatform? targetPlatform,
    String? flavor,
  });
}

class _ManifestAssetBundleFactory implements AssetBundleFactory {
  _ManifestAssetBundleFactory({
    required Logger logger,
    required FileSystem fileSystem,
    required Platform platform,
    bool splitDeferredAssets = false,
  }) : _logger = logger,
       _fileSystem = fileSystem,
       _platform = platform,
       _splitDeferredAssets = splitDeferredAssets;

  final Logger _logger;
  final FileSystem _fileSystem;
  final Platform _platform;
  final bool _splitDeferredAssets;

  @override
  AssetBundle createBundle() => ManifestAssetBundle(
    logger: _logger,
    fileSystem: _fileSystem,
    platform: _platform,
    flutterRoot: Cache.flutterRoot!,
    splitDeferredAssets: _splitDeferredAssets,
  );
}

/// An asset bundle based on a pubspec.yaml file.
class ManifestAssetBundle implements AssetBundle {
  /// Constructs an [ManifestAssetBundle] that gathers the set of assets from the
  /// pubspec.yaml manifest.
  ManifestAssetBundle({
    required Logger logger,
    required FileSystem fileSystem,
    required Platform platform,
    required String flutterRoot,
    bool splitDeferredAssets = false,
  }) : _logger = logger,
       _fileSystem = fileSystem,
       _platform = platform,
       _flutterRoot = flutterRoot,
       _splitDeferredAssets = splitDeferredAssets,
       _licenseCollector = LicenseCollector(fileSystem: fileSystem);

  final Logger _logger;
  final FileSystem _fileSystem;
  final LicenseCollector _licenseCollector;
  final Platform _platform;
  final String _flutterRoot;
  final bool _splitDeferredAssets;

  @override
  final Map<String, AssetBundleEntry> entries = <String, AssetBundleEntry>{};

  @override
  final Map<String, Map<String, AssetBundleEntry>> deferredComponentsEntries =
      <String, Map<String, AssetBundleEntry>>{};

  @override
  final List<File> inputFiles = <File>[];

  // If an asset corresponds to a wildcard directory, then it may have been
  // updated without changes to the manifest. These are only tracked for
  // the current project.
  final Map<Uri, Directory> _wildcardDirectories = <Uri, Directory>{};

  DateTime? _lastBuildTimestamp;

  // We assume the main asset is designed for a device pixel ratio of 1.0.
  static const String _kAssetManifestJsonFilename = 'AssetManifest.json';
  static const String _kAssetManifestBinFilename = 'AssetManifest.bin';
  static const String _kAssetManifestBinJsonFilename = 'AssetManifest.bin.json';

  static const String _kNoticeFile = 'NOTICES';
  // Comically, this can't be name with the more common .gz file extension
  // because when it's part of an AAR and brought into another APK via gradle,
  // gradle individually traverses all the files of the AAR and unzips .gz
  // files (b/37117906). A less common .Z extension still describes how the
  // file is formatted if users want to manually inspect the application
  // bundle and is recognized by default file handlers on OS such as macOS.˚
  static const String _kNoticeZippedFile = 'NOTICES.Z';

  @override
  bool wasBuiltOnce() => _lastBuildTimestamp != null;

  @override
  bool needsBuild({String manifestPath = defaultManifestPath}) {
    final DateTime? lastBuildTimestamp = _lastBuildTimestamp;
    if (lastBuildTimestamp == null) {
      return true;
    }

    final FileStat manifestStat = _fileSystem.file(manifestPath).statSync();
    if (manifestStat.type == FileSystemEntityType.notFound) {
      return true;
    }

    for (final Directory directory in _wildcardDirectories.values) {
      if (!directory.existsSync()) {
        return true; // directory was deleted.
      }
      for (final File file in directory.listSync().whereType<File>()) {
        final DateTime dateTime = file.statSync().modified;
        if (dateTime.isAfter(lastBuildTimestamp)) {
          return true;
        }
      }
    }

    return manifestStat.modified.isAfter(lastBuildTimestamp);
  }

  @override
  Future<int> build({
    String manifestPath = defaultManifestPath,
    FlutterProject? flutterProject,
    required String packageConfigPath,
    bool deferredComponentsEnabled = false,
    TargetPlatform? targetPlatform,
    String? flavor,
  }) async {
    if (flutterProject == null) {
      try {
        flutterProject = FlutterProject.fromDirectory(_fileSystem.file(manifestPath).parent);
      } on Exception catch (e) {
        _logger.printStatus('Error detected in pubspec.yaml:', emphasis: true);
        _logger.printError('$e');
        return 1;
      }
    }

    final FlutterManifest flutterManifest = flutterProject.manifest;
    // If the last build time isn't set before this early return, empty pubspecs will
    // hang on hot reload, as the incremental dill files will never be copied to the
    // device.
    _lastBuildTimestamp = DateTime.now();
    if (flutterManifest.isEmpty) {
      entries[_kAssetManifestJsonFilename] = AssetBundleEntry(
        DevFSStringContent('{}'),
        kind: AssetKind.regular,
        transformers: const <AssetTransformerEntry>[],
      );
      final ByteData emptyAssetManifest =
          const StandardMessageCodec().encodeMessage(<dynamic, dynamic>{})!;
      entries[_kAssetManifestBinFilename] = AssetBundleEntry(
        DevFSByteContent(
          emptyAssetManifest.buffer.asUint8List(0, emptyAssetManifest.lengthInBytes),
        ),
        kind: AssetKind.regular,
        transformers: const <AssetTransformerEntry>[],
      );
      // Create .bin.json on web builds.
      if (targetPlatform == TargetPlatform.web_javascript) {
        entries[_kAssetManifestBinJsonFilename] = AssetBundleEntry(
          DevFSStringContent('""'),
          kind: AssetKind.regular,
          transformers: const <AssetTransformerEntry>[],
        );
      }
      return 0;
    }

    final String assetBasePath = _fileSystem.path.dirname(_fileSystem.path.absolute(manifestPath));
    final File packageConfigFile = _fileSystem.file(packageConfigPath);
    inputFiles.add(packageConfigFile);
    final PackageConfig packageConfig = await loadPackageConfigWithLogging(
      packageConfigFile,
      logger: _logger,
    );
    final List<Uri> wildcardDirectories = <Uri>[];

    // The _assetVariants map contains an entry for each asset listed
    // in the pubspec.yaml file's assets and font sections. The
    // value of each image asset is a list of resolution-specific "variants",
    // see _AssetDirectoryCache.
    final Map<_Asset, List<_Asset>>? assetVariants = _parseAssets(
      packageConfig,
      flutterManifest,
      wildcardDirectories,
      assetBasePath,
      targetPlatform,
      flavor: flavor,
    );

    if (assetVariants == null) {
      return 1;
    }

    // Parse assets for deferred components.
    final Map<String, Map<_Asset, List<_Asset>>> deferredComponentsAssetVariants =
        _parseDeferredComponentsAssets(
          flutterManifest,
          packageConfig,
          assetBasePath,
          wildcardDirectories,
          flutterProject.directory,
          flavor: flavor,
        );
    if (!_splitDeferredAssets || !deferredComponentsEnabled) {
      // Include the assets in the regular set of assets if not using deferred
      // components.
      deferredComponentsAssetVariants.values.forEach(assetVariants.addAll);
      deferredComponentsAssetVariants.clear();
      deferredComponentsEntries.clear();
    }

    final bool includesMaterialFonts = flutterManifest.usesMaterialDesign;
    final List<Map<String, Object?>> fonts = _parseFonts(
      flutterManifest,
      packageConfig,
      primary: true,
    );

    // Add fonts, assets, and licenses from packages.
    final Map<String, List<File>> additionalLicenseFiles = <String, List<File>>{};
    for (final Package package in packageConfig.packages) {
      final Uri packageUri = package.packageUriRoot;
      if (packageUri.scheme == 'file') {
        final String packageManifestPath = _fileSystem.path.fromUri(
          packageUri.resolve('../pubspec.yaml'),
        );
        inputFiles.add(_fileSystem.file(packageManifestPath));
        final FlutterManifest? packageFlutterManifest = FlutterManifest.createFromPath(
          packageManifestPath,
          logger: _logger,
          fileSystem: _fileSystem,
        );
        if (packageFlutterManifest == null) {
          continue;
        }
        // Collect any additional licenses from each package.
        final List<File> licenseFiles = <File>[];
        for (final String relativeLicensePath in packageFlutterManifest.additionalLicenses) {
          final String absoluteLicensePath = _fileSystem.path.fromUri(
            package.root.resolve(relativeLicensePath),
          );
          licenseFiles.add(_fileSystem.file(absoluteLicensePath).absolute);
        }
        additionalLicenseFiles[packageFlutterManifest.appName] = licenseFiles;

        // Skip the app itself
        if (packageFlutterManifest.appName == flutterManifest.appName) {
          continue;
        }
        final String packageBasePath = _fileSystem.path.dirname(packageManifestPath);

        final Map<_Asset, List<_Asset>>? packageAssets = _parseAssets(
          packageConfig,
          packageFlutterManifest,
          // Do not track wildcard directories for dependencies.
          <Uri>[],
          packageBasePath,
          targetPlatform,
          packageName: package.name,
          attributedPackage: package,
          flavor: flavor,
        );

        if (packageAssets == null) {
          return 1;
        }
        assetVariants.addAll(packageAssets);
        if (!includesMaterialFonts && packageFlutterManifest.usesMaterialDesign) {
          _logger.printError(
            'package:${package.name} has `uses-material-design: true` set but '
            'the primary pubspec contains `uses-material-design: false`. '
            'If the application needs material icons, then `uses-material-design` '
            ' must be set to true.',
          );
        }
        fonts.addAll(
          _parseFonts(
            packageFlutterManifest,
            packageConfig,
            packageName: package.name,
            primary: false,
          ),
        );
      }
    }

    // Save the contents of each image, image variant, and font
    // asset in entries.
    for (final _Asset asset in assetVariants.keys) {
      final File assetFile = asset.lookupAssetFile(_fileSystem);
      final List<_Asset> variants = assetVariants[asset]!;
      if (!assetFile.existsSync() && variants.isEmpty) {
        _logger.printStatus('Error detected in pubspec.yaml:', emphasis: true);
        _logger.printError('No file or variants found for $asset.\n');
        if (asset.package != null) {
          _logger.printError('This asset was included from package ${asset.package?.name}.');
        }
        return 1;
      }
      // The file name for an asset's "main" entry is whatever appears in
      // the pubspec.yaml file. The main entry's file must always exist for
      // font assets. It need not exist for an image if resolution-specific
      // variant files exist. An image's main entry is treated the same as a
      // "1x" resolution variant and if both exist then the explicit 1x
      // variant is preferred.
      if (assetFile.existsSync() && !variants.contains(asset)) {
        variants.insert(0, asset);
      }
      for (final _Asset variant in variants) {
        final File variantFile = variant.lookupAssetFile(_fileSystem);
        inputFiles.add(variantFile);
        assert(variantFile.existsSync());
        _setIfConfigurationChanged(
          entries,
          variant.entryUri.path,
          AssetBundleEntry(
            DevFSFileContent(variantFile),
            kind: variant.kind,
            transformers: variant.transformers,
          ),
        );
      }
    }
    // Save the contents of each deferred component image, image variant, and font
    // asset in deferredComponentsEntries.
    for (final String componentName in deferredComponentsAssetVariants.keys) {
      deferredComponentsEntries[componentName] = <String, AssetBundleEntry>{};
      final Map<_Asset, List<_Asset>> assetsMap = deferredComponentsAssetVariants[componentName]!;
      for (final _Asset asset in assetsMap.keys) {
        final File assetFile = asset.lookupAssetFile(_fileSystem);
        if (!assetFile.existsSync() && assetsMap[asset]!.isEmpty) {
          _logger.printStatus('Error detected in pubspec.yaml:', emphasis: true);
          _logger.printError('No file or variants found for $asset.\n');
          if (asset.package != null) {
            _logger.printError('This asset was included from package ${asset.package?.name}.');
          }
          return 1;
        }
        // The file name for an asset's "main" entry is whatever appears in
        // the pubspec.yaml file. The main entry's file must always exist for
        // font assets. It need not exist for an image if resolution-specific
        // variant files exist. An image's main entry is treated the same as a
        // "1x" resolution variant and if both exist then the explicit 1x
        // variant is preferred.
        if (assetFile.existsSync() && !assetsMap[asset]!.contains(asset)) {
          assetsMap[asset]!.insert(0, asset);
        }
        for (final _Asset variant in assetsMap[asset]!) {
          final File variantFile = variant.lookupAssetFile(_fileSystem);
          assert(variantFile.existsSync());
          _setIfConfigurationChanged(
            deferredComponentsEntries[componentName]!,
            variant.entryUri.path,
            AssetBundleEntry(
              DevFSFileContent(variantFile),
              kind: AssetKind.regular,
              transformers: variant.transformers,
            ),
          );
        }
      }
    }
    final List<_Asset> materialAssets = <_Asset>[
      if (flutterManifest.usesMaterialDesign) ..._getMaterialFonts(),
      // For all platforms, include the shaders unconditionally. They are
      // small, and whether they're used is determined only by the app source
      // code and not by the Flutter manifest.
      ..._getMaterialShaders(),
    ];
    for (final _Asset asset in materialAssets) {
      final File assetFile = asset.lookupAssetFile(_fileSystem);
      assert(assetFile.existsSync(), 'Missing ${assetFile.path}');
      entries[asset.entryUri.path] ??= AssetBundleEntry(
        DevFSFileContent(assetFile),
        kind: asset.kind,
        transformers: const <AssetTransformerEntry>[],
      );
    }

    // Update wildcard directories we can detect changes in them.
    for (final Uri uri in wildcardDirectories) {
      _wildcardDirectories[uri] ??= _fileSystem.directory(uri);
    }

    final Map<String, List<String>> assetManifest = _createAssetManifest(
      assetVariants,
      deferredComponentsAssetVariants,
    );
    final DevFSByteContent assetManifestBinary = _createAssetManifestBinary(assetManifest);
    final DevFSStringContent assetManifestJson = DevFSStringContent(json.encode(assetManifest));
    final DevFSStringContent fontManifest = DevFSStringContent(json.encode(fonts));
    final LicenseResult licenseResult = _licenseCollector.obtainLicenses(
      packageConfig,
      additionalLicenseFiles,
    );
    if (licenseResult.errorMessages.isNotEmpty) {
      licenseResult.errorMessages.forEach(_logger.printError);
      return 1;
    }

    additionalDependencies = licenseResult.dependencies;
    inputFiles.addAll(additionalDependencies);

    if (wildcardDirectories.isNotEmpty) {
      // Force the depfile to contain missing files so that Gradle does not skip
      // the task. Wildcard directories are not compatible with full incremental
      // builds. For more context see https://github.com/flutter/flutter/issues/56466 .
      _logger.printTrace(
        'Manifest contained wildcard assets. Inserting missing file into '
        'build graph to force rerun. for more information see #56466.',
      );
      final int suffix = Object().hashCode;
      additionalDependencies.add(
        _fileSystem.file('DOES_NOT_EXIST_RERUN_FOR_WILDCARD$suffix').absolute,
      );
    }

    _setIfChanged(_kAssetManifestJsonFilename, assetManifestJson, AssetKind.regular);
    _setIfChanged(_kAssetManifestBinFilename, assetManifestBinary, AssetKind.regular);
    // Create .bin.json on web builds.
    if (targetPlatform == TargetPlatform.web_javascript) {
      final DevFSStringContent assetManifestBinaryJson = DevFSStringContent(
        json.encode(base64.encode(assetManifestBinary.bytes)),
      );
      _setIfChanged(_kAssetManifestBinJsonFilename, assetManifestBinaryJson, AssetKind.regular);
    }
    _setIfChanged(kFontManifestJson, fontManifest, AssetKind.regular);
    _setLicenseIfChanged(licenseResult.combinedLicenses, targetPlatform);
    return 0;
  }

  @override
  List<File> additionalDependencies = <File>[];
  void _setIfChanged(String key, DevFSContent content, AssetKind assetKind) {
    final DevFSContent? oldContent = entries[key]?.content;
    // In the case that the content is unchanged, we want to avoid an overwrite
    // as the isModified property may be reset to true,
    if (oldContent is DevFSByteContent &&
        content is DevFSByteContent &&
        _compareIntLists(oldContent.bytes, content.bytes)) {
      return;
    }

    entries[key] = AssetBundleEntry(
      content,
      kind: assetKind,
      transformers: const <AssetTransformerEntry>[],
    );
  }

  static bool _compareIntLists(List<int> o1, List<int> o2) {
    if (o1.length != o2.length) {
      return false;
    }

    for (int index = 0; index < o1.length; index++) {
      if (o1[index] != o2[index]) {
        return false;
      }
    }

    return true;
  }

  void _setIfConfigurationChanged(
    Map<String, AssetBundleEntry> entryMap,
    String key,
    AssetBundleEntry entry,
  ) {
    final AssetBundleEntry? existingEntry = entryMap[key];
    if (existingEntry == null || !entry.hasEquivalentConfigurationWith(existingEntry)) {
      entryMap[key] = entry;
    }
  }

  void _setLicenseIfChanged(String combinedLicenses, TargetPlatform? targetPlatform) {
    // On the web, don't compress the NOTICES file since the client doesn't have
    // dart:io to decompress it. So use the standard _setIfChanged to check if
    // the strings still match.
    if (targetPlatform == TargetPlatform.web_javascript) {
      _setIfChanged(_kNoticeFile, DevFSStringContent(combinedLicenses), AssetKind.regular);
      return;
    }

    // On other platforms, let the NOTICES file be compressed. But use a
    // specialized DevFSStringCompressingBytesContent class to compare
    // the uncompressed strings to not incur decompression/decoding while making
    // the comparison.
    if (!entries.containsKey(_kNoticeZippedFile) ||
        (entries[_kNoticeZippedFile]?.content as DevFSStringCompressingBytesContent?)?.equals(
              combinedLicenses,
            ) !=
            true) {
      entries[_kNoticeZippedFile] = AssetBundleEntry(
        DevFSStringCompressingBytesContent(
          combinedLicenses,
          // A zlib dictionary is a hinting string sequence with the most
          // likely string occurrences at the end. This ends up just being
          // common English words with domain specific words like copyright.
          hintString: 'copyrightsoftwaretothisinandorofthe',
        ),
        kind: AssetKind.regular,
        transformers: const <AssetTransformerEntry>[],
      );
    }
  }

  List<_Asset> _getMaterialFonts() {
    final List<_Asset> result = <_Asset>[];
    for (final Map<String, Object> family in kMaterialFonts) {
      final Object? fonts = family['fonts'];
      if (fonts == null) {
        continue;
      }
      for (final Map<String, Object> font in fonts as List<Map<String, String>>) {
        final String? asset = font['asset'] as String?;
        if (asset == null) {
          continue;
        }
        final Uri entryUri = _fileSystem.path.toUri(asset);
        result.add(
          _Asset(
            baseDir: _fileSystem.path.join(
              _flutterRoot,
              'bin',
              'cache',
              'artifacts',
              'material_fonts',
            ),
            relativeUri: Uri(path: entryUri.pathSegments.last),
            entryUri: entryUri,
            package: null,
            kind: AssetKind.font,
          ),
        );
      }
    }

    return result;
  }

  List<_Asset> _getMaterialShaders() {
    final String shaderPath = _fileSystem.path.join(
      _flutterRoot,
      'packages',
      'flutter',
      'lib',
      'src',
      'material',
      'shaders',
    );
    // This file will exist in a real invocation unless the git checkout is
    // corrupted somehow, but unit tests generally don't create this file
    // in their mock file systems. Leaving it out in those cases is harmless.
    if (!_fileSystem.directory(shaderPath).existsSync()) {
      return <_Asset>[];
    }

    final List<_Asset> result = <_Asset>[];
    for (final String shader in kMaterialShaders) {
      final Uri entryUri = _fileSystem.path.toUri(shader);
      result.add(
        _Asset(
          baseDir: shaderPath,
          relativeUri: Uri(path: entryUri.pathSegments.last),
          entryUri: entryUri,
          package: null,
          kind: AssetKind.shader,
        ),
      );
    }

    return result;
  }

  List<Map<String, Object?>> _parseFonts(
    FlutterManifest manifest,
    PackageConfig packageConfig, {
    String? packageName,
    required bool primary,
  }) {
    return <Map<String, Object?>>[
      if (primary && manifest.usesMaterialDesign) ...kMaterialFonts,
      if (packageName == null)
        ...manifest.fontsDescriptor
      else
        for (final Font font in _parsePackageFonts(manifest, packageName, packageConfig))
          font.descriptor,
    ];
  }

  Map<String, Map<_Asset, List<_Asset>>> _parseDeferredComponentsAssets(
    FlutterManifest flutterManifest,
    PackageConfig packageConfig,
    String assetBasePath,
    List<Uri> wildcardDirectories,
    Directory projectDirectory, {
    String? flavor,
  }) {
    final List<DeferredComponent>? components = flutterManifest.deferredComponents;
    final Map<String, Map<_Asset, List<_Asset>>> deferredComponentsAssetVariants =
        <String, Map<_Asset, List<_Asset>>>{};
    if (components == null) {
      return deferredComponentsAssetVariants;
    }
    for (final DeferredComponent component in components) {
      final _AssetDirectoryCache cache = _AssetDirectoryCache(_fileSystem);
      final Map<_Asset, List<_Asset>> componentAssets = <_Asset, List<_Asset>>{};
      for (final AssetsEntry assetsEntry in component.assets) {
        if (assetsEntry.uri.path.endsWith('/')) {
          wildcardDirectories.add(assetsEntry.uri);
          _parseAssetsFromFolder(
            packageConfig,
            flutterManifest,
            assetBasePath,
            cache,
            componentAssets,
            assetsEntry.uri,
            flavors: assetsEntry.flavors,
            transformers: assetsEntry.transformers,
          );
        } else {
          _parseAssetFromFile(
            packageConfig,
            flutterManifest,
            assetBasePath,
            cache,
            componentAssets,
            assetsEntry.uri,
            flavors: assetsEntry.flavors,
            transformers: assetsEntry.transformers,
          );
        }
      }

      componentAssets.removeWhere(
        (_Asset asset, List<_Asset> variants) => !asset.matchesFlavor(flavor),
      );
      deferredComponentsAssetVariants[component.name] = componentAssets;
    }
    return deferredComponentsAssetVariants;
  }

  Map<String, List<String>> _createAssetManifest(
    Map<_Asset, List<_Asset>> assetVariants,
    Map<String, Map<_Asset, List<_Asset>>> deferredComponentsAssetVariants,
  ) {
    final Map<String, List<String>> manifest = <String, List<String>>{};
    final Map<_Asset, List<String>> entries = <_Asset, List<String>>{};
    assetVariants.forEach((_Asset main, List<_Asset> variants) {
      entries[main] = <String>[for (final _Asset variant in variants) variant.entryUri.path];
    });
    for (final Map<_Asset, List<_Asset>> componentAssets
        in deferredComponentsAssetVariants.values) {
      componentAssets.forEach((_Asset main, List<_Asset> variants) {
        entries[main] = <String>[for (final _Asset variant in variants) variant.entryUri.path];
      });
    }
    final List<_Asset> sortedKeys =
        entries.keys.toList()
          ..sort((_Asset left, _Asset right) => left.entryUri.path.compareTo(right.entryUri.path));
    for (final _Asset main in sortedKeys) {
      final String decodedEntryPath = Uri.decodeFull(main.entryUri.path);
      final List<String> rawEntryVariantsPaths = entries[main]!;
      final List<String> decodedEntryVariantPaths =
          rawEntryVariantsPaths.map((String value) => Uri.decodeFull(value)).toList();
      manifest[decodedEntryPath] = decodedEntryVariantPaths;
    }
    return manifest;
  }

  // Matches path-like strings ending in a number followed by an 'x'.
  // Example matches include "assets/animals/2.0x", "plants/3x", and "2.7x".
  static final RegExp _extractPixelRatioFromKeyRegExp = RegExp(r'/?(\d+(\.\d*)?)x$');

  DevFSByteContent _createAssetManifestBinary(Map<String, List<String>> assetManifest) {
    double? parseScale(String key) {
      final Uri assetUri = Uri.parse(key);
      String directoryPath = '';
      if (assetUri.pathSegments.length > 1) {
        directoryPath = assetUri.pathSegments[assetUri.pathSegments.length - 2];
      }

      final Match? match = _extractPixelRatioFromKeyRegExp.firstMatch(directoryPath);
      if (match != null && match.groupCount > 0) {
        return double.parse(match.group(1)!);
      }

      return null;
    }

    final Map<String, dynamic> result = <String, dynamic>{};

    for (final MapEntry<String, dynamic> manifestEntry in assetManifest.entries) {
      final List<dynamic> resultVariants = <dynamic>[];
      final List<String> entries = (manifestEntry.value as List<dynamic>).cast<String>();
      for (final String variant in entries) {
        final Map<String, dynamic> resultVariant = <String, dynamic>{};
        final double? variantDevicePixelRatio = parseScale(variant);
        resultVariant['asset'] = variant;
        if (variantDevicePixelRatio != null) {
          resultVariant['dpr'] = variantDevicePixelRatio;
        }
        resultVariants.add(resultVariant);
      }
      result[manifestEntry.key] = resultVariants;
    }

    final ByteData message = const StandardMessageCodec().encodeMessage(result)!;
    return DevFSByteContent(message.buffer.asUint8List(0, message.lengthInBytes));
  }

  /// Prefixes family names and asset paths of fonts included from packages with
  /// 'packages/<package_name>'
  List<Font> _parsePackageFonts(
    FlutterManifest manifest,
    String packageName,
    PackageConfig packageConfig,
  ) {
    final List<Font> packageFonts = <Font>[];
    for (final Font font in manifest.fonts) {
      final List<FontAsset> packageFontAssets = <FontAsset>[];
      for (final FontAsset fontAsset in font.fontAssets) {
        final Uri assetUri = fontAsset.assetUri;
        if (assetUri.pathSegments.first == 'packages' &&
            !_fileSystem.isFileSync(
              _fileSystem.path.fromUri(
                packageConfig[packageName]?.packageUriRoot.resolve('../${assetUri.path}'),
              ),
            )) {
          packageFontAssets.add(
            FontAsset(fontAsset.assetUri, weight: fontAsset.weight, style: fontAsset.style),
          );
        } else {
          packageFontAssets.add(
            FontAsset(
              Uri(pathSegments: <String>['packages', packageName, ...assetUri.pathSegments]),
              weight: fontAsset.weight,
              style: fontAsset.style,
            ),
          );
        }
      }
      packageFonts.add(Font('packages/$packageName/${font.familyName}', packageFontAssets));
    }
    return packageFonts;
  }

  /// Given an assetBase location and a pubspec.yaml Flutter manifest, return a
  /// map of assets to asset variants.
  ///
  /// Returns null on missing assets.
  ///
  /// Given package: 'test_package' and an assets directory like this:
  ///
  /// - assets/foo
  /// - assets/var1/foo
  /// - assets/var2/foo
  /// - assets/bar
  ///
  /// This will return:
  /// ```none
  /// {
  ///   asset: packages/test_package/assets/foo: [
  ///     asset: packages/test_package/assets/foo,
  ///     asset: packages/test_package/assets/var1/foo,
  ///     asset: packages/test_package/assets/var2/foo,
  ///   ],
  ///   asset: packages/test_package/assets/bar: [
  ///     asset: packages/test_package/assets/bar,
  ///   ],
  /// }
  /// ```
  Map<_Asset, List<_Asset>>? _parseAssets(
    PackageConfig packageConfig,
    FlutterManifest flutterManifest,
    List<Uri> wildcardDirectories,
    String assetBase,
    TargetPlatform? targetPlatform, {
    String? packageName,
    Package? attributedPackage,
    required String? flavor,
  }) {
    final Map<_Asset, List<_Asset>> result = <_Asset, List<_Asset>>{};

    final _AssetDirectoryCache cache = _AssetDirectoryCache(_fileSystem);
    for (final AssetsEntry assetsEntry in flutterManifest.assets) {
      if (assetsEntry.uri.path.endsWith('/')) {
        wildcardDirectories.add(assetsEntry.uri);
        _parseAssetsFromFolder(
          packageConfig,
          flutterManifest,
          assetBase,
          cache,
          result,
          assetsEntry.uri,
          packageName: packageName,
          attributedPackage: attributedPackage,
          flavors: assetsEntry.flavors,
          transformers: assetsEntry.transformers,
        );
      } else {
        _parseAssetFromFile(
          packageConfig,
          flutterManifest,
          assetBase,
          cache,
          result,
          assetsEntry.uri,
          packageName: packageName,
          attributedPackage: attributedPackage,
          flavors: assetsEntry.flavors,
          transformers: assetsEntry.transformers,
        );
      }
    }

    result.removeWhere((_Asset asset, List<_Asset> variants) {
      if (!asset.matchesFlavor(flavor)) {
        _logger.printTrace(
          'Skipping assets entry "${asset.entryUri.path}" since '
          'its configured flavor(s) did not match the provided flavor (if any).\n'
          'Configured flavors: ${asset.flavors.join(', ')}\n',
        );
        return true;
      }
      return false;
    });

    for (final Uri shaderUri in flutterManifest.shaders) {
      _parseAssetFromFile(
        packageConfig,
        flutterManifest,
        assetBase,
        cache,
        result,
        shaderUri,
        packageName: packageName,
        attributedPackage: attributedPackage,
        assetKind: AssetKind.shader,
        flavors: <String>{},
        transformers: <AssetTransformerEntry>[],
      );
    }

    for (final Uri modelUri in flutterManifest.models) {
      _parseAssetFromFile(
        packageConfig,
        flutterManifest,
        assetBase,
        cache,
        result,
        modelUri,
        packageName: packageName,
        attributedPackage: attributedPackage,
        assetKind: AssetKind.model,
        flavors: <String>{},
        transformers: <AssetTransformerEntry>[],
      );
    }

    // Add assets referenced in the fonts section of the manifest.
    for (final Font font in flutterManifest.fonts) {
      for (final FontAsset fontAsset in font.fontAssets) {
        final _Asset baseAsset = _resolveAsset(
          packageConfig,
          assetBase,
          fontAsset.assetUri,
          packageName,
          attributedPackage,
          assetKind: AssetKind.font,
          flavors: <String>{},
          transformers: <AssetTransformerEntry>[],
        );
        final File baseAssetFile = baseAsset.lookupAssetFile(_fileSystem);
        if (!baseAssetFile.existsSync()) {
          _logger.printError(
            'Error: unable to locate asset entry in pubspec.yaml: "${fontAsset.assetUri}".',
          );
          return null;
        }
        result[baseAsset] = <_Asset>[];
      }
    }

    return result;
  }

  void _parseAssetsFromFolder(
    PackageConfig packageConfig,
    FlutterManifest flutterManifest,
    String assetBase,
    _AssetDirectoryCache cache,
    Map<_Asset, List<_Asset>> result,
    Uri assetUri, {
    String? packageName,
    Package? attributedPackage,
    required Set<String> flavors,
    required List<AssetTransformerEntry> transformers,
  }) {
    final String directoryPath;
    try {
      directoryPath = _fileSystem.path.join(
        assetBase,
        assetUri.toFilePath(windows: _platform.isWindows),
      );
    } on UnsupportedError catch (e) {
      throwToolExit(
        'Unable to search for asset files in directory path "${assetUri.path}". '
        'Please ensure that this entry in pubspec.yaml is a valid file path.\n'
        'Error details:\n$e',
      );
    }

    if (!_fileSystem.directory(directoryPath).existsSync()) {
      _logger.printError('Error: unable to find directory entry in pubspec.yaml: $directoryPath');
      return;
    }

    final Iterable<FileSystemEntity> entities = _fileSystem.directory(directoryPath).listSync();

    final Iterable<File> files = entities.whereType<File>();
    for (final File file in files) {
      final String relativePath = _fileSystem.path.relative(file.path, from: assetBase);
      final Uri uri = Uri.file(relativePath, windows: _platform.isWindows);

      _parseAssetFromFile(
        packageConfig,
        flutterManifest,
        assetBase,
        cache,
        result,
        uri,
        packageName: packageName,
        attributedPackage: attributedPackage,
        originUri: assetUri,
        flavors: flavors,
        transformers: transformers,
      );
    }
  }

  void _parseAssetFromFile(
    PackageConfig packageConfig,
    FlutterManifest flutterManifest,
    String assetBase,
    _AssetDirectoryCache cache,
    Map<_Asset, List<_Asset>> result,
    Uri assetUri, {
    Uri? originUri,
    String? packageName,
    Package? attributedPackage,
    AssetKind assetKind = AssetKind.regular,
    required Set<String> flavors,
    required List<AssetTransformerEntry> transformers,
  }) {
    final _Asset asset = _resolveAsset(
      packageConfig,
      assetBase,
      assetUri,
      packageName,
      attributedPackage,
      assetKind: assetKind,
      originUri: originUri,
      flavors: flavors,
      transformers: transformers,
    );

    _checkForFlavorConflicts(asset, result.keys.toList());

    final List<_Asset> variants = <_Asset>[];
    final File assetFile = asset.lookupAssetFile(_fileSystem);

    for (final String path in cache.variantsFor(assetFile.path)) {
      final String relativePath = _fileSystem.path.relative(path, from: asset.baseDir);
      final Uri relativeUri = _fileSystem.path.toUri(relativePath);
      final Uri? entryUri =
          asset.symbolicPrefixUri == null
              ? relativeUri
              : asset.symbolicPrefixUri?.resolveUri(relativeUri);
      if (entryUri != null) {
        variants.add(
          _Asset(
            baseDir: asset.baseDir,
            entryUri: entryUri,
            relativeUri: relativeUri,
            package: attributedPackage,
            kind: assetKind,
            flavors: flavors,
            transformers: transformers,
          ),
        );
      }
    }

    result[asset] = variants;
  }

  // Since it is not clear how overlapping asset declarations should work in the
  // presence of conditions such as `flavor`, we throw an Error.
  //
  // To be more specific, it is not clear if conditions should be combined with
  // or-logic or and-logic, or if it should depend on the specificity of the
  // declarations (file versus directory). If you would like examples, consider these:
  //
  // ```yaml
  // # Should assets/free.mp3 always be included since "assets/" has no flavor?
  // assets:
  //   - assets/
  //   - path: assets/free.mp3
  //     flavor: free
  //
  // # Should "assets/paid/pip.mp3" be included for both the "paid" and "free" flavors?
  // # Or, since "assets/paid/pip.mp3" is more specific than "assets/paid/"", should
  // # it take precedence over the latter (included only in "free" flavor)?
  // assets:
  //   - path: assets/paid/
  //     flavor: paid
  //   - path: assets/paid/pip.mp3
  //     flavor: free
  //   - asset
  // ```
  //
  // Since it is not obvious what logic (if any) would be intuitive and preferable
  // to the vast majority of users (if any), we play it safe by throwing a `ToolExit`
  // in any of these situations. We can always loosen up this restriction later
  // without breaking anyone.
  void _checkForFlavorConflicts(_Asset newAsset, List<_Asset> previouslyParsedAssets) {
    bool cameFromDirectoryEntry(_Asset asset) {
      return asset.originUri.path.endsWith('/');
    }

    String flavorErrorInfo(_Asset asset) {
      if (asset.flavors.isEmpty) {
        return 'An entry with the path "${asset.originUri}" does not specify any flavors.';
      }

      final Iterable<String> flavorsWrappedWithQuotes = asset.flavors.map((String e) => '"$e"');
      return 'An entry with the path "${asset.originUri}" specifies the flavor(s): '
          '${flavorsWrappedWithQuotes.join(', ')}.';
    }

    final _Asset? preExistingAsset =
        previouslyParsedAssets
            .where((_Asset other) => other.entryUri == newAsset.entryUri)
            .firstOrNull;

    if (preExistingAsset == null || preExistingAsset.hasEquivalentFlavorsWith(newAsset)) {
      return;
    }

    final StringBuffer errorMessage = StringBuffer(
      'Multiple assets entries include the file '
      '"${newAsset.entryUri.path}", but they specify different lists of flavors.\n',
    );

    errorMessage.writeln(flavorErrorInfo(preExistingAsset));
    errorMessage.writeln(flavorErrorInfo(newAsset));

    if (cameFromDirectoryEntry(newAsset) || cameFromDirectoryEntry(preExistingAsset)) {
      errorMessage.writeln();
      errorMessage.write(
        'Consider organizing assets with different flavors '
        'into different directories.',
      );
    }

    throwToolExit(errorMessage.toString());
  }

  _Asset _resolveAsset(
    PackageConfig packageConfig,
    String assetsBaseDir,
    Uri assetUri,
    String? packageName,
    Package? attributedPackage, {
    Uri? originUri,
    AssetKind assetKind = AssetKind.regular,
    required Set<String> flavors,
    required List<AssetTransformerEntry> transformers,
  }) {
    final String assetPath = _fileSystem.path.fromUri(assetUri);
    if (assetUri.pathSegments.first == 'packages' &&
        !_fileSystem.isFileSync(_fileSystem.path.join(assetsBaseDir, assetPath))) {
      // The asset is referenced in the pubspec.yaml as
      // 'packages/PACKAGE_NAME/PATH/TO/ASSET .
      final _Asset? packageAsset = _resolvePackageAsset(
        assetUri,
        packageConfig,
        attributedPackage,
        assetKind: assetKind,
        originUri: originUri,
        flavors: flavors,
        transformers: transformers,
      );
      if (packageAsset != null) {
        return packageAsset;
      }
    }

    return _Asset(
      baseDir: assetsBaseDir,
      entryUri:
          packageName == null
              ? assetUri // Asset from the current application.
              : Uri(
                pathSegments: <String>['packages', packageName, ...assetUri.pathSegments],
              ), // Asset from, and declared in $packageName.
      relativeUri: assetUri,
      package: attributedPackage,
      originUri: originUri,
      kind: assetKind,
      flavors: flavors,
      transformers: transformers,
    );
  }

  _Asset? _resolvePackageAsset(
    Uri assetUri,
    PackageConfig packageConfig,
    Package? attributedPackage, {
    AssetKind assetKind = AssetKind.regular,
    Uri? originUri,
    Set<String>? flavors,
    List<AssetTransformerEntry>? transformers,
  }) {
    assert(assetUri.pathSegments.first == 'packages');
    if (assetUri.pathSegments.length > 1) {
      final String packageName = assetUri.pathSegments[1];
      final Package? package = packageConfig[packageName];
      final Uri? packageUri = package?.packageUriRoot;
      if (packageUri != null && packageUri.scheme == 'file') {
        return _Asset(
          baseDir: _fileSystem.path.fromUri(packageUri),
          entryUri: assetUri,
          relativeUri: Uri(pathSegments: assetUri.pathSegments.sublist(2)),
          package: attributedPackage,
          kind: assetKind,
          originUri: originUri,
          flavors: flavors,
          transformers: transformers,
        );
      }
    }
    _logger.printStatus('Error detected in pubspec.yaml:', emphasis: true);
    _logger.printError('Could not resolve package for asset $assetUri.\n');
    if (attributedPackage != null) {
      _logger.printError('This asset was included from package ${attributedPackage.name}');
    }
    return null;
  }
}

@immutable
class _Asset {
  const _Asset({
    required this.baseDir,
    Uri? originUri,
    required this.relativeUri,
    required this.entryUri,
    required this.package,
    this.kind = AssetKind.regular,
    Set<String>? flavors,
    List<AssetTransformerEntry>? transformers,
  }) : originUri = originUri ?? entryUri,
       flavors = flavors ?? const <String>{},
       transformers = transformers ?? const <AssetTransformerEntry>[];

  final String baseDir;

  final Package? package;

  /// The platform-independent URL provided by the user in the pubspec that this
  /// asset was found from.
  final Uri originUri;

  /// A platform-independent URL where this asset can be found on disk on the
  /// host system relative to [baseDir].
  final Uri relativeUri;

  /// A platform-independent URL representing the entry for the asset manifest.
  final Uri entryUri;

  final AssetKind kind;

  final Set<String> flavors;

  final List<AssetTransformerEntry> transformers;

  File lookupAssetFile(FileSystem fileSystem) {
    return fileSystem.file(fileSystem.path.join(baseDir, fileSystem.path.fromUri(relativeUri)));
  }

  /// The delta between what the entryUri is and the relativeUri (e.g.,
  /// packages/flutter_gallery).
  Uri? get symbolicPrefixUri {
    if (entryUri == relativeUri) {
      return null;
    }
    final int index = entryUri.path.indexOf(relativeUri.path);
    return index == -1 ? null : Uri(path: entryUri.path.substring(0, index));
  }

  bool matchesFlavor(String? flavor) {
    if (flavors.isEmpty) {
      return true;
    }

    if (flavor == null) {
      return false;
    }

    return flavors.contains(flavor);
  }

  bool hasEquivalentFlavorsWith(_Asset other) {
    final Set<String> assetFlavors = flavors.toSet();
    final Set<String> otherFlavors = other.flavors.toSet();
    return assetFlavors.length == otherFlavors.length &&
        assetFlavors.every((String e) => otherFlavors.contains(e));
  }

  @override
  String toString() => 'asset: $entryUri';

  @override
  bool operator ==(Object other) {
    if (identical(other, this)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is _Asset &&
        other.baseDir == baseDir &&
        other.relativeUri == relativeUri &&
        other.entryUri == entryUri &&
        other.kind == kind &&
        hasEquivalentFlavorsWith(other);
  }

  @override
  int get hashCode => Object.hashAll(<Object>[baseDir, relativeUri, entryUri, kind, ...flavors]);
}

// Given an assets directory like this:
//
// assets/foo.png
// assets/2x/foo.png
// assets/3.0x/foo.png
// assets/bar/foo.png
// assets/bar.png
//
// variantsFor('assets/foo.png') => ['/assets/foo.png', '/assets/2x/foo.png', 'assets/3.0x/foo.png']
// variantsFor('assets/bar.png') => ['/assets/bar.png']
// variantsFor('assets/bar/foo.png') => ['/assets/bar/foo.png']
class _AssetDirectoryCache {
  _AssetDirectoryCache(this._fileSystem);

  final FileSystem _fileSystem;
  final Map<String, List<String>> _cache = <String, List<String>>{};
  final Map<String, List<File>> _variantsPerFolder = <String, List<File>>{};

  List<String> variantsFor(String assetPath) {
    final String directoryName = _fileSystem.path.dirname(assetPath);

    try {
      if (!_fileSystem.directory(directoryName).existsSync()) {
        return const <String>[];
      }
    } on FileSystemException catch (e) {
      throwToolExit(
        'Unable to check the existence of asset file "$assetPath". '
        'Ensure that the asset file is declared as a valid local file system path.\n'
        'Details: $e',
      );
    }

    if (_cache.containsKey(assetPath)) {
      return _cache[assetPath]!;
    }
    if (!_variantsPerFolder.containsKey(directoryName)) {
      _variantsPerFolder[directoryName] =
          _fileSystem
              .directory(directoryName)
              .listSync()
              .whereType<Directory>()
              .where((Directory dir) => _assetVariantDirectoryRegExp.hasMatch(dir.basename))
              .expand((Directory dir) => dir.listSync())
              .whereType<File>()
              .toList();
    }
    final File assetFile = _fileSystem.file(assetPath);
    final List<File> potentialVariants = _variantsPerFolder[directoryName]!;
    final String basename = assetFile.basename;
    return _cache[assetPath] = <String>[
      // It's possible that the user specifies only explicit variants (e.g. .../1x/asset.png),
      // so there does not necessarily need to be a file at the given path.
      if (assetFile.existsSync()) assetPath,
      ...potentialVariants
          .where((File file) => file.basename == basename)
          .map((File file) => file.path),
    ];
  }
}
