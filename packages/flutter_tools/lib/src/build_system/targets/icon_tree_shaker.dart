// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:mime/mime.dart' as mime;
import 'package:process/process.dart';

import '../../artifacts.dart';
import '../../base/common.dart';
import '../../base/file_system.dart';
import '../../base/io.dart';
import '../../base/logger.dart';
import '../../base/process.dart';
import '../../build_info.dart';
import '../../convert.dart';
import '../../devfs.dart';
import '../build_system.dart';

List<Map<String, Object?>> _getList(Object? object, String errorMessage) {
  if (object is List<Object?>) {
    return object.cast<Map<String, Object?>>();
  }
  throw IconTreeShakerException._(errorMessage);
}

/// A class that wraps the functionality of the const finder package and the
/// font subset utility to tree shake unused icons from fonts.
class IconTreeShaker {
  /// Creates a wrapper for icon font subsetting.
  ///
  /// If the `fontManifest` parameter is null, [enabled] will return false since
  /// there are no fonts to shake.
  ///
  /// The constructor will validate the environment and print a warning if
  /// font subsetting has been requested in a debug build mode.
  IconTreeShaker(
    this._environment,
    DevFSStringContent? fontManifest, {
    required ProcessManager processManager,
    required Logger logger,
    required FileSystem fileSystem,
    required Artifacts artifacts,
    required TargetPlatform targetPlatform,
  }) : _processManager = processManager,
       _logger = logger,
       _fs = fileSystem,
       _artifacts = artifacts,
       _fontManifest = fontManifest?.string,
       _targetPlatform = targetPlatform {
    if (_environment.defines[kIconTreeShakerFlag] == 'true' &&
        _environment.defines[kBuildMode] == 'debug') {
      logger.printError(
        'Font subsetting is not supported in debug mode. The '
        '--tree-shake-icons flag will be ignored.',
      );
    }
  }

  /// The MIME types for supported font sets.
  static const Set<String> kTtfMimeTypes = <String>{
    'font/ttf', // based on internet search
    'font/opentype',
    'font/otf',
    'application/x-font-opentype',
    'application/x-font-otf',
    'application/x-font-ttf', // based on running locally.
  };

  /// The [Source] inputs that targets using this should depend on.
  ///
  /// See [Target.inputs].
  static const List<Source> inputs = <Source>[
    Source.pattern(
      '{FLUTTER_ROOT}/packages/flutter_tools/lib/src/build_system/targets/icon_tree_shaker.dart',
    ),
    Source.artifact(Artifact.constFinder),
    Source.artifact(Artifact.fontSubset),
  ];

  final Environment _environment;
  final String? _fontManifest;
  Future<void>? _iconDataProcessing;
  Map<String, _IconTreeShakerData>? _iconData;

  final ProcessManager _processManager;
  final Logger _logger;
  final FileSystem _fs;
  final Artifacts _artifacts;
  final TargetPlatform _targetPlatform;

  /// Whether font subsetting should be used for this [Environment].
  bool get enabled =>
      _fontManifest != null &&
      _environment.defines[kIconTreeShakerFlag] == 'true' &&
      _environment.defines[kBuildMode] != 'debug';

  // Fills the [_iconData] map.
  Future<void> _getIconData(Environment environment) async {
    if (!enabled) {
      return;
    }

    final File appDill = environment.buildDir.childFile('app.dill');
    if (!appDill.existsSync()) {
      throw IconTreeShakerException._(
        'Expected to find kernel file at ${appDill.path}, but no file found.',
      );
    }
    final File constFinder = _fs.file(_artifacts.getArtifactPath(Artifact.constFinder));
    final File dart = _fs.file(_artifacts.getArtifactPath(Artifact.engineDartBinary));

    final Map<String, List<int>> iconData = await _findConstants(dart, constFinder, appDill);
    final Set<String> familyKeys = iconData.keys.toSet();

    final Map<String, String> fonts = await _parseFontJson(
      _fontManifest!, // Guarded by `enabled`.
      familyKeys,
    );

    if (fonts.length != iconData.length) {
      environment.logger.printStatus(
        'Expected to find fonts for ${iconData.keys}, but found '
        '${fonts.keys}. This usually means you are referring to '
        'font families in an IconData class but not including them '
        'in the assets section of your pubspec.yaml, are missing '
        'the package that would include them, or are missing '
        '"uses-material-design: true".',
      );
    }

    final Map<String, _IconTreeShakerData> result = <String, _IconTreeShakerData>{};
    const int kSpacePoint = 32;
    for (final MapEntry<String, String> entry in fonts.entries) {
      final List<int>? codePoints = iconData[entry.key];
      if (codePoints == null) {
        throw IconTreeShakerException._(
          'Expected to font code points for ${entry.key}, but none were found.',
        );
      }

      // Add space as an optional code point, as web uses it to measure the font height.
      final List<int> optionalCodePoints =
          _targetPlatform == TargetPlatform.web_javascript ? <int>[kSpacePoint] : <int>[];
      result[entry.value] = _IconTreeShakerData(
        family: entry.key,
        relativePath: entry.value,
        codePoints: codePoints,
        optionalCodePoints: optionalCodePoints,
      );
    }
    _iconData = result;
  }

  /// Calls font-subset, which transforms the [input] font file to a
  /// subsetted version at [outputPath].
  ///
  /// If [enabled] is false, or the relative path is not recognized as an icon
  /// font used in the Flutter application, this returns false.
  /// If the font-subset subprocess fails, it will [throwToolExit].
  /// Otherwise, it will return true.
  Future<bool> subsetFont({
    required File input,
    required String outputPath,
    required String relativePath,
  }) async {
    if (!enabled) {
      return false;
    }
    if (input.lengthSync() < 12) {
      return false;
    }
    final String? mimeType = mime.lookupMimeType(
      input.path,
      headerBytes: await input.openRead(0, 12).first,
    );
    if (!kTtfMimeTypes.contains(mimeType)) {
      return false;
    }
    await (_iconDataProcessing ??= _getIconData(_environment));
    assert(_iconData != null);

    final _IconTreeShakerData? iconTreeShakerData = _iconData![relativePath];
    if (iconTreeShakerData == null) {
      return false;
    }

    final File fontSubset = _fs.file(_artifacts.getArtifactPath(Artifact.fontSubset));
    if (!fontSubset.existsSync()) {
      throw IconTreeShakerException._('The font-subset utility is missing. Run "flutter doctor".');
    }

    final List<String> cmd = <String>[fontSubset.path, outputPath, input.path];
    final Iterable<String> requiredCodePointStrings = iconTreeShakerData.codePoints.map(
      (int codePoint) => codePoint.toString(),
    );
    final Iterable<String> optionalCodePointStrings = iconTreeShakerData.optionalCodePoints.map(
      (int codePoint) => 'optional:$codePoint',
    );
    final String codePointsString = requiredCodePointStrings
        .followedBy(optionalCodePointStrings)
        .join(' ');
    _logger.printTrace(
      'Running font-subset: ${cmd.join(' ')}, '
      'using codepoints $codePointsString',
    );
    final Process fontSubsetProcess = await _processManager.start(cmd);
    try {
      await ProcessUtils.writelnToStdinUnsafe(
        stdin: fontSubsetProcess.stdin,
        line: codePointsString,
      );
      await fontSubsetProcess.stdin.flush();
      await fontSubsetProcess.stdin.close();
    } on Exception {
      // handled by checking the exit code.
    }

    final int code = await fontSubsetProcess.exitCode;
    if (code != 0) {
      _logger.printTrace(await utf8.decodeStream(fontSubsetProcess.stdout));
      _logger.printError(await utf8.decodeStream(fontSubsetProcess.stderr));
      throw IconTreeShakerException._('Font subsetting failed with exit code $code.');
    }
    _logger.printStatus(getSubsetSummaryMessage(input, _fs.file(outputPath)));
    return true;
  }

  @visibleForTesting
  String getSubsetSummaryMessage(File inputFont, File outputFont) {
    final String fontName = inputFont.basename;
    final double inputSize = inputFont.lengthSync().toDouble();
    final double outputSize = outputFont.lengthSync().toDouble();
    final double reductionBytes = inputSize - outputSize;
    final String reductionPercentage = (reductionBytes / inputSize * 100).toStringAsFixed(1);
    return 'Font asset "$fontName" was tree-shaken, reducing it from '
        '${inputSize.ceil()} to ${outputSize.ceil()} bytes '
        '($reductionPercentage% reduction). Tree-shaking can be disabled '
        'by providing the --no-tree-shake-icons flag when building your app.';
  }

  /// Returns a map of { fontFamily: relativePath } pairs.
  Future<Map<String, String>> _parseFontJson(String fontManifestData, Set<String> families) async {
    final Map<String, String> result = <String, String>{};
    final List<Map<String, Object?>> fontList = _getList(
      json.decode(fontManifestData),
      'FontManifest.json invalid: expected top level to be a list of objects.',
    );

    for (final Map<String, Object?> map in fontList) {
      final Object? familyKey = map['family'];
      if (familyKey is! String) {
        throw IconTreeShakerException._(
          'FontManifest.json invalid: expected the family value to be a string, '
          'got: ${map['family']}.',
        );
      }
      if (!families.contains(familyKey)) {
        continue;
      }
      final List<Map<String, Object?>> fonts = _getList(
        map['fonts'],
        'FontManifest.json invalid: expected "fonts" to be a list of objects.',
      );
      if (fonts.length != 1) {
        throw IconTreeShakerException._(
          'This tool cannot process icon fonts with multiple fonts in a '
          'single family.',
        );
      }
      final Object? asset = fonts.first['asset'];
      if (asset is! String) {
        throw IconTreeShakerException._(
          'FontManifest.json invalid: expected "asset" value to be a string, '
          'got: ${map['assets']}.',
        );
      }
      result[familyKey] = asset;
    }
    return result;
  }

  Future<Map<String, List<int>>> _findConstants(File dart, File constFinder, File appDill) async {
    final List<String> cmd = <String>[
      dart.path,
      constFinder.path,
      '--kernel-file',
      appDill.path,
      '--class-library-uri',
      'package:flutter/src/widgets/icon_data.dart',
      '--class-name',
      'IconData',
      '--annotation-class-name',
      '_StaticIconProvider',
      '--annotation-class-library-uri',
      'package:flutter/src/widgets/icon_data.dart',
    ];
    _logger.printTrace('Running command: ${cmd.join(' ')}');
    final ProcessResult constFinderProcessResult = await _processManager.run(cmd);

    if (constFinderProcessResult.exitCode != 0) {
      throw IconTreeShakerException._('ConstFinder failure: ${constFinderProcessResult.stderr}');
    }
    final Object? constFinderMap = json.decode(constFinderProcessResult.stdout as String);
    if (constFinderMap is! Map<String, Object?>) {
      throw IconTreeShakerException._(
        'Invalid ConstFinder output: expected a top level JSON object, '
        'got $constFinderMap.',
      );
    }
    final _ConstFinderResult constFinderResult = _ConstFinderResult(constFinderMap);
    if (constFinderResult.hasNonConstantLocations) {
      _logger.printError(
        'This application cannot tree shake icons fonts. '
        'It has non-constant instances of IconData at the '
        'following locations:',
        emphasis: true,
      );
      for (final Map<String, Object?> location in constFinderResult.nonConstantLocations) {
        _logger.printError(
          '- ${location['file']}:${location['line']}:${location['column']}',
          indent: 2,
          hangingIndent: 4,
        );
      }
      throwToolExit(
        'Avoid non-constant invocations of IconData or try to '
        'build again with --no-tree-shake-icons.',
      );
    }
    return _parseConstFinderResult(constFinderResult);
  }

  Map<String, List<int>> _parseConstFinderResult(_ConstFinderResult constants) {
    final Map<String, List<int>> result = <String, List<int>>{};
    for (final Map<String, Object?> iconDataMap in constants.constantInstances) {
      final Object? package = iconDataMap['fontPackage'];
      final Object? fontFamily = iconDataMap['fontFamily'];
      final Object? codePoint = iconDataMap['codePoint'];
      if ((package ?? '') is! String || (fontFamily ?? '') is! String || codePoint is! num) {
        throw IconTreeShakerException._(
          'Invalid ConstFinder result. Expected "fontPackage" to be a String, '
          '"fontFamily" to be a String, and "codePoint" to be an int, '
          'got: $iconDataMap.',
        );
      }
      if (fontFamily == null) {
        _logger.printTrace(
          'Expected to find fontFamily for constant IconData with codepoint: '
          '$codePoint, but found fontFamily: $fontFamily. This usually means '
          'you are relying on the system font. Alternatively, font families in '
          'an IconData class can be provided in the assets section of your '
          'pubspec.yaml, or you are missing "uses-material-design: true".',
        );
        continue;
      }
      final String family = fontFamily as String;
      final String key = package == null ? family : 'packages/$package/$family';
      result[key] ??= <int>[];
      result[key]!.add(codePoint.round());
    }
    return result;
  }
}

class _ConstFinderResult {
  _ConstFinderResult(this.result);

  final Map<String, Object?> result;

  late final List<Map<String, Object?>> constantInstances = _getList(
    result['constantInstances'],
    'Invalid ConstFinder output: Expected "constInstances" to be a list of objects.',
  );

  late final List<Map<String, Object?>> nonConstantLocations = _getList(
    result['nonConstantLocations'],
    'Invalid ConstFinder output: Expected "nonConstLocations" to be a list of objects',
  );

  bool get hasNonConstantLocations => nonConstantLocations.isNotEmpty;
}

/// The font family name, relative path to font file, and list of code points
/// the application is using.
class _IconTreeShakerData {
  /// All parameters are required.
  const _IconTreeShakerData({
    required this.family,
    required this.relativePath,
    required this.codePoints,
    required this.optionalCodePoints,
  });

  /// The font family name, e.g. "MaterialIcons".
  final String family;

  /// The relative path to the font file.
  final String relativePath;

  /// The list of code points for the font.
  final List<int> codePoints;

  /// The list of code points to be optionally added, if they exist in the
  /// input font. Otherwise, the tool will silently omit them.
  final List<int> optionalCodePoints;

  @override
  String toString() => 'FontSubsetData($family, $relativePath, $codePoints)';
}

class IconTreeShakerException implements Exception {
  IconTreeShakerException._(this.message);

  final String message;

  @override
  String toString() =>
      'IconTreeShakerException: $message\n\n'
      'To disable icon tree shaking, pass --no-tree-shake-icons to the requested '
      'flutter build command';
}
