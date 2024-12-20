// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:archive/archive.dart';
import 'package:archive/archive_io.dart';
import 'package:meta/meta.dart';
import 'package:unified_analytics/unified_analytics.dart';
import 'package:vm_snapshot_analysis/treemap.dart';

import '../convert.dart';
import 'common.dart';
import 'file_system.dart';
import 'logger.dart';
import 'terminal.dart';

/// A class to analyze APK and AOT snapshot and generate a breakdown of the data.
class SizeAnalyzer {
  SizeAnalyzer({
    required FileSystem fileSystem,
    required Logger logger,
    required Analytics analytics,
    Pattern appFilenamePattern = 'libapp.so',
  }) : _analytics = analytics,
       _fileSystem = fileSystem,
       _logger = logger,
       _appFilenamePattern = appFilenamePattern;

  final FileSystem _fileSystem;
  final Logger _logger;
  final Pattern _appFilenamePattern;
  final Analytics _analytics;
  String? _appFilename;

  static const String aotSnapshotFileName = 'aot-snapshot.json';
  static const int tableWidth = 80;
  static const int _kAotSizeMaxDepth = 2;
  static const int _kZipSizeMaxDepth = 1;

  /// Analyze the [aotSnapshot] in an uncompressed output directory.
  Future<Map<String, Object?>> analyzeAotSnapshot({
    required Directory outputDirectory,
    required File aotSnapshot,
    required File precompilerTrace,
    required String type,
    String? excludePath,
  }) async {
    _logger.printStatus('▒' * tableWidth);
    _logger.printStatus('━' * tableWidth);
    final _SymbolNode aotAnalysisJson = _parseDirectory(
      outputDirectory,
      outputDirectory.parent.path,
      excludePath,
    );

    // Convert an AOT snapshot file into a map.
    final Object? decodedAotSnapshot = json.decode(aotSnapshot.readAsStringSync());
    if (decodedAotSnapshot == null) {
      throwToolExit('AOT snapshot is invalid for analysis');
    }
    final Map<String, Object?> processedAotSnapshotJson = treemapFromJson(decodedAotSnapshot);
    final _SymbolNode? aotSnapshotJsonRoot = _parseAotSnapshot(processedAotSnapshotJson);

    for (final _SymbolNode firstLevelPath in aotAnalysisJson.children) {
      _printEntitySize(firstLevelPath.name, byteSize: firstLevelPath.byteSize, level: 1);
      // Print the expansion of lib directory to show more info for `appFilename`.
      if (firstLevelPath.name == _fileSystem.path.basename(outputDirectory.path) &&
          aotSnapshotJsonRoot != null) {
        _printLibChildrenPaths(firstLevelPath, '', aotSnapshotJsonRoot, _kAotSizeMaxDepth, 0);
      }
    }

    _logger.printStatus('▒' * tableWidth);

    Map<String, Object?> apkAnalysisJson = aotAnalysisJson.toJson();

    apkAnalysisJson['type'] = type; // one of apk, aab, ios, macos, windows, or linux.

    apkAnalysisJson = _addAotSnapshotDataToAnalysis(
      apkAnalysisJson: apkAnalysisJson,
      path: _locatedAotFilePath,
      aotSnapshotJson: processedAotSnapshotJson,
      precompilerTrace:
          json.decode(precompilerTrace.readAsStringSync()) as Map<String, Object?>? ??
          <String, Object?>{},
    );

    assert(_appFilename != null);
    _analytics.send(Event.codeSizeAnalysis(platform: type));
    return apkAnalysisJson;
  }

  /// Analyzes [apk] and [aotSnapshot] to output a [Map] object that includes
  /// the breakdown of the both files, where the breakdown of [aotSnapshot] is placed
  /// under 'lib/arm64-v8a/$_appFilename'.
  ///
  /// [kind] must be one of 'apk' or 'aab'.
  /// The [aotSnapshot] can be either instruction sizes snapshot or a v8 snapshot.
  Future<Map<String, Object?>> analyzeZipSizeAndAotSnapshot({
    required File zipFile,
    required File aotSnapshot,
    required File precompilerTrace,
    required String kind,
  }) async {
    assert(kind == 'apk' || kind == 'aab');
    _logger.printStatus('▒' * tableWidth);
    _printEntitySize(
      '${zipFile.basename} (total compressed)',
      byteSize: zipFile.lengthSync(),
      level: 0,
      showColor: false,
    );
    _logger.printStatus('━' * tableWidth);

    final _SymbolNode apkAnalysisRoot = _parseUnzipFile(zipFile);

    // Convert an AOT snapshot file into a map.
    final Object? decodedAotSnapshot = json.decode(aotSnapshot.readAsStringSync());
    if (decodedAotSnapshot == null) {
      throwToolExit('AOT snapshot is invalid for analysis');
    }
    final Map<String, Object?> processedAotSnapshotJson = treemapFromJson(decodedAotSnapshot);
    final _SymbolNode? aotSnapshotJsonRoot = _parseAotSnapshot(processedAotSnapshotJson);
    if (aotSnapshotJsonRoot != null) {
      for (final _SymbolNode firstLevelPath in apkAnalysisRoot.children) {
        _printLibChildrenPaths(firstLevelPath, '', aotSnapshotJsonRoot, _kZipSizeMaxDepth, 0);
      }
    }
    _logger.printStatus('▒' * tableWidth);

    Map<String, Object?> apkAnalysisJson = apkAnalysisRoot.toJson();

    apkAnalysisJson['type'] = kind;

    assert(_appFilename != null);
    apkAnalysisJson = _addAotSnapshotDataToAnalysis(
      apkAnalysisJson: apkAnalysisJson,
      path: _locatedAotFilePath,
      aotSnapshotJson: processedAotSnapshotJson,
      precompilerTrace:
          json.decode(precompilerTrace.readAsStringSync()) as Map<String, Object?>? ??
          <String, Object?>{},
    );
    _analytics.send(Event.codeSizeAnalysis(platform: kind));
    return apkAnalysisJson;
  }

  _SymbolNode _parseUnzipFile(File zipFile) {
    final Archive archive = ZipDecoder().decodeBytes(zipFile.readAsBytesSync());
    final Map<List<String>, int> pathsToSize = <List<String>, int>{};

    for (final ArchiveFile archiveFile in archive.files) {
      final InputStreamBase? rawContent = archiveFile.rawContent;
      if (rawContent != null) {
        pathsToSize[_fileSystem.path.split(archiveFile.name)] = rawContent.length;
      }
    }
    return _buildSymbolTree(pathsToSize);
  }

  _SymbolNode _parseDirectory(Directory directory, String relativeTo, String? excludePath) {
    final Map<List<String>, int> pathsToSize = <List<String>, int>{};
    for (final File file in directory.listSync(recursive: true).whereType<File>()) {
      if (excludePath != null && file.uri.pathSegments.contains(excludePath)) {
        continue;
      }
      final List<String> path = _fileSystem.path.split(
        _fileSystem.path.relative(file.path, from: relativeTo),
      );
      pathsToSize[path] = file.lengthSync();
    }
    return _buildSymbolTree(pathsToSize);
  }

  List<String> _locatedAotFilePath = <String>[];

  List<String> _buildNodeName(_SymbolNode start, _SymbolNode? parent) {
    final List<String> results = <String>[start.name];
    while (parent != null && parent.name != 'Root') {
      results.insert(0, parent.name);
      parent = parent.parent;
    }
    return results;
  }

  _SymbolNode _buildSymbolTree(Map<List<String>, int> pathsToSize) {
    final _SymbolNode rootNode = _SymbolNode('Root');
    _SymbolNode currentNode = rootNode;

    for (final List<String> paths in pathsToSize.keys) {
      for (final String path in paths) {
        _SymbolNode? childWithPathAsName = currentNode.childByName(path);

        if (childWithPathAsName == null) {
          childWithPathAsName = _SymbolNode(path);
          if (matchesPattern(path, pattern: _appFilenamePattern) != null) {
            _appFilename = path;
            childWithPathAsName.name += ' (Dart AOT)';
            _locatedAotFilePath = _buildNodeName(childWithPathAsName, currentNode);
          } else if (path == 'libflutter.so') {
            childWithPathAsName.name += ' (Flutter Engine)';
          }
          currentNode.addChild(childWithPathAsName);
        }
        childWithPathAsName.addSize(pathsToSize[paths] ?? 0);
        currentNode = childWithPathAsName;
      }
      currentNode = rootNode;
    }
    return rootNode;
  }

  /// Prints all children paths for the lib/ directory in an APK.
  ///
  /// A brief summary of aot snapshot is printed under 'lib/arm64-v8a/$_appFilename'.
  void _printLibChildrenPaths(
    _SymbolNode currentNode,
    String totalPath,
    _SymbolNode aotSnapshotJsonRoot,
    int maxDepth,
    int currentDepth,
  ) {
    totalPath += currentNode.name;

    assert(_appFilename != null);
    if (currentNode.children.isNotEmpty &&
        currentNode.name != '$_appFilename (Dart AOT)' &&
        currentDepth < maxDepth &&
        currentNode.byteSize >= 1000) {
      for (final _SymbolNode child in currentNode.children) {
        _printLibChildrenPaths(
          child,
          '$totalPath/',
          aotSnapshotJsonRoot,
          maxDepth,
          currentDepth + 1,
        );
      }
      _leadingPaths = totalPath.split('/')..removeLast();
    } else {
      // Print total path and size if currentNode does not have any children and is
      // larger than 1KB
      final bool isAotSnapshotPath = _locatedAotFilePath.join('/').contains(totalPath);
      if (currentNode.byteSize >= 1000 || isAotSnapshotPath) {
        _printEntitySize(
          totalPath,
          byteSize: currentNode.byteSize,
          level: 1,
          emphasis: currentNode.children.isNotEmpty,
        );
        if (isAotSnapshotPath) {
          _printAotSnapshotSummary(aotSnapshotJsonRoot, level: totalPath.split('/').length);
        }
        _leadingPaths = totalPath.split('/')..removeLast();
      }
    }
  }

  /// Go through the AOT gen snapshot size JSON and print out a collapsed summary
  /// for the first package level.
  void _printAotSnapshotSummary(
    _SymbolNode aotSnapshotRoot, {
    int maxDirectoriesShown = 20,
    required int level,
  }) {
    _printEntitySize(
      'Dart AOT symbols accounted decompressed size',
      byteSize: aotSnapshotRoot.byteSize,
      level: level,
      emphasis: true,
    );

    final List<_SymbolNode> sortedSymbols =
        aotSnapshotRoot.children.toList()
          // Remove entries like  @unknown, @shared, and @stubs as well as private dart libraries
          //  which are not interpretable by end users.
          ..removeWhere(
            (_SymbolNode node) => node.name.startsWith('@') || node.name.startsWith('dart:_'),
          )
          ..sort((_SymbolNode a, _SymbolNode b) => b.byteSize.compareTo(a.byteSize));
    for (final _SymbolNode node in sortedSymbols.take(maxDirectoriesShown)) {
      // Node names will have an extra leading `package:*` name, remove it to
      // avoid extra nesting.
      _printEntitySize(
        _formatExtraLeadingPackages(node.name),
        byteSize: node.byteSize,
        level: level + 1,
      );
    }
  }

  String _formatExtraLeadingPackages(String name) {
    if (!name.startsWith('package')) {
      return name;
    }
    final List<String> chunks = name.split('/');
    if (chunks.length < 2) {
      return name;
    }
    chunks.removeAt(0);
    return chunks.join('/');
  }

  /// Adds breakdown of aot snapshot data as the children of the node at the given path.
  Map<String, Object?> _addAotSnapshotDataToAnalysis({
    required Map<String, Object?> apkAnalysisJson,
    required List<String> path,
    required Map<String, Object?> aotSnapshotJson,
    required Map<String, Object?> precompilerTrace,
  }) {
    Map<String, Object?> currentLevel = apkAnalysisJson;
    currentLevel['precompiler-trace'] = precompilerTrace;
    while (path.isNotEmpty) {
      final List<Map<String, Object?>>? children =
          currentLevel['children'] as List<Map<String, Object?>>?;
      final Map<String, Object?> childWithPathAsName =
          children?.firstWhere(
            (Map<String, Object?> child) => (child['n'] as String?) == path.first,
          ) ??
          <String, Object?>{};
      path.removeAt(0);
      currentLevel = childWithPathAsName;
    }
    currentLevel['children'] = aotSnapshotJson['children'];
    return apkAnalysisJson;
  }

  List<String> _leadingPaths = <String>[];

  /// Print an entity's name with its size on the same line.
  void _printEntitySize(
    String entityName, {
    required int byteSize,
    required int level,
    bool showColor = true,
    bool emphasis = false,
  }) {
    final String formattedSize = _prettyPrintBytes(byteSize);

    TerminalColor color = TerminalColor.green;
    if (formattedSize.endsWith('MB')) {
      color = TerminalColor.cyan;
    } else if (formattedSize.endsWith('KB')) {
      color = TerminalColor.yellow;
    }

    // Compute any preceding directories, and compare this to the stored
    // directories (in _leadingPaths) for the last entity that was printed. The
    // similarly determines whether or not leading directory information needs to
    // be printed.
    final List<String> localSegments = entityName.split('/')..removeLast();
    int i = 0;
    while (i < _leadingPaths.length &&
        i < localSegments.length &&
        _leadingPaths[i] == localSegments[i]) {
      i += 1;
    }
    for (; i < localSegments.length; i += 1) {
      _logger.printStatus('${localSegments[i]}/', indent: (level + i) * 2, emphasis: true);
    }
    _leadingPaths = localSegments;

    final String baseName = _fileSystem.path.basename(entityName);
    final int spaceInBetween =
        tableWidth - (level + i) * 2 - baseName.length - formattedSize.length;
    _logger.printStatus(
      baseName + ' ' * spaceInBetween,
      newline: false,
      emphasis: emphasis,
      indent: (level + i) * 2,
    );
    _logger.printStatus(formattedSize, color: showColor ? color : null);
  }

  String _prettyPrintBytes(int numBytes) {
    const int kB = 1024;
    const int mB = kB * 1024;
    if (numBytes < kB) {
      return '$numBytes B';
    } else if (numBytes < mB) {
      return '${(numBytes / kB).round()} KB';
    } else {
      return '${(numBytes / mB).round()} MB';
    }
  }

  _SymbolNode? _parseAotSnapshot(Map<String, Object?> aotSnapshotJson) {
    final bool isLeafNode = aotSnapshotJson['children'] == null;
    if (!isLeafNode) {
      return _buildNodeWithChildren(aotSnapshotJson);
    } else {
      // TODO(peterdjlee): Investigate why there are leaf nodes with size of null.
      final int? byteSize = aotSnapshotJson['value'] as int?;
      if (byteSize == null) {
        return null;
      }
      return _buildNode(aotSnapshotJson, byteSize);
    }
  }

  _SymbolNode _buildNode(
    Map<String, Object?> aotSnapshotJson,
    int byteSize, {
    List<_SymbolNode> children = const <_SymbolNode>[],
  }) {
    final String name = aotSnapshotJson['n']! as String;
    final Map<String, _SymbolNode> childrenMap = <String, _SymbolNode>{};

    for (final _SymbolNode child in children) {
      childrenMap[child.name] = child;
    }

    return _SymbolNode(name, byteSize: byteSize)..addAllChildren(children);
  }

  /// Builds a node by recursively building all of its children first
  /// in order to calculate the sum of its children's sizes.
  _SymbolNode? _buildNodeWithChildren(Map<String, Object?> aotSnapshotJson) {
    final List<Object?> rawChildren = aotSnapshotJson['children'] as List<Object?>? ?? <Object?>[];
    final List<_SymbolNode> symbolNodeChildren = <_SymbolNode>[];
    int totalByteSize = 0;

    // Given a child, build its subtree.
    for (final Object? child in rawChildren) {
      if (child == null) {
        continue;
      }
      final _SymbolNode? childTreemapNode = _parseAotSnapshot(child as Map<String, Object?>);
      if (childTreemapNode != null) {
        symbolNodeChildren.add(childTreemapNode);
        totalByteSize += childTreemapNode.byteSize;
      }
    }

    // If none of the children matched the diff tree type
    if (totalByteSize == 0) {
      return null;
    } else {
      return _buildNode(aotSnapshotJson, totalByteSize, children: symbolNodeChildren);
    }
  }
}

/// A node class that represents a single symbol for AOT size snapshots.
class _SymbolNode {
  _SymbolNode(this.name, {this.byteSize = 0}) : _children = <String, _SymbolNode>{};

  /// The human friendly identifier for this node.
  String name;

  int byteSize;
  void addSize(int sizeToBeAdded) {
    byteSize += sizeToBeAdded;
  }

  _SymbolNode? get parent => _parent;
  _SymbolNode? _parent;

  Iterable<_SymbolNode> get children => _children.values;
  final Map<String, _SymbolNode> _children;

  _SymbolNode? childByName(String name) => _children[name];

  _SymbolNode addChild(_SymbolNode child) {
    assert(child.parent == null);
    assert(!_children.containsKey(child.name), 'Cannot add duplicate child key ${child.name}');

    child._parent = this;
    _children[child.name] = child;
    return child;
  }

  void addAllChildren(List<_SymbolNode> children) {
    children.forEach(addChild);
  }

  Map<String, Object?> toJson() {
    final List<Map<String, Object?>> childrenAsJson = <Map<String, Object?>>[
      for (final _SymbolNode child in children) child.toJson(),
    ];

    return <String, Object?>{
      'n': name,
      'value': byteSize,
      if (childrenAsJson.isNotEmpty) 'children': childrenAsJson,
    };
  }
}

/// Matches `pattern` against the entirety of `string`.
@visibleForTesting
Match? matchesPattern(String string, {required Pattern pattern}) {
  final Match? match = pattern.matchAsPrefix(string);
  return (match != null && match.end == string.length) ? match : null;
}
