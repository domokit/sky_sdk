// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:collection';
import 'dart:io';

final TokenLogger tokenLogger = TokenLogger();

/// Class to keep track of used tokens and versions.
class TokenLogger {
  TokenLogger();

  void init({
    required Map<String, dynamic> allTokens,
    required Map<String, List<String>> versionMap,
  }) {
    _allTokens = allTokens;
    _versionMap = versionMap;
  }

  /// Map of all tokens to their values.
  late Map<String, dynamic> _allTokens;

  // Map of versions to their token files.
  late Map<String, List<String>> _versionMap;

  // Sorted set of used tokens.
  final SplayTreeSet<String> _usedTokens = SplayTreeSet<String>();

  // Set of tokens that were referenced on some templates, but do not exist.
  final Set<String> _unavailableTokens = <String>{};

  void clear() {
    _allTokens.clear();
    _versionMap.clear();
    _usedTokens.clear();
    _unavailableTokens.clear();
  }

  /// Logs a token.
  void log(String token) {
    if (!_allTokens.containsKey(token)) {
      _unavailableTokens.add(token);
      return;
    }
    _usedTokens.add(token);
  }

  /// Prints version usage to the console.
  void printVersionUsage({required bool verbose}) {
    final String versionsString = 'Versions used: ${_versionMap.keys.join(', ')}';
    print(versionsString);
    if (verbose) {
      for (final String version in _versionMap.keys) {
        print('  $version:');
        final List<String> files = List<String>.from(_versionMap[version]!);
        files.sort();
        for (final String file in files) {
          print('    $file');
        }
      }
      print('');
    }
  }

  /// Prints tokens usage to the console.
  void printTokensUsage({required bool verbose}) {
    final Set<String> allTokensSet = _allTokens.keys.toSet();

    if (verbose) {
      for (final String token in SplayTreeSet<String>.from(allTokensSet).toList()) {
        if (_usedTokens.contains(token)) {
          print('✅ $token');
        } else {
          print('❌ $token');
        }
      }
      print('');
    }

    print('Tokens used: ${_usedTokens.length}/${_allTokens.length}');

    if (_unavailableTokens.isNotEmpty) {
      print('');
      print(
        '\x1B[31m'
        'Some referenced tokens do not exist: ${_unavailableTokens.length}'
        '\x1B[0m',
      );
      for (final String token in _unavailableTokens) {
        print('  $token');
      }
    }
  }

  /// Dumps version and tokens usage to a file.
  void dumpToFile(String path) {
    final File file = File(path);
    file.createSync(recursive: true);
    final String versionsString = 'Versions used, ${_versionMap.keys.join(', ')}';
    file.writeAsStringSync('$versionsString\n${_usedTokens.join(',\n')}\n');
  }
}
