// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as p;

// This script verifies that the release binaries only export the expected
// symbols.
//
// Android binaries (libflutter.so) should only export one symbol "JNI_OnLoad"
// of type "T".
//
// iOS binaries (Flutter.framework/Flutter) should only export Objective-C
// Symbols from the Flutter namespace. These are either of type
// "(__DATA,__common)" or "(__DATA,__objc_data)".

/// Takes the path to the out directory as the first argument, and the path to
/// the buildtools directory as the second argument.
///
/// If the second argument is not specified, for backwards compatibility, it is
/// assumed that it is ../buildtools relative to the first parameter (the out
/// directory).
void main(List<String> arguments) {
  if (arguments.isEmpty || arguments.length > 2) {
    print('usage: dart verify_exported.dart OUT_DIR [BUILDTOOLS]');
    exit(1);
  }
  String outPath = arguments.first;
  if (p.isRelative(outPath)) {
    /// If path is relative then create a full path starting from the engine checkout
    /// repository.
    if (!Platform.environment.containsKey('ENGINE_CHECKOUT_PATH')) {
      print('ENGINE_CHECKOUT_PATH env variable is mandatory when using relative destination path');
      exit(1);
    }
    final String engineCheckoutPath = Platform.environment['ENGINE_CHECKOUT_PATH']!;
    outPath = p.join(engineCheckoutPath, outPath);
  }
  final String buildToolsPath =
      arguments.length == 1 ? p.join(p.dirname(outPath), 'flutter', 'buildtools') : arguments[1];

  String platform;
  if (Platform.isLinux) {
    platform = 'linux-x64';
  } else if (Platform.isMacOS) {
    platform = 'mac-x64';
  } else {
    throw UnimplementedError('Script only support running on Linux or MacOS.');
  }
  final String nmPath = p.join(buildToolsPath, platform, 'clang', 'bin', 'llvm-nm');
  if (!Directory(outPath).existsSync()) {
    print('error: build out directory not found: $outPath');
    exit(1);
  }

  final Iterable<String> releaseBuilds = Directory(outPath)
      .listSync()
      .whereType<Directory>()
      .map<String>((FileSystemEntity dir) => p.basename(dir.path))
      .where((String s) => s.contains('_release'));

  final Iterable<String> iosReleaseBuilds = releaseBuilds.where((String s) => s.startsWith('ios_'));
  final Iterable<String> androidReleaseBuilds = releaseBuilds.where(
    (String s) => s.startsWith('android_'),
  );
  final Iterable<String> hostReleaseBuilds = releaseBuilds.where(
    (String s) => s.startsWith('host_'),
  );

  int failures = 0;
  failures += _checkIos(outPath, nmPath, iosReleaseBuilds);
  failures += _checkAndroid(outPath, nmPath, androidReleaseBuilds);
  if (Platform.isLinux) {
    failures += _checkLinux(outPath, nmPath, hostReleaseBuilds);
  }
  print('Failing checks: $failures');
  exit(failures);
}

int _checkIos(String outPath, String nmPath, Iterable<String> builds) {
  int failures = 0;
  for (final String build in builds) {
    final String libFlutter = p.join(outPath, build, 'Flutter.framework', 'Flutter');
    if (!File(libFlutter).existsSync()) {
      print('SKIPPING: $libFlutter does not exist.');
      continue;
    }
    final ProcessResult nmResult = Process.runSync(nmPath, <String>['-gUm', libFlutter]);
    if (nmResult.exitCode != 0) {
      print('ERROR: failed to execute "nm -gUm $libFlutter":\n${nmResult.stderr}');
      failures++;
      continue;
    }
    final Iterable<NmEntry> unexpectedEntries = NmEntry.parse(nmResult.stdout as String).where((
      NmEntry entry,
    ) {
      final bool cSymbol =
          (entry.type == '(__DATA,__common)' || entry.type == '(__DATA,__const)') &&
          entry.name.startsWith('_Flutter');
      final bool cInternalSymbol =
          entry.type == '(__TEXT,__text)' && entry.name.startsWith('_InternalFlutter');
      final bool objcSymbol =
          entry.type == '(__DATA,__objc_data)' &&
          (entry.name.startsWith(r'_OBJC_METACLASS_$_Flutter') ||
              entry.name.startsWith(r'_OBJC_CLASS_$_Flutter'));
      return !(cSymbol || cInternalSymbol || objcSymbol);
    });
    if (unexpectedEntries.isNotEmpty) {
      print('ERROR: $libFlutter exports unexpected symbols:');
      print(
        unexpectedEntries.fold<String>('', (String previous, NmEntry entry) {
          return '${previous == '' ? '' : '$previous\n'}     ${entry.type} ${entry.name}';
        }),
      );
      failures++;
    } else {
      print('OK: $libFlutter');
    }
  }
  return failures;
}

int _checkAndroid(String outPath, String nmPath, Iterable<String> builds) {
  int failures = 0;
  for (final String build in builds) {
    final String libFlutter = p.join(outPath, build, 'libflutter.so');
    if (!File(libFlutter).existsSync()) {
      print('SKIPPING: $libFlutter does not exist.');
      continue;
    }
    final ProcessResult nmResult = Process.runSync(nmPath, <String>['-gU', libFlutter]);
    if (nmResult.exitCode != 0) {
      print('ERROR: failed to execute "nm -gU $libFlutter":\n${nmResult.stderr}');
      failures++;
      continue;
    }
    final Iterable<NmEntry> entries = NmEntry.parse(nmResult.stdout as String);
    final Map<String, String> entryMap = <String, String>{
      for (final NmEntry entry in entries) entry.name: entry.type,
    };
    final Map<String, String> expectedSymbols = <String, String>{
      'JNI_OnLoad': 'T',
      '_binary_icudtl_dat_size': 'R',
      '_binary_icudtl_dat_start': 'R',
    };
    final Map<String, String> badSymbols = <String, String>{};
    for (final String key in entryMap.keys) {
      final bool isValidFlutterGpuSymbol =
          key.startsWith('InternalFlutterGpu') && entryMap[key] == 'T';
      if (!isValidFlutterGpuSymbol && entryMap[key] != expectedSymbols[key]) {
        badSymbols[key] = entryMap[key]!;
      }
    }
    if (badSymbols.isNotEmpty) {
      print('ERROR: $libFlutter exports the wrong symbols');
      print(' Expected $expectedSymbols');
      print(' Library has $entryMap.');
      failures++;
    } else {
      print('OK: $libFlutter');
    }
  }
  return failures;
}

int _checkLinux(String outPath, String nmPath, Iterable<String> builds) {
  int failures = 0;
  for (final String build in builds) {
    final String libFlutter = p.join(outPath, build, 'libflutter_engine.so');
    if (!File(libFlutter).existsSync()) {
      print('SKIPPING: $libFlutter does not exist.');
      continue;
    }
    final ProcessResult nmResult = Process.runSync(nmPath, <String>['-gUD', libFlutter]);
    if (nmResult.exitCode != 0) {
      print('ERROR: failed to execute "nm -gUD $libFlutter":\n${nmResult.stderr}');
      failures++;
      continue;
    }
    final List<NmEntry> entries = NmEntry.parse(nmResult.stdout as String).toList();
    for (final NmEntry entry in entries) {
      if (entry.type != 'T' && entry.type != 'R') {
        print('ERROR: $libFlutter exports an unexpected symbol type: ($entry)');
        print(' Library has $entries.');
        failures++;
        break;
      }
      if (!(entry.name.startsWith('Flutter') ||
          entry.name.startsWith('__Flutter') ||
          entry.name.startsWith('kFlutter') ||
          entry.name.startsWith('InternalFlutter') ||
          entry.name.startsWith('kInternalFlutter'))) {
        print('ERROR: $libFlutter exports an unexpected symbol name: ($entry)');
        print(' Library has $entries.');
        failures++;
        break;
      }
    }
  }
  return failures;
}

class NmEntry {
  NmEntry._(this.type, this.name);

  final String type;
  final String name;

  static Iterable<NmEntry> parse(String stdout) {
    return LineSplitter.split(stdout).map((String line) {
      final List<String> parts = line.split(' ');
      return NmEntry._(parts[1], parts.last);
    });
  }

  @override
  String toString() => '$name: $type';
}
