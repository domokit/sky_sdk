// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';

import 'package:path/path.dart' as p;

import '../logger.dart';
import 'command.dart';
import 'flags.dart';

/// The `cleanup` command for removing unnecessary on-disk artifacts.
final class CleanupCommand extends CommandBase {
  /// Constructs the `cleanup` command.
  CleanupCommand({required super.environment, super.help = false, super.usageLineLength}) {
    argParser.addFlag(
      dryRunFlag,
      abbr: 'd',
      help: 'Write changes to stdout without modifying the file system.',
      negatable: false,
    );

    argParser.addOption(
      'untouched-since',
      defaultsTo: () {
        const thirtyDays = Duration(days: 30);
        final dateTime = environment.now().subtract(thirtyDays);
        return _toDateString(dateTime);
      }(),
      help: 'What date to consider artifacts old enough to safely remove.',
      valueHelp: 'YYYY-MM-DD',
    );
  }

  @override
  String get name => 'cleanup';

  @override
  String get description => 'Removes stale or unnecessary on-disk artifacts.';

  @override
  List<String> get aliases => const ['gc'];

  static final _dateString = RegExp(r'^(\d{4})-(\d{2})-(\d{2})$');

  @override
  Future<int> run() async {
    final dryRun = argResults!.flag('dry-run');
    final since = () {
      final yyyyMmDd = argResults!.option('untouched-since')!;
      final dateMatch = _dateString.matchAsPrefix(yyyyMmDd);
      if (dateMatch == null) {
        throw FatalError('Invalid --untouched-since: $yyyyMmDd');
      }
      return DateTime(
        int.parse(dateMatch.group(1)!),
        int.parse(dateMatch.group(2)!),
        int.parse(dateMatch.group(3)!),
      );
    }();

    // Look at the directories in "out" for ones older than "since".
    environment.logger.status('Checking ${environment.engine.outDir.path}...');
    final toDelete = [
      await for (final entity in environment.engine.outDir.list())
        if (entity is Directory && _shouldDelete(entity, ifAccessedLaterThan: since)) entity,
    ]..sort((a, b) => a.path.compareTo(b.path));

    if (toDelete.isEmpty) {
      environment.logger.status('No directories were accessed later than ${_toDateString(since)}.');
      return 0;
    }

    final totalSize = toDelete.fold(0, (p, n) => p + _getSizeRecursive(n));

    if (dryRun) {
      environment.logger.status(
        'The following directories were accessed later than ${_toDateString(since)}:',
      );
      for (final e in toDelete) {
        environment.logger.status('  ${p.basename(e.path)}');
      }
      environment.logger.status(
        'Run without --dry-run to reclaim '
        '${_toReadableBytes(totalSize.toDouble())}.',
      );
      return 0;
    }

    final spinner = environment.logger.startSpinner();
    for (final e in toDelete) {
      try {
        await e.delete(recursive: true);
      } on FileSystemException catch (_) {
        environment.logger.warning('Failed to delete ${p.basename(e.path)}');
      }
    }
    spinner.finish();

    environment.logger.status(
      'Deleted ${toDelete.length} output directories and reclaimed '
      '${_toReadableBytes(totalSize.toDouble())}.',
    );

    return 0;
  }

  static bool _shouldDelete(Directory entity, {required DateTime ifAccessedLaterThan}) {
    final accessed = entity.statSync().accessed;
    return accessed.isBefore(ifAccessedLaterThan);
  }
}

int _getSizeRecursive(Directory dir) {
  return dir.listSync().fold(0, (p, n) => p + n.statSync().size);
}

String _toDateString(DateTime dateTime) {
  final output = StringBuffer('${dateTime.year}-');
  output.write(dateTime.month.toString().padLeft(2, '0'));
  output.write('-');
  output.write(dateTime.day.toString().padLeft(2, '0'));
  return output.toString();
}

String _toReadableBytes(double bytes) {
  var type = _FileSize.bytes;
  if (bytes >= 1024) {
    type = _FileSize.kilobytes;
    bytes = bytes / 1024;
  }
  if (bytes >= 1024) {
    type = _FileSize.megabytes;
    bytes = bytes / 1024;
  }
  if (bytes >= 1024) {
    type = _FileSize.gigabytes;
    bytes = bytes / 1024;
  }
  return '${bytes.toStringAsFixed(2)}${type.suffix}';
}

enum _FileSize {
  bytes('bytes'),
  kilobytes('KB'),
  megabytes('MB'),
  gigabytes('GB');

  const _FileSize(this.suffix);
  final String suffix;
}
