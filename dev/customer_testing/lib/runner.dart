// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:path/path.dart' as path;

import 'customer_test.dart';

Future<bool> runTests({
  int repeat = 1,
  bool skipOnFetchFailure = false,
  bool verbose = false,
  int numberShards = 1,
  int shardIndex = 0,
  required List<File> files,
}) async {
  if (verbose) {
    print('Starting run_tests.dart...');
  }

  // Best attempt at evenly splitting tests among the shards
  final List<File> shardedFiles = <File>[];
  for (int i = shardIndex; i < files.length; i += numberShards) {
    shardedFiles.add(files[i]);
  }

  int testCount = 0;
  int failures = 0;

  if (verbose) {
    final String s = files.length == 1 ? '' : 's';
    if (numberShards > 1) {
      final String ss = shardedFiles.length == 1 ? '' : 's';
      print(
        '${files.length} file$s specified. ${shardedFiles.length} test$ss in shard #$shardIndex ($numberShards shards total).',
      );
    } else {
      print('${files.length} file$s specified.');
    }
    print('');
  }

  if (verbose) {
    if (numberShards > 1) {
      print('Tests in this shard:');
    } else {
      print('Tests:');
    }
    for (final File file in shardedFiles) {
      print(file.path);
    }
  }
  print('');

  for (final File file in shardedFiles) {
    // Always print name of running task for debugging individual customer test
    // suites.
    print('Processing ${file.path}...');

    void failure(String message) {
      print('ERROR: $message');
      failures += 1;
    }

    CustomerTest instructions;
    try {
      instructions = CustomerTest(file);
    } on FormatException catch (error) {
      failure(error.message);
      print('');
      continue;
    } on FileSystemException catch (error) {
      failure(error.message);
      print('');
      continue;
    }

    bool success = true;

    final Directory checkout = Directory.systemTemp.createTempSync(
      'flutter_customer_testing.${path.basenameWithoutExtension(file.path)}.',
    );
    if (verbose) {
      print('Created temporary directory: ${checkout.path}');
    }
    try {
      assert(instructions.fetch.isNotEmpty);
      for (final String fetchCommand in instructions.fetch) {
        success = await shell(
          fetchCommand,
          checkout,
          verbose: verbose,
          silentFailure: skipOnFetchFailure,
        );
        if (!success) {
          if (skipOnFetchFailure) {
            if (verbose) {
              print('Skipping (fetch failed).');
            } else {
              print('Skipping ${file.path} (fetch failed).');
            }
          } else {
            failure('Failed to fetch repository.');
          }
          break;
        }
      }
      if (success) {
        final Directory customerRepo = Directory(path.join(checkout.path, 'tests'));
        for (final String setupCommand in instructions.setup) {
          if (verbose) {
            print('Running setup command: $setupCommand');
          }
          success = await shell(setupCommand, customerRepo, verbose: verbose);
          if (!success) {
            failure('Setup command failed: $setupCommand');
            break;
          }
        }
        for (final Directory updateDirectory in instructions.update) {
          final Directory resolvedUpdateDirectory = Directory(
            path.join(customerRepo.path, updateDirectory.path),
          );
          if (verbose) {
            print('Updating code in ${resolvedUpdateDirectory.path}...');
          }
          if (!File(path.join(resolvedUpdateDirectory.path, 'pubspec.yaml')).existsSync()) {
            failure(
              'The directory ${updateDirectory.path}, which was specified as an update directory, does not contain a "pubspec.yaml" file.',
            );
            success = false;
            break;
          }
          success = await shell('flutter packages get', resolvedUpdateDirectory, verbose: verbose);
          if (!success) {
            failure(
              'Could not run "flutter pub get" in ${updateDirectory.path}, which was specified as an update directory.',
            );
            break;
          }
          success = await shell('dart fix --apply', resolvedUpdateDirectory, verbose: verbose);
          if (!success) {
            failure(
              'Could not run "dart fix" in ${updateDirectory.path}, which was specified as an update directory.',
            );
            break;
          }
        }
        if (success) {
          if (verbose) {
            print('Running tests...');
          }
          if (instructions.iterations != null && instructions.iterations! < repeat) {
            if (verbose) {
              final String s = instructions.iterations == 1 ? '' : 's';
              print(
                'Limiting to ${instructions.iterations} round$s rather than $repeat rounds because of "iterations" directive.',
              );
            }
            repeat = instructions.iterations!;
          }
          final Stopwatch stopwatch = Stopwatch()..start();
          for (int iteration = 0; iteration < repeat; iteration += 1) {
            if (verbose && repeat > 1) {
              print('Round ${iteration + 1} of $repeat.');
            }
            for (final String testCommand in instructions.tests) {
              testCount += 1;
              success = await shell(testCommand, customerRepo, verbose: verbose);
              if (!success) {
                failure(
                  'One or more tests from ${path.basenameWithoutExtension(file.path)} failed.',
                );
                break;
              }
            }
          }
          stopwatch.stop();
          // Always print test runtime for debugging.
          print(
            'Tests finished in ${(stopwatch.elapsed.inSeconds / repeat).toStringAsFixed(2)} seconds per iteration.',
          );
        }
      }
    } finally {
      if (verbose) {
        print('Deleting temporary directory...');
      }
      try {
        checkout.deleteSync(recursive: true);
      } on FileSystemException {
        print('Failed to delete "${checkout.path}".');
      }
    }
    if (!success) {
      final String s = instructions.contacts.length == 1 ? '' : 's';
      print('Contact$s: ${instructions.contacts.join(", ")}');
    }
    if (verbose || !success) {
      print('');
    }
  }
  if (failures > 0) {
    final String s = failures == 1 ? '' : 's';
    print('$failures failure$s.');
    return false;
  }
  print('$testCount tests all passed!');
  return true;
}

final RegExp _spaces = RegExp(r' +');

Future<bool> shell(
  String command,
  Directory directory, {
  bool verbose = false,
  bool silentFailure = false,
  void Function()? failedCallback,
}) async {
  if (verbose) {
    print('>> $command');
  }
  Process process;
  if (Platform.isWindows) {
    process = await Process.start('CMD.EXE', <String>[
      '/S',
      '/C',
      command,
    ], workingDirectory: directory.path);
  } else {
    final List<String> segments = command.trim().split(_spaces);
    process = await Process.start(
      segments.first,
      segments.skip(1).toList(),
      workingDirectory: directory.path,
    );
  }
  final List<String> output = <String>[];
  utf8.decoder
      .bind(process.stdout)
      .transform(const LineSplitter())
      .listen(verbose ? printLog : output.add);
  utf8.decoder
      .bind(process.stderr)
      .transform(const LineSplitter())
      .listen(verbose ? printLog : output.add);
  final bool success = await process.exitCode == 0;
  if (success || silentFailure) {
    return success;
  }
  if (!verbose) {
    if (failedCallback != null) {
      failedCallback();
    }
    print('>> $command');
    output.forEach(printLog);
  }
  return success;
}

void printLog(String line) {
  print('| $line'.trimRight());
}
