// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:args/args.dart';

import 'package:flutter_devicelab/framework/ab.dart';
import 'package:flutter_devicelab/framework/utils.dart';

String kRawSummaryOpt = 'raw-summary';
String kTabTableOpt = 'tsv-table';
String kAsciiTableOpt = 'ascii-table';

void _usage(String error) {
  stderr.writeln(error);
  stderr.writeln('Usage:\n');
  stderr.writeln(_argParser.usage);
  exitCode = 1;
}

Future<void> main(List<String> rawArgs) async {
  ArgResults args;
  try {
    args = _argParser.parse(rawArgs);
  } on FormatException catch (error) {
    _usage('${error.message}\n');
    return;
  }

  final List<String> jsonFiles = args.rest.isNotEmpty ? args.rest : <String>['ABresults.json'];

  for (final String filename in jsonFiles) {
    final File file = File(filename);
    if (!file.existsSync()) {
      _usage('File "$filename" does not exist');
      return;
    }

    ABTest test;
    try {
      test = ABTest.fromJsonMap(
        const JsonDecoder().convert(await file.readAsString()) as Map<String, dynamic>,
      );
    } catch (error) {
      _usage('Could not parse json file "$filename"');
      return;
    }

    if (args[kRawSummaryOpt] as bool) {
      section('Raw results for "$filename"');
      print(test.rawResults());
    }
    if (args[kTabTableOpt] as bool) {
      section('A/B comparison for "$filename"');
      print(test.printSummary());
    }
    if (args[kAsciiTableOpt] as bool) {
      section('Formatted summary for "$filename"');
      print(test.asciiSummary());
    }
  }
}

/// Command-line options for the `summarize.dart` command.
final ArgParser _argParser =
    ArgParser()
      ..addFlag(
        kAsciiTableOpt,
        defaultsTo: true,
        help: 'Prints the summary in a table formatted nicely for terminal output.',
      )
      ..addFlag(
        kTabTableOpt,
        defaultsTo: true,
        help: 'Prints the summary in a table with tabs for easy spreadsheet entry.',
      )
      ..addFlag(
        kRawSummaryOpt,
        defaultsTo: true,
        help:
            'Prints all per-run data collected by the A/B test formatted with\n'
            'tabs for easy spreadsheet entry.',
      );
