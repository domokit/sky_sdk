// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/foundation.dart';
/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter/scheduler.dart';
library;

import 'dart:async';

import 'package:meta/meta.dart' show visibleForTesting;

/// Signature for callbacks passed to [LicenseRegistry.addLicense].
typedef LicenseEntryCollector = Stream<LicenseEntry> Function();

/// A string that represents one paragraph in a [LicenseEntry].
///
/// See [LicenseEntry.paragraphs].
class LicenseParagraph {
  /// Creates a string for a license entry paragraph.
  const LicenseParagraph(this.text, this.indent);

  /// The text of the paragraph. Should not have any leading or trailing whitespace.
  final String text;

  /// How many steps of indentation the paragraph has.
  ///
  /// * 0 means the paragraph is not indented.
  /// * 1 means the paragraph is indented one unit of indentation.
  /// * 2 means the paragraph is indented two units of indentation.
  ///
  /// ...and so forth.
  ///
  /// In addition, the special value [centeredIndent] can be used to indicate
  /// that rather than being indented, the paragraph is centered.
  final int indent; // can be set to centeredIndent

  /// A constant that represents "centered" alignment for [indent].
  static const int centeredIndent = -1;
}

/// A license that covers part of the application's software or assets, to show
/// in an interface such as the [LicensePage].
///
/// For optimal performance, [LicenseEntry] objects should only be created on
/// demand in [LicenseEntryCollector] callbacks passed to
/// [LicenseRegistry.addLicense].
abstract class LicenseEntry {
  /// Abstract const constructor. This constructor enables subclasses to provide
  /// const constructors so that they can be used in const expressions.
  const LicenseEntry();

  /// The names of the packages that this license entry applies to.
  Iterable<String> get packages;

  /// The paragraphs of the license, each as a [LicenseParagraph] consisting of
  /// a string and some formatting information. Paragraphs can include newline
  /// characters, but this is discouraged as it results in ugliness.
  Iterable<LicenseParagraph> get paragraphs;
}

enum _LicenseEntryWithLineBreaksParserState { beforeParagraph, inParagraph }

/// Variant of [LicenseEntry] for licenses that separate paragraphs with blank
/// lines and that hard-wrap text within paragraphs. Lines that begin with one
/// or more space characters are also assumed to introduce new paragraphs,
/// unless they start with the same number of spaces as the previous line, in
/// which case it's assumed they are a continuation of an indented paragraph.
///
/// {@tool snippet}
///
/// For example, the BSD license in this format could be encoded as follows:
///
/// ```dart
/// void initMyLibrary() {
///   LicenseRegistry.addLicense(() => Stream<LicenseEntry>.value(
///     const LicenseEntryWithLineBreaks(<String>['my_library'], '''
/// Copyright 2016 The Sample Authors. All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are
/// met:
///
///    * Redistributions of source code must retain the above copyright
/// notice, this list of conditions and the following disclaimer.
///    * Redistributions in binary form must reproduce the above
/// copyright notice, this list of conditions and the following disclaimer
/// in the documentation and/or other materials provided with the
/// distribution.
///    * Neither the name of Example Inc. nor the names of its
/// contributors may be used to endorse or promote products derived from
/// this software without specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
/// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
/// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
/// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
/// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
/// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
/// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
/// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
/// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
/// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.''',
///     ),
///   ));
/// }
/// ```
/// {@end-tool}
///
/// This would result in a license with six [paragraphs], the third, fourth, and
/// fifth being indented one level.
///
/// ## Performance considerations
///
/// Computing the paragraphs is relatively expensive. Doing the work for one
/// license per frame is reasonable; doing more at the same time is ill-advised.
/// Consider doing all the work at once using [compute] to move the work to
/// another thread, or spreading the work across multiple frames using
/// [SchedulerBinding.scheduleTask].
class LicenseEntryWithLineBreaks extends LicenseEntry {
  /// Create a license entry for a license whose text is hard-wrapped within
  /// paragraphs and has paragraph breaks denoted by blank lines or with
  /// indented text.
  const LicenseEntryWithLineBreaks(this.packages, this.text);

  @override
  final List<String> packages;

  /// The text of the license.
  ///
  /// The text will be split into paragraphs according to the following
  /// conventions:
  ///
  /// * Lines starting with a different number of space characters than the
  ///   previous line start a new paragraph, with those spaces removed.
  /// * Blank lines start a new paragraph.
  /// * Other line breaks are replaced by a single space character.
  /// * Leading spaces on a line are removed.
  ///
  /// For each paragraph, the algorithm attempts (using some rough heuristics)
  /// to identify how indented the paragraph is, or whether it is centered.
  final String text;

  @override
  Iterable<LicenseParagraph> get paragraphs {
    int lineStart = 0;
    int currentPosition = 0;
    int lastLineIndent = 0;
    int currentLineIndent = 0;
    int? currentParagraphIndentation;
    _LicenseEntryWithLineBreaksParserState state =
        _LicenseEntryWithLineBreaksParserState.beforeParagraph;
    final List<String> lines = <String>[];
    final List<LicenseParagraph> result = <LicenseParagraph>[];

    void addLine() {
      assert(lineStart < currentPosition);
      lines.add(text.substring(lineStart, currentPosition));
    }

    LicenseParagraph getParagraph() {
      assert(lines.isNotEmpty);
      assert(currentParagraphIndentation != null);
      final LicenseParagraph result = LicenseParagraph(
        lines.join(' '),
        currentParagraphIndentation!,
      );
      assert(result.text.trimLeft() == result.text);
      assert(result.text.isNotEmpty);
      lines.clear();
      return result;
    }

    while (currentPosition < text.length) {
      switch (state) {
        case _LicenseEntryWithLineBreaksParserState.beforeParagraph:
          assert(lineStart == currentPosition);
          switch (text[currentPosition]) {
            case ' ':
              lineStart = currentPosition + 1;
              currentLineIndent += 1;
              state = _LicenseEntryWithLineBreaksParserState.beforeParagraph;
            case '\t':
              lineStart = currentPosition + 1;
              currentLineIndent += 8;
              state = _LicenseEntryWithLineBreaksParserState.beforeParagraph;
            case '\r':
            case '\n':
            case '\f':
              if (lines.isNotEmpty) {
                result.add(getParagraph());
              }
              if (text[currentPosition] == '\r' &&
                  currentPosition < text.length - 1 &&
                  text[currentPosition + 1] == '\n') {
                currentPosition += 1;
              }
              lastLineIndent = 0;
              currentLineIndent = 0;
              currentParagraphIndentation = null;
              lineStart = currentPosition + 1;
              state = _LicenseEntryWithLineBreaksParserState.beforeParagraph;
            case '[':
              // This is a bit of a hack for the LGPL 2.1, which does something like this:
              //
              //   [this is a
              //    single paragraph]
              //
              // ...near the top.
              currentLineIndent += 1;
              continue startParagraph;
            startParagraph:
            default:
              if (lines.isNotEmpty && currentLineIndent > lastLineIndent) {
                result.add(getParagraph());
                currentParagraphIndentation = null;
              }
              // The following is a wild heuristic for guessing the indentation level.
              // It happens to work for common variants of the BSD and LGPL licenses.
              if (currentParagraphIndentation == null) {
                if (currentLineIndent > 10) {
                  currentParagraphIndentation = LicenseParagraph.centeredIndent;
                } else {
                  currentParagraphIndentation = currentLineIndent ~/ 3;
                }
              }
              state = _LicenseEntryWithLineBreaksParserState.inParagraph;
          }
        case _LicenseEntryWithLineBreaksParserState.inParagraph:
          switch (text[currentPosition]) {
            case '\n':
              addLine();
              lastLineIndent = currentLineIndent;
              currentLineIndent = 0;
              lineStart = currentPosition + 1;
              state = _LicenseEntryWithLineBreaksParserState.beforeParagraph;
            case '\f':
              addLine();
              result.add(getParagraph());
              lastLineIndent = 0;
              currentLineIndent = 0;
              currentParagraphIndentation = null;
              lineStart = currentPosition + 1;
              state = _LicenseEntryWithLineBreaksParserState.beforeParagraph;
            default:
              state = _LicenseEntryWithLineBreaksParserState.inParagraph;
          }
      }
      currentPosition += 1;
    }
    switch (state) {
      case _LicenseEntryWithLineBreaksParserState.beforeParagraph:
        if (lines.isNotEmpty) {
          result.add(getParagraph());
        }
      case _LicenseEntryWithLineBreaksParserState.inParagraph:
        addLine();
        result.add(getParagraph());
    }
    return result;
  }
}

/// A registry for packages to add licenses to, so that they can be displayed
/// together in an interface such as the [LicensePage].
///
/// Packages can register their licenses using [addLicense]. User interfaces
/// that wish to show all the licenses can obtain them by calling [licenses].
///
/// The flutter tool will automatically collect the contents of all the LICENSE
/// files found at the root of each package into a single LICENSE file in the
/// default asset bundle. Each license in that file is separated from the next
/// by a line of eighty hyphens (`-`), and begins with a list of package names
/// that the license applies to, one to a line, separated from the next by a
/// blank line. The `services` package registers a license collector that splits
/// that file and adds each entry to the registry.
///
/// The LICENSE files in each package can either consist of a single license, or
/// can be in the format described above. In the latter case, each component
/// license and list of package names is merged independently.
///
/// See also:
///
///  * [showAboutDialog], which shows a Material-style dialog with information
///    about the application, including a button that shows a [LicensePage] that
///    uses this API to select licenses to show.
///  * [AboutListTile], which is a widget that can be added to a [Drawer]. When
///    tapped it calls [showAboutDialog].
abstract final class LicenseRegistry {
  static List<LicenseEntryCollector>? _collectors;

  /// Adds licenses to the registry.
  ///
  /// To avoid actually manipulating the licenses unless strictly necessary,
  /// licenses are added by adding a closure that returns a list of
  /// [LicenseEntry] objects. The closure is only called if [licenses] is itself
  /// called; in normal operation, if the user does not request to see the
  /// licenses, the closure will not be called.
  static void addLicense(LicenseEntryCollector collector) {
    _collectors ??= <LicenseEntryCollector>[];
    _collectors!.add(collector);
  }

  /// Returns the licenses that have been registered.
  ///
  /// Generating the list of licenses is expensive.
  static Stream<LicenseEntry> get licenses {
    if (_collectors == null) {
      return const Stream<LicenseEntry>.empty();
    }

    late final StreamController<LicenseEntry> controller;
    controller = StreamController<LicenseEntry>(
      onListen: () async {
        for (final LicenseEntryCollector collector in _collectors!) {
          await controller.addStream(collector());
        }
        await controller.close();
      },
    );
    return controller.stream;
  }

  /// Resets the internal state of [LicenseRegistry]. Intended for use in
  /// testing.
  @visibleForTesting
  static void reset() {
    _collectors = null;
  }
}
