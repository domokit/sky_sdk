// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:core' hide RegExp;

import 'patterns.dart';

void _stripFullLines(List<String> lines) {
  // Strip full-line decorations, e.g. horizontal lines, and trailing whitespace.
  for (int index = 0; index < lines.length; index += 1) {
    lines[index] = lines[index].trimRight();
    if (fullDecorations.matchAsPrefix(lines[index]) != null) {
      if (index == 0 || lines[index].length != lines[index - 1].length) {
        // (we leave the decorations if it's just underlining the previous line)
        lines[index] = '';
      }
    }
  }
  // Remove leading and trailing blank lines
  while (lines.isNotEmpty && lines.first == '') {
    lines.removeAt(0);
  }
  while (lines.isNotEmpty && lines.last == '') {
    lines.removeLast();
  }
}

bool _stripIndentation(List<String> lines) {
  // Try stripping leading indentation.
  String? prefix;
  bool removeTrailingBlockEnd = false;
  if (lines.first.startsWith('/*') && lines.last.startsWith(' *')) {
    // In addition to the leadingDecorations, we also support one specific
    // kind of multiline decoration, the /*...*/ block comment.
    prefix = ' *';
    removeTrailingBlockEnd = true;
  } else {
    prefix = leadingDecorations.matchAsPrefix(lines.first)?.group(0);
  }
  if (prefix != null &&
      lines.skip(1).every((String line) => line.startsWith(prefix!) || prefix.startsWith(line))) {
    final int prefixLength = prefix.length;
    for (int index = 0; index < lines.length; index += 1) {
      final String line = lines[index];
      if (line.length > prefixLength) {
        lines[index] = line.substring(prefixLength);
      } else {
        lines[index] = '';
      }
    }
    if (removeTrailingBlockEnd) {
      if (lines.last == '/') {
        // This removes the line with the trailing "*/" when we had a "/*" at the top.
        lines.removeLast();
      }
    }
    return true;
  }
  return false;
}

bool _unindentLeadingParagraphs(List<String> lines) {
  // Try removing leading spaces
  // (we know that this loop terminates before the end of the block because
  // otherwise the previous section would have stripped a common prefix across
  // the entire block)
  //
  // For example, this will change:
  //
  //     foo
  //   bar
  //
  // ...into:
  //
  //   foo
  //   bar
  //
  assert(' '.startsWith(leadingDecorations));
  if (lines.first.startsWith(' ')) {
    int lineCount = 0;
    String line = lines.first;
    int leadingBlockIndent = line.length; // arbitrarily big number
    do {
      int indentWidth = 1;
      while (indentWidth < line.length && line[indentWidth] == ' ') {
        indentWidth += 1;
      }
      if (indentWidth < leadingBlockIndent) {
        leadingBlockIndent = indentWidth;
      }
      lineCount += 1;
      assert(lineCount < lines.length);
      line = lines[lineCount];
    } while (line.startsWith(' '));
    assert(leadingBlockIndent > 0);
    for (int index = 0; index < lineCount; index += 1) {
      lines[index] = lines[index].substring(leadingBlockIndent, lines[index].length);
    }
    return true;
  }
  return false;
}

bool _minorRemovals(List<String> lines) {
  bool didEdits = false;
  // Try removing stray leading spaces (but only one space).
  for (int index = 0; index < lines.length; index += 1) {
    if (lines[index].startsWith(' ') && !lines[index].startsWith('  ')) {
      lines[index] = lines[index].substring(1, lines[index].length);
      didEdits = true;
    }
  }
  // Try stripping HTML leading and trailing block (<!--/-->) comment markers.
  if (lines.first.contains('<!--') || lines.last.contains('-->')) {
    lines.first = lines[0].replaceFirst('<!--', '');
    lines.last = lines[0].replaceFirst('-->', '');
    didEdits = true;
  }
  // Try stripping C-style leading block comment markers.
  // We don't do this earlier because if it's a multiline block comment
  // we want to be careful about stripping the trailing aligned "*/".
  if (lines.first.startsWith('/*')) {
    lines.first = lines.first.substring(2, lines.first.length);
    didEdits = true;
  }
  // Try stripping trailing decorations (specifically, stray "*/"s).
  for (int index = 0; index < lines.length; index += 1) {
    if (lines[index].endsWith('*/')) {
      lines[index] = lines[index].substring(0, lines[index].length - 2);
      didEdits = true;
    }
  }
  return didEdits;
}

/// This function takes a block of text potentially decorated with leading
/// prefix indents, horizontal lines, C-style comment blocks, blank lines, etc,
/// and removes all such incidental material leaving only the significant text.
String reformat(String body) {
  final List<String> lines = body.split('\n');
  while (true) {
    // The order of the following checks is important. If any test changes
    // something that could affect an earlier test, then we should "continue" back
    // to the top of the loop.
    _stripFullLines(lines);
    if (lines.isEmpty) {
      // We've stripped everything, give up.
      return '';
    }
    if (_stripIndentation(lines)) {
      continue; // Go back to top since we may have more blank lines to strip now.
    }
    if (_unindentLeadingParagraphs(lines)) {
      continue; // Go back to the top since we may have more indentation to strip now.
    }
    if (_minorRemovals(lines)) {
      continue; // Go back to the top since we may have new stuff to strip.
    }
    // If we get here, we could not find anything else to do to the text to clean it up.
    break;
  }
  return lines.join('\n');
}

String stripAsciiArt(String input) {
  // Look for images so that we can remove them.
  final List<String> lines = input.split('\n');
  for (final List<String> image in asciiArtImages) {
    assert(image.isNotEmpty);
    // Look for the image starting on each line.
    search:
    for (int index = 0; index < lines.length - image.length; index += 1) {
      final int x = lines[index].indexOf(image[0]);
      if (x >= 0) {
        int width = image[0].length;
        // Found the first line, check to see if we have a complete image.
        for (int imageLine = 1; imageLine < image.length; imageLine += 1) {
          if (lines[index + imageLine].indexOf(image[imageLine]) != x) {
            continue search; // Not a complete image.
          }
          if (image[imageLine].length > width) {
            width = image[imageLine].length;
          }
        }
        // Now remove the image.
        for (int imageLine = 0; imageLine < image.length; imageLine += 1) {
          final String text = lines[index + imageLine];
          assert(text.length > x);
          if (text.length >= x + width) {
            lines[index + imageLine] =
                text.substring(0, x) + text.substring(x + width, text.length);
          } else {
            lines[index + imageLine] = text.substring(0, x);
          }
        }
      }
    }
  }
  return lines.join('\n');
}
