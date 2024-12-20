// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Splits [text] into a list of [TextFragment]s.
///
/// Various subclasses can perform the fragmenting based on their own criteria.
///
/// See:
///
/// - [LineBreakFragmenter]: Fragments text based on line break opportunities.
/// - [BidiFragmenter]: Fragments text based on directionality.
abstract class TextFragmenter {
  const TextFragmenter(this.text);

  /// The text to be fragmented.
  final String text;

  /// Performs the fragmenting of [text] and returns a list of [TextFragment]s.
  List<TextFragment> fragment();
}

/// Represents a fragment produced by [TextFragmenter].
abstract class TextFragment {
  const TextFragment(this.start, this.end);

  final int start;
  final int end;

  /// Whether this fragment's range overlaps with the range from [start] to [end].
  bool overlapsWith(int start, int end) {
    return start < this.end && this.start < end;
  }
}
