// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;
import 'dart:ui';

// Used to randomize data.
//
// Using constant seed for reproducibility.
final math.Random _random = math.Random(0);

/// Random words used by benchmarks that contain text.
final List<String> lipsum = 'Lorem ipsum dolor sit amet, consectetur adipiscing '
        'elit. Vivamus ut ligula a neque mattis posuere. Sed suscipit lobortis '
        'sodales. Morbi sed neque molestie, hendrerit odio ac, aliquam velit. '
        'Curabitur non quam sit amet nibh sollicitudin ultrices. Fusce '
        'ullamcorper bibendum commodo. In et feugiat nisl. Aenean vulputate in '
        'odio vestibulum ultricies. Nunc dolor libero, hendrerit eu urna sit '
        'amet, pretium iaculis nulla. Ut porttitor nisl et leo iaculis, vel '
        'fringilla odio pulvinar. Ut eget ligula id odio auctor egestas nec a '
        'nisl. Aliquam luctus dolor et magna posuere mattis. '
        'Suspendisse fringilla nisl et massa congue, eget '
        'imperdiet lectus porta. Vestibulum sed dui sed dui porta imperdiet ut in risus. '
        'Fusce diam purus, faucibus id accumsan sit amet, semper a sem. Sed aliquam '
        'lacus eget libero ultricies, quis hendrerit tortor posuere. Pellentesque '
        'sagittis eu est in maximus. Proin auctor fringilla dolor in hendrerit. Nam '
        'pulvinar rhoncus tellus. Nullam vel mauris semper, volutpat tellus at, sagittis '
        'lectus. Donec vitae nibh mauris. Morbi posuere sem id eros tristique tempus. '
        'Vivamus lacinia sapien neque, eu semper purus gravida ut.'
    .split(' ');

/// Generates strings and builds pre-laid out paragraphs to be used by
/// benchmarks.
List<Paragraph> generateLaidOutParagraphs({
  required int paragraphCount,
  required int minWordCountPerParagraph,
  required int maxWordCountPerParagraph,
  required double widthConstraint,
  required Color color,
}) {
  final List<Paragraph> strings = <Paragraph>[];
  int wordPointer = 0; // points to the next word in lipsum to extract
  for (int i = 0; i < paragraphCount; i++) {
    final int wordCount =
        minWordCountPerParagraph +
        _random.nextInt(maxWordCountPerParagraph - minWordCountPerParagraph + 1);
    final List<String> string = <String>[];
    for (int j = 0; j < wordCount; j++) {
      string.add(lipsum[wordPointer]);
      wordPointer = (wordPointer + 1) % lipsum.length;
    }

    final ParagraphBuilder builder =
        ParagraphBuilder(ParagraphStyle(fontFamily: 'sans-serif'))
          ..pushStyle(TextStyle(color: color, fontSize: 18.0))
          ..addText(string.join(' '))
          ..pop();
    final Paragraph paragraph = builder.build();

    // Fill half the screen.
    paragraph.layout(ParagraphConstraints(width: widthConstraint));
    strings.add(paragraph);
  }
  return strings;
}
