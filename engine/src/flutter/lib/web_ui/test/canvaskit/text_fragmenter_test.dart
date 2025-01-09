// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';
import 'dart:typed_data';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';

import 'common.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  setUpCanvasKitTest();

  group('$fragmentUsingIntlSegmenter', () {
    test('fragments text into words', () {
      final Uint32List breaks = fragmentUsingIntlSegmenter(
        'Hello world 你好世界',
        IntlSegmenterGranularity.word,
      );
      expect(breaks, orderedEquals(<int>[0, 5, 6, 11, 12, 14, 16]));
    });

    test('fragments multi-line text into words', () {
      final Uint32List breaks = fragmentUsingIntlSegmenter(
        'Lorem ipsum\ndolor 你好世界 sit\namet',
        IntlSegmenterGranularity.word,
      );
      expect(
        breaks,
        orderedEquals(<int>[
          0, 5, 6, 11, 12, // "Lorem ipsum\n"
          17, 18, 20, 22, 23, 26, 27, // "dolor 你好世界 sit\n"
          31, // "amet"
        ]),
      );
    });

    test('fragments text into grapheme clusters', () {
      // The smiley emoji has a length of 2.
      // The family emoji has a length of 11.
      final Uint32List breaks = fragmentUsingIntlSegmenter(
        'Lorem🙂ipsum👨‍👩‍👧‍👦',
        IntlSegmenterGranularity.grapheme,
      );
      expect(
        breaks,
        orderedEquals(<int>[
          0, 1, 2, 3, 4, 5, 7, // "Lorem🙂"
          8, 9, 10, 11, 12, 23, // "ipsum👨‍👩‍👧‍👦"
        ]),
      );
    });

    test('fragments multi-line text into grapheme clusters', () {
      // The smiley emojis have a length of 2 each.
      // The family emoji has a length of 11.
      final Uint32List breaks = fragmentUsingIntlSegmenter(
        'Lorem🙂\nipsum👨‍👩‍👧‍👦dolor\n😄',
        IntlSegmenterGranularity.grapheme,
      );
      expect(
        breaks,
        orderedEquals(<int>[
          0, 1, 2, 3, 4, 5, 7, 8, // "Lorem🙂\n"
          9, 10, 11, 12, 13, 24, // "ipsum👨‍👩‍👧‍👦"
          25, 26, 27, 28, 29, 30, 32, // "dolor😄\n"
        ]),
      );
    });
  }, skip: !browserSupportsCanvaskitChromium);

  group('$fragmentUsingV8LineBreaker', () {
    const int kSoft = 0;
    const int kHard = 1;

    test('fragments text into soft and hard line breaks', () {
      final Uint32List breaks = fragmentUsingV8LineBreaker('Lorem-ipsum 你好🙂\nDolor sit');
      expect(
        breaks,
        orderedEquals(<int>[
          0, kSoft,
          6, kSoft, // "Lorem-"
          12, kSoft, // "ipsum "
          13, kSoft, // "你"
          14, kSoft, // "好"
          17, kHard, // "🙂\n"
          23, kSoft, // "Dolor "
          26, kSoft, // "sit"
        ]),
      );
    });
  }, skip: !browserSupportsCanvaskitChromium);

  group('segmentText', () {
    setUp(() {
      segmentationCache.clear();
    });

    tearDown(() {
      segmentationCache.clear();
    });

    test('segments correctly', () {
      const String text = 'Lorem-ipsum 你好🙂\nDolor sit';
      final SegmentationResult segmentation = segmentText(text);
      expect(segmentation.words, fragmentUsingIntlSegmenter(text, IntlSegmenterGranularity.word));
      expect(
        segmentation.graphemes,
        fragmentUsingIntlSegmenter(text, IntlSegmenterGranularity.grapheme),
      );
      expect(segmentation.breaks, fragmentUsingV8LineBreaker(text));
    });

    test('caches segmentation results in LRU fashion', () {
      const String text1 = 'hello';
      segmentText(text1);
      expect(segmentationCache.small.debugItemQueue, hasLength(1));
      expect(segmentationCache.small[text1], isNotNull);

      const String text2 = 'world';
      segmentText(text2);
      expect(segmentationCache.small.debugItemQueue, hasLength(2));
      expect(segmentationCache.small[text2], isNotNull);

      // "world" was segmented last, so it should be first, as in most recently used.
      expect(segmentationCache.small.debugItemQueue.first.key, 'world');
      expect(segmentationCache.small.debugItemQueue.last.key, 'hello');
    });

    test('puts segmentation results in the appropriate cache', () {
      final String smallText = 'a' * (kSmallParagraphCacheSpec.maxTextLength - 1);
      segmentText(smallText);
      expect(segmentationCache.small.debugItemQueue, hasLength(1));
      expect(segmentationCache.medium.debugItemQueue, hasLength(0));
      expect(segmentationCache.large.debugItemQueue, hasLength(0));
      expect(segmentationCache.small[smallText], isNotNull);
      segmentationCache.clear();

      final String mediumText = 'a' * (kMediumParagraphCacheSpec.maxTextLength - 1);
      segmentText(mediumText);
      expect(segmentationCache.small.debugItemQueue, hasLength(0));
      expect(segmentationCache.medium.debugItemQueue, hasLength(1));
      expect(segmentationCache.large.debugItemQueue, hasLength(0));
      expect(segmentationCache.medium[mediumText], isNotNull);
      segmentationCache.clear();

      final String largeText = 'a' * (kLargeParagraphCacheSpec.maxTextLength - 1);
      segmentText(largeText);
      expect(segmentationCache.small.debugItemQueue, hasLength(0));
      expect(segmentationCache.medium.debugItemQueue, hasLength(0));
      expect(segmentationCache.large.debugItemQueue, hasLength(1));
      expect(segmentationCache.large[largeText], isNotNull);
      segmentationCache.clear();

      // Should not cache extremely large texts.
      final String tooLargeText = 'a' * (kLargeParagraphCacheSpec.maxTextLength + 1);
      segmentText(tooLargeText);
      expect(segmentationCache.small.debugItemQueue, hasLength(0));
      expect(segmentationCache.medium.debugItemQueue, hasLength(0));
      expect(segmentationCache.large.debugItemQueue, hasLength(0));
      segmentationCache.clear();
    });

    test('has a limit on the number of entries', () {
      testCacheCapacity(segmentationCache.small, kSmallParagraphCacheSpec);
      testCacheCapacity(segmentationCache.medium, kMediumParagraphCacheSpec);
      testCacheCapacity(segmentationCache.large, kLargeParagraphCacheSpec);
    }, timeout: const Timeout.factor(4));
  }, skip: !browserSupportsCanvaskitChromium);
}

void testCacheCapacity(LruCache<String, SegmentationResult> cache, SegmentationCacheSpec spec) {
  // 1. Fill the cache.
  for (int i = 0; i < spec.cacheSize; i++) {
    final String text = _randomString(spec.maxTextLength);
    segmentText(text);
    // The segmented text should have been added to the cache.
    // TODO(mdebbar): This may fail if the random string generator generates
    //                the same string twice.
    expect(cache.debugItemQueue, hasLength(i + 1));
  }

  // 2. Make sure the cache is full.
  expect(cache.length, spec.cacheSize);

  // 3. Add more items to the cache.
  for (int i = 0; i < 10; i++) {
    final String text = _randomString(spec.maxTextLength);
    segmentText(text);
    // The cache size should remain the same.
    expect(cache.debugItemQueue, hasLength(spec.cacheSize));
  }

  // 4. Clear the cache.
  cache.clear();
}

int _seed = 0;
String _randomString(int length) {
  const String allChars =
      ' 1234567890'
      'abcdefghijklmnopqrstuvwxyz'
      'ABCDEFGHIJKLMNOPQRSTUVWXYZ';

  final String text = '*' * length;
  return text.replaceAllMapped(
    '*',
    // Passing a seed so the results are reproducible.
    (_) => allChars[Random(_seed++).nextInt(allChars.length)],
  );
}
