// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';
import 'package:ui/src/engine.dart';

abstract class FallbackFontRegistry {
  List<int> getMissingCodePoints(List<int> codePoints, List<String> fontFamilies);
  Future<void> loadFallbackFont(String familyName, String string);
  void updateFallbackFontFamilies(List<String> families);
}

bool _isNotoSansSC(NotoFont font) => font.name.startsWith('Noto Sans SC');
bool _isNotoSansTC(NotoFont font) => font.name.startsWith('Noto Sans TC');
bool _isNotoSansHK(NotoFont font) => font.name.startsWith('Noto Sans HK');
bool _isNotoSansJP(NotoFont font) => font.name.startsWith('Noto Sans JP');
bool _isNotoSansKR(NotoFont font) => font.name.startsWith('Noto Sans KR');

/// Global static font fallback data.
class FontFallbackManager {
  factory FontFallbackManager(FallbackFontRegistry registry) =>
      FontFallbackManager._(registry, getFallbackFontList());

  FontFallbackManager._(this.registry, this.fallbackFonts)
    : _notoSymbols = fallbackFonts.singleWhere(
        (NotoFont font) => font.name == 'Noto Sans Symbols',
      ) {
    downloadQueue = FallbackFontDownloadQueue(this);
  }

  final FallbackFontRegistry registry;

  late final FallbackFontDownloadQueue downloadQueue;

  /// Code points that no known font has a glyph for.
  final Set<int> codePointsWithNoKnownFont = <int>{};

  /// Code points which are known to be covered by at least one fallback font.
  final Set<int> knownCoveredCodePoints = <int>{};

  final List<NotoFont> fallbackFonts;

  // By default, we use the system language to determine the user's preferred
  // language. This can be overridden through [debugUserPreferredLanguage] for testing.
  String _language = domWindow.navigator.language;

  @visibleForTesting
  String get debugUserPreferredLanguage => _language;

  @visibleForTesting
  set debugUserPreferredLanguage(String value) {
    _language = value;
  }

  final NotoFont _notoSymbols;

  Future<void> _idleFuture = Future<void>.value();

  final List<String> globalFontFallbacks = <String>['Roboto'];

  /// A list of code points to check against the global fallback fonts.
  final Set<int> _codePointsToCheckAgainstFallbackFonts = <int>{};

  /// This is [true] if we have scheduled a check for missing code points.
  ///
  /// We only do this once a frame, since checking if a font supports certain
  /// code points is very expensive.
  bool _scheduledCodePointCheck = false;

  Future<void> debugWhenIdle() {
    Future<void>? result;
    assert(() {
      result = _idleFuture;
      return true;
    }());

    if (result != null) {
      return result!;
    }

    throw UnimplementedError();
  }

  /// Determines if the given [text] contains any code points which are not
  /// supported by the current set of fonts.
  void ensureFontsSupportText(String text, List<String> fontFamilies) {
    // TODO(hterkelsen): Make this faster for the common case where the text
    // is supported by the given fonts.
    if (debugDisableFontFallbacks) {
      return;
    }

    // We have a cache of code points which are known to be covered by at least
    // one of our fallback fonts, and a cache of code points which are known not
    // to be covered by any fallback font. From the given text, construct a set
    // of code points which need to be checked.
    final Set<int> runesToCheck = <int>{};
    for (final int rune in text.runes) {
      // Filter out code points that don't need checking.
      if (!(rune < 160 || // ASCII and Unicode control points.
          knownCoveredCodePoints.contains(rune) || // Points we've already covered
          codePointsWithNoKnownFont.contains(rune)) // Points that don't have a fallback font
      ) {
        runesToCheck.add(rune);
      }
    }
    if (runesToCheck.isEmpty) {
      return;
    }

    final List<int> codePoints = runesToCheck.toList();
    final List<int> missingCodePoints = registry.getMissingCodePoints(codePoints, fontFamilies);

    if (missingCodePoints.isNotEmpty) {
      addMissingCodePoints(codePoints);
    }
  }

  void addMissingCodePoints(List<int> codePoints) {
    _codePointsToCheckAgainstFallbackFonts.addAll(codePoints);
    if (!_scheduledCodePointCheck) {
      _scheduledCodePointCheck = true;
      _idleFuture = Future<void>.delayed(Duration.zero, () async {
        _ensureFallbackFonts();
        _scheduledCodePointCheck = false;
        await downloadQueue.waitForIdle();
      });
    }
  }

  /// Checks the missing code points against the current set of fallback fonts
  /// and starts downloading new fallback fonts if the current set can't cover
  /// the code points.
  void _ensureFallbackFonts() {
    _scheduledCodePointCheck = false;
    // We don't know if the remaining code points are covered by our fallback
    // fonts. Check them and update the cache.
    if (_codePointsToCheckAgainstFallbackFonts.isEmpty) {
      return;
    }
    final List<int> codePoints = _codePointsToCheckAgainstFallbackFonts.toList();
    _codePointsToCheckAgainstFallbackFonts.clear();
    findFontsForMissingCodePoints(codePoints);
  }

  void registerFallbackFont(String family) {
    // Insert emoji font before all other fallback fonts so we use the emoji
    // whenever it's available.
    if (family.startsWith('Noto Color Emoji') || family == 'Noto Emoji') {
      if (globalFontFallbacks.first == 'Roboto') {
        globalFontFallbacks.insert(1, family);
      } else {
        globalFontFallbacks.insert(0, family);
      }
    } else {
      globalFontFallbacks.add(family);
    }
  }

  /// Finds the minimum set of fonts which covers all of the [codePoints].
  ///
  /// Since set cover is NP-complete, we approximate using a greedy algorithm
  /// which finds the font which covers the most code points. If multiple CJK
  /// fonts match the same number of code points, we choose one based on the
  /// user's locale.
  ///
  /// If a code point is not covered by any font, it is added to
  /// [codePointsWithNoKnownFont] so it can be omitted next time to avoid
  /// searching for fonts unnecessarily.
  void findFontsForMissingCodePoints(List<int> codePoints) {
    final List<int> missingCodePoints = <int>[];

    final List<FallbackFontComponent> requiredComponents = <FallbackFontComponent>[];
    final List<NotoFont> candidateFonts = <NotoFont>[];

    // Collect the components that cover the code points.
    for (final int codePoint in codePoints) {
      final FallbackFontComponent component = codePointToComponents.lookup(codePoint);
      if (component.fonts.isEmpty) {
        missingCodePoints.add(codePoint);
      } else {
        // A zero cover count means we have not yet seen this component.
        if (component.coverCount == 0) {
          requiredComponents.add(component);
        }
        component.coverCount++;
      }
    }

    // Aggregate the component cover counts to the fonts that use the component.
    for (final FallbackFontComponent component in requiredComponents) {
      for (final NotoFont font in component.fonts) {
        // A zero cover cover count means we have not yet seen this font.
        if (font.coverCount == 0) {
          candidateFonts.add(font);
        }
        font.coverCount += component.coverCount;
        font.coverComponents.add(component);
      }
    }

    final List<NotoFont> selectedFonts = <NotoFont>[];

    while (candidateFonts.isNotEmpty) {
      final NotoFont selectedFont = _selectFont(candidateFonts);
      selectedFonts.add(selectedFont);

      // All the code points in the selected font are now covered. Zero out each
      // component that is used by the font and adjust the counts of other fonts
      // that use the same components.
      for (final FallbackFontComponent component in <FallbackFontComponent>[
        ...selectedFont.coverComponents,
      ]) {
        for (final NotoFont font in component.fonts) {
          font.coverCount -= component.coverCount;
          font.coverComponents.remove(component);
        }
        component.coverCount = 0;
      }
      assert(selectedFont.coverCount == 0);
      assert(selectedFont.coverComponents.isEmpty);
      // The selected font will have a zero cover count, but other fonts may
      // too. Remove these from further consideration.
      candidateFonts.removeWhere((NotoFont font) => font.coverCount == 0);
    }

    selectedFonts.forEach(downloadQueue.add);

    // Report code points not covered by any fallback font and ensure we don't
    // process those code points again.
    if (missingCodePoints.isNotEmpty) {
      if (!downloadQueue.isPending) {
        printWarning(
          'Could not find a set of Noto fonts to display all missing '
          'characters. Please add a font asset for the missing characters.'
          ' See: https://flutter.dev/docs/cookbook/design/fonts',
        );
        codePointsWithNoKnownFont.addAll(missingCodePoints);
      }
    }
  }

  NotoFont _selectFont(List<NotoFont> fonts) {
    int maxCodePointsCovered = -1;
    final List<NotoFont> bestFonts = <NotoFont>[];
    NotoFont? bestFont;

    for (final NotoFont font in fonts) {
      if (font.coverCount > maxCodePointsCovered) {
        bestFonts.clear();
        bestFonts.add(font);
        bestFont = font;
        maxCodePointsCovered = font.coverCount;
      } else if (font.coverCount == maxCodePointsCovered) {
        bestFonts.add(font);
        // Tie-break with the lowest index which corresponds to a font name
        // being earlier in the list of fonts in the font fallback data
        // generator.
        if (font.index < bestFont!.index) {
          bestFont = font;
        }
      }
    }

    NotoFont? bestFontForLanguage;
    if (bestFonts.length > 1) {
      // If the list of best fonts are all CJK fonts, choose the best one based
      // on user preferred language. Otherwise just choose the first font.
      if (bestFonts.every(
        (NotoFont font) =>
            _isNotoSansSC(font) ||
            _isNotoSansTC(font) ||
            _isNotoSansHK(font) ||
            _isNotoSansJP(font) ||
            _isNotoSansKR(font),
      )) {
        if (_language == 'zh-Hans' ||
            _language == 'zh-CN' ||
            _language == 'zh-SG' ||
            _language == 'zh-MY') {
          bestFontForLanguage = bestFonts.firstWhereOrNull(_isNotoSansSC);
        } else if (_language == 'zh-Hant' || _language == 'zh-TW' || _language == 'zh-MO') {
          bestFontForLanguage = bestFonts.firstWhereOrNull(_isNotoSansTC);
        } else if (_language == 'zh-HK') {
          bestFontForLanguage = bestFonts.firstWhereOrNull(_isNotoSansHK);
        } else if (_language == 'ja') {
          bestFontForLanguage = bestFonts.firstWhereOrNull(_isNotoSansJP);
        } else if (_language == 'ko') {
          bestFontForLanguage = bestFonts.firstWhereOrNull(_isNotoSansKR);
        } else {
          // Default to `Noto Sans SC` when the user preferred language is not CJK.
          bestFontForLanguage = bestFonts.firstWhereOrNull(_isNotoSansSC);
        }
      } else {
        // To be predictable, if there is a tie for best font, choose a font
        // from this list first, then just choose the first font.
        if (bestFonts.contains(_notoSymbols)) {
          bestFont = _notoSymbols;
        } else {
          final notoSansSC = bestFonts.firstWhereOrNull(_isNotoSansSC);
          if (notoSansSC != null) {
            bestFont = notoSansSC;
          }
        }
      }
    }
    return bestFontForLanguage ?? bestFont!;
  }

  late final List<FallbackFontComponent> fontComponents = _decodeFontComponents(encodedFontSets);

  late final _UnicodePropertyLookup<FallbackFontComponent> codePointToComponents =
      _UnicodePropertyLookup<FallbackFontComponent>.fromPackedData(
        encodedFontSetRanges,
        fontComponents,
      );

  List<FallbackFontComponent> _decodeFontComponents(String data) {
    return <FallbackFontComponent>[
      for (final String componentData in data.split(','))
        FallbackFontComponent(_decodeFontSet(componentData)),
    ];
  }

  List<NotoFont> _decodeFontSet(String data) {
    final List<NotoFont> result = <NotoFont>[];
    int previousIndex = -1;
    int prefix = 0;
    for (int i = 0; i < data.length; i++) {
      final int code = data.codeUnitAt(i);

      if (kFontIndexDigit0 <= code && code < kFontIndexDigit0 + kFontIndexRadix) {
        final int delta = prefix * kFontIndexRadix + (code - kFontIndexDigit0);
        final int index = previousIndex + delta + 1;
        result.add(fallbackFonts[index]);
        previousIndex = index;
        prefix = 0;
      } else if (kPrefixDigit0 <= code && code < kPrefixDigit0 + kPrefixRadix) {
        prefix = prefix * kPrefixRadix + (code - kPrefixDigit0);
      } else {
        throw StateError('Unreachable');
      }
    }
    return result;
  }
}

/// A lookup structure from code point to a property type [P].
class _UnicodePropertyLookup<P> {
  _UnicodePropertyLookup._(this._boundaries, this._values);

  factory _UnicodePropertyLookup.fromPackedData(String packedData, List<P> propertyEnumValues) {
    final List<int> boundaries = <int>[];
    final List<P> values = <P>[];

    int start = 0;
    int prefix = 0;
    int size = 1;

    for (int i = 0; i < packedData.length; i++) {
      final int code = packedData.codeUnitAt(i);
      if (kRangeValueDigit0 <= code && code < kRangeValueDigit0 + kRangeValueRadix) {
        final int index = prefix * kRangeValueRadix + (code - kRangeValueDigit0);
        final P value = propertyEnumValues[index];
        start += size;
        boundaries.add(start);
        values.add(value);
        prefix = 0;
        size = 1;
      } else if (kRangeSizeDigit0 <= code && code < kRangeSizeDigit0 + kRangeSizeRadix) {
        size = prefix * kRangeSizeRadix + (code - kRangeSizeDigit0) + 2;
        prefix = 0;
      } else if (kPrefixDigit0 <= code && code < kPrefixDigit0 + kPrefixRadix) {
        prefix = prefix * kPrefixRadix + (code - kPrefixDigit0);
      } else {
        throw StateError('Unreachable');
      }
    }
    if (start != kMaxCodePoint + 1) {
      throw StateError('Bad map size: $start');
    }

    return _UnicodePropertyLookup<P>._(boundaries, values);
  }

  /// There are two parallel lists - one of boundaries between adjacent unicode
  /// ranges and second of the values for the ranges.
  ///
  /// `_boundaries[i]` is the open-interval end of the `i`th range and the start
  /// of the `i+1`th range. The implicit start of the 0th range is zero.
  ///
  /// `_values[i]` is the value for the range [`_boundaries[i-1]`, `_boundaries[i]`).
  /// Default values are stored as explicit ranges.
  ///
  /// Example: the unicode range properies `[10-50]=>A`, `[100]=>B`, with
  /// default value `X` would be represented as:
  ///
  ///     boundaries:  [10, 51, 100, 101, 1114112]
  ///     values:      [ X,  A,   X,   B,       X]
  ///
  final List<int> _boundaries;
  final List<P> _values;

  int get length => _boundaries.length;

  P lookup(int value) {
    assert(0 <= value && value <= kMaxCodePoint);
    assert(_boundaries.last == kMaxCodePoint + 1);
    int start = 0, end = _boundaries.length;
    while (true) {
      if (start == end) {
        return _values[start];
      }
      final int mid = start + (end - start) ~/ 2;
      if (value >= _boundaries[mid]) {
        start = mid + 1;
      } else {
        end = mid;
      }
    }
  }

  /// Iterate over the ranges, calling [action] with the start and end
  /// (inclusive) code points and value.
  void forEachRange(void Function(int start, int end, P value) action) {
    int start = 0;
    for (int i = 0; i < _boundaries.length; i++) {
      final int end = _boundaries[i];
      final P value = _values[i];
      action(start, end - 1, value);
      start = end;
    }
  }
}

class FallbackFontDownloadQueue {
  FallbackFontDownloadQueue(this.fallbackManager);

  final FontFallbackManager fallbackManager;

  String get fallbackFontUrlPrefix => configuration.fontFallbackBaseUrl;

  final Set<NotoFont> downloadedFonts = <NotoFont>{};
  final Map<String, NotoFont> pendingFonts = <String, NotoFont>{};

  bool get isPending => pendingFonts.isNotEmpty;

  void Function(String family)? debugOnLoadFontFamily;

  Completer<void>? _idleCompleter;

  Future<void> waitForIdle() {
    if (_idleCompleter == null) {
      // We're already idle
      return Future<void>.value();
    } else {
      return _idleCompleter!.future;
    }
  }

  void add(NotoFont font) {
    if (downloadedFonts.contains(font) || pendingFonts.containsKey(font.url)) {
      return;
    }
    final bool firstInBatch = pendingFonts.isEmpty;
    pendingFonts[font.url] = font;
    _idleCompleter ??= Completer<void>();
    if (firstInBatch) {
      Timer.run(startDownloads);
    }
  }

  Future<void> startDownloads() async {
    final Map<String, Future<void>> downloads = <String, Future<void>>{};
    final List<String> downloadedFontFamilies = <String>[];
    for (final NotoFont font in pendingFonts.values) {
      downloads[font.url] = Future<void>(() async {
        try {
          final String url = '$fallbackFontUrlPrefix${font.url}';
          debugOnLoadFontFamily?.call(font.name);
          await fallbackManager.registry.loadFallbackFont(font.name, url);
          downloadedFontFamilies.add(font.url);
        } catch (e) {
          pendingFonts.remove(font.url);
          printWarning(
            'Failed to load font ${font.name} at '
            '$fallbackFontUrlPrefix${font.url}',
          );
          printWarning(e.toString());
          return;
        }
        downloadedFonts.add(font);
      });
    }

    await Future.wait<void>(downloads.values);

    // Register fallback fonts in a predictable order. Otherwise, the fonts
    // change their precedence depending on the download order causing
    // visual differences between app reloads.
    downloadedFontFamilies.sort();
    for (final String url in downloadedFontFamilies) {
      final NotoFont font = pendingFonts.remove(url)!;
      fallbackManager.registerFallbackFont(font.name);
    }

    if (pendingFonts.isEmpty) {
      fallbackManager.registry.updateFallbackFontFamilies(fallbackManager.globalFontFallbacks);
      sendFontChangeMessage();
      final Completer<void> idleCompleter = _idleCompleter!;
      _idleCompleter = null;
      idleCompleter.complete();
    } else {
      await startDownloads();
    }
  }
}
