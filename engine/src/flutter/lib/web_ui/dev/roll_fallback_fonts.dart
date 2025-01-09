// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show ByteConversionSink, jsonDecode, utf8;
import 'dart:io' as io;
import 'dart:typed_data';

import 'package:args/command_runner.dart';
import 'package:convert/convert.dart';
import 'package:crypto/crypto.dart' as crypto;
import 'package:http/http.dart' as http;
import 'package:path/path.dart' as path;

// ignore: avoid_relative_lib_imports
import '../lib/src/engine/noto_font_encoding.dart';

import 'cipd.dart';
import 'environment.dart';
import 'exceptions.dart';
import 'utils.dart';

const String expectedUrlPrefix = 'https://fonts.gstatic.com/s/';

class RollFallbackFontsCommand extends Command<bool> with ArgUtils<bool> {
  RollFallbackFontsCommand() {
    argParser.addOption(
      'key',
      defaultsTo: '',
      help:
          'The Google Fonts API key. Used to get data about fonts hosted on '
          'Google Fonts.',
    );
    argParser.addFlag(
      'dry-run',
      help:
          'Whether or not to push changes to CIPD. When --dry-run is set, the '
          'script will download everything and attempt to prepare the bundle '
          'but will stop before publishing. When not set, the bundle will be '
          'published.',
      negatable: false,
    );
  }

  @override
  final String name = 'roll-fallback-fonts';

  @override
  final String description =
      'Generate fallback font data from GoogleFonts and '
      'upload fonts to cipd.';

  String get apiKey => stringArg('key');
  bool get isDryRun => boolArg('dry-run');

  @override
  Future<bool> run() async {
    await _generateFallbackFontData();
    return true;
  }

  Future<void> _generateFallbackFontData() async {
    final http.Client client = http.Client();
    final List<_FontInfo> fallbackFontInfo = <_FontInfo>[
      ...await _processSplitFallbackFonts(client, splitFallbackFonts),
      ...await _processFallbackFonts(client, apiFallbackFonts),
    ];

    final List<String> failedUrls = await _checkForLicenseAttributions(client, fallbackFontInfo);
    if (failedUrls.isNotEmpty) {
      throw ToolExit('Could not find license attribution at:\n - ${failedUrls.join('\n - ')}');
    }

    final List<_Font> fallbackFontData = <_Font>[];

    final Map<String, String> charsetForFamily = <String, String>{};
    final io.Directory fontDir = await io.Directory.systemTemp.createTemp('flutter_fallback_fonts');
    print('Downloading fonts into temp directory: ${fontDir.path}');
    final AccumulatorSink<crypto.Digest> hashSink = AccumulatorSink<crypto.Digest>();
    final ByteConversionSink hasher = crypto.sha256.startChunkedConversion(hashSink);

    for (final (:family, :uri) in fallbackFontInfo) {
      print('Downloading $family...');
      final http.Response fontResponse = await client.get(uri);
      if (fontResponse.statusCode != 200) {
        throw ToolExit('Failed to download font for $family');
      }
      final String urlSuffix = getUrlSuffix(uri);
      final io.File fontFile = io.File(path.join(fontDir.path, urlSuffix));

      final Uint8List bodyBytes = fontResponse.bodyBytes;
      hasher.add(utf8.encode(urlSuffix));
      hasher.add(bodyBytes);

      await fontFile.create(recursive: true);
      await fontFile.writeAsBytes(bodyBytes, flush: true);
      final io.ProcessResult fcQueryResult = await io.Process.run('fc-query', <String>[
        '--format=%{charset}',
        '--',
        fontFile.path,
      ]);
      final String encodedCharset = fcQueryResult.stdout as String;
      charsetForFamily[family] = encodedCharset;
    }

    final StringBuffer sb = StringBuffer();

    int index = 0;
    for (final _FontInfo fontInfo in fallbackFontInfo) {
      final List<int> starts = <int>[];
      final List<int> ends = <int>[];
      final String charset = charsetForFamily[fontInfo.family]!;
      for (final String range in charset.split(' ')) {
        // Range is one hexadecimal number or two, separated by `-`.
        final List<String> parts = range.split('-');
        if (parts.length != 1 && parts.length != 2) {
          throw ToolExit('Malformed charset range "$range"');
        }
        final int first = int.parse(parts.first, radix: 16);
        final int last = int.parse(parts.last, radix: 16);
        starts.add(first);
        ends.add(last);
      }

      fallbackFontData.add(_Font(fontInfo, index++, starts, ends));
    }

    final String fontSetsCode = _computeEncodedFontSets(fallbackFontData);

    sb.writeln('// Copyright 2013 The Flutter Authors. All rights reserved.');
    sb.writeln(
      '// Use of this source code is governed by a BSD-style license '
      'that can be',
    );
    sb.writeln('// found in the LICENSE file.');
    sb.writeln();
    sb.writeln('// DO NOT EDIT! This file is generated. See:');
    sb.writeln('// dev/roll_fallback_fonts.dart');
    sb.writeln("import 'noto_font.dart';");
    sb.writeln();
    sb.writeln('List<NotoFont> getFallbackFontList() => <NotoFont>[');

    for (final _Font font in fallbackFontData) {
      final String family = font.info.family;
      final String urlSuffix = getUrlSuffix(font.info.uri);
      sb.writeln(" NotoFont('$family', '$urlSuffix'),");
    }
    sb.writeln('];');
    sb.writeln();
    sb.write(fontSetsCode);

    final io.File fontDataFile = io.File(
      path.join(environment.webUiRootDir.path, 'lib', 'src', 'engine', 'font_fallback_data.dart'),
    );
    await fontDataFile.writeAsString(sb.toString());

    final io.File licenseFile = io.File(path.join(fontDir.path, 'LICENSE.txt'));
    const String licenseString = r'''
© Copyright 2015-2021 Google LLC. All Rights Reserved.

This Font Software is licensed under the SIL Open Font License, Version 1.1.
This license is copied below, and is also available with a FAQ at:
http://scripts.sil.org/OFL


-----------------------------------------------------------
SIL OPEN FONT LICENSE Version 1.1 - 26 February 2007
-----------------------------------------------------------

PREAMBLE
The goals of the Open Font License (OFL) are to stimulate worldwide
development of collaborative font projects, to support the font creation
efforts of academic and linguistic communities, and to provide a free and
open framework in which fonts may be shared and improved in partnership
with others.

The OFL allows the licensed fonts to be used, studied, modified and
redistributed freely as long as they are not sold by themselves. The
fonts, including any derivative works, can be bundled, embedded,
redistributed and/or sold with any software provided that any reserved
names are not used by derivative works. The fonts and derivatives,
however, cannot be released under any other type of license. The
requirement for fonts to remain under this license does not apply
to any document created using the fonts or their derivatives.

DEFINITIONS
"Font Software" refers to the set of files released by the Copyright
Holder(s) under this license and clearly marked as such. This may
include source files, build scripts and documentation.

"Reserved Font Name" refers to any names specified as such after the
copyright statement(s).

"Original Version" refers to the collection of Font Software components as
distributed by the Copyright Holder(s).

"Modified Version" refers to any derivative made by adding to, deleting,
or substituting -- in part or in whole -- any of the components of the
Original Version, by changing formats or by porting the Font Software to a
new environment.

"Author" refers to any designer, engineer, programmer, technical
writer or other person who contributed to the Font Software.

PERMISSION & CONDITIONS
Permission is hereby granted, free of charge, to any person obtaining
a copy of the Font Software, to use, study, copy, merge, embed, modify,
redistribute, and sell modified and unmodified copies of the Font
Software, subject to the following conditions:

1) Neither the Font Software nor any of its individual components,
in Original or Modified Versions, may be sold by itself.

2) Original or Modified Versions of the Font Software may be bundled,
redistributed and/or sold with any software, provided that each copy
contains the above copyright notice and this license. These can be
included either as stand-alone text files, human-readable headers or
in the appropriate machine-readable metadata fields within text or
binary files as long as those fields can be easily viewed by the user.

3) No Modified Version of the Font Software may use the Reserved Font
Name(s) unless explicit written permission is granted by the corresponding
Copyright Holder. This restriction only applies to the primary font name as
presented to the users.

4) The name(s) of the Copyright Holder(s) or the Author(s) of the Font
Software shall not be used to promote, endorse or advertise any
Modified Version, except to acknowledge the contribution(s) of the
Copyright Holder(s) and the Author(s) or with their explicit written
permission.

5) The Font Software, modified or unmodified, in part or in whole,
must be distributed entirely under this license, and must not be
distributed under any other license. The requirement for fonts to
remain under this license does not apply to any document created
using the Font Software.

TERMINATION
This license becomes null and void if any of the above conditions are
not met.

DISCLAIMER
THE FONT SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO ANY WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT
OF COPYRIGHT, PATENT, TRADEMARK, OR OTHER RIGHT. IN NO EVENT SHALL THE
COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
INCLUDING ANY GENERAL, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL
DAMAGES, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF THE USE OR INABILITY TO USE THE FONT SOFTWARE OR FROM
OTHER DEALINGS IN THE FONT SOFTWARE.
''';
    final List<int> licenseData = utf8.encode(licenseString);
    await licenseFile.create(recursive: true);
    await licenseFile.writeAsBytes(licenseData);
    hasher.add(licenseData);
    hasher.close();

    final crypto.Digest digest = hashSink.events.single;
    final String versionString = digest.toString();
    const String packageName = 'flutter/flutter_font_fallbacks';
    if (await cipdKnowsPackageVersion(package: packageName, versionTag: versionString)) {
      print('Package already exists with hash $versionString. Skipping upload');
    } else {
      print('Uploading fallback fonts to CIPD with hash $versionString');
      await uploadDirectoryToCipd(
        directory: fontDir,
        packageName: packageName,
        configFileName: 'cipd.flutter_font_fallbacks.yaml',
        description: 'A set of Noto fonts to fall back to for use in testing.',
        root: fontDir.path,
        version: versionString,
        isDryRun: isDryRun,
      );
    }

    print('Setting new fallback fonts deps version to $versionString');
    final String depFilePath = path.join(environment.engineSrcDir.path, 'flutter', 'DEPS');
    await runProcess('gclient', <String>[
      'setdep',
      '--revision=src/flutter/third_party/google_fonts_for_unit_tests:$packageName@$versionString',
      '--deps-file=$depFilePath',
    ]);
  }

  Future<List<_FontInfo>> _processFallbackFonts(
    http.Client client,
    List<String> requestedFonts,
  ) async {
    if (apiKey.isEmpty) {
      throw UsageException('No Google Fonts API key provided', argParser.usage);
    }
    final List<_FontInfo> processedFonts = <_FontInfo>[];
    final http.Response response = await client.get(
      Uri.parse('https://www.googleapis.com/webfonts/v1/webfonts?capability=WOFF2&key=$apiKey'),
    );
    if (response.statusCode != 200) {
      throw ToolExit('Failed to download Google Fonts list.');
    }
    final Map<String, dynamic> googleFontsResult =
        jsonDecode(response.body) as Map<String, dynamic>;
    final List<Map<String, dynamic>> fontDatas =
        (googleFontsResult['items'] as List<dynamic>).cast<Map<String, dynamic>>();
    for (final Map<String, Object?> fontData in fontDatas) {
      final String family = fontData['family']! as String;
      if (requestedFonts.contains(family)) {
        final files = fontData['files']! as Map<String, Object?>;
        final Uri uri = Uri.parse(files['regular']! as String).replace(scheme: 'https');
        processedFonts.add((family: family, uri: uri));
      }
    }
    return processedFonts;
  }

  Future<List<_FontInfo>> _processSplitFallbackFonts(
    http.Client client,
    List<String> requestedFonts,
  ) async {
    final List<_FontInfo> processedFonts = <_FontInfo>[];
    for (final String font in requestedFonts) {
      final String modifiedFontName = font.replaceAll(' ', '+');
      final Uri cssUri = Uri.parse('https://fonts.googleapis.com/css2?family=$modifiedFontName');
      final http.Response response = await client.get(
        cssUri,
        headers: <String, String>{
          // Spoof the User-Agent so Google Fonts serves WOFF2 fonts
          'User-Agent':
              'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) '
              'Chrome/112.0.0.0 Safari/537.36',
        },
      );
      final String cssString = response.body;
      // Match the patterns that look like:
      // `src: url(...some url...)`
      final r = RegExp(r'src:\s*url\((https?://[^)]+?\.woff2)\)');
      int familyCount = 0;
      // Give each font shard a unique family name.
      for (final match in r.allMatches(cssString)) {
        final String family = '$font $familyCount';
        final Uri uri = Uri.parse(match.group(1)!);
        processedFonts.add((family: family, uri: uri));
        familyCount += 1;
      }
    }
    return processedFonts;
  }
}

/// Fonts that should be downloaded directly from the Google Fonts API.
const List<String> apiFallbackFonts = <String>[
  'Noto Sans',
  'Noto Music',
  'Noto Sans Symbols',
  'Noto Sans Adlam',
  'Noto Sans Anatolian Hieroglyphs',
  'Noto Sans Arabic',
  'Noto Sans Armenian',
  'Noto Sans Avestan',
  'Noto Sans Balinese',
  'Noto Sans Bamum',
  'Noto Sans Bassa Vah',
  'Noto Sans Batak',
  'Noto Sans Bengali',
  'Noto Sans Bhaiksuki',
  'Noto Sans Brahmi',
  'Noto Sans Buginese',
  'Noto Sans Buhid',
  'Noto Sans Canadian Aboriginal',
  'Noto Sans Carian',
  'Noto Sans Caucasian Albanian',
  'Noto Sans Chakma',
  'Noto Sans Cham',
  'Noto Sans Cherokee',
  'Noto Sans Coptic',
  'Noto Sans Cypriot',
  'Noto Sans Deseret',
  'Noto Sans Devanagari',
  'Noto Sans Elbasan',
  'Noto Sans Elymaic',
  'Noto Sans Ethiopic',
  'Noto Sans Georgian',
  'Noto Sans Glagolitic',
  'Noto Sans Gothic',
  'Noto Sans Grantha',
  'Noto Sans Gujarati',
  'Noto Sans Gunjala Gondi',
  'Noto Sans Gurmukhi',
  'Noto Sans Hanunoo',
  'Noto Sans Hatran',
  'Noto Sans Hebrew',
  'Noto Sans Imperial Aramaic',
  'Noto Sans Indic Siyaq Numbers',
  'Noto Sans Inscriptional Pahlavi',
  'Noto Sans Inscriptional Parthian',
  'Noto Sans Javanese',
  'Noto Sans Kaithi',
  'Noto Sans Kannada',
  'Noto Sans Kayah Li',
  'Noto Sans Kharoshthi',
  'Noto Sans Khmer',
  'Noto Sans Khojki',
  'Noto Sans Khudawadi',
  'Noto Sans Lao',
  'Noto Sans Lepcha',
  'Noto Sans Limbu',
  'Noto Sans Linear A',
  'Noto Sans Linear B',
  'Noto Sans Lisu',
  'Noto Sans Lycian',
  'Noto Sans Lydian',
  'Noto Sans Mahajani',
  'Noto Sans Malayalam',
  'Noto Sans Mandaic',
  'Noto Sans Manichaean',
  'Noto Sans Marchen',
  'Noto Sans Masaram Gondi',
  'Noto Sans Math',
  'Noto Sans Mayan Numerals',
  'Noto Sans Medefaidrin',
  'Noto Sans Meetei Mayek',
  'Noto Sans Meroitic',
  'Noto Sans Miao',
  'Noto Sans Modi',
  'Noto Sans Mongolian',
  'Noto Sans Mro',
  'Noto Sans Multani',
  'Noto Sans Myanmar',
  'Noto Sans NKo',
  'Noto Sans Nabataean',
  'Noto Sans New Tai Lue',
  'Noto Sans Newa',
  'Noto Sans Nushu',
  'Noto Sans Ogham',
  'Noto Sans Ol Chiki',
  'Noto Sans Old Hungarian',
  'Noto Sans Old Italic',
  'Noto Sans Old North Arabian',
  'Noto Sans Old Permic',
  'Noto Sans Old Persian',
  'Noto Sans Old Sogdian',
  'Noto Sans Old South Arabian',
  'Noto Sans Old Turkic',
  'Noto Sans Oriya',
  'Noto Sans Osage',
  'Noto Sans Osmanya',
  'Noto Sans Pahawh Hmong',
  'Noto Sans Palmyrene',
  'Noto Sans Pau Cin Hau',
  'Noto Sans Phags Pa',
  'Noto Sans Phoenician',
  'Noto Sans Psalter Pahlavi',
  'Noto Sans Rejang',
  'Noto Sans Runic',
  'Noto Sans Saurashtra',
  'Noto Sans Sharada',
  'Noto Sans Shavian',
  'Noto Sans Siddham',
  'Noto Sans Sinhala',
  'Noto Sans Sogdian',
  'Noto Sans Sora Sompeng',
  'Noto Sans Soyombo',
  'Noto Sans Sundanese',
  'Noto Sans Syloti Nagri',
  'Noto Sans Syriac',
  'Noto Sans Tagalog',
  'Noto Sans Tagbanwa',
  'Noto Sans Tai Le',
  'Noto Sans Tai Tham',
  'Noto Sans Tai Viet',
  'Noto Sans Takri',
  'Noto Sans Tamil',
  'Noto Sans Tamil Supplement',
  'Noto Sans Telugu',
  'Noto Sans Thaana',
  'Noto Sans Thai',
  'Noto Sans Tifinagh',
  'Noto Sans Tirhuta',
  'Noto Sans Ugaritic',
  'Noto Sans Vai',
  'Noto Sans Wancho',
  'Noto Sans Warang Citi',
  'Noto Sans Yi',
  'Noto Sans Zanabazar Square',
  'Noto Serif Tibetan',
];

/// Fonts which are split up into several smaller subfonts. These need special
/// handling.
const List<String> splitFallbackFonts = <String>[
  'Noto Color Emoji',
  'Noto Sans Symbols 2',
  'Noto Sans Cuneiform',
  'Noto Sans Duployan',
  'Noto Sans Egyptian Hieroglyphs',
  'Noto Sans HK',
  'Noto Sans JP',
  'Noto Sans KR',
  'Noto Sans SC',
  'Noto Sans TC',
];

String getUrlSuffix(Uri fontUri) {
  final String urlString = fontUri.toString();
  if (!urlString.startsWith(expectedUrlPrefix)) {
    throw ToolExit('Unexpected url format received from Google Fonts API: $urlString.');
  }
  return urlString.substring(expectedUrlPrefix.length);
}

/// Checks the license attribution for unique fonts in the [fallbackFontInfo]
/// list.
///
/// Returns a list of URLs that failed to contain the license attribution.
Future<List<String>> _checkForLicenseAttributions(
  http.Client client,
  List<_FontInfo> fallbackFontInfo,
) async {
  const String googleFontsUpstream = 'https://github.com/google/fonts/tree/main/ofl';
  const String attributionString =
      'This Font Software is licensed under the SIL Open Font License, Version 1.1.';

  final failedUrls = <String>[];

  final uniqueFontPackageNames = <String>{};
  for (final (family: _, :uri) in fallbackFontInfo) {
    final String urlSuffix = getUrlSuffix(uri);
    final String fontPackageName = urlSuffix.split('/').first;
    uniqueFontPackageNames.add(fontPackageName);
  }

  for (final String fontPackageName in uniqueFontPackageNames) {
    final String fontLicenseUrl = '$googleFontsUpstream/$fontPackageName/OFL.txt';
    final http.Response response = await client.get(Uri.parse(fontLicenseUrl));
    if (response.statusCode != 200) {
      failedUrls.add(fontLicenseUrl);
      continue;
    }
    final String licenseString = response.body;
    if (!licenseString.contains(attributionString)) {
      failedUrls.add(fontLicenseUrl);
    }
  }

  return failedUrls;
}

// The basic information of a font: its [family] (name) and [uri].
typedef _FontInfo = ({String family, Uri uri});

class _Font {
  _Font(this.info, this.index, this.starts, this.ends);

  final _FontInfo info;
  final int index;
  final List<int> starts;
  final List<int> ends; // inclusive ends

  static int compare(_Font a, _Font b) => a.index.compareTo(b.index);

  String get shortName =>
      _shortName + String.fromCharCodes('$index'.codeUnits.map((int ch) => ch - 48 + 0x2080));

  String get _shortName =>
      info.family.startsWith('Noto Sans ')
          ? info.family.substring('Noto Sans '.length)
          : info.family;
}

/// The boundary of a range of a font.
class _Boundary {
  _Boundary(this.value, this.isStart, this.font);
  final int value; // inclusive start or exclusive end.
  final bool isStart;
  final _Font font;

  static int compare(_Boundary a, _Boundary b) => a.value.compareTo(b.value);
}

class _Range {
  _Range(this.start, this.end, this.fontSet);
  final int start;
  final int end;
  final _FontSet fontSet;

  @override
  String toString() {
    return '[${start.toRadixString(16)}, ${end.toRadixString(16)}]'
        ' (${end - start + 1})'
        ' ${fontSet.description()}';
  }
}

/// A canonical representative for a set of _Fonts. The fonts are stored in
/// order of increasing `_Font.index`.
class _FontSet {
  _FontSet(this.fonts);

  /// The number of [_Font]s in this set.
  int get length => fonts.length;

  /// The members of this set.
  final List<_Font> fonts;

  /// Number of unicode ranges that are supported by this set of fonts.
  int rangeCount = 0;

  /// The serialization order of this set. This index is assigned after building
  /// all the sets.
  late final int index;

  static int orderByDecreasingRangeCount(_FontSet a, _FontSet b) {
    final int r = b.rangeCount.compareTo(a.rangeCount);
    if (r != 0) {
      return r;
    }
    return orderByLexicographicFontIndexes(a, b);
  }

  static int orderByLexicographicFontIndexes(_FontSet a, _FontSet b) {
    for (int i = 0; i < a.length && i < b.length; i++) {
      final int r = _Font.compare(a.fonts[i], b.fonts[i]);
      if (r != 0) {
        return r;
      }
    }
    assert(a.length != b.length); // _FontSets are canonical.
    return a.length - b.length;
  }

  @override
  String toString() {
    return description();
  }

  String description() {
    return fonts.map((_Font font) => font.shortName).join(', ');
  }
}

/// A trie node [1] used to find the canonical _FontSet.
///
/// [1]: https://en.wikipedia.org/wiki/Trie
class _TrieNode {
  final Map<_Font, _TrieNode> _children = <_Font, _TrieNode>{};
  _FontSet? fontSet;

  /// Inserts a string of fonts into the trie and returns the trie node
  /// representing the string. [this] must be the root node of the trie.
  ///
  /// Inserting the same sequence again will traverse the same path through the
  /// trie and return the same node, canonicalizing the sequence to its
  /// representative node.
  _TrieNode insertSequenceAtRoot(Iterable<_Font> fonts) {
    _TrieNode node = this;
    for (final _Font font in fonts) {
      node = node._children[font] ??= _TrieNode();
    }
    return node;
  }
}

/// Computes the Dart source code for the encoded data structures used by the
/// fallback font selection algorithm.
///
/// The data structures allow the fallback font selection algorithm to quickly
/// determine which fonts support a given code point. The structures are
/// essentially a map from a code point to a set of fonts that support that code
/// point.
///
/// The universe of code points is partitioned into a set of subsets, or
/// components, where each component contains all the code points that are in
/// exactly the same set of fonts. A font can be considered to be a union of
/// some subset of the components and may share components with other fonts.  A
/// `_FontSet` is used to represent a component and the set of fonts that use
/// the component. One way to visualize this is as a Venn diagram. The fonts are
/// the overlapping circles and the components are the spaces between the lines.
///
/// The emitted data structures are
///
///  (1) A list of sets of fonts.
///  (2) A list of code point ranges mapping to an index of list (1).
///
/// Each set of fonts is represented as a list of font indexes. The indexes are
/// always increasing so the delta is stored. The stored value is biased by -1
/// (i.e.  `delta - 1`) since a delta is never less than 1. The deltas are STMR
/// encoded.
///
/// A code point with no fonts is mapped to an empty set of fonts. This allows
/// the list of code point ranges to be complete, covering every code
/// point. There are no gaps between ranges; instead there are some ranges that
/// map to the empty set. Each range is encoded as the size (number of code
/// points) in the range followed by the value which is the index of the
/// corresponding set in the list of sets.
///
///
/// STMR (Self terminating multiple radix) encoding
/// ---
///
/// This encoding is a minor adaptation of [VLQ encoding][1], using different
/// ranges of characters to represent continuing or terminating digits instead
/// of using a 'continuation' bit.
///
/// The separators between the numbers can be a significant proportion of the
/// number of characters needed to encode a sequence of numbers as a string.
/// Instead values are encoded with two kinds of digits: prefix digits and
/// terminating digits. Each kind of digit uses a different set of characters,
/// and the radix (number of digit characters) can differ between the different
/// kinds of digit.  Lets say we use decimal digits `0`..`9` for prefix digits
/// and `A`..`Z` as terminating digits.
///
///     M = ('M' - 'A') = 12
///     38M = (3 * 10 + 8) * 26 + 12 = 38 * 26 + 12 = 1000
///
/// Choosing a large terminating radix is especially effective when most of the
/// encoded values are small, as is the case with delta-encoding.
///
/// There can be multiple terminating digit kinds to represent different sorts
/// of values. For the range table, the size uses a different terminating digit,
/// 'a'..'z'. This allows the very common size of 1 (accounting over a third of
/// the range sizes) to be omitted. A range is encoded as either
/// `<size><value>`, or `<value>` with an implicit size of 1.  Since the size 1
/// can be implicit, it is always implicit, and the stored sizes are biased by
/// -2.
///
/// | encoding | value | size |
/// | :---     | ---:  | ---: |
/// | A        | 0     | 1    |
/// | B        | 1     | 1    |
/// | 38M      | 1000  | 1    |
/// | aA       | 0     | 2    |
/// | bB       | 1     | 3    |
/// | zZ       | 25    | 27   |
/// | 1a1A     | 26    | 28   |
/// | 38a38M   | 1000  | 1002 |
///
/// STMR-encoded strings are decoded efficiently by a simple loop that updates
/// the current value and performs some additional operation for a terminating
/// digit, e.g. recording the optional size, or creating a range.
///
/// [1]: https://en.wikipedia.org/wiki/Variable-length_quantity

String _computeEncodedFontSets(List<_Font> fonts) {
  final List<_Range> ranges = <_Range>[];
  final List<_FontSet> allSets = <_FontSet>[];

  {
    // The fonts have their supported code points provided as list of inclusive
    // [start, end] ranges. We want to intersect all of these ranges and find
    // the fonts that overlap each intersected range.
    //
    // It is easier to work with the boundaries of the ranges rather than the
    // ranges themselves. The boundaries of the intersected ranges is the union
    // of the boundaries of the individual font ranges.  We scan the boundaries
    // in increasing order, keeping track of the current set of fonts that are
    // in the current intersected range.  Each time the boundary value changes,
    // the current set of fonts is canonicalized and recorded.
    //
    // There has to be a wiki article for this algorithm but I didn't find one.
    final List<_Boundary> boundaries = <_Boundary>[];
    for (final _Font font in fonts) {
      for (final int start in font.starts) {
        boundaries.add(_Boundary(start, true, font));
      }
      for (final int end in font.ends) {
        boundaries.add(_Boundary(end + 1, false, font));
      }
    }
    boundaries.sort(_Boundary.compare);

    // The trie root represents the empty set of fonts.
    final _TrieNode trieRoot = _TrieNode();
    final Set<_Font> currentElements = <_Font>{};

    void newRange(int start, int end) {
      // Ensure we are using the canonical font order.
      final List<_Font> fonts = List<_Font>.of(currentElements)..sort(_Font.compare);
      final _TrieNode node = trieRoot.insertSequenceAtRoot(fonts);
      final _FontSet fontSet = node.fontSet ??= _FontSet(fonts);
      if (fontSet.rangeCount == 0) {
        allSets.add(fontSet);
      }
      fontSet.rangeCount++;
      final _Range range = _Range(start, end, fontSet);
      ranges.add(range);
    }

    int start = 0;
    for (final _Boundary boundary in boundaries) {
      final int value = boundary.value;
      if (value > start) {
        // Boundary has changed, record the pending range `[start, value - 1]`,
        // and start a new range at `value`. `value` must be > 0 to get here.
        newRange(start, value - 1);
        start = value;
      }
      if (boundary.isStart) {
        currentElements.add(boundary.font);
      } else {
        currentElements.remove(boundary.font);
      }
    }
    assert(currentElements.isEmpty);
    // Ensure the ranges cover the whole unicode code point space.
    if (start <= kMaxCodePoint) {
      newRange(start, kMaxCodePoint);
    }
  }

  print('${allSets.length} sets covering ${ranges.length} ranges');

  // Sort _FontSets by the number of ranges that map to that _FontSet, so that
  // _FontSets that are referenced from many ranges have smaller indexes.  This
  // makes the range table encoding smaller, by about half.
  allSets.sort(_FontSet.orderByDecreasingRangeCount);

  for (int i = 0; i < allSets.length; i++) {
    allSets[i].index = i;
  }

  final StringBuffer code = StringBuffer();

  final StringBuffer sb = StringBuffer();
  int totalEncodedLength = 0;

  void encode(int value, int radix, int firstDigitCode) {
    final int prefix = value ~/ radix;
    assert(kPrefixDigit0 == '0'.codeUnitAt(0) && kPrefixRadix == 10);
    if (prefix != 0) {
      sb.write(prefix);
    }
    sb.writeCharCode(firstDigitCode + value.remainder(radix));
  }

  for (final _FontSet fontSet in allSets) {
    int previousFontIndex = -1;
    for (final _Font font in fontSet.fonts) {
      final int fontIndexDelta = font.index - previousFontIndex;
      previousFontIndex = font.index;
      encode(fontIndexDelta - 1, kFontIndexRadix, kFontIndexDigit0);
    }
    if (fontSet != allSets.last) {
      sb.write(',');
    }
    final String fragment = sb.toString();
    sb.clear();
    totalEncodedLength += fragment.length;

    final int length = fontSet.fonts.length;
    code.write('    // #${fontSet.index}: $length font');
    if (length != 1) {
      code.write('s');
    }
    if (length > 0) {
      code.write(': ${fontSet.description()}');
    }
    code.writeln('.');

    code.writeln("    '$fragment'");
  }

  final StringBuffer declarations = StringBuffer();

  final int references = allSets.fold(0, (int sum, _FontSet set) => sum + set.length);
  declarations
    ..writeln(
      '// ${allSets.length} unique sets of fonts'
      ' containing $references font references'
      ' encoded in $totalEncodedLength characters',
    )
    ..writeln('const String encodedFontSets =')
    ..write(code)
    ..writeln('    ;');

  // Encode ranges.
  code.clear();
  totalEncodedLength = 0;

  for (final _Range range in ranges) {
    final int start = range.start;
    final int end = range.end;
    final int index = range.fontSet.index;
    final int size = end - start + 1;

    // Encode <size><index> or <index> for unit ranges.
    if (size >= 2) {
      encode(size - 2, kRangeSizeRadix, kRangeSizeDigit0);
    }
    encode(index, kRangeValueRadix, kRangeValueDigit0);

    final String encoding = sb.toString();
    sb.clear();
    totalEncodedLength += encoding.length;

    String description = start.toRadixString(16);
    if (end != start) {
      description = '$description-${end.toRadixString(16)}';
    }
    if (range.fontSet.fonts.isNotEmpty) {
      description = '${description.padRight(12)} #$index';
    }
    final String encodingText = "'$encoding'".padRight(10);
    code.writeln('    $encodingText // $description');
  }

  declarations
    ..writeln()
    ..writeln('// ${ranges.length} ranges encoded in $totalEncodedLength characters')
    ..writeln('const String encodedFontSetRanges =')
    ..write(code)
    ..writeln('    ;');

  return declarations.toString();
}
