// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:core' hide RegExp;
import 'dart:io' as system;

import 'cache.dart';
import 'formatter.dart';
import 'limits.dart';
import 'patterns.dart';
import 'regexp_debug.dart';

class FetchedContentsOf extends Key {
  FetchedContentsOf(super.value);
}

enum LicenseType {
  afl,
  apache,
  apacheNotice,
  apsl,
  bison,
  bsd,
  eclipse,
  freetype,
  gpl,
  icu,
  defaultTemplate, // metatype: a license that applies to a file without an internal license; only used when searching for a license
  ietf,
  ijg,
  lgpl,
  libpng,
  llvm,
  mit,
  mpl,
  openssl,
  unicode,
  unknown,
  vulkan,
  zlib,
}

LicenseType convertLicenseNameToType(String? name) {
  switch (name) {
    case 'Apache':
    case 'Apache-2.0.txt':
    case 'LICENSE-APACHE-2.0.txt':
    case 'LICENSE.vulkan':
    case 'APACHE-LICENSE-2.0':
      return LicenseType.apache;
    case 'BSD':
    case 'BSD-3-Clause.txt':
    case 'BSD.txt':
      return LicenseType.bsd;
    case 'COPYING-LGPL-2.1':
    case 'LICENSE-LGPL-2':
    case 'LICENSE-LGPL-2.1':
      return LicenseType.lgpl;
    case 'COPYING-GPL-3':
    case 'GPL-3.0-only.txt':
    case 'GPLv2.TXT':
      return LicenseType.gpl;
    case 'FTL.TXT':
      return LicenseType.freetype;
    case 'zlib.h':
      return LicenseType.zlib;
    case 'png.h':
      return LicenseType.libpng;
    case 'ICU':
      return LicenseType.icu;
    case 'Apple Public Source License':
      return LicenseType.apsl;
    case 'OpenSSL':
      return LicenseType.openssl;
    case 'COPYING-MPL-1.1':
    case 'LICENSE.MPLv2':
    case 'http://mozilla.org/MPL/2.0/':
      return LicenseType.mpl;
    case 'COPYRIGHT.vulkan':
      return LicenseType.vulkan;
    case 'LICENSE.MIT':
    case 'MIT':
    case 'MIT.txt':
      return LicenseType.mit;
    // file names that don't say what the type is
    case 'COPYING':
    case 'COPYING.LIB': // lgpl usually
    case 'COPYING.RUNTIME': // gcc exception usually
    case 'COPYING.txt':
    case 'COPYRIGHT.musl':
    case 'Copyright':
    case 'LICENSE':
    case 'LICENSE-APPLE':
    case 'LICENSE.TXT':
    case 'LICENSE.cssmin':
    case 'LICENSE.md':
    case 'LICENSE.rst':
    case 'LICENSE.txt':
    case 'License.txt':
    case 'NOTICE':
    case 'NOTICE.txt':
    case 'copyright':
    case 'extreme.indiana.edu.license.TXT':
    case 'extreme.indiana.edu.license.txt':
    case 'javolution.license.TXT':
    case 'javolution.license.txt':
    case 'libyaml-license.txt':
    case 'license.html':
    case 'license.patch':
    case 'license.txt':
    case 'mh-bsd-gcc':
    case 'pivotal.labs.license.txt':
      return LicenseType.unknown;
  }
  throw 'unknown license type: $name';
}

LicenseType convertBodyToType(String body) {
  if (body.startsWith(lrApache) && body.contains(lrLLVM)) {
    return LicenseType.llvm;
  }
  if (body.startsWith(lrApache)) {
    return LicenseType.apache;
  }
  if (body.startsWith(lrMPL)) {
    return LicenseType.mpl;
  }
  if (body.startsWith(lrGPL)) {
    return LicenseType.gpl;
  }
  if (body.startsWith(lrAPSL)) {
    return LicenseType.apsl;
  }
  if (body.contains(lrOpenSSL)) {
    return LicenseType.openssl;
  }
  if (body.contains(lrBSD)) {
    return LicenseType.bsd;
  }
  if (body.contains(lrMIT)) {
    return LicenseType.mit;
  }
  if (body.contains(lrZlib)) {
    return LicenseType.zlib;
  }
  if (body.contains(lrPNG)) {
    return LicenseType.libpng;
  }
  if (body.contains(lrBison)) {
    return LicenseType.bison;
  }
  return LicenseType.unknown;
}

// API exposed by the classes in main.dart
abstract class LicenseSource {
  String get name;
  String get libraryName;
  String get officialSourceLocation;
  List<License>? nearestLicensesFor(String name);
  License? nearestLicenseOfType(LicenseType type);
  License? nearestLicenseWithName(String name, {String? authors});
}

// Represents a license/file pairing, with metadata saying where the license came from.
class Assignment {
  const Assignment(this.license, this.target, this.source);
  final License license;
  final String target;
  final LicenseSource source;
}

// Represents a group of files assigned to the same license, so that we can avoid
// duplicating licenses in the output.
class GroupedLicense {
  GroupedLicense(this.type, this.body);
  final LicenseType type;
  final String body;

  // The names of files to which this license applies.
  final Set<String> targets = <String>{};

  // The libraries from which those files originate.
  final Set<String> libraries = <String>{};

  // How we determined the license applied to these files.
  final Set<String> origins = <String>{};

  String toStringDebug() {
    final StringBuffer result = StringBuffer();
    result.writeln('=' * 100);
    (libraries.map((String s) => 'LIBRARY: $s').toList()..sort()).forEach(result.writeln);
    (origins.map((String s) => 'ORIGIN: $s').toList()..sort()).forEach(result.writeln);
    result.writeln('TYPE: $type');
    (targets.map((String s) => 'FILE: $s').toList()..sort()).forEach(result.writeln);
    result.writeln('-' * 100);
    if (body.isEmpty) {
      result.writeln('<THIS BLOCK INTENTIONALLY LEFT BLANK>');
    } else {
      result.writeln(body);
    }
    result.writeln('=' * 100);
    return result.toString();
  }

  String toStringFormal() {
    final StringBuffer result = StringBuffer();
    (libraries.toList()..sort()).forEach(result.writeln);
    result.writeln();
    assert(body.isNotEmpty);
    result.write(body);
    return result.toString();
  }
}

List<GroupedLicense> groupLicenses(Iterable<Assignment> assignments) {
  final Map<String, GroupedLicense> groups = <String, GroupedLicense>{};
  for (final Assignment assignment in assignments) {
    final String body = assignment.license.toStringBody(assignment.source);
    final GroupedLicense entry = groups.putIfAbsent(
      body,
      () => GroupedLicense(assignment.license.type, body),
    );
    entry.targets.add(assignment.target);
    entry.libraries.add(assignment.source.libraryName);
    entry.origins.add(assignment.license.origin);
  }
  final List<GroupedLicense> results = groups.values.toList();
  results.sort((GroupedLicense a, GroupedLicense b) => a.body.compareTo(b.body));
  return results;
}

abstract class License {
  factory License.unique(
    String body,
    LicenseType type, {
    bool reformatted = false,
    required String origin,
    String? authors,
    bool yesWeKnowWhatItLooksLikeButItIsNot = false,
  }) {
    if (!reformatted) {
      body = reformat(body);
    }
    final License result = UniqueLicense._(
      body,
      type,
      origin: origin,
      yesWeKnowWhatItLooksLikeButItIsNot: yesWeKnowWhatItLooksLikeButItIsNot,
      authors: authors,
    );
    assert(() {
      if (result is! UniqueLicense || result.type != type) {
        throw 'tried to add a UniqueLicense $type, but it was a duplicate of a ${result.runtimeType} ${result.type}';
      }
      return true;
    }());
    return result;
  }

  factory License.template(
    String body,
    LicenseType type, {
    bool reformatted = false,
    required String origin,
    String? authors,
  }) {
    if (!reformatted) {
      body = reformat(body);
    }
    final License result = TemplateLicense._autosplit(body, type, origin: origin, authors: authors);
    assert(() {
      if (result is! TemplateLicense || result.type != type) {
        throw 'tried to add a TemplateLicense $type, but it was a duplicate of a ${result.runtimeType} ${result.type}';
      }
      return true;
    }());
    return result;
  }

  factory License.multiLicense(
    String body,
    LicenseType type, {
    bool reformatted = false,
    String? authors,
    required String origin,
  }) {
    if (!reformatted) {
      body = reformat(body);
    }
    final License result = MultiLicense._(body, type, origin: origin, authors: authors);
    assert(() {
      if (result is! MultiLicense || result.type != type) {
        throw 'tried to add a MultiLicense $type, but it was a duplicate of a ${result.runtimeType} ${result.type}';
      }
      return true;
    }());
    return result;
  }

  factory License.message(
    String body,
    LicenseType type, {
    bool reformatted = false,
    required String origin,
  }) {
    if (!reformatted) {
      body = reformat(body);
    }
    final License result = MessageLicense._(body, type, origin: origin);
    assert(() {
      if (result is! MessageLicense || result.type != type) {
        throw 'tried to add a MessageLicense $type, but it was a duplicate of a ${result.runtimeType} ${result.type}';
      }
      return true;
    }());
    return result;
  }

  factory License.blank(String body, LicenseType type, {required String origin}) {
    final License result = BlankLicense._(reformat(body), type, origin: origin);
    assert(() {
      if (result is! BlankLicense || result.type != type) {
        throw 'tried to add a BlankLicense $type, but it was a duplicate of a ${result.runtimeType} ${result.type}';
      }
      return true;
    }());
    return result;
  }

  factory License.mozilla(String body, {required String origin}) {
    body = reformat(body);
    final License result = MozillaLicense._(body, LicenseType.mpl, origin: origin);
    assert(() {
      if (result is! MozillaLicense) {
        throw 'tried to add a MozillaLicense, but it was a duplicate of a ${result.runtimeType} ${result.type}';
      }
      return true;
    }());
    return result;
  }

  factory License.fromMultipleBlocks(
    List<String> bodies,
    LicenseType type, {
    String? authors,
    required String origin,
    bool yesWeKnowWhatItLooksLikeButItIsNot = false,
  }) {
    final String body = bodies.map((String s) => reformat(s)).join('\n\n');
    return MultiLicense._(
      body,
      type,
      authors: authors,
      origin: origin,
      yesWeKnowWhatItLooksLikeButItIsNot: yesWeKnowWhatItLooksLikeButItIsNot,
    );
  }

  factory License.fromBodyAndType(
    String body,
    LicenseType type, {
    bool reformatted = false,
    required String origin,
  }) {
    if (!reformatted) {
      body = reformat(body);
    }
    final License result;
    switch (type) {
      case LicenseType.bsd:
      case LicenseType.mit:
        result = TemplateLicense._autosplit(body, type, origin: origin);
      case LicenseType.apache:
      case LicenseType.freetype:
      case LicenseType.ijg:
      case LicenseType.ietf:
      case LicenseType.libpng:
      case LicenseType.llvm: // The LLVM license is an Apache variant
      case LicenseType.unicode:
      case LicenseType.unknown:
      case LicenseType.vulkan:
      case LicenseType.zlib:
        result = MessageLicense._(body, type, origin: origin);
      case LicenseType.apacheNotice:
        result = UniqueLicense._(body, type, origin: origin);
      case LicenseType.mpl:
        result = MozillaLicense._(body, type, origin: origin);
      // The exception in the license of Bison allows redistributing larger
      // works "under terms of your choice"; we choose terms that don't require
      // any notice in the binary distribution.
      case LicenseType.bison:
        result = BlankLicense._(body, type, origin: origin);
      case LicenseType.icu:
      case LicenseType.openssl:
        throw 'Use License.fromMultipleBlocks rather than License.fromBodyAndType for the ICU and OpenSSL licenses.';
      case LicenseType.afl:
      case LicenseType.apsl:
      case LicenseType.eclipse:
      case LicenseType.gpl:
      case LicenseType.lgpl:
        result = DisallowedLicense._(body, type, origin: origin);
      case LicenseType.defaultTemplate:
        throw 'should not be creating a LicenseType.defaultTemplate license, it is not a real type';
    }
    assert(result.type == type);
    return result;
  }

  factory License.fromBodyAndName(String body, String name, {required String origin}) {
    body = reformat(body);
    LicenseType type = convertLicenseNameToType(name);
    if (type == LicenseType.unknown) {
      type = convertBodyToType(body);
    }
    return License.fromBodyAndType(body, type, reformatted: true, origin: origin);
  }

  factory License.fromBody(String body, {required String origin, bool reformatted = false}) {
    if (!reformatted) {
      body = reformat(body);
    }
    final LicenseType type = convertBodyToType(body);
    return License.fromBodyAndType(body, type, reformatted: true, origin: origin);
  }

  factory License.fromCopyrightAndLicense(
    String copyright,
    String template,
    LicenseType type, {
    required String origin,
  }) {
    copyright = reformat(copyright);
    template = reformat(template);
    return TemplateLicense._(copyright, template, type, origin: origin);
  }

  factory License.fromIdentifyingReference(
    String identifyingReference, {
    required String referencer,
  }) {
    String body;
    LicenseType type = LicenseType.unknown;
    switch (identifyingReference) {
      case 'Apache-2.0 OR MIT': // SPDX ID
      case 'Apache-2.0': // SPDX ID
      case 'Apache:2.0':
      case 'http://www.apache.org/licenses/LICENSE-2.0':
      case 'https://www.apache.org/licenses/LICENSE-2.0':
        // If you're wondering why Abseil has what appears to be a duplicate copy of
        // the Apache license, it's because of this:
        // https://github.com/abseil/abseil-cpp/pull/270/files#r793181143
        body = system.File('data/apache-license-2.0').readAsStringSync();
        type = LicenseType.apache;
      case 'Apache-2.0 WITH LLVM-exception': // SPDX ID
      case 'https://llvm.org/LICENSE.txt':
        body = system.File('data/apache-license-2.0-with-llvm-exception').readAsStringSync();
        type = LicenseType.llvm;
      case 'https://developers.google.com/open-source/licenses/bsd':
        body = system.File('data/google-bsd').readAsStringSync();
        type = LicenseType.bsd;
      case 'http://polymer.github.io/LICENSE.txt':
        body = system.File('data/polymer-bsd').readAsStringSync();
        type = LicenseType.bsd;
      case 'http://www.eclipse.org/legal/epl-v10.html':
        body = system.File('data/eclipse-1.0').readAsStringSync();
        type = LicenseType.eclipse;
      case 'COPYING3:3':
        body = system.File('data/gpl-3.0').readAsStringSync();
        type = LicenseType.gpl;
      case 'COPYING.LIB:2':
      case 'COPYING.LIother.m_:2': // blame hyatt
        body = system.File('data/library-gpl-2.0').readAsStringSync();
        type = LicenseType.lgpl;
      case 'GNU Lesser:2':
      // there has never been such a license, but the authors said they meant the LGPL2.1
      case 'GNU Lesser:2.1':
        body = system.File('data/lesser-gpl-2.1').readAsStringSync();
        type = LicenseType.lgpl;
      case 'COPYING.RUNTIME:3.1':
      case 'GCC Runtime Library Exception:3.1':
        body = system.File('data/gpl-gcc-exception-3.1').readAsStringSync();
      case 'Academic Free License:3.0':
        body = system.File('data/academic-3.0').readAsStringSync();
        type = LicenseType.afl;
      case 'Mozilla Public License:1.1':
        body = system.File('data/mozilla-1.1').readAsStringSync();
        type = LicenseType.mpl;
      case 'http://mozilla.org/MPL/2.0/:2.0':
        body = system.File('data/mozilla-2.0').readAsStringSync();
        type = LicenseType.mpl;
      case 'MIT': // SPDX ID
      case 'http://opensource->org/licenses/MIT': // i don't even
      case 'http://opensource.org/licenses/MIT':
      case 'https://opensource.org/licenses/MIT':
        body = system.File('data/mit').readAsStringSync();
        type = LicenseType.mit;
      case 'Unicode-DFS-2016': // SPDX ID
      case 'http://unicode.org/copyright.html#Exhibit1':
      case 'http://www.unicode.org/copyright.html#License':
      case 'http://www.unicode.org/copyright.html':
      case 'http://www.unicode.org/terms_of_use.html': // redirects to copyright.html
      case 'https://www.unicode.org/copyright.html':
      case 'https://www.unicode.org/terms_of_use.html': // redirects to copyright.html
        body = system.File('data/unicode').readAsStringSync();
        type = LicenseType.unicode;
      case 'http://www.ietf.org/rfc/rfc3454.txt':
        body = system.File('data/ietf').readAsStringSync();
        type = LicenseType.ietf;
      default:
        throw 'unknown identifyingReference $identifyingReference';
    }
    return License.fromBodyAndType(
      body,
      type,
      origin: '$identifyingReference referenced by $referencer',
    );
  }

  License._(
    String body,
    this.type, {
    required this.origin,
    String? authors,
    bool yesWeKnowWhatItLooksLikeButItIsNot = false,
  }) : authors = authors ?? _readAuthors(body) {
    assert(() {
      try {
        switch (type) {
          case LicenseType.afl:
          case LicenseType.apsl:
          case LicenseType.eclipse:
          case LicenseType.gpl:
          case LicenseType.lgpl:
            // We do not want this kind of license in our build.
            assert(this is DisallowedLicense);
          case LicenseType.apache:
          case LicenseType.freetype:
          case LicenseType.ijg:
          case LicenseType.ietf:
          case LicenseType.libpng:
          case LicenseType.llvm:
          case LicenseType.unicode:
          case LicenseType.vulkan:
          case LicenseType.zlib:
            assert(this is MessageLicense);
          case LicenseType.apacheNotice:
            assert(this is UniqueLicense);
          case LicenseType.bison:
            assert(this is BlankLicense);
          case LicenseType.bsd:
          case LicenseType.mit:
            assert(this is TemplateLicense);
          case LicenseType.icu:
          case LicenseType.openssl:
            assert(this is MultiLicense);
          case LicenseType.mpl:
            assert(this is MozillaLicense);
          case LicenseType.unknown:
            assert(this is MessageLicense || this is UniqueLicense);
          case LicenseType.defaultTemplate:
            assert(false, 'should not be creating LicenseType.defaultTemplate license');
        }
      } on AssertionError {
        throw 'incorrectly created a $runtimeType for a $type';
      }
      return true;
    }());
    final LicenseType detectedType = convertBodyToType(body);
    if (detectedType != LicenseType.unknown &&
        detectedType != type &&
        !yesWeKnowWhatItLooksLikeButItIsNot) {
      throw 'Created a license of type $type but it looks like $detectedType:\n---------\n$body\n-----------';
    }
    if (type != LicenseType.apache &&
        type != LicenseType.llvm &&
        type != LicenseType.vulkan &&
        type != LicenseType.apacheNotice &&
        body.contains('Apache')) {
      throw 'Non-Apache license (type=$type, detectedType=$detectedType) contains the word "Apache"; maybe it\'s a notice?:\n---\n$body\n---';
    }
    if (detectedType != LicenseType.unknown &&
        detectedType != type &&
        !yesWeKnowWhatItLooksLikeButItIsNot) {
      throw 'Created a license of type $type but it looks like $detectedType.';
    }
    if (body.contains(trailingColon)) {
      throw 'incomplete license detected:\n---\n$body\n---';
    }
    bool isUTF8 = true;
    late List<int> latin1Encoded;
    try {
      latin1Encoded = latin1.encode(body);
      isUTF8 = false;
    } on ArgumentError {
      /* Fall through to next encoding check. */
    }
    if (!isUTF8) {
      bool isAscii = false;
      try {
        ascii.decode(latin1Encoded);
        isAscii = true;
      } on FormatException {
        /* Fall through to next encoding check */
      }
      if (isAscii) {
        return;
      }
      try {
        utf8.decode(latin1Encoded);
        isUTF8 = true;
      } on FormatException {
        /* We check isUTF8 below and throw if necessary */
      }
      if (isUTF8) {
        throw 'tried to create a License object with text that appears to have been misdecoded as Latin1 instead of as UTF-8:\n$body';
      }
    }
  }

  final String? authors;
  final String origin;
  final LicenseType type;

  Assignment assignLicenses(String target, LicenseSource source) {
    return Assignment(this, target, source);
  }

  // This takes a second license, which has been pre-split into copyright and
  // licenseBody, and uses this license to expand it into a new license. How
  // this works depends on this license; for example BSD licenses typically take
  // their body and put on the given copyright, (and the licenseBody argument
  // here in those cases is usually a reference to that license); some other
  // licenses turn into two, one for the original license and one for this
  // copyright/body pair.
  Iterable<License> _expandTemplate(String copyright, String licenseBody, {required String origin});

  String toStringBody(LicenseSource source);

  static final RegExp _copyrightForAuthors = RegExp(
    r'Copyright [-0-9 ,(cC)©]+\b(The .+ Authors)\.',
    caseSensitive: false,
  );

  static String? _readAuthors(String body) {
    final List<Match> matches = _copyrightForAuthors.allMatches(body).toList();
    if (matches.isEmpty) {
      return null;
    }
    if (matches.length > 1) {
      throw 'found too many authors for this copyright:\n$body\n\n${StackTrace.current}\n\n';
    }
    return matches[0].group(1);
  }

  @override
  String toString() => '$runtimeType ($type) from $origin';
}

class _LineRange {
  _LineRange(this.start, this.end, this._body);
  final int start;
  final int end;
  final String _body;
  String? _value;
  String get value {
    _value ??= _body.substring(start, end);
    return _value!;
  }
}

Iterable<_LineRange> _walkLinesBackwards(String body, int start) sync* {
  int? end;
  while (start > 0) {
    start -= 1;
    if (body[start] == '\n') {
      if (end != null) {
        yield _LineRange(start + 1, end, body);
      }
      end = start;
    }
  }
  if (end != null) {
    yield _LineRange(start, end, body);
  }
}

Iterable<_LineRange> _walkLinesForwards(String body, {int start = 0, int? end}) sync* {
  int? startIndex = start == 0 || body[start - 1] == '\n' ? start : null;
  int endIndex = startIndex ?? start;
  end ??= body.length;
  while (endIndex < end) {
    if (body[endIndex] == '\n') {
      if (startIndex != null) {
        yield _LineRange(startIndex, endIndex, body);
      }
      startIndex = endIndex + 1;
    }
    endIndex += 1;
  }
  if (startIndex != null) {
    yield _LineRange(startIndex, endIndex, body);
  }
}

class _SplitLicense {
  _SplitLicense(this._body, this._split)
    : assert(_split == 0 || _split == _body.length || _body[_split] == '\n');

  final String _body;
  final int _split;
  String getCopyright() => _body.substring(0, _split);
  String getConditions() =>
      _split >= _body.length ? '' : _body.substring(_split == 0 ? 0 : _split + 1);
}

_SplitLicense _splitLicense(String body, {bool verifyResults = true}) {
  final Iterator<_LineRange> lines = _walkLinesForwards(body).iterator;
  if (!lines.moveNext()) {
    throw 'tried to split empty license';
  }
  int end = 0;
  String endReason;
  while (true) {
    final String line = lines.current.value;
    if (line == 'Author:' ||
        line == 'This code is derived from software contributed to Berkeley by' ||
        line == 'The Initial Developer of the Original Code is') {
      if (!lines.moveNext()) {
        throw 'unexpected end of block instead of author when looking for copyright';
      }
      if (lines.current.value.trim() == '') {
        throw 'unexpectedly blank line instead of author when looking for copyright';
      }
      end = lines.current.end;
      if (!lines.moveNext()) {
        endReason = 'ran out of text after author';
        break;
      }
    } else if (line.startsWith('Authors:') || line == 'Other contributors:') {
      if (line != 'Authors:') {
        // assume this line contained an author as well
        end = lines.current.end;
      }
      if (!lines.moveNext()) {
        throw 'unexpected end of license when reading list of authors while looking for copyright';
      }
      final String firstAuthor = lines.current.value;
      int subindex = 0;
      while (subindex < firstAuthor.length && (firstAuthor[subindex] == ' ')) {
        subindex += 1;
      }
      if (subindex == 0 || subindex > firstAuthor.length) {
        throw 'unexpected blank line instead of authors found when looking for copyright';
      }
      end = lines.current.end;
      final String prefix = firstAuthor.substring(0, subindex);
      bool hadMoreLines;
      while ((hadMoreLines = lines.moveNext()) && lines.current.value.startsWith(prefix)) {
        final String nextAuthor = lines.current.value.substring(prefix.length);
        if (nextAuthor == '' || nextAuthor[0] == ' ') {
          throw 'unexpectedly ragged author list when looking for copyright';
        }
        end = lines.current.end;
      }
      if (!hadMoreLines) {
        endReason = 'ran out of text while collecting authors';
        break;
      }
    } else if (line.contains(halfCopyrightPattern)) {
      do {
        if (!lines.moveNext()) {
          throw 'unexpected end of block instead of copyright holder when looking for copyright';
        }
        if (lines.current.value.trim() == '') {
          throw 'unexpectedly blank line instead of copyright holder when looking for copyright';
        }
        end = lines.current.end;
      } while (lines.current.value.contains(trailingComma));
      if (!lines.moveNext()) {
        endReason = 'ran out of text after matching halfCopyrightPattern/trailingComma sequence';
        break;
      }
    } else if (copyrightStatementPatterns.any(line.contains)) {
      end = lines.current.end;
      if (!lines.moveNext()) {
        endReason = 'ran out of text after copyright statement pattern';
        break;
      }
    } else {
      endReason = 'line did not match any copyright patterns ("$line")';
      break;
    }
  }
  if (verifyResults &&
      'Copyright ('.allMatches(body, end).isNotEmpty &&
      !body.startsWith('If you modify libpng')) {
    throw 'the license seems to contain a copyright:\n===copyright===\n${body.substring(0, end)}\n===license===\n${body.substring(end)}\n=========\ntermination reason: $endReason';
  }
  return _SplitLicense(body, end);
}

class _PartialLicenseMatch {
  _PartialLicenseMatch(
    this._body,
    this.start,
    this.split,
    this.end,
    this._match, {
    required this.hasCopyrights,
  }) : assert(split >= start),
       assert(split == start || _body[split] == '\n');

  final String _body;
  final int start;
  final int split;
  final int end;
  final Match _match;
  String? group(int? index) => _match.group(index!);
  String? getAuthors() {
    final Match? match = authorPattern.firstMatch(getCopyrights());
    if (match != null) {
      return match.group(1);
    }
    return null;
  }

  String getCopyrights() => _body.substring(start, split);
  String getConditions() => _body.substring(split + 1, end);
  String getEntireLicense() => _body.substring(start, end);
  final bool hasCopyrights;
}

// Look for all matches of `pattern` in `body` and return them along with associated copyrights.
Iterable<_PartialLicenseMatch> _findLicenseBlocks(
  String body,
  RegExp pattern,
  int firstPrefixIndex,
  int indentPrefixIndex, {
  bool needsCopyright = true,
}) sync* {
  // I tried doing this with one big RegExp initially, but that way lay madness.
  for (final Match match in pattern.allMatches(body)) {
    assert(match.groupCount >= firstPrefixIndex);
    assert(match.groupCount >= indentPrefixIndex);
    int start = match.start;
    final String fullPrefix = '${match.group(firstPrefixIndex)}${match.group(indentPrefixIndex)}';
    // first we walk back to the start of the block that has the same prefix (e.g.
    // the start of this comment block)
    int firstLineOffset = 0;
    bool lastWasBlank = false;
    bool foundNonBlank = false;
    for (final _LineRange range in _walkLinesBackwards(body, start)) {
      String line = range.value;
      bool isBlockCommentLine;
      if (line.length > 3 && line.endsWith('*/')) {
        int index = line.length - 3;
        while (line[index] == ' ') {
          index -= 1;
        }
        line = line.substring(0, index + 1);
        isBlockCommentLine = true;
      } else {
        isBlockCommentLine = false;
      }
      if (line.isEmpty || fullPrefix.startsWith(line)) {
        // this is a blank line
        if (lastWasBlank && (foundNonBlank || !needsCopyright)) {
          break;
        }
        lastWasBlank = true;
      } else if (!isBlockCommentLine && line.startsWith('/*')) {
        start = range.start;
        firstLineOffset = 2;
        break;
      } else if (line.startsWith('<!--')) {
        start = range.start;
        firstLineOffset = 4;
        break;
      } else if (fullPrefix.isNotEmpty && !line.startsWith(fullPrefix)) {
        break;
      } else if (licenseFragments.any((RegExp pattern) => line.contains(pattern))) {
        // we're running into another license, abort, abort!
        break;
      } else {
        lastWasBlank = false;
        foundNonBlank = true;
      }
      start = range.start;
    }
    // then we walk forward dropping anything until the first line that matches what
    // we think might be part of a copyright statement
    bool foundAny = false;
    RegExp? debugFirstPattern;
    copyrightSearch:
    for (final _LineRange range in _walkLinesForwards(body, start: start, end: match.start)) {
      String line = range.value;
      if (firstLineOffset > 0) {
        line = line.substring(firstLineOffset);
        firstLineOffset = 0;
      } else if (line.startsWith(fullPrefix)) {
        line = line.substring(fullPrefix.length);
      } else {
        assert(
          line.isEmpty || fullPrefix.startsWith(line),
          'invariant violated: expected this to be a blank line but it was "$line" (prefix is "$fullPrefix").',
        );
        continue copyrightSearch;
      }
      for (final RegExp pattern in copyrightStatementLeadingPatterns) {
        if (line.contains(pattern)) {
          start = range.start;
          foundAny = true;
          debugFirstPattern = pattern;
          break copyrightSearch;
        }
      }
    }
    // At this point we have figured out what might be copyright text before the license (if anything).
    int split;
    if (!foundAny) {
      if (needsCopyright) {
        throw 'could not find copyright before license\nlicense body was:\n---\n${body.substring(match.start, match.end)}\n---\nfile was:\n---\n$body\n---';
      }
      start = match.start;
      split = match.start;
    } else {
      final String copyrights = body.substring(start, match.start);
      final String undecoratedCopyrights = reformat(copyrights);
      // Let's try splitting the copyright block as if it was a license.
      // This will tell us if we collected something in the copyright block
      // that was more license than copyright and that therefore should be
      // examined closer.
      final _SplitLicense consistencyCheck = _splitLicense(
        undecoratedCopyrights,
        verifyResults: false,
      );
      final String conditions = consistencyCheck.getConditions();
      if (conditions != '') {
        // Copyright lines long enough to spill to a second line can create
        // false positives; try to weed those out.
        final String resplitCopyright = consistencyCheck.getCopyright();
        if (resplitCopyright.trim().contains('\n') ||
            conditions.trim().contains('\n') ||
            resplitCopyright.length < 70 ||
            conditions.length > 15) {
          throw 'potential license text caught in _findLicenseBlocks copyright dragnet:\n---\n$conditions\n---\nundecorated copyrights was:\n---\n$undecoratedCopyrights\n---\ncopyrights was:\n---\n$copyrights\n---\nblock was:\n---\n${body.substring(start, match.end)}\n---\nfirst line matched: $debugFirstPattern\n---\npattern:\n$pattern\n---\n${StackTrace.current}\n=============';
        }
      }

      if (!copyrights.contains(anySlightSignOfCopyrights)) {
        throw 'could not find copyright before license block:\n---\ncopyrights was:\n---\n$copyrights\n---\nblock was:\n---\n${body.substring(start, match.end)}\n---';
      }
      assert(
        body[match.start - 1] == '\n',
        'match did not start at a newline; match.start = ${match.start}, match.end = ${match.end}, split at: "${body[match.start - 1]}"',
      );
      split = match.start - 1;
    }
    yield _PartialLicenseMatch(body, start, split, match.end, match, hasCopyrights: foundAny);
  }
}

class _LicenseMatch {
  _LicenseMatch(
    this.license,
    this.start,
    this.end, {
    this.debug = '',
    this.ignoreWhenCheckingOverlappingRegions = false,
    this.missingCopyrights = false,
  });
  final License license;
  final int start;
  final int end;
  final String debug;
  final bool ignoreWhenCheckingOverlappingRegions;
  final bool missingCopyrights;

  @override
  String toString() {
    return '$start..$end: $license';
  }
}

Iterable<_LicenseMatch> _expand(
  License template,
  String copyright,
  String body,
  int start,
  int end, {
  String debug = '',
  required String origin,
}) sync* {
  final List<License> results =
      template._expandTemplate(reformat(copyright), body, origin: origin).toList();
  if (results.isEmpty) {
    throw 'license could not be expanded';
  }
  yield _LicenseMatch(results.first, start, end, debug: 'expanding template for $debug');
  if (results.length > 1) {
    yield* results
        .skip(1)
        .map(
          (License license) => _LicenseMatch(
            license,
            start,
            end,
            ignoreWhenCheckingOverlappingRegions: true,
            debug: 'expanding subsequent template for $debug',
          ),
        );
  }
}

Iterable<_LicenseMatch> _tryReferenceByFilename(
  String body,
  LicenseFileReferencePattern pattern,
  LicenseSource parentDirectory, {
  required String origin,
}) sync* {
  if (pattern.copyrightIndex != null) {
    for (final Match match in pattern.pattern.allMatches(body)) {
      final String copyright = match.group(pattern.copyrightIndex!)!;
      final String? authors =
          pattern.authorIndex != null ? match.group(pattern.authorIndex!) : null;
      final String filename = match.group(pattern.fileIndex)!;
      final License? template = parentDirectory.nearestLicenseWithName(filename, authors: authors);
      if (template == null) {
        throw 'failed to find template $filename in $parentDirectory (authors=$authors)';
      }
      assert(reformat(copyright) != '');
      final String entireLicense = body.substring(match.start, match.end);
      yield* _expand(
        template,
        copyright,
        entireLicense,
        match.start,
        match.end,
        debug: '_tryReferenceByFilename (with explicit copyright) looking for $filename',
        origin: origin,
      );
    }
  } else {
    for (final _PartialLicenseMatch match in _findLicenseBlocks(
      body,
      pattern.pattern,
      pattern.firstPrefixIndex,
      pattern.indentPrefixIndex,
      needsCopyright: pattern.needsCopyright,
    )) {
      final String? authors = match.getAuthors();
      String? filename = match.group(pattern.fileIndex);
      if (filename == 'modp_b64.c') {
        filename = 'modp_b64.cc'; // it was renamed but other files reference the old name
      }
      // There's also special cases for fuchsia/sdk/linux/dart/zircon/lib/src/fakes/handle_disposition.dart
      // (which points to "The Flutter Authors" instead of "The Fuchsia Authors" for mysterious reasons) and
      // third_party/angle/src/common/fuchsia_egl/fuchsia_egl.* (which does something similar), but those
      // files never reach here because they're marked as binary files in filesystem.dart.
      final License? template = parentDirectory.nearestLicenseWithName(filename!, authors: authors);
      if (template == null) {
        throw 'failed to find accompanying "$filename" in $parentDirectory\n'
            'block:\n---\n${match.getEntireLicense()}\n---';
      }
      if (match.getCopyrights() == '') {
        yield _LicenseMatch(
          template,
          match.start,
          match.end,
          debug: '_tryReferenceByFilename (with failed copyright search) looking for $filename',
        );
      } else {
        yield* _expand(
          template,
          match.getCopyrights(),
          match.getEntireLicense(),
          match.start,
          match.end,
          debug: '_tryReferenceByFilename (with successful copyright search) looking for $filename',
          origin: origin,
        );
      }
    }
  }
}

Iterable<_LicenseMatch> _tryReferenceByType(
  String body,
  RegExp pattern,
  LicenseSource parentDirectory, {
  required String origin,
}) sync* {
  for (final _PartialLicenseMatch match in _findLicenseBlocks(
    body,
    pattern,
    1,
    2,
    needsCopyright: false,
  )) {
    final LicenseType type = convertLicenseNameToType(match.group(3));
    final License? template = parentDirectory.nearestLicenseOfType(type);
    if (template == null) {
      throw 'failed to find accompanying $type license in $parentDirectory';
    }
    if (match.getCopyrights() == '') {
      yield _LicenseMatch(
        template,
        match.start,
        match.end,
        debug: '_tryReferenceByType (without copyrights) for type $type',
      );
    } else {
      yield* _expand(
        template,
        match.getCopyrights(),
        match.getEntireLicense(),
        match.start,
        match.end,
        debug: '_tryReferenceByType (with successful copyright search) for type $type',
        origin: origin,
      );
    }
  }
}

License _dereferenceLicense(
  int groupIndex,
  String? Function(int index) group,
  LicenseReferencePattern pattern,
  LicenseSource parentDirectory, {
  required String origin,
}) {
  License? result =
      pattern.checkLocalFirst ? parentDirectory.nearestLicenseWithName(group(groupIndex)!) : null;
  result ??= License.fromIdentifyingReference(group(groupIndex)!, referencer: origin);
  return result;
}

Iterable<_LicenseMatch> _tryReferenceByIdentifyingReference(
  String body,
  LicenseReferencePattern pattern,
  LicenseSource parentDirectory, {
  required String origin,
}) sync* {
  for (final _PartialLicenseMatch match in _findLicenseBlocks(
    body,
    pattern.pattern!,
    1,
    2,
    needsCopyright: false,
  )) {
    if (pattern.spdx) {
      // Per legal advice, we allowlist the use of SPDX headers. Currently, we
      // recognize SPDX headers in code from Khronos. To identify such code, we
      // examine the copyright that came with the SPDX header or we look at the
      // library name. We also accept the headers in some libcxx files that have
      // their own license and that otherwise would trip up our code checking
      // for missed licenses and copyrights. To identify those, we use the
      // library name and the parent directory name or copyright.
      bool allowSpdx = false;
      final String copyrights = match.getCopyrights();
      if (copyrights.contains('The Khronos Group') ||
          parentDirectory.libraryName == 'spirv-cross' ||
          (parentDirectory.libraryName == 'libcxx' && parentDirectory.name == 'ryu') ||
          (parentDirectory.libraryName == 'libcxx' && copyrights.contains('Microsoft'))) {
        allowSpdx = true;
      }
      if (!allowSpdx) {
        // Skip this match.
        continue;
      }
    }
    assert(
      match.group(3) != null,
      'pattern ${pattern.pattern!} did not have three groups when matched against:\n---\n$body\n---\nmatch: $match',
    );
    final License template = _dereferenceLicense(
      3,
      match.group,
      pattern,
      parentDirectory,
      origin: origin,
    );
    if (match.getCopyrights() == '') {
      yield _LicenseMatch(
        template,
        match.start,
        match.end,
        debug: '_tryReferenceByIdentifyingReference (without copyrights)',
      );
    } else {
      yield* _expand(
        template,
        match.getCopyrights(),
        match.getEntireLicense(),
        match.start,
        match.end,
        debug: '_tryReferenceByIdentifyingReference (with copyright)',
        origin: origin,
      );
    }
  }
}

Iterable<_LicenseMatch> _tryInline(
  String body,
  RegExp pattern, {
  required bool needsCopyright,
  required String origin,
}) sync* {
  for (final _PartialLicenseMatch match in _findLicenseBlocks(
    body,
    pattern,
    1,
    2,
    needsCopyright: needsCopyright,
  )) {
    yield _LicenseMatch(
      License.fromBody(match.getEntireLicense(), origin: origin),
      match.start,
      match.end,
      debug: '_tryInline',
      missingCopyrights: needsCopyright && !match.hasCopyrights,
    );
  }
}

Iterable<_LicenseMatch> _tryStray(
  String body,
  RegExp pattern,
  LicenseSource parentDirectory, {
  required String origin,
}) sync* {
  // this one doesn't look for copyrights (that's the point, the patterns _are_ the copyrights)
  bool gotTemplate = false;
  License? template;
  for (final Match match in pattern.allMatches(body)) {
    if (!gotTemplate) {
      template = parentDirectory.nearestLicenseOfType(LicenseType.defaultTemplate);
      gotTemplate = true;
    }
    if (template != null) {
      yield* _expand(
        template,
        match.group(0)!,
        match.group(0)!,
        match.start,
        match.end,
        debug: '_tryStray (with template)',
        origin: origin,
      );
    } else {
      yield _LicenseMatch(
        License.fromBody(match.group(0)!, origin: origin),
        match.start,
        match.end,
        debug: '_tryStray',
      );
    }
  }
}

Iterable<_LicenseMatch> _tryForwardReferencePattern(
  String fileContents,
  ForwardReferencePattern pattern,
  License template,
  LicenseSource source, {
  required String origin,
}) sync* {
  String? body;
  for (final _PartialLicenseMatch match in _findLicenseBlocks(
    fileContents,
    pattern.pattern,
    pattern.firstPrefixIndex,
    pattern.indentPrefixIndex,
  )) {
    body ??= template.toStringBody(source);
    if (!body.contains(pattern.targetPattern)) {
      throw 'forward license reference to unexpected license\n'
          'license:\n---\n$body\n---\nexpected pattern:\n---\n${pattern.targetPattern}\n---';
    }
    yield* _expand(
      template,
      match.getCopyrights(),
      match.getEntireLicense(),
      match.start,
      match.end,
      debug: '_tryForwardReferencePattern',
      origin: origin,
    );
  }
}

List<License> determineLicensesFor(
  String fileContents,
  String filename,
  LicenseSource? parentDirectory, {
  required String origin,
}) {
  if (parentDirectory == null) {
    throw 'Fatal error: determineLicensesFor was called with parentDirectory=null!';
  }
  if (fileContents.length > kMaxSize) {
    fileContents = fileContents.substring(0, kMaxSize);
  }
  final List<_LicenseMatch> results = <_LicenseMatch>[];
  fileContents = stripAsciiArt(fileContents);
  results.addAll(
    csReferencesByFilename.expand(
      (LicenseFileReferencePattern pattern) =>
          _tryReferenceByFilename(fileContents, pattern, parentDirectory, origin: origin),
    ),
  );
  results.addAll(
    csReferencesByType.expand(
      (RegExp pattern) =>
          _tryReferenceByType(fileContents, pattern, parentDirectory, origin: origin),
    ),
  );
  results.addAll(
    csReferencesByIdentifyingReference.expand(
      (LicenseReferencePattern pattern) => _tryReferenceByIdentifyingReference(
        fileContents,
        pattern,
        parentDirectory,
        origin: origin,
      ),
    ),
  );
  results.addAll(
    csTemplateLicenses.expand(
      (RegExp pattern) => _tryInline(fileContents, pattern, needsCopyright: true, origin: origin),
    ),
  );
  results.addAll(
    csNoticeLicenses.expand(
      (RegExp pattern) => _tryInline(fileContents, pattern, needsCopyright: false, origin: origin),
    ),
  );
  _LicenseMatch? usedTemplate;
  if (results.length == 1) {
    final _LicenseMatch target = results.single;
    results.addAll(
      csForwardReferenceLicenses.expand(
        (ForwardReferencePattern pattern) => _tryForwardReferencePattern(
          fileContents,
          pattern,
          target.license,
          parentDirectory,
          origin: origin,
        ),
      ),
    );
    if (results.length > 1) {
      usedTemplate = target;
    }
  }
  for (final _LicenseMatch match in results.where(
    (_LicenseMatch match) => match.missingCopyrights,
  )) {
    throw 'found a license for $filename but could not match its copyright:\n----8<----\n${match.license}\n----8<----';
  }
  if (results.isEmpty) {
    if ((fileContents.contains(copyrightMentionPattern) &&
            fileContents.contains(licenseMentionPattern)) &&
        !fileContents.contains(copyrightMentionOkPattern)) {
      throw 'failed to find any license but saw unmatched potential copyright and license statements; first twenty lines:\n----8<----\n${fileContents.split("\n").take(20).join("\n")}\n----8<----';
    }
  }
  // Some files have the odd copyright that isn't explicitly attached to a
  // license; we treat those as notice licenses. Only such copyrights
  // allowlisted in csStrayCopyrights are handled this way, though. For each of
  // these, we have to make sure they don't overlap any of the actual licenses
  // matched earlier, so we check for overlaps on each one first.
  strays:
  for (final _LicenseMatch stray in csStrayCopyrights.expand(
    (RegExp pattern) => _tryStray(fileContents, pattern, parentDirectory, origin: origin),
  )) {
    for (final _LicenseMatch full in results) {
      if (stray.start >= full.start && stray.end <= full.end) {
        continue strays;
      }
    }
    results.add(stray);
  }
  final List<_LicenseMatch> verificationList = results.toList();
  if (usedTemplate != null && !verificationList.contains(usedTemplate)) {
    verificationList.add(usedTemplate);
  }
  verificationList.sort((_LicenseMatch a, _LicenseMatch b) {
    final int result = a.start - b.start;
    if (result != 0) {
      return result;
    }
    return a.end - b.end;
  });
  int position = 0;
  for (final _LicenseMatch m in verificationList) {
    if (m.ignoreWhenCheckingOverlappingRegions) {
      continue;
    } // some text expanded into multiple licenses, so overlapping is expected
    if (position > m.start) {
      system.stderr.writeln('\n\noverlapping licenses:');
      for (final _LicenseMatch n in results) {
        system.stderr.writeln(
          'license match: ${n.start}..${n.end}, ${n.license.runtimeType}, ${n.debug}\n'
          '  first line: ${n.license.toStringBody(parentDirectory).split("\n").first}\n'
          '  last line: ${n.license.toStringBody(parentDirectory).split("\n").last}',
        );
      }
      throw 'overlapping licenses in $filename (one ends at $position, another starts at ${m.start})';
    }
    if (position < m.start) {
      final String substring = fileContents.substring(position, m.start);
      if (substring.contains(copyrightMentionPattern) &&
          !substring.contains(copyrightMentionOkPattern)) {
        throw 'there is another unmatched potential copyright statement in $filename:\n  $position..${m.start}: "$substring"\nmatched licenses: $results';
      }
      if (substring.contains(licenseMentionPattern)) {
        throw 'there is another unmatched potential license in $filename:\n  $position..${m.start}: "$substring"\nmatched licenses: $results';
      }
    }
    position = m.end;
  }
  final String substring = fileContents.substring(position);
  if (substring.contains(copyrightMentionPattern) &&
      !substring.contains(copyrightMentionOkPattern)) {
    throw 'there is an unmatched potential copyright statement in $filename:\n  $position..end: "$substring"\nmatched licenses: $results';
  }
  if (substring.contains(licenseMentionPattern)) {
    throw 'there is an unmatched potential license in $filename:\n  $position..end: "$substring"\nmatched licenses: $results';
  }
  return results.map((_LicenseMatch entry) => entry.license).toSet().toList();
}

License? interpretAsRedirectLicense(
  String fileContents,
  LicenseSource parentDirectory, {
  required String origin,
}) {
  _SplitLicense split;
  try {
    split = _splitLicense(fileContents);
  } on String {
    return null;
  }
  final String body = split.getConditions().trim();
  License? result;
  for (final LicenseReferencePattern pattern in csReferencesByIdentifyingReference) {
    if (pattern.spdx) {
      // We don't support SPDX headers in files that use _RepositoryLicenseRedirectFile.
      // Before changing this, obtain legal advice.
      continue;
    }
    final Match? match = pattern.pattern!.matchAsPrefix(body);
    if (match != null && match.start == 0 && match.end == body.length) {
      final License candidate = _dereferenceLicense(
        3,
        match.group as String? Function(int?),
        pattern,
        parentDirectory,
        origin: origin,
      );
      if (result != null) {
        throw 'Multiple potential matches in interpretAsRedirectLicense in $parentDirectory; body was:\n------8<------\n$fileContents\n------8<------';
      }
      result = candidate;
    }
  }
  return result;
}

// the kind of license that just wants to show a message (e.g. the JPEG one)
class MessageLicense extends License {
  MessageLicense._(this.body, LicenseType type, {required String origin})
    : super._(body, type, origin: origin);

  final String body;

  @override
  String toStringBody(LicenseSource source) => body;

  @override
  Iterable<License> _expandTemplate(
    String copyright,
    String licenseBody, {
    required String origin,
  }) => <License>[this];
}

// the kind of license that says to include the copyright and the license text (e.g. BSD)
class TemplateLicense extends License {
  TemplateLicense._(
    this.defaultCopyright,
    this.terms,
    LicenseType type, {
    String? authors,
    required String origin,
  }) : assert(!defaultCopyright.endsWith('\n')),
       assert(!terms.startsWith('\n')),
       assert(terms.isNotEmpty),
       super._('$defaultCopyright\n\n$terms', type, origin: origin, authors: authors);

  factory TemplateLicense._autosplit(
    String body,
    LicenseType type, {
    String? authors,
    required String origin,
  }) {
    final _SplitLicense bits = _splitLicense(body);
    final String copyright = bits.getCopyright();
    final String terms = bits.getConditions();
    assert(
      (copyright.isEmpty && terms == body) ||
          ('$copyright\n$terms' == body) ||
          (copyright == body && terms.isEmpty),
      '_splitLicense contract violation.\nCOPYRIGHT:\n===\n$copyright\n===\nTERMS:\n===\n$terms\n===\nBODY:\n===\n$body\n===\n',
    );
    int copyrightLength = copyright.length;
    while (copyrightLength > 0 && copyright[copyrightLength - 1] == '\n') {
      copyrightLength -= 1;
    }
    int termsStart = 0;
    while (termsStart < terms.length && terms[termsStart] == '\n') {
      termsStart += 1;
    }
    return TemplateLicense._(
      copyright.substring(0, copyrightLength),
      terms.substring(termsStart),
      type,
      authors: authors,
      origin: origin,
    );
  }

  final String defaultCopyright;
  final String terms;

  @override
  String toStringBody(LicenseSource source) {
    if (defaultCopyright.isEmpty) {
      return terms;
    }
    return '$defaultCopyright\n\n$terms';
  }

  @override
  Iterable<License> _expandTemplate(
    String copyright,
    String licenseBody, {
    required String origin,
  }) {
    return <License>[
      License.fromCopyrightAndLicense(copyright, terms, type, origin: '$origin + ${this.origin}'),
    ];
  }
}

// The kind of license that expands to two license blocks a main license and the referring block
// (e.g. OpenSSL).
//
// This is a lawyer-suggested workaround for handling BSD-style licenses where instead of there
// being a single license block where it's obvious what is meant by "above copyright notice" and
// "this list of conditions", we instead have a bunch of similar licenses, each with their own
// copyright, plus there's a copyright in the file that (probably) doesn't exactly match any of the
// copyrights in the file, plus some text saying that that license applies to the file.
class MultiLicense extends License {
  MultiLicense._(
    this.body,
    LicenseType type, {
    String? authors,
    required String origin,
    bool yesWeKnowWhatItLooksLikeButItIsNot = false,
  }) : super._(
         body,
         type,
         origin: origin,
         authors: authors,
         yesWeKnowWhatItLooksLikeButItIsNot: yesWeKnowWhatItLooksLikeButItIsNot,
       );

  final String body;

  @override
  String toStringBody(LicenseSource source) => body;

  @override
  Iterable<License> _expandTemplate(
    String copyright,
    String licenseBody, {
    required String origin,
  }) {
    // Sometimes a license (e.g. the OpenSSL license in the BoringSSL package) is referenced
    // from a file that has its own copyright header. When that happens we just print the referenced
    // license and the reference to that license separately because it's not at all clear how we're
    // supposed to merge them otherwise.
    licenseBody = reformat(licenseBody);
    assert(
      licenseBody.startsWith(copyright),
      'copyright:\n---\n$copyright\n---\nlicenseBody:\n---\n$licenseBody\n---',
    );
    return <License>[
      this,
      License.fromBody(licenseBody, origin: '$origin (with ${this.origin})', reformatted: true),
    ];
  }
}

// the kind of license that should not be combined with separate copyright notices
class UniqueLicense extends License {
  UniqueLicense._(
    this.body,
    LicenseType type, {
    required String origin,
    String? authors,
    bool yesWeKnowWhatItLooksLikeButItIsNot = false,
  }) : super._(
         body,
         type,
         origin: origin,
         yesWeKnowWhatItLooksLikeButItIsNot: yesWeKnowWhatItLooksLikeButItIsNot,
         authors: authors,
       );

  final String body;

  @override
  String toStringBody(LicenseSource source) => body;

  @override
  Iterable<License> _expandTemplate(
    String copyright,
    String licenseBody, {
    required String origin,
  }) {
    throw 'attempted to expand non-template license with "$copyright"\ntemplate was: $this';
  }
}

// the kind of license that doesn't need to be reported anywhere
class BlankLicense extends License {
  BlankLicense._(super.body, super.type, {required super.origin}) : super._();

  @override
  Iterable<License> _expandTemplate(
    String copyright,
    String licenseBody, {
    required String origin,
  }) {
    // We don't care what copyrights this kind of license has, we don't need
    // to report them. Just report |this| (which is always blank, see below).
    return <License>[this];
  }

  @override
  String toStringBody(LicenseSource source) => '';
}

// MPL
class MozillaLicense extends License {
  MozillaLicense._(this.body, LicenseType type, {required String origin})
    : assert(type == LicenseType.mpl),
      super._(body, type, origin: origin);

  final String body;

  @override
  Assignment assignLicenses(String target, LicenseSource source) {
    if (source.libraryName != 'fallback_root_certificates') {
      throw 'Only fallack_root_certificates is allowed to use the MPL.';
    }
    return Assignment(this, target, source);
  }

  @override
  Iterable<License> _expandTemplate(
    String copyright,
    String licenseBody, {
    required String origin,
  }) {
    throw 'attempted to expand non-template license with "$copyright"\ntemplate was: $this';
  }

  @override
  String toStringBody(LicenseSource source) {
    final StringBuffer result = StringBuffer();
    result.writeln(body);
    result.writeln();
    result.writeln(
      "You may obtain a copy of this library's Source Code Form from: ${source.officialSourceLocation}",
    );
    return result.toString();
  }
}

class DisallowedLicense extends License {
  DisallowedLicense._(this.body, LicenseType type, {required String origin})
    : super._(body, type, origin: origin);

  final String body;

  @override
  Assignment assignLicenses(String target, LicenseSource source) {
    throw '$target (in ${source.libraryName}) attempted to use $origin which is a disallowed license type ($type)';
  }

  @override
  Iterable<License> _expandTemplate(
    String copyright,
    String licenseBody, {
    required String origin,
  }) {
    throw 'attempted to use $origin which is a disallowed license type ($type)';
  }

  @override
  String toStringBody(LicenseSource source) {
    throw '${source.libraryName} attempted to use $origin which is a disallowed license type ($type)';
  }
}
