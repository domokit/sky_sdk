// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// See README in this directory for information on how this code is organized.

import 'dart:core' hide RegExp;
import 'dart:io' as system;
import 'dart:math' as math;

import 'package:args/args.dart';
import 'package:collection/collection.dart' show IterableExtension;
import 'package:crypto/crypto.dart' as crypto;
import 'package:path/path.dart' as path;

import 'filesystem.dart' as fs;
import 'licenses.dart';
import 'paths.dart';
import 'patterns.dart';
import 'regexp_debug.dart';

abstract class _RepositoryEntry implements Comparable<_RepositoryEntry> {
  _RepositoryEntry(this.parent, this.io);
  final _RepositoryDirectory? parent;
  final fs.IoNode io;
  String get name => io.name;

  @override
  int compareTo(_RepositoryEntry other) => toString().compareTo(other.toString());

  @override
  String toString() => io.fullName;
}

abstract class _RepositoryFile extends _RepositoryEntry {
  _RepositoryFile(_RepositoryDirectory super.parent, fs.File super.io);

  fs.File get ioFile => super.io as fs.File;
}

abstract class _RepositoryLicensedFile extends _RepositoryFile {
  _RepositoryLicensedFile(super.parent, super.io);

  // Returns the License that is found inside this file.
  //
  // Used when one file says it is covered by the license of another file.
  License extractInternalLicense() {
    throw 'tried to extract a license from $this but it does not contain text';
  }

  // Creates the mapping from the file to its licenses.
  //
  // Subclasses implement things like applying licenses found within the file
  // itself, applying licenses that point to other files, etc.
  //
  // The assignments are later grouped when generating the output, see
  // groupLicenses in licenses.dart.
  Iterable<Assignment> assignLicenses() {
    final List<License> licenses = parent!.nearestLicensesFor(name);
    if (licenses.isEmpty) {
      throw 'file has no detectable license and no in-scope default license file';
    }
    return licenses.map((License license) => license.assignLicenses(io.fullName, parent!));
  }
}

class _RepositorySourceFile extends _RepositoryLicensedFile {
  _RepositorySourceFile(super.parent, fs.TextFile super.io) : _contents = _readFile(io);

  fs.TextFile get ioTextFile => super.io as fs.TextFile;

  final String _contents;

  static String _readFile(fs.TextFile ioTextFile) {
    late String contents;
    try {
      contents = ioTextFile.readString();
    } on FormatException {
      throw 'non-UTF8 data in $ioTextFile';
    }
    return contents;
  }

  List<License>? _internalLicenses;

  @override
  License extractInternalLicense() {
    _internalLicenses ??= determineLicensesFor(_contents, name, parent, origin: io.fullName);
    if (_internalLicenses == null || _internalLicenses!.isEmpty) {
      throw 'tried to extract a license from $this but there is nothing here';
    }
    if (_internalLicenses!.length > 1) {
      throw 'tried to extract a license from $this but there are multiple licenses in this file';
    }
    return _internalLicenses!.single;
  }

  @override
  Iterable<Assignment> assignLicenses() {
    _internalLicenses ??= determineLicensesFor(_contents, name, parent, origin: io.fullName);
    final List<License>? licenses = _internalLicenses;
    if (licenses != null && licenses.isNotEmpty) {
      return licenses.map((License license) => license.assignLicenses(io.fullName, parent!));
    }
    return super.assignLicenses();
  }
}

class _RepositoryBinaryFile extends _RepositoryLicensedFile {
  _RepositoryBinaryFile(super.parent, super.io);

  @override
  Iterable<Assignment> assignLicenses() {
    final List<License> licenses = parent!.nearestLicensesFor(name);
    if (licenses.isEmpty) {
      throw 'no license file found in scope for ${io.fullName}';
    }
    return licenses.map((License license) => license.assignLicenses(io.fullName, parent!));
  }
}

// LICENSES

abstract class _RepositoryLicenseFile extends _RepositoryFile {
  _RepositoryLicenseFile(super.parent, fs.TextFile super.io);

  // returns any licenses that apply specifically to the file named "name"
  List<License> licensesFor(String name) => licenses;

  // returns the license that applies to files looking for a license of the given type
  License? licenseOfType(LicenseType type);

  // returns the license that applies to files looking for a license of the given name
  License? licenseWithName(String name);

  License? get defaultLicense;
  List<License> get licenses;
}

abstract class _RepositorySingleLicenseFile extends _RepositoryLicenseFile {
  _RepositorySingleLicenseFile(super.parent, super.io, this.license);

  final License license;

  @override
  License? licenseWithName(String name) {
    if (this.name == name) {
      return license;
    }
    return null;
  }

  @override
  License? licenseOfType(LicenseType type) {
    assert(type != LicenseType.unknown);
    if (type == license.type) {
      return license;
    }
    return null;
  }

  @override
  License get defaultLicense => license;

  @override
  List<License> get licenses => <License>[license];
}

class _RepositoryGeneralSingleLicenseFile extends _RepositorySingleLicenseFile {
  _RepositoryGeneralSingleLicenseFile(_RepositoryDirectory parent, fs.TextFile io)
    : super(parent, io, _parseLicense(io));

  static License _parseLicense(fs.TextFile io) {
    final String body = io.readString();
    int start = 0;
    Match? match;
    while ((match = licenseHeaders.matchAsPrefix(body, start)) != null) {
      assert(match!.end > start);
      start = match!.end; // https://github.com/dart-lang/sdk/issues/50264
    }
    return License.fromBodyAndName(body.substring(start), io.name, origin: io.fullName);
  }
}

// Avoid using this unless the license file has multiple licenses in a weird mishmash.
// For example, the RapidJSON license which mixes BSD and MIT in somewhat confusing ways
// (e.g. having the copyright for one above the terms for the other and so on).
class _RepositoryOpaqueLicenseFile extends _RepositorySingleLicenseFile {
  _RepositoryOpaqueLicenseFile(_RepositoryDirectory parent, fs.TextFile io)
    : super(
        parent,
        io,
        License.unique(
          io.readString(),
          LicenseType.unknown,
          origin: io.fullName,
          yesWeKnowWhatItLooksLikeButItIsNot: true,
        ),
      );
}

class _RepositoryReadmeIjgFile extends _RepositorySingleLicenseFile {
  _RepositoryReadmeIjgFile(_RepositoryDirectory parent, fs.TextFile io)
    : super(parent, io, _parseLicense(io));

  // The message we are required to include in our output.
  //
  // We include it by just including the whole license.
  static const String _message =
      'this software is based in part on the work of the Independent JPEG Group';

  // The license text that says we should output _message.
  //
  // We inject _message into it, munged such that each space character can also
  // match a newline, so that if the license wraps the text, it still matches.
  static final RegExp _pattern = RegExp(
    r'Permission is hereby granted to use, copy, modify, and distribute this\n'
    r'software \(or portions thereof\) for any purpose, without fee, subject to these\n'
    r'conditions:\n'
    r'\(1\) If any part of the source code for this software is distributed, then this\n'
    r'README file must be included, with this copyright and no-warranty notice\n'
    r'unaltered; and any additions, deletions, or changes to the original files\n'
    r'must be clearly indicated in accompanying documentation\.\n'
    r'\(2\) If only executable code is distributed, then the accompanying\n'
    r'documentation must state that "'
    '${_message.replaceAll(" ", "[ \n]+")}'
    r'"\.\n'
    r'\(3\) Permission for use of this software is granted only if the user accepts\n'
    r'full responsibility for any undesirable consequences; the authors accept\n'
    r'NO LIABILITY for damages of any kind\.\n',
  );

  static License _parseLicense(fs.TextFile io) {
    final String body = io.readString();
    if (!body.contains(_pattern)) {
      throw 'unexpected contents in IJG README';
    }
    return License.message(body, LicenseType.ijg, origin: io.fullName);
  }

  @override
  License? licenseWithName(String name) {
    if (this.name == name) {
      return license;
    }
    return null;
  }
}

class _RepositoryDartLicenseFile extends _RepositorySingleLicenseFile {
  _RepositoryDartLicenseFile(_RepositoryDirectory parent, fs.TextFile io)
    : super(parent, io, _parseLicense(io));

  static final RegExp _pattern = RegExp(r'(Copyright (?:.|\n)+)$');

  static License _parseLicense(fs.TextFile io) {
    final Match? match = _pattern.firstMatch(io.readString());
    if (match == null || match.groupCount != 1) {
      throw 'unexpected Dart license file contents';
    }
    return License.template(match.group(1)!, LicenseType.bsd, origin: io.fullName);
  }
}

class _RepositoryLibPngLicenseFile extends _RepositorySingleLicenseFile {
  _RepositoryLibPngLicenseFile(_RepositoryDirectory parent, fs.TextFile io)
    : super(parent, io, License.message(io.readString(), LicenseType.libpng, origin: io.fullName)) {
    _verifyLicense(io);
  }

  static void _verifyLicense(fs.TextFile io) {
    final String contents = io.readString();
    if (!contents.contains(RegExp('COPYRIGHT NOTICE, DISCLAIMER, and LICENSE:?')) ||
        !contents.contains('png')) {
      throw 'unexpected libpng license file contents:\n----8<----\n$contents\n----<8----';
    }
  }
}

class _RepositoryLibJpegTurboLicenseFile extends _RepositoryLicenseFile {
  _RepositoryLibJpegTurboLicenseFile(_RepositoryDirectory parent, fs.TextFile io)
    : super(parent, io) {
    _parseLicense(io);
  }

  static final RegExp _pattern = RegExp(
    r'libjpeg-turbo is covered by three compatible BSD-style open source licenses:\n'
    r'\n'
    r'- The IJG \(Independent JPEG Group\) License, which is listed in\n'
    r'  \[README\.ijg\]\(README\.ijg\)\n'
    r'\n'
    r'  This license applies to the libjpeg API library and associated programs\n'
    r'  \(any code inherited from libjpeg, and any modifications to that code\.\)\n'
    r'\n'
    r'- The Modified \(3-clause\) BSD License, which is listed in\n'
    r'  \[turbojpeg\.c\]\(turbojpeg\.c\)\n'
    r'\n'
    r'  This license covers the TurboJPEG API library and associated programs\.\n'
    r'\n'
    r'- The zlib License, which is listed in \[simd/jsimdext\.inc\]\(simd/jsimdext\.inc\)\n'
    r'\n'
    r'  This license is a subset of the other two, and it covers the libjpeg-turbo\n'
    r'  SIMD extensions\.\n',
  );

  static void _parseLicense(fs.TextFile io) {
    final String body = io.readString();
    if (!body.contains(_pattern)) {
      throw 'unexpected contents in libjpeg-turbo LICENSE';
    }
  }

  @override
  License? licenseWithName(String name) {
    return null;
  }

  @override
  License? licenseOfType(LicenseType type) {
    return null;
  }

  @override
  License? get defaultLicense => null;

  List<License>? _licenses;

  @override
  List<License> get licenses {
    if (_licenses == null) {
      final _RepositoryLicenseFile readme =
          parent!.getChildByName('README.ijg') as _RepositoryReadmeIjgFile;
      final _RepositorySourceFile main =
          parent!.getChildByName('turbojpeg.c') as _RepositorySourceFile;
      final _RepositoryDirectory simd = parent!.getChildByName('simd') as _RepositoryDirectory;
      final _RepositorySourceFile zlib =
          simd.getChildByName('jsimdext.inc') as _RepositorySourceFile;
      _licenses = <License>[];
      _licenses!.addAll(readme.licenses);
      _licenses!.add(main.extractInternalLicense());
      _licenses!.add(zlib.extractInternalLicense());
    }
    return _licenses!;
  }
}

class _RepositoryFreetypeLicenseFile extends _RepositoryLicenseFile {
  _RepositoryFreetypeLicenseFile(super.parent, super.io) : _target = _parseLicense(io);

  static final RegExp _pattern = RegExp(
    r'FREETYPE LICENSES\n'
    r'-----------------\n'
    r'\n'
    r'The FreeType  2 font  engine is  copyrighted work  and cannot  be used\n'
    r'legally without  a software  license\.  In order  to make  this project\n'
    r'usable to  a vast majority of  developers, we distribute it  under two\n'
    r'mutually exclusive open-source licenses\.\n'
    r'\n'
    r'This means that \*you\* must choose  \*one\* of the two licenses described\n'
    r'below, then obey all its terms and conditions when using FreeType 2 in\n'
    r'any of your projects or products\.\n'
    r'\n'
    r'  - The FreeType License,  found in the file  `docs/(FTL\.TXT)`, which is\n'
    r'    similar to the  original BSD license \*with\*  an advertising clause\n'
    r'    that forces  you to explicitly  cite the FreeType project  in your\n'
    r"    product's  documentation\.  All  details are  in the  license file\.\n"
    r"    This license is suited to products which don't use the GNU General\n"
    r'    Public License\.\n'
    r'\n'
    r'    Note that  this license  is compatible to  the GNU  General Public\n'
    r'    License version 3, but not version 2\.\n'
    r'\n'
    r'  - The   GNU   General   Public   License   version   2,   found   in\n'
    r'    `docs/GPLv2\.TXT`  \(any  later  version  can  be  used  also\),  for\n'
    r'    programs  which  already  use  the  GPL\.  Note  that  the  FTL  is\n'
    r'    incompatible with GPLv2 due to its advertisement clause\.\n'
    r'\n'
    r'The contributed  BDF and PCF  drivers come  with a license  similar to\n'
    r'that  of the  X Window  System\.   It is  compatible to  the above  two\n'
    r'licenses \(see files `src/bdf/README`  and `src/pcf/README`\)\.  The same\n'
    r'holds   for   the   source    code   files   `src/base/fthash\.c`   and\n'
    r'`include/freetype/internal/fthash\.h`; they wer part  of the BDF driver\n'
    r'in earlier FreeType versions\.\n'
    r'\n'
    r'The gzip  module uses the  zlib license \(see  `src/gzip/zlib\.h`\) which\n'
    r'too is compatible to the above two licenses\.\n'
    r'\n'
    r'The  MD5 checksum  support  \(only used  for  debugging in  development\n'
    r'builds\) is in the public domain\.\n'
    r'\n*'
    r'--- end of LICENSE\.TXT ---\n*$',
  );

  static String _parseLicense(fs.TextFile io) {
    final Match? match = _pattern.firstMatch(io.readString());
    if (match == null || match.groupCount != 1) {
      throw 'unexpected Freetype license file contents';
    }
    return match.group(1)!;
  }

  final String _target;
  List<License>? _targetLicense;

  void _warmCache() {
    if (parent != null && _targetLicense == null) {
      final License? license = parent!.nearestLicenseWithName(_target);
      if (license == null) {
        throw 'Could not find license in Freetype directory. Make sure $_target is not excluded from consideration.';
      }
      _targetLicense = <License>[license];
    }
  }

  @override
  License? licenseOfType(LicenseType type) => null;

  @override
  License? licenseWithName(String name) => null;

  @override
  License get defaultLicense {
    _warmCache();
    return _targetLicense!.single;
  }

  @override
  List<License> get licenses {
    _warmCache();
    return _targetLicense ?? const <License>[];
  }
}

class _RepositoryIcuLicenseFile extends _RepositorySingleLicenseFile {
  factory _RepositoryIcuLicenseFile(_RepositoryDirectory parent, fs.TextFile io) {
    final Match? match = _pattern.firstMatch(_fixup(io.readString()));
    if (match == null) {
      throw 'could not parse ICU license file';
    }
    const int groupCount = 22;
    assert(
      match.groupCount == groupCount,
      'ICU: expected $groupCount groups, but got ${match.groupCount}',
    );
    const int timeZoneGroup = 18;
    if (match.group(timeZoneGroup)!.contains(copyrightMentionPattern)) {
      throw 'ICU: unexpected copyright in time zone database group\n:${match.group(timeZoneGroup)}';
    }
    if (!match.group(timeZoneGroup)!.contains('7.  Database Ownership')) {
      throw 'ICU: unexpected text in time zone database group\n:${match.group(timeZoneGroup)}';
    }
    const int gplGroup1 = 20;
    const int gplGroup2 = 21;
    if (!match.group(gplGroup1)!.contains(gplExceptionExplanation1) ||
        !match.group(gplGroup2)!.contains(gplExceptionExplanation2)) {
      throw 'ICU: did not find GPL exception in GPL-licensed files';
    }
    const Set<int> skippedGroups = <int>{timeZoneGroup, gplGroup1, gplGroup2};
    return _RepositoryIcuLicenseFile._(
      parent,
      io,
      License.template(match.group(2)!, LicenseType.bsd, origin: io.fullName),
      License.fromMultipleBlocks(
        match
            .groups(
              Iterable<int>.generate(
                groupCount,
                (int index) => index + 1,
              ).where((int index) => !skippedGroups.contains(index)).toList(),
            )
            .cast<String>(),
        LicenseType.icu,
        origin: io.fullName,
        yesWeKnowWhatItLooksLikeButItIsNot: true,
      ),
    );
  }

  _RepositoryIcuLicenseFile._(
    _RepositoryDirectory parent,
    fs.TextFile io,
    this.template,
    License license,
  ) : super(parent, io, license);

  // Every paragraph of the license is mentioned. All newlines are disregarded [\n+].
  static final RegExp _pattern = RegExp(
    // Starting from ICU 74.1 this has been changed to Unicode License V3.
    r'^(UNICODE LICENSE V3\n+' // group
    r'COPYRIGHT AND PERMISSION NOTICE(?:.|\n)+?'
    r')(Permission is hereby granted(?:.|\n)+?)' // template group
    r'\n+-+\n+'
    // From here onwards should be the same as before 74.1
    r'(Third-Party Software Licenses\n+' // group
    r'This section contains third-party software notices and/or additional\n'
    r'terms for licensed third-party software components included within ICU\n'
    r'libraries\.\n+)'
    r'-+\n+'
    r'(ICU License - ICU 1\.8\.1 to ICU 57.1[ \n]+?' // group
    r'COPYRIGHT AND PERMISSION NOTICE\n+'
    r'Copyright (?:.|\n)+?)\n+'
    r'-+\n+'
    r'(Chinese/Japanese Word Break Dictionary Data \(cjdict\.txt\)\n+)' // group
    r'( #     The Google Chrome software developed by Google is licensed under\n?' // group
    r' # the BSD license\. Other software included in this distribution is\n?'
    r' # provided under other licenses, as set forth below\.\n'
    r' #\n'
    r' #  The BSD License\n'
    r' #  http://opensource\.org/licenses/bsd-license\.php\n'
    r' # +Copyright(?:.|\n)+?)\n'
    r' #\n'
    r' #\n'
    r'( #  The word list in cjdict.txt are generated by combining three word lists\n?' // group
    r' # listed below with further processing for compound word breaking\. The\n?'
    r' # frequency is generated with an iterative training against Google web\n?'
    r' # corpora\.\n'
    r' #\n' // if this section is taken out, make sure to remove the cjdict.txt exclusion in paths.dart
    r' #  \* Libtabe \(Chinese\)\n'
    r' #    - https://sourceforge\.net/project/\?group_id=1519\n'
    r' #    - Its license terms and conditions are shown below\.\n'
    r' #\n'
    r' #  \* IPADIC \(Japanese\)\n'
    r' #    - http://chasen\.aist-nara\.ac\.jp/chasen/distribution\.html\n'
    r' #    - Its license terms and conditions are shown below\.\n'
    r' #\n)'
    r' #  ---------COPYING\.libtabe ---- BEGIN--------------------\n'
    r' #\n'
    r' # +/\*\n'
    r'( # +\* Copyright (?:.|\n)+?)\n' // group
    r' # +\*/\n'
    r' #\n'
    r' # +/\*\n'
    r'( # +\* Copyright (?:.|\n)+?)\n' // group
    r' # +\*/\n'
    r' #\n'
    r'( # +Copyright (?:.|\n)+?)\n' // group
    r' #\n'
    r' # +---------------COPYING\.libtabe-----END--------------------------------\n'
    r' #\n'
    r' #\n'
    r' # +---------------COPYING\.ipadic-----BEGIN-------------------------------\n'
    r' #\n'
    r'( # +Copyright (?:.|\n)+?)\n' // group
    r' #\n'
    r' # +---------------COPYING\.ipadic-----END----------------------------------\n+'
    r'-+\n+'
    r'(Lao Word Break Dictionary Data \(laodict\.txt\)\n)' // group
    r'\n' // if this section is taken out, make sure to remove the laodict.txt exclusion in paths.dart
    r'( # +Copyright(?:.|\n)+?)' // group
    r'( # +This file is derived(?:.|\n)+?)\n'
    r'\n'
    r'-+\n'
    r'\n'
    r'(Burmese Word Break Dictionary Data \(burmesedict\.txt\)\n)' // group
    r'\n' // if this section is taken out, make sure to remove the burmesedict.txt exclusion in paths.dart
    r'( # +Copyright(?:.|\n)+?)\n' // group
    r' # +-+\n'
    r'( # +Copyright(?:.|\n)+?)\n' // group
    r' # +-+\n+'
    r'-+\n+'
    r'( *Time Zone Database\n'
    r'(?:.|\n)+)\n' // group (not really a license -- excluded)
    r'\n'
    r'-+\n'
    r'\n'
    r'(Google double-conversion\n' // group
    r'\n'
    r'Copyright(?:.|\n)+)\n'
    r'\n'
    r'-+\n'
    r'\n'
    r'(File: aclocal\.m4 \(only for ICU4C\)\n' // group (excluded)
    r'Section: pkg\.m4 - Macros to locate and utilise pkg-config\.\n+'
    r'Copyright (?:.|\n)+?)\n'
    r'\n'
    r'-+\n'
    r'\n'
    r'(File: config\.guess \(only for ICU4C\)\n+' // group (excluded)
    r'This file is free software(?:.|\n)+?)\n'
    r'\n'
    r'-+\n'
    r'\n'
    r'(File: install-sh \(only for ICU4C\)\n+' // group
    r'Copyright(?:.|\n)+)',
  );

  static const String gplExceptionExplanation1 =
      'As a special exception to the GNU General Public License, if you\n'
      'distribute this file as part of a program that contains a\n'
      'configuration script generated by Autoconf, you may include it under\n'
      'the same distribution terms that you use for the rest of that\n'
      'program.\n'
      '\n'
      '\n'
      '(The condition for the exception is fulfilled because\n'
      'ICU4C includes a configuration script generated by Autoconf,\n'
      'namely the `configure` script.)';

  static const String gplExceptionExplanation2 =
      'As a special exception to the GNU General Public License, if you\n'
      'distribute this file as part of a program that contains a\n'
      'configuration script generated by Autoconf, you may include it under\n'
      'the same distribution terms that you use for the rest of that\n'
      'program.  This Exception is an additional permission under section 7\n'
      'of the GNU General Public License, version 3 ("GPLv3").\n'
      '\n'
      '\n'
      '(The condition for the exception is fulfilled because\n'
      'ICU4C includes a configuration script generated by Autoconf,\n'
      'namely the `configure` script.)';

  // Fixes an error in the license's formatting that our reformatter wouldn't be
  // able to figure out on its own and which would otherwise completely mess up
  // the reformatting of this license file.
  static String _fixup(String license) {
    return license.replaceAll(
      ' #       *                    Sinica. All rights reserved.',
      ' #   *                    Sinica. All rights reserved.',
    );
  }

  final License template;

  @override
  License? licenseOfType(LicenseType type) {
    if (type == LicenseType.defaultTemplate) {
      return template;
    }
    return super.licenseOfType(type);
  }
}

class _RepositoryCxxStlDualLicenseFile extends _RepositoryLicenseFile {
  _RepositoryCxxStlDualLicenseFile(super.parent, super.io) : _licenses = _parseLicenses(io);

  static final RegExp _pattern = RegExp(
    r'^'
    r'==============================================================================\n'
    r'The LLVM Project is under the Apache License v2\.0 with LLVM Exceptions:\n'
    r'==============================================================================\n'
    r'\n('
    r' *Apache License\n'
    r' *Version 2.0, January 2004\n'
    r' *http://www.apache.org/licenses/\n'
    r'\n'
    r'.+?)\n+'
    r'---- LLVM Exceptions to the Apache 2.0 License ----'
    r'.+?'
    r'==============================================================================\n'
    r'Software from third parties included in the LLVM Project:\n'
    r'==============================================================================\n'
    r'The LLVM Project contains third party software which is under different license\n'
    r'terms\. All such code will be identified clearly using at least one of two\n'
    r'mechanisms:\n'
    r'1\) It will be in a separate directory tree with its own `LICENSE\.txt` or\n'
    r' *`LICENSE` file at the top containing the specific license and restrictions\n'
    r' *which apply to that software, or\n'
    r'2\) It will contain specific license and restriction terms at the top of every\n'
    r' *file\.\n'
    r'\n'
    r'==============================================================================\n'
    r'Legacy LLVM License \(https://llvm\.org/docs/DeveloperPolicy\.html#legacy\):\n'
    r'==============================================================================\n'
    r'\n'
    r'The libc\+\+(?:abi)? library is dual licensed under both the University of Illinois\n'
    r'"BSD-Like" license and the MIT license\. *As a user of this code you may choose\n'
    r'to use it under either license\. *As a contributor, you agree to allow your code\n'
    r'to be used under both\.\n'
    r'\n'
    r'Full text of the relevant licenses is included below\.\n'
    r'\n'
    r'==============================================================================\n'
    r'\n'
    r'University of Illinois/NCSA\n'
    r'Open Source License\n'
    r'\n'
    r'(Copyright \(c\) 2009-2019 by the contributors listed in CREDITS\.TXT\n'
    r'\n'
    r'All rights reserved\.\n'
    r'\n'
    r'Developed by:\n'
    r'\n'
    r' *LLVM Team\n'
    r'\n'
    r' *University of Illinois at Urbana-Champaign\n'
    r'\n'
    r' *http://llvm\.org\n'
    r'\n'
    r'Permission is hereby granted, free of charge, to any person obtaining a copy of\n'
    r'this software and associated documentation files \(the "Software"\), to deal with\n'
    r'the Software without restriction, including without limitation the rights to\n'
    r'use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies\n'
    r'of the Software, and to permit persons to whom the Software is furnished to do\n'
    r'so, subject to the following conditions:\n'
    r'\n'
    r' *\* Redistributions of source code must retain the above copyright notice,\n'
    r' *this list of conditions and the following disclaimers\.\n'
    r'\n'
    r' *\* Redistributions in binary form must reproduce the above copyright notice,\n'
    r' *this list of conditions and the following disclaimers in the\n'
    r' *documentation and/or other materials provided with the distribution\.\n'
    r'\n'
    r' *\* Neither the names of the LLVM Team, University of Illinois at\n'
    r' *Urbana-Champaign, nor the names of its contributors may be used to\n'
    r' *endorse or promote products derived from this Software without specific\n'
    r' *prior written permission\.\n'
    r'\n'
    r'THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR\n'
    r'IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS\n'
    r'FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT\.  IN NO EVENT SHALL THE\n'
    r'CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER\n'
    r'LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,\n'
    r'OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE\n'
    r'SOFTWARE\.)\n*'
    r'==============================================================================\n*'
    r'(Copyright \(c\) 2009-2014 by the contributors listed in CREDITS\.TXT\n'
    r'\n'
    r'Permission is hereby granted, free of charge, to any person obtaining a copy\n'
    r'of this software and associated documentation files \(the "Software"\), to deal\n'
    r'in the Software without restriction, including without limitation the rights\n'
    r'to use, copy, modify, merge, publish, distribute, sublicense, and/or sell\n'
    r'copies of the Software, and to permit persons to whom the Software is\n'
    r'furnished to do so, subject to the following conditions:\n'
    r'\n'
    r'The above copyright notice and this permission notice shall be included in\n'
    r'all copies or substantial portions of the Software\.\n'
    r'\n'
    r'THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR\n'
    r'IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,\n'
    r'FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT\. IN NO EVENT SHALL THE\n'
    r'AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER\n'
    r'LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,\n'
    r'OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN\n'
    r'THE SOFTWARE\.)\n*'
    r'$',
    dotAll: true,
  );

  static List<License> _parseLicenses(fs.TextFile io) {
    final Match? match = _pattern.firstMatch(io.readString());
    if (match == null) {
      throw 'unexpected license file contents';
    }
    if (match.groupCount != 3) {
      throw 'internal error; match count inconsistency\nRemainder:[[${match.input.substring(match.end)}]]';
    }
    return <License>[
      // License.fromBodyAndType(match.group(1), LicenseType.apache), // the exception says we can ignore this
      License.fromBodyAndType(match.group(2)!, LicenseType.bsd, origin: io.fullName),
      License.fromBodyAndType(match.group(3)!, LicenseType.mit, origin: io.fullName),
    ];
  }

  final List<License> _licenses;

  @override
  License licenseOfType(LicenseType type) {
    throw 'tried to look up a dual-license license by type ("$type")';
  }

  @override
  License licenseWithName(String name) {
    throw 'tried to look up a dual-license license by name ("$name")';
  }

  @override
  License get defaultLicense => _licenses[0];

  @override
  List<License> get licenses => _licenses;
}

class _RepositoryKhronosLicenseFile extends _RepositoryLicenseFile {
  _RepositoryKhronosLicenseFile(super.parent, super.io) : _licenses = _parseLicenses(io);

  static final RegExp _pattern = RegExp(
    r'^(Copyright .+?)\n'
    r'SGI FREE SOFTWARE LICENSE B[^\n]+\n\n'
    r'(Copyright .+?)$',
    dotAll: true,
  );

  static List<License> _parseLicenses(fs.TextFile io) {
    final Match? match = _pattern.firstMatch(io.readString());
    if (match == null || match.groupCount != 2) {
      throw 'unexpected Khronos license file contents';
    }
    return <License>[
      License.fromBodyAndType(match.group(1)!, LicenseType.mit, origin: io.fullName),
      License.fromBodyAndType(match.group(2)!, LicenseType.mit, origin: io.fullName),
    ];
  }

  final List<License> _licenses;

  @override
  License licenseOfType(LicenseType type) {
    throw 'tried to look up a combination license by type ("$type")';
  }

  @override
  License licenseWithName(String name) {
    throw 'tried to look up a combination license by name ("$name")';
  }

  @override
  License get defaultLicense {
    throw 'there is no default Khronos license';
  }

  @override
  List<License> get licenses => _licenses;
}

/// The BoringSSL license file.
///
/// This file contains a bunch of different licenses, but other files
/// refer to it as if it was a monolithic license so we sort of have
/// to treat the whole thing as a MultiLicense.
class _RepositoryOpenSSLLicenseFile extends _RepositorySingleLicenseFile {
  _RepositoryOpenSSLLicenseFile(_RepositoryDirectory parent, fs.TextFile io)
    : super(parent, io, _parseLicense(io));

  static final RegExp _pattern = RegExp(
    // advice is to skip the first 27 lines of this file in the LICENSE file
    r'^BoringSSL is a fork of OpenSSL. As such, .+?'
    r'The following are Google-internal bug numbers where explicit permission from\n'
    r'some authors is recorded for use of their work\. \(This is purely for our own\n'
    r'record keeping\.\)\n+'
    r'[0-9+ \n]+\n+'
    r'(' // 1
    r' *OpenSSL License\n'
    r' *---------------\n+)'
    r'(.+?)\n+' // 2
    r'(' // 3
    r' *Original SSLeay License\n'
    r' *-----------------------\n+)'
    r'(.+?)\n+' // 4
    r'( *ISC license used for completely new code in BoringSSL:\n+)' // 5
    r'(.+?)\n+' // 6
    r'( *The code in third_party/fiat carries the MIT license:\n+)' // 7
    r'(.+?)\n+' // 8
    r'(' // 9
    r' *Licenses for support code\n'
    r' *-------------------------\n+)'
    r'(.+?)\n+' // 10
    r'(BoringSSL uses the Chromium test infrastructure to run a continuous build,\n' // 11
    r'trybots etc\. The scripts which manage this, and the script for generating build\n'
    r'metadata, are under the Chromium license\. Distributing code linked against\n'
    r'BoringSSL does not trigger this license\.)\n+'
    r'(.+?)\n+$', // 12
    dotAll: true,
  );

  static License _parseLicense(fs.TextFile io) {
    final Match? match = _pattern.firstMatch(io.readString());
    if (match == null) {
      throw 'Failed to match OpenSSL license pattern.';
    }
    assert(match.groupCount == 12);
    return License.fromMultipleBlocks(
      List<String>.generate(match.groupCount, (int index) => match.group(index + 1)!).toList(),
      LicenseType.openssl,
      origin: io.fullName,
      authors: 'The OpenSSL Project Authors',
      yesWeKnowWhatItLooksLikeButItIsNot: true, // looks like BSD, but...
    );
  }
}

class _RepositoryFuchsiaSdkLinuxLicenseFile extends _RepositorySingleLicenseFile {
  _RepositoryFuchsiaSdkLinuxLicenseFile(_RepositoryDirectory parent, fs.TextFile io)
    : super(parent, io, _parseLicense(io));

  static const String _pattern =
      'The majority of files in this project use the Apache 2.0 License.\n'
      'There are a few exceptions and their license can be found in the source.\n'
      'Any license deviations from Apache 2.0 are "more permissive" licenses.\n';

  static License _parseLicense(fs.TextFile io) {
    final String body = io.readString();
    if (!body.startsWith(_pattern)) {
      throw 'unexpected Fuchsia license file contents';
    }
    return License.fromBodyAndType(
      body.substring(_pattern.length),
      LicenseType.apache,
      origin: io.fullName,
    );
  }
}

/// The BoringSSL license file.
///
/// This file contains a bunch of different licenses, but other files
/// refer to it as if it was a monolithic license so we sort of have
/// to treat the whole thing as a MultiLicense.
class _RepositoryVulkanApacheLicenseFile extends _RepositorySingleLicenseFile {
  _RepositoryVulkanApacheLicenseFile(_RepositoryDirectory parent, fs.TextFile io)
    : super(parent, io, _parseLicense(io));

  static const String _prefix =
      'The majority of files in this project use the Apache 2.0 License.\n'
      'There are a few exceptions and their license can be found in the source.\n'
      'Any license deviations from Apache 2.0 are "more permissive" licenses.\n'
      "Any file without a license in it's source defaults to the repository Apache 2.0 License.\n"
      '\n'
      '===========================================================================================\n'
      '\n';

  static License _parseLicense(fs.TextFile io) {
    final String body = io.readString();
    if (!body.startsWith(_prefix)) {
      throw 'Failed to match Vulkan Apache license prefix.';
    }
    return License.fromBody(body.substring(_prefix.length), origin: io.fullName);
  }
}

// DIRECTORIES

typedef _Constructor = _RepositoryFile Function(_RepositoryDirectory parent, fs.TextFile);

class _RepositoryDirectory extends _RepositoryEntry implements LicenseSource {
  _RepositoryDirectory(super.parent, fs.Directory super.io) {
    crawl();
  }

  fs.Directory get ioDirectory => super.io as fs.Directory;
  fs.Directory get rootDirectory => parent != null ? parent!.rootDirectory : ioDirectory;

  final List<_RepositoryDirectory> _subdirectories = <_RepositoryDirectory>[];
  final List<_RepositoryLicensedFile> _files = <_RepositoryLicensedFile>[];
  final List<_RepositoryLicenseFile> _licenses = <_RepositoryLicenseFile>[];
  static final List<fs.IoNode> _excluded = <fs.IoNode>[];

  List<_RepositoryDirectory> get subdirectories => _subdirectories;

  final Map<String, _RepositoryEntry> _childrenByName = <String, _RepositoryEntry>{};

  void crawl() {
    for (final fs.IoNode entry in ioDirectory.walk) {
      if (shouldRecurse(entry)) {
        assert(!_childrenByName.containsKey(entry.name));
        if (entry is fs.Directory) {
          final _RepositoryDirectory child = createSubdirectory(entry);
          _subdirectories.add(child);
          _childrenByName[child.name] = child;
        } else if (entry is fs.File) {
          try {
            final _RepositoryFile child = createFile(entry);
            if (child is _RepositoryLicensedFile) {
              _files.add(child);
            } else {
              assert(child is _RepositoryLicenseFile);
              _licenses.add(child as _RepositoryLicenseFile);
            }
            _childrenByName[child.name] = child;
          } catch (error, stack) {
            throw 'failed to handle $entry\n$error\nOriginal stack:\n$stack';
          }
        } else {
          assert(entry is fs.Link);
        }
      } else {
        _excluded.add(entry);
      }
    }
    for (final _RepositoryDirectory child in virtualSubdirectories) {
      _subdirectories.add(child);
      _childrenByName[child.name] = child;
    }
  }

  // Override this to add additional child directories that do not represent a
  // direct child of this directory's filesystem node.
  List<_RepositoryDirectory> get virtualSubdirectories => <_RepositoryDirectory>[];

  bool shouldRecurse(fs.IoNode entry) {
    if (entry is fs.File) {
      if (skippedCommonFiles.contains(entry.name)) {
        return false;
      }
      if (skippedCommonExtensions.contains(path.extension(entry.name))) {
        return false;
      }
    } else if (entry is fs.Directory) {
      if (skippedCommonDirectories.contains(entry.name)) {
        return false;
      }
    } else {
      throw 'unexpected entry type ${entry.runtimeType}: ${entry.fullName}';
    }
    final String target = path.relative(entry.fullName, from: rootDirectory.fullName);
    if (skippedPaths.contains(target)) {
      return false;
    }
    for (final Pattern pattern in skippedFilePatterns) {
      if (target.contains(pattern)) {
        return false;
      }
    }
    return true;
  }

  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'third_party') {
      return _RepositoryGenericThirdPartyDirectory(this, entry);
    }
    return _RepositoryDirectory(this, entry);
  }

  // the bit at the beginning excludes files like "license.py".
  static final RegExp _licenseNamePattern = RegExp(
    r'^(?!.*\.py$)(?!.*(?:no|update)-copyright)(?!.*mh-bsd-gcc).*\b_*(?:license(?!\.html)|copying|copyright|notice|l?gpl|GPLv2|bsd|mit|mpl?|ftl|Apache)_*\b',
    caseSensitive: false,
  );

  static const Map<String, _Constructor> _specialCaseFiles = <String, _Constructor>{
    '/flutter/third_party/boringssl/src/LICENSE': _RepositoryOpenSSLLicenseFile.new,
    '/flutter/third_party/dart/LICENSE': _RepositoryDartLicenseFile.new,
    '/flutter/third_party/freetype2/LICENSE.TXT': _RepositoryFreetypeLicenseFile.new,
    '/flutter/third_party/icu/LICENSE': _RepositoryIcuLicenseFile.new,
    '/flutter/third_party/inja/third_party/include/nlohmann/json.hpp': _RepositoryInjaJsonFile.new,
    '/flutter/third_party/libcxx/LICENSE.TXT': _RepositoryCxxStlDualLicenseFile.new,
    '/flutter/third_party/libcxxabi/LICENSE.TXT': _RepositoryCxxStlDualLicenseFile.new,
    '/flutter/third_party/libjpeg-turbo/src/LICENSE': _RepositoryLibJpegTurboLicenseFile.new,
    '/flutter/third_party/libjpeg-turbo/src/README.ijg': _RepositoryReadmeIjgFile.new,
    '/flutter/third_party/libpng/LICENSE': _RepositoryLibPngLicenseFile.new,
    '/flutter/third_party/rapidjson/LICENSE': _RepositoryOpaqueLicenseFile.new,
    '/flutter/third_party/rapidjson/license.txt': _RepositoryOpaqueLicenseFile.new,
    '/flutter/third_party/vulkan-deps/vulkan-validation-layers/src/LICENSE.txt':
        _RepositoryVulkanApacheLicenseFile.new,
    '/fuchsia/sdk/linux/LICENSE.vulkan': _RepositoryFuchsiaSdkLinuxLicenseFile.new,
    '/fuchsia/sdk/mac/LICENSE.vulkan': _RepositoryFuchsiaSdkLinuxLicenseFile.new,
    '/third_party/khronos/LICENSE': _RepositoryKhronosLicenseFile.new,
  };

  _RepositoryFile createFile(fs.IoNode entry) {
    final String key = '/${path.relative(entry.fullName, from: rootDirectory.fullName)}';
    final _Constructor? constructor = _specialCaseFiles[key];
    if (entry is fs.TextFile) {
      if (constructor != null) {
        return constructor(this, entry);
      }
      if (entry.name.contains(_licenseNamePattern)) {
        return _RepositoryGeneralSingleLicenseFile(this, entry);
      }
      return _RepositorySourceFile(this, entry);
    }
    return _RepositoryBinaryFile(this, entry as fs.File);
  }

  int get count =>
      _files.length +
      _subdirectories.fold<int>(0, (int count, _RepositoryDirectory child) => count + child.count);

  bool _canGoUp() {
    assert(parent != null || isLicenseRoot);
    return isLicenseRootException || (!isLicenseRoot && !parent!.subdirectoriesAreLicenseRoots);
  }

  @override
  List<License> nearestLicensesFor(String name) {
    if (_licenses.isEmpty) {
      if (_canGoUp()) {
        return parent!.nearestLicensesFor('${io.name}/$name');
      }
      return const <License>[];
    }
    return _licenses.expand((_RepositoryLicenseFile license) {
      return license.licensesFor(name);
    }).toList();
  }

  final Map<LicenseType, License?> _nearestLicenseOfTypeCache = <LicenseType, License?>{};

  /// Searches the current directory, all parent directories up to the license
  /// root, and all their descendants, for a license of the specified type.
  @override
  License? nearestLicenseOfType(LicenseType type) {
    return _nearestLicenseOfTypeCache.putIfAbsent(type, () {
      final License? result = _localLicenseWithType(type);
      if (result != null) {
        return result;
      }
      if (_canGoUp()) {
        return parent!.nearestLicenseOfType(type);
      }
      return _fullWalkDownForLicenseWithType(type);
    });
  }

  /// Searches the current directory for licenses of the specified type.
  License? _localLicenseWithType(LicenseType type) {
    final List<License> licenses =
        _licenses.expand((_RepositoryLicenseFile license) {
          final License? result = license.licenseOfType(type);
          if (result != null) {
            return <License>[result];
          }
          return const <License>[];
        }).toList();
    if (licenses.length > 1) {
      print('unexpectedly found multiple matching licenses in $name of type $type');
      return null;
    }
    if (licenses.isNotEmpty) {
      return licenses.single;
    }
    return null;
  }

  /// Searches all subdirectories (depth-first) for a license of the specified type.
  License? _fullWalkDownForLicenseWithType(LicenseType type) {
    for (final _RepositoryDirectory directory in _subdirectories) {
      if (directory._canGoUp()) {
        // avoid crawling into other license scopes
        final License? result =
            directory._localLicenseWithType(type) ??
            directory._fullWalkDownForLicenseWithType(type);
        if (result != null) {
          return result;
        }
      }
    }
    return null;
  }

  final Map<String, License?> _nearestLicenseWithNameCache = <String, License?>{};

  @override
  License? nearestLicenseWithName(String name, {String? authors}) {
    assert(!name.contains('\x00'));
    assert(authors == null || !authors.contains('\x00'));
    assert(authors == null || authors != '${null}');
    final License? result = _nearestLicenseWithNameCache.putIfAbsent('$name\x00$authors', () {
      final License? result = _localLicenseWithName(name, authors: authors);
      if (result != null) {
        return result;
      }
      if (_canGoUp()) {
        return parent!.nearestLicenseWithName(name, authors: authors);
      }
      return _fullWalkDownForLicenseWithName(name, authors: authors) ??
          (authors != null ? parent?._fullWalkUpForLicenseWithName(name, authors: authors) : null);
    });
    return result;
  }

  License? _localLicenseWithName(String name, {String? authors}) {
    final _RepositoryEntry? entry = _childrenByName[name];
    License? license;
    if (entry is _RepositoryLicensedFile) {
      license = entry.extractInternalLicense();
    } else if (entry is _RepositoryLicenseFile) {
      license = entry.defaultLicense;
    } else if (entry != null) {
      if (authors == null) {
        throw 'found "$name" in $this but it was a ${entry.runtimeType}';
      }
    }
    if (license != null && authors != null && authors != license.authors) {
      license = null;
    }
    return license;
  }

  License? _fullWalkUpForLicenseWithName(String name, {required String authors}) {
    // When looking for a license specific to certain authors, we want to walk
    // to the top of the local license root, then from there check all the
    // ancestors and all the descendants.
    //
    // We check even the ancestors (on the other side of license root
    // boundaries) for this because when we know which authors we're looking
    // for, it's reasonable to look all the way up the tree (e.g. the Flutter
    // Authors license is at the root, and is sometimes mentioned in various
    // files deep inside third party directories).
    return _localLicenseWithName(name, authors: authors) ??
        parent?._fullWalkUpForLicenseWithName(name, authors: authors);
  }

  License? _fullWalkDownForLicenseWithName(String name, {String? authors}) {
    for (final _RepositoryDirectory directory in _subdirectories) {
      if (directory._canGoUp()) {
        // avoid crawling into other license scopes
        final License? result =
            directory._localLicenseWithName(name, authors: authors) ??
            directory._fullWalkDownForLicenseWithName(name, authors: authors);
        if (result != null) {
          return result;
        }
      }
    }
    return null;
  }

  /// Unless isLicenseRootException is true, we should not walk up the tree from
  /// here looking for licenses.
  bool get isLicenseRoot => parent == null;

  /// Unless isLicenseRootException is true on a child, the child should not
  /// walk up the tree to here looking for licenses.
  bool get subdirectoriesAreLicenseRoots => false;

  @override
  String get libraryName {
    if (isLicenseRoot || parent!.subdirectoriesAreLicenseRoots) {
      return name;
    }
    if (parent!.parent == null) {
      throw '$this is not a license root';
    }
    return parent!.libraryName;
  }

  /// Overrides isLicenseRoot and parent.subdirectoriesAreLicenseRoots for cases
  /// where a directory contains license roots instead of being one. This
  /// allows, for example, the expat third_party directory to contain a
  /// subdirectory with expat while itself containing a BUILD file that points
  /// to the LICENSE in the root of the repo.
  bool get isLicenseRootException => false;

  _RepositoryEntry getChildByName(String name) {
    assert(_childrenByName.containsKey(name), 'missing $name in ${io.fullName}');
    return _childrenByName[name]!;
  }

  Iterable<Assignment> assignLicenses(_Progress progress) {
    final List<Assignment> result = <Assignment>[];
    // Report licenses for files in this directory
    for (final _RepositoryLicensedFile file in _files) {
      try {
        progress.label = '$file';
        final Iterable<Assignment> licenses = file.assignLicenses();
        assert(licenses.isNotEmpty);
        result.addAll(licenses);
        progress.advance(success: true);
      } catch (e, stack) {
        system.stderr.writeln('\nerror searching for copyright in: ${file.io}\n$e');
        if (e is! String) {
          system.stderr.writeln(stack);
        }
        system.stderr.writeln('\n');
        progress.advance(success: false);
      }
    }
    // Recurse to subdirectories
    for (final _RepositoryDirectory directory in _subdirectories) {
      result.addAll(directory.assignLicenses(progress));
    }
    return result;
  }

  int get fileCount {
    int result = _files.length;
    for (final _RepositoryDirectory directory in _subdirectories) {
      result += directory.fileCount;
    }
    return result;
  }

  Iterable<_RepositoryLicensedFile> get _signatureFiles sync* {
    for (final _RepositoryLicensedFile file in _files) {
      yield file;
    }
    for (final _RepositoryDirectory directory in _subdirectories) {
      if (directory.includeInSignature) {
        yield* directory._signatureFiles;
      }
    }
  }

  Stream<List<int>> _signatureStream(List<_RepositoryLicensedFile> files) async* {
    for (final _RepositoryLicensedFile file in files) {
      yield file.io.fullName.codeUnits;
      yield file.ioFile.readBytes()!;
    }
  }

  /// Compute a signature representing a hash of all the licensed files within
  /// this directory tree.
  Future<String> get signature async {
    final List<_RepositoryLicensedFile> allFiles = _signatureFiles.toList();
    allFiles.sort(
      (_RepositoryLicensedFile a, _RepositoryLicensedFile b) =>
          a.io.fullName.compareTo(b.io.fullName),
    );
    final crypto.Digest digest = await crypto.md5.bind(_signatureStream(allFiles)).single;
    return digest.bytes.map((int e) => e.toRadixString(16).padLeft(2, '0')).join();
  }

  /// True if this directory's contents should be included when computing the signature.
  bool get includeInSignature => true;

  @override
  String get officialSourceLocation {
    throw 'license requested source location for directory that should not need it';
  }
}

class _RepositoryReachOutFile extends _RepositoryLicensedFile {
  _RepositoryReachOutFile(super.parent, super.io, this.offset);

  final int offset;

  @override
  Iterable<Assignment> assignLicenses() {
    _RepositoryDirectory? directory = parent;
    int index = offset;
    while (index > 1) {
      if (directory == null) {
        break;
      }
      directory = directory.parent;
      index -= 1;
    }
    return directory!
        .nearestLicensesFor(name)
        .map((License license) => license.assignLicenses(io.fullName, parent!));
  }
}

class _RepositoryReachOutDirectory extends _RepositoryDirectory {
  _RepositoryReachOutDirectory(
    _RepositoryDirectory super.parent,
    super.io,
    this.reachOutFilenames,
    this.offset,
  );

  final Set<String> reachOutFilenames;
  final int offset;

  @override
  _RepositoryFile createFile(fs.IoNode entry) {
    if (reachOutFilenames.contains(entry.name)) {
      return _RepositoryReachOutFile(this, entry as fs.File, offset);
    }
    return super.createFile(entry);
  }
}

class _RepositoryInjaJsonFile extends _RepositorySourceFile {
  _RepositoryInjaJsonFile(super.parent, super.io);
  @override
  License extractInternalLicense() {
    throw '$this does not have a clearly extractable license';
  }

  static final RegExp _pattern = RegExp(
    r'^(.*?)' // 1
    r'Licensed under the MIT License <http://opensource\.org/licenses/MIT>\.\n'
    r'SPDX-License-Identifier: MIT\n'
    r'(Copyright \(c\) 2013-2019 Niels Lohmann <http://nlohmann\.me>\.)\n' // 2
    r'\n'
    r'(Permission is hereby  granted, free of charge, to any  person obtaining a copy\n' // 3
    r'of this software and associated  documentation files \(the "Software"\), to deal\n'
    r'in the Software  without restriction, including without  limitation the rights\n'
    r'to  use, copy,  modify, merge,  publish, distribute,  sublicense, and/or  sell\n'
    r'copies  of  the Software,  and  to  permit persons  to  whom  the Software  is\n'
    r'furnished to do so, subject to the following conditions:\n'
    r'\n'
    r'The above copyright notice and this permission notice shall be included in all\n'
    r'copies or substantial portions of the Software\.\n'
    r'\n'
    r'THE SOFTWARE  IS PROVIDED "AS  IS", WITHOUT WARRANTY  OF ANY KIND,  EXPRESS OR\n'
    r'IMPLIED,  INCLUDING BUT  NOT  LIMITED TO  THE  WARRANTIES OF  MERCHANTABILITY,\n'
    r'FITNESS FOR  A PARTICULAR PURPOSE AND  NONINFRINGEMENT\. IN NO EVENT  SHALL THE\n'
    r'AUTHORS  OR COPYRIGHT  HOLDERS  BE  LIABLE FOR  ANY  CLAIM,  DAMAGES OR  OTHER\n'
    r'LIABILITY, WHETHER IN AN ACTION OF  CONTRACT, TORT OR OTHERWISE, ARISING FROM,\n'
    r'OUT OF OR IN CONNECTION WITH THE SOFTWARE  OR THE USE OR OTHER DEALINGS IN THE\n'
    r'SOFTWARE\.)\n'
    r'(.*?)' // 4
    r' \* Created by Evan Nemerson <evan@nemerson\.com>\n'
    r' \*\n'
    r' \* To the extent possible under law, the author\(s\) have dedicated all\n'
    r' \* copyright and related and neighboring rights to this software to\n'
    r' \* the public domain worldwide\. This software is distributed without\n'
    r' \* any warranty\.\n'
    r' \*\n'
    r' \* For details, see <http://creativecommons\.org/publicdomain/zero/1\.0/>\.\n'
    r' \* SPDX-License-Identifier: CC0-1\.0\n'
    r'(.*?)' // 5
    r'The code is distributed under the MIT license, (Copyright \(c\) 2009 Florian Loitsch\.)\n' // 6
    r'(.*?)' // 7
    r'    @copyright (Copyright \(c\) 2008-2009 Bjoern Hoehrmann <bjoern@hoehrmann\.de>)\n' // 8
    r'(.*?)' // 9
    r'    `copyright` \| The copyright line for the library as string\.\n'
    r'(.*?)' // 10
    r'        result\["copyright"\] = "\(C\) 2013-2020 Niels Lohmann";\n'
    r'(.*)$', // 11
    dotAll: true, // isMultiLine is false, so ^ and $ only match start and end.
  );

  @override
  Iterable<Assignment> assignLicenses() {
    if (_internalLicenses == null) {
      final Match? match = _pattern.matchAsPrefix(_contents);
      if (match == null) {
        throw '${io.fullName} has changed contents.';
      }
      final String license = match.group(3)!;
      _internalLicenses =
          match.groups(const <int>[2, 6, 8]).map<License>((String? copyright) {
            assert(copyright!.contains('Copyright'));
            return License.fromCopyrightAndLicense(
              copyright!,
              license,
              LicenseType.mit,
              origin: io.fullName,
            );
          }).toList();
      assert(
        !match
            .groups(const <int>[1, 4, 5, 7, 9, 10, 11])
            .any((String? text) => text!.contains(copyrightMentionPattern)),
      );
    }
    return _internalLicenses!.map(
      (License license) => license.assignLicenses(io.fullName, parent!),
    );
  }
}

/// The `src/` directory created by gclient sync (a.k.a. buildroot).
///
/// This directory is the parent of the flutter/engine repository. The license
/// crawler begins its search starting from this directory and recurses down
/// from it.
///
/// This is not the root of the flutter/engine repository.
/// [_RepositoryFlutterDirectory] represents that.
class _EngineSrcDirectory extends _RepositoryDirectory {
  _EngineSrcDirectory(fs.Directory io) : super(null, io);

  @override
  String get libraryName {
    throw 'Package failed to determine library name.';
  }

  @override
  bool get isLicenseRoot => true;

  @override
  bool get subdirectoriesAreLicenseRoots => false;

  @override
  bool shouldRecurse(fs.IoNode entry) {
    return entry.name !=
            'third_party' // all third_party components have been moved to flutter/third_party
            &&
        super.shouldRecurse(entry);
  }

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'flutter') {
      return _RepositoryFlutterDirectory(this, entry);
    }
    if (entry.name == 'gpu') {
      return _RepositoryGpuShimDirectory(this, entry);
    }
    if (entry.name == 'fuchsia') {
      return _RepositoryFuchsiaDirectory(this, entry);
    }
    return super.createSubdirectory(entry);
  }

  @override
  List<_RepositoryDirectory> get virtualSubdirectories {
    // Dart and Skia are updated more frequently than other third party
    // libraries and is therefore represented as a separate top-level component.
    final fs.Directory flutterNode = findChildDirectory(ioDirectory, 'flutter')!;
    final fs.Directory flutterThirdPartyNode = findChildDirectory(flutterNode, 'third_party')!;
    final fs.Directory skiaNode = findChildDirectory(flutterThirdPartyNode, 'skia')!;
    final fs.Directory dartNode = findChildDirectory(flutterThirdPartyNode, 'dart')!;

    return <_RepositoryDirectory>[
      _RepositoryDartDirectory(this, dartNode),
      _RepositorySkiaDirectory(this, skiaNode),
    ];
  }
}

class _RepositoryGenericThirdPartyDirectory extends _RepositoryDirectory {
  _RepositoryGenericThirdPartyDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  bool get subdirectoriesAreLicenseRoots => true;
}

class _RepositoryExpatDirectory extends _RepositoryDirectory {
  _RepositoryExpatDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  bool get isLicenseRootException => true;

  @override
  bool get subdirectoriesAreLicenseRoots => true;
}

class _RepositoryFreetypeDirectory extends _RepositoryDirectory {
  _RepositoryFreetypeDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  List<License> nearestLicensesFor(String name) {
    final List<License> result = super.nearestLicensesFor(name);
    if (result.isEmpty) {
      final License? license = nearestLicenseWithName('LICENSE.TXT');
      assert(license != null);
      if (license != null) {
        return <License>[license];
      }
    }
    return result;
  }

  @override
  License? nearestLicenseOfType(LicenseType type) {
    if (type == LicenseType.freetype) {
      final License? result = nearestLicenseWithName('FTL.TXT');
      assert(result != null);
      return result;
    }
    return super.nearestLicenseOfType(type);
  }

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'src') {
      return _RepositoryFreetypeSrcDirectory(this, entry);
    }
    return super.createSubdirectory(entry);
  }
}

class _RepositoryFreetypeSrcDirectory extends _RepositoryDirectory {
  _RepositoryFreetypeSrcDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'gzip') {
      return _RepositoryFreetypeSrcGZipDirectory(this, entry);
    }
    return super.createSubdirectory(entry);
  }
}

class _RepositoryFreetypeSrcGZipDirectory extends _RepositoryDirectory {
  _RepositoryFreetypeSrcGZipDirectory(_RepositoryDirectory super.parent, super.io);

  // advice was to make this directory's inffixed.h file (which has no license)
  // use the license in zlib.h.

  @override
  List<License> nearestLicensesFor(String name) {
    final License? zlib = nearestLicenseWithName('zlib.h');
    assert(zlib != null);
    if (zlib != null) {
      return <License>[zlib];
    }
    return super.nearestLicensesFor(name);
  }

  @override
  License? nearestLicenseOfType(LicenseType type) {
    if (type == LicenseType.zlib) {
      final License? result = nearestLicenseWithName('zlib.h');
      assert(result != null);
      return result;
    }
    return super.nearestLicenseOfType(type);
  }
}

class _RepositoryFallbackRootCertificatesDirectory extends _RepositoryDirectory {
  _RepositoryFallbackRootCertificatesDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  String get officialSourceLocation {
    final system.ProcessResult result = system.Process.runSync('git', <String>[
      'rev-parse',
      'HEAD',
    ], workingDirectory: '$this');
    if (result.exitCode != 0) {
      throw 'Failed to run "git rev-parse HEAD"; got non-zero exit code ${result.exitCode}\nstdout:\n${result.stdout}\nstderr:\n${result.stderr}';
    }

    final String rev = result.stdout as String;
    return 'https://dart.googlesource.com/sdk/+/$rev/third_party/fallback_root_certificates/';
  }
}

class _RepositoryZLibDirectory extends _RepositoryDirectory {
  _RepositoryZLibDirectory(_RepositoryDirectory super.parent, super.io);

  // Some files in this directory refer to "MiniZip_info.txt".
  // As best we can tell, that refers to a file that itself includes the
  // exact same license text as in LICENSE.

  @override
  License? nearestLicenseWithName(String name, {String? authors}) {
    if (name == 'MiniZip_info.txt') {
      return super.nearestLicenseWithName('LICENSE', authors: authors)!;
    }
    return super.nearestLicenseWithName(name, authors: authors);
  }

  @override
  License? nearestLicenseOfType(LicenseType type) {
    if (type == LicenseType.zlib) {
      return nearestLicenseWithName('LICENSE')!;
    }
    return super.nearestLicenseOfType(type);
  }
}

class _RepositoryDartDirectory extends _RepositoryDirectory {
  _RepositoryDartDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  bool get isLicenseRoot => true;

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'third_party') {
      return _RepositoryDartThirdPartyDirectory(this, entry);
    }
    return super.createSubdirectory(entry);
  }
}

class _RepositoryDartThirdPartyDirectory extends _RepositoryGenericThirdPartyDirectory {
  _RepositoryDartThirdPartyDirectory(super.parent, super.io);

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'fallback_root_certificates') {
      return _RepositoryFallbackRootCertificatesDirectory(this, entry);
    }
    return super.createSubdirectory(entry);
  }
}

class _RepositorySkiaDirectory extends _RepositoryDirectory {
  _RepositorySkiaDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  bool get isLicenseRoot => true;

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'third_party') {
      return _RepositorySkiaThirdPartyDirectory(this, entry);
    }
    return super.createSubdirectory(entry);
  }
}

class _RepositorySkiaThirdPartyDirectory extends _RepositoryGenericThirdPartyDirectory {
  _RepositorySkiaThirdPartyDirectory(super.parent, super.io);

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'ktx') {
      return _RepositoryReachOutDirectory(this, entry, const <String>{'ktx.h', 'ktx.cpp'}, 2);
    }
    if (entry.name == 'libmicrohttpd') {
      return _RepositoryReachOutDirectory(this, entry, const <String>{'MHD_config.h'}, 2);
    }
    if (entry.name == 'libwebp') {
      return _RepositorySkiaLibWebPDirectory(this, entry);
    }
    if (entry.name == 'libsdl') {
      return _RepositorySkiaLibSdlDirectory(this, entry);
    }
    return super.createSubdirectory(entry);
  }
}

class _RepositorySkiaLibWebPDirectory extends _RepositoryDirectory {
  _RepositorySkiaLibWebPDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'webp') {
      return _RepositoryReachOutDirectory(this, entry, const <String>{'config.h'}, 3);
    }
    return super.createSubdirectory(entry);
  }
}

class _RepositorySkiaLibSdlDirectory extends _RepositoryDirectory {
  _RepositorySkiaLibSdlDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  bool get isLicenseRootException => true;
}

class _RepositoryBoringSSLDirectory extends _RepositoryDirectory {
  _RepositoryBoringSSLDirectory(_RepositoryDirectory super.parent, super.io);

  // This directory contains boringssl itself in the 'src' subdirectory,
  // and the rest of files are code generated from tools in that directory.
  // We redirect any license queries to the src/LICENSE file.

  _RepositoryDirectory get src => getChildByName('src') as _RepositoryDirectory;

  @override
  List<License> nearestLicensesFor(String name) {
    final List<License> result = super.nearestLicensesFor(name);
    if (result.isEmpty) {
      return (src.getChildByName('LICENSE') as _RepositoryLicenseFile).licenses;
    }
    return result;
  }

  @override
  License? nearestLicenseOfType(LicenseType type) {
    assert(!src._canGoUp());
    return super.nearestLicenseOfType(type) ?? src.nearestLicenseOfType(type);
  }

  @override
  License? nearestLicenseWithName(String name, {String? authors}) {
    assert(!src._canGoUp());
    final License? result =
        super.nearestLicenseWithName(name, authors: authors) ??
        src.nearestLicenseWithName(name, authors: authors);
    return result;
  }

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'src') {
      // This is the actual BoringSSL library.
      return _RepositoryBoringSSLSourceDirectory(this, entry);
    }
    return super.createSubdirectory(entry);
  }
}

class _RepositoryBoringSSLSourceDirectory extends _RepositoryDirectory {
  _RepositoryBoringSSLSourceDirectory(_RepositoryDirectory super.parent, super.io);

  // This directory is called "src" because of the way we import boringssl. The
  // parent is called "boringssl". Since we are a licenseRoot, the default
  // "libraryName" implementation would use "src", so we force it to go up here.
  @override
  String get libraryName => parent!.libraryName;

  @override
  bool get isLicenseRoot => true;
}

class _RepositoryFlutterDirectory extends _RepositoryDirectory {
  _RepositoryFlutterDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  String get libraryName => 'engine';

  @override
  bool get isLicenseRoot => true;

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'third_party') {
      return _RepositoryFlutterThirdPartyDirectory(this, entry);
    }
    return _RepositoryDirectory(this, entry);
  }
}

class _RepositoryFlutterThirdPartyDirectory extends _RepositoryGenericThirdPartyDirectory {
  _RepositoryFlutterThirdPartyDirectory(super.parent, super.io);

  @override
  bool shouldRecurse(fs.IoNode entry) {
    return entry.name !=
            'skia' // handled as a virtual directory of the root
            &&
        entry.name !=
            'dart' // handled as a virtual directory of the root
            &&
        super.shouldRecurse(entry);
  }

  @override
  _RepositoryDirectory createSubdirectory(fs.Directory entry) {
    if (entry.name == 'boringssl') {
      return _RepositoryBoringSSLDirectory(this, entry);
    }
    if (entry.name == 'expat') {
      return _RepositoryExpatDirectory(this, entry);
    }
    if (entry.name == 'freetype2') {
      return _RepositoryFreetypeDirectory(this, entry);
    }
    if (entry.name == 'vulkan-deps') {
      return _RepositoryGenericThirdPartyDirectory(this, entry);
    }
    if (entry.name == 'zlib') {
      return _RepositoryZLibDirectory(this, entry);
    }
    return super.createSubdirectory(entry);
  }
}

class _RepositoryFuchsiaDirectory extends _RepositoryDirectory {
  _RepositoryFuchsiaDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  String get libraryName => 'fuchsia_sdk';

  @override
  bool get isLicenseRoot => true;
}

class _RepositoryGpuShimDirectory extends _RepositoryDirectory {
  _RepositoryGpuShimDirectory(_RepositoryDirectory super.parent, super.io);

  @override
  String get libraryName => 'engine';
}

/// The license tool directory.
///
/// This is a special-case root node that is not used for license aggregation,
/// but simply to compute a signature for the license tool itself. When this
/// signature changes, we force re-run license collection for all components in
/// order to verify the tool itself still produces the same output.
class _RepositoryFlutterLicenseToolDirectory extends _RepositoryDirectory {
  _RepositoryFlutterLicenseToolDirectory(fs.Directory io) : super(null, io);

  @override
  bool shouldRecurse(fs.IoNode entry) {
    return entry.name != 'data' && super.shouldRecurse(entry);
  }
}

// BOOTSTRAPPING LOGIC

fs.Directory? findChildDirectory(fs.Directory parent, String name) {
  return parent.walk.firstWhereOrNull(
        // from IterableExtension in package:collection
        (fs.IoNode child) => child.name == name,
      )
      as fs.Directory?;
}

class _Progress {
  _Progress(this.max, {this.quiet = false}) : millisecondsBetweenUpdates = quiet ? 10000 : 0 {
    // This may happen when a git client contains left-over empty component
    // directories after DEPS file changes.
    if (max <= 0) {
      throw ArgumentError('Progress.max must be > 0 but was: $max');
    }
  }

  final int max;
  final bool quiet;
  final int millisecondsBetweenUpdates;
  int get withLicense => _withLicense;
  int _withLicense = 0;
  int get withoutLicense => _withoutLicense;
  int _withoutLicense = 0;
  String get label => _label;
  String _label = '';
  int _lastLength = 0;
  set label(String value) {
    if (value.length > 60) {
      value = '.../${value.substring(math.max(0, value.lastIndexOf('/', value.length - 45) + 1))}';
    }
    if (_label != value) {
      _label = value;
      update();
    }
  }

  void advance({required bool success}) {
    if (success) {
      _withLicense += 1;
    } else {
      _withoutLicense += 1;
    }
    update();
  }

  Stopwatch? _lastUpdate;
  void update({bool flush = false}) {
    if (_lastUpdate == null ||
        _lastUpdate!.elapsedMilliseconds >= millisecondsBetweenUpdates ||
        flush) {
      _lastUpdate ??= Stopwatch();
      if (!quiet) {
        final String line = toString();
        system.stderr.write('\r$line');
        if (_lastLength > line.length) {
          system.stderr.write(' ' * (_lastLength - line.length));
        }
        _lastLength = line.length;
      }
      _lastUpdate!.reset();
      _lastUpdate!.start();
    }
  }

  void flush() => update(flush: true);
  bool get hadErrors => _withoutLicense > 0;
  @override
  String toString() {
    final int percent = (100.0 * (_withLicense + _withoutLicense) / max).round();
    return '${(_withLicense + _withoutLicense).toString().padLeft(10)} of ${max.toString().padRight(6)} '
        '${'█' * (percent ~/ 10)}${'░' * (10 - (percent ~/ 10))} $percent% '
        '${_withoutLicense > 0 ? "($_withoutLicense missing licenses) " : ""}'
        '$label';
  }
}

final RegExp _signaturePattern = RegExp(
  r'^Signature: (\w+)$',
  multiLine: true,
  expectNoMatch: true,
);

/// Reads the signature from a golden file.
String? _readSignature(String goldenPath) {
  try {
    final system.File goldenFile = system.File(goldenPath);
    if (!goldenFile.existsSync()) {
      system.stderr.writeln('    Could not find signature file ($goldenPath).');
      return null;
    }
    final String goldenSignature = goldenFile.readAsStringSync();
    final Match? goldenMatch = _signaturePattern.matchAsPrefix(goldenSignature);
    if (goldenMatch != null) {
      return goldenMatch.group(1);
    }
    system.stderr.writeln('    Signature file ($goldenPath) did not match expected pattern.');
  } on system.FileSystemException {
    system.stderr.writeln('    Failed to read signature file ($goldenPath).');
    return null;
  }
  return null;
}

/// Writes a signature to an [system.IOSink] in the expected format.
void _writeSignature(String signature, system.IOSink sink) {
  sink.writeln('Signature: $signature\n');
}

// Checks for changes to the license tool itself.
//
// Returns true if changes are detected.
Future<bool> _computeLicenseToolChanges(
  _RepositoryDirectory root, {
  required String goldenSignaturePath,
  required String outputSignaturePath,
}) async {
  final fs.Directory flutterNode = findChildDirectory(root.ioDirectory, 'flutter')!;
  final fs.Directory toolsNode = findChildDirectory(flutterNode, 'tools')!;
  final fs.Directory licenseNode = findChildDirectory(toolsNode, 'licenses')!;
  final _RepositoryDirectory licenseToolDirectory = _RepositoryFlutterLicenseToolDirectory(
    licenseNode,
  );
  final String toolSignature = await licenseToolDirectory.signature;
  final system.IOSink sink = system.File(outputSignaturePath).openWrite();
  _writeSignature(toolSignature, sink);
  await sink.close();
  final String? goldenSignature = _readSignature(goldenSignaturePath);
  return toolSignature != goldenSignature;
}

/// Collects licenses for the specified component.
///
/// If [writeSignature] is set, the signature is written to the output file.
/// If [force] is set, collection is run regardless of whether or not the signature matches.
Future<void> _collectLicensesForComponent(
  _RepositoryDirectory componentRoot, {
  required String inputGoldenPath,
  String? outputGoldenPath,
  required bool writeSignature,
  required bool force,
  required bool quiet,
}) async {
  final String signature = await componentRoot.signature;
  if (writeSignature) {
    // Check whether the golden file matches the signature of the current contents of this directory.
    // (We only do this for components where we write the signature, since if there's no signature,
    // there's no point trying to read it...)
    final String? goldenSignature = _readSignature(inputGoldenPath);
    if (!force && goldenSignature == signature) {
      system.stderr.writeln('    Skipping this component - no change in signature');
      return;
    }
  }

  final _Progress progress = _Progress(componentRoot.fileCount, quiet: quiet);

  final system.File outFile = system.File(outputGoldenPath!);
  final system.IOSink sink = outFile.openWrite();
  if (writeSignature) {
    _writeSignature(signature, sink);
  }

  final List<GroupedLicense> licenses = groupLicenses(componentRoot.assignLicenses(progress));
  if (progress.hadErrors) {
    throw 'Had failures while collecting licenses.';
  }
  progress.label = 'Dumping results...';
  progress.flush();
  final List<String> output =
      licenses.map((GroupedLicense license) => license.toStringDebug()).toList();
  for (int index = 0; index < output.length; index += 1) {
    // The strings we look for here are strings which we do not expect to see in
    // any of the licenses we use. They either represent examples of misparsing
    // licenses (issues we've previously run into and fixed), or licenses we
    // know we are trying to avoid (e.g. the GPL, or licenses that only apply to
    // test content which shouldn't get built at all).
    // If you find that one of these tests is getting hit, and it's not obvious
    // to you why the relevant license is a problem, please ask around (e.g. try
    // asking Hixie). Do not merely remove one of these checks, sometimes the
    // issues involved are relatively subtle.
    if (output[index].contains('Version: MPL 1.1/GPL 2.0/LGPL 2.1')) {
      throw 'Unexpected trilicense block found in:\n${output[index]}';
    }
    if (output[index].contains(
      'The contents of this file are subject to the Mozilla Public License Version',
    )) {
      throw 'Unexpected MPL block found in:\n${output[index]}';
    }
    if (output[index].contains('You should have received a copy of the GNU')) {
      throw 'Unexpected GPL block found in:\n${output[index]}';
    }
    if (output[index].contains('Contents of this folder are ported from')) {
      throw 'Unexpected block found in:\n${output[index]}';
    }
    if (output[index].contains(
      'https://github.com/w3c/web-platform-tests/tree/master/selectors-api',
    )) {
      throw 'Unexpected W3C content found in:\n${output[index]}';
    }
    if (output[index].contains(
      'http://www.w3.org/Consortium/Legal/2008/04-testsuite-copyright.html',
    )) {
      throw 'Unexpected W3C copyright found in:\n${output[index]}';
    }
    if (output[index].contains('It is based on commit')) {
      throw 'Unexpected content found in:\n${output[index]}';
    }
    if (output[index].contains(
      'The original code is covered by the dual-licensing approach described in:',
    )) {
      throw 'Unexpected old license reference found in:\n${output[index]}';
    }
    if (output[index].contains('must choose')) {
      throw 'Unexpected indecisiveness found in:\n${output[index]}';
    }
  }
  sink.writeln(output.join('\n'));
  sink.writeln('Total license count: ${licenses.length}');

  await sink.close();
  progress.label = 'Done.';
  progress.flush();
  system.stderr.writeln();
}

// MAIN

Future<void> main(List<String> arguments) async {
  final ArgParser parser =
      ArgParser()
        ..addOption('src', help: 'The root of the engine source.')
        ..addOption(
          'out',
          help: 'The directory where output is written. (Ignored if used with --release.)',
        )
        ..addOption('golden', help: 'The directory containing golden results.')
        ..addFlag('quiet', help: 'If set, the diagnostic output is much less verbose.')
        ..addFlag('verbose', help: 'If set, print additional information to help with development.')
        ..addFlag('release', help: 'Print output in the format used for product releases.');

  final ArgResults argResults = parser.parse(arguments);
  final bool quiet = argResults['quiet'] as bool;
  final bool verbose = argResults['verbose'] as bool;
  final bool releaseMode = argResults['release'] as bool;
  if (argResults['src'] == null) {
    print('Flutter license script: Must provide --src directory');
    print(parser.usage);
    system.exit(1);
  }
  if (!releaseMode) {
    if (argResults['out'] == null || argResults['golden'] == null) {
      print(
        'Flutter license script: Must provide --out and --golden directories in non-release mode',
      );
      print(parser.usage);
      system.exit(1);
    }
    if (!system.FileSystemEntity.isDirectorySync(argResults['golden'] as String)) {
      print('Flutter license script: Golden directory does not exist');
      print(parser.usage);
      system.exit(1);
    }
    final system.Directory out = system.Directory(argResults['out'] as String);
    if (!out.existsSync()) {
      out.createSync(recursive: true);
    }
  }

  try {
    system.stderr.writeln('Finding files...');
    final fs.FileSystemDirectory rootDirectory = fs.FileSystemDirectory.fromPath(
      argResults['src'] as String,
    );
    final _RepositoryDirectory root = _EngineSrcDirectory(rootDirectory);

    if (releaseMode) {
      system.stderr.writeln('Collecting licenses...');
      final _Progress progress = _Progress(root.fileCount, quiet: quiet);
      final List<GroupedLicense> licenses = groupLicenses(root.assignLicenses(progress));
      if (progress.hadErrors) {
        throw 'Had failures while collecting licenses.';
      }
      progress.label = 'Dumping results...';
      progress.flush();
      final String output = licenses
          .where((GroupedLicense license) => license.body.isNotEmpty)
          .map((GroupedLicense license) => license.toStringFormal())
          .join('\n${"-" * 80}\n');
      print(output);
      progress.label = 'Done.';
      progress.flush();
      system.stderr.writeln();
    } else {
      // If changes are detected to the license tool itself, force collection
      // for all components in order to check we're still generating correct
      // output.
      const String toolSignatureFilename = 'tool_signature';
      final bool forceRunAll = await _computeLicenseToolChanges(
        root,
        goldenSignaturePath: path.join(argResults['golden'] as String, toolSignatureFilename),
        outputSignaturePath: path.join(argResults['out'] as String, toolSignatureFilename),
      );
      if (forceRunAll) {
        system.stderr.writeln(
          'Detected changes to license tool. Forcing license collection for all components.',
        );
      }
      final List<String> usedGoldens = <String>[];
      bool isFirstComponent = true;
      for (final _RepositoryDirectory component in root.subdirectories) {
        system.stderr.writeln('Collecting licenses for ${component.io.name}');
        _RepositoryDirectory componentRoot;
        if (isFirstComponent) {
          // For the first component, we can use the results of the initial repository crawl.
          isFirstComponent = false;
          componentRoot = component;
        } else {
          // For other components, we need a clean repository that does not
          // contain any state left over from previous components.
          componentRoot = _EngineSrcDirectory(
            rootDirectory,
          ).subdirectories.firstWhere((_RepositoryDirectory dir) => dir.name == component.name);
        }
        final String goldenFileName = 'licenses_${component.io.name}';
        await _collectLicensesForComponent(
          componentRoot,
          inputGoldenPath: path.join(argResults['golden'] as String, goldenFileName),
          outputGoldenPath: path.join(argResults['out'] as String, goldenFileName),
          writeSignature: component.io.name != 'flutter',
          // Always run the full license check on the flutter tree. The flutter
          // tree is relatively small and changes frequently in ways that do not
          // affect the license output, and we don't want to require updates to
          // the golden signature for those changes.
          force: forceRunAll || component.io.name == 'flutter',
          quiet: quiet,
        );
        usedGoldens.add(goldenFileName);
      }
      final Set<String> unusedGoldens =
          system.Directory(argResults['golden'] as String)
              .listSync()
              .map<String>((system.FileSystemEntity file) => path.basename(file.path))
              .where((String name) => name.startsWith('licenses_'))
              .toSet()
            ..removeAll(usedGoldens);
      if (unusedGoldens.isNotEmpty) {
        system.stderr.writeln(
          'The following golden files in ${argResults['golden']} are unused and need to be deleted:',
        );
        unusedGoldens.map((String s) => ' * $s').forEach(system.stderr.writeln);
        system.exit(1);
      }
      // write to disk the list of files we did _not_ cover, so it's easier to catch in diffs
      final String excluded = (_RepositoryDirectory._excluded
              .map((fs.IoNode node) => node.fullName)
              .toSet()
              .toList()
            ..sort())
          .join('\n');
      system.File(
        path.join(argResults['out'] as String, 'excluded_files'),
      ).writeAsStringSync('$excluded\n');
    }
  } catch (e, stack) {
    system.stderr.writeln();
    system.stderr.writeln('failure: $e\n$stack');
    system.stderr.writeln('aborted.');
    system.exit(1);
  } finally {
    if (verbose) {
      RegExp.printDiagnostics();
    }
  }
}
