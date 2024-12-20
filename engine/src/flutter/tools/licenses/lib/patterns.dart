// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// ignore_for_file: use_raw_strings, prefer_interpolation_to_compose_strings

import 'dart:core' hide RegExp;

import 'regexp_debug.dart';

// COMMON PATTERNS

// _findLicenseBlocks in licenses.dart cares about the two captures here
// it also handles the first line being slightly different ("/*" vs " *" for example)
const String kIndent = r'^((?:[-;@#<!.\\"/* ]*(?:REM[-;@#<!.\\"/* ]*)?[-;@#<!.\"/*]+)?)( *)';

final RegExp stripDecorations = RegExp(
  r'^((?:(?:[-;@#<!.\\"/* ]*(?:REM[-;@#<!.\\"/* ]*)?[-;@#<!.\"/*]+)?) *)(.*?)[ */]*$',
  multiLine: true,
  caseSensitive: false,
);

// _reformat patterns (see also irrelevantText below)
final RegExp leadingDecorations = RegExp(r'^(?://|#|;|@|REM\b|/?\*|--| )');
final RegExp fullDecorations = RegExp(r'^ *[-=]{2,}$');
final RegExp trailingColon = RegExp(r':(?: |\*|/|\n|=|-)*$', expectNoMatch: true);

final RegExp newlinePattern = RegExp(r'\r\n?');
final RegExp nonSpace = RegExp('[^ ]');

// Used to check if something that we expect to contain a copyright does contain a copyright. Should be very easy to match.
final RegExp anySlightSignOfCopyrights = RegExp(r'©|copyright|\(c\)', caseSensitive: false);

// Used to check if something that we don't expect to contain a copyright does contain a copyright. Should be harder to match.
// The copyrightMentionOkPattern below is used to further exclude files that do match this.
final RegExp copyrightMentionPattern = RegExp(
  r'©|copyright [0-9]|\(c\) [0-9]|copyright \(c\)',
  caseSensitive: false,
);

// Used to check if something that we don't expect to contain a license does contain a license.
final RegExp licenseMentionPattern = RegExp(
  r'Permission is hereby granted|warrant[iy]|derivative works|redistribution|copyleft',
  caseSensitive: false,
  expectNoMatch: true,
);

// Used to allow-list files that match copyrightMentionPattern due to false positives.
// Files that match this will not be caught if they add new otherwise unmatched licenses!
final RegExp copyrightMentionOkPattern = RegExp(
  // if a (multiline) block matches this, we ignore it even if it matches copyrightMentionPattern/licenseMentionPattern
  r'(?:These are covered by the following copyright:'
  r'|\$YEAR' // clearly part of a template string
  r'|LICENSE BLOCK' // MPL license block header/footer
  r'|2117.+COPYRIGHT' // probably U+2117 mention...
  r'|// The copyright below was added in 2009, but I see no record'
  r'|This ICU code derived from:'
  r'|the contents of which are also included in zip.h' // seen in minizip's unzip.c, but the upshot of the crazy license situation there is that we don't have to do anything
  r'|" inflate 1\.3\.0\.\d Copyright 1995-2023 Mark Adler ";'
  r'|" deflate 1\.3\.0\.\d Copyright 1995-2023 Jean-loup Gailly and Mark Adler ";'
  r'|const char zip_copyright\[\] =" zip 1\.01 Copyright 1998-2004 Gilles Vollant - http://www.winimage.com/zLibDll";'
  r'|#define JCOPYRIGHT_SHORT "Copyright \(C\) 1991-2016 The libjpeg-turbo Project and many others"'
  r"|r'[^']*©[^']*'" // e.g. flutter/third_party/web_locale_keymap/lib/web_locale_keymap/key_mappings.g.dart
  // the following are all bits from ICU source files
  // (you'd think ICU would be more consistent with its copyrights given how much
  // source code there is in ICU just for generating copyrights)
  r'|VALUE "LegalCopyright"'
  r'|const char inflate_copyright\[\] =\n *" inflate 1\.2\.11 Copyright 1995-2017 Mark Adler ";' // found in some freetype files
  r'|" Copyright \(C\) 2016 and later: Unicode, Inc\. and others\. License & terms of use: http://www\.unicode\.org/copyright\.html "'
  r'|"\* / \\\\& ⁊ # % † ‡ ‧ ° © ® ™]"'
  // It is very likely for copyright-like chars to appear in Unicode
  // data files in contexts unrelated to copyright.
  // Imagine that, who would have thought!
  //                      v--- this one.
  r'|"\\\\& ⁊ # % † ‡ ‧ ° © ® ™]"'
  r'|" \*   Copyright \(C\) International Business Machines\n"'
  r'|fprintf\(out, "// Copyright \(C\) 2016 and later: Unicode, Inc\. and others\.\\n"\);'
  r'|fprintf\(out, "// License & terms of use: http://www\.unicode\.org/copyright\.html\\n\\n"\);'
  r'|fprintf\(out, "/\*\* Copyright \(C\) 2007-2016, International Business Machines Corporation and Others\. All Rights Reserved\. \*\*/\\n\\n"\);'
  r'|\\\(C\\\) ↔ ©;'
  r'|" \*   Copyright \(C\) International Business Machines\\n"'
  r'|"%s Copyright \(C\) %d and later: Unicode, Inc\. and others\.\\n"'
  r'|"%s License & terms of use: http://www\.unicode\.org/copyright\.html\\n",'
  r'|"%s Copyright \(C\) 1999-2016, International Business Machines\\n"'
  r'|"%s Corporation and others\.  All Rights Reserved\.\\n",'
  r'|\\mainpage' // q.v. third_party/vulkan_memory_allocator/include/vk_mem_alloc.h
  r'|" \* Copyright 2017 Google Inc\.\\n"'
  r')',
  caseSensitive: false,
  multiLine: true,
);

// Used to extact the "authors" pattern for copyrights that use that (like Flutter's).
final RegExp authorPattern = RegExp(
  r'Copyright .+(The .+ Authors)\. +All rights reserved\.',
  caseSensitive: false,
);

// Lines that are found at the top of license files but aren't strictly part of the license.
final RegExp licenseHeaders = RegExp(
  r'.+ Library\n|' // e.g. "xxHash Library"
  r'.*(BSD|MIT|SGI).* (?:License|LICENSE)(?: [A-Z])?(?: \(.+\))?:? *\n|' // e.g. "BSD 3-Clause License", "The MIT License (MIT):", "SGI FREE SOFTWARE LICENSE B (Version 2.0, Sept. 18, 2008)"; NOT "Apache License" (that _is_ part of the license)
  r'All MurmurHash source files are placed in the public domain\.\n|'
  r'The license below applies to all other code in SMHasher:\n|'
  r'amalgamate.py - Amalgamate C source and header files\n|'
  r'\n',
);

// copyright blocks start with the first line matching this
// (used by _findLicenseBlocks)
final List<RegExp> copyrightStatementLeadingPatterns = <RegExp>[
  RegExp(
    r'^ *(?:Portions(?: created by the Initial Developer)?(?: are)? )?Copyright.+$',
    caseSensitive: false,
  ),
  RegExp(r'^.*(?:All )?rights? reserved\.$', caseSensitive: false),
  RegExp(r'^.*© [0-9]{4} .+$'),
];

// patterns used by _splitLicense to extend the copyright block
final RegExp halfCopyrightPattern = RegExp(
  r'^(?: *(?:Copyright(?: \(c\))? [-0-9, ]+(?: by)?|Written [0-9]+)[ */]*)$',
  caseSensitive: false,
);
final RegExp trailingComma = RegExp(r',[ */]*$');

// copyright blocks end with the last line that matches this, rest is considered license
// (used by _splitLicense)
final List<RegExp> copyrightStatementPatterns = <RegExp>[
  ...copyrightStatementLeadingPatterns,
  RegExp(r'^(?:Google )?Author\(?s?\)?: .+', caseSensitive: false),
  RegExp(r'^Written by .+', caseSensitive: false),
  RegExp(r'^Originally written by .+', caseSensitive: false),
  RegExp(r"^based on (?:code in )?['`][^'`]+['`]$", caseSensitive: false),
  RegExp(r'^Based on .+, written by .+, [0-9]+\.$', caseSensitive: false),
  RegExp(r'^(?:Based on the )?x86 SIMD extension for IJG JPEG library(?: - version [0-9.]+|,)?$'),
  RegExp(r'^Derived from [a-z._/]+$'),
  RegExp(r'^ *This is part of .+, a .+ library\.$'),
  RegExp(r'^(?:Modification )?[Dd]eveloped [-0-9]+ by .+\.$', caseSensitive: false),
  RegExp(r'^Modified .+[:.]$', caseSensitive: false),
  RegExp(r'^(?:[^ ]+ )?Modifications:$', caseSensitive: false),
  RegExp(r'^ *Modifications for', caseSensitive: false),
  RegExp(r'^ *Modifications of', caseSensitive: false),
  RegExp(r'^Modifications Copyright \(C\) .+', caseSensitive: false),
  RegExp(r'^\(Royal Institute of Technology, Stockholm, Sweden\)\.$'),
  RegExp(r'^FT_Raccess_Get_HeaderInfo\(\) and raccess_guess_darwin_hfsplus\(\) are$'),
  RegExp(r'^derived from ftobjs\.c\.$'),
  // RegExp(r'^ *Condition of use and distribution are the same than zlib :$'),
  RegExp(
    r'^Unicode and the Unicode Logo are registered trademarks of Unicode, Inc\. in the U.S. and other countries\.$',
  ),
  RegExp(r'^$'),
];

// patterns that indicate we're running into another license
final List<RegExp> licenseFragments = <RegExp>[
  RegExp(r'SUCH DAMAGE\.'),
  RegExp(r'found in the LICENSE file'),
  RegExp(r'This notice may not be removed'),
  RegExp(r'SPDX-License-Identifier'),
  RegExp(r'terms of use'),
  RegExp(r'implied warranty'),
  RegExp(r'^ *For more info read ([^ ]+)$'),
];

const String _linebreak = r' *(?:(?:\*/ *|[*#])?(?:\n\1 *(?:\*/ *)?)*\n\1\2 *)?';
const String _linebreakLoose = r' *(?:(?:\*/ *|[*#])?\r?\n(?:-|;|#|<|!|/|\*| |REM)*)*';

// LICENSE RECOGNIZERS

final RegExp lrLLVM = RegExp(
  r'--- LLVM Exceptions to the Apache 2\.0 License ----$',
  multiLine: true,
);
final RegExp lrApache = RegExp(r'^(?: |\n)*Apache License\b');
final RegExp lrMPL = RegExp(r'^(?: |\n)*Mozilla Public License Version 2\.0\n');
final RegExp lrGPL = RegExp(r'^(?: |\n)*GNU GENERAL PUBLIC LICENSE\n');
final RegExp lrAPSL = RegExp(
  r'^APPLE PUBLIC SOURCE LICENSE Version 2\.0 +- +August 6, 2003',
  expectNoMatch: true,
);
final RegExp lrMIT = RegExp(
  r'Permission(?: |\n)+is(?: |\n)+hereby(?: |\n)+granted,(?: |\n)+free(?: |\n)+of(?: |\n)+charge,(?: |\n)+to(?: |\n)+any(?: |\n)+person(?: |\n)+obtaining(?: |\n)+a(?: |\n)+copy(?: |\n)+of(?: |\n)+this(?: |\n)+software(?: |\n)+and(?: |\n)+associated(?: |\n)+documentation(?: |\n)+files(?: |\n)+\(the(?: |\n)+"Software"\),(?: |\n)+to(?: |\n)+deal(?: |\n)+in(?: |\n)+the(?: |\n)+Software(?: |\n)+without(?: |\n)+restriction,(?: |\n)+including(?: |\n)+without(?: |\n)+limitation(?: |\n)+the(?: |\n)+rights(?: |\n)+to(?: |\n)+use,(?: |\n)+copy,(?: |\n)+modify,(?: |\n)+merge,(?: |\n)+publish,(?: |\n)+distribute,(?: |\n)+sublicense,(?: |\n)+and/or(?: |\n)+sell(?: |\n)+copies(?: |\n)+of(?: |\n)+the(?: |\n)+Software,(?: |\n)+and(?: |\n)+to(?: |\n)+permit(?: |\n)+persons(?: |\n)+to(?: |\n)+whom(?: |\n)+the(?: |\n)+Software(?: |\n)+is(?: |\n)+furnished(?: |\n)+to(?: |\n)+do(?: |\n)+so,(?: |\n)+subject(?: |\n)+to(?: |\n)+the(?: |\n)+following(?: |\n)+conditions:',
);
final RegExp lrOpenSSL = RegExp(r'OpenSSL.+dual license', expectNoMatch: true);
final RegExp lrBSD = RegExp(
  r'Redistribution(?: |\n)+and(?: |\n)+use(?: |\n)+in(?: |\n)+source(?: |\n)+and(?: |\n)+binary(?: |\n)+forms(?:(?: |\n)+of(?: |\n)+the(?: |\n)+software(?: |\n)+as(?: |\n)+well(?: |\n)+as(?: |\n)+documentation)?,(?: |\n)+with(?: |\n)+or(?: |\n)+without(?: |\n)+modification,(?: |\n)+are(?: |\n)+permitted(?: |\n)+provided(?: |\n)+that(?: |\n)+the(?: |\n)+following(?: |\n)+conditions(?: |\n)+are(?: |\n)+met:',
);
final RegExp lrPNG = RegExp(
  r'This code is released under the libpng license\.|PNG Reference Library License',
);
final RegExp lrBison = RegExp(
  r'This special exception was added by the Free Software Foundation in *\n *version 2.2 of Bison.',
);

// Matching this exactly is important since it's likely there will be similar
// licenses with more requirements.
final RegExp lrZlib = RegExp(
  r"This software is provided 'as-is', without any express or implied\n"
  r'warranty\. +In no event will the authors be held liable for any damages\n'
  r'arising from the use of this software\.\n'
  r'\n'
  r'Permission is granted to anyone to use this software for any purpose,\n'
  r'including commercial applications, and to alter it and redistribute it\n'
  r'freely, subject to the following restrictions:\n'
  r'\n'
  r'1\. The origin of this software must not be misrepresented; you must not\n'
  r'   claim that you wrote the original software\. If you use this software\n'
  r'   in a product, an acknowledgment in the product documentation would(?: be)?\n'
  r'   (?:be )?appreciated but is not required\.\n'
  r'2\. Altered source versions must be plainly marked as such, and must not(?: be)?\n'
  r'   (?: be)?misrepresented as being the original software\.\n'
  r'3\. This notice may not be removed or altered from any source(?: |\n   )distribution\.'
  r'$', // no more terms!
);

// ASCII ART PATTERNS

// If these images are found in a file, they are stripped before we look for license patterns.
final List<List<String>> asciiArtImages =
    <String>[
      r'''
 ___        _
|_ _|_ __  (_) __ _
 | || '_ \ | |/ _` |
 | || | | || | (_| |
|___|_| |_|/ |\__,_|
         |__/''',
    ].map((String image) => image.split('\n')).toList();

// FORWARD REFERENCE

class ForwardReferencePattern {
  ForwardReferencePattern({
    required this.firstPrefixIndex,
    required this.indentPrefixIndex,
    required this.pattern,
    required this.targetPattern,
  });
  final int firstPrefixIndex;
  final int indentPrefixIndex;
  final RegExp pattern;
  final RegExp targetPattern;
}

final List<ForwardReferencePattern> csForwardReferenceLicenses = <ForwardReferencePattern>[
  // used with _tryForwardReferencePattern

  // OpenSSL
  ForwardReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    pattern: RegExp(
      kIndent +
          r'(?:' +
          (r'Portions of the attached software \("Contribution"\) are developed by .+ and are contributed to the OpenSSL project\.'
              .replaceAll(' ', _linebreak)) +
          r'|' +
          (r'The .+ included herein is developed by .+, and is contributed to the OpenSSL project\.'
              .replaceAll(' ', _linebreak)) +
          r'|'
              r'(?:\1? *\n)+'
              r')*'
              r'\1\2 *' +
          (r'The .+ is licensed pursuant to the OpenSSL open source license provided (?:below|above)\.'
              .replaceAll(' ', _linebreak)),
      multiLine: true,
      caseSensitive: false,
    ),
    targetPattern: RegExp('Redistribution and use in source and binary forms(?:.|\n)+OpenSSL'),
  ),

  ForwardReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    pattern: RegExp(
      kIndent + r'This code is released under the libpng license\. \(See LICENSE, below\.\)',
      multiLine: true,
    ),
    targetPattern: RegExp('PNG Reference Library License'),
  ),
];

// REFERENCES TO OTHER FILES

class LicenseFileReferencePattern {
  LicenseFileReferencePattern({
    required this.firstPrefixIndex,
    required this.indentPrefixIndex,
    this.copyrightIndex,
    this.authorIndex,
    required this.fileIndex,
    required this.pattern,
    this.needsCopyright = true,
  });
  final int firstPrefixIndex;
  final int indentPrefixIndex;
  final int? copyrightIndex;
  final int? authorIndex;
  final int fileIndex;
  final bool needsCopyright;
  final RegExp pattern;

  @override
  String toString() {
    return '$pattern ($firstPrefixIndex $indentPrefixIndex c=$copyrightIndex (needs? $needsCopyright) a=$authorIndex, f=$fileIndex';
  }
}

final List<LicenseFileReferencePattern> csReferencesByFilename = <LicenseFileReferencePattern>[
  // used with _tryReferenceByFilename

  // libpng files
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    fileIndex: 3,
    pattern: RegExp(
      kIndent +
          r'This code is released under the libpng license. For conditions of distribution and use, see the disclaimer and license in (png.h)\b'
              .replaceAll(' ', _linebreak),
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // zlib files
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    fileIndex: 3,
    pattern: RegExp(
      kIndent +
          r'(?:detect_data_type\(\) function provided freely by Cosmin Truta, 2006 )?'
                  r'For conditions of distribution and use, see copyright notice in (zlib.h|jsimdext.inc)\b'
              .replaceAll(' ', _linebreak),
      multiLine: true,
    ),
  ),

  // typical of much Google-written code
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    fileIndex: 3,
    pattern: RegExp(
      kIndent +
          r'Use of this source(?: code)? is governed by a BS?D-style license that can be found in the '
              .replaceAll(' ', _linebreak) +
          r'([^ ]+) file\b(?! or at)',
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // Chromium's zlib extensions.
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    copyrightIndex: 3,
    authorIndex: 4,
    fileIndex: 5,
    pattern: RegExp(
      kIndent +
                  // The " \* " is needed to ensure that both copyright lines start with
                  // the comment decoration in the captured match, otherwise it won't be
                  // stripped from the second line when generating output.
                  r'((?: \* Copyright \(C\) 2017 ARM, Inc.\n)?'
                  r'Copyright .+(The .+ Authors)(?:\. +All rights reserved\.)?)\n'
                  r'Use of this source code is governed by a BSD-style license that can be\n'
                  r'found in the Chromium source repository ([^ ]+) file.'
              .replaceAll(r'\n', _linebreakLoose),
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // Mojo code
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    copyrightIndex: 3,
    authorIndex: 4,
    fileIndex: 5,
    pattern: RegExp(
      kIndent +
          r'(Copyright .+(the .+ authors)\. +All rights reserved.) +' +
          r'Use of this source(?: code)? is governed by a BS?D-style license that can be found in the '
              .replaceAll(' ', _linebreak) +
          r'([^ ]+) file\b(?! or at)',
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // ANGLE .json files
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    copyrightIndex: 3,
    authorIndex: 4,
    fileIndex: 5,
    pattern: RegExp(
      kIndent +
          r'(Copyright .+(The .+ Authors)\. +All rights reserved.)", *\n'
              r'^\1\2Use of this source code is governed by a BSD-style license that can be", *\n'
              r'^\1\2found in the ([^ ]+) file.",',
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // typical of Dart-derived files
  LicenseFileReferencePattern(
    firstPrefixIndex: 2,
    indentPrefixIndex: 3,
    copyrightIndex: 1,
    authorIndex: 4,
    fileIndex: 5,
    pattern: RegExp(
      r'(' +
          kIndent +
          r'Copyright .+(the .+ authors)\[?\. '
                  r'Please see the AUTHORS file for details. All rights (?:re|solve)served\.) '
                  r'Use of this source(?: code)? is governed by a BS?D-style license '
                  r'that can be found in the '
              .replaceAll(' ', _linebreakLoose) +
          r'([^ ]+) file\b(?! or at)',
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // Seen in libjpeg-turbo
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    fileIndex: 3,
    pattern: RegExp(
      kIndent +
          r"(?:This file is part of the Independent JPEG Group's software.)? "
                  r'(?:It was modified by The libjpeg-turbo Project to include only code (?:and information)? relevant to libjpeg-turbo\.)? '
                  r'For conditions of distribution and use, see the accompanying '
                  r'(README.ijg)'
              .replaceAll(' ', _linebreakLoose),
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // Seen in FreeType software
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    fileIndex: 3,
    pattern: RegExp(
      kIndent +
          r'This file (?:is part of the FreeType project, and )?may only be used,? '
                  r'modified,? and distributed under the terms of the FreeType project '
                  r'license, (LICENSE\.TXT). By continuing to use, modify, or distribute this '
                  r'file you indicate that you have read the license and understand and '
                  r'accept it fully\.'
              .replaceAll(' ', _linebreak),
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // Seen in FreeType cff software from Adobe
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    fileIndex: 3,
    pattern: RegExp(
      kIndent +
          r'This software, and all works of authorship, whether in source or '
                  r'object code form as indicated by the copyright notice\(s\) included '
                  r'herein \(collectively, the "Work"\) is made available, and may only be '
                  r'used, modified, and distributed under the FreeType Project License, '
                  r'(LICENSE\.TXT)\. Additionally, subject to the terms and conditions of the '
                  r'FreeType Project License, each contributor to the Work hereby grants '
                  r'to any individual or legal entity exercising permissions granted by '
                  r'the FreeType Project License and this section \(hereafter, "You" or '
                  r'"Your"\) a perpetual, worldwide, non-exclusive, no-charge, '
                  r'royalty-free, irrevocable \(except as stated in this section\) patent '
                  r'license to make, have made, use, offer to sell, sell, import, and '
                  r'otherwise transfer the Work, where such license applies only to those '
                  r'patent claims licensable by such contributor that are necessarily '
                  r'infringed by their contribution\(s\) alone or by combination of their '
                  r'contribution\(s\) with the Work to which such contribution\(s\) was '
                  r'submitted\. If You institute patent litigation against any entity '
                  r'\(including a cross-claim or counterclaim in a lawsuit\) alleging that '
                  r'the Work or a contribution incorporated within the Work constitutes '
                  r'direct or contributory patent infringement, then any patent licenses '
                  r'granted to You under this License for that Work shall terminate as of '
                  r'the date such litigation is filed\. '
                  r'By using, modifying, or distributing the Work you indicate that you '
                  r'have read and understood the terms and conditions of the '
                  r'FreeType Project License as well as those provided in this section, '
                  r'and you accept them fully\.'
              .replaceAll(' ', _linebreak),
      multiLine: true,
    ),
  ),

  // BoringSSL
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    fileIndex: 3,
    pattern: RegExp(
      kIndent +
          r'Licensed under the OpenSSL license \(the "License"\)\. You may not use '
                  r'this file except in compliance with the License\. You can obtain a copy '
                  r'in the file (LICENSE) in the source distribution or at '
                  r'https://www\.openssl\.org/source/license\.html'
              .replaceAll(' ', _linebreak),
      multiLine: true,
    ),
  ),

  // Seen in Microsoft files
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    fileIndex: 3,
    pattern: RegExp(
      kIndent +
          r'Licensed under the MIT License\. '
                  r'See (License\.txt) in the project root for license information\.'
              .replaceAll(' ', _linebreak),
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // Seen in React Native files
  LicenseFileReferencePattern(
    firstPrefixIndex: 1,
    indentPrefixIndex: 2,
    fileIndex: 3,
    pattern: RegExp(
      kIndent +
          r'This source code is licensed under the MIT license found in the '
                  r'(LICENSE) file in the root directory of this source tree.'
              .replaceAll(' ', _linebreak),
      multiLine: true,
    ),
  ),
];

// INDIRECT REFERENCES TO OTHER FILES

final List<RegExp> csReferencesByType = <RegExp>[
  // used with _tryReferenceByType
  // groups 1 and 2 are the prefix, group 3 is the license type
  RegExp(
    kIndent +
        r'Written by Andy Polyakov <appro@openssl\.org> for the OpenSSL '
                r'project\. The module is, however, dual licensed under (OpenSSL) and '
                r'CRYPTOGAMS licenses depending on where you obtain it\. For further '
                r'details see http://www\.openssl\.org/~appro/cryptogams/\. '
                r'Permission to use under GPL terms is granted\.'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  // MPL
  // fallback_root_certificates
  RegExp(
    r'/\* This Source Code Form is subject to the terms of the Mozilla Public *\n'
    r'^( \*)( )License, v\. 2\.0\. +If a copy of the MPL was not distributed with this *\n'
    r'^\1\2file, You can obtain one at (http://mozilla\.org/MPL/2\.0/)\.',
    multiLine: true,
  ),

  // JSON (MIT)
  RegExp(
    kIndent + r'The code is distributed under the (MIT) license, Copyright (.+)\.$',
    multiLine: true,
  ),

  // BoringSSL
  RegExp(
    kIndent +
        r'Rights for redistribution and usage in source and binary forms are '
                r'granted according to the (OpenSSL) license\. Warranty of any kind is '
                r'disclaimed\.'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  RegExp(
    kIndent +
        (r'The portions of the attached software \("Contribution"\) is developed by '
                r'Nokia Corporation and is licensed pursuant to the (OpenSSL) open source '
                r'license\. '
                r'\n '
                r'The Contribution, originally written by Mika Kousa and Pasi Eronen of '
                r'Nokia Corporation, consists of the "PSK" \(Pre-Shared Key\) ciphersuites '
                r'support \(see RFC 4279\) to OpenSSL\. '
                r'\n '
                r'No patent licenses or other rights except those expressly stated in '
                r'the OpenSSL open source license shall be deemed granted or received '
                r'expressly, by implication, estoppel, or otherwise\. '
                r'\n '
                r'No assurances are provided by Nokia that the Contribution does not '
                r'infringe the patent or other intellectual property rights of any third '
                r'party or that the license provides you with all the necessary rights '
                r'to make use of the Contribution\. '
                r'\n '
                r'THE SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND\. IN '
                r'ADDITION TO THE DISCLAIMERS INCLUDED IN THE LICENSE, NOKIA '
                r'SPECIFICALLY DISCLAIMS ANY LIABILITY FOR CLAIMS BROUGHT BY YOU OR ANY '
                r'OTHER ENTITY BASED ON INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS OR '
                r'OTHERWISE\.'
            .replaceAll(' ', _linebreak)),
    multiLine: true,
  ),

  // TODO(ianh): Confirm this is the right way to handle this.
  RegExp(
    kIndent +
        r'.+ support in OpenSSL originally developed by '
                r'SUN MICROSYSTEMS, INC\., and contributed to the (OpenSSL) project\.'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  // Brotli (MIT)
  RegExp(kIndent + r'Distributed under (MIT) license\.', multiLine: true),

  RegExp(
    kIndent +
        r'This software is made available under the terms of the (ICU) License -- ICU 1\.8\.1 and later\.'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),
];

class LicenseReferencePattern {
  LicenseReferencePattern({
    this.firstPrefixIndex = 1,
    this.indentPrefixIndex = 2,
    this.licenseIndex = 3,
    this.checkLocalFirst = true,
    this.spdx =
        false, // indicates whether this is a reference via the SPDX syntax rather than a statement in English
    required this.pattern,
  });

  final int? firstPrefixIndex;
  final int? indentPrefixIndex;
  final int? licenseIndex;
  final bool checkLocalFirst;
  final bool spdx;
  final RegExp? pattern;
}

final List<LicenseReferencePattern> csReferencesByIdentifyingReference = <LicenseReferencePattern>[
  // used with _tryReferenceByIdentifyingReference
  LicenseReferencePattern(
    pattern: RegExp(kIndent + r'For terms of use, see ([^ \n]+)', multiLine: true),
  ),

  LicenseReferencePattern(
    pattern: RegExp(kIndent + r'License & terms of use: ([^ \n]+)', multiLine: true),
  ),

  LicenseReferencePattern(
    pattern: RegExp(kIndent + r'For more info read (MiniZip_info.txt)', multiLine: true),
  ),

  // SPDX
  LicenseReferencePattern(
    checkLocalFirst: false,
    spdx:
        true, // indicates that this is a reference via the SPDX syntax rather than a statement in English
    pattern: RegExp(kIndent + r'SPDX-License-Identifier: (.+)', multiLine: true),
  ),

  // Apache reference.
  // Seen in Android code.
  // TODO(ianh): For this license we only need to include the text once, not once per copyright
  // TODO(ianh): For this license we must also include all the NOTICE text (see section 4d)
  LicenseReferencePattern(
    checkLocalFirst: false,
    pattern: RegExp(
      kIndent +
          r'Licensed under the Apache License, Version 2\.0 \(the "License"\); *\n'
              r'^\1\2you may not use this file except in compliance with the License\. *\n'
              r'^(?:\1\2Copyright \(c\) 2015-2017 Valve Corporation\n)?' // https://github.com/KhronosGroup/Vulkan-ValidationLayers/pull/4930
              r'^\1\2You may obtain a copy of the License at *\n'
              r'^(?:(?:\1\2? *)? *\n)*'
              r'^\1\2 *(https?://www\.apache\.org/licenses/LICENSE-2\.0) *\n'
              r'^(?:(?:\1\2? *)? *\n)*'
              r'^\1\2 *Unless required by applicable law or agreed to in writing, software *\n'
              r'^\1\2 *distributed under the License is distributed on an "AS IS" BASIS, *\n'
              r'^\1\2 *WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied\. *\n'
              r'^\1\2 *See the License for the specific language governing permissions and *\n'
              r'^\1\2 *limitations under the License\.',
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // MIT
  // LicenseReferencePattern(
  //   checkLocalFirst: false,
  //   pattern: RegExp(
  //     kIndent +
  //     (
  //      r'Use of this source code is governed by a MIT-style '
  //      r'license that can be found in the LICENSE file or at '
  //      r'(https://opensource.org/licenses/MIT)'
  //      .replaceAll(' ', _linebreak)
  //     ),
  //     multiLine: true,
  //     caseSensitive: false,
  //   )
  // ),

  // MIT
  // the crazy s/./->/ thing is someone being over-eager with search-and-replace in rapidjson
  LicenseReferencePattern(
    checkLocalFirst: false,
    pattern: RegExp(
      kIndent +
          r'Licensed under the MIT License \(the "License"\); you may not use this file except *\n'
              r'^\1\2in compliance with the License(?:\.|->) You may obtain a copy of the License at *\n'
              r'^\1\n'
              r'^\1\2(http://opensource(?:\.|->)org/licenses/MIT) *\n'
              r'^\1\n'
              r'^\1\2Unless required by applicable law or agreed to in writing, software distributed *\n'
              r'^\1\2under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR *\n'
              r'^\1\2CONDITIONS OF ANY KIND, either express or implied(?:\.|->) See the License for the *\n'
              r'^\1\2specific language governing permissions and limitations under the License(?:\.|->)',
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // Observatory (polymer)
  LicenseReferencePattern(
    checkLocalFirst: false,
    pattern: RegExp(
      kIndent +
          r'This code may only be used under the BSD style license found at (http://polymer.github.io/LICENSE.txt)$',
      multiLine: true,
      caseSensitive: false,
    ),
  ),

  // LLVM (Apache License v2.0 with LLVM Exceptions)
  LicenseReferencePattern(
    checkLocalFirst: false,
    pattern: RegExp(
      kIndent +
          (r'Part of the LLVM Project, under the Apache License v2\.0 with LLVM Exceptions\. '
                  r'See (https://llvm\.org/LICENSE\.txt) for license information\.'
              .replaceAll(' ', _linebreak)),
      multiLine: true,
    ),
  ),

  // Seen in RFC-derived files in ICU
  LicenseReferencePattern(
    checkLocalFirst: false,
    pattern: RegExp(
      kIndent +
          r'This file was generated from RFC 3454 \((http://www.ietf.org/rfc/rfc3454.txt)\) '
                  r'Copyright \(C\) The Internet Society \(2002\)\. All Rights Reserved\.'
              .replaceAll(' ', _linebreak),
      multiLine: true,
    ),
  ),
];

// INLINE LICENSES

final List<RegExp> csTemplateLicenses = <RegExp>[
  // used with _tryInline, with needsCopyright: true (will only match if preceded by a copyright notice)
  // should have two groups, prefixes 1 and 2

  // BoringSSL
  RegExp(
    kIndent +
        r'Redistribution and use in source and binary forms, with or without *\n'
            r'^\1\2modification, are permitted provided that the following conditions *\n'
            r'^\1\2are met: *\n'
            r'^(?:(?:\1\2? *)? *\n)*'
            r'^\1\2(?:[-*1-9.)/ ]+)Redistributions of source code must retain the above copyright *\n'
            r'^\1\2 *notice, this list of conditions and the following disclaimer\. *\n'
            r'^(?:(?:\1\2? *)? *\n)*'
            r'^\1\2(?:[-*1-9.)/ ]+)Redistributions in binary form must reproduce the above copyright *\n'
            r'^\1\2 *notice, this list of conditions and the following disclaimer in *\n'
            r'^\1\2 *the documentation and/or other materials provided with the *\n'
            r'^\1\2 *distribution\. *\n'
            r'^(?:(?:\1\2? *)? *\n)*'
            r'^\1\2(?:[-*1-9.)/ ]+)All advertising materials mentioning features or use of this *\n'
            r'^\1\2 *software must display the following acknowledgment: *\n'
            r'^\1\2 *"This product includes software developed by the OpenSSL Project *\n'
            r'^\1\2 *for use in the OpenSSL Toolkit\. \(http://www\.OpenSSL\.org/\)" *\n'
            r'^(?:(?:\1\2? *)? *\n)*'
            r'^\1\2(?:[-*1-9.)/ ]+)The names "OpenSSL Toolkit" and "OpenSSL Project" must not be used to *\n'
            r'^\1\2 *endorse or promote products derived from this software without *\n'
            r'^\1\2 *prior written permission\. +For written permission, please contact *\n'
            r'^\1\2 *[^@ ]+@OpenSSL\.org\. *\n'
            r'^(?:(?:\1\2? *)? *\n)*'
            r'^\1\2(?:[-*1-9.)/ ]+)Products derived from this software may not be called "OpenSSL" *\n'
            r'^\1\2 *nor may "OpenSSL" appear in their names without prior written *\n'
            r'^\1\2 *permission of the OpenSSL Project\. *\n'
            r'^(?:(?:\1\2? *)? *\n)*'
            r'^\1\2(?:[-*1-9.)/ ]+)Redistributions of any form whatsoever must retain the following *\n'
            r'^\1\2 *acknowledgment: *\n'
            r'^\1\2 *"This product includes software developed by the OpenSSL Project *\n'
            r'^\1\2 *for use in the OpenSSL Toolkit \(http://www\.OpenSSL\.org/\)" *\n'
            r'^(?:(?:\1\2? *)? *\n)*'
            r"^\1\2THIS SOFTWARE IS PROVIDED BY THE OpenSSL PROJECT ``AS IS'' AND ANY *\n"
            r'^\1\2EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE *\n'
            r'^\1\2IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR *\n'
            r'^\1\2PURPOSE ARE DISCLAIMED\. +IN NO EVENT SHALL THE OpenSSL PROJECT OR *\n'
            r'^\1\2ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, *\n'
            r'^\1\2SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES \(INCLUDING, BUT *\n'
            r'^\1\2NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; *\n'
            r'^\1\2LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION\) *\n'
            r'^\1\2HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, *\n'
            r'^\1\2STRICT LIABILITY, OR TORT \(INCLUDING NEGLIGENCE OR OTHERWISE\) *\n'
            r'^\1\2ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED *\n'
            r'^\1\2OF THE POSSIBILITY OF SUCH DAMAGE\.',
    multiLine: true,
    caseSensitive: false,
  ),

  RegExp(
    kIndent +
        r'(?:This package is an SSL implementation written *\n'
            r'^\1\2by Eric Young \(eay@cryptsoft\.com\)\. *\n'
            r'^\1\2The implementation was written so as to conform with Netscapes SSL\. *\n'
            r'^(?:(?:\1\2?g? *)? *\n)*'
            r'^\1\2This library is free for commercial and non-commercial use as long as *\n'
            r'^\1\2the following conditions are aheared to\. +The following conditions *\n'
            r'^\1\2apply to all code found in this distribution, be it the RC4, RSA, *\n'
            r'^\1\2lhash, DES, etc\., code; not just the SSL code\. +The SSL documentation *\n'
            r'^\1\2included with this distribution is covered by the same copyright terms *\n'
            r'^\1\2except that the holder is Tim Hudson \(tjh@cryptsoft\.com\)\. *\n'
            r'^(?:(?:\1\2?g? *)? *\n)*'
            r"^\1\2Copyright remains Eric Young's, and as such any Copyright notices in *\n"
            r'^\1\2the code are not to be removed\. *\n'
            r'^\1\2If this package is used in a product, Eric Young should be given attribution *\n'
            r'^\1\2as the author of the parts of the library used\. *\n'
            r'^\1\2This can be in the form of a textual message at program startup or *\n'
            r'^\1\2in documentation \(online or textual\) provided with the package\. *\n'
            r'^(?:(?:\1\2?g? *)? *\n)*'
            r'^\1\2)?Redistribution and use in source and binary forms, with or without *\n'
            r'^\1\2modification, are permitted provided that the following conditions *\n'
            r'^\1\2are met: *\n'
            r'^\1\2(?:[-*1-9.)/ ]+)Redistributions of source code must retain the copyright *\n'
            r'^\1\2 *notice, this list of conditions and the following disclaimer\. *\n'
            r'^\1\2(?:[-*1-9.)/ ]+)Redistributions in binary form must reproduce the above copyright *\n'
            r'^\1\2 *notice, this list of conditions and the following disclaimer in the *\n'
            r'^\1\2 *documentation and/or other materials provided with the distribution\. *\n'
            r'^\1\2(?:[-*1-9.)/ ]+)All advertising materials mentioning features or use of this software *\n'
            r'^\1\2 *must display the following acknowledgement: *\n'
            r'^\1\2 *"This product includes cryptographic software written by *\n'
            r'^\1\2 *Eric Young \(eay@cryptsoft\.com\)" *\n'
            r"^\1\2 *The word 'cryptographic' can be left out if the rouines from the library *\n" // TODO(ianh): File a bug on the number of analyzer errors you get if you replace the " characters on this line with '
            r'^\1\2 *being used are not cryptographic related :-\)\. *\n'
            r'^\1\2(?:[-*1-9.)/ ]+)If you include any Windows specific code \(or a derivative thereof\) fromg? *\n'
            r'^\1\2 *the apps directory \(application code\) you must include an acknowledgement: *\n'
            r'^\1\2 *"This product includes software written by Tim Hudson \(tjh@cryptsoft\.com\)" *\n'
            r'^(?:(?:\1\2?g? *)? *\n)*'
            r"^\1\2THIS SOFTWARE IS PROVIDED BY ERIC YOUNG ``AS IS'' AND *\n"
            r'^\1\2ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE *\n'
            r'^\1\2IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *\n'
            r'^\1\2ARE DISCLAIMED\. +IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE *\n'
            r'^\1\2FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *\n'
            r'^\1\2DAMAGES \(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS *\n'
            r'^\1\2OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION\) *\n'
            r'^\1\2HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT *\n'
            r'^\1\2LIABILITY, OR TORT \(INCLUDING NEGLIGENCE OR OTHERWISE\) ARISING IN ANY WAY *\n'
            r'^\1\2OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF *\n'
            r'^\1\2SUCH DAMAGE\. *\n'
            r'^(?:(?:\1\2?g? *)? *\n)*'
            r'^\1\2The licence and distribution terms for any publically available version or *\n'
            r'^\1\2derivative of this code cannot be changed\. +i\.e\. +this code cannot simply be *\n'
            r'^\1\2copied and put under another distribution licence *\n'
            r'^\1\2\[including the GNU Public Licence\.\]',
    multiLine: true,
    caseSensitive: false,
  ),

  // BSD-DERIVED LICENSES
  RegExp(
    kIndent +
        // Some files in ANGLE prefix the license with a description of the license.
        r'(?:BSD 2-Clause License \(https?://www.opensource.org/licenses/bsd-license.php\))?' +
        _linebreak +
        ('Redistribution and use in source and binary forms, with or without '
                'modification, are permitted provided that the following conditions are met:'
            .replaceAll(' ', _linebreak)) +
        // the conditions:
        r'(?:' +
        // indented blank lines
        _linebreak +
        // truly blank lines
        r'|\n+' +
        // ad clause - ucb
        r'|(?:[-*1-9.)/ ]*)' +
        ('All advertising materials mentioning features or use of this software '
                'must display the following acknowledgement: This product includes software '
                'developed by the University of California, Berkeley and its contributors\\.'
            .replaceAll(' ', _linebreak)) +
        // ad clause - netbsd
        r'|(?:[-*1-9.)/ ]*)' +
        ('All advertising materials mentioning features or use of this software '
                'must display the following acknowledgement: This product includes software '
                'developed by the NetBSD Foundation, Inc\\. and its contributors\\.'
            .replaceAll(' ', _linebreak)) +
        // ack clause
        r'|(?:[-*1-9.)/ ]*)' +
        (r'The origin of this software must not be misrepresented; you must not claim '
                r'that you wrote the original software\. If you use this software in a product, '
                r'an acknowledgment in the product documentation would be appreciated but is '
                r'not required\.'
            .replaceAll(' ', _linebreak)) +
        r'|(?:[-*1-9.)/ ]*)' +
        (r'Altered source versions must be plainly marked as such, and must not be '
                r'misrepresented as being the original software\.'
            .replaceAll(' ', _linebreak)) +
        // no ad clauses
        r'|(?:[-*1-9.)/ ]*)' +
        ('Neither my name, .+, nor the names of any other contributors to the code '
                'use may not be used to endorse or promote products derived from this '
                'software without specific prior written permission\\.'
            .replaceAll(' ', _linebreak)) +
        r'|(?:[-*1-9.)/ ]*)' +
        ('The name of the author may not be used to endorse or promote products '
                'derived from this software without specific prior written permission\\.?'
            .replaceAll(' ', _linebreak)) +
        r'|(?:[-*1-9.)/ ]*)' +
        ('Neither the name of .+ nor the names of its contributors may be used '
                'to endorse or promote products derived from this software without '
                'specific prior written permission\\.'
            .replaceAll(' ', _linebreak)) +
        // notice clauses
        r'|(?:[-*1-9.)/ ]*)' +
        ('Redistributions of source code must retain the above copyright notice, '
                'this list of conditions and the following disclaimer\\.'
            .replaceAll(' ', _linebreak)) +
        r'|(?:[-*1-9.)/ ]*)' +
        ('Redistributions in binary form must reproduce the above copyright notice, '
                'this list of conditions and the following disclaimer in the documentation '
                'and/or other materials provided with the distribution\\.'
            .replaceAll(' ', _linebreak)) +
        r'|(?:[-*1-9.)/ ]*)' +
        ('Redistributions in binary form must reproduce the above copyright notice, '
                'this list of conditions and the following disclaimer\\.'
            .replaceAll(' ', _linebreak)) +
        // end of conditions
        r')*' +
        // disclaimers
        ('THIS SOFTWARE IS PROVIDED (?:BY .+(?: .+)? )?["“`]+AS IS["”\']+,? AND ANY EXPRESS OR IMPLIED '
                'WARRANTIES,(?::tabnew)? INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF '
                'MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED\\. IN NO EVENT '
                'SHALL .+(?: .+)? BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR '
                'CONSEQUENTIAL DAMAGES \\(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS '
                'OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION\\) HOWEVER CAUSED '
                'AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT \\(INCLUDING '
                'NEGLIGENCE OR OTHERWISE\\) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF '
                'ADVISED OF THE POSSIBILITY OF SUCH DAMAGE\\.'
            .replaceAll(' ', _linebreak)),
    multiLine: true,
    caseSensitive: false,
  ),

  // MIT-DERIVED LICENSES

  // Seen in Mesa, among others.
  RegExp(
    kIndent +
        r'(?:' // this bit is optional
            r'Licensed under the MIT license:\n' // seen in expat
            r'\1\2? *\n' // blank line
            r'\1\2' // this is the prefix for the next block (handled by kIndent if this optional bit is skipped)
            r')?' // end of optional bit
            +
        (r'Permission is hereby granted, free of charge, to any person obtaining '
                r'a copy of this software and(?: /or)? associated documentation files \(the "(?:Software|Materials) "\), '
                r'to deal in the (?:Software|Materials) without restriction, including without limitation '
                r'the rights to use, copy, modify, merge, publish, distribute, sub license, '
                r'and/or sell copies of the (?:Software|Materials), and to permit persons to whom the '
                r'(?:Software|Materials) (?:is|are) furnished to do so, subject to the following conditions:'
            .replaceAll(' ', _linebreak)) +
        r'(?:' +
        (r'(?:(?:\1\2?(?: *| -*))? *\n)*' // A version with "// -------" between sections was seen in ffx_spd, hence the -*.
            +
            r'|'
                r'\1\2 '
                r'The above copyright notice and this permission notice'
                r'(?: \(including the next paragraph\))? '
                r'shall be included in all copies or substantial portions '
                r'of the (?:Software|Materials)\.'
                r'|'
                r'\1\2 '
                r'The above copyright notice including the dates of first publication and either this '
                r'permission notice or a reference to .+ shall be '
                r'included in all copies or substantial portions of the Software.'
                r'|'
                r'\1\2 '
                r'In addition, the following condition applies:'
                r'|'
                r'\1\2 '
                r'All redistributions must retain an intact copy of this copyright notice and disclaimer\.'
                r'|'
                r'\1\2 '
                r'MODIFICATIONS TO THIS FILE MAY MEAN IT NO LONGER ACCURATELY REFLECTS KHRONOS '
                r'STANDARDS. THE UNMODIFIED, NORMATIVE VERSIONS OF KHRONOS SPECIFICATIONS AND '
                r'HEADER INFORMATION ARE LOCATED AT https://www\.khronos\.org/registry/'
                r'|'
                r'\1\2 '
                r'THE (?:SOFTWARE|MATERIALS) (?:IS|ARE) PROVIDED "AS -? IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS '
                r'OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, '
                r'FITNESS FOR A PARTICULAR PURPOSE AND NON-?INFRINGEMENT\. IN NO EVENT SHALL '
                r'.+(?: .+)? BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER '
                r'IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,(?: )?OUT OF OR IN '
                r'CONNECTION WITH THE (?:SOFTWARE|MATERIALS) OR THE USE OR OTHER DEALINGS IN THE (?:SOFTWARE|MATERIALS)\.'
                r'|'
                r'\1\2 '
                r'THE (?:SOFTWARE|MATERIALS) (?:IS|ARE) PROVIDED "AS -? IS" AND WITHOUT WARRANTY OF ANY KIND, '
                r'EXPRESS, IMPLIED OR OTHERWISE, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY '
                r'OR FITNESS FOR A PARTICULAR PURPOSE\.'
                r'|'
                r'\1\2 '
                r'IN NO EVENT SHALL .+ BE LIABLE FOR ANY SPECIAL, INCIDENTAL, INDIRECT OR CONSEQUENTIAL '
                r'DAMAGES OF ANY KIND, OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, '
                r'WHETHER OR NOT ADVISED OF THE POSSIBILITY OF DAMAGE, AND ON ANY THEORY OF LIABILITY, ARISING '
                r'OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE\.'
                r'|'
                r'\1\2 '
                r'Except as contained in this notice, the name of .+ shall not '
                r'be used in advertising or otherwise to promote the sale, use or other dealings in '
                r'this Software without prior written authorization from .+\.'.replaceAll(' ', _linebreak)) +
        r')*',
    multiLine: true,
    caseSensitive: false,
  ),

  RegExp(
    kIndent +
        r'Boost Software License - Version 1\.0 - August 17th, 2003\n' +
        r'\n' +
        (r'\1\2Permission is hereby granted, free of charge, to any person or '
                r'organization obtaining a copy of the software and accompanying '
                r'documentation covered by this license \(the "Software"\) to use, '
                r'reproduce, display, distribute, execute, and transmit the Software, and '
                r'to prepare derivative works of the Software, and to permit third-parties '
                r'to whom the Software is furnished to do so, all subject to the following:\n'
            .replaceAll(' ', _linebreak)) +
        r'\n' +
        (r'\1\2The copyright notices in the Software and this entire statement, '
                r'including the above license grant, this restriction and the following '
                r'disclaimer, must be included in all copies of the Software, in whole or '
                r'in part, and all derivative works of the Software, unless such copies or '
                r'derivative works are solely in the form of machine-executable object '
                r'code generated by a source language processor\.\n'
            .replaceAll(' ', _linebreak)) +
        r'\n' +
        (r'\1\2THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, '
                r'EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF '
                r'MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, TITLE AND '
                r'NON-INFRINGEMENT\. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR ANYONE '
                r'DISTRIBUTING THE SOFTWARE BE LIABLE FOR ANY DAMAGES OR OTHER LIABILITY, '
                r'WHETHER IN CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN '
                r'CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE '
                r'SOFTWARE\.'
            .replaceAll(' ', _linebreak)),
    multiLine: true,
    caseSensitive: false,
  ),

  // BISON LICENSE
  // Seen in some ANGLE source. The usage falls under the "special exception" clause.
  RegExp(
    kIndent +
        r'This program is free software: you can redistribute it and/or modify\n'
            r'^(?:\1\2)?it under the terms of the GNU General Public License as published by\n'
            r'^(?:\1\2)?the Free Software Foundation, either version 3 of the License, or\n'
            r'^(?:\1\2)?\(at your option\) any later version.\n'
            r'^(?:\1\2)?\n*'
            r'^(?:\1\2)?This program is distributed in the hope that it will be useful,\n'
            r'^(?:\1\2)?but WITHOUT ANY WARRANTY; without even the implied warranty of\n'
            r'^(?:\1\2)?MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n'
            r'^(?:\1\2)?GNU General Public License for more details.\n'
            r'^(?:\1\2)?\n*'
            r'^(?:\1\2)?You should have received a copy of the GNU General Public License\n'
            r'^(?:\1\2)?along with this program.  If not, see <https?://www.gnu.org/licenses/>.  \*/\n'
            r'^(?:\1\2)?\n*' +
        kIndent +
        r'As a special exception, you may create a larger work that contains\n'
            r'^(?:\1\2)?part or all of the Bison parser skeleton and distribute that work\n'
            r"^(?:\1\2)?under terms of your choice, so long as that work isn't itself a\n"
            r'^(?:\1\2)?parser generator using the skeleton or a modified version thereof\n'
            r'^(?:\1\2)?as a parser skeleton.  Alternatively, if you modify or redistribute\n'
            r'^(?:\1\2)?the parser skeleton itself, you may \(at your option\) remove this\n'
            r'^(?:\1\2)?special exception, which will cause the skeleton and the resulting\n'
            r'^(?:\1\2)?Bison output files to be licensed under the GNU General Public\n'
            r'^(?:\1\2)?License without this special exception.\n'
            r'^(?:\1\2)?\n*'
            r'^(?:\1\2)?This special exception was added by the Free Software Foundation in\n'
            r'^(?:\1\2)?version 2.2 of Bison.  \*/\n',
    multiLine: true,
    caseSensitive: false,
  ),

  // NVIDIA license found in glslang
  RegExp(
    kIndent +
        r'NVIDIA Corporation\("NVIDIA"\) supplies this software to you in\n'
            r'\1\2consideration of your agreement to the following terms, and your use,\n'
            r'\1\2installation, modification or redistribution of this NVIDIA software\n'
            r'\1\2constitutes acceptance of these terms\.  If you do not agree with these\n'
            r'\1\2terms, please do not use, install, modify or redistribute this NVIDIA\n'
            r'\1\2software\.\n'
            r'\1(?:\2)?\n'
            r'\1\2In consideration of your agreement to abide by the following terms, and\n'
            r'\1\2subject to these terms, NVIDIA grants you a personal, non-exclusive\n'
            r"\1\2license, under NVIDIA's copyrights in this original NVIDIA software \(the\n"
            r'\1\2"NVIDIA Software"\), to use, reproduce, modify and redistribute the\n'
            r'\1\2NVIDIA Software, with or without modifications, in source and/or binary\n'
            r'\1\2forms; provided that if you redistribute the NVIDIA Software, you must\n'
            r'\1\2retain the copyright notice of NVIDIA, this notice and the following\n'
            r'\1\2text and disclaimers in all such redistributions of the NVIDIA Software\.\n'
            r'\1\2Neither the name, trademarks, service marks nor logos of NVIDIA\n'
            r'\1\2Corporation may be used to endorse or promote products derived from the\n'
            r'\1\2NVIDIA Software without specific prior written permission from NVIDIA\.\n'
            r'\1\2Except as expressly stated in this notice, no other rights or licenses\n'
            r'\1\2express or implied, are granted by NVIDIA herein, including but not\n'
            r'\1\2limited to any patent rights that may be infringed by your derivative\n'
            r'\1\2works or by other works in which the NVIDIA Software may be\n'
            r'\1\2incorporated\. No hardware is licensed hereunder\.\n'
            r'\1(?:\2)?\n'
            r'\1\2THE NVIDIA SOFTWARE IS BEING PROVIDED ON AN "AS IS" BASIS, WITHOUT\n'
            r'\1\2WARRANTIES OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED,\n'
            r'\1\2INCLUDING WITHOUT LIMITATION, WARRANTIES OR CONDITIONS OF TITLE,\n'
            r'\1\2NON-INFRINGEMENT, MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, OR\n'
            r'\1\2ITS USE AND OPERATION EITHER ALONE OR IN COMBINATION WITH OTHER\n'
            r'\1\2PRODUCTS\.\n'
            r'\1(?:\2)?\n'
            r'\1\2IN NO EVENT SHALL NVIDIA BE LIABLE FOR ANY SPECIAL, INDIRECT,\n'
            r'\1\2INCIDENTAL, EXEMPLARY, CONSEQUENTIAL DAMAGES \(INCLUDING, BUT NOT LIMITED\n'
            r'\1\2TO, LOST PROFITS; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF\n'
            r'\1\2USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION\) OR ARISING IN ANY WAY\n'
            r'\1\2OUT OF THE USE, REPRODUCTION, MODIFICATION AND/OR DISTRIBUTION OF THE\n'
            r'\1\2NVIDIA SOFTWARE, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT,\n'
            r'\1\2TORT \(INCLUDING NEGLIGENCE\), STRICT LIABILITY OR OTHERWISE, EVEN IF\n'
            r'\1\2NVIDIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE\.\n',
    multiLine: true,
  ),

  // OTHER BRIEF LICENSES

  // Seen in the NDK
  RegExp(
    kIndent +
        r'Permission to use, copy, modify, and/or distribute this software for any *\n'
            r'^\1\2purpose with or without fee is hereby granted, provided that the above *\n'
            r'^\1\2copyright notice and this permission notice appear in all copies\. *\n'
            r'^(?:(?:\1\2? *)? *\n)*'
            r'^\1\2THE SOFTWARE IS PROVIDED "AS IS" AND .+ DISCLAIMS ALL WARRANTIES(?: WITH)? *\n'
            r'^\1\2(?:WITH )?REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF(?: MERCHANTABILITY)? *\n'
            r'^\1\2(?:MERCHANTABILITY )?AND FITNESS\. +IN NO EVENT SHALL .+ BE LIABLE FOR(?: ANY(?: SPECIAL, DIRECT,)?)? *\n'
            r'^\1\2(?:(?:ANY )?SPECIAL, DIRECT, )?INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES(?: WHATSOEVER RESULTING FROM)? *\n'
            r'^\1\2(?:WHATSOEVER RESULTING FROM )?LOSS OF USE, DATA OR PROFITS, WHETHER IN AN(?: ACTION(?: OF CONTRACT, NEGLIGENCE)?)? *\n'
            r'^\1\2(?:(?:ACTION )?OF CONTRACT, NEGLIGENCE )?OR OTHER TORTIOUS ACTION, ARISING OUT OF(?: OR IN(?: CONNECTION WITH THE USE OR)?)? *\n'
            r'^\1\2(?:(?:OR IN )?CONNECTION WITH THE USE OR )?PERFORMANCE OF THIS SOFTWARE\.',
    multiLine: true,
  ),

  // harfbuzz
  RegExp(
    kIndent +
        r'Permission is hereby granted, without written agreement and without *\n'
            r'^\1\2license or royalty fees, to use, copy, modify, and distribute this *\n'
            r'^\1\2software and its documentation for any purpose, provided that the *\n'
            r'^\1\2above copyright notice and the following two paragraphs appear in *\n'
            r'^\1\2all copies of this software\. *\n'
            r'^(?:(?:\1\2? *)? *\n)*'
            r'^\1\2IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE TO ANY PARTY FOR *\n'
            r'^\1\2DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES *\n'
            r'^\1\2ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN *\n'
            r'^\1\2IF THE COPYRIGHT HOLDER HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH *\n'
            r'^\1\2DAMAGE\. *\n'
            r'^(?:(?:\1\2? *)? *\n)*'
            r'^\1\2THE COPYRIGHT HOLDER SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, *\n'
            r'^\1\2BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND *\n'
            r'^\1\2FITNESS FOR A PARTICULAR PURPOSE\. +THE SOFTWARE PROVIDED HEREUNDER IS *\n'
            r'^\1\2ON AN "AS IS" BASIS, AND THE COPYRIGHT HOLDER HAS NO OBLIGATION TO *\n'
            r'^\1\2PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS\. *\n',
    multiLine: true,
  ),

  // freetype2.
  RegExp(
    kIndent +
        r'Permission to use, copy, modify, distribute, and sell this software and its '
                r'documentation for any purpose is hereby granted without fee, provided that '
                r'the above copyright notice appear in all copies and that both that '
                r'copyright notice and this permission notice appear in supporting '
                r'documentation\. '
                r'The above copyright notice and this permission notice shall be included in '
                r'all copies or substantial portions of the Software\. '
                r'THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR '
                r'IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, '
                r'FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT\.  IN NO EVENT SHALL THE '
                r'OPEN GROUP BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN '
                r'AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN '
                r'CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE\. '
                r'Except as contained in this notice, the name of The Open Group shall not be '
                r'used in advertising or otherwise to promote the sale, use or other dealings '
                r'in this Software without prior written authorization from The Open Group\. '
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  // libjpeg-turbo
  RegExp(
    kIndent +
        r'Permission to use, copy, modify, and distribute this software and its '
                r'documentation for any purpose and without fee is hereby granted, provided '
                r'that the above copyright notice appear in all copies and that both that '
                r'copyright notice and this permission notice appear in supporting '
                r'documentation\. This software is provided "as is" without express or '
                r'implied warranty\.'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),
];

// LICENSES WE JUST DISPLAY VERBATIM

final List<RegExp> csNoticeLicenses = <RegExp>[
  // used with _tryInline, with needsCopyright: false
  // should have two groups, prefixes 1 and 2
  RegExp(
    kIndent +
        r'COPYRIGHT NOTICE, DISCLAIMER, and LICENSE\n'
            r'(?:\1.*\n)+?'
            r'(?=\1\2END OF COPYRIGHT NOTICE, DISCLAIMER, and LICENSE\.)',
    multiLine: true,
  ),

  // ideally this would be template-expanded against the referenced license, but
  // there's two licenses in the file in question and it's not exactly obvious
  // how to programmatically select the right one... for now just including the
  // text verbatim should be enough.
  RegExp(
    kIndent +
        r'Portions of the attached software \("Contribution"\) are developed by '
                r'SUN MICROSYSTEMS, INC\., and are contributed to the OpenSSL project\. '
                r'\n '
                r'The Contribution is licensed pursuant to the Eric Young open source '
                r'license provided above\.'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  // ideally this would be template-expanded against the referenced license, but
  // there's two licenses in the file in question and it's not exactly obvious
  // how to programmatically select the right one... for now just including the
  // text verbatim should be enough.
  RegExp(
    kIndent +
        r'Portions of the attached software \("Contribution"\) are developed by '
                r'SUN MICROSYSTEMS, INC\., and are contributed to the OpenSSL project\. '
                r'\n '
                r'The Contribution is licensed pursuant to the OpenSSL open source '
                r'license provided above\.'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  // Freetype
  RegExp(
    kIndent +
        r'This software was written by Alexander Peslyak in 2001\. No copyright is '
                r'claimed, and the software is hereby placed in the public domain\. In case '
                r'this attempt to disclaim copyright and place the software in the public '
                r'domain is deemed null and void, then the software is Copyright \(c\) 2001 '
                r'Alexander Peslyak and it is hereby released to the general public under the '
                r'following terms: Redistribution and use in source and binary forms, with or '
                r"without modification, are permitted\. There\'s ABSOLUTELY NO WARRANTY, "
                r'express or implied\.(?: \(This is a heavily cut-down "BSD license"\.\))?'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  // Ahem font non-copyright
  RegExp(
    r'(//)( )License for the Ahem font embedded below is from:\n'
    r'\1\2https://www\.w3\.org/Style/CSS/Test/Fonts/Ahem/COPYING\n'
    r'\1\n'
    r'\1\2The Ahem font in this directory belongs to the public domain\. In\n'
    r'\1\2jurisdictions that do not recognize public domain ownership of these\n'
    r'\1\2files, the following Creative Commons Zero declaration applies:\n'
    r'\1\n'
    r'\1\2<http://labs\.creativecommons\.org/licenses/zero-waive/1\.0/us/legalcode>\n'
    r'\1\n'
    r'\1\2which is quoted below:\n'
    r'\1\n'
    r'\1\2  The person who has associated a work with this document \(the "Work"\)\n'
    r'\1\2  affirms that he or she \(the "Affirmer"\) is the/an author or owner of\n'
    r'\1\2  the Work\. The Work may be any work of authorship, including a\n'
    r'\1\2  database\.\n'
    r'\1\n'
    r'\1\2  The Affirmer hereby fully, permanently and irrevocably waives and\n'
    r'\1\2  relinquishes all of her or his copyright and related or neighboring\n'
    r'\1\2  legal rights in the Work available under any federal or state law,\n'
    r'\1\2  treaty or contract, including but not limited to moral rights,\n'
    r'\1\2  publicity and privacy rights, rights protecting against unfair\n'
    r'\1\2  competition and any rights protecting the extraction, dissemination\n'
    r'\1\2  and reuse of data, whether such rights are present or future, vested\n'
    r'\1\2  or contingent \(the "Waiver"\)\. The Affirmer makes the Waiver for the\n'
    r"\1\2  benefit of the public at large and to the detriment of the Affirmer's\n"
    r'\1\2  heirs or successors\.\n'
    r'\1\n'
    r'\1\2  The Affirmer understands and intends that the Waiver has the effect\n'
    r"\1\2  of eliminating and entirely removing from the Affirmer's control all\n"
    r'\1\2  the copyright and related or neighboring legal rights previously held\n'
    r'\1\2  by the Affirmer in the Work, to that extent making the Work freely\n'
    r'\1\2  available to the public for any and all uses and purposes without\n'
    r'\1\2  restriction of any kind, including commercial use and uses in media\n'
    r'\1\2  and formats or by methods that have not yet been invented or\n'
    r'\1\2  conceived\. Should the Waiver for any reason be judged legally\n'
    r'\1\2  ineffective in any jurisdiction, the Affirmer hereby grants a free,\n'
    r'\1\2  full, permanent, irrevocable, nonexclusive and worldwide license for\n'
    r'\1\2  all her or his copyright and related or neighboring legal rights in\n'
    r'\1\2  the Work\.',
    multiLine: true,
  ),

  RegExp(
    r'()()punycode\.c 0\.4\.0 \(2001-Nov-17-Sat\)\n'
    r'\1\2http://www\.cs\.berkeley\.edu/~amc/idn/\n'
    r'\1\2Adam M\. Costello\n'
    r'\1\2http://www\.nicemice\.net/amc/\n'
    r'\1\2\n'
    r'\1\2Disclaimer and license\n'
    r'\1\2\n'
    r'\1\2    Regarding this entire document or any portion of it \(including\n'
    r'\1\2    the pseudocode and C code\), the author makes no guarantees and\n'
    r'\1\2    is not responsible for any damage resulting from its use\.  The\n'
    r'\1\2    author grants irrevocable permission to anyone to use, modify,\n'
    r'\1\2    and distribute it in any way that does not diminish the rights\n'
    r'\1\2    of anyone else to use, modify, and distribute it, provided that\n'
    r'\1\2    redistributed derivative works do not contain misleading author or\n'
    r'\1\2    version information\.  Derivative works need not be licensed under\n'
    r'\1\2    similar terms\.\n',
    multiLine: true,
  ),

  RegExp(
    kIndent +
        r'This file is provided as-is by Unicode, Inc\. \(The Unicode Consortium\)\. '
                r'No claims are made as to fitness for any particular purpose\.  No '
                r'warranties of any kind are expressed or implied\.  The recipient '
                r'agrees to determine applicability of information provided\.  If this '
                r'file has been provided on optical media by Unicode, Inc\., the sole '
                r'remedy for any claim will be exchange of defective media within 90 '
                r'days of receipt\.\n'
                r'\1\n'
                r'\1\2Unicode, Inc\. hereby grants the right to freely use the information '
                r'supplied in this file in the creation of products supporting the '
                r'Unicode Standard, and to make copies of this file in any form for '
                r'internal or external distribution as long as this notice remains '
                r'attached\. '
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  RegExp(
    kIndent +
        r'We are also required to state that "The Graphics Interchange Format\(c\) '
                r'is the Copyright property of CompuServe Incorporated. GIF\(sm\) is a '
                r'Service Mark property of CompuServe Incorporated."'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  // ZLIB-LIKE LICENSES

  // Seen in libjpeg-turbo (with a copyright), zlib.h
  RegExp(
    kIndent +
        r" This software is provided 'as-is', without any express or implied "
                r'warranty\. In no event will the authors be held liable for any damages '
                r'arising from the use of this software\.'
            .replaceAll(' ', _linebreak) +
        (r' Permission is granted to anyone to use this software for any purpose, '
                r'including commercial applications, and to alter it and redistribute it '
                r'freely, subject to the following restrictions:'
            .replaceAll(' ', _linebreak)) +
        r'(?:' +
        _linebreak +
        r'(?:' +
        r'|\n+' +
        r'|(?:[-*1-9.)/ ]*)' +
        (r'The origin of this software must not be misrepresented; you must not '
                r'claim that you wrote the original software\. If you use this software '
                r'in a product, an acknowledgment in the product documentation would be '
                r'appreciated but is not required\.'
            .replaceAll(' ', _linebreak)) +
        r'|(?:[-*1-9.)/ ]*)' +
        (r'Altered source versions must be plainly marked as such, and must not be '
                r'misrepresented as being the original software\. '
            .replaceAll(' ', _linebreak)) +
        r'|(?:[-*1-9.)/ ]*)' +
        (r"If you meet \(any of\) the author\(s\), you're encouraged to buy them a beer, "
                r'a drink or whatever is suited to the situation, given that you like the '
                r'software\. '
            .replaceAll(' ', _linebreak)) +
        r'|(?:[-*1-9.)/ ]*)' +
        (r'This notice may not be removed or altered from any source distribution\.'.replaceAll(
          ' ',
          _linebreak,
        )) +
        r'))+',
    multiLine: true,
    caseSensitive: false,
  ),
];

final List<RegExp> csStrayCopyrights = <RegExp>[
  // a file in BoringSSL
  RegExp(
    kIndent +
        r'DTLS code by Eric Rescorla <ekr@rtfm\.com> '
                r'\n '
                r'Copyright \(C\) 2006, Network Resonance, Inc\. '
                r'Copyright \(C\) 2011, RTFM, Inc\.'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  // thaidict.txt has a weird indenting thing going on
  RegExp(
    r'^ *# *Copyright \(c\) [-0-9]+ International Business Machines Corporation,\n'
    r' *# *Apple Inc\., and others\. All Rights Reserved\.',
    multiLine: true,
  ),

  // Found in a lot of ICU files
  RegExp(
    kIndent +
        r'Copyright \([Cc]\) [-, 0-9{}]+ ' +
        '(?:Google, )?International Business Machines '
                r'Corporation(?:(?:, Google,?)? and others)?\. All [Rr]ights [Rr]eserved\.'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  // Found in ICU files
  RegExp(
    kIndent +
        r'Copyright \([Cc]\) [-0-9,]+ IBM(?: Corp(?:oration)?)?(?:, Inc\.)?(?: and [Oo]thers)?\.(?: All rights reserved\.)?',
    multiLine: true,
  ),

  // Found in ICU files
  RegExp(
    kIndent + r'Copyright \(C\) [0-9]+ and later: Unicode, Inc\. and others\.',
    multiLine: true,
  ),

  // Found in ICU files
  RegExp(
    kIndent + r'Copyright \(c\) [-0-9 ]+ Unicode, Inc\.  All Rights reserved\.',
    multiLine: true,
  ),

  // Found in ICU files
  RegExp(kIndent + r'Copyright \(C\) [-,0-9]+ ,? Yahoo! Inc\.', multiLine: true),

  // Found in some ICU files
  RegExp(kIndent + r'Copyright [0-9]+ and onwards Google Inc.', multiLine: true),

  // Found in some ICU files
  RegExp(kIndent + r'Copyright [0-9]+ Google Inc. All Rights Reserved.', multiLine: true),

  // Found in some ICU files
  RegExp(
    kIndent +
        r'Copyright \(C\) [-0-9]+, Apple Inc\.(?:; Unicode, Inc\.;)? and others\. All Rights Reserved\.',
    multiLine: true,
  ),

  // rapidjson
  RegExp(
    kIndent +
        r'The above software in this distribution may have been modified by THL A29 Limited '
                r'\("Tencent Modifications"\)\. All Tencent Modifications are Copyright \(C\) 2015 THL A29 Limited\.'
            .replaceAll(' ', _linebreak),
    multiLine: true,
  ),

  // minizip
  RegExp(kIndent + r'Copyright \(C\) [-0-9]+ Gilles Vollant', multiLine: true),

  // Skia
  RegExp(kIndent + r'Copyright [-0-9]+ Google LLC\.', multiLine: true),

  // flutter/third_party/inja/third_party/include/hayai/hayai_clock.hpp
  // Advice was to just ignore these copyright notices given the LICENSE.md file
  // in the same directory.
  RegExp(
    kIndent +
        r'Copyright \(C\) 2011 Nick Bruun <nick@bruun\.co>\n'
            r'\1\2Copyright \(C\) 2013 Vlad Lazarenko <vlad@lazarenko\.me>\n'
            r'\1\2Copyright \(C\) 2014 Nicolas Pauss <nicolas\.pauss@gmail\.com>',
    multiLine: true,
  ),
];
