#!/usr/bin/env python3
#
# Copyright 2013 The Flutter Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import sys, math
from operator import itemgetter
from itertools import groupby
import unicodedata
import fontforge

NAME = "FlutterTest"
# Turn off auto-hinting and enable manual hinting. FreeType skips auto-hinting
# if the font's family name is in a hard-coded "tricky" font list.
TRICKY_NAME = "MingLiU"
EM = 1024
DESCENT = -EM // 4
ASCENT = EM + DESCENT
# -143 and 20 are the underline location and width Ahem uses.
UPOS = -143 * 1000 // EM
UWIDTH = 20 * 1000 // EM

### Font Metadata and Metrics

font = fontforge.font()
font.familyname = TRICKY_NAME
font.fullname = NAME
font.fontname = NAME

# This sets the relevant fields in the os2 table and hhea table.
font.ascent = ASCENT
font.descent = -DESCENT
font.upos = UPOS
font.uwidth = UWIDTH
font.hhea_linegap = 0
font.os2_typolinegap = 0

font.horizontalBaseline = (
    ("hang", "ideo", "romn"),
    (
        ("latn", "romn", (ASCENT, DESCENT, 0), ()),
        ("grek", "romn", (ASCENT, DESCENT, 0), ()),
        ("hani", "ideo", (ASCENT, DESCENT, 0), ()),
    ),
)

### TrueType Hinting

# Hints are ignored on macOS.
#
# These hints only **vertically** adjust the outlines, for better vertical
# alignment in golden tests. They don't affect the font or the glyphs' public
# metrics available to the framework, so they typically don't affect non-golden
# tests.
#
# The hinting goals are:
#
# 1. Aligning the key points on glyph outlines between glyphs, when different
#    types of glyphs are placed side by side. E.g., for a given point size, "p"
#    and "É" should never overlap vertically, and "p" and "x" should be
#    bottom-aligned.
#
# 2. Aligning the top and the bottom of the "x" glyph with the background. With
#    point size = 14, since the em square's y-extent is 3.5 px (256 * 14 / 1024)
#    below the baseline and 10.5 px above the baseline, the glyph's CBOX will be
#    "rounded out" (3.5 -> 4, 10.5 -> 11). So "x" is going to be misaligned with
#    the background by +0.5 px when rasterized without proper grid-fitting.

# Allocate space in cvt.
font.cvt = [0]
# gcd is used to avoid overflowing, this works for the current ASCENT and EM value.
gcd = math.gcd(ASCENT, EM)
# The control value program is for computing the y-offset (in pixels) to move
# the embox's top edge to grid. The end result will be stored to CVT entry 0.
# CVT[0] = (pointSize * ASCENT / EM) - ceil(pointSize * ASCENT / EM)
prep_program = f"""
    RTG
    PUSHW_1
    0
    MPS
    PUSHW_1
    {(ASCENT << 6) // gcd}
    MUL
    PUSHW_1
    {EM // gcd}
    DIV
    DUP
    CEILING
    SUB
    WCVTP
"""
font.setTableData("prep", fontforge.parseTTInstrs(prep_program))


def glyph_program(glyph):
  # Shift Zone 1 by CVT[0]. In FreeType SHZ actually shifts the zone zp2
  # points to, instead of top of the stack. That's probably a bug.
  instructions = """
        SVTCA[0]
        PUSHB_4
        0
        0
        0
        0
        SZPS
        MIRP[0000]
        SRP2
        PUSHB_3
        1
        1
        1
        SZP2
        SHZ[0]
        SZPS
    """

  # Round To Grid every on-curve point, but ignore those who are on the ASCENT
  # or DESCENT line. This step keeps "p" (ascent flushed) and "É" (descent
  # flushed)'s y extents from overlapping each other.
  for index, point in enumerate([p for contour in glyph.foreground for p in contour]):
    if point.y not in [ASCENT, DESCENT]:
      instructions += f"""
                PUSHB_1
                {index}
                MDAP[1]
            """
  return fontforge.parseTTInstrs(instructions)


### Creating Glyphs Outlines


def square_glyph(glyph):
  pen = glyph.glyphPen()
  # Counter Clockwise
  pen.moveTo((0, DESCENT))
  pen.lineTo((0, ASCENT))
  pen.lineTo((EM, ASCENT))
  pen.lineTo((EM, DESCENT))
  pen.closePath()
  glyph.ttinstrs = glyph_program(glyph)


def ascent_flushed_glyph(glyph):
  pen = glyph.glyphPen()
  pen.moveTo((0, DESCENT))
  pen.lineTo((0, 0))
  pen.lineTo((EM, 0))
  pen.lineTo((EM, DESCENT))
  pen.closePath()
  glyph.ttinstrs = glyph_program(glyph)


def descent_flushed_glyph(glyph):
  pen = glyph.glyphPen()
  pen.moveTo((0, 0))
  pen.lineTo((0, ASCENT))
  pen.lineTo((EM, ASCENT))
  pen.lineTo((EM, 0))
  pen.closePath()
  glyph.ttinstrs = glyph_program(glyph)


def not_def_glyph(glyph):
  pen = glyph.glyphPen()
  # Counter Clockwise for the outer contour.
  pen.moveTo((EM // 8, 0))
  pen.lineTo((EM // 8, ASCENT))
  pen.lineTo((EM - EM // 8, ASCENT))
  pen.lineTo((EM - EM // 8, 0))
  pen.closePath()
  # Clockwise, inner contour.
  pen.moveTo((EM // 4, EM // 8))
  pen.lineTo((EM - EM // 4, EM // 8))
  pen.lineTo((EM - EM // 4, ASCENT - EM // 8))
  pen.lineTo((EM // 4, ASCENT - EM // 8))
  pen.closePath()
  glyph.ttinstrs = glyph_program(glyph)


def unicode_range(fromUnicode, throughUnicode):
  return range(fromUnicode, throughUnicode + 1)


square_codepoints = [
    codepoint for l in [
        unicode_range(0x21, 0x26),
        unicode_range(0x28, 0x6F),
        unicode_range(0x71, 0x7E),
        unicode_range(0xA1, 0xC8),
        unicode_range(0xCA, 0xFF),
        [0x131],
        unicode_range(0x152, 0x153),
        [0x178, 0x192],
        unicode_range(0x2C6, 0x2C7),
        [0x2C9],
        unicode_range(0x2D8, 0x2DD),
        [0x394, 0x3A5, 0x3A7, 0x3A9, 0x3BC, 0x3C0],
        unicode_range(0x2013, 0x2014),
        unicode_range(0x2018, 0x201A),
        unicode_range(0x201C, 0x201E),
        unicode_range(0x2020, 0x2022),
        [0x2026, 0x2030],
        unicode_range(0x2039, 0x203A),
        [0x2044, 0x2122, 0x2126, 0x2202, 0x2206, 0x220F],
        unicode_range(0x2211, 0x2212),
        unicode_range(0x2219, 0x221A),
        [0x221E, 0x222B, 0x2248, 0x2260],
        unicode_range(0x2264, 0x2265),
        [
            0x22F2, 0x25CA, 0x3007, 0x4E00, 0x4E03, 0x4E09, 0x4E5D, 0x4E8C, 0x4E94, 0x516B, 0x516D,
            0x5341, 0x56D7, 0x56DB, 0x571F, 0x6728, 0x6C34, 0x706B, 0x91D1
        ],
        unicode_range(0xF000, 0xF002),
    ] for codepoint in l
] + [0x70] + [ord(c) for c in "中文测试文本是否正确"]

no_path_codepoints = [
    #(codepoint, advance %)
    (0x0020, 1),
    (0x00A0, 1),
    (0x2003, 1),
    (0x3000, 1),
    (0x2002, 1 / 2),
    (0x2004, 1 / 3),
    (0x2005, 1 / 4),
    (0x2006, 1 / 6),
    (0x2009, 1 / 5),
    (0x200A, 1 / 10),
    (0xFEFF, 0),
    (0x200B, 0),
    (0x200C, 0),
    (0x200D, 0),
]


def create_glyph(name, contour):
  glyph = font.createChar(-1, name)
  contour(glyph)
  glyph.width = EM
  return glyph


if square_codepoints:
  create_glyph("Square", square_glyph).altuni = square_codepoints
create_glyph("Ascent Flushed", ascent_flushed_glyph).unicode = 0x70
create_glyph("Descent Flushed", descent_flushed_glyph).unicode = 0xC9
create_glyph(".notdef", not_def_glyph).unicode = -1


def create_no_path_glyph(codepoint, advance_percentage):
  name = "Zero Advance" if advance_percentage == 0 else (
      "Full Advance" if advance_percentage == 1 else f"1/{(int)(1/advance_percentage)} Advance"
  )
  no_path_glyph = font.createChar(codepoint, name)
  no_path_glyph.width = (int)(EM * advance_percentage)
  return no_path_glyph


for (codepoint, advance_percentage) in no_path_codepoints:
  if (codepoint in square_codepoints):
    raise ValueError(f"{hex(codepoint)} is occupied.")
  create_no_path_glyph(codepoint, advance_percentage)

font.generate(sys.argv[1] if len(sys.argv) >= 2 else "test_font.ttf")

### Printing Glyph Map Stats

scripts = set()
for glyph in font.glyphs():
  if glyph.unicode >= 0:
    scripts.add(fontforge.scriptFromUnicode(glyph.unicode))
  for codepoint, _, _ in glyph.altuni or []:
    scripts.add(fontforge.scriptFromUnicode(codepoint))
script_list = list(scripts)
script_list.sort()

print(f"|     \ Script <br />Glyph | {' | '.join(script_list)} |")
print(" | :--- " + " | :----: " * len(script_list) + "|")

for glyph in font.glyphs():
  if glyph.unicode < 0 and not glyph.altuni:
    continue
  glyph_mapping = {}
  if glyph.unicode >= 0:
    glyph_mapping[fontforge.scriptFromUnicode(glyph.unicode)] = [glyph.unicode]
  for codepoint, _, _ in glyph.altuni or []:
    script = fontforge.scriptFromUnicode(codepoint)
    if script in glyph_mapping:
      glyph_mapping[script].append(codepoint)
    else:
      glyph_mapping[script] = [codepoint]

  codepoints_by_script = [glyph_mapping.get(script, []) for script in script_list]

  def describe_codepoint_range(codepoints):
    if not codepoints:
      return ""
    codepoints.sort()
    codepoint_ranges = [
        list(map(itemgetter(1), group))
        for key, group in groupby(enumerate(codepoints), lambda x: x[0] - x[1])
    ]
    characters = [chr(c) for c in codepoints]

    def map_char(c):
      if c == "`":
        return "`` ` ``"
      if c == "|":
        return "`\\|`"
      if c.isprintable() and (not c.isspace()):
        return f"`{c}`"
      return "`<" + unicodedata.name(c, hex(ord(c))) + ">`"

    full_list = " ".join([map_char(c) for c in characters])
    return "**codepoint(s):** " + ", ".join([
        f"{hex(r[0])}-{hex(r[-1])}" if len(r) > 1 else hex(r[0]) for r in codepoint_ranges
    ]) + "<br />" + "**character(s):** " + full_list

  print(
      f"| {glyph.glyphname} | {' | '.join([describe_codepoint_range(l) for l in codepoints_by_script])} |"
  )
