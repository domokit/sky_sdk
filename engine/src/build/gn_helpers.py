# Copyright 2014 The Chromium Authors. All rights reserved.
# Use of this source code is governed by a BSD-style license that can be
# found in the LICENSE file.

import sys

"""Helper functions useful when writing scripts that are run from GN's
exec_script function."""

class GNError(Exception):
  pass


# Computes ASCII code of an element of encoded Python 2 str / Python 3 bytes.
_Ord = ord if sys.version_info.major < 3 else lambda c: c


def _TranslateToGnChars(s):
  for decoded_ch in s.encode('utf-8'):  # str in Python 2, bytes in Python 3.
    code = _Ord(decoded_ch)  # int
    if code in (34, 36, 92):  # For '"', '$', or '\\'.
      yield '\\' + chr(code)
    elif 32 <= code < 127:
      yield chr(code)
    else:
      yield '$0x%02X' % code


def ToGNString(value, pretty=False):
  """Returns a stringified GN equivalent of a Python value.

  Args:
    value: The Python value to convert.
    pretty: Whether to pretty print. If true, then non-empty lists are rendered
        recursively with one item per line, with indents. Otherwise lists are
        rendered without new line.
  Returns:
    The stringified GN equivalent to |value|.

  Raises:
    GNError: |value| cannot be printed to GN.
  """

  if sys.version_info.major < 3:
    basestring_compat = basestring
  else:
    basestring_compat = str

  # Emits all output tokens without intervening whitespaces.
  def GenerateTokens(v, level):
    if isinstance(v, basestring_compat):
      yield '"' + ''.join(_TranslateToGnChars(v)) + '"'

    elif isinstance(v, bool):
      yield 'true' if v else 'false'

    elif isinstance(v, int):
      yield str(v)

    elif isinstance(v, list):
      yield '['
      for i, item in enumerate(v):
        if i > 0:
          yield ','
        for tok in GenerateTokens(item, level + 1):
          yield tok
      yield ']'

    elif isinstance(v, dict):
      if level > 0:
        yield '{'
      for key in sorted(v):
        if not isinstance(key, basestring_compat):
          raise GNError('Dictionary key is not a string.')
        if not key or key[0].isdigit() or not key.replace('_', '').isalnum():
          raise GNError('Dictionary key is not a valid GN identifier.')
        yield key  # No quotations.
        yield '='
        for tok in GenerateTokens(v[key], level + 1):
          yield tok
      if level > 0:
        yield '}'

    else:  # Not supporting float: Add only when needed.
      raise GNError('Unsupported type when printing to GN.')

  can_start = lambda tok: tok and tok not in ',}]='
  can_end = lambda tok: tok and tok not in ',{[='

  # Adds whitespaces, trying to keep everything (except dicts) in 1 line.
  def PlainGlue(gen):
    prev_tok = None
    for i, tok in enumerate(gen):
      if i > 0:
        if can_end(prev_tok) and can_start(tok):
          yield '\n'  # New dict item.
        elif prev_tok == '[' and tok == ']':
          yield '  '  # Special case for [].
        elif tok != ',':
          yield ' '
      yield tok
      prev_tok = tok

  # Adds whitespaces so non-empty lists can span multiple lines, with indent.
  def PrettyGlue(gen):
    prev_tok = None
    level = 0
    for i, tok in enumerate(gen):
      if i > 0:
        if can_end(prev_tok) and can_start(tok):
          yield '\n' + '  ' * level  # New dict item.
        elif tok == '=' or prev_tok in '=':
          yield ' '  # Separator before and after '=', on same line.
      if tok in ']}':
        level -= 1
      # Exclude '[]' and '{}' cases.
      if int(prev_tok == '[') + int(tok == ']') == 1 or \
         int(prev_tok == '{') + int(tok == '}') == 1:
        yield '\n' + '  ' * level
      yield tok
      if tok in '[{':
        level += 1
      if tok == ',':
        yield '\n' + '  ' * level
      prev_tok = tok

  token_gen = GenerateTokens(value, 0)
  ret = ''.join((PrettyGlue if pretty else PlainGlue)(token_gen))
  # Add terminating '\n' for dict |value| or multi-line output.
  if isinstance(value, dict) or '\n' in ret:
    return ret + '\n'
  return ret

