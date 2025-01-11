// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Converts Western Arabic numerals to Eastern Arabic numerals used in Kurdish Sorani.
String toKurdishSoraniNumerals(dynamic number) {
  final String numStr = number.toString();
  return numStr.replaceAllMapped(RegExp(r'[0-9]'), (Match match) {
    switch (match[0]) {
      case '0':
        return '٠';
      case '1':
        return '١';
      case '2':
        return '٢';
      case '3':
        return '٣';
      case '4':
        return '٤';
      case '5':
        return '٥';
      case '6':
        return '٦';
      case '7':
        return '٧';
      case '8':
        return '٨';
      case '9':
        return '٩';
      default:
        return match[0]!;
    }
  });
}
