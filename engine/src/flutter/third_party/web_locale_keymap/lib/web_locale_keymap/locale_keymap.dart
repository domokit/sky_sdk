//---------------------------------------------------------------------------------------------
//  Copyright (c) 2022 Google LLC
//  Licensed under the MIT License. See License.txt in the project root for license information.
//--------------------------------------------------------------------------------------------*/

import 'key_mappings.g.dart';

int? _characterToLogicalKey(String? key) {
  // We have yet to find a case where length >= 2 is useful.
  if (key == null || key.length >= 2) {
    return null;
  }
  final int result = key.toLowerCase().codeUnitAt(0);
  return result;
}

/// Maps locale-sensitive keys from KeyboardEvent properties to a logical key.
class LocaleKeymap {
  /// Create a [LocaleKeymap] for Windows.
  LocaleKeymap.win() : _mapping = getMappingDataWin();

  /// Create a [LocaleKeymap] for Linux.
  LocaleKeymap.linux() : _mapping = getMappingDataLinux();

  /// Create a [LocaleKeymap] for Darwin.
  LocaleKeymap.darwin() : _mapping = getMappingDataDarwin();

  /// Return a logical key mapped from KeyboardEvent properties.
  ///
  /// This method handles all printable characters, including letters, digits,
  /// and symbols.
  ///
  /// Before calling this method, the caller should have eliminated cases where
  /// the event key is a "key name", such as "Shift" or "AudioVolumnDown".
  ///
  /// If the return value is null, there's no way to derive a meaningful value
  /// from the printable information of the event.
  int? getLogicalKey(String? eventCode, String? eventKey, int eventKeyCode) {
    final int? result = _mapping[eventCode]?[eventKey];
    if (result == kUseKeyCode) {
      return eventKeyCode;
    }
    if (result == null) {
      if ((eventCode ?? '').isEmpty && (eventKey ?? '').isEmpty) {
        return null;
      }
      final int? heuristicResult = heuristicMapper(eventCode ?? '', eventKey ?? '');
      if (heuristicResult != null) {
        return heuristicResult;
      }
      // Characters: map to unicode zone.
      //
      // While characters are usually resolved in the last step, this can happen
      // in non-latin layouts when a non-latin character is on a symbol key (ru,
      // Semicolon-ж) or on an alnum key that has been assigned elsewhere (hu,
      // Digit0-Ö).
      final int? characterLogicalKey = _characterToLogicalKey(eventKey);
      if (characterLogicalKey != null) {
        return characterLogicalKey;
      }
    }
    return result;
  }

  final Map<String, Map<String, int>> _mapping;
}
