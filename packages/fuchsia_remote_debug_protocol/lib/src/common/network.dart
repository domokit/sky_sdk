// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:core';

/// Determines whether `address` is a valid IPv6 or IPv4 address.
///
/// Throws an [ArgumentError] if the address is neither.
void validateAddress(String address) {
  if (!(isIpV4Address(address) || isIpV6Address(address))) {
    throw ArgumentError('"$address" is neither a valid IPv4 nor IPv6 address');
  }
}

/// Returns true if `address` is a valid IPv6 address.
bool isIpV6Address(String address) {
  try {
    // parseIpv6Address fails if there's a zone ID. Since this is still a valid
    // IP, remove any zone ID before parsing.
    final List<String> addressParts = address.split('%');
    Uri.parseIPv6Address(addressParts[0]);
    return true;
  } on FormatException {
    return false;
  }
}

/// Returns true if `address` is a valid IPv4 address.
bool isIpV4Address(String address) {
  try {
    Uri.parseIPv4Address(address);
    return true;
  } on FormatException {
    return false;
  }
}
