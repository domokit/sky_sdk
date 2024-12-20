// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/base64.h"

#include "flutter/fml/logging.h"

#include <cstdint>

#define DecodePad -2
#define EncodePad 64

static const char kDefaultEncode[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/=";

static const signed char kDecodeData[] = {
    62, -1, -1,        -1, 63, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, -1,
    -1, -1, DecodePad, -1, -1, -1, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,
    10, 11, 12,        13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25,
    -1, -1, -1,        -1, -1, -1, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
    36, 37, 38,        39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51};

namespace flutter {

Base64::Error Base64::Decode(const void* srcv,
                             size_t srcLength,
                             void* dstv,
                             size_t* dstLength) {
  const unsigned char* src = static_cast<const unsigned char*>(srcv);
  unsigned char* dst = static_cast<unsigned char*>(dstv);

  int i = 0;
  bool padTwo = false;
  bool padThree = false;
  char unsigned const* const end = src + srcLength;
  while (src < end) {
    unsigned char bytes[4] = {0, 0, 0, 0};
    int byte = 0;
    do {
      unsigned char srcByte = *src++;
      if (srcByte == 0) {
        *dstLength = i;
        return Error::kNone;
      }
      if (srcByte <= ' ') {
        continue;  // treat as white space
      }
      if (srcByte < '+' || srcByte > 'z') {
        return Error::kBadChar;
      }
      signed char decoded = kDecodeData[srcByte - '+'];
      bytes[byte] = decoded;
      if (decoded != DecodePad) {
        if (decoded < 0) {
          return Error::kBadChar;
        }
        byte++;
        if (*src) {
          continue;
        }
        if (byte == 0) {
          *dstLength = i;
          return Error::kNone;
        }
        if (byte == 4) {
          break;
        }
      }
      // As an optimization, if we find an equals sign
      // we assume all future bytes to read are the
      // appropriate number of padding equals signs.
      if (byte < 2) {
        return Error::kBadPadding;
      }
      padThree = true;
      if (byte == 2) {
        padTwo = true;
      }
      break;
    } while (byte < 4);
    int two = 0;
    int three = 0;
    if (dst) {
      int one = static_cast<uint8_t>(bytes[0] << 2);
      two = bytes[1];
      one |= two >> 4;
      two = static_cast<uint8_t>((two << 4) & 0xFF);
      three = bytes[2];
      two |= three >> 2;
      three = static_cast<uint8_t>((three << 6) & 0xFF);
      three |= bytes[3];
      FML_DCHECK(one < 256 && two < 256 && three < 256);
      dst[i] = static_cast<unsigned char>(one);
    }
    i++;
    if (padTwo) {
      break;
    }
    if (dst) {
      dst[i] = static_cast<unsigned char>(two);
    }
    i++;
    if (padThree) {
      break;
    }
    if (dst) {
      dst[i] = static_cast<unsigned char>(three);
    }
    i++;
  }
  *dstLength = i;
  return Error::kNone;
}

size_t Base64::Encode(const void* srcv, size_t length, void* dstv) {
  FML_DCHECK(dstv);
  const unsigned char* src = static_cast<const unsigned char*>(srcv);
  unsigned char* dst = static_cast<unsigned char*>(dstv);

  const char* encode = kDefaultEncode;
  size_t remainder = length % 3;
  char unsigned const* const end = &src[length - remainder];
  while (src < end) {
    unsigned a = *src++;
    unsigned b = *src++;
    unsigned c = *src++;
    int d = c & 0x3F;
    c = (c >> 6 | b << 2) & 0x3F;
    b = (b >> 4 | a << 4) & 0x3F;
    a = a >> 2;
    // NOLINTBEGIN(clang-analyzer-core.NullDereference)
    *dst++ = encode[a];
    *dst++ = encode[b];
    *dst++ = encode[c];
    *dst++ = encode[d];
    // NOLINTEND(clang-analyzer-core.NullDereference)
  }
  if (remainder > 0) {
    int k1 = 0;
    int k2 = EncodePad;
    int a = static_cast<uint8_t>(*src++);
    if (remainder == 2) {
      int b = *src++;
      k1 = b >> 4;
      k2 = (b << 2) & 0x3F;
    }
    // NOLINTBEGIN(clang-analyzer-core.NullDereference)
    *dst++ = encode[a >> 2];
    *dst++ = encode[(k1 | a << 4) & 0x3F];
    *dst++ = encode[k2];
    *dst++ = encode[EncodePad];
    // NOLINTEND(clang-analyzer-core.NullDereference)
  }
  return EncodedSize(length);
}

}  // namespace flutter
