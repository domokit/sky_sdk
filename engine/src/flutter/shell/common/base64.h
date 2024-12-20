// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_COMMON_BASE64_H_
#define FLUTTER_SHELL_COMMON_BASE64_H_

#include <cstddef>

namespace flutter {

struct Base64 {
 public:
  enum class Error {
    kNone,
    kBadPadding,
    kBadChar,
  };

  /**
     Base64 encodes src into dst.

     @param dst a pointer to a buffer large enough to receive the result.

     @return the required length of dst for encoding.
  */
  static size_t Encode(const void* src, size_t length, void* dst);

  /**
     Returns the length of the buffer that needs to be allocated to encode
     srcDataLength bytes.
  */
  static size_t EncodedSize(size_t srcDataLength) {
    // Take the floor of division by 3 to find the number of groups that need to
    // be encoded. Each group takes 4 bytes to be represented in base64.
    return ((srcDataLength + 2) / 3) * 4;
  }

  /**
     Base64 decodes src into dst.

     This can be called once with 'dst' nullptr to get the required size,
     then again with an allocated 'dst' pointer to do the actual decoding.

     @param dst nullptr or a pointer to a buffer large enough to receive the
     result

     @param dstLength assigned the length dst is required to be. Must not be
     nullptr.
  */
  [[nodiscard]] static Error Decode(const void* src,
                                    size_t srcLength,
                                    void* dst,
                                    size_t* dstLength);
};

}  // namespace flutter

#endif  // FLUTTER_SHELL_COMMON_BASE64_H_
