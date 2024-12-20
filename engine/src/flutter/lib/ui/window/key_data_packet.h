// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_WINDOW_KEY_DATA_PACKET_H_
#define FLUTTER_LIB_UI_WINDOW_KEY_DATA_PACKET_H_

#include <functional>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/lib/ui/window/key_data.h"

namespace flutter {

// A byte stream representing a key event, to be sent to the framework.
//
// Changes to the marshalling format here must also be made to
// io/flutter/embedding/android/KeyData.java.
class KeyDataPacket {
 public:
  // Build the key data packet by providing information.
  //
  // The `character` is a nullable C-string that ends with a '\0'.
  KeyDataPacket(const KeyData& event, const char* character);
  ~KeyDataPacket();

  // Prevent copying.
  KeyDataPacket(KeyDataPacket const&) = delete;
  KeyDataPacket& operator=(KeyDataPacket const&) = delete;

  const std::vector<uint8_t>& data() const { return data_; }

 private:
  // Packet structure:
  // | CharDataSize |     (1 field)
  // |   Key Data   |     (kKeyDataFieldCount fields)
  // |   CharData   |     (CharDataSize bits)

  uint8_t* CharacterSizeStart() { return data_.data(); }
  uint8_t* KeyDataStart() { return CharacterSizeStart() + sizeof(uint64_t); }
  uint8_t* CharacterStart() { return KeyDataStart() + sizeof(KeyData); }

  std::vector<uint8_t> data_;
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_WINDOW_KEY_DATA_PACKET_H_
