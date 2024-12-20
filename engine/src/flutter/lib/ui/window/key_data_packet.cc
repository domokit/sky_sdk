// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/lib/ui/window/key_data_packet.h"

#include <cstring>
#include <iostream>

#include "flutter/fml/logging.h"

namespace flutter {

KeyDataPacket::KeyDataPacket(const KeyData& event, const char* character) {
  size_t char_size = character == nullptr ? 0 : strlen(character);
  uint64_t char_size_64 = char_size;
  data_.resize(sizeof(char_size_64) + sizeof(KeyData) + char_size);
  memcpy(CharacterSizeStart(), &char_size_64, sizeof(char_size_64));
  memcpy(KeyDataStart(), &event, sizeof(KeyData));
  if (character != nullptr) {
    memcpy(CharacterStart(), character, char_size);
  }
}

KeyDataPacket::~KeyDataPacket() = default;

}  // namespace flutter
