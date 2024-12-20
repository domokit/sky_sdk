// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_LIB_UI_WINDOW_POINTER_DATA_PACKET_H_
#define FLUTTER_LIB_UI_WINDOW_POINTER_DATA_PACKET_H_

#include <cstring>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/lib/ui/window/pointer_data.h"

namespace flutter {

class PointerDataPacket {
 public:
  explicit PointerDataPacket(size_t count);
  PointerDataPacket(uint8_t* data, size_t num_bytes);
  ~PointerDataPacket();

  void SetPointerData(size_t i, const PointerData& data);
  PointerData GetPointerData(size_t i) const;
  size_t GetLength() const;
  const std::vector<uint8_t>& data() const { return data_; }

 private:
  std::vector<uint8_t> data_;

  FML_DISALLOW_COPY_AND_ASSIGN(PointerDataPacket);
};

}  // namespace flutter

#endif  // FLUTTER_LIB_UI_WINDOW_POINTER_DATA_PACKET_H_
