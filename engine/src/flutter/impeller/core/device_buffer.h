// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_CORE_DEVICE_BUFFER_H_
#define FLUTTER_IMPELLER_CORE_DEVICE_BUFFER_H_

#include <memory>
#include <string>

#include "impeller/core/allocator.h"
#include "impeller/core/buffer_view.h"
#include "impeller/core/device_buffer_descriptor.h"
#include "impeller/core/range.h"

namespace impeller {

class DeviceBuffer {
 public:
  virtual ~DeviceBuffer();

  [[nodiscard]] bool CopyHostBuffer(const uint8_t* source,
                                    Range source_range,
                                    size_t offset = 0u);

  virtual bool SetLabel(std::string_view label) = 0;

  virtual bool SetLabel(std::string_view label, Range range) = 0;

  /// @brief Create a buffer view of this entire buffer.
  static BufferView AsBufferView(std::shared_ptr<DeviceBuffer> buffer);

  const DeviceBufferDescriptor& GetDeviceBufferDescriptor() const;

  virtual uint8_t* OnGetContents() const = 0;

  /// Make any pending writes visible to the GPU.
  ///
  /// This method must be called if the device pointer provided by
  /// [OnGetContents] is written to without using [CopyHostBuffer]. On Devices
  /// with coherent host memory, this method will not perform extra work.
  ///
  /// If the range is not provided, the entire buffer is flushed.
  virtual void Flush(std::optional<Range> range = std::nullopt) const;

  virtual void Invalidate(std::optional<Range> range = std::nullopt) const;

 protected:
  const DeviceBufferDescriptor desc_;

  explicit DeviceBuffer(DeviceBufferDescriptor desc);

  virtual bool OnCopyHostBuffer(const uint8_t* source,
                                Range source_range,
                                size_t offset) = 0;

 private:
  DeviceBuffer(const DeviceBuffer&) = delete;

  DeviceBuffer& operator=(const DeviceBuffer&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_CORE_DEVICE_BUFFER_H_
