// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/device_buffer_gles.h"

#include <cstring>
#include <memory>

#include "impeller/base/allocation.h"
#include "impeller/base/config.h"

namespace impeller {

DeviceBufferGLES::DeviceBufferGLES(DeviceBufferDescriptor desc,
                                   std::shared_ptr<ReactorGLES> reactor,
                                   std::shared_ptr<Allocation> backing_store)
    : DeviceBuffer(desc),
      reactor_(std::move(reactor)),
      backing_store_(std::move(backing_store)) {}

// |DeviceBuffer|
DeviceBufferGLES::~DeviceBufferGLES() {
  if (handle_.has_value() && !handle_->IsDead()) {
    reactor_->CollectHandle(*handle_);
  }
}

// |DeviceBuffer|
uint8_t* DeviceBufferGLES::OnGetContents() const {
  if (!reactor_) {
    return nullptr;
  }
  return backing_store_->GetBuffer();
}

// |DeviceBuffer|
bool DeviceBufferGLES::OnCopyHostBuffer(const uint8_t* source,
                                        Range source_range,
                                        size_t offset) {
  if (!reactor_) {
    return false;
  }

  if (offset + source_range.length >
      backing_store_->GetLength().GetByteSize()) {
    return false;
  }

  std::memmove(backing_store_->GetBuffer() + offset,
               source + source_range.offset, source_range.length);
  Flush(Range{offset, source_range.length});

  return true;
}

std::optional<GLuint> DeviceBufferGLES::GetHandle() const {
  if (handle_.has_value()) {
    return reactor_->GetGLHandle(*handle_);
  } else {
    return std::nullopt;
  }
}

void DeviceBufferGLES::Flush(std::optional<Range> range) const {
  if (!range.has_value()) {
    dirty_range_ = Range{
        0, static_cast<size_t>(backing_store_->GetLength().GetByteSize())};
  } else {
    if (dirty_range_.has_value()) {
      dirty_range_ = dirty_range_->Merge(range.value());
    } else {
      dirty_range_ = range.value();
    }
  }
}

static GLenum ToTarget(DeviceBufferGLES::BindingType type) {
  switch (type) {
    case DeviceBufferGLES::BindingType::kArrayBuffer:
      return GL_ARRAY_BUFFER;
    case DeviceBufferGLES::BindingType::kElementArrayBuffer:
      return GL_ELEMENT_ARRAY_BUFFER;
    case DeviceBufferGLES::BindingType::kUniformBuffer:
      return GL_UNIFORM_BUFFER;
  }
  FML_UNREACHABLE();
}

bool DeviceBufferGLES::BindAndUploadDataIfNecessary(BindingType type) const {
  if (!reactor_) {
    return false;
  }

  if (!handle_.has_value()) {
    handle_ = reactor_->CreateUntrackedHandle(HandleType::kBuffer);
#ifdef IMPELLER_DEBUG
    if (handle_.has_value() && label_.has_value()) {
      reactor_->SetDebugLabel(*handle_, *label_);
    }
#endif
  }

  auto buffer = reactor_->GetGLHandle(*handle_);
  if (!buffer.has_value()) {
    return false;
  }

  const auto target_type = ToTarget(type);
  const auto& gl = reactor_->GetProcTable();

  gl.BindBuffer(target_type, buffer.value());
  if (!initialized_) {
    gl.BufferData(target_type, backing_store_->GetLength().GetByteSize(),
                  nullptr, GL_DYNAMIC_DRAW);
    initialized_ = true;
  }

  if (dirty_range_.has_value()) {
    auto range = dirty_range_.value();
    gl.BufferSubData(target_type, range.offset, range.length,
                     backing_store_->GetBuffer() + range.offset);
    dirty_range_ = std::nullopt;
  }

  return true;
}

// |DeviceBuffer|
bool DeviceBufferGLES::SetLabel(std::string_view label) {
#ifdef IMPELLER_DEBUG
  label_ = label;
  if (handle_.has_value()) {
    reactor_->SetDebugLabel(*handle_, label);
  }
#endif  // IMPELLER_DEBUG
  return true;
}

// |DeviceBuffer|
bool DeviceBufferGLES::SetLabel(std::string_view label, Range range) {
  // Cannot support debug label on the range. Set the label for the entire
  // range.
  return SetLabel(label);
}

const uint8_t* DeviceBufferGLES::GetBufferData() const {
  return backing_store_->GetBuffer();
}

void DeviceBufferGLES::UpdateBufferData(
    const std::function<void(uint8_t* data, size_t length)>&
        update_buffer_data) {
  if (update_buffer_data) {
    update_buffer_data(backing_store_->GetBuffer(),
                       backing_store_->GetLength().GetByteSize());
    Flush(Range{
        0, static_cast<size_t>(backing_store_->GetLength().GetByteSize())});
  }
}

}  // namespace impeller
