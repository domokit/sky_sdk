// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "impeller/renderer/backend/gles/blit_pass_gles.h"

#include <memory>

#include "flutter/fml/trace_event.h"
#include "fml/closure.h"
#include "impeller/core/formats.h"
#include "impeller/renderer/backend/gles/blit_command_gles.h"
#include "impeller/renderer/backend/gles/proc_table_gles.h"

namespace impeller {

BlitPassGLES::BlitPassGLES(std::shared_ptr<ReactorGLES> reactor)
    : reactor_(std::move(reactor)),
      is_valid_(reactor_ && reactor_->IsValid()) {}

// |BlitPass|
BlitPassGLES::~BlitPassGLES() = default;

// |BlitPass|
bool BlitPassGLES::IsValid() const {
  return is_valid_;
}

// |BlitPass|
void BlitPassGLES::OnSetLabel(std::string_view label) {
  label_ = std::string(label);
}

[[nodiscard]] bool EncodeCommandsInReactor(
    const std::shared_ptr<Allocator>& transients_allocator,
    const ReactorGLES& reactor,
    const std::vector<std::unique_ptr<BlitEncodeGLES>>& commands,
    const std::string& label) {
  TRACE_EVENT0("impeller", "BlitPassGLES::EncodeCommandsInReactor");

  if (commands.empty()) {
    return true;
  }

  const auto& gl = reactor.GetProcTable();

  fml::ScopedCleanupClosure pop_pass_debug_marker(
      [&gl]() { gl.PopDebugGroup(); });
  if (!label.empty()) {
    gl.PushDebugGroup(label);
  } else {
    pop_pass_debug_marker.Release();
  }

  for (const auto& command : commands) {
    fml::ScopedCleanupClosure pop_cmd_debug_marker(
        [&gl]() { gl.PopDebugGroup(); });
    auto label = command->GetLabel();
    if (!label.empty()) {
      gl.PushDebugGroup(label);
    } else {
      pop_cmd_debug_marker.Release();
    }

    if (!command->Encode(reactor)) {
      return false;
    }
  }

  return true;
}

// |BlitPass|
bool BlitPassGLES::EncodeCommands(
    const std::shared_ptr<Allocator>& transients_allocator) const {
  if (!IsValid()) {
    return false;
  }
  if (commands_.empty()) {
    return true;
  }

  std::shared_ptr<const BlitPassGLES> shared_this = shared_from_this();
  return reactor_->AddOperation([transients_allocator,
                                 blit_pass = std::move(shared_this),
                                 label = label_](const auto& reactor) {
    auto result = EncodeCommandsInReactor(transients_allocator, reactor,
                                          blit_pass->commands_, label);
    FML_CHECK(result) << "Must be able to encode GL commands without error.";
  });
}

// |BlitPass|
bool BlitPassGLES::OnCopyTextureToTextureCommand(
    std::shared_ptr<Texture> source,
    std::shared_ptr<Texture> destination,
    IRect source_region,
    IPoint destination_origin,
    std::string_view label) {
  auto command = std::make_unique<BlitCopyTextureToTextureCommandGLES>();
  command->label = label;
  command->source = std::move(source);
  command->destination = std::move(destination);
  command->source_region = source_region;
  command->destination_origin = destination_origin;

  commands_.push_back(std::move(command));
  return true;
}

// |BlitPass|
bool BlitPassGLES::OnCopyTextureToBufferCommand(
    std::shared_ptr<Texture> source,
    std::shared_ptr<DeviceBuffer> destination,
    IRect source_region,
    size_t destination_offset,
    std::string_view label) {
  auto command = std::make_unique<BlitCopyTextureToBufferCommandGLES>();
  command->label = label;
  command->source = std::move(source);
  command->destination = std::move(destination);
  command->source_region = source_region;
  command->destination_offset = destination_offset;

  commands_.push_back(std::move(command));
  return true;
}

// |BlitPass|
bool BlitPassGLES::OnCopyBufferToTextureCommand(
    BufferView source,
    std::shared_ptr<Texture> destination,
    IRect destination_region,
    std::string_view label,
    uint32_t mip_level,
    uint32_t slice,
    bool convert_to_read) {
  auto command = std::make_unique<BlitCopyBufferToTextureCommandGLES>();
  command->label = label;
  command->source = std::move(source);
  command->destination = std::move(destination);
  command->destination_region = destination_region;
  command->label = label;
  command->mip_level = mip_level;
  command->slice = slice;

  commands_.push_back(std::move(command));
  return true;
}

// |BlitPass|
bool BlitPassGLES::OnGenerateMipmapCommand(std::shared_ptr<Texture> texture,
                                           std::string_view label) {
  auto command = std::make_unique<BlitGenerateMipmapCommandGLES>();
  command->label = label;
  command->texture = std::move(texture);

  commands_.push_back(std::move(command));
  return true;
}

// |BlitPass|
bool BlitPassGLES::ResizeTexture(const std::shared_ptr<Texture>& source,
                                 const std::shared_ptr<Texture>& destination) {
  auto command = std::make_unique<BlitResizeTextureCommandGLES>();
  command->source = source;
  command->destination = destination;

  commands_.push_back(std::move(command));
  return true;
}

}  // namespace impeller
